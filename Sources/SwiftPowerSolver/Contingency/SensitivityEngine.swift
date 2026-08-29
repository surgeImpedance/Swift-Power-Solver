import Foundation

/// Builds the public sensitivity results over the SAME arithmetic
/// `DistributionFactors` has always used. This adds an interface, not a solver
/// (D64 §1); A1 froze the arithmetic and nothing here reaches into it.
public struct SensitivityEngine: Sendable {

    /// **Control 2 — the coarse garbage-solve guard (D65 §4).**
    ///
    /// Enforced when non-nil. See `nodalBalanceResidual` for what is measured
    /// and why this is NOT the quantity D65's spread was measured on.
    public var residualTolerance: Double?

    public init(residualTolerance: Double? = defaultResidualTolerance) {
        self.residualTolerance = residualTolerance
    }

    /// Calibrated at unit 1a against the nodal-balance residual on case300,
    /// case1354 and case9241 — see the unit 1a report for the measured spread.
    /// Deliberately far above the healthy maximum: this control is asked ONLY to
    /// catch a garbage solve, where the signal is order-1 against order-1e-12.
    /// D65 §4 removed its near-singularity duty, because the distribution is
    /// continuous and no scalar threshold can separate that case.
    public static let defaultResidualTolerance = 1e-6

    // MARK: - PTDF

    /// Injection-shift factors `∂F_branch/∂P_bus` for `net`, keyed by the
    /// network's `FactorsSignature` and screened by control 2 (the residual
    /// guard) when `residualTolerance` is non-nil.
    ///
    /// Throws `.singularAdmittanceMatrix(worstResidual:)` when the solve is
    /// numerically hollow (a rank-deficient network that Accelerate's QR
    /// "solves" — D65), and the reference-scheme errors from `classify`.
    /// The result is PURE sensitivity: for a complete DC flow on a
    /// shifter-bearing network, pair it with `PhaseShiftTerms.of(_:)` via
    /// `PTDFResult.completeDCBranchFlows` — the shift terms deliberately do
    /// not live here (C1).
    public func ptdf(_ net: BusBranchNetwork,
                     slack: SlackReference = .networkDefined) throws -> PTDFResult {
        // Total: `ptdfCore` returns nil only when a cancellation hook fires,
        // and this entry passes none.
        try ptdfCore(net, slack: slack, isCancelled: nil)!
    }

    /// `ptdf(_:slack:)` with a cooperative cancellation hook — the entry a
    /// consumer with a cancellable UI uses INSTEAD of going around the engine
    /// to `DistributionFactors.buildCancellable` (stage 1 of the ruled seam
    /// close, 2026-08-29).
    ///
    /// **Contract (frozen by publishing this):** `nil` means CANCELLED and
    /// nothing else — the same idiom `buildCancellable` already documents;
    /// nothing partial is observable and nothing is cached. Cancellation is
    /// deliberately NOT a thrown error: the throw channel is where control 2's
    /// refusal lives, and a consumer must never be able to swallow a refusal
    /// and a cancel with one catch arm. The hook is polled between build
    /// stages and once per parallel row chunk (inherited from
    /// `buildCancellable`; ~2 s worst-case latency at 9,241 buses).
    ///
    /// **A non-nil result passed the residual guard** (when
    /// `residualTolerance` is non-nil), exactly as for `ptdf(_:slack:)` —
    /// both entries and `lodf`'s self-build run through one core, so there is
    /// no variant of this engine that skips control 2.
    public func ptdf(_ net: BusBranchNetwork,
                     slack: SlackReference = .networkDefined,
                     isCancelled: @escaping @Sendable () -> Bool) throws -> PTDFResult? {
        try ptdfCore(net, slack: slack, isCancelled: isCancelled)
    }

    /// THE one code path from a network to a `PTDFResult`. Every public entry
    /// — both `ptdf` overloads and `lodf`'s self-build branch — lands here,
    /// which is the mechanism (not the promise) that keeps control 2 single:
    /// `nodalBalanceResidual` has exactly one caller in the module, and no
    /// entry can reach a factors build around it.
    private func ptdfCore(_ net: BusBranchNetwork, slack: SlackReference,
                          isCancelled: (@Sendable () -> Bool)?) throws -> PTDFResult? {
        let signature = try FactorsSignature.of(net)
        _ = try classify(net, slack: slack)          // validates the reference scheme

        guard let factors = DistributionFactors.buildCancellable(
            net, isCancelled: isCancelled) else { return nil }
        let n = net.busCount, nbr = net.branches.count
        let branchOrder = (0..<nbr).map(BranchID.init)
        let busOrder = (0..<n).map(BusID.init)

        var residual: Double?
        if let tol = residualTolerance {
            let r = Self.nodalBalanceResidual(net, factors: factors)
            guard r <= tol else {
                throw SensitivityError.singularAdmittanceMatrix(worstResidual: r)
            }
            residual = r          // carried as provenance for the flat export (R3)
        }

        let base = PTDFResult(basis: .dcLossless, slack: .networkDefined,
                              signature: signature, branchOrder: branchOrder,
                              busOrder: busOrder, storage: .base(factors),
                              solveResidual: residual)
        switch slack {
        case .networkDefined:
            return base
        case .uniformlyDistributed(let buses):
            let w = 1.0 / Double(buses.count)
            return Self.shift(base, weights: Dictionary(
                uniqueKeysWithValues: buses.map { ($0, w) }), slack: slack)
        case .distributed(let weights):
            let total = weights.values.reduce(0, +)
            let normalised = weights.mapValues { $0 / total }
            return Self.shift(base, weights: normalised, slack: slack)
        }
    }

    /// `PTDF_w[l,b] = PTDF[l,b] − Σ_c w_c · PTDF[l,c]` — a post-shift of the
    /// network-defined base. Base-invariant when the weights sum to 1, so there
    /// is no second factorization (A2).
    private static func shift(_ base: PTDFResult, weights: [BusID: Double],
                              slack: SlackReference) -> PTDFResult {
        let n = base.busOrder.count, nbr = base.branchOrder.count
        var out = [Double](repeating: 0, count: nbr * n)
        for k in 0..<nbr {
            var offset = 0.0
            for (bus, w) in weights { offset += w * base.value(branch: k, bus: bus.index) }
            for j in 0..<n { out[k * n + j] = base.value(branch: k, bus: j) - offset }
        }
        return PTDFResult(basis: base.basis, slack: slack, signature: base.signature,
                          branchOrder: base.branchOrder, busOrder: base.busOrder,
                          storage: .dense(out), solveResidual: base.solveResidual)
    }

    // MARK: - LODF

    /// Line-outage distribution factors for `net`, carrying D1's three
    /// controls: structural islanding (`isOutageIslanding`, branchable and
    /// non-throwing), the throwing accessors (`.islandingOutage` on an
    /// islanding column), and the unthresholded `conditioning(outaging:)`
    /// annotation.
    ///
    /// Pass `from:` to reuse an already-built PTDF; its signature is
    /// validated against `net` first (`.signatureMismatch` on disagreement).
    /// Only a `.networkDefined`-slack PTDF carries reusable base factors — a
    /// post-shifted (`.dense`) result is validated but the factors are
    /// rebuilt.
    public func lodf(_ net: BusBranchNetwork,
                     from ptdf: PTDFResult? = nil) throws -> LODFResult {
        let signature = try FactorsSignature.of(net)
        if let ptdf, ptdf.signature != signature {
            throw SensitivityError.signatureMismatch(expected: ptdf.signature,
                                                    actual: signature)
        }
        let factors: DistributionFactors
        if case .base(let f)? = ptdf?.storage {
            factors = f
        } else {
            // Self-build goes through the SAME guarded core as `ptdf` — until
            // 2026-08-29 this branch called `DistributionFactors.build`
            // directly, so a standalone `lodf(net)` on a rank-deficient
            // network returned garbage values through the engine's own front
            // door while `ptdf` refused. One core, no variant that skips
            // control 2: this call now throws `.singularAdmittanceMatrix`
            // there instead. Total force-unwrap: no cancellation hook.
            let built = try ptdfCore(net, slack: .networkDefined, isCancelled: nil)!
            guard case .base(let f) = built.storage else {
                throw SensitivityError.invalidNetworkParameter(
                    "unreachable: a .networkDefined core build is always .base storage")
            }
            factors = f
        }

        let nbr = net.branches.count
        let live = net.buses.map { $0.type != .isolated }
        let bridges = NetworkConnectivity.bridgeBranches(net, live: live)
        var active = [Bool](repeating: false, count: nbr)
        var conditioning = [Double](repeating: .nan, count: nbr)
        for (k, br) in net.branches.enumerated() {
            guard br.inService, live[br.from], live[br.to] else { continue }
            active[k] = true
            let h = factors.ptdf(branch: k, bus: br.from)
                  - factors.ptdf(branch: k, bus: br.to)
            conditioning[k] = abs(1 - h)
        }
        return LODFResult(signature: signature,
                          branchOrder: (0..<nbr).map(BranchID.init),
                          factors: factors, bridges: bridges,
                          active: active, conditioning: conditioning)
    }

    // MARK: - OTDF

    /// PTDF on the network with `branch` out, derived from PTDF and LODF —
    /// **no per-contingency factorization** (brief §3).
    ///
    ///     OTDF_k[l, b] = PTDF[l, b] + LODF[l, k] · PTDF[k, b]
    public func otdf(from ptdf: PTDFResult, lodf: LODFResult,
                     outaging branch: BranchID) throws -> PTDFResult {
        guard ptdf.signature == lodf.signature else {
            throw SensitivityError.signatureMismatch(expected: ptdf.signature,
                                                    actual: lodf.signature)
        }
        guard branch.index >= 0 && branch.index < ptdf.branchOrder.count else {
            throw SensitivityError.unknownBranch(branch)
        }
        if lodf.isOutageIslanding(branch) {
            throw SensitivityError.islandingOutage(branch)
        }
        let n = ptdf.busOrder.count, nbr = ptdf.branchOrder.count
        var out = [Double](repeating: 0, count: nbr * n)
        for l in 0..<nbr {
            let f = lodf.factors.lodf(monitored: l, outaged: branch.index)
            for b in 0..<n {
                out[l * n + b] = ptdf.value(branch: l, bus: b)
                    + f * ptdf.value(branch: branch.index, bus: b)
            }
        }
        return PTDFResult(basis: ptdf.basis, slack: ptdf.slack,
                          signature: ptdf.signature, branchOrder: ptdf.branchOrder,
                          busOrder: ptdf.busOrder, storage: .dense(out),
                          solveResidual: ptdf.solveResidual)
    }

    // MARK: - Control 2's quantity

    /// `max_j max_i |Σ_k A[i,k]·PTDF[k,j] − δ_ij|` over live non-reference buses.
    ///
    /// Nodal balance: `Σ_k A[i,k]·F_k = P_i`, so differentiating,
    /// `Σ_k A[i,k]·PTDF[k,j] = δ_ij` for any non-reference `i`. A garbage solve
    /// breaks this by order 1.
    ///
    /// **This is NOT the quantity D65's measured spread describes**, and the
    /// difference is deliberate. D65 measured `‖B_red·x − e‖∞`, which requires
    /// the SOLVED INVERSE — an intermediate that lives inside
    /// `DistributionFactors.build` and is not recoverable from PTDF (PTDF holds
    /// only differences `b_k(X[f]−X[t])`). Reaching it would mean opening the
    /// frozen type. This checks the DELIVERED ARTIFACT instead, which is a
    /// stronger target, and is calibrated by its own measurement — transferring
    /// D65's threshold across a change of instrument is exactly what the
    /// wrong-quantity rule forbids.
    static func nodalBalanceResidual(_ net: BusBranchNetwork,
                                     factors: DistributionFactors) -> Double {
        let n = net.busCount
        let live = net.buses.map { $0.type != .isolated }
        let isRef = net.buses.map { $0.type == .slack }
        var worst = 0.0
        var acc = [Double](repeating: 0, count: n)
        for j in 0..<n where live[j] && !isRef[j] {
            for i in 0..<n { acc[i] = 0 }
            for (k, br) in net.branches.enumerated() {
                guard br.inService, live[br.from], live[br.to] else { continue }
                let v = factors.ptdf(branch: k, bus: j)
                acc[br.from] += v
                acc[br.to] -= v
            }
            for i in 0..<n where live[i] && !isRef[i] {
                worst = max(worst, abs(acc[i] - (i == j ? 1 : 0)))
            }
        }
        return worst
    }
}
