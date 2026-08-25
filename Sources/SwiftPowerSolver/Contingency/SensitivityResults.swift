import CryptoKit
import Foundation

// MARK: - Phase-shift terms (C1's lifetime split, K2's superset constraint)

/// Identity of the phase-shift terms. **A STRICT SUPERSET of `FactorsSignature`
/// (D64 §10 / K2), and the nesting is structural rather than documented.**
///
/// `pFinj_k = −b_k · shift_k` depends on BOTH `shift` and `b`. A signature
/// covering only `shiftRad` would let a reactance or tap change correctly
/// invalidate the factors while INCORRECTLY leaving the shift terms valid —
/// recompute factors, reuse shift terms, stale flows on a network whose
/// susceptances moved. Embedding the whole `FactorsSignature` makes the
/// lifetimes nest by construction: anything invalidating factors also
/// invalidates shift terms, never the reverse.
public struct PhaseShiftSignature: Hashable, Sendable, CustomStringConvertible {
    /// The factors signature this is a superset OF. Public so the nesting is
    /// checkable by a caller and by gate 6.14, not merely asserted.
    public let factors: FactorsSignature
    public let digest: String
    public var description: String { "PhaseShiftSignature(\(digest))" }

    static func of(_ net: BusBranchNetwork,
                   factors: FactorsSignature) throws -> PhaseShiftSignature {
        var hasher = SHA256()
        hasher.update(data: Data(factors.digest.utf8))
        for br in net.branches {
            let shift = br.inService ? br.shiftRad : 0
            guard shift.isFinite else {
                throw SensitivityError.invalidNetworkParameter(
                    "a non-finite phase shift cannot be hashed")
            }
            let normalised = shift == 0 ? 0.0 : shift
            withUnsafeBytes(of: normalised.bitPattern.littleEndian) {
                hasher.update(bufferPointer: $0)
            }
        }
        let d = hasher.finalize().compactMap { String(format: "%02x", $0) }
            .joined().prefix(32)
        return PhaseShiftSignature(factors: factors, digest: String(d))
    }
}

/// The additive phase-shift contribution to DC flow — deliberately NOT carried
/// on `PTDFResult` (C1).
///
/// Phase shift contributes an additive term to base flow and NOTHING to PTDF.
/// The two therefore have different invalidation lifetimes, and objects with
/// different lifetimes must not share cache identity. Recomputing these is
/// O(n_branch) — microseconds against a ~4 s factors build — so the split costs
/// nothing.
public struct PhaseShiftTerms: Sendable {
    public let signature: PhaseShiftSignature
    public let branchOrder: [BranchID]
    public let busOrder: [BusID]
    /// `pFinj_k = −b_k · shift_k`, parallel to `branchOrder`.
    public let branchFlowPu: [Double]
    /// Per-bus aggregation of `pFinj` (from `+=`, to `−=`), parallel to `busOrder`.
    public let busInjectionPu: [Double]

    public static func of(_ net: BusBranchNetwork) throws -> PhaseShiftTerms {
        let factorsSig = try FactorsSignature.of(net)
        let sig = try PhaseShiftSignature.of(net, factors: factorsSig)
        let (live, _) = try classify(net, slack: .networkDefined)
        let model = DCModel(net: net, live: live)
        return PhaseShiftTerms(
            signature: sig,
            branchOrder: (0..<net.branches.count).map(BranchID.init),
            busOrder: (0..<net.busCount).map(BusID.init),
            branchFlowPu: model.pFinj,
            busInjectionPu: model.pBusInj)
    }
}

// MARK: - PTDF

/// Injection sensitivities. **Factors only** — the phase-shift terms live in
/// `PhaseShiftTerms` (C1). Keyed by `FactorsSignature`.
public struct PTDFResult: Sendable {
    public let basis: SensitivityBasis
    public let slack: SlackReference
    public let signature: FactorsSignature
    public let branchOrder: [BranchID]
    public let busOrder: [BusID]

    enum Storage: Sendable {
        case base(DistributionFactors)
        case dense([Double])          // row-major, branchOrder x busOrder
    }
    let storage: Storage
    var busCount: Int { busOrder.count }

    init(basis: SensitivityBasis, slack: SlackReference, signature: FactorsSignature,
         branchOrder: [BranchID], busOrder: [BusID], storage: Storage) {
        self.basis = basis; self.slack = slack; self.signature = signature
        self.branchOrder = branchOrder; self.busOrder = busOrder; self.storage = storage
    }

    func value(branch k: Int, bus j: Int) -> Double {
        switch storage {
        case .base(let f):  return f.ptdf(branch: k, bus: j)
        case .dense(let v): return v[k * busCount + j]
        }
    }

    private func check(_ b: BranchID) throws {
        guard b.index >= 0 && b.index < branchOrder.count else {
            throw SensitivityError.unknownBranch(b)
        }
    }
    private func check(_ b: BusID) throws {
        guard b.index >= 0 && b.index < busOrder.count else {
            throw SensitivityError.unknownBus(b)
        }
    }

    // ---- SURFACE 1: pure sensitivity, dF/dP. No offset, ever. ----------------
    //
    // NAMING IS LOAD-BEARING (C1). Every member below returns a DERIVATIVE.
    // The complete-flow surface is `completeDCBranchFlows(...)` and is the only
    // thing on this type that returns a flow. A caller reaching for dF/dP
    // cannot land on an offset by mistyping, because no name here is a
    // near-miss for the other.

    /// `dF_branch / dP_bus`, MW per MW.
    public subscript(branch: BranchID, bus: BusID) -> Double {
        get throws {
            try check(branch); try check(bus)
            return value(branch: branch.index, bus: bus.index)
        }
    }

    /// One branch's full sensitivity row, in `busOrder`.
    public func sensitivityRow(_ branch: BranchID) throws -> [Double] {
        try check(branch)
        return (0..<busCount).map { value(branch: branch.index, bus: $0) }
    }

    /// Sensitivity of `branch`'s flow to a transfer from one bus to another —
    /// the branch-to-branch building block, `PTDF[l,f] − PTDF[l,t]`.
    public func transferSensitivity(on branch: BranchID,
                                    from: BusID, to: BusID) throws -> Double {
        try check(branch); try check(from); try check(to)
        return value(branch: branch.index, bus: from.index)
             - value(branch: branch.index, bus: to.index)
    }

    /// Flat row-major sensitivities in `(branchOrder × busOrder)` order, for RL
    /// feature assembly. Derivatives, never flows.
    public func rowMajorValues() -> [Double] {
        switch storage {
        case .dense(let v): return v
        case .base:
            var out = [Double](repeating: 0, count: branchOrder.count * busCount)
            for k in 0..<branchOrder.count {
                for j in 0..<busCount { out[k * busCount + j] = value(branch: k, bus: j) }
            }
            return out
        }
    }

    // ---- SURFACE 2: complete DC flow. The ONLY flow-returning member. --------

    /// Complete DC branch flow: `PTDF·(P − pBusInj) + pFinj`.
    ///
    /// **Takes the shift terms as a PARAMETER and validates BOTH signatures**
    /// against `network` (C1 / K2). It is not possible to obtain a complete flow
    /// from this type without supplying the shift terms, which is what stops a
    /// caller silently omitting the additive term.
    ///
    /// `injections` are the PHYSICAL bus injections (generation − load − shunt).
    /// The `pBusInj` subtraction is applied here rather than by the caller,
    /// because a caller who has to remember it is a caller who will forget it.
    public func completeDCBranchFlows(
        injections: [BusID: Double],
        phaseShift: PhaseShiftTerms,
        network: BusBranchNetwork
    ) throws -> [BranchID: Double] {
        let live = try FactorsSignature.of(network)
        guard live == signature else {
            throw SensitivityError.signatureMismatch(expected: signature, actual: live)
        }
        let shiftSig = try PhaseShiftSignature.of(network, factors: live)
        guard shiftSig == phaseShift.signature else {
            throw SensitivityError.signatureMismatch(expected: signature, actual: live)
        }
        var p = [Double](repeating: 0, count: busCount)
        for (bus, v) in injections {
            try check(bus)
            p[bus.index] = v
        }
        for j in 0..<busCount { p[j] -= phaseShift.busInjectionPu[j] }

        var out: [BranchID: Double] = [:]
        out.reserveCapacity(branchOrder.count)
        for k in 0..<branchOrder.count {
            var acc = 0.0
            for j in 0..<busCount { acc += value(branch: k, bus: j) * p[j] }
            out[BranchID(k)] = acc + phaseShift.branchFlowPu[k]
        }
        return out
    }
}

// MARK: - LODF, with D1's three controls

/// Outage redistribution factors, carrying **three controls that each do
/// exactly one job** (D65 §4, §5).
public struct LODFResult: Sendable {
    public let signature: FactorsSignature
    public let branchOrder: [BranchID]
    let factors: DistributionFactors
    /// Control 1 — connectivity. Structural, exact, no threshold.
    let bridges: [Bool]
    /// In service and both terminals live.
    let active: [Bool]
    /// Control 3 — `|1 − h_k|` per outage. Decides nothing.
    let conditioning: [Double]

    private func check(_ b: BranchID) throws {
        guard b.index >= 0 && b.index < branchOrder.count else {
            throw SensitivityError.unknownBranch(b)
        }
    }

    /// **Control 1.** True when outaging `branch` disconnects the network.
    /// Structural — a graph bridge — never a threshold on `1 − h_k`. Non-throwing
    /// so callers can filter before querying.
    public func isOutageIslanding(_ branch: BranchID) -> Bool {
        guard branch.index >= 0 && branch.index < bridges.count else { return false }
        return bridges[branch.index]
    }

    /// `dF_monitored / dF_outaged`. Throws `.islandingOutage` for a bridge: the
    /// stored column is zero there, which is coherent for an OUT-OF-SERVICE
    /// branch (`F_k = 0`, so the superposition is the identity) and incoherent
    /// for a bridge. Storage cannot tell them apart; the API must.
    public subscript(monitored: BranchID, outaged: BranchID) -> Double {
        get throws {
            try check(monitored); try check(outaged)
            if bridges[outaged.index] {
                throw SensitivityError.islandingOutage(outaged)
            }
            return factors.lodf(monitored: monitored.index, outaged: outaged.index)
        }
    }

    /// **Control 3 — the conditioning annotation. IT DECIDES NOTHING.**
    ///
    /// `|1 − h_k|`, the LODF denominator's distance from zero. Exposed so a
    /// consumer can see that a large factor came from a small denominator.
    ///
    /// **It BOUNDS; it does not PREDICT (D66 §4 / G1).** This is not a loose
    /// estimator of LODF error — it is not an estimator at all, and no
    /// calibration factor would make it one. Measured on case9241: branches 8012
    /// and 8013 carry IDENTICAL `|1 − h| = 5.348e-04` and differ by **1.48×** in
    /// actual error; across four near-bridges `|1 − h|` varies 17% while the
    /// measured error varies 3.8×. Reading `5.348e-04` as "four digits lost" is
    /// wrong by a measured **1.88–2.52 digits, in the SAFE direction**.
    ///
    /// Caveat, stated because the sample is small: n = 4, one case, all
    /// near-bridges. Enough to forbid the misuse, not enough to characterise the
    /// relationship.
    ///
    /// **The moment a threshold on this value gates behaviour, the control D65
    /// §4 removed has been rebuilt.** Use it to inform a human, never to branch.
    public func conditioning(outaging branch: BranchID) throws -> Double {
        try check(branch)
        return conditioning[branch.index]
    }

    /// Post-contingency flows by superposition: `F_m + LODF[m,k]·F_k`.
    public func postContingencyFlows(
        outaging branch: BranchID,
        baseFlows: [BranchID: Double]
    ) throws -> [BranchID: Double] {
        try check(branch)
        if bridges[branch.index] { throw SensitivityError.islandingOutage(branch) }
        let fk = baseFlows[branch] ?? 0
        var out: [BranchID: Double] = [:]
        out.reserveCapacity(branchOrder.count)
        for k in 0..<branchOrder.count {
            let m = BranchID(k)
            guard k != branch.index else { out[m] = 0; continue }
            let base = baseFlows[m] ?? 0
            out[m] = base + factors.lodf(monitored: k, outaged: branch.index) * fk
        }
        return out
    }
}
