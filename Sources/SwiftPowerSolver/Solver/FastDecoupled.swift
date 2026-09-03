import Foundation
import Accelerate

// Fast-Decoupled Power Flow (Stott & Alsac, 1974), matching MATPOWER/pypower
// (makeB + fdpf) so the construction can be verified against that reference:
//
//   - B′ (all non-slack buses) drives the angle update  B′·Δθ  = ΔP/V
//   - B″ (PQ buses only)      drives the magnitude one  B″·ΔV = ΔQ/V
//   - both matrices depend only on topology/parameters, so they are factorized
//     once and reused across every iteration — and, through
//     `FDPFFactorizationCache`, across every step of a quasi-static sweep.
//
// Matrix construction is pypower `makeB`, implemented literally by modifying a
// copy of each branch and reusing `YbusBuilder.admittance` — the exact same
// arithmetic the full Ybus uses, so the semantics cannot drift:
//
//   B′  = -imag(Ybus) of: bus shunt B zeroed, branch charging zeroed, taps
//         cancelled, phase shifts KEPT; XB additionally zeroes r.
//   B″  = -imag(Ybus) of: phase shifts zeroed, taps/charging/shunts KEPT;
//         BX additionally zeroes r.
//
// (Note two places the classic textbook prose differs from MATPOWER: makeB
// keeps phase shifts in B′ and zeroes only the shunt SUSCEPTANCE there. The
// reference implementation wins; `FDPFMakeBTests` pins both matrices against
// pypower's own makeB output.)
//
// Iteration is MATPOWER-style half-iterations on the SCALED mismatch
// (ΔP_i/V_i, ΔQ_i/V_i): P half-step, convergence check on both norms, Q
// half-step, check again. FDPF converges linearly, so `maxIterations` (full
// P+Q rounds) is deliberately generous.
//
// v1 scope, by design (see DECISIONS.md D6 in substation-lab/macos):
//   - NO generator Q-limit enforcement: PV→PQ switching changes the PQ set,
//     which invalidates B″ and forces refactorization each restart. Callers
//     that need limits go through `PowerFlowEngine`, which routes the request
//     to FDPF warm start → Newton-Raphson so NR does the finishing.
//   - NO distributed slack, same routing.

public enum FDPFVariant: String, Sendable, Equatable, CaseIterable {
    /// Classic Stott-Alsac: B′ neglects resistance, B″ includes it.
    case xb
    /// Van Amerongen: B′ includes resistance, B″ neglects it. More robust on
    /// high-r/x networks — the package default.
    case bx
}

// MARK: - B′ / B″ construction (pypower makeB)

enum FDPFMatrices {

    /// Full bus-space triplets of B′ = -imag(Ybus of the makeB-modified
    /// network). Duplicates are NOT summed here; the CSC compression sums them
    /// in supplied order, exactly as the NR Jacobian path does.
    static func bPrimeEntries(_ net: BusBranchNetwork,
                              variant: FDPFVariant) -> [(row: Int, col: Int, value: Double)] {
        var entries: [(row: Int, col: Int, value: Double)] = []
        entries.reserveCapacity(4 * net.branches.count)
        for branch in net.branches where branch.inService {
            var mb = branch
            mb.b = 0                     // zero out line charging shunts
            mb.tap = 1                   // cancel out taps
            if variant == .xb { mb.r = 0 }   // XB: zero out line resistance
            let y = YbusBuilder.admittance(of: mb)
            entries.append((branch.from, branch.from, -y.yff.im))
            entries.append((branch.from, branch.to, -y.yft.im))
            entries.append((branch.to, branch.from, -y.ytf.im))
            entries.append((branch.to, branch.to, -y.ytt.im))
        }
        // Bus shunt susceptance is zeroed in B′ (makeB: temp_bus[:, BS] = 0);
        // conductance never reaches -imag, so no bus terms at all.
        return entries
    }

    /// Full bus-space triplets of B″, same conventions as `bPrimeEntries`.
    static func bDoublePrimeEntries(_ net: BusBranchNetwork,
                                    variant: FDPFVariant) -> [(row: Int, col: Int, value: Double)] {
        var entries: [(row: Int, col: Int, value: Double)] = []
        entries.reserveCapacity(4 * net.branches.count + net.busCount)
        for branch in net.branches where branch.inService {
            var mb = branch
            mb.shiftRad = 0                  // zero out phase shifters
            if variant == .bx { mb.r = 0 }   // BX: zero out line resistance
            let y = YbusBuilder.admittance(of: mb)
            entries.append((branch.from, branch.from, -y.yff.im))
            entries.append((branch.from, branch.to, -y.yft.im))
            entries.append((branch.to, branch.from, -y.ytf.im))
            entries.append((branch.to, branch.to, -y.ytt.im))
        }
        for (i, bus) in net.buses.enumerated() where bus.bsPu != 0 {
            entries.append((i, i, -bus.bsPu))
        }
        return entries
    }

    /// Restrict full bus-space triplets to the square submatrix over `buses`,
    /// re-indexed 0..<buses.count. `position[bus]` must be the reduced index
    /// or -1. Entry order is preserved (stable), so duplicate summing in the
    /// CSC compression stays deterministic.
    static func submatrix(_ entries: [(row: Int, col: Int, value: Double)],
                          position: [Int]) -> [(row: Int, col: Int, value: Double)] {
        var out: [(row: Int, col: Int, value: Double)] = []
        out.reserveCapacity(entries.count)
        for e in entries {
            let r = position[e.row], c = position[e.col]
            if r >= 0 && c >= 0 { out.append((r, c, e.value)) }
        }
        return out
    }
}

// MARK: - Reusable sparse factorization

/// A QR factorization kept alive across solves (the whole point of FDPF: B′
/// and B″ are constant, so factor once, back-solve every half-iteration).
///
/// SEQUENTIAL USE ONLY. Accelerate factorization workspaces must not be
/// shared across threads — see the measured note in
/// `SparseLinearSolver.solve(n:entries:rhsColumns:)`. Every use here is
/// single-threaded: one solve loop, one sweep loop.
final class SparseQRFactorization {
    let dim: Int
    // The CSC buffers are kept alive for the factorization's lifetime rather
    // than relying on SparseFactor having copied everything it needs.
    private let columnStarts: UnsafeMutablePointer<Int>
    private let rowIndices: UnsafeMutablePointer<Int32>
    private let values: UnsafeMutablePointer<Double>
    private let nnz: Int
    private let factorization: SparseOpaqueFactorization_Double?

    /// Fails (returns nil) if the matrix is structurally or numerically
    /// singular. A dim of 0 is valid and solves to the empty vector.
    init?(n: Int, entries: [(row: Int, col: Int, value: Double)]) {
        dim = n
        let (cs, ri, vals) = SparseLinearSolver.compressToCSC(n: n, entries: entries)
        nnz = vals.count
        columnStarts = .allocate(capacity: cs.count)
        columnStarts.update(from: cs, count: cs.count)
        rowIndices = .allocate(capacity: max(1, ri.count))
        rowIndices.update(from: ri, count: ri.count)
        values = .allocate(capacity: max(1, vals.count))
        values.update(from: vals, count: vals.count)

        if n == 0 {
            factorization = nil        // nothing to factor; solve returns []
            return
        }
        let structure = SparseMatrixStructure(
            rowCount: Int32(n), columnCount: Int32(n),
            columnStarts: columnStarts, rowIndices: rowIndices,
            attributes: SparseAttributes_t(), blockSize: 1)
        let matrix = SparseMatrix_Double(structure: structure, data: values)
        let f = SparseFactor(SparseFactorizationQR, matrix)
        guard f.status == SparseStatusOK else {
            SparseCleanup(f)
            columnStarts.deallocate(); rowIndices.deallocate(); values.deallocate()
            return nil
        }
        factorization = f
    }

    /// Back-solve A·x = rhs. Returns nil if the solution is non-finite.
    func solve(_ rhs: [Double]) -> [Double]? {
        precondition(rhs.count == dim)
        guard let factorization else { return [] }      // dim == 0
        var x = [Double](repeating: 0, count: dim)
        var b = rhs
        b.withUnsafeMutableBufferPointer { bp in
            x.withUnsafeMutableBufferPointer { xp in
                let bVec = DenseVector_Double(count: Int32(dim), data: bp.baseAddress!)
                let xVec = DenseVector_Double(count: Int32(dim), data: xp.baseAddress!)
                SparseSolve(factorization, bVec, xVec)
            }
        }
        guard x.allSatisfy({ $0.isFinite }) else { return nil }
        return x
    }

    deinit {
        if let factorization { SparseCleanup(factorization) }
        columnStarts.deallocate()
        rowIndices.deallocate()
        values.deallocate()
    }
}

// MARK: - Factorization cache

/// Caches the factorized B′/B″ pair across solves. B′ and B″ depend only on
/// topology and branch/shunt parameters — not on loads, setpoints, or voltage
/// state — so a quasi-static sweep whose steps only move loads and generator P
/// reuses one factorization pair for the whole horizon.
///
/// Invalidation is by exact key comparison against everything the matrices
/// (and their row/column index sets) are built from: variant, bus types, bus
/// shunt susceptance, all branch parameters, and the pv/pq membership. Same
/// triggers as a Ybus rebuild, narrowed to the fields that reach -imag.
///
/// Not thread-safe; use from one solve loop at a time (see
/// `SparseQRFactorization`).
public final class FDPFFactorizationCache {
    public init() {}

    /// Factorizations performed through this cache — B′ and B″ each count
    /// one, so a topology-static sweep of any length measures exactly 2.
    public private(set) var factorizationCount = 0

    struct Key: Equatable {
        var variant: FDPFVariant
        var busTypes: [BusBranchNetwork.BusType]
        var bsPu: [Double]
        var branches: [BusBranchNetwork.Branch]
        var pvpq: [Int]
        var pq: [Int]
    }

    private var key: Key?
    private var bPrime: SparseQRFactorization?
    private var bDoublePrime: SparseQRFactorization?

    /// Returns the cached pair when the key matches, otherwise rebuilds and
    /// refactorizes both. Returns nil when either matrix is singular (not
    /// cached, so a later valid network is not blocked by one bad state).
    func factorizations(for net: BusBranchNetwork, variant: FDPFVariant,
                        pvpq: [Int], pq: [Int],
                        pvpqPosition: [Int], pqPosition: [Int])
        -> (bPrime: SparseQRFactorization, bDoublePrime: SparseQRFactorization)? {
        let newKey = Key(variant: variant,
                         busTypes: net.buses.map(\.type),
                         bsPu: net.buses.map(\.bsPu),
                         branches: net.branches,
                         pvpq: pvpq, pq: pq)
        if let key, key == newKey, let bp = bPrime, let bpp = bDoublePrime {
            return (bp, bpp)
        }
        let bpEntries = FDPFMatrices.submatrix(
            FDPFMatrices.bPrimeEntries(net, variant: variant), position: pvpqPosition)
        let bppEntries = FDPFMatrices.submatrix(
            FDPFMatrices.bDoublePrimeEntries(net, variant: variant), position: pqPosition)
        factorizationCount += 2
        guard let bp = SparseQRFactorization(n: pvpq.count, entries: bpEntries),
              let bpp = SparseQRFactorization(n: pq.count, entries: bppEntries) else {
            key = nil; bPrime = nil; bDoublePrime = nil
            return nil
        }
        key = newKey
        bPrime = bp
        bDoublePrime = bpp
        return (bp, bpp)
    }
}

// MARK: - Solver

public struct FastDecoupledSolver: PowerFlowSolver {

    public init() {}

    public func solve(_ net: BusBranchNetwork,
                      options: PowerFlowOptions = PowerFlowOptions()) -> PowerFlowSolution {
        solve(net, options: options, cache: nil)
    }

    /// Standalone FDPF to convergence. Pass a `cache` to reuse the B′/B″
    /// factorizations across repeated solves of the same topology.
    public func solve(_ net: BusBranchNetwork,
                      options: PowerFlowOptions,
                      cache: FDPFFactorizationCache?) -> PowerFlowSolution {
        // v1 feature guards — `PowerFlowEngine` routes these to FDPF warm
        // start → NR instead of ever hitting this error.
        if options.enforceQLimits {
            return failed(net, reason: "standalone FDPF does not enforce generator "
                + "Q limits (PV→PQ switching would invalidate B″); use "
                + "PowerFlowEngine, which routes to FDPF warm start → Newton-Raphson")
        }
        if net.generators.contains(where: { $0.inService && ($0.slackWeight ?? 0) != 0 }) {
            return failed(net, reason: "standalone FDPF does not support distributed "
                + "slack; use PowerFlowEngine, which routes to FDPF warm start → "
                + "Newton-Raphson")
        }

        let state = iterate(net, options: options,
                            tolerancePu: options.tolerancePu,
                            maxRounds: options.maxIterations,
                            cache: cache)
        guard state.converged else {
            return failed(net, reason: state.failureReason ?? "did not converge",
                          iterations: state.rounds,
                          mismatchPu: state.finalMismatchPu)
        }
        return assemble(net, state: state)
    }

    // MARK: Half-iteration loop (internal seam shared with the warm-start path)

    struct IterateState {
        var vm: [Double]
        var va: [Double]
        /// Completed full P+Q rounds (MATPOWER's iteration counter).
        var rounds: Int
        var converged: Bool
        var failureReason: String?
        /// max(normP, normQ) of the scaled mismatch at exit; NaN when the
        /// iterates went non-finite.
        var finalMismatchPu: Double
        // Carried for result assembly.
        var live: [Bool]
        var isSlack: [Bool]
        var gensAtBus: [[Int]]
        var pCalc: [Double]
        var qCalc: [Double]
    }

    /// Run FDPF half-iterations to `tolerancePu` or `maxRounds`, whichever
    /// first. Never throws away the iterate: on non-convergence the current
    /// vm/va are still returned, which is what makes this usable as a
    /// warm-start stage (hand NR the best available seed).
    func iterate(_ net: BusBranchNetwork,
                 options: PowerFlowOptions,
                 tolerancePu: Double,
                 maxRounds: Int,
                 cache: FDPFFactorizationCache?) -> IterateState {
        let n = net.busCount
        let ybus = YbusBuilder.build(net)

        // --- classification (matches NewtonRaphsonSolver) -------------------
        var gensAtBus = [[Int]](repeating: [], count: n)
        for (g, gen) in net.generators.enumerated() where gen.inService {
            gensAtBus[gen.bus].append(g)
        }
        var isSlack = [Bool](repeating: false, count: n)
        var isPV = [Bool](repeating: false, count: n)
        var live = [Bool](repeating: false, count: n)
        for (i, bus) in net.buses.enumerated() {
            switch bus.type {
            case .slack: isSlack[i] = true; live[i] = true
            case .pv: isPV[i] = !gensAtBus[i].isEmpty; live[i] = true
            case .pq: live[i] = true
            case .isolated: break
            }
        }
        func state(_ vm: [Double], _ va: [Double], rounds: Int, converged: Bool,
                   reason: String?, mismatch: Double,
                   pCalc: [Double], qCalc: [Double]) -> IterateState {
            IterateState(vm: vm, va: va, rounds: rounds, converged: converged,
                         failureReason: reason, finalMismatchPu: mismatch,
                         live: live, isSlack: isSlack, gensAtBus: gensAtBus,
                         pCalc: pCalc, qCalc: qCalc)
        }

        let slackBuses = (0..<n).filter { isSlack[$0] }
        guard !slackBuses.isEmpty else {
            return state([Double](repeating: 1, count: n),
                         [Double](repeating: 0, count: n),
                         rounds: 0, converged: false, reason: "no slack bus",
                         mismatch: .nan,
                         pCalc: [Double](repeating: 0, count: n),
                         qCalc: [Double](repeating: 0, count: n))
        }

        let pv = (0..<n).filter { isPV[$0] && live[$0] && !isSlack[$0] }
        let pq = (0..<n).filter { live[$0] && !isSlack[$0] && !isPV[$0] }
        let pvpq = pv + pq
        var pvpqPosition = [Int](repeating: -1, count: n)
        for (r, bus) in pvpq.enumerated() { pvpqPosition[bus] = r }
        var pqPosition = [Int](repeating: -1, count: n)
        for (r, bus) in pq.enumerated() { pqPosition[bus] = r }

        // --- specified injections (pu) --------------------------------------
        var pSpec = [Double](repeating: 0, count: n)
        var qSpec = [Double](repeating: 0, count: n)
        for (i, bus) in net.buses.enumerated() {
            pSpec[i] = -bus.pLoadPu
            qSpec[i] = -bus.qLoadPu
        }
        for gen in net.generators where gen.inService {
            pSpec[gen.bus] += gen.pPu
        }

        // --- initial voltage (flat start / warm start, gen setpoints) -------
        // Identical seeding to NewtonRaphsonSolver, including the case118-style
        // non-zero slack reference angle and the opt-in warm-start fields.
        var vm = [Double](repeating: 1.0, count: n)
        var va = [Double](repeating: 0.0, count: n)
        if let refGen = net.generators.first(where: { $0.inService && isSlack[$0.bus] }) {
            va = [Double](repeating: refGen.vaRefRad, count: n)
        }
        if let vm0 = options.initialVmPu, vm0.count == n {
            for i in 0..<n where vm0[i].isFinite && vm0[i] > 0 { vm[i] = vm0[i] }
        }
        if let va0 = options.initialVaRad, va0.count == n {
            for i in 0..<n where va0[i].isFinite { va[i] = va0[i] }
        }
        for gen in net.generators where gen.inService {
            vm[gen.bus] = gen.vSetPu
            if isSlack[gen.bus] { va[gen.bus] = gen.vaRefRad }
        }

        // --- mismatch machinery ---------------------------------------------
        var pCalc = [Double](repeating: 0, count: n)
        var qCalc = [Double](repeating: 0, count: n)
        func computeInjections() {
            for i in 0..<n {
                var p = 0.0, q = 0.0
                for k in ybus.rowStarts[i]..<ybus.rowStarts[i + 1] {
                    let j = ybus.columns[k]
                    let g = ybus.re[k], b = ybus.im[k]
                    let theta = va[i] - va[j]
                    let c = cos(theta), s = sin(theta)
                    p += vm[j] * (g * c + b * s)
                    q += vm[j] * (g * s - b * c)
                }
                pCalc[i] = vm[i] * p
                qCalc[i] = vm[i] * q
            }
        }

        // Scaled mismatches (pypower fdpf: mis = (S(V) − Sspec) / Vm).
        var mP = [Double](repeating: 0, count: pvpq.count)
        var mQ = [Double](repeating: 0, count: pq.count)
        var normP = 0.0, normQ = 0.0
        func computeMismatch() {
            computeInjections()
            normP = 0; normQ = 0
            for (r, bus) in pvpq.enumerated() {
                mP[r] = (pCalc[bus] - pSpec[bus]) / vm[bus]
                normP = max(normP, abs(mP[r]))
            }
            for (r, bus) in pq.enumerated() {
                mQ[r] = (qCalc[bus] - qSpec[bus]) / vm[bus]
                normQ = max(normQ, abs(mQ[r]))
            }
        }

        computeMismatch()
        if normP < tolerancePu && normQ < tolerancePu {
            return state(vm, va, rounds: 0, converged: true, reason: nil,
                         mismatch: max(normP, normQ), pCalc: pCalc, qCalc: qCalc)
        }

        // --- factorize B′ / B″ (once; cached across calls when provided) ----
        let localCache = cache ?? FDPFFactorizationCache()
        guard let (bPrime, bDoublePrime) = localCache.factorizations(
            for: net, variant: options.fdpfVariant, pvpq: pvpq, pq: pq,
            pvpqPosition: pvpqPosition, pqPosition: pqPosition) else {
            return state(vm, va, rounds: 0, converged: false,
                         reason: "singular B′/B″ (\(options.fdpfVariant.rawValue.uppercased()))",
                         mismatch: max(normP, normQ), pCalc: pCalc, qCalc: qCalc)
        }

        // --- half-iterations -------------------------------------------------
        for round in 1...max(1, maxRounds) {
            // P half: B′·Δθ = ΔP/V, θ ← θ − Δθ.
            guard let dVa = bPrime.solve(mP) else {
                return state(vm, va, rounds: round - 1, converged: false,
                             reason: "B′ solve produced non-finite Δθ",
                             mismatch: max(normP, normQ), pCalc: pCalc, qCalc: qCalc)
            }
            for (r, bus) in pvpq.enumerated() { va[bus] -= dVa[r] }
            computeMismatch()
            if normP < tolerancePu && normQ < tolerancePu {
                return state(vm, va, rounds: round, converged: true, reason: nil,
                             mismatch: max(normP, normQ), pCalc: pCalc, qCalc: qCalc)
            }

            // Q half: B″·ΔV = ΔQ/V, V ← V − ΔV.
            if !pq.isEmpty {
                guard let dVm = bDoublePrime.solve(mQ) else {
                    return state(vm, va, rounds: round - 1, converged: false,
                                 reason: "B″ solve produced non-finite ΔV",
                                 mismatch: max(normP, normQ), pCalc: pCalc, qCalc: qCalc)
                }
                for (r, bus) in pq.enumerated() { vm[bus] -= dVm[r] }
                computeMismatch()
                if normP < tolerancePu && normQ < tolerancePu {
                    return state(vm, va, rounds: round, converged: true, reason: nil,
                                 mismatch: max(normP, normQ), pCalc: pCalc, qCalc: qCalc)
                }
            }

            // A divergent FDPF blows up fast; stop at the first non-finite
            // norm so the failure is reported as divergence, not iterations.
            if !normP.isFinite || !normQ.isFinite {
                return state(vm, va, rounds: round, converged: false,
                             reason: "diverged to non-finite mismatch",
                             mismatch: .nan, pCalc: pCalc, qCalc: qCalc)
            }
        }
        return state(vm, va, rounds: maxRounds, converged: false,
                     reason: "max iterations (\(maxRounds)) reached, "
                         + String(format: "mismatch %.3e pu", max(normP, normQ)),
                     mismatch: max(normP, normQ), pCalc: pCalc, qCalc: qCalc)
    }

    // MARK: Result assembly

    /// Branch flows and single-slack generator dispatch from a converged
    /// state — the same arithmetic as `NewtonRaphsonSolver`'s result block
    /// (which stays untouched; FDPF v1 has no pinning and no distributed
    /// slack, so only the plain paths are reproduced here).
    private func assemble(_ net: BusBranchNetwork, state: IterateState) -> PowerFlowSolution {
        let n = net.busCount
        var vmOut = state.vm, vaOut = state.va
        for i in 0..<n where !state.live[i] {
            vmOut[i] = .nan
            vaOut[i] = .nan
        }

        var flows = [BranchFlow](repeating: .zero, count: net.branches.count)
        for (k, branch) in net.branches.enumerated() {
            guard branch.inService, state.live[branch.from], state.live[branch.to] else { continue }
            let y = YbusBuilder.admittance(of: branch)
            let vf = ComplexD.polar(magnitude: state.vm[branch.from], angle: state.va[branch.from])
            let vt = ComplexD.polar(magnitude: state.vm[branch.to], angle: state.va[branch.to])
            let sf = vf * (y.yff * vf + y.yft * vt).conjugate
            let st = vt * (y.ytf * vf + y.ytt * vt).conjugate
            flows[k] = BranchFlow(pFromPu: sf.re, qFromPu: sf.im,
                                  pToPu: st.re, qToPu: st.im)
        }

        var genP = [Double](repeating: 0, count: net.generators.count)
        var genQ = [Double](repeating: 0, count: net.generators.count)
        for i in 0..<n where !state.gensAtBus[i].isEmpty && state.live[i] {
            let gens = state.gensAtBus[i]
            let qTotal = state.qCalc[i] + net.buses[i].qLoadPu
            for g in gens { genQ[g] = qTotal / Double(gens.count) }
            if state.isSlack[i] {
                let pTotal = state.pCalc[i] + net.buses[i].pLoadPu
                for g in gens { genP[g] = pTotal / Double(gens.count) }
            } else {
                for g in gens { genP[g] = net.generators[g].pPu }
            }
        }

        return PowerFlowSolution(
            converged: true, failureReason: nil, iterations: state.rounds,
            vmPu: vmOut, vaRad: vaOut, branchFlows: flows,
            genPPu: genP, genQPu: genQ, pinnedGenIndices: [],
            solutionPath: .fdpf,
            stages: [SolveStage(kind: .fdpf, iterations: state.rounds,
                                converged: true,
                                finalMismatchPu: state.finalMismatchPu)],
            // D80: consumed ≡ scheduled until a solver evaluates ZIP.
            loadPPu: net.buses.map(\.pLoadPu),
            loadQPu: net.buses.map(\.qLoadPu))
    }

    private func failed(_ net: BusBranchNetwork, reason: String,
                        iterations: Int = 0, mismatchPu: Double? = nil) -> PowerFlowSolution {
        PowerFlowSolution(
            converged: false, failureReason: reason, iterations: iterations,
            vmPu: [Double](repeating: .nan, count: net.busCount),
            vaRad: [Double](repeating: .nan, count: net.busCount),
            branchFlows: [BranchFlow](repeating: .zero, count: net.branches.count),
            genPPu: [Double](repeating: 0, count: net.generators.count),
            genQPu: [Double](repeating: 0, count: net.generators.count),
            pinnedGenIndices: [],
            solutionPath: .failed,
            stages: [SolveStage(kind: .fdpf, iterations: iterations,
                                converged: false, finalMismatchPu: mismatchPu)])
    }
}
