import Foundation

// DC power flow (MATPOWER makeBdc / dcpf formulation), the linear counterpart
// of the AC Newton-Raphson solver. Same input seam (BusBranchNetwork), same
// output type (PowerFlowSolution), same slack / classification / live-island
// conventions as NewtonRaphsonSolver, so DC and AC results are directly
// comparable on the same network.
//
// Assumptions: flat voltage (Vm = 1 pu), lossless (R and charging ignored,
// series reactance only), small angle differences. The active-power balance
// reduces to a single linear system  B'·θ = Pinj  solved once (no iteration).
//
//   b_k   = 1 / (x_k · tap_k)                 branch DC susceptance
//   B'    : Σ b_k on the diagonal of each end, −b_k off-diagonal (from/to)
//   Pfinj_k = −b_k · shift_k                  phase-shifter power injection
//   Pinj_i = Σ gen.p − load − gs − Pbusinj_i  bus injection (Pbusinj from Pfinj)
//   Pf_k  = b_k · (θ_f − θ_t − shift_k)       branch flow (P only); Pt = −Pf
//
// Reuses the real Accelerate sparse solve (SparseLinearSolver) that the AC
// Jacobian uses. PTDF / LODF factors (Piece 3, N-1) build directly on the
// `Bf` / `Pfinj` pieces exposed by DCModel below — marked "// PTDF" — but are
// deliberately not built here.

public struct DCPowerFlowSolver: PowerFlowSolver {

    public init() {}

    /// DC ignores the NR-only option fields (tolerance, maxIterations,
    /// enforceQLimits): the solve is a single direct linear solve.
    public func solve(_ net: BusBranchNetwork,
                      options: PowerFlowOptions = PowerFlowOptions()) -> PowerFlowSolution {
        _ = options
        let n = net.busCount

        // --- classification (mirrors NewtonRaphsonSolver) -------------------
        var isSlack = [Bool](repeating: false, count: n)
        var live = [Bool](repeating: false, count: n)
        for (i, bus) in net.buses.enumerated() {
            switch bus.type {
            case .slack: isSlack[i] = true; live[i] = true
            case .pv, .pq: live[i] = true
            case .isolated: break
            }
        }
        let slackBuses = (0..<n).filter { isSlack[$0] }
        guard !slackBuses.isEmpty else { return failed(net, reason: "no slack bus") }

        // Slack reference angles: each slack bus holds its gen's vaRefRad
        // (usually 0; case118 pins 30°), consistent with the AC path.
        var thetaRef = [Double](repeating: 0, count: n)
        for gen in net.generators where gen.inService && isSlack[gen.bus] {
            thetaRef[gen.bus] = gen.vaRefRad
        }

        // --- B' model -------------------------------------------------------
        let model = DCModel(net: net, live: live)

        // --- bus injections Pinj (pu) ---------------------------------------
        // Pinj = Σ gen.p − load − shunt gs − phase-shift injection.
        var pInj = [Double](repeating: 0, count: n)
        for (i, bus) in net.buses.enumerated() where live[i] {
            pInj[i] = -bus.pLoadPu - bus.gsPu - model.pBusInj[i]
        }
        for gen in net.generators where gen.inService && live[gen.bus] {
            pInj[gen.bus] += gen.pPu
        }

        // --- reduce to the non-slack live buses and solve -------------------
        let pvpq = (0..<n).filter { live[$0] && !isSlack[$0] }
        var reducedIndex = [Int](repeating: -1, count: n)
        for (r, bus) in pvpq.enumerated() { reducedIndex[bus] = r }

        var theta = thetaRef                 // slack buses keep their reference
        if !pvpq.isEmpty {
            var entries: [(row: Int, col: Int, value: Double)] = []
            entries.reserveCapacity(pvpq.count * 4)
            var rhs = [Double](repeating: 0, count: pvpq.count)
            for (r, i) in pvpq.enumerated() {
                rhs[r] = pInj[i]
                for (j, val) in model.row(i) {
                    guard live[j] else { continue }
                    if isSlack[j] {
                        rhs[r] -= val * thetaRef[j]     // move slack coupling to RHS
                    } else {
                        entries.append((r, reducedIndex[j], val))
                    }
                }
            }
            guard let solved = SparseLinearSolver.solve(
                n: pvpq.count, entries: entries, rhs: rhs) else {
                return failed(net, reason: "singular B' matrix")
            }
            for (r, i) in pvpq.enumerated() { theta[i] = solved[r] }
        }

        // --- assemble solution ---------------------------------------------
        var vmOut = [Double](repeating: 1.0, count: n)
        var vaOut = theta
        for i in 0..<n where !live[i] { vmOut[i] = .nan; vaOut[i] = .nan }

        // Branch flows: P only, Pt = −Pf (lossless).
        var flows = [BranchFlow](repeating: .zero, count: net.branches.count)
        for (k, branch) in net.branches.enumerated() {
            guard branch.inService, live[branch.from], live[branch.to] else { continue }
            let pf = model.bSeries[k] * (theta[branch.from] - theta[branch.to]
                                         - branch.shiftRad)
            flows[k] = BranchFlow(pFromPu: pf, qFromPu: 0, pToPu: -pf, qToPu: 0)
        }

        // Generator dispatch: PV/PQ gens hold their P; slack gens absorb the
        // island imbalance (same per-bus split rule as NR). Q is zero in DC.
        var gensAtBus = [[Int]](repeating: [], count: n)
        for (g, gen) in net.generators.enumerated() where gen.inService {
            gensAtBus[gen.bus].append(g)
        }
        var genP = [Double](repeating: 0, count: net.generators.count)
        let genQ = [Double](repeating: 0, count: net.generators.count)
        for i in 0..<n where !gensAtBus[i].isEmpty && live[i] {
            let gens = gensAtBus[i]
            if isSlack[i] {
                // Reconstruct the slack bus generation from the network flow:
                // Pgen = (B'·θ)_i + load + gs + Pbusinj.
                var pNet = 0.0
                for (j, val) in model.row(i) where live[j] { pNet += val * theta[j] }
                let pTotal = pNet + net.buses[i].pLoadPu + net.buses[i].gsPu
                    + model.pBusInj[i]
                for g in gens { genP[g] = pTotal / Double(gens.count) }
            } else {
                for g in gens { genP[g] = net.generators[g].pPu }
            }
        }

        return PowerFlowSolution(
            converged: true, failureReason: nil, iterations: 1,
            vmPu: vmOut, vaRad: vaOut, branchFlows: flows,
            genPPu: genP, genQPu: genQ, pinnedGenIndices: [])
    }

    private func failed(_ net: BusBranchNetwork, reason: String) -> PowerFlowSolution {
        PowerFlowSolution(
            converged: false, failureReason: reason, iterations: 0,
            vmPu: [Double](repeating: .nan, count: net.busCount),
            vaRad: [Double](repeating: .nan, count: net.busCount),
            branchFlows: [BranchFlow](repeating: .zero, count: net.branches.count),
            genPPu: [Double](repeating: 0, count: net.generators.count),
            genQPu: [Double](repeating: 0, count: net.generators.count),
            pinnedGenIndices: [])
    }
}

// MARK: - DC network model (B', Bf, phase-shift injections)

/// The DC building blocks from MATPOWER makeBdc. `bSeries` and `pFinj` are the
/// per-branch pieces PTDF / LODF (Piece 3) build on; `B'` is exposed row-wise
/// for the injection solve and the slack-generation reconstruction.
struct DCModel {
    let n: Int
    /// Per-branch DC susceptance b = 1/(x·tap); 0 for out-of-service / dead
    /// branches. Parallel to net.branches. // PTDF: Bf rows are ±bSeries.
    let bSeries: [Double]
    /// Per-branch phase-shift injection −b·shift. Parallel to net.branches.
    /// // PTDF: enters the flow equation as Pf = Bf·θ + pFinj.
    let pFinj: [Double]
    /// Per-bus phase-shift injection Σ over incident branches (from += , to −=).
    let pBusInj: [Double]

    /// B' stored as adjacency rows: bRows[i] = [(col j, value)], including the
    /// diagonal. Duplicate (parallel-branch) columns are summed.
    private let bRows: [[(Int, Double)]]

    init(net: BusBranchNetwork, live: [Bool]) {
        let n = net.busCount
        var bSeries = [Double](repeating: 0, count: net.branches.count)
        var pFinj = [Double](repeating: 0, count: net.branches.count)
        var pBusInj = [Double](repeating: 0, count: n)
        var acc = [[Int: Double]](repeating: [:], count: n)

        for (k, br) in net.branches.enumerated() {
            guard br.inService, live[br.from], live[br.to] else { continue }
            let tap = br.tap <= 0 ? 1.0 : br.tap
            let b = 1.0 / (br.x * tap)
            bSeries[k] = b
            let finj = -b * br.shiftRad
            pFinj[k] = finj
            pBusInj[br.from] += finj
            pBusInj[br.to] -= finj
            let f = br.from, t = br.to
            acc[f][f, default: 0] += b
            acc[t][t, default: 0] += b
            acc[f][t, default: 0] -= b
            acc[t][f, default: 0] -= b
        }

        self.n = n
        self.bSeries = bSeries
        self.pFinj = pFinj
        self.pBusInj = pBusInj
        self.bRows = acc.map { row in row.sorted { $0.key < $1.key }.map { ($0.key, $0.value) } }
    }

    /// Non-zero entries of B' row `i`, as (column, value).
    func row(_ i: Int) -> [(Int, Double)] { bRows[i] }
}
