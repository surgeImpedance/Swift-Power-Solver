import Foundation

// Classic polar-form Newton-Raphson AC power flow, matching pandapower/pypower
// (newtonpf + pfsoln) so results validate against the pandapower oracle:
//
//   - PV / PQ / slack classification from bus types + generators; isolated
//     (de-energized) buses are excluded and report NaN voltages.
//   - Mismatch F = S(V) - Sspec, convergence on max |F| (pu).
//   - Jacobian in polar coordinates over the Ybus sparsity pattern; sparse
//     QR solve each iteration (Accelerate).
//   - Generator Q-limits: after each converged solve, PV buses whose gens
//     violate a limit are switched to PQ with Q pinned at the limit, then
//     re-solved from the current voltage (pandapower enforce_q_lims). Pinned
//     gens are never released.
//   - Multiple slack buses are supported (each holds V, θ = 0); a future
//     distributed slack plugs into exactly two places, marked "DISTRIBUTED
//     SLACK" below.

public struct NewtonRaphsonSolver: PowerFlowSolver {

    public init() {}

    public func solve(_ net: BusBranchNetwork,
                      options: PowerFlowOptions = PowerFlowOptions()) -> PowerFlowSolution {
        let n = net.busCount
        let ybus = YbusBuilder.build(net)

        // --- classification -------------------------------------------------
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
        let slackBuses = (0..<n).filter { isSlack[$0] }
        guard !slackBuses.isEmpty else {
            return failed(net, reason: "no slack bus")
        }

        // --- specified injections (pu) --------------------------------------
        // DISTRIBUTED SLACK: participation factors would scale an extra
        // mismatch term added to pSpec here.
        var pSpec = [Double](repeating: 0, count: n)
        var qSpec = [Double](repeating: 0, count: n)
        for (i, bus) in net.buses.enumerated() {
            pSpec[i] = -bus.pLoadPu
            qSpec[i] = -bus.qLoadPu
        }
        for gen in net.generators where gen.inService {
            pSpec[gen.bus] += gen.pPu
        }

        // --- initial voltage (flat start, gen setpoints) ---------------------
        // Angles start at the slack reference (usually 0; case118 pins 30°),
        // and the slack buses hold theirs for the whole solve.
        var vm = [Double](repeating: 1.0, count: n)
        var va = [Double](repeating: 0.0, count: n)
        if let refGen = net.generators.first(where: { $0.inService && isSlack[$0.bus] }) {
            va = [Double](repeating: refGen.vaRefRad, count: n)
        }
        // Warm start (opt-in): seed voltages from a provided guess. Only finite,
        // positive magnitudes are taken (a prior step's NaN dead-bus falls back
        // to flat); with the default nil guess these blocks are skipped and the
        // start is bit-identical to a flat start.
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

        // --- outer loop: NR + PV->PQ switching ------------------------------
        var pinnedGens = Set<Int>()
        var pinnedQOfGen = [Int: Double]()                   // gen -> limit it sits at
        var pinnedQAtBus = [Double](repeating: 0, count: n)  // sum of pinned gen Q
        var totalIterations = 0
        var pCalc = [Double](repeating: 0, count: n)
        var qCalc = [Double](repeating: 0, count: n)

        for _ in 0...max(1, net.generators.count) {
            // PV buses that still control voltage (some gen not pinned).
            var pvNow = [Bool](repeating: false, count: n)
            for i in 0..<n where isPV[i] && !isSlack[i] {
                pvNow[i] = gensAtBus[i].contains { !pinnedGens.contains($0) }
            }
            // Q setpoint including pinned generator output.
            var qSpecNow = qSpec
            for i in 0..<n { qSpecNow[i] += pinnedQAtBus[i] }
            // PV buses hold their setpoint voltage.
            for i in 0..<n where pvNow[i] {
                if let g = gensAtBus[i].first(where: { !pinnedGens.contains($0) }) {
                    vm[i] = net.generators[g].vSetPu
                }
            }

            let pv = (0..<n).filter { pvNow[$0] && live[$0] }
            let pq = (0..<n).filter { live[$0] && !isSlack[$0] && !pvNow[$0] }

            let inner = newtonIterate(
                ybus: ybus, pSpec: pSpec, qSpec: qSpecNow,
                pv: pv, pq: pq, vm: &vm, va: &va,
                pCalc: &pCalc, qCalc: &qCalc, options: options)
            totalIterations += inner.iterations
            guard inner.converged else {
                return failed(net, reason: inner.reason ?? "did not converge",
                              iterations: totalIterations)
            }

            guard options.enforceQLimits else { break }

            // Check reactive limits at PV buses (pandapower checks after
            // convergence and re-solves; slack gens are never limited).
            var violations = false
            for i in pv {
                let unpinned = gensAtBus[i].filter { !pinnedGens.contains($0) }
                guard !unpinned.isEmpty else { continue }
                // Q the unpinned gens must supply together.
                let qNeeded = qCalc[i] + net.buses[i].qLoadPu - pinnedQAtBus[i]
                let qMin = unpinned.reduce(0) { $0 + net.generators[$1].qMinPu }
                let qMax = unpinned.reduce(0) { $0 + net.generators[$1].qMaxPu }
                if qNeeded > qMax {
                    for g in unpinned {
                        pinnedGens.insert(g)
                        pinnedQOfGen[g] = net.generators[g].qMaxPu
                        pinnedQAtBus[i] += net.generators[g].qMaxPu
                    }
                    violations = true
                } else if qNeeded < qMin {
                    for g in unpinned {
                        pinnedGens.insert(g)
                        pinnedQOfGen[g] = net.generators[g].qMinPu
                        pinnedQAtBus[i] += net.generators[g].qMinPu
                    }
                    violations = true
                }
            }
            if !violations { break }
        }

        // --- results ----------------------------------------------------------
        var vmOut = vm, vaOut = va
        for i in 0..<n where !live[i] {
            vmOut[i] = .nan
            vaOut[i] = .nan
        }

        var flows = [BranchFlow](repeating: .zero, count: net.branches.count)
        for (k, branch) in net.branches.enumerated() {
            guard branch.inService, live[branch.from], live[branch.to] else { continue }
            let y = YbusBuilder.admittance(of: branch)
            let vf = ComplexD.polar(magnitude: vm[branch.from], angle: va[branch.from])
            let vt = ComplexD.polar(magnitude: vm[branch.to], angle: va[branch.to])
            let sf = vf * (y.yff * vf + y.yft * vt).conjugate
            let st = vt * (y.ytf * vf + y.ytt * vt).conjugate
            flows[k] = BranchFlow(pFromPu: sf.re, qFromPu: sf.im,
                                  pToPu: st.re, qToPu: st.im)
        }

        // Generator dispatch (pfsoln): bus injection + load = generation.
        var genP = [Double](repeating: 0, count: net.generators.count)
        var genQ = [Double](repeating: 0, count: net.generators.count)
        for i in 0..<n where !gensAtBus[i].isEmpty && live[i] {
            let gens = gensAtBus[i]
            // Reactive: pinned gens sit at their limit; the rest share equally.
            let unpinned = gens.filter { !pinnedGens.contains($0) }
            for g in gens where pinnedGens.contains(g) {
                genQ[g] = pinnedQOfGen[g] ?? 0
            }
            if !unpinned.isEmpty {
                let qRemaining = qCalc[i] + net.buses[i].qLoadPu - pinnedQAtBus[i]
                for g in unpinned { genQ[g] = qRemaining / Double(unpinned.count) }
            }
            // Active: PV gens keep their setpoint; slack buses absorb the
            // island imbalance. DISTRIBUTED SLACK: replace this per-bus
            // assignment with participation-factor shares.
            if isSlack[i] {
                let pTotal = pCalc[i] + net.buses[i].pLoadPu
                for g in gens { genP[g] = pTotal / Double(gens.count) }
            } else {
                for g in gens { genP[g] = net.generators[g].pPu }
            }
        }

        return PowerFlowSolution(
            converged: true, failureReason: nil, iterations: totalIterations,
            vmPu: vmOut, vaRad: vaOut, branchFlows: flows,
            genPPu: genP, genQPu: genQ, pinnedGenIndices: pinnedGens)
    }

    // MARK: - Inner Newton iteration

    private struct InnerResult {
        var converged: Bool
        var iterations: Int
        var reason: String?
    }

    /// Iterate to convergence for a fixed PV/PQ split. Updates vm/va in place
    /// and leaves the final calculated injections in pCalc/qCalc.
    private func newtonIterate(
        ybus: SparseComplexMatrix,
        pSpec: [Double], qSpec: [Double],
        pv: [Int], pq: [Int],
        vm: inout [Double], va: inout [Double],
        pCalc: inout [Double], qCalc: inout [Double],
        options: PowerFlowOptions
    ) -> InnerResult {
        let n = ybus.n
        let pvpq = pv + pq
        let npvpq = pvpq.count
        let npq = pq.count
        let dim = npvpq + npq

        // bus -> row/column position
        var thetaIndex = [Int](repeating: -1, count: n)  // in 0..<npvpq
        for (r, bus) in pvpq.enumerated() { thetaIndex[bus] = r }
        var vmIndex = [Int](repeating: -1, count: n)     // in 0..<npq
        for (r, bus) in pq.enumerated() { vmIndex[bus] = r }

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

        for iteration in 0...options.maxIterations {
            computeInjections()

            // Mismatch F = calc - spec.
            var f = [Double](repeating: 0, count: dim)
            var normF = 0.0
            for (r, bus) in pvpq.enumerated() {
                f[r] = pCalc[bus] - pSpec[bus]
                normF = max(normF, abs(f[r]))
            }
            for (r, bus) in pq.enumerated() {
                f[npvpq + r] = qCalc[bus] - qSpec[bus]
                normF = max(normF, abs(f[npvpq + r]))
            }
            if normF < options.tolerancePu {
                return InnerResult(converged: true, iterations: iteration, reason: nil)
            }
            if iteration == options.maxIterations {
                return InnerResult(converged: false, iterations: iteration,
                                   reason: "max iterations (\(options.maxIterations)) "
                                       + String(format: "reached, mismatch %.3e pu", normF))
            }
            if dim == 0 {
                // Only slack buses live: nothing to solve, already consistent.
                return InnerResult(converged: true, iterations: iteration, reason: nil)
            }

            // Jacobian over the Ybus sparsity pattern.
            var entries: [(row: Int, col: Int, value: Double)] = []
            entries.reserveCapacity(4 * ybus.nonZeroCount)
            for i in 0..<n {
                let rP = thetaIndex[i]                       // ΔP row (θ-order)
                let rQ = vmIndex[i]                          // ΔQ row
                if rP < 0 && rQ < 0 { continue }
                for k in ybus.rowStarts[i]..<ybus.rowStarts[i + 1] {
                    let j = ybus.columns[k]
                    let g = ybus.re[k], b = ybus.im[k]
                    let cTheta = thetaIndex[j]               // dθ_j column
                    let cVm = vmIndex[j]                     // dVm_j column
                    if cTheta < 0 && cVm < 0 { continue }

                    var dPdT = 0.0, dPdV = 0.0, dQdT = 0.0, dQdV = 0.0
                    if i == j {
                        dPdT = -qCalc[i] - b * vm[i] * vm[i]
                        dPdV = pCalc[i] / vm[i] + g * vm[i]
                        dQdT = pCalc[i] - g * vm[i] * vm[i]
                        dQdV = qCalc[i] / vm[i] - b * vm[i]
                    } else {
                        let theta = va[i] - va[j]
                        let c = cos(theta), s = sin(theta)
                        dPdT = vm[i] * vm[j] * (g * s - b * c)
                        dPdV = vm[i] * (g * c + b * s)
                        dQdT = -vm[i] * vm[j] * (g * c + b * s)
                        dQdV = vm[i] * (g * s - b * c)
                    }
                    if rP >= 0 {
                        if cTheta >= 0 { entries.append((rP, cTheta, dPdT)) }
                        if cVm >= 0 { entries.append((rP, npvpq + cVm, dPdV)) }
                    }
                    if rQ >= 0 {
                        if cTheta >= 0 { entries.append((npvpq + rQ, cTheta, dQdT)) }
                        if cVm >= 0 { entries.append((npvpq + rQ, npvpq + cVm, dQdV)) }
                    }
                }
            }

            guard let dx = SparseLinearSolver.solve(n: dim, entries: entries, rhs: f) else {
                return InnerResult(converged: false, iterations: iteration,
                                   reason: "singular Jacobian")
            }
            for (r, bus) in pvpq.enumerated() { va[bus] -= dx[r] }
            for (r, bus) in pq.enumerated() { vm[bus] -= dx[npvpq + r] }
        }
        return InnerResult(converged: false, iterations: options.maxIterations,
                           reason: "did not converge")
    }

    // MARK: - Helpers

    private func failed(_ net: BusBranchNetwork, reason: String,
                        iterations: Int = 0) -> PowerFlowSolution {
        PowerFlowSolution(
            converged: false, failureReason: reason, iterations: iterations,
            vmPu: [Double](repeating: .nan, count: net.busCount),
            vaRad: [Double](repeating: .nan, count: net.busCount),
            branchFlows: [BranchFlow](repeating: .zero, count: net.branches.count),
            genPPu: [Double](repeating: 0, count: net.generators.count),
            genQPu: [Double](repeating: 0, count: net.generators.count),
            pinnedGenIndices: [])
    }
}
