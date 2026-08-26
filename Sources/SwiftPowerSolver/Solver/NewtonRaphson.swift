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

        // --- distributed slack (opt-in) -------------------------------------
        // Any in-service generator carrying a slack weight switches the solve
        // to distributed slack (pandapower distributed_slack=True): weights are
        // normalized to sum to 1 across contributors and aggregated per bus; the
        // first slack bus stays the angle reference (θ fixed) while every
        // contributor shares the imbalance by weight. With no weights this is
        // all inert and the solve below is the single-slack path unchanged.
        let contributors = net.generators.indices.filter {
            net.generators[$0].inService && (net.generators[$0].slackWeight ?? 0) != 0
        }
        let distributed = !contributors.isEmpty
        let angleRef = slackBuses[0]
        var swGen = [Double](repeating: 0, count: net.generators.count)  // per-gen, normalized
        var swBus = [Double](repeating: 0, count: n)                     // per-bus, normalized
        if distributed {
            let total = contributors.reduce(0.0) { $0 + (net.generators[$1].slackWeight ?? 0) }
            for g in contributors {
                swGen[g] = (net.generators[g].slackWeight ?? 0) / total
                swBus[net.generators[g].bus] += swGen[g]
            }
        }

        // --- specified injections (pu) --------------------------------------
        //
        // `effectiveP` is each generator's fixed active injection: its setpoint,
        // or — once distributed slack has pinned it at a regulating limit — that
        // limit. Contributors carry an additional −swGen·slack on top, solved
        // for. With no P limits nothing is ever pinned, so this stays equal to
        // `pPu` throughout and pSpec is exactly what it was before.
        var effectiveP = net.generators.map(\.pPu)
        var pSpec = [Double](repeating: 0, count: n)
        var qSpec = [Double](repeating: 0, count: n)
        for (i, bus) in net.buses.enumerated() {
            pSpec[i] = -bus.pLoadPu
            qSpec[i] = -bus.qLoadPu
        }
        func rebuildPSpec() {
            for (i, bus) in net.buses.enumerated() { pSpec[i] = -bus.pLoadPu }
            for (g, gen) in net.generators.enumerated() where gen.inService {
                pSpec[gen.bus] += effectiveP[g]
            }
        }
        rebuildPSpec()

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
        var qLimitRestartCount = 0                           // outer re-pin cycles
        var pCalc = [Double](repeating: 0, count: n)
        var qCalc = [Double](repeating: 0, count: n)

        // Distributed-slack scalar: the imbalance shared across contributors.
        // Initial guess = total generation setpoint − total load (pandapower's
        // (ΣPg − ΣPd)/baseMVA, already per-unit here). Unused in single-slack.
        let totalLoadPu = net.buses.reduce(0.0) { $0 + $1.pLoadPu }
        func slackGuess() -> Double {
            var g = 0.0
            for (i, gen) in net.generators.enumerated() where gen.inService {
                g += effectiveP[i]
            }
            return g - totalLoadPu
        }
        var slack = distributed ? slackGuess() : 0.0

        // P-limit (regulating-range) state. Empty and inert unless a contributor
        // actually carries a finite limit that the distribution violates.
        var pPinnedGens = Set<Int>()          // held at pMin/pMax
        var pSaturatedGens = Set<Int>()       // regulating beyond its range (see below)
        var activeContributors = Set(contributors)

        /// Re-normalize participation across the contributors still regulating,
        /// so the remaining units pick up a pinned unit's share in proportion to
        /// their own participation — not equally.
        func renormalizeParticipation() {
            for i in 0..<n { swBus[i] = 0 }
            for g in net.generators.indices where !activeContributors.contains(g) {
                swGen[g] = 0
            }
            // `.sorted()` ON BOTH, AND IT IS LOAD-BEARING, NOT TIDINESS.
            // `activeContributors` is a `Set<Int>`, and Swift randomizes Set
            // iteration order per PROCESS. Folding a sum over it — and
            // accumulating into `swBus` — therefore adds the same numbers in a
            // different order on every launch, and floating-point addition is
            // not associative: the result moves by 1-2 ULP between runs of the
            // same input. That is D-RL-04. Measured here before the fix: a
            // network with a P-limit pin and weights 1.0 against five at
            // 1.1e-16 produced THREE distinct output digests across 16 launches
            // of one binary. The site at :237 below was already sorted, which
            // is what made these two read as oversights rather than a
            // convention.
            let ordered = activeContributors.sorted()
            let total = ordered.reduce(0.0) {
                $0 + (net.generators[$1].slackWeight ?? 0)
            }
            guard total > 0 else { return }
            for g in ordered {
                swGen[g] = (net.generators[g].slackWeight ?? 0) / total
                swBus[net.generators[g].bus] += swGen[g]
            }
        }

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

            let pv: [Int]
            let pq: [Int]
            if distributed {
                // Only the angle-reference bus is θ-fixed; other slack buses are
                // voltage-controlled θ-variable buses (PV), like pandapower
                // demoting extra refs to PV. A PV bus whose gens are all pinned
                // drops to PQ, exactly as in single-slack.
                pv = (0..<n).filter { $0 != angleRef && live[$0] && (pvNow[$0] || isSlack[$0]) }
                pq = (0..<n).filter { $0 != angleRef && live[$0] && !pvNow[$0] && !isSlack[$0] }
            } else {
                pv = (0..<n).filter { pvNow[$0] && live[$0] }
                pq = (0..<n).filter { live[$0] && !isSlack[$0] && !pvNow[$0] }
            }

            let inner: InnerResult
            if distributed {
                inner = newtonIterateDistributed(
                    ybus: ybus, pSpec: pSpec, qSpec: qSpecNow,
                    pv: pv, pq: pq, angleRef: angleRef, swBus: swBus,
                    vm: &vm, va: &va, slack: &slack,
                    pCalc: &pCalc, qCalc: &qCalc, options: options)
            } else {
                inner = newtonIterate(
                    ybus: ybus, pSpec: pSpec, qSpec: qSpecNow,
                    pv: pv, pq: pq, vm: &vm, va: &va,
                    pCalc: &pCalc, qCalc: &qCalc, options: options)
            }
            totalIterations += inner.iterations
            guard inner.converged else {
                return failed(net, reason: inner.reason ?? "did not converge",
                              iterations: totalIterations)
            }

            var violations = false

            // --- P limits: pin and redistribute ------------------------------
            //
            // The MW analogue of the Q-limit switching below, and checked the
            // same way: after convergence, not during. A contributor delivering
            // outside its regulating range is fixed AT the violated bound, drops
            // out of the participating set, and the remaining units re-normalize
            // and re-solve — so they absorb its share in proportion to their own
            // participation. Pinning pushes the survivors FURTHER in the same
            // direction, so a pin can cascade; it can never require un-pinning,
            // which is what makes this loop monotone and free of cycling.
            //
            // Every violator pins in one pass rather than one-at-a-time: all
            // contributors move together with the sign of `slack`, so a unit
            // past its bound now cannot be brought back inside by pinning
            // another. Batch and sequential pinning therefore reach the same
            // fixed point, and batching is order-independent.
            //
            // This convention is ours, not pandapower's — pandapower does not
            // enforce P limits at all (see `Generator.pMinPu`). It is validated
            // by network equivalence instead: the pinned answer must equal
            // pandapower's UNLIMITED distributed slack on the network where the
            // pinned units are fixed at their limits with zero participation.
            if distributed && !activeContributors.isEmpty {
                var toPin: [(gen: Int, limit: Double)] = []
                for g in activeContributors.sorted() {
                    let delivered = effectiveP[g] - swGen[g] * slack
                    if let hi = net.generators[g].pMaxPu, delivered > hi {
                        toPin.append((g, hi))
                    } else if let lo = net.generators[g].pMinPu, delivered < lo {
                        toPin.append((g, lo))
                    }
                }
                // SATURATION. If every remaining contributor is past its range
                // there is nothing left to balance the island, and pinning them
                // all would leave the slack column identically zero — a singular
                // Jacobian reported as a numeric failure rather than the physical
                // condition it is. So the largest-participation unit (ties by
                // lowest index) keeps regulating, beyond its range, and is
                // reported as saturated. The caller sees a converged answer and
                // an explicit "AGC has run out of room" signal.
                if toPin.count == activeContributors.count {
                    let keep = activeContributors.sorted {
                        swGen[$0] != swGen[$1] ? swGen[$0] > swGen[$1] : $0 < $1
                    }[0]
                    toPin.removeAll { $0.gen == keep }
                    pSaturatedGens.insert(keep)
                }
                if !toPin.isEmpty {
                    for (g, limit) in toPin {
                        pPinnedGens.insert(g)
                        activeContributors.remove(g)
                        effectiveP[g] = limit
                    }
                    renormalizeParticipation()
                    rebuildPSpec()
                    slack = slackGuess()
                    violations = true
                }
            }

            guard options.enforceQLimits else {
                if !violations { break } else { continue }
            }

            // Check reactive limits at PV buses (pandapower checks after
            // convergence and re-solves; slack gens are never limited).
            var qPinnedThisPass = false
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
                    qPinnedThisPass = true
                } else if qNeeded < qMin {
                    for g in unpinned {
                        pinnedGens.insert(g)
                        pinnedQOfGen[g] = net.generators[g].qMinPu
                        pinnedQAtBus[i] += net.generators[g].qMinPu
                    }
                    violations = true
                    qPinnedThisPass = true
                }
            }
            if qPinnedThisPass { qLimitRestartCount += 1 }
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
            // Active: distributed slack shares the imbalance by weight —
            // each contributor delivers its setpoint minus its share of the
            // slack (gen.pPu − sw·slack); non-contributors keep their setpoint.
            // Single-slack: PV gens keep their setpoint; the slack bus absorbs
            // the island imbalance.
            // A P-pinned unit has swGen == 0 and effectiveP == its limit, so the
            // one expression covers both: pinned units report exactly the bound
            // they are held at, regulating units their setpoint less their share.
            if distributed {
                for g in gens { genP[g] = effectiveP[g] - swGen[g] * slack }
            } else if isSlack[i] {
                let pTotal = pCalc[i] + net.buses[i].pLoadPu
                for g in gens { genP[g] = pTotal / Double(gens.count) }
            } else {
                for g in gens { genP[g] = net.generators[g].pPu }
            }
        }

        return PowerFlowSolution(
            converged: true, failureReason: nil, iterations: totalIterations,
            vmPu: vmOut, vaRad: vaOut, branchFlows: flows,
            genPPu: genP, genQPu: genQ, pinnedGenIndices: pinnedGens,
            pLimitedGenIndices: pPinnedGens,
            pSaturatedGenIndices: pSaturatedGens,
            qLimitRestarts: qLimitRestartCount)
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

    // MARK: - Inner Newton iteration (distributed slack)

    /// Distributed-slack variant: an extra scalar unknown `slack` is added, the
    /// angle-reference bus gains a P-equation, and each bus's active mismatch
    /// carries `+ swBus·slack` (pandapower's `mis = V·conj(Ybus·V) − Sbus +
    /// slack_weights·slack`). Variables `[slack, θ_pvpq, Vm_pq]`, equations
    /// `[P_angleRef, P_pvpq, Q_pq]` — square, +1/+1 over the single-slack solve.
    /// θ of the angle reference stays fixed, so power is shared but the angle
    /// anchor is not.
    private func newtonIterateDistributed(
        ybus: SparseComplexMatrix,
        pSpec: [Double], qSpec: [Double],
        pv: [Int], pq: [Int], angleRef: Int, swBus: [Double],
        vm: inout [Double], va: inout [Double], slack: inout Double,
        pCalc: inout [Double], qCalc: inout [Double],
        options: PowerFlowOptions
    ) -> InnerResult {
        let n = ybus.n
        let pvpq = pv + pq
        let npvpq = pvpq.count
        let npq = pq.count
        let dim = 1 + npvpq + npq                        // slack + θ_pvpq + Vm_pq

        // Row / column maps. Column 0 is the slack scalar. P-equation rows:
        // 0 = angle reference, 1..npvpq = pvpq. θ columns: 1..npvpq (the angle
        // reference has none). Q-row and Vm-column share the same index.
        var pRow = [Int](repeating: -1, count: n)
        pRow[angleRef] = 0
        for (r, bus) in pvpq.enumerated() { pRow[bus] = 1 + r }
        var thetaCol = [Int](repeating: -1, count: n)
        for (r, bus) in pvpq.enumerated() { thetaCol[bus] = 1 + r }
        var vmRC = [Int](repeating: -1, count: n)
        for (r, bus) in pq.enumerated() { vmRC[bus] = 1 + npvpq + r }

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

            // Mismatch: P rows carry the distributed slack term + sw·slack.
            var f = [Double](repeating: 0, count: dim)
            var normF = 0.0
            func setF(_ row: Int, _ value: Double) {
                f[row] = value; normF = max(normF, abs(value))
            }
            setF(0, pCalc[angleRef] - pSpec[angleRef] + swBus[angleRef] * slack)
            for (r, bus) in pvpq.enumerated() {
                setF(1 + r, pCalc[bus] - pSpec[bus] + swBus[bus] * slack)
            }
            for (r, bus) in pq.enumerated() {
                setF(1 + npvpq + r, qCalc[bus] - qSpec[bus])
            }
            if normF < options.tolerancePu {
                return InnerResult(converged: true, iterations: iteration, reason: nil)
            }
            if iteration == options.maxIterations {
                return InnerResult(converged: false, iterations: iteration,
                                   reason: "max iterations (\(options.maxIterations)) "
                                       + String(format: "reached, mismatch %.3e pu", normF))
            }

            var entries: [(row: Int, col: Int, value: Double)] = []
            entries.reserveCapacity(4 * ybus.nonZeroCount + npvpq + 1)

            // Slack column: ∂P[i]/∂slack = swBus[i] (∂Q/∂slack = 0).
            if swBus[angleRef] != 0 { entries.append((0, 0, swBus[angleRef])) }
            for (r, bus) in pvpq.enumerated() where swBus[bus] != 0 {
                entries.append((1 + r, 0, swBus[bus]))
            }

            // Power-flow Jacobian over the Ybus sparsity pattern.
            for i in 0..<n {
                let rP = pRow[i]                              // ΔP row
                let rQ = vmRC[i]                              // ΔQ row (pq only)
                if rP < 0 && rQ < 0 { continue }
                for k in ybus.rowStarts[i]..<ybus.rowStarts[i + 1] {
                    let j = ybus.columns[k]
                    let g = ybus.re[k], b = ybus.im[k]
                    let cTheta = thetaCol[j]                  // dθ_j column
                    let cVm = vmRC[j]                         // dVm_j column
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
                        if cVm >= 0 { entries.append((rP, cVm, dPdV)) }
                    }
                    if rQ >= 0 {
                        if cTheta >= 0 { entries.append((rQ, cTheta, dQdT)) }
                        if cVm >= 0 { entries.append((rQ, cVm, dQdV)) }
                    }
                }
            }

            guard let dx = SparseLinearSolver.solve(n: dim, entries: entries, rhs: f) else {
                return InnerResult(converged: false, iterations: iteration,
                                   reason: "singular Jacobian")
            }
            slack -= dx[0]
            for (r, bus) in pvpq.enumerated() { va[bus] -= dx[1 + r] }
            for (r, bus) in pq.enumerated() { vm[bus] -= dx[1 + npvpq + r] }
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
