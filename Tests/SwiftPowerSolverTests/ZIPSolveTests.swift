import XCTest
@testable import SwiftPowerSolver

// D80 step 4b — the package solvers READ the ZIP components, compared for the
// first time against the oracles written before any Swift ZIP result existed
// (`Reference/zip.json` = pandapower 3.2.1; `Reference/zip_matpower.json` =
// MATPOWER 8.0 on identical data; ZIP exploration §4.2, §5.2 step 3).
//
// Gates: the existing NR gates — vm 1e-6 pu, va 1e-6 rad, flows 1e-4 MW/MVAr.
//
// WHICH ORACLE FOR WHAT, and why (measured 2026-09-03, D80 step 4b):
//   * Q-limits OFF: pandapower — it agrees with MATPOWER on identical data to
//     ≤ 1e-11 pu (case14/39) and carries the consumed load (res_load).
//   * Q-limits ON: MATPOWER — pandapower's `pfsoln._update_q` reconstructs
//     generator Q from the SCHEDULED `bus[:, QD]`, not the voltage-dependent
//     load, so its Q-limit check carries the PV-bus trap (§3.2). On case118
//     it pins NOTHING under ZIP; MATPOWER pins eight buses and the Swift
//     solvers land on MATPOWER's answer to 2.6e-07 pu (the export-path
//     residual). `testPandapowerQLimitBlockCarriesThePVTrapOnCase118` keeps
//     that finding as a control rather than prose.
//   * Iteration counts: MATPOWER's full-Newton count only, never
//     pandapower's partial-Newton 8/13/8.
//   * case118 angles vs MATPOWER are NOT gated: the export path carries a
//     1.3e-05 rad residual at constant power (§5.4 item 11). Angles on
//     case118 are gated against pandapower (Q-off, 1e-12) and fork-vs-package
//     (`NRAgreementTests`, 1e-9) instead.
final class ZIPSolveTests: XCTestCase {

    private let vmTol = 1e-6, vaTol = 1e-6, flowTol = 1e-4

    private func zipNetwork(_ name: String) throws -> (BusBranchNetwork, ReferenceCase, ZIPReference.Case) {
        let ref = try ReferenceCase.load(name)
        let z = try ZIPReference.load()
        let c = try XCTUnwrap(z.cases[name])
        var net = ref.network()
        let k = z.coefficients
        for i in net.buses.indices {
            net.buses[i].pLoadZPu = k.zP * net.buses[i].pLoadPu
            net.buses[i].pLoadIPu = k.iP * net.buses[i].pLoadPu
            net.buses[i].qLoadZPu = k.zQ * net.buses[i].qLoadPu
            net.buses[i].qLoadIPu = k.iQ * net.buses[i].qLoadPu
        }
        XCTAssertTrue(net.hasVoltageDependentLoad, name)
        return (net, ref, c)
    }

    private func maxDvm(_ a: [Double], _ b: [Double]) -> Double { zip(a, b).map { abs($0 - $1) }.max() ?? 0 }
    private func maxDvaRad(_ sol: PowerFlowSolution, _ vaDeg: [Double]) -> Double {
        zip(sol.vaRad, vaDeg).map { abs($0 - $1 * .pi / 180) }.max() ?? 0
    }
    private func pinnedBuses(_ sol: PowerFlowSolution, _ ref: ReferenceCase) -> Set<Int> {
        Set(sol.pinnedGenIndices.map { ref.gens[$0].bus })
    }
    private func slackBuses(_ ref: ReferenceCase) -> Set<Int> {
        Set(ref.buses.enumerated().compactMap { $0.element.type == 3 ? $0.offset : nil })
    }

    private func assertMatchesPandapower(_ sol: PowerFlowSolution, _ oracle: ZIPReference.Solution,
                                         _ ref: ReferenceCase, _ tag: String) {
        XCTAssertTrue(sol.converged, "\(tag): \(sol.failureReason ?? "")")
        let dvm = maxDvm(sol.vmPu, oracle.vmPu), dva = maxDvaRad(sol, oracle.vaDeg)
        XCTAssertLessThan(dvm, vmTol, "\(tag): max|ΔVm| \(dvm)")
        XCTAssertLessThan(dva, vaTol, "\(tag): max|ΔVa| \(dva)")
        var dflow = 0.0
        if let flows = oracle.branchFlows {
            for (k, f) in flows.enumerated() {
                dflow = max(dflow, abs(sol.branchFlows[k].pFromPu * ref.baseMva - f.pfMw))
                dflow = max(dflow, abs(sol.branchFlows[k].qFromPu * ref.baseMva - f.qfMvar))
            }
            XCTAssertLessThan(dflow, flowTol, "\(tag): max|Δflow| \(dflow) MW")
        }
        if let cl = oracle.consumedLoad {
            var dp = 0.0
            for (k, bus) in cl.bus.enumerated() {
                dp = max(dp, abs(sol.loadPPu[bus] * ref.baseMva - cl.pMw[k]))
                dp = max(dp, abs(sol.loadQPu[bus] * ref.baseMva - cl.qMvar[k]))
            }
            XCTAssertLessThan(dp, flowTol, "\(tag): consumed load max|Δ| \(dp) MW")
        }
        print("\(tag): it=\(sol.iterations) max|ΔVm|=\(String(format: "%.2e", dvm)) pu "
              + "max|ΔVa|=\(String(format: "%.2e", dva)) rad max|Δflow|=\(String(format: "%.2e", dflow)) MW")
    }

    // MARK: - Q-limits OFF: pandapower is the oracle (and anti-vacuity stays live)

    func testNewtonRaphsonMatchesPandapowerWithQLimitsOff() throws {
        for name in ["case14", "case39", "case118"] {
            let (net, ref, c) = try zipNetwork(name)
            let oracle = try XCTUnwrap(c.zip["default"])
            let sol = NewtonRaphsonSolver().solve(net, options: PowerFlowOptions(tolerancePu: 1e-12))
            assertMatchesPandapower(sol, oracle, ref, "\(name)/default NR")
            let cp = try XCTUnwrap(c.constantPower["default"])
            XCTAssertGreaterThan(maxDvm(sol.vmPu, cp.vmPu), vmTol, "\(name): ZIP answer ≡ constant power — vacuous")
        }
    }

    // MARK: - Q-limits ON: MATPOWER is the oracle (pinned set and magnitudes)

    func testNewtonRaphsonMatchesMATPOWERWithQLimitsOn() throws {
        let mp = try ZIPMatpowerReference.load()
        for name in ["case14", "case39", "case118"] {
            let (net, ref, _) = try zipNetwork(name)
            let oracle = try XCTUnwrap(mp.cases[name]?["q_lims"])
            let sol = NewtonRaphsonSolver().solve(net, options: PowerFlowOptions(tolerancePu: 1e-12, enforceQLimits: true))
            XCTAssertTrue(sol.converged, name)
            let dvm = maxDvm(sol.vmPu, oracle.vmPu), dva = maxDvaRad(sol, oracle.vaDeg)
            let slack = slackBuses(ref)
            let matpowerLimitedTheSlack = !Set(oracle.gensAtAQLimitBus0).intersection(slack).isEmpty
            if matpowerLimitedTheSlack {
                // case14: MATPOWER's enforce_q_lims also limits the SLACK
                // generator (and re-references); pandapower and this solver
                // never limit a slack. Measured 8.1e-03 pu apart under ZIP and
                // 7.7e-03 at constant power — a Q-limit-strategy difference,
                // not ZIP. Here Q-on is gated against pandapower's block
                // instead, which is untrapped on this case (no PV bus moves),
                // and fork-vs-package at 1e-9 (`NRAgreementTests`).
                let pp = try XCTUnwrap(try ZIPReference.load().cases[name]?.zip["q_lims"])
                XCTAssertLessThan(maxDvm(sol.vmPu, pp.vmPu), vmTol, "\(name) q_lims vs pandapower (slack-limit case)")
                XCTAssertGreaterThan(dvm, 1e-3, "\(name): premise — MATPOWER's slack-limited answer differs (measured 8.1e-03)")
                print("\(name)/q_lims: MATPOWER limits the slack; gated against pandapower instead (max|ΔVm| vs MATPOWER \(String(format: "%.2e", dvm)))")
                continue
            }
            XCTAssertLessThan(dvm, vmTol, "\(name) q_lims vs MATPOWER: max|ΔVm| \(dvm)")
            if name != "case118" {
                XCTAssertLessThan(dva, vaTol, "\(name) q_lims vs MATPOWER: max|ΔVa| \(dva)")
            }
            // The pinned set: MATPOWER lists every generator sitting at a Q
            // limit; the Swift solver never pins a slack.
            let expected = Set(oracle.gensAtAQLimitBus0).subtracting(slack)
            XCTAssertEqual(pinnedBuses(sol, ref), expected, "\(name): PV buses switched to PQ (MATPOWER)")
            print("\(name)/q_lims NR vs MATPOWER: it=\(sol.iterations) (MATPOWER \(oracle.iterations)) "
                  + "max|ΔVm|=\(String(format: "%.2e", dvm)) max|ΔVa|=\(String(format: "%.2e", dva)) rad pinned=\(pinnedBuses(sol, ref).sorted())")
        }
    }

    /// The finding kept as a control: pandapower's ZIP q_lims block on case118
    /// is the PV-bus trap. Swift lands on MATPOWER's magnitudes and pinned
    /// set; pandapower's block is 6.5e-04 pu away and pins nothing.
    func testPandapowerQLimitBlockCarriesThePVTrapOnCase118() throws {
        let (net, ref, c) = try zipNetwork("case118")
        let pp = try XCTUnwrap(c.zip["q_lims"])
        let mp = try XCTUnwrap(try ZIPMatpowerReference.load().cases["case118"]?["q_lims"])
        let sol = NewtonRaphsonSolver().solve(net, options: PowerFlowOptions(tolerancePu: 1e-12, enforceQLimits: true))
        XCTAssertTrue(sol.converged)
        let toPP = maxDvm(sol.vmPu, pp.vmPu), toMP = maxDvm(sol.vmPu, mp.vmPu)
        XCTAssertLessThan(toMP, vmTol, "Swift must sit on MATPOWER's case118 ZIP Q-on answer")
        XCTAssertGreaterThan(toPP, 1e-4, "premise: pandapower's block is the trapped one (measured 6.5e-04 pu)")
        let ppPinned = Set(pp.busType.enumerated().compactMap { i, t in ref.buses[i].type == 2 && t == 1 ? i : nil })
        XCTAssertTrue(ppPinned.isEmpty, "premise: pandapower switched no PV bus under ZIP on case118")
        XCTAssertEqual(pinnedBuses(sol, ref).count, 8)
        print("case118 ZIP q_lims: swift→MATPOWER \(String(format: "%.2e", toMP)) pu, swift→pandapower \(String(format: "%.2e", toPP)) pu")
    }

    /// Iteration count against MATPOWER's full-Newton count only. Measured
    /// 2026-09-03 (tolerancePu 1e-12, flat, Q-off): Swift 4/5/4 vs MATPOWER
    /// 4/5/5 — case118 one fewer, at the tolerance edge on data that differs
    /// by the export residual. Pinned at ±1, never against pandapower's
    /// partial-Newton 8/13/8.
    func testIterationCountIsMATPOWERsFullNewtonCount() throws {
        let mp = try ZIPMatpowerReference.load()
        for name in ["case14", "case39", "case118"] {
            let (net, _, c) = try zipNetwork(name)
            let sol = NewtonRaphsonSolver().solve(net, options: PowerFlowOptions(tolerancePu: 1e-12))
            XCTAssertTrue(sol.converged)
            let mpIt = try XCTUnwrap(mp.cases[name]?["default"]?.iterations)
            let ppIt = try XCTUnwrap(c.zip["default"]?.iterations)
            XCTAssertLessThanOrEqual(abs(sol.iterations - mpIt), 1, "\(name): Swift \(sol.iterations) vs MATPOWER \(mpIt)")
            XCTAssertLessThan(sol.iterations, ppIt, "\(name): a full Newton must beat pandapower's partial-Newton count \(ppIt)")
            print("\(name) ZIP NR iterations: swift=\(sol.iterations) matpower=\(mpIt) pandapower=\(ppIt)")
        }
    }

    // MARK: - FDPF (mismatch-only) and the engine reach the same answer

    func testFDPFMismatchOnlyConvergesToTheZIPAnswer() throws {
        for name in ["case14", "case39", "case118"] {
            let (net, ref, c) = try zipNetwork(name)
            let oracle = try XCTUnwrap(c.zip["default"])
            for variant in [FDPFVariant.bx, .xb] {
                var o = PowerFlowOptions(tolerancePu: 1e-8, maxIterations: 200)
                o.fdpfVariant = variant
                let sol = FastDecoupledSolver().solve(net, options: o)
                assertMatchesPandapower(sol, oracle, ref, "\(name) FDPF-\(variant.rawValue)")
                var cp = net
                for i in cp.buses.indices { cp.buses[i].pLoadZPu = 0; cp.buses[i].pLoadIPu = 0; cp.buses[i].qLoadZPu = 0; cp.buses[i].qLoadIPu = 0 }
                let base = FastDecoupledSolver().solve(cp, options: o)
                print("\(name) FDPF-\(variant.rawValue) rounds: constant-power=\(base.iterations) zip=\(sol.iterations)")
            }
            // The engine's warm-start → NR path with Q-limits lands on the same
            // answer the direct NR does (MATPOWER's, by the test above).
            var e = PowerFlowOptions(enforceQLimits: true)
            e.method = .fastDecoupledWarmStart
            e.autoFallback = true
            let engine = PowerFlowEngine().solve(net, options: e)
            let direct = NewtonRaphsonSolver().solve(net, options: PowerFlowOptions(enforceQLimits: true))
            XCTAssertTrue(engine.converged && direct.converged, name)
            XCTAssertLessThan(maxDvm(engine.vmPu, direct.vmPu), vmTol, "\(name): engine warm-start vs direct NR")
            XCTAssertEqual(engine.pinnedGenIndices, direct.pinnedGenIndices, name)
        }
    }

    // MARK: - DC is a no-op under ZIP

    func testDCIsUnchangedByComponents() throws {
        for name in ["case14", "case118"] {
            let (net, ref, _) = try zipNetwork(name)
            let a = DCPowerFlowSolver().solve(ref.network(), options: PowerFlowOptions())
            let b = DCPowerFlowSolver().solve(net, options: PowerFlowOptions())
            XCTAssertEqual(a.vaRad.map(\.bitPattern), b.vaRad.map(\.bitPattern), name)
            XCTAssertEqual(a.loadPPu.map(\.bitPattern), b.loadPPu.map(\.bitPattern), name)
        }
    }
}
