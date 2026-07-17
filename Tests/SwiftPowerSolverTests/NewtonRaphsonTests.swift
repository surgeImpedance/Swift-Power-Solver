import XCTest
@testable import SwiftPowerSolver

// Piece 1 oracle, part 1: NR on the IEEE cases must reproduce pandapower's
// solution (same ppc-level input data, so any difference is a solver bug).
//
// Tolerances: vm 1e-6 pu / va 1e-6 rad required; expected agreement is much
// tighter (~1e-10) since both solvers converge to mismatch < 1e-12 pu.
final class NewtonRaphsonTests: XCTestCase {

    private let vmTol = 1e-6        // pu
    private let vaTol = 1e-6        // rad
    private let flowTol = 1e-4      // MW / MVAr
    private let genTol = 1e-4       // MW / MVAr

    private func validate(case name: String, qLims: Bool) throws {
        let ref = try ReferenceCase.load(name)
        let net = ref.network()
        let refSol = qLims ? ref.solutions.qLims : ref.solutions.plain

        var options = PowerFlowOptions()
        options.tolerancePu = 1e-12          // matches the dump's 1e-10 MVA
        options.maxIterations = 30
        options.enforceQLimits = qLims
        let sol = NewtonRaphsonSolver().solve(net, options: options)

        XCTAssertTrue(sol.converged, "\(name): \(sol.failureReason ?? "?")")

        // --- voltages ---------------------------------------------------
        let vmStats = ErrorStats(reference: refSol.vmPu, computed: sol.vmPu)
        let vaStats = ErrorStats(reference: refSol.vaDeg.map { $0 * .pi / 180 },
                                 computed: sol.vaRad)
        XCTAssertLessThanOrEqual(vmStats.maxError, vmTol,
                                 "\(name): " + vmStats.description("Vm", unit: "pu"))
        XCTAssertLessThanOrEqual(vaStats.maxError, vaTol,
                                 "\(name): " + vaStats.description("Va", unit: "rad"))

        // --- branch flows (MW / MVAr) -------------------------------------
        let base = net.baseMVA
        let refFlows = refSol.branchFlows
        let pf = ErrorStats(reference: refFlows.map(\.pfMw),
                            computed: sol.branchFlows.map { $0.pFromPu * base })
        let qf = ErrorStats(reference: refFlows.map(\.qfMvar),
                            computed: sol.branchFlows.map { $0.qFromPu * base })
        let pt = ErrorStats(reference: refFlows.map(\.ptMw),
                            computed: sol.branchFlows.map { $0.pToPu * base })
        let qt = ErrorStats(reference: refFlows.map(\.qtMvar),
                            computed: sol.branchFlows.map { $0.qToPu * base })
        for (stats, label) in [(pf, "P from"), (qf, "Q from"), (pt, "P to"), (qt, "Q to")] {
            XCTAssertLessThanOrEqual(stats.maxError, flowTol,
                                     "\(name): " + stats.description(label, unit: "MW/MVAr"))
        }

        // --- generation, aggregated per bus -------------------------------
        // (pypower distributes Q among same-bus gens by a range rule; the
        // per-bus totals are what must agree.)
        func perBus(_ values: [Double]) -> [Int: Double] {
            var out: [Int: Double] = [:]
            for (g, gen) in net.generators.enumerated() where gen.inService {
                out[gen.bus, default: 0] += values[g]
            }
            return out
        }
        let refP = perBus(refSol.genPMw.map { $0 / base })
        let refQ = perBus(refSol.genQMvar.map { $0 / base })
        let ourP = perBus(sol.genPPu)
        let ourQ = perBus(sol.genQPu)
        for (bus, p) in refP {
            XCTAssertEqual(ourP[bus] ?? .nan, p, accuracy: genTol / base,
                           "\(name): gen P at bus \(bus)")
        }
        for (bus, q) in refQ {
            XCTAssertEqual(ourQ[bus] ?? .nan, q, accuracy: genTol / base,
                           "\(name): gen Q at bus \(bus)")
        }

        print(String(format: "%@%@ NR vs pandapower: %d iterations, max|ΔVm| %.3e pu, "
                     + "max|ΔVa| %.3e rad, max|Δflow| %.3e MVA",
                     name, qLims ? " (q_lims)" : "", sol.iterations,
                     vmStats.maxError, vaStats.maxError,
                     max(pf.maxError, qf.maxError, pt.maxError, qt.maxError)))
    }

    func testCase14() throws { try validate(case: "case14", qLims: false) }
    func testCase39() throws { try validate(case: "case39", qLims: false) }
    func testCase118() throws { try validate(case: "case118", qLims: false) }

    func testCase14QLims() throws { try validate(case: "case14", qLims: true) }
    func testCase39QLims() throws { try validate(case: "case39", qLims: true) }
    func testCase118QLims() throws { try validate(case: "case118", qLims: true) }

    /// The q_lims run must actually pin gens in case118 (6 in the reference)
    /// — guards against the enforcement silently never triggering.
    func testCase118QLimsPinsGenerators() throws {
        let ref = try ReferenceCase.load("case118")
        var options = PowerFlowOptions()
        options.tolerancePu = 1e-12
        options.enforceQLimits = true
        let sol = NewtonRaphsonSolver().solve(ref.network(), options: options)
        XCTAssertTrue(sol.converged)
        XCTAssertFalse(sol.pinnedGenIndices.isEmpty, "no generators were Q-limited")

        // Reference pinning: gens whose solved Q sits exactly on a limit.
        var refPinned = Set<Int>()
        for (g, gen) in ref.gens.enumerated() {
            let q = ref.solutions.qLims.genQMvar[g]
            if ref.buses[gen.bus].type != 3,
               abs(q - gen.qmaxMvar) < 1e-9 || abs(q - gen.qminMvar) < 1e-9 {
                refPinned.insert(g)
            }
        }
        XCTAssertEqual(sol.pinnedGenIndices, refPinned)
    }
}
