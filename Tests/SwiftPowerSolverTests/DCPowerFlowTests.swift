import XCTest
@testable import SwiftPowerSolver

// Piece 2 oracle: DC power flow on the IEEE cases must reproduce pandapower's
// rundcpp solution (same ppc-level input data, so any difference is a solver
// bug). DC is linear, so agreement is expected at/near machine precision —
// like the AC path — not merely under the 1e-6 tolerance.
final class DCPowerFlowTests: XCTestCase {

    private let vaTol = 1e-6        // rad
    private let flowTol = 1e-4      // MW (loosened from 1e-6 pu * base)

    private func validate(case name: String) throws {
        let ref = try ReferenceCase.load(name)
        guard let dc = ref.dc else {
            throw XCTSkip("\(name).json has no dc block — run Tools/dump_reference.py")
        }
        let net = ref.network()
        let sol = DCPowerFlowSolver().solve(net)

        XCTAssertTrue(sol.converged, "\(name): \(sol.failureReason ?? "?")")

        // DC holds flat voltage.
        for (i, vm) in sol.vmPu.enumerated() where vm.isFinite {
            XCTAssertEqual(vm, 1.0, accuracy: 0, "\(name): Vm not flat at bus \(i)")
        }

        // --- bus angles ----------------------------------------------------
        let vaStats = ErrorStats(reference: dc.vaDeg.map { $0 * .pi / 180 },
                                 computed: sol.vaRad)
        XCTAssertLessThanOrEqual(vaStats.maxError, vaTol,
                                 "\(name): " + vaStats.description("Va", unit: "rad"))

        // --- branch P flows (MW) -------------------------------------------
        let base = net.baseMVA
        let pf = ErrorStats(reference: dc.branchPFromMw,
                            computed: sol.branchFlows.map { $0.pFromPu * base })
        let pt = ErrorStats(reference: dc.branchPToMw,
                            computed: sol.branchFlows.map { $0.pToPu * base })
        XCTAssertLessThanOrEqual(pf.maxError, flowTol,
                                 "\(name): " + pf.description("P from", unit: "MW"))
        XCTAssertLessThanOrEqual(pt.maxError, flowTol,
                                 "\(name): " + pt.description("P to", unit: "MW"))

        // Q is identically zero in DC.
        for (k, f) in sol.branchFlows.enumerated() {
            XCTAssertEqual(f.qFromPu, 0, "\(name): branch \(k) has nonzero Q from")
            XCTAssertEqual(f.qToPu, 0, "\(name): branch \(k) has nonzero Q to")
        }

        print(String(format: "%@ DC vs pandapower: max|ΔVa| %.3e rad (at %d), "
                     + "mean %.3e | max|ΔP| %.3e MW (at %d), mean %.3e",
                     name, vaStats.maxError, vaStats.worstIndex, vaStats.meanError,
                     pf.maxError, pf.worstIndex, pf.meanError))
    }

    func testCase14() throws { try validate(case: "case14") }
    func testCase39() throws { try validate(case: "case39") }
    func testCase118() throws { try validate(case: "case118") }

    /// case118 pins its slack at 30° — DC must carry the non-zero reference
    /// angle through (the AC harness's "30° surprise", checked here for DC).
    func testCase118SlackReferenceAngleHeld() throws {
        let ref = try ReferenceCase.load("case118")
        let net = ref.network()
        let sol = DCPowerFlowSolver().solve(net)
        let slackBus = net.buses.firstIndex { $0.type == .slack }!
        let slackGen = net.generators.first { $0.bus == slackBus }!
        XCTAssertEqual(sol.vaRad[slackBus], slackGen.vaRefRad, accuracy: 1e-12,
                       "slack angle not held at its reference")
        XCTAssertGreaterThan(abs(slackGen.vaRefRad), 0.5, "expected the 30° pin")
    }
}
