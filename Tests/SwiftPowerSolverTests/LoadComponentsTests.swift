import XCTest
@testable import SwiftPowerSolver

// D80 / ZIP exploration §5.2 step 2 — the load-component DATA MODEL, and the
// proof that it is inert.
//
// Four absolute Z/I components on `Bus`, consumed-load fields on
// `PowerFlowSolution`, components on `LoadStep.BusLoad`. NO SOLVER READS THE
// COMPONENTS. This file pins that claim the only way it can be pinned: a
// network carrying nonzero components must solve BIT-IDENTICALLY to the same
// network with the components zeroed, on every solver — because the solvers
// never look. When a solver starts evaluating the polynomial (step 4), the
// bit-identity assertions here are the ones that must be re-premised, and
// `hasVoltageDependentLoad` is the flag that selects the unchanged
// constant-power path.
final class LoadComponentsTests: XCTestCase {

    private func withComponents(_ net: BusBranchNetwork) -> BusBranchNetwork {
        var z = net
        // Nontrivial mix on every loaded bus: 30% Z, 30% I of P; 50% Z, 30% I of Q.
        for i in z.buses.indices where z.buses[i].pLoadPu != 0 || z.buses[i].qLoadPu != 0 {
            z.buses[i].pLoadZPu = 0.3 * z.buses[i].pLoadPu
            z.buses[i].pLoadIPu = 0.3 * z.buses[i].pLoadPu
            z.buses[i].qLoadZPu = 0.5 * z.buses[i].qLoadPu
            z.buses[i].qLoadIPu = 0.3 * z.buses[i].qLoadPu
        }
        return z
    }

    // MARK: - Model

    func testComponentsDefaultToZeroAndFlagIsFalse() throws {
        let net = try ReferenceCase.load("case14").network()
        for b in net.buses {
            XCTAssertEqual(b.pLoadZPu, 0); XCTAssertEqual(b.pLoadIPu, 0)
            XCTAssertEqual(b.qLoadZPu, 0); XCTAssertEqual(b.qLoadIPu, 0)
            XCTAssertFalse(b.hasVoltageDependentLoad)
        }
        XCTAssertFalse(net.hasVoltageDependentLoad)
        let z = withComponents(net)
        XCTAssertTrue(z.hasVoltageDependentLoad)
        XCTAssertGreaterThan(z.buses.filter(\.hasVoltageDependentLoad).count, 0,
                             "the fixture must carry a nontrivial mix or this file is vacuous")
        // `pLoadPu` keeps its meaning: the TOTAL is untouched by setting components.
        for (a, b) in zip(net.buses, z.buses) {
            XCTAssertEqual(a.pLoadPu.bitPattern, b.pLoadPu.bitPattern)
            XCTAssertEqual(a.qLoadPu.bitPattern, b.qLoadPu.bitPattern)
        }
    }

    // MARK: - Consumed load ≡ scheduled while nothing evaluates ZIP

    func testConsumedLoadEqualsScheduledOnEverySolver() throws {
        let net = try ReferenceCase.load("case14").network()
        let nr = NewtonRaphsonSolver().solve(net, options: PowerFlowOptions(tolerancePu: 1e-12))
        let fd = FastDecoupledSolver().solve(net, options: PowerFlowOptions(tolerancePu: 1e-8))
        let dc = DCPowerFlowSolver().solve(net, options: PowerFlowOptions())
        for (name, sol) in [("nr", nr), ("fdpf", fd), ("dc", dc)] {
            XCTAssertTrue(sol.converged, name)
            XCTAssertEqual(sol.loadPPu.count, net.busCount, "\(name): loadPPu parallel to buses")
            XCTAssertEqual(sol.loadQPu.count, net.busCount, "\(name): loadQPu parallel to buses")
            for (i, b) in net.buses.enumerated() {
                XCTAssertEqual(sol.loadPPu[i].bitPattern, b.pLoadPu.bitPattern, "\(name) bus \(i) P")
                XCTAssertEqual(sol.loadQPu[i].bitPattern, b.qLoadPu.bitPattern, "\(name) bus \(i) Q")
            }
        }
        // Failed solves carry no consumed-load figure rather than a wrong one.
        var bad = net
        for i in bad.buses.indices { bad.buses[i].pLoadPu *= 5; bad.buses[i].qLoadPu *= 5 }
        let failed = NewtonRaphsonSolver().solve(bad, options: PowerFlowOptions())
        XCTAssertFalse(failed.converged, "premise: case14 ×5 must fail")
        XCTAssertTrue(failed.loadPPu.isEmpty && failed.loadQPu.isEmpty)
    }

    // MARK: - Inertness: nonzero components change NOTHING in any solver

    func testComponentsAreInertOnEverySolver() throws {
        for name in ["case14", "case39", "case118"] {
            let net = try ReferenceCase.load(name).network()
            let z = withComponents(net)
            XCTAssertTrue(z.hasVoltageDependentLoad, name)

            for qlims in [false, true] {
                let a = NewtonRaphsonSolver().solve(net, options: PowerFlowOptions(tolerancePu: 1e-12, enforceQLimits: qlims))
                let b = NewtonRaphsonSolver().solve(z, options: PowerFlowOptions(tolerancePu: 1e-12, enforceQLimits: qlims))
                assertBitIdentical(a, b, "\(name) NR qlims=\(qlims)")
            }
            let fa = FastDecoupledSolver().solve(net, options: PowerFlowOptions(tolerancePu: 1e-8))
            let fb = FastDecoupledSolver().solve(z, options: PowerFlowOptions(tolerancePu: 1e-8))
            assertBitIdentical(fa, fb, "\(name) FDPF")
            let da = DCPowerFlowSolver().solve(net, options: PowerFlowOptions())
            let db = DCPowerFlowSolver().solve(z, options: PowerFlowOptions())
            assertBitIdentical(da, db, "\(name) DC")
            var opts = PowerFlowOptions(enforceQLimits: true)
            opts.method = .fastDecoupledWarmStart
            opts.autoFallback = true
            let ea = PowerFlowEngine().solve(net, options: opts)
            let eb = PowerFlowEngine().solve(z, options: opts)
            assertBitIdentical(ea, eb, "\(name) engine warm-start")
            XCTAssertEqual(ea.solutionPath, eb.solutionPath, name)
        }
    }

    /// The sweep carries components per step and applies them; with the
    /// components riding along, a sweep over a component-carrying network is
    /// bit-identical to the same sweep over the plain one.
    func testSweepIsInertToComponents() throws {
        let net = try ReferenceCase.load("case14").network()
        let z = withComponents(net)
        func steps(_ n: BusBranchNetwork) -> [LoadStep] {
            [0.9, 1.0, 1.1].map { m in
                var loads: [Int: LoadStep.BusLoad] = [:]
                for (i, b) in n.buses.enumerated() where b.pLoadPu != 0 || b.qLoadPu != 0 {
                    loads[i] = .init(pPu: b.pLoadPu * m, qPu: b.qLoadPu * m,
                                     pZPu: b.pLoadZPu * m, pIPu: b.pLoadIPu * m,
                                     qZPu: b.qLoadZPu * m, qIPu: b.qLoadIPu * m)
                }
                return LoadStep(busLoads: loads)
            }
        }
        let a = TimeSeriesSweep().run(base: net, steps: steps(net), options: PowerFlowOptions(enforceQLimits: true))
        let b = TimeSeriesSweep().run(base: z, steps: steps(z), options: PowerFlowOptions(enforceQLimits: true))
        XCTAssertEqual(a.count, b.count)
        for (x, y) in zip(a, b) {
            XCTAssertEqual(x.converged, y.converged)
            XCTAssertEqual(x.iterations, y.iterations)
            XCTAssertEqual(x.vmPu.map(\.bitPattern), y.vmPu.map(\.bitPattern))
            XCTAssertEqual(x.vaRad.map(\.bitPattern), y.vaRad.map(\.bitPattern))
        }
    }

    private func assertBitIdentical(_ a: PowerFlowSolution, _ b: PowerFlowSolution, _ tag: String) {
        XCTAssertEqual(a.converged, b.converged, tag)
        XCTAssertEqual(a.iterations, b.iterations, tag)
        XCTAssertEqual(a.vmPu.map(\.bitPattern), b.vmPu.map(\.bitPattern), "\(tag) vm")
        XCTAssertEqual(a.vaRad.map(\.bitPattern), b.vaRad.map(\.bitPattern), "\(tag) va")
        XCTAssertEqual(a.genPPu.map(\.bitPattern), b.genPPu.map(\.bitPattern), "\(tag) genP")
        XCTAssertEqual(a.genQPu.map(\.bitPattern), b.genQPu.map(\.bitPattern), "\(tag) genQ")
        XCTAssertEqual(a.pinnedGenIndices, b.pinnedGenIndices, tag)
        XCTAssertEqual(a.branchFlows.map { $0.pFromPu.bitPattern }, b.branchFlows.map { $0.pFromPu.bitPattern }, "\(tag) flows")
        XCTAssertEqual(a.loadPPu.map(\.bitPattern), b.loadPPu.map(\.bitPattern), "\(tag) loadP")
    }
}
