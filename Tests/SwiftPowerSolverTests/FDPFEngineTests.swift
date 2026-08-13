import XCTest
@testable import SwiftPowerSolver

// PowerFlowEngine dispatch, warm start, and the Gate 2 fallback behavior.

final class FDPFEngineTests: XCTestCase {

    // MARK: - Fixtures

    /// A feasible network that Newton-Raphson cannot solve from a flat start:
    /// a load fed through a 60° phase-shifting transformer. The solution
    /// (angles near −60°) is ordinary, but at flat start the P–θ Jacobian
    /// block through the shifter scales with cos(60°+δ) and the first Newton
    /// steps overshoot into divergence. FDPF's constant B′ (shift folded into
    /// the off-diagonals per makeB) walks to the right basin instead. This is
    /// the large-shift / malformed-network regime the fallback exists for —
    /// verified against pandapower: NR flat-start FAILs, fdbx/fdxb converge.
    private func phaseShiftNetwork() -> BusBranchNetwork {
        BusBranchNetwork(
            baseMVA: 100,
            buses: [
                .init(type: .slack, baseKv: 138),
                .init(type: .pq, baseKv: 138),
                .init(type: .pq, baseKv: 138, pLoadPu: 0.2, qLoadPu: 0.04),
            ],
            branches: [
                .init(from: 0, to: 1, r: 0.01, x: 0.05),
                .init(from: 1, to: 2, r: 0, x: 0.2, shiftRad: 60 * .pi / 180),
            ],
            generators: [
                .init(bus: 0, pPu: 0, vSetPu: 1.02),
            ])
    }

    /// Deep-infeasible: nothing can converge (case14 at 5× load is far past
    /// the nose point).
    private func infeasibleNetwork() throws -> BusBranchNetwork {
        var net = try ReferenceCase.load("case14").network()
        for i in net.buses.indices {
            net.buses[i].pLoadPu *= 5
            net.buses[i].qLoadPu *= 5
        }
        return net
    }

    // MARK: - Gate 0 support: the default path is the NR path

    func testEngineDefaultOptionsMatchDirectNewtonRaphson() throws {
        let net = try ReferenceCase.load("case118").network()
        let options = PowerFlowOptions(enforceQLimits: true)
        let direct = NewtonRaphsonSolver().solve(net, options: options)
        let engine = PowerFlowEngine().solve(net, options: options)
        // Same code path, so bit-identical, not merely close.
        XCTAssertEqual(direct.vmPu, engine.vmPu)
        XCTAssertEqual(direct.vaRad, engine.vaRad)
        XCTAssertEqual(direct.iterations, engine.iterations)
        XCTAssertEqual(direct.pinnedGenIndices, engine.pinnedGenIndices)
        XCTAssertEqual(engine.solutionPath, .nr)
        XCTAssertEqual(engine.stages.map(\.kind), [.newtonRaphson])
    }

    // MARK: - Warm-start method

    func testWarmStartMethodConvergesAndReportsPath() throws {
        let net = try ReferenceCase.load("case118").network()
        var options = PowerFlowOptions()
        options.method = .fastDecoupledWarmStart
        let sol = PowerFlowEngine().solve(net, options: options)
        XCTAssertTrue(sol.converged)
        XCTAssertEqual(sol.solutionPath, .fdpfWarmStartNR)
        XCTAssertEqual(sol.stages.map(\.kind), [.fdpfWarmStart, .newtonRaphson])
        XCTAssertTrue(sol.stages[1].converged)

        let cold = NewtonRaphsonSolver().solve(net, options: PowerFlowOptions())
        var dV = 0.0
        for i in 0..<net.busCount where cold.vmPu[i].isFinite {
            dV = max(dV, abs(sol.vmPu[i] - cold.vmPu[i]))
        }
        XCTAssertLessThan(dV, 1e-6)
        XCTAssertLessThanOrEqual(sol.stages[1].iterations, cold.iterations,
                                 "warm-started NR should not need more iterations than cold")
        print("warm start case118: \(sol.stages[0].iterations) FDPF rounds + "
              + "\(sol.stages[1].iterations) NR iterations (cold NR: \(cold.iterations))")
    }

    /// Q-limit enforcement with method .fastDecoupled is routed through the
    /// warm-start path (v1 has no FDPF PV→PQ switching) — and the result must
    /// be the NR-with-limits answer, pinning included.
    func testFastDecoupledWithQLimitsRoutesThroughNR() throws {
        let ref = try ReferenceCase.load("case14")
        let net = ref.network()
        var options = PowerFlowOptions(enforceQLimits: true)
        options.method = .fastDecoupled
        let sol = PowerFlowEngine().solve(net, options: options)
        XCTAssertTrue(sol.converged)
        XCTAssertEqual(sol.solutionPath, .fdpfWarmStartNR)

        let nr = NewtonRaphsonSolver().solve(net, options: PowerFlowOptions(enforceQLimits: true))
        XCTAssertEqual(sol.pinnedGenIndices, nr.pinnedGenIndices)
        let vm = ErrorStats(reference: nr.vmPu, computed: sol.vmPu)
        XCTAssertLessThan(vm.maxError, 1e-6)
    }

    /// Standalone FastDecoupledSolver refuses the options it cannot honor,
    /// pointing at the engine instead of silently ignoring them.
    func testStandaloneFDPFRejectsUnsupportedOptions() throws {
        let net = try ReferenceCase.load("case14").network()
        let sol = FastDecoupledSolver().solve(net, options: PowerFlowOptions(enforceQLimits: true))
        XCTAssertFalse(sol.converged)
        XCTAssertTrue(sol.failureReason?.contains("PowerFlowEngine") ?? false)

        var dslack = net
        dslack.generators[0].slackWeight = 1
        let sol2 = FastDecoupledSolver().solve(dslack, options: PowerFlowOptions())
        XCTAssertFalse(sol2.converged)
        XCTAssertTrue(sol2.failureReason?.contains("distributed slack") ?? false)
    }

    // MARK: - Gate 2: fallback recovery

    func testNewtonRaphsonAloneFailsOnPhaseShiftNetwork() {
        let sol = NewtonRaphsonSolver().solve(phaseShiftNetwork(), options: PowerFlowOptions())
        XCTAssertFalse(sol.converged, "if NR now solves this from flat, the Gate 2 "
                       + "fixture needs a harder network")
    }

    func testAutoFallbackRecoversPhaseShiftNetwork() {
        let net = phaseShiftNetwork()
        var options = PowerFlowOptions()
        options.autoFallback = true
        let sol = PowerFlowEngine().solve(net, options: options)
        XCTAssertTrue(sol.converged, "fallback chain did not recover: \(sol.failureReason ?? "-")")
        XCTAssertTrue(sol.solutionPath == .fdpfWarmStartNRFallback
                      || sol.solutionPath == .fdpfFallback,
                      "unexpected path \(sol.solutionPath)")
        XCTAssertEqual(sol.stages.first?.kind, .newtonRaphson)
        XCTAssertFalse(sol.stages[0].converged, "stage 0 must record the NR divergence")

        // The recovered answer must be the FDPF answer (the physical solution
        // near −60°), self-consistent to the reference solve.
        let fd = FastDecoupledSolver().solve(net, options: PowerFlowOptions())
        XCTAssertTrue(fd.converged)
        var dV = 0.0, dA = 0.0
        for i in 0..<net.busCount {
            dV = max(dV, abs(sol.vmPu[i] - fd.vmPu[i]))
            dA = max(dA, abs(sol.vaRad[i] - fd.vaRad[i]))
        }
        XCTAssertLessThan(dV, 1e-6)
        XCTAssertLessThan(dA, 1e-6)
        XCTAssertLessThan(sol.vaRad[2], -0.9, "load-side angle should sit near −60°")
        print("Gate 2 recovery: path=\(sol.solutionPath.rawValue), stages="
              + sol.stages.map { "\($0.kind.rawValue)(\($0.iterations))" }.joined(separator: " → "))
    }

    func testNoFallbackWithoutOptIn() {
        let sol = PowerFlowEngine().solve(phaseShiftNetwork(), options: PowerFlowOptions())
        XCTAssertFalse(sol.converged)
        XCTAssertEqual(sol.solutionPath, .failed)
        XCTAssertEqual(sol.stages.map(\.kind), [.newtonRaphson])
    }

    // MARK: - Gate 2: graceful failure when nothing converges

    func testGracefulFailureWhenNothingConverges() throws {
        let net = try infeasibleNetwork()
        var options = PowerFlowOptions()
        options.autoFallback = true
        let sol = PowerFlowEngine().solve(net, options: options)

        XCTAssertFalse(sol.converged)
        XCTAssertEqual(sol.solutionPath, .failed)
        let reason = try XCTUnwrap(sol.failureReason)
        XCTAssertTrue(reason.contains("Newton-Raphson failed"), reason)
        XCTAssertTrue(reason.contains("FDPF warm start"), reason)
        XCTAssertTrue(reason.contains("full FDPF"), reason)
        // No partial state: NaN voltages, zero flows, zero dispatch — the
        // same failed-solve shape every caller already handles.
        XCTAssertTrue(sol.vmPu.allSatisfy(\.isNaN))
        XCTAssertTrue(sol.branchFlows.allSatisfy { $0 == .zero })
        XCTAssertTrue(sol.genPPu.allSatisfy { $0 == 0 })
        // Every stage is on the record.
        XCTAssertGreaterThanOrEqual(sol.stages.count, 3)
        print("Gate 2 graceful failure: " + sol.stages
            .map { "\($0.kind.rawValue)(\($0.iterations), conv=\($0.converged))" }
            .joined(separator: " → "))
    }

    /// With Q-limits requested, the full-FDPF stage must be SKIPPED (it cannot
    /// honor them) and say so — a fallback may never silently drop a feature.
    func testFallbackNeverDropsQLimitEnforcement() throws {
        var net = try infeasibleNetwork()
        net.generators = net.generators.map { g in
            var g = g
            g.qMinPu = -0.5
            g.qMaxPu = 0.5
            return g
        }
        var options = PowerFlowOptions(enforceQLimits: true)
        options.autoFallback = true
        let sol = PowerFlowEngine().solve(net, options: options)
        XCTAssertFalse(sol.converged)
        XCTAssertEqual(sol.solutionPath, .failed)
        XCTAssertTrue(sol.failureReason?.contains("full-FDPF fallback skipped") ?? false,
                      sol.failureReason ?? "-")
        XCTAssertFalse(sol.stages.map(\.kind).suffix(1).contains(.fdpf),
                       "no full-FDPF stage may run when Q-limits are requested")
    }
}
