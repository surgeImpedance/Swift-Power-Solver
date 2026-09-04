import XCTest
@testable import SwiftPowerSolver

/// D80 (eighteenth sitting, item a) — the KCL post-condition: a converged
/// POWER mismatch whose CURRENT mismatch |ΔS|/V is not also under the same
/// tolerance is refused.
///
/// Why the power criterion passes on a collapse: every injection at a bus
/// scales with V, and so does every load term without a constant-power part,
/// so at V → 0 the power balance holds for any current imbalance. Measured
/// at `d93b9bc`: a pure-I load at 1,500 MW "converged" at V = 6.28e-12 pu
/// with |ΔS|/V = 1.01 pu; a pure-Z load at 1,500 MW through the engine's
/// warm start "converged" at ~1e-13 pu — a collapse is not a pure-I
/// property, it belongs to every bus without a constant-power load.
///
/// Written before the post-condition and measured to fail against `05a40c6`.
final class KCLPostConditionTests: XCTestCase {

    private func pureI(_ mw: Double) -> BusBranchNetwork { OptionABlastRadiusProbe.twoBus(loadMw: mw, zP: 0, iP: 1, zQ: 0, iQ: 1) }
    private func pureZ(_ mw: Double) -> BusBranchNetwork { OptionABlastRadiusProbe.twoBus(loadMw: mw, zP: 1, iP: 0, zQ: 1, iQ: 0) }
    private func mix(_ mw: Double) -> BusBranchNetwork { OptionABlastRadiusProbe.twoBus(loadMw: mw, zP: 0.3, iP: 0.3, zQ: 0.5, iQ: 0.3) }

    // MARK: - The collapse is refused

    func testThePureICollapseIsRefusedInCurrent() {
        for (tol, q) in [(1e-8, false), (1e-8, true), (1e-12, false)] {
            let sol = NewtonRaphsonSolver().solve(pureI(1500), options: PowerFlowOptions(tolerancePu: tol, maxIterations: 60, enforceQLimits: q))
            XCTAssertFalse(sol.converged, "tol \(tol) q \(q): V = \(sol.vmPu)")
            let reason = sol.failureReason ?? ""
            XCTAssertTrue(reason.contains("converged in power, not in current"), reason)
            XCTAssertTrue(reason.contains("bus 1"), reason)
            // The iterate is kept: the collapsed magnitude is visible to the caller.
            XCTAssertGreaterThan(sol.vmPu[1], 0); XCTAssertLessThan(sol.vmPu[1], 1e-9, "\(sol.vmPu)")
        }
    }

    /// The pure-Z collapse reached through the engine's warm start: the
    /// warm NR stage is refused, the engine's cold retry finds the physical
    /// root (0.566130 pu, the direct-NR answer), and the route says so.
    func testThePureZCollapseThroughTheWarmStartIsRefusedAndTheEngineRecovers() {
        var warm = PowerFlowOptions(); warm.method = .fastDecoupledWarmStart; warm.autoFallback = true
        let sol = PowerFlowEngine().solve(pureZ(1500), options: warm)
        XCTAssertTrue(sol.converged, sol.failureReason ?? "")
        XCTAssertEqual(sol.vmPu[1], 0.566130, accuracy: 1e-6, "the physical root, not the collapse")
        XCTAssertEqual(sol.solutionPath, .nr, "the cold retry produced the answer")
        let direct = NewtonRaphsonSolver().solve(pureZ(1500), options: PowerFlowOptions())
        XCTAssertEqual(sol.vmPu[1], direct.vmPu[1], accuracy: 1e-9)
        XCTAssertFalse(sol.stages.first { $0.kind == .newtonRaphson }?.converged ?? true,
                       "premise: the warm NR stage was the refused one — \(sol.stages)")
    }

    /// The one converging arm the post-condition re-routes (measured on the
    /// ladder, attributed): pure Z at 1,300 MW through the warm start met
    /// |ΔS| < 1e-8 at V = 0.608 with |ΔS|/V = 1.39e-8 — inside the power
    /// tolerance, outside the current tolerance. Refused, retried cold: the
    /// same point (to 1e-9), one more iteration, route "nr".
    func testAWarmStartIterateOutsideTheCurrentToleranceIsRetriedCold() {
        var warm = PowerFlowOptions(); warm.method = .fastDecoupledWarmStart; warm.autoFallback = true
        let sol = PowerFlowEngine().solve(pureZ(1300), options: warm)
        XCTAssertTrue(sol.converged, sol.failureReason ?? "")
        XCTAssertEqual(sol.solutionPath, .nr)
        XCTAssertEqual(sol.iterations, 5)
        let direct = NewtonRaphsonSolver().solve(pureZ(1300), options: PowerFlowOptions())
        XCTAssertEqual(sol.vmPu[1], direct.vmPu[1], accuracy: 1e-9)
        XCTAssertEqual(sol.vmPu[1], 0.608297, accuracy: 1e-6)
    }

    // MARK: - Controls

    /// Positive solutions are untouched: the same trajectory, the same
    /// values, no reason (the control the blast-radius probe measures at
    /// scale — 0 of 60 reference arms moved).
    func testPositiveSolutionsAreUntouched() throws {
        let sol = NewtonRaphsonSolver().solve(mix(300), options: PowerFlowOptions())
        XCTAssertTrue(sol.converged); XCTAssertNil(sol.failureReason)
        XCTAssertEqual(sol.iterations, 4); XCTAssertEqual(sol.vmPu[1], 0.886297, accuracy: 5e-7)
        let net = try ReferenceCase.load("case14").network()
        let c = NewtonRaphsonSolver().solve(net, options: PowerFlowOptions())
        XCTAssertTrue(c.converged); XCTAssertNil(c.failureReason); XCTAssertEqual(c.iterations, 4)
    }

    /// FDPF's own criterion is current-shaped (ΔP/V, ΔQ/V) and never
    /// reported a collapse: the pure-I ladder past the nose fails there as
    /// divergence, before and after this item.
    func testFDPFNeverReportedTheCollapse() {
        for mw in [1500.0, 2000, 4000] {
            let sol = FastDecoupledSolver().solve(pureI(mw), options: PowerFlowOptions(maxIterations: 200))
            XCTAssertFalse(sol.converged, "\(Int(mw)) MW: \(sol.vmPu)")
            XCTAssertTrue((sol.failureReason ?? "").contains("max iterations"), sol.failureReason ?? "nil")
        }
    }
}
