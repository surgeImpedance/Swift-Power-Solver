import XCTest
@testable import SwiftPowerSolver

/// D80 (seventeenth sitting) — the package's own post-condition guard on a
/// negative voltage magnitude.
///
/// The finding (sixteenth sitting, reproduced here in package units): polar
/// Newton updates |V| additively and nothing stops an iterate crossing zero;
/// below zero the ZIP polynomial is evaluated on the SIGNED value, so a
/// constant-current term flips sign and the equations acquire roots the
/// network does not have. The solver then reports `converged = true` at a
/// negative magnitude. Measured at `5639906` on the two-bus reproducer with
/// the §4.1 mix: NR "converges" at 1,200 MW in 16 iterations at −0.3277 pu;
/// FDPF-BX at 1,100 MW in 138 rounds at −0.2969; FDPF-XB at 1,200 MW in 113
/// rounds at −0.3277; the engine's warm start at 900 MW at −0.2398 — every
/// path, not only Newton. Written before the guard and measured to fail
/// against the unchanged solvers.
final class NegativeMagnitudeGuardTests: XCTestCase {

    private func mix(_ mw: Double) -> BusBranchNetwork {
        OptionABlastRadiusProbe.twoBus(loadMw: mw, zP: 0.3, iP: 0.3, zQ: 0.5, iQ: 0.3)
    }

    func testNewtonRaphsonRefusesANegativeMagnitude() {
        for q in [false, true] {
            let sol = NewtonRaphsonSolver().solve(mix(1200), options: PowerFlowOptions(enforceQLimits: q))
            XCTAssertFalse(sol.converged, "q-limits \(q)")
            XCTAssertTrue((sol.failureReason ?? "").contains("negative voltage magnitude"), sol.failureReason ?? "nil")
            XCTAssertTrue((sol.failureReason ?? "").contains("bus 1"), sol.failureReason ?? "nil")
            XCTAssertTrue(sol.vmPu.contains { $0 < 0 }, "the iterate is kept for diagnosis, not blanked")
        }
    }

    func testFastDecoupledRefusesANegativeMagnitude() {
        for (mw, variant) in [(1100.0, FDPFVariant.bx), (1200.0, .xb)] {
            var o = PowerFlowOptions(maxIterations: 200)
            o.fdpfVariant = variant
            let sol = FastDecoupledSolver().solve(mix(mw), options: o)
            XCTAssertFalse(sol.converged, "\(variant.rawValue) at \(Int(mw)) MW")
            XCTAssertTrue((sol.failureReason ?? "").contains("negative voltage magnitude"), sol.failureReason ?? "nil")
            XCTAssertEqual(sol.solutionPath, .failed)
            XCTAssertTrue(sol.vmPu.contains { $0 < 0 })
        }
    }

    func testTheEngineNeverReportsANegativeMagnitudeAsConverged() {
        var warm = PowerFlowOptions(); warm.method = .fastDecoupledWarmStart; warm.autoFallback = true
        var auto = PowerFlowOptions(); auto.autoFallback = true
        for (mw, o) in [(900.0, warm), (1100.0, auto)] {
            let sol = PowerFlowEngine().solve(mix(mw), options: o)
            XCTAssertFalse(sol.converged, "\(Int(mw)) MW")
            XCTAssertTrue((sol.failureReason ?? "").contains("negative voltage magnitude"), sol.failureReason ?? "nil")
            XCTAssertEqual(sol.solutionPath, .failed)
        }
    }

    /// Control: only a constant-CURRENT term produces the artifact. Pure
    /// impedance converges positive at the same load; constant power diverges
    /// honestly.
    func testTheITermIsTheMechanism() {
        let z = NewtonRaphsonSolver().solve(
            OptionABlastRadiusProbe.twoBus(loadMw: 1200, zP: 1, iP: 0, zQ: 1, iQ: 0), options: PowerFlowOptions())
        XCTAssertTrue(z.converged, z.failureReason ?? "")
        XCTAssertTrue(z.vmPu.allSatisfy { $0 > 0 })
        let p = NewtonRaphsonSolver().solve(
            OptionABlastRadiusProbe.twoBus(loadMw: 1200, zP: 0, iP: 0, zQ: 0, iQ: 0), options: PowerFlowOptions())
        XCTAssertFalse(p.converged)
        XCTAssertTrue((p.failureReason ?? "").contains("max iterations"), p.failureReason ?? "nil")
    }

    /// Control: a converging case is untouched by a post-condition.
    func testAConvergingCaseIsUntouched() {
        let sol = NewtonRaphsonSolver().solve(mix(300), options: PowerFlowOptions())
        XCTAssertTrue(sol.converged); XCTAssertNil(sol.failureReason)
        XCTAssertEqual(sol.iterations, 4)
        XCTAssertEqual(sol.vmPu[1], 0.8863, accuracy: 5e-4)
        let f = FastDecoupledSolver().solve(mix(300), options: PowerFlowOptions(maxIterations: 200))
        XCTAssertTrue(f.converged); XCTAssertNil(f.failureReason)
        XCTAssertEqual(f.vmPu[1], sol.vmPu[1], accuracy: 1e-6)
    }
}
