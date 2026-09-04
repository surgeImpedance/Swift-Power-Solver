import XCTest
@testable import SwiftPowerSolver

/// D80 (seventeenth sitting) — the package's own post-condition guard on a
/// negative voltage magnitude, and what Option A did to its subject.
///
/// HISTORY, because the file's shape changed between the two commits of
/// this cycle and the record must show why. At `12e7add` (the guard, 1/2)
/// the three subject tests drove the two-bus reproducer with the §4.1 mix
/// through NR, FDPF and the engine and asserted the refusal; written before
/// the guard, they failed 18 ways against `5639906` (NR at 1,200 MW:
/// converged=true, 16 it, −0.3277 pu; FDPF-BX at 1,100 MW: 138 rounds,
/// −0.2969; FDPF-XB at 1,200: 113 rounds, −0.3277; engine warm start at
/// 900: −0.2398) and passed with it. Then Option A (2/2) evaluated the load
/// polynomial on |V| and the spurious roots ceased to exist: every one of
/// those arms now DIVERGES honestly (max iterations, mismatch of order 1–10
/// pu), so no solver-level reproducer of a negative "convergence" exists on
/// the ladder any more — measured over 396 probe arms, zero refusals. The
/// guard is belt-and-braces there. It stays, because a negative magnitude
/// that IS a root of the even equations — a legitimate solution in its
/// non-canonical representation, (−V, θ) for (V, θ+π) — is reachable in
/// principle whenever an iterate crosses zero, and the guard is what keeps
/// that representation from being reported as an operating point. So the
/// guard is now witnessed at the unit it is: the post-condition itself.
final class NegativeMagnitudeGuardTests: XCTestCase {

    private func mix(_ mw: Double) -> BusBranchNetwork {
        OptionABlastRadiusProbe.twoBus(loadMw: mw, zP: 0.3, iP: 0.3, zQ: 0.5, iQ: 0.3)
    }

    // MARK: - The post-condition

    func testTheGuardRefusesANegativeMagnitudeAndNamesTheBus() {
        let reason = PowerFlowSolution.nonPhysicalMagnitude([1.0, -0.327679, 0.95])
        XCTAssertNotNil(reason)
        XCTAssertTrue((reason ?? "").contains("negative voltage magnitude"), reason ?? "nil")
        XCTAssertTrue((reason ?? "").contains("at 1 bus(es) (bus 1, V = -0.3277 pu)"), reason ?? "nil")
        let two = PowerFlowSolution.nonPhysicalMagnitude([-1.0, 0.5, -0.25]) ?? ""
        XCTAssertTrue(two.contains("at 2 bus(es) (bus 0, V = -1.0000 pu)"), two)
    }

    func testTheGuardPassesPositiveAndDeEnergizedMagnitudes() {
        XCTAssertNil(PowerFlowSolution.nonPhysicalMagnitude([1.0, 0.886297, 1.05]))
        XCTAssertNil(PowerFlowSolution.nonPhysicalMagnitude([1.0, .nan, 0.5]), "NaN is a de-energized bus")
        XCTAssertNil(PowerFlowSolution.nonPhysicalMagnitude([1.0, 6.280523147999544e-12]),
                     "the pure-I collapse is positive — see OptionATests")
        XCTAssertNil(PowerFlowSolution.nonPhysicalMagnitude([]))
    }

    // MARK: - The reproducer, after Option A: honest divergence, no refusal

    /// The arms that reported a negative "convergence" at `5639906`, through
    /// every solver, now fail as divergence and never as a refusal. Pinned
    /// so a regression in EITHER direction is visible: a refusal reappearing
    /// means the polynomial lost its evenness; a convergence means a root
    /// appeared where the ladder measured none.
    func testTheFormerlyNegativeArmsNowDivergeHonestly() {
        var arms: [(String, PowerFlowSolution)] = []
        for q in [false, true] {
            arms.append(("nr 1200 q=\(q)", NewtonRaphsonSolver().solve(mix(1200), options: PowerFlowOptions(enforceQLimits: q))))
        }
        for (mw, variant) in [(1100.0, FDPFVariant.bx), (1200.0, .xb)] {
            var o = PowerFlowOptions(maxIterations: 200); o.fdpfVariant = variant
            arms.append(("fdpf-\(variant.rawValue) \(Int(mw))", FastDecoupledSolver().solve(mix(mw), options: o)))
        }
        var warm = PowerFlowOptions(); warm.method = .fastDecoupledWarmStart; warm.autoFallback = true
        var auto = PowerFlowOptions(); auto.autoFallback = true
        arms.append(("engine-warm 900", PowerFlowEngine().solve(mix(900), options: warm)))
        arms.append(("engine-nr-auto 1100", PowerFlowEngine().solve(mix(1100), options: auto)))
        for (name, sol) in arms {
            XCTAssertFalse(sol.converged, name)
            let reason = sol.failureReason ?? ""
            XCTAssertTrue(reason.contains("max iterations"), "\(name): expected honest divergence — \(reason)")
            XCTAssertFalse(reason.contains("voltage magnitude"), "\(name): a refusal means the polynomial is no longer even — \(reason)")
            XCTAssertFalse(sol.vmPu.contains { $0.isFinite && $0 < 0 }, "\(name): \(sol.vmPu)")
        }
    }

    /// Control: only a constant-CURRENT term produced the artifact, and the
    /// two neighbours are unchanged by Option A — pure impedance converges
    /// positive at the same load, constant power diverges honestly.
    func testTheITermWasTheMechanism() {
        let z = NewtonRaphsonSolver().solve(
            OptionABlastRadiusProbe.twoBus(loadMw: 1200, zP: 1, iP: 0, zQ: 1, iQ: 0), options: PowerFlowOptions())
        XCTAssertTrue(z.converged, z.failureReason ?? "")
        XCTAssertTrue(z.vmPu.allSatisfy { $0 > 0 })
        XCTAssertEqual(z.iterations, 5)
        let p = NewtonRaphsonSolver().solve(
            OptionABlastRadiusProbe.twoBus(loadMw: 1200, zP: 0, iP: 0, zQ: 0, iQ: 0), options: PowerFlowOptions())
        XCTAssertFalse(p.converged)
        XCTAssertTrue((p.failureReason ?? "").contains("max iterations"), p.failureReason ?? "nil")
    }

    /// Control: a converging case is untouched by a post-condition and by
    /// Option A (its iterates never cross zero): 4 iterations, 0.886297 pu at
    /// `5639906`, `12e7add` and here.
    func testAConvergingCaseIsUntouched() {
        let sol = NewtonRaphsonSolver().solve(mix(300), options: PowerFlowOptions())
        XCTAssertTrue(sol.converged); XCTAssertNil(sol.failureReason)
        XCTAssertEqual(sol.iterations, 4)
        XCTAssertEqual(sol.vmPu[1], 0.886297, accuracy: 5e-7)
        let f = FastDecoupledSolver().solve(mix(300), options: PowerFlowOptions(maxIterations: 200))
        XCTAssertTrue(f.converged); XCTAssertNil(f.failureReason)
        XCTAssertEqual(f.iterations, 6)
        XCTAssertEqual(f.vmPu[1], sol.vmPu[1], accuracy: 1e-6)
    }
}
