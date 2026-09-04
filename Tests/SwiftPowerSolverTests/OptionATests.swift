import XCTest
@testable import SwiftPowerSolver

/// D80 (seventeenth sitting) — Option A: the ZIP polynomial is evaluated on
/// |V|, the physical model (a constant-current load draws current in
/// proportion to |V|, a constant-impedance one to |V|²; neither knows the
/// sign of a polar magnitude), plus the guard's re-examination of V = 0.
///
/// What Option A does to the root structure, stated so it can be checked:
/// with every load term even in V, the mismatch at (−V, θ) equals the
/// mismatch at (V, θ + π) — the same phasor — so a negative-magnitude point
/// is a root of the |V| equations exactly when its canonical form is a root
/// of the physical equations. The sixteenth sitting's spurious root
/// (−0.3277 pu at 1,200 MW) is not one: its canonical form carries a
/// mismatch of order 1 pu (`testTheSixteenthSittingRootIsNotARootOfTheEvenEquations`).
/// A negative magnitude that IS a root can still be reached in principle —
/// it is a legitimate solution in a non-canonical representation — which is
/// why the guard stays.
///
/// Written before the edit; the even-polynomial and zero-volt tests were
/// measured to fail against the guard-only build (`12e7add`).
final class OptionATests: XCTestCase {

    private func mix(_ mw: Double) -> BusBranchNetwork {
        OptionABlastRadiusProbe.twoBus(loadMw: mw, zP: 0.3, iP: 0.3, zQ: 0.5, iQ: 0.3)
    }
    private func pureI(_ mw: Double) -> BusBranchNetwork {
        OptionABlastRadiusProbe.twoBus(loadMw: mw, zP: 0, iP: 1, zQ: 0, iQ: 1)
    }

    // MARK: - The polynomial

    /// P_L and Q_L are even in V; their derivatives are odd.
    func testThePolynomialIsEvenInV() {
        let zip = ZIPLoad(mix(1200))
        XCTAssertTrue(zip.on)
        for v in [0.25, 0.327679, 0.8, 1.0, 1.7] {
            XCTAssertEqual(zip.p(1, -v).bitPattern, zip.p(1, v).bitPattern, "P_L(−\(v)) ≠ P_L(\(v))")
            XCTAssertEqual(zip.q(1, -v).bitPattern, zip.q(1, v).bitPattern, "Q_L(−\(v)) ≠ Q_L(\(v))")
            XCTAssertEqual(zip.dPdV(1, -v).bitPattern, (-zip.dPdV(1, v)).bitPattern, "∂P_L/∂V is odd")
            XCTAssertEqual(zip.dQdV(1, -v).bitPattern, (-zip.dQdV(1, v)).bitPattern, "∂Q_L/∂V is odd")
        }
        // Anti-vacuity: the I term is non-zero, so the signed polynomial
        // would NOT have been even here.
        XCTAssertNotEqual(zip.pI[1], 0)
    }

    /// Control (passes on both sides of the edit, and is labelled so): for a
    /// positive magnitude the evaluation is the pre-edit expression, bit for
    /// bit — |v| is v and the sign factor is exactly 1.
    func testPositiveMagnitudesEvaluateBitIdentically() {
        let zip = ZIPLoad(mix(1200))
        for v in [0.0, 1e-300, 0.25, 0.886297, 1.0, 1.05, 3.0] {
            let p = zip.pP[1] + zip.pI[1] * v + zip.pZ[1] * v * v
            let q = zip.qP[1] + zip.qI[1] * v + zip.qZ[1] * v * v
            XCTAssertEqual(zip.p(1, v).bitPattern, p.bitPattern, "v = \(v)")
            XCTAssertEqual(zip.q(1, v).bitPattern, q.bitPattern, "v = \(v)")
            XCTAssertEqual(zip.dPdV(1, v).bitPattern, (zip.pI[1] + 2 * zip.pZ[1] * v).bitPattern, "v = \(v)")
            XCTAssertEqual(zip.dQdV(1, v).bitPattern, (zip.qI[1] + 2 * zip.qZ[1] * v).bitPattern, "v = \(v)")
        }
    }

    // MARK: - The root structure

    /// The sixteenth sitting's negative "solution" is not a root of the even
    /// equations. Its canonical form (|V|, θ + π) is fed to Newton with a
    /// zero-iteration budget, which reports the mismatch at the seed; a root
    /// would read below tolerance. Measurement, printed; the assertion is
    /// only that it is not small.
    func testTheSixteenthSittingRootIsNotARootOfTheEvenEquations() throws {
        let net = mix(1200)
        // The refused iterate at the guard commit `12e7add`, recovered from
        // the solver there and recorded: V = −0.32767874071912473 pu at
        // θ = 8.475837 rad (Newton's unwrapped angle). Its canonical form is
        // the seed. (After Option A the reproducer diverges instead — the
        // point cannot be re-derived from the solver, which is the finding.)
        let vmSeed = [1.0, 0.32767874071912473]
        let vaSeed = [0.0, 11.617430682634485]
        let after = NewtonRaphsonSolver().solve(net, options: PowerFlowOptions())
        XCTAssertFalse(after.converged, "premise: the 1,200 MW mix no longer converges at all — \(after.vmPu)")
        XCTAssertTrue((after.failureReason ?? "").contains("max iterations"), after.failureReason ?? "nil")
        var o = PowerFlowOptions(maxIterations: 0)
        o.initialVmPu = vmSeed; o.initialVaRad = vaSeed
        let probe = NewtonRaphsonSolver().solve(net, options: o)
        XCTAssertFalse(probe.converged, "the canonical form of the spurious root must not be a root: \(probe.failureReason ?? "")")
        let reason = probe.failureReason ?? ""
        // "… reached, mismatch 2.359e+00 pu" — the number is the second-to-last token.
        let mismatch = Double(reason.split(separator: " ").dropLast().last.map(String.init) ?? "") ?? .nan
        print("canonical form of the sixteenth-sitting root: seed V = \(vmSeed[1]), θ = \(vaSeed[1]) → \(reason)")
        XCTAssertGreaterThan(mismatch, 1e-3, "mismatch at the canonical seed: \(reason)")
    }

    // MARK: - The "V = 0" collapse, re-examined

    /// The sixteenth sitting recorded the pure-I collapse as "V = 0.0000
    /// exactly". That was DISPLAY PRECISION (%.4f). Measured while writing a
    /// fixture to refuse V = 0: the iterate is 6.28e-12 pu (NR, both Q
    /// settings) and 6.35e-13 (engine warm start) — positive, never zero. The
    /// point is a root of the power-balance equations in the limit V → 0
    /// (P_L → 0 for a pure-I load, and the network injects nothing into a
    /// zero-volt bus), so the ABSOLUTE power tolerance is met by a collapsing
    /// voltage: a tolerance artifact, not a solution, and not a sign or a
    /// value a `≤ 0` guard could ever see. A guard on exact zero would be
    /// inert; a floor would be a threshold, which this cycle does not tune.
    /// Recorded as a measurement, with the mechanism made visible: tighten
    /// the tolerance and the "solution" moves further toward zero.
    ///
    /// EIGHTEENTH SITTING: the collapse is now REFUSED by the KCL
    /// post-condition (`KCLPostConditionTests`) — "converged in power, not
    /// in current" — with the iterate kept, so the measurement below reads
    /// the same magnitudes off the refused result. The finding stands; what
    /// changed is that the solver no longer calls it a solution.
    func testThePureICollapseIsAToleranceArtifactNotAZero() {
        var vAtTolerance: [Double: Double] = [:]
        for tol in [1e-8, 1e-10, 1e-12] {
            let sol = NewtonRaphsonSolver().solve(pureI(1500), options: PowerFlowOptions(tolerancePu: tol, maxIterations: 60))
            XCTAssertFalse(sol.converged, "tol \(tol): the collapse is refused")
            XCTAssertTrue((sol.failureReason ?? "").contains("not in current"), sol.failureReason ?? "nil")
            let v = sol.vmPu[1]
            XCTAssertGreaterThan(v, 0, "tol \(tol): the collapsed magnitude is positive, never zero")
            XCTAssertLessThan(v, 1e-9, "tol \(tol): but it is a collapsed bus — \(v)")
            vAtTolerance[tol] = v
            print("pure-I 1,500 MW at tolerance \(tol): power-converged \(sol.iterations) it, V = \(v) pu, refused")
        }
        XCTAssertLessThan(vAtTolerance[1e-12]!, vAtTolerance[1e-8]!, "tightening the tolerance drives the root toward zero — a tolerance artifact")
    }

    /// Control: a pure-I load below the collapse converges positive and is
    /// untouched (1,300 MW: V = 0.1290, measured at 5639906).
    func testAPositivePureIRootIsUntouched() {
        let sol = NewtonRaphsonSolver().solve(pureI(1300), options: PowerFlowOptions())
        XCTAssertTrue(sol.converged, sol.failureReason ?? "")
        XCTAssertEqual(sol.iterations, 8)
        XCTAssertEqual(sol.vmPu[1], 0.129029, accuracy: 1e-6)
    }
}
