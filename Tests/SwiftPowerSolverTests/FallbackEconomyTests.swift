import XCTest
@testable import SwiftPowerSolver

// Sweep fallback economy + the Q-limit re-pin counter — the outcome of the
// 2026-08-14 pin-seeding investigation (seeding itself was REJECTED: the
// outer loop never releases pins, measured consecutive pinned sets are nearly
// disjoint at load shoulders, and the case9241 sweep cost that motivated it
// is divergence-ladder burn on infeasible steps, not re-pinning).
final class FallbackEconomyTests: XCTestCase {

    /// case14 day with an infeasible plateau in the middle (×5 load is far
    /// past the nose point) and a recovery on both sides.
    private let multipliers: [Double] = [1.0, 1.05, 5.0, 5.0, 5.0, 1.05, 1.0]

    private func steps(_ net: BusBranchNetwork) -> [LoadStep] {
        multipliers.map { m in
            var loads: [Int: LoadStep.BusLoad] = [:]
            for (i, bus) in net.buses.enumerated()
                where bus.pLoadPu != 0 || bus.qLoadPu != 0 {
                loads[i] = .init(pPu: bus.pLoadPu * m, qPu: bus.qLoadPu * m)
            }
            return LoadStep(busLoads: loads)
        }
    }

    private func options() -> PowerFlowOptions {
        var o = PowerFlowOptions(enforceQLimits: true)
        o.autoFallback = true
        return o
    }

    // MARK: - The counter

    func testQLimitRestartCounter() throws {
        let net = try ReferenceCase.load("case118").network()
        let on = NewtonRaphsonSolver().solve(net, options: PowerFlowOptions(enforceQLimits: true))
        XCTAssertTrue(on.converged)
        XCTAssertGreaterThanOrEqual(on.qLimitRestarts, 1,
                                    "case118 with Q-limits pins generators; the counter must see it")
        XCTAssertFalse(on.pinnedGenIndices.isEmpty)

        let off = NewtonRaphsonSolver().solve(net, options: PowerFlowOptions(enforceQLimits: false))
        XCTAssertEqual(off.qLimitRestarts, 0)
        print("counter case118: restarts=\(on.qLimitRestarts) pins=\(on.pinnedGenIndices.count) "
              + "iters=\(on.iterations) (no-qlims iters=\(off.iterations))")
    }

    // MARK: - Economy behavior

    func testEconomySkipsLadderAfterFailedStepAndRecovers() throws {
        let net = try ReferenceCase.load("case14").network()
        let sw = TimeSeriesSweep().run(base: net, steps: steps(net), options: options())
        XCTAssertEqual(sw.count, 7)

        XCTAssertTrue(sw[0].converged)
        XCTAssertTrue(sw[1].converged)
        XCTAssertFalse(sw[1].recoverySkipped)

        // Step 2: first infeasible step — pays the FULL chain (its verdict is
        // the anchor every later skip leans on).
        XCTAssertFalse(sw[2].converged)
        XCTAssertFalse(sw[2].recoverySkipped)
        XCTAssertGreaterThan(sw[2].iterations, 60,
                             "the anchor step runs the whole recovery ladder")

        // Steps 3–4: economized — primary attempt only, reason says so.
        for i in 3...4 {
            XCTAssertFalse(sw[i].converged)
            XCTAssertTrue(sw[i].recoverySkipped, "step \(i)")
            XCTAssertLessThanOrEqual(sw[i].iterations, 31, "step \(i): ladder must be skipped")
            XCTAssertTrue(sw[i].failureReason?.contains("recovery chain skipped") ?? false)
        }

        // Step 5: regime recovers THROUGH the economized primary attempt.
        XCTAssertTrue(sw[5].converged)
        XCTAssertTrue(sw[5].recoverySkipped)
        XCTAssertEqual(sw[5].solutionPath, .nr)
        // Step 6: back to full behavior.
        XCTAssertTrue(sw[6].converged)
        XCTAssertFalse(sw[6].recoverySkipped)
    }

    /// Converged steps are bit-identical with economy on or off — the
    /// economized attempt IS the chain's own first attempt, from the same warm
    /// start. Failed steps differ only in how much ladder they burned.
    func testConvergedStepsBitIdenticalOnVsOff() throws {
        let net = try ReferenceCase.load("case14").network()
        let on = TimeSeriesSweep().run(base: net, steps: steps(net), options: options(),
                                       skipRecoveryAfterFailedStep: true)
        let off = TimeSeriesSweep().run(base: net, steps: steps(net), options: options(),
                                        skipRecoveryAfterFailedStep: false)
        XCTAssertEqual(on.map(\.converged), off.map(\.converged),
                       "economy must not change which steps converge on this day")
        for i in on.indices where on[i].converged {
            XCTAssertEqual(on[i].vmPu, off[i].vmPu, "step \(i): vm")
            XCTAssertEqual(on[i].vaRad, off[i].vaRad, "step \(i): va")
            XCTAssertEqual(on[i].solutionPath, off[i].solutionPath, "step \(i): path")
        }
        let itOn = on.map(\.iterations).reduce(0, +)
        let itOff = off.map(\.iterations).reduce(0, +)
        XCTAssertLessThan(itOn, itOff, "economy must cut total iterations on a failing day")
        print("economy iterations: on=\(itOn) off=\(itOff) "
              + "per-step on=\(on.map(\.iterations)) off=\(off.map(\.iterations))")
    }

    /// Gate D: repeated economized sweeps are bit-identical; the policy depends
    /// only on the previous step's converged flag, never on hidden state.
    func testEconomizedSweepIsDeterministic() throws {
        let net = try ReferenceCase.load("case14").network()
        let a = TimeSeriesSweep().run(base: net, steps: steps(net), options: options())
        let b = TimeSeriesSweep().run(base: net, steps: steps(net), options: options())
        // PREMISE, asserted (2026-09-02): the ×5 plateau must actually fail, or
        // the NaN-carrying path this test exists to pin is never exercised and
        // every assertion below passes vacuously. Its siblings in this file
        // assert the premise; this one did not — it was the one silently-green
        // member of the infeasibility-premised set (ZIP exploration §7.1).
        XCTAssertFalse(a[2].converged, "premise: the ×5 plateau step must fail")
        XCTAssertTrue(a[2].vmPu.contains { $0.isNaN },
                      "premise: a failed step carries NaN voltages")
        for (x, y) in zip(a, b) {
            XCTAssertEqual(x.converged, y.converged)
            // Bit patterns, not ==: failed steps carry NaN voltages, and
            // NaN != NaN would fail the comparison the test exists to make.
            XCTAssertEqual(x.vmPu.map(\.bitPattern), y.vmPu.map(\.bitPattern))
            XCTAssertEqual(x.vaRad.map(\.bitPattern), y.vaRad.map(\.bitPattern))
            XCTAssertEqual(x.iterations, y.iterations)
            XCTAssertEqual(x.recoverySkipped, y.recoverySkipped)
            XCTAssertEqual(x.qLimitRestarts, y.qLimitRestarts)
        }
    }

    /// A pure-NR sweep (no fallback requested) never economizes — there is no
    /// ladder to skip, and the pre-FDPF code path is untouched.
    func testPureNRSweepUnaffected() throws {
        let net = try ReferenceCase.load("case14").network()
        let sw = TimeSeriesSweep().run(base: net, steps: steps(net),
                                       options: PowerFlowOptions(enforceQLimits: true))
        XCTAssertTrue(sw.allSatisfy { !$0.recoverySkipped })
        XCTAssertEqual(sw.map(\.converged), [true, true, false, false, false, true, true])
    }
}
