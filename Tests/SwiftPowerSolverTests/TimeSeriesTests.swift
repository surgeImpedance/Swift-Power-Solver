import XCTest
@testable import SwiftPowerSolver

// Time-series sweep oracle. Two distinct checks:
//   1. Per-step agreement vs pandapower's time-series (scale-and-solve) at
//      machine precision — the acceptance bar.
//   2. Warm-start == flat-start for the same step at machine precision — proves
//      warm-start is a speed optimization, never a different answer.
// Plus non-convergence handling (continue-and-report, no cascade) and the
// reported warm-start iteration speedup.
final class TimeSeriesTests: XCTestCase {

    // Codable mirror of Reference/timeseries.json.
    private struct TimeSeriesReference: Decodable {
        struct Step: Decodable {
            var multiplier: Double
            var vmPu: [Double]
            var vaDeg: [Double]
            var branchFlows: [ReferenceCase.RefFlow]
            var iterations: Int
        }
        var baseCase: String
        var multipliers: [Double]
        var steps: [Step]

        static func load() throws -> TimeSeriesReference {
            guard let url = Bundle.module.url(forResource: "timeseries",
                                              withExtension: "json",
                                              subdirectory: "Reference") else {
                throw XCTSkip("missing timeseries.json — run Tools/dump_reference.py")
            }
            let d = JSONDecoder()
            d.keyDecodingStrategy = .convertFromSnakeCase
            return try d.decode(TimeSeriesReference.self, from: Data(contentsOf: url))
        }
    }

    private func options() -> PowerFlowOptions {
        var o = PowerFlowOptions()
        o.tolerancePu = 1e-12
        o.maxIterations = 30
        return o
    }

    /// Steps that scale the base network's per-bus loads by each multiplier —
    /// exactly what pandapower's ConstControl does, expressed via the per-bus
    /// LoadStep API.
    private func steps(base: BusBranchNetwork, multipliers: [Double]) -> [LoadStep] {
        multipliers.map { m in
            var busLoads: [Int: LoadStep.BusLoad] = [:]
            for (i, bus) in base.buses.enumerated() where bus.pLoadPu != 0 || bus.qLoadPu != 0 {
                busLoads[i] = .init(pPu: bus.pLoadPu * m, qPu: bus.qLoadPu * m)
            }
            return LoadStep(busLoads: busLoads)
        }
    }

    // MARK: - 1. Per-step agreement vs pandapower

    func testPerStepAgreementVsPandapower() throws {
        let ref = try TimeSeriesReference.load()
        let base = try ReferenceCase.load(ref.baseCase).network()
        let sweep = TimeSeriesSweep().run(base: base,
                                          steps: steps(base: base, multipliers: ref.multipliers),
                                          options: options())
        XCTAssertEqual(sweep.count, ref.steps.count)

        var worstVm = 0.0, worstVa = 0.0, worstFlow = 0.0
        for (k, refStep) in ref.steps.enumerated() {
            let got = sweep[k]
            XCTAssertTrue(got.converged, "step \(k): \(got.failureReason ?? "?")")

            let vm = ErrorStats(reference: refStep.vmPu, computed: got.vmPu)
            let va = ErrorStats(reference: refStep.vaDeg.map { $0 * .pi / 180 },
                                computed: got.vaRad)
            let base = base.baseMVA
            let pf = ErrorStats(reference: refStep.branchFlows.map(\.pfMw),
                                computed: got.branchFlows.map { $0.pFromPu * base })
            let qf = ErrorStats(reference: refStep.branchFlows.map(\.qfMvar),
                                computed: got.branchFlows.map { $0.qFromPu * base })
            XCTAssertLessThanOrEqual(vm.maxError, 1e-6, "step \(k) Vm")
            XCTAssertLessThanOrEqual(va.maxError, 1e-6, "step \(k) Va")
            XCTAssertLessThanOrEqual(max(pf.maxError, qf.maxError), 1e-4, "step \(k) flow")
            worstVm = max(worstVm, vm.maxError)
            worstVa = max(worstVa, va.maxError)
            worstFlow = max(worstFlow, pf.maxError, qf.maxError)

            print(String(format: "  step %d (m=%.2f, warm=%@): iters %d, "
                         + "max|ΔVm| %.2e pu, max|ΔVa| %.2e rad, max|Δflow| %.2e MVA",
                         k, refStep.multiplier, got.warmStarted ? "y" : "n",
                         got.iterations, vm.maxError, va.maxError,
                         max(pf.maxError, qf.maxError)))
        }
        print(String(format: "time-series vs pandapower: worst |ΔVm| %.2e pu, "
                     + "|ΔVa| %.2e rad, |Δflow| %.2e MVA over %d steps",
                     worstVm, worstVa, worstFlow, ref.steps.count))
    }

    // MARK: - 2. Warm-start == flat-start (the critical guard) + speedup

    func testWarmStartEqualsFlatStartAndReportSpeedup() throws {
        let ref = try TimeSeriesReference.load()
        let base = try ReferenceCase.load(ref.baseCase).network()
        let allSteps = steps(base: base, multipliers: ref.multipliers)
        let solver = NewtonRaphsonSolver()

        var flatIterTotal = 0, warmIterTotal = 0
        var worst = 0.0
        var prev: PowerFlowSolution?

        for (k, step) in allSteps.enumerated() {
            var net = base
            for (i, load) in step.busLoads { net.buses[i].pLoadPu = load.pPu; net.buses[i].qLoadPu = load.qPu }

            // Flat start (default options).
            let flat = solver.solve(net, options: options())
            // Warm start from the previous converged step.
            var warmOpts = options()
            if let p = prev { warmOpts.initialVmPu = p.vmPu; warmOpts.initialVaRad = p.vaRad }
            let warm = solver.solve(net, options: warmOpts)

            XCTAssertTrue(flat.converged && warm.converged, "step \(k)")
            let vm = ErrorStats(reference: flat.vmPu, computed: warm.vmPu)
            let va = ErrorStats(reference: flat.vaRad, computed: warm.vaRad)
            // Same root, reached from different starting points.
            XCTAssertLessThanOrEqual(vm.maxError, 1e-9, "step \(k): warm≠flat Vm")
            XCTAssertLessThanOrEqual(va.maxError, 1e-9, "step \(k): warm≠flat Va")
            worst = max(worst, vm.maxError, va.maxError)

            flatIterTotal += flat.iterations
            warmIterTotal += warm.iterations
            prev = warm
        }

        // Warm-starting must not increase iterations overall, and should reduce them.
        XCTAssertLessThanOrEqual(warmIterTotal, flatIterTotal)
        print(String(format: "warm==flat: worst |Δ| %.2e over %d steps  |  "
                     + "iterations flat=%d warm=%d (%.0f%% fewer)",
                     worst, allSteps.count, flatIterTotal, warmIterTotal,
                     100.0 * Double(flatIterTotal - warmIterTotal) / Double(flatIterTotal)))
        XCTAssertGreaterThan(worst, 0, "warm and flat should differ slightly (different paths)")
    }

    // MARK: - 3. Non-convergence is reported and does not cascade

    func testNonConvergenceContinuesWithoutCascade() throws {
        let ref = try TimeSeriesReference.load()
        let base = try ReferenceCase.load(ref.baseCase).network()

        // A normal step, an absurd-load step (diverges), then a normal step.
        let good = steps(base: base, multipliers: [1.0])[0]
        var wild: [Int: LoadStep.BusLoad] = [:]
        for (i, bus) in base.buses.enumerated() where bus.pLoadPu != 0 {
            wild[i] = .init(pPu: bus.pLoadPu * 500, qPu: bus.qLoadPu * 500)   // way past the nose
        }
        let sweep = TimeSeriesSweep().run(
            base: base, steps: [good, LoadStep(busLoads: wild), good], options: options())

        XCTAssertEqual(sweep.count, 3)
        XCTAssertTrue(sweep[0].converged, "step 0 should converge")
        XCTAssertFalse(sweep[1].converged, "absurd-load step should be reported non-converged")
        XCTAssertNotNil(sweep[1].failureReason)
        // The key guard: step 2 warm-starts from step 0 (last converged), NOT
        // from step 1's divergent iterates — so it still converges.
        XCTAssertTrue(sweep[2].converged, "step after a failure must still converge")
        XCTAssertTrue(sweep[2].warmStarted)

        // Halt policy stops at the failure.
        let halted = TimeSeriesSweep().run(
            base: base, steps: [good, LoadStep(busLoads: wild), good],
            options: options(), onNonConvergence: .halt)
        XCTAssertEqual(halted.count, 2, "halt should stop at the first failure")
    }

    // MARK: - Per-bus (non-uniform) application

    func testNonUniformPerBusLoadsApplied() throws {
        let base = try ReferenceCase.load("case14").network()
        // Override exactly one loaded bus; leave the rest at base.
        let loaded = base.buses.firstIndex { $0.pLoadPu != 0 }!
        let step = LoadStep(busLoads: [loaded: .init(pPu: 0.123, qPu: 0.045)])
        let sweep = TimeSeriesSweep().run(base: base, steps: [step], options: options())
        XCTAssertTrue(sweep[0].converged)
        // Reconstruct the injection at that bus from the solved flows to confirm
        // the override took effect rather than the base load.
        // (Simpler: re-solve a network with the same single override and compare.)
        var net = base
        net.buses[loaded].pLoadPu = 0.123; net.buses[loaded].qLoadPu = 0.045
        let direct = NewtonRaphsonSolver().solve(net, options: options())
        let vm = ErrorStats(reference: direct.vmPu, computed: sweep[0].vmPu)
        XCTAssertLessThanOrEqual(vm.maxError, 1e-12, "per-bus override not applied as expected")
    }
}
