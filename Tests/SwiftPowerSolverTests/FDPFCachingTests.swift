import XCTest
@testable import SwiftPowerSolver

// Gate 3, package half: factorization reuse across a topology-static sweep,
// cache invalidation on parameter change, and bit-identical repeatability.
// (The case9241 per-iteration cost comparison runs app-side, where that
// fixture lives.)

final class FDPFCachingTests: XCTestCase {

    private func multiplierSteps(_ net: BusBranchNetwork, _ multipliers: [Double]) -> [LoadStep] {
        multipliers.map { m in
            var loads: [Int: LoadStep.BusLoad] = [:]
            for (i, bus) in net.buses.enumerated()
                where bus.pLoadPu != 0 || bus.qLoadPu != 0 {
                loads[i] = .init(pPu: bus.pLoadPu * m, qPu: bus.qLoadPu * m)
            }
            return LoadStep(busLoads: loads)
        }
    }

    /// Factor count == 2 for a whole 24-step sweep when topology is static —
    /// the B′/B″ pair is built once and reused for every step and every
    /// half-iteration.
    func testSweepFactorizesExactlyOnce() throws {
        let net = try ReferenceCase.load("case118").network()
        let multipliers: [Double] = (0..<24).map { (h: Int) -> Double in
            let phase = Double(h) / 24.0 * 2.0 * Double.pi
            return 1.0 + 0.3 * sin(phase)
        }
        let steps = multiplierSteps(net, multipliers)
        var options = PowerFlowOptions()
        options.method = .fastDecoupled

        let cache = FDPFFactorizationCache()
        let results = TimeSeriesSweep().run(base: net, steps: steps, options: options,
                                            fdpfCache: cache)
        XCTAssertEqual(results.count, steps.count)
        XCTAssertTrue(results.allSatisfy { $0.converged })
        XCTAssertTrue(results.allSatisfy { $0.solutionPath == SolutionPath.fdpf })
        XCTAssertEqual(cache.factorizationCount, 2,
                       "topology never changed: B′ and B″ must be factorized exactly once each")
    }

    /// Any parameter that reaches the B matrices invalidates the cache; the
    /// next solve refactorizes (count 2 → 4).
    func testCacheInvalidatesOnParameterChange() throws {
        var net = try ReferenceCase.load("case14").network()
        var options = PowerFlowOptions()
        options.method = .fastDecoupled
        let cache = FDPFFactorizationCache()
        let engine = PowerFlowEngine()

        XCTAssertTrue(engine.solve(net, options: options, fdpfCache: cache).converged)
        XCTAssertEqual(cache.factorizationCount, 2)
        // Same network again: pure reuse.
        XCTAssertTrue(engine.solve(net, options: options, fdpfCache: cache).converged)
        XCTAssertEqual(cache.factorizationCount, 2)
        // Topology/parameter change: rebuild.
        net.branches[0].x *= 1.1
        XCTAssertTrue(engine.solve(net, options: options, fdpfCache: cache).converged)
        XCTAssertEqual(cache.factorizationCount, 4)
        // Load-only change: NOT an invalidation.
        net.buses[3].pLoadPu *= 1.5
        XCTAssertTrue(engine.solve(net, options: options, fdpfCache: cache).converged)
        XCTAssertEqual(cache.factorizationCount, 4)
    }

    /// Cached and uncached solves produce bit-identical results — the cache is
    /// a speed seam, never an answer seam.
    func testCachedSolveIsBitIdenticalToUncached() throws {
        let net = try FDPFReferenceCase.load("case300").network()
        var options = PowerFlowOptions()
        options.method = .fastDecoupled
        let cache = FDPFFactorizationCache()
        let engine = PowerFlowEngine()
        let warmup = engine.solve(net, options: options, fdpfCache: cache)
        XCTAssertTrue(warmup.converged)
        let cached = engine.solve(net, options: options, fdpfCache: cache)
        let uncached = engine.solve(net, options: options, fdpfCache: nil)
        XCTAssertEqual(cached.vmPu, uncached.vmPu)
        XCTAssertEqual(cached.vaRad, uncached.vaRad)
        XCTAssertEqual(cached.iterations, uncached.iterations)
    }

    /// Gate 3 determinism: repeated runs are bit-identical, solve and sweep.
    func testRepeatedRunsAreBitIdentical() throws {
        let net = try FDPFReferenceCase.load("case300").network()
        for variant in FDPFVariant.allCases {
            var options = PowerFlowOptions()
            options.method = .fastDecoupled
            options.fdpfVariant = variant
            let a = PowerFlowEngine().solve(net, options: options)
            let b = PowerFlowEngine().solve(net, options: options)
            XCTAssertTrue(a.converged)
            XCTAssertEqual(a.vmPu, b.vmPu, "\(variant): vm not bit-identical")
            XCTAssertEqual(a.vaRad, b.vaRad, "\(variant): va not bit-identical")
            XCTAssertEqual(a.branchFlows, b.branchFlows, "\(variant): flows not bit-identical")
            XCTAssertEqual(a.genQPu, b.genQPu, "\(variant): genQ not bit-identical")
        }

        let base = try ReferenceCase.load("case118").network()
        let steps = multiplierSteps(base, [0.8, 1.0, 1.2, 0.9])
        var options = PowerFlowOptions()
        options.method = .fastDecoupledWarmStart
        let r1 = TimeSeriesSweep().run(base: base, steps: steps, options: options)
        let r2 = TimeSeriesSweep().run(base: base, steps: steps, options: options)
        for (a, b) in zip(r1, r2) {
            XCTAssertEqual(a.vmPu, b.vmPu)
            XCTAssertEqual(a.vaRad, b.vaRad)
        }
    }
}
