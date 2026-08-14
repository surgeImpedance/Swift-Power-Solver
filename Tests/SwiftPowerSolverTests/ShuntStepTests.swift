import XCTest
@testable import SwiftPowerSolver

// LoadStep.busShuntsPu — the per-step switched-shunt channel (substation-lab
// D11). Additive: an empty map is bit-identical to the fixed-shunt sweep.
final class ShuntStepTests: XCTestCase {

    func testShuntOverrideMatchesDirectSolve() throws {
        let net = try ReferenceCase.load("case118").network()
        // Pick a bus that carries a shunt in the fixture.
        let bus = try XCTUnwrap(net.buses.firstIndex { $0.bsPu != 0 })
        let scaled = LoadStep.BusShunt(gsPu: net.buses[bus].gsPu,
                                       bsPu: net.buses[bus].bsPu * 1.3)

        let step = LoadStep(busShuntsPu: [bus: scaled])
        let swept = TimeSeriesSweep().run(base: net, steps: [step],
                                          options: PowerFlowOptions())
        XCTAssertEqual(swept.count, 1)
        XCTAssertTrue(swept[0].converged)

        var direct = net
        direct.buses[bus].gsPu = scaled.gsPu
        direct.buses[bus].bsPu = scaled.bsPu
        let sol = NewtonRaphsonSolver().solve(direct, options: PowerFlowOptions())
        XCTAssertEqual(swept[0].vmPu, sol.vmPu, "shunt-stepped sweep == direct solve, bitwise")
        XCTAssertEqual(swept[0].vaRad, sol.vaRad)
        // And it genuinely differs from the unmodified network.
        let base = NewtonRaphsonSolver().solve(net, options: PowerFlowOptions())
        XCTAssertNotEqual(swept[0].vmPu, base.vmPu)
    }

    /// Empty busShuntsPu (the default) leaves the sweep bit-identical to the
    /// pre-field behavior — the additive proof.
    func testEmptyShuntMapIsInert() throws {
        let net = try ReferenceCase.load("case14").network()
        let loads = Dictionary(uniqueKeysWithValues: net.buses.enumerated().map {
            ($0.offset, LoadStep.BusLoad(pPu: $0.element.pLoadPu * 1.1,
                                         qPu: $0.element.qLoadPu * 1.1))
        })
        let a = TimeSeriesSweep().run(base: net, steps: [LoadStep(busLoads: loads)],
                                      options: PowerFlowOptions())
        let b = TimeSeriesSweep().run(base: net,
                                      steps: [LoadStep(busLoads: loads, busShuntsPu: [:])],
                                      options: PowerFlowOptions())
        XCTAssertEqual(a[0].vmPu, b[0].vmPu)
        XCTAssertEqual(a[0].iterations, b[0].iterations)
    }

    /// B″ depends on bus susceptance, so a shunt-varying FDPF sweep must
    /// refactorize per distinct shunt state — the cache key already covers
    /// bsPu; this pins that the invalidation actually fires (and the existing
    /// load-only test pins that it never fires spuriously).
    func testFDPFCacheInvalidatesOnShuntSteps() throws {
        let net = try ReferenceCase.load("case118").network()
        let bus = try XCTUnwrap(net.buses.firstIndex { $0.bsPu != 0 })
        let steps = (0..<4).map { i in
            LoadStep(busShuntsPu: [bus: .init(gsPu: net.buses[bus].gsPu,
                                              bsPu: net.buses[bus].bsPu * (1.0 + 0.1 * Double(i)))])
        }
        var options = PowerFlowOptions()
        options.method = .fastDecoupled
        let cache = FDPFFactorizationCache()
        let swept = TimeSeriesSweep().run(base: net, steps: steps, options: options,
                                          fdpfCache: cache)
        XCTAssertTrue(swept.allSatisfy(\.converged))
        XCTAssertEqual(cache.factorizationCount, 2 * steps.count,
                       "each distinct shunt state must refactorize B′/B″")
    }
}
