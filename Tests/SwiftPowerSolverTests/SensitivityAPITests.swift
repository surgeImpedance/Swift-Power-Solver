import Foundation
import XCTest
@testable import SwiftPowerSolver

/// Unit 1a's permitted test surface: gate **6.14** (K2's signature-superset
/// constraint) plus the calibration measurement the shipped
/// `SensitivityEngine.defaultResidualTolerance` cites. Gates 6.1–6.13, 6.15 and
/// 6.16 are unit 3's.
final class SensitivityAPITests: XCTestCase {

    private func note(_ m: String) {
        FileHandle.standardError.write(Data((m + "\n").utf8))
    }

    /// A four-bus loop carrying ONE phase shifter, so the fixture is not an
    /// empty payload for a test whose whole subject is the shift term.
    private func shifterLoop(x: Double = 0.10, shiftRad: Double = 0.05)
        -> BusBranchNetwork {
        BusBranchNetwork(
            baseMVA: 100,
            buses: [
                .init(type: .slack, baseKv: 138),
                .init(type: .pq, baseKv: 138, pLoadPu: 0.4),
                .init(type: .pq, baseKv: 138, pLoadPu: 0.3),
                .init(type: .pq, baseKv: 138, pLoadPu: 0.2),
            ],
            branches: [
                .init(from: 0, to: 1, r: 0.01, x: x),
                .init(from: 1, to: 2, r: 0.01, x: 0.10, shiftRad: shiftRad),
                .init(from: 2, to: 3, r: 0.01, x: 0.10),
                .init(from: 3, to: 0, r: 0.01, x: 0.10),
            ],
            generators: [.init(bus: 0, pPu: 0.9, vSetPu: 1.0)])
    }

    // MARK: - Gate 6.14 — the shift signature is a STRICT SUPERSET

    func test614_shiftSignatureIsAStrictSupersetOfFactorsSignature() throws {
        let net = shifterLoop()

        // MODE 1 (empty payload): the fixture must actually carry a shifter, or
        // every assertion below is about nothing.
        let shifters = net.branches.filter { $0.shiftRad != 0 }.count
        XCTAssertEqual(shifters, 1, "fixture must carry a phase shifter")

        let f0 = try FactorsSignature.of(net)
        let s0 = try PhaseShiftTerms.of(net).signature

        // The nesting is STRUCTURAL — it holds by construction, and is asserted
        // so a future change that flattens it fails here.
        XCTAssertEqual(s0.factors, f0,
                       "the shift signature must EMBED the factors signature")

        // --- direction 1: mutate `x` on the SHIFTER-BEARING branch. Both move.
        // This is K2's actual subject: pFinj = -b*shift depends on b as well as
        // shift, so a reactance change must invalidate the shift terms too.
        var xMoved = net
        xMoved.branches[1].x = 0.20
        let f1 = try FactorsSignature.of(xMoved)
        let s1 = try PhaseShiftTerms.of(xMoved).signature
        XCTAssertNotEqual(f1, f0, "changing x must move the factors signature")
        XCTAssertNotEqual(s1, s0, "changing x must ALSO move the shift signature")
        XCTAssertEqual(s1.factors, f1, "nesting must survive the mutation")

        // --- direction 2: mutate shiftRad only. Shift moves, factors does NOT.
        // This is what makes it a STRICT superset rather than an alias: if the
        // factors signature moved here, B2's whole benefit is gone and a shifter
        // retune would discard valid factors.
        let shiftMoved = shifterLoop(shiftRad: 0.09)
        let f2 = try FactorsSignature.of(shiftMoved)
        let s2 = try PhaseShiftTerms.of(shiftMoved).signature
        XCTAssertEqual(f2, f0, "changing ONLY shiftRad must NOT move the factors signature")
        XCTAssertNotEqual(s2, s0, "changing shiftRad must move the shift signature")

        note("6.14: shifters=\(shifters)  f0=\(f0.digest.prefix(12))  "
             + "s0=\(s0.digest.prefix(12))  x->both moved  shift->only shift moved")
    }

    // MARK: - Calibration for `defaultResidualTolerance` (NOT a gate)

    /// The measurement the shipped constant cites. Without it the constant is a
    /// ghost. This is calibration, not gate 6.x — it reports a spread and
    /// asserts only that the healthy maximum sits far below the shipped
    /// tolerance and the singular case far above it.
    func testResidualCalibration() throws {
        var rows: [(String, Double)] = []
        for name in ["case14", "case39", "case118"] {
            let net = try ReferenceCase.load(name).network()
            let f = DistributionFactors.build(net)
            rows.append((name, SensitivityEngine.nodalBalanceResidual(net, factors: f)))
        }
        // The rank-deficient fixture: two islands, a slack in only one.
        let singular = BusBranchNetwork(
            baseMVA: 100,
            buses: [.init(type: .slack, baseKv: 138), .init(type: .pq, baseKv: 138, pLoadPu: 0.5),
                    .init(type: .pq, baseKv: 138, pLoadPu: 0.3), .init(type: .pq, baseKv: 138, pLoadPu: 0.2)],
            branches: [.init(from: 0, to: 1, r: 0.01, x: 0.10),
                       .init(from: 2, to: 3, r: 0.01, x: 0.10)],
            generators: [.init(bus: 0, pPu: 1.0, vSetPu: 1.0)])
        let sf = DistributionFactors.build(singular)
        let sr = SensitivityEngine.nodalBalanceResidual(singular, factors: sf)

        for (n, r) in rows { note(String(format: "1a CALIB healthy  %@  nodal residual = %.6e", n, r)) }
        note(String(format: "1a CALIB singular twoIslandOneSlack  nodal residual = %.6e", sr))
        let worstHealthy = rows.map(\.1).max() ?? 0
        note(String(format: "1a CALIB worst healthy = %.6e, singular = %.6e, "
                    + "shipped tolerance = %.0e", worstHealthy, sr,
                    SensitivityEngine.defaultResidualTolerance))

        XCTAssertLessThan(worstHealthy, SensitivityEngine.defaultResidualTolerance,
                          "healthy networks must pass the shipped tolerance")
        XCTAssertGreaterThan(sr, SensitivityEngine.defaultResidualTolerance,
                             "the rank-deficient fixture must fail it")
    }

    /// The engine must refuse a garbage solve rather than return factors
    /// (D65 §3 item 5). Not gate 6.x — this pins the control wired in 1a.
    func testEngineThrowsOnTheSingularFixture() throws {
        let singular = BusBranchNetwork(
            baseMVA: 100,
            buses: [.init(type: .slack, baseKv: 138), .init(type: .pq, baseKv: 138, pLoadPu: 0.5),
                    .init(type: .pq, baseKv: 138, pLoadPu: 0.3), .init(type: .pq, baseKv: 138, pLoadPu: 0.2)],
            branches: [.init(from: 0, to: 1, r: 0.01, x: 0.10),
                       .init(from: 2, to: 3, r: 0.01, x: 0.10)],
            generators: [.init(bus: 0, pPu: 1.0, vSetPu: 1.0)])
        XCTAssertThrowsError(try SensitivityEngine().ptdf(singular)) { err in
            guard case SensitivityError.singularAdmittanceMatrix(let r) = err else {
                return XCTFail("expected .singularAdmittanceMatrix, got \(err)")
            }
            self.note(String(format: "1a: engine refused the singular fixture, residual %.6e",
                             r ?? .nan))
        }
    }
}
