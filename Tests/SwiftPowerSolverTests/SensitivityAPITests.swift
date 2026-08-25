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

    // MARK: - Gate 6.17 (Q1) — the single-bus post-shift is BITWISE identical

    /// `PTDF_networkDefined[:, s] = 0` for a reference bus `s`, so post-shifting
    /// by `s` subtracts a zero column. On a network with exactly one slack bus,
    /// `.uniformlyDistributed([s])` must therefore equal `.networkDefined`
    /// **bitwise** — no tolerance, no oracle.
    ///
    /// That identity — changing slack from `s` to `b` is subtraction of column
    /// `b` — is the whole justification for implementing the case as a
    /// post-shift instead of opening the frozen build to support an explicit
    /// reference set. If this gate ever fails, the justification is void.
    func test617_singleBusPostShiftIsBitwiseIdenticalToNetworkDefined() throws {
        for name in ["case14", "case39", "case118"] {
            let net = try ReferenceCase.load(name).network()
            let slacks = (0..<net.busCount).filter { net.buses[$0].type == .slack }
            // MODE 1 (empty payload): the identity is only claimed for a single
            // reference, so a multi-slack fixture would silently test nothing.
            XCTAssertEqual(slacks.count, 1, "\(name) must have exactly one slack")

            let engine = SensitivityEngine()
            let nd = try engine.ptdf(net, slack: .networkDefined)
            let ud = try engine.ptdf(net, slack: .uniformlyDistributed([BusID(slacks[0])]))

            let a = nd.rowMajorValues(), b = ud.rowMajorValues()
            XCTAssertEqual(a.count, b.count)
            var differing = 0
            for i in 0..<min(a.count, b.count)
            where a[i].bitPattern != b[i].bitPattern { differing += 1 }
            XCTAssertEqual(differing, 0,
                "\(name): \(differing) of \(a.count) values differ BITWISE between "
                + ".networkDefined and .uniformlyDistributed([slack])")
            note("6.17 \(name): \(a.count) values, \(differing) differing bitwise")
        }
    }

    // MARK: - Q2 — C2's graded family against the NEW instrument

    /// The nodal-balance residual was calibrated on healthy cases and the fully
    /// rank-deficient fixture — **endpoints only**, which is exactly the
    /// calibration C2 demolished for the previous instrument. Nothing about that
    /// finding was specific to `‖B_red·x − e‖∞`; it was about what endpoints
    /// hide. So the graded family runs against THIS instrument before 1e-6 is
    /// treated as settled.
    func testQ2_gradedFamilyAgainstNodalBalanceResidual() throws {
        func gradedTie(xTie: Double?) -> BusBranchNetwork {
            var branches: [BusBranchNetwork.Branch] = [
                .init(from: 0, to: 1, r: 0.01, x: 0.10),
                .init(from: 2, to: 3, r: 0.01, x: 0.10),
            ]
            if let xTie { branches.append(.init(from: 1, to: 2, r: 0.01, x: xTie)) }
            return BusBranchNetwork(
                baseMVA: 100,
                buses: [.init(type: .slack, baseKv: 138),
                        .init(type: .pq, baseKv: 138, pLoadPu: 0.5),
                        .init(type: .pq, baseKv: 138, pLoadPu: 0.3),
                        .init(type: .pq, baseKv: 138, pLoadPu: 0.2)],
                branches: branches,
                generators: [.init(bus: 0, pPu: 1.0, vSetPu: 1.0)])
        }

        note("Q2: xTie        nodal residual   engine verdict @1e-6")
        var residuals: [(Double, Double)] = []
        for x in [1e-9, 1e-6, 1e-3, 1e-1, 1e0, 1e2, 1e4, 1e6, 1e8, 1e10, 1e12, 1e14, 1e16] {
            let net = gradedTie(xTie: x)
            let f = DistributionFactors.build(net)
            let r = SensitivityEngine.nodalBalanceResidual(net, factors: f)
            residuals.append((x, r))
            let accepted = (try? SensitivityEngine().ptdf(net)) != nil
            note(String(format: "Q2: %-10.0e  %-15.6e  %@", x, r,
                        accepted ? "accepted" : "REFUSED"))
        }
        let disconnected = gradedTie(xTie: nil)
        let dr = SensitivityEngine.nodalBalanceResidual(
            disconnected, factors: DistributionFactors.build(disconnected))
        note(String(format: "Q2: (no tie)    %-15.6e  <- rank-deficient endpoint", dr))

        var worstJump = 0.0, jumpAt = ""
        for i in 1..<residuals.count {
            let a = max(residuals[i - 1].1, 1e-300), b = max(residuals[i].1, 1e-300)
            if b / a > worstJump { worstJump = b / a; jumpAt = "\(residuals[i-1].0) -> \(residuals[i].0)" }
        }
        note(String(format: "Q2: max residual across the family = %.6e; largest "
                    + "consecutive jump %.3e x at xTie %@",
                    residuals.map(\.1).max() ?? 0, worstJump, jumpAt))
        XCTAssertFalse(residuals.isEmpty)
    }
}
