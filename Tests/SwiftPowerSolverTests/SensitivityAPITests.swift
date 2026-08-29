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

    /// The two mismatch branches of `completeDCBranchFlows` throw DIFFERENT
    /// errors carrying the SIGNATURES THAT ACTUALLY DISAGREE. This asserts the
    /// diagnostic CONTENT, not merely that something threw: until 2026-08-29
    /// the shift branch threw `.signatureMismatch(expected:actual:)` with two
    /// IDENTICAL factors signatures — line 207 had already proved them equal —
    /// so the check fired and the message pointed at the wrong repair on
    /// exactly the shifter-bearing networks C1 defends.
    func testMismatchDiagnosticsNameTheSignaturesThatDisagree() throws {
        let net = shifterLoop()
        XCTAssertEqual(net.branches.filter { $0.shiftRad != 0 }.count, 1,
                       "fixture must carry a phase shifter")
        let ptdf = try SensitivityEngine().ptdf(net)
        let terms = try PhaseShiftTerms.of(net)
        let inj: [BusID: Double] = [BusID(1): -0.4, BusID(2): -0.3, BusID(3): -0.2]

        // Same network: both signatures validate and flows come back.
        XCTAssertNoThrow(try ptdf.completeDCBranchFlows(
            injections: inj, phaseShift: terms, network: net))

        // Retune ONLY the shifter: factors stay valid, the terms are stale.
        let retuned = shifterLoop(shiftRad: 0.09)
        XCTAssertThrowsError(try ptdf.completeDCBranchFlows(
            injections: inj, phaseShift: terms, network: retuned)) { err in
            guard case let SensitivityError.shiftSignatureMismatch(expected, actual) = err else {
                return XCTFail("expected .shiftSignatureMismatch, got \(err)")
            }
            XCTAssertEqual(expected, terms.signature,
                           "`expected` must be the STALE terms' signature")
            XCTAssertNotEqual(expected, actual,
                              "the diagnostic must carry a DISAGREEING pair")
            XCTAssertEqual(expected.factors, actual.factors,
                           "factors agree here — which is what makes this the shift branch")
        }

        // Move a reactance instead: the FACTORS branch fires, with ITS pair.
        let xMoved = shifterLoop(x: 0.20)
        XCTAssertThrowsError(try ptdf.completeDCBranchFlows(
            injections: inj, phaseShift: terms, network: xMoved)) { err in
            guard case let SensitivityError.signatureMismatch(expected, actual) = err else {
                return XCTFail("expected .signatureMismatch, got \(err)")
            }
            XCTAssertEqual(expected, ptdf.signature,
                           "`expected` must be the stale PTDF's signature")
            XCTAssertNotEqual(expected, actual,
                              "the factors diagnostic must also carry a disagreeing pair")
        }
    }

    /// The widening ruled 2026-08-29: a consumer can RECOMPUTE the shift
    /// signature from a network, so "are these terms current for this
    /// network" is checkable without calling the flow path and catching.
    /// Exercises the PUBLIC one-argument `of`; the two-argument fast path
    /// stays internal.
    func testShiftSignatureIsRecomputableByACaller() throws {
        let net = shifterLoop()
        let terms = try PhaseShiftTerms.of(net)
        XCTAssertEqual(try PhaseShiftSignature.of(net), terms.signature,
                       "recomputing from the same network must match the terms' signature")
        XCTAssertNotEqual(try PhaseShiftSignature.of(shifterLoop(shiftRad: 0.09)),
                          terms.signature,
                          "a retuned shifter must be detectable by recomputation")
        XCTAssertEqual(try PhaseShiftSignature.of(net).factors,
                       try FactorsSignature.of(net),
                       "the public path must embed the same factors signature")
    }

    // MARK: - Calibration for `defaultResidualTolerance` (NOT a gate)

    /// The measurement the shipped constant cites. Without it the constant is a
    /// ghost. This is calibration, not gate 6.x — it reports a spread and
    /// asserts only that the healthy maximum sits far below the shipped
    /// tolerance and the singular case far above it.
    ///
    /// **SOLE CARRIER (2026-08-29): the shipped 1e-6 rests on this test and
    /// on nothing else.** It is the only place the NODAL-BALANCE quantity the
    /// guard actually enforces is asserted against the tolerance in both
    /// directions. `SingularFactorsTests`' residual measurements are the
    /// SUPERSEDED column quantity (`‖B_red·x − e‖∞`) and back no shipped
    /// threshold — see `testMeasureColumnResidualAtScale`.
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

    // MARK: - Unit 2 measurement: does wrapping COPY? (not a gate)

    /// `PTDFResult` and `LODFResult` wrap matrices reaching 1.19 GB at
    /// case9241. Array COW should make wrapping free, and `Sendable` was added
    /// in 1a precisely so results could hold factors without copying — **but
    /// that is an argument, not a measurement.** An unintended copy shows as
    /// roughly a gigabyte of extra peak and passes every correctness gate.
    ///
    /// Baselines: build 3,845 ms / 4,126 ms; peak `phys_footprint` 5.796 GiB.
    /// Those are the two quantities the iPad decision depends on (D67).
    func testUnit2_enginePathFootprintAtScale() throws {
        guard let path = FactorsIdentityTests.factorsCasePaths()
                  .first(where: { $0.contains("9241") }) else {
            XCTFail("No case9241 fixture (env unset AND committed copy missing) "
                    + "— the unit 2 measurement cannot run")
            return
        }
        let d = JSONDecoder(); d.keyDecodingStrategy = .convertFromSnakeCase
        let net = try d.decode(FactorsIdentityTests.NetworkFixture.self,
                               from: Data(contentsOf: URL(fileURLWithPath: path))).network()

        let baseline = FootprintTests.physFootprint()
        let peak = UnsafeMutablePointer<Int>.allocate(capacity: 1)
        peak.pointee = baseline
        let stop = UnsafeMutablePointer<Bool>.allocate(capacity: 1)
        stop.pointee = false
        defer { peak.deallocate(); stop.deallocate() }
        let sampler = Thread {
            while !stop.pointee {
                peak.pointee = max(peak.pointee, FootprintTests.physFootprint())
                usleep(2000)
            }
        }
        sampler.start()

        let t0 = ContinuousClock.now
        let engine = SensitivityEngine()
        let ptdf = try engine.ptdf(net)
        let lodf = try engine.lodf(net, from: ptdf)
        let dt = ContinuousClock.now - t0
        let after = FootprintTests.physFootprint()
        stop.pointee = true
        Thread.sleep(forTimeInterval: 0.02)

        let ms = Double(dt.components.seconds) * 1000
               + Double(dt.components.attoseconds) / 1e15
        func gib(_ b: Int) -> String { String(format: "%.3f GiB", Double(b) / 1_073_741_824) }
        note(String(format: "unit2 case9241 ENGINE PATH: %.0f ms (baseline build 3845/4126 ms)", ms))
        note("unit2   peak phys_footprint  : \(gib(peak.pointee - baseline)) "
             + "(baseline 5.796 GiB)")
        note("unit2   retained after       : \(gib(after - baseline))")
        note("unit2   residual carried     : \(String(describing: ptdf.solveResidual))")
        // ATTRIBUTION. The baseline is `DistributionFactors.build` ALONE; the
        // engine path additionally computes the nodal-balance residual
        // (control 2, O(n·(n+nbr)) and always on by default), the connectivity
        // bridge set, and the conditioning array. Comparing the totals without
        // splitting them would report a "regression" that is really new work.
        let t1 = ContinuousClock.now
        _ = DistributionFactors.build(net)
        let dtBuild = ContinuousClock.now - t1
        let msBuild = Double(dtBuild.components.seconds) * 1000
                    + Double(dtBuild.components.attoseconds) / 1e15

        let t2 = ContinuousClock.now
        _ = try SensitivityEngine(residualTolerance: nil).ptdf(net)
        let dtNoResid = ContinuousClock.now - t2
        let msNoResid = Double(dtNoResid.components.seconds) * 1000
                      + Double(dtNoResid.components.attoseconds) / 1e15

        // Stated at the resolution the measurement actually supports. `ms`
        // covers ptdf(with residual) + lodf(); `msNoResid` covers ptdf(without
        // residual) only. Their difference is therefore residual + connectivity
        // + conditioning COMBINED, and this measurement does not separate them.
        note(String(format: "unit2   attribution: DistributionFactors.build alone %.0f ms | "
                    + "ptdf without residual %.0f ms | full engine path %.0f ms",
                    msBuild, msNoResid, ms))
        note(String(format: "unit2   -> engine path costs ~%.0f ms over the bare build: "
                    + "control 2's residual PLUS connectivity PLUS conditioning, "
                    + "not separated by this measurement", ms - msBuild))
        // ISOLATE the residual, so the sampling question is decided on a
        // number rather than on a delta that bundles three things.
        let f = DistributionFactors.build(net)
        let t3 = ContinuousClock.now
        _ = SensitivityEngine.nodalBalanceResidual(net, factors: f)
        let dtR = ContinuousClock.now - t3
        let msR = Double(dtR.components.seconds) * 1000
                + Double(dtR.components.attoseconds) / 1e15
        let t4 = ContinuousClock.now
        _ = NetworkConnectivity.bridgeBranches(net)
        let dtC = ContinuousClock.now - t4
        let msC = Double(dtC.components.seconds) * 1000
                + Double(dtC.components.attoseconds) / 1e15
        note(String(format: "unit3a ISOLATED at case9241: residual %.0f ms | "
                    + "connectivity %.0f ms | bare build %.0f ms", msR, msC, msBuild))
        XCTAssertFalse(lodf.branchOrder.isEmpty)
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
