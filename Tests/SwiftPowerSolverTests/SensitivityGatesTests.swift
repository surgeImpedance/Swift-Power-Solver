import Foundation
import XCTest
@testable import SwiftPowerSolver

/// Unit 3a — the gates runnable against fixtures that already exist.
/// Fixture-building gates are 3b's, kept separate because every empty-payload
/// failure this milestone found came from a fixture not containing the thing
/// under test, and burying those among the straightforward gates guarantees
/// they get less scrutiny.
final class SensitivityGatesTests: XCTestCase {

    private func note(_ m: String) {
        FileHandle.standardError.write(Data((m + "\n").utf8))
    }
    private let cases = ["case14", "case39", "case118"]

    // MARK: 6.1 — zero PTDF column for every REFERENCE bus
    //
    // The isolated-bus half of 6.1 is 3b's: 1b's coverage table measured
    // isolated = 0 across all seven fixtures, so it would be an empty payload
    // here. Split rather than reported as covered.
    func test6_1_zeroColumnForEveryReferenceBus() throws {
        for name in cases {
            let net = try ReferenceCase.load(name).network()
            let refs = (0..<net.busCount).filter { net.buses[$0].type == .slack }
            XCTAssertFalse(refs.isEmpty, "\(name): no reference bus — empty payload")
            let ptdf = try SensitivityEngine().ptdf(net)
            var worst = 0.0
            for k in 0..<net.branches.count {
                for s in refs {
                    worst = max(worst, abs(try ptdf[BranchID(k), BusID(s)]))
                }
            }
            note("6.1 \(name): \(refs.count) reference bus(es), "
                 + "\(net.branches.count) branches, max |PTDF[l,s]| = \(worst)")
            XCTAssertEqual(worst, 0.0, "\(name): reference column must be EXACTLY zero")
        }
    }

    // MARK: 6.3 — the qualified diagonal
    //
    // `-1` for an ACTIVE outage, `0` for islanding. The out-of-service arm is
    // 3b's: oos = 0 across every fixture (1b coverage), so it is an empty
    // payload here.
    func test6_3_qualifiedDiagonal() throws {
        for name in cases {
            let net = try ReferenceCase.load(name).network()
            let f = DistributionFactors.build(net)
            let bridges = NetworkConnectivity.bridgeBranches(net)
            var active = 0, islanding = 0
            for k in 0..<net.branches.count {
                let d = f.lodf(monitored: k, outaged: k)
                if bridges[k] {
                    islanding += 1
                    XCTAssertEqual(d, 0.0, "\(name) branch \(k): islanding diagonal must be 0")
                } else {
                    active += 1
                    XCTAssertEqual(d, -1.0, "\(name) branch \(k): active diagonal must be -1 EXACTLY")
                }
            }
            XCTAssertGreaterThan(active, 0, "\(name): no active outage — empty payload")
            XCTAssertGreaterThan(islanding, 0, "\(name): no islanding outage — empty payload")
            note("6.3 \(name): \(active) active diagonals == -1 exactly, "
                 + "\(islanding) islanding diagonals == 0")
        }
    }

    // MARK: 6.6 — islanding: predicate true, query throws, no NaN/Inf public
    func test6_6_islandingSurface() throws {
        for name in cases {
            let net = try ReferenceCase.load(name).network()
            let engine = SensitivityEngine()
            let lodf = try engine.lodf(net, from: try engine.ptdf(net))
            let bridges = (0..<net.branches.count).filter { lodf.isOutageIslanding(BranchID($0)) }
            XCTAssertFalse(bridges.isEmpty, "\(name): no bridge — empty payload")

            for k in bridges {
                XCTAssertThrowsError(try lodf[BranchID(0), BranchID(k)]) { e in
                    guard case SensitivityError.islandingOutage = e else {
                        return XCTFail("\(name) branch \(k): expected .islandingOutage, got \(e)")
                    }
                }
                XCTAssertThrowsError(try lodf.postContingencyFlows(
                    outaging: BranchID(k), baseFlows: [:]))
            }
            // No NaN/Inf on ANY public surface, across every non-islanding pair.
            var nonFinite = 0
            for m in 0..<net.branches.count {
                for k in 0..<net.branches.count where !lodf.isOutageIslanding(BranchID(k)) {
                    let v = try lodf[BranchID(m), BranchID(k)]
                    if !v.isFinite { nonFinite += 1 }
                }
                let c = try lodf.conditioning(outaging: BranchID(m))
                if !(c.isFinite || c.isNaN) { nonFinite += 1 }
            }
            XCTAssertEqual(nonFinite, 0, "\(name): non-finite value on a public surface")
            note("6.6 \(name): \(bridges.count) bridges, all throw .islandingOutage; "
                 + "0 non-finite across \(net.branches.count)^2 public queries")
        }
    }

    // MARK: 6.11 — signature completeness, BOTH directions
    func test6_11_signatureCompleteness() throws {
        let net = try ReferenceCase.load("case118").network()
        let base = try FactorsSignature.of(net)

        // POSITIVE — this is where the evidence is. Each mutation is an input
        // the factors genuinely depend on; the signature MUST move.
        var m1 = net; m1.branches[3].x *= 1.5
        var m2 = net; m2.branches[3].tap = 1.07
        var m3 = net; m3.branches[3].inService = false
        var m4 = net; m4.branches[3].to = m4.branches[5].to
        var m5 = net; m5.buses[7].type = .isolated
        var m6 = net; m6.buses[7].type = .slack
        for (label, mutated) in [("x", m1), ("tap", m2), ("inService", m3),
                                 ("terminal", m4), ("isolated", m5), ("slack", m6)] {
            XCTAssertNotEqual(try FactorsSignature.of(mutated), base,
                              "mutating \(label) MUST move the signature")
        }

        // NEGATIVE — passes BY CONSTRUCTION and is not evidence about today's
        // code. These quantities are not inputs to the hash, so the test cannot
        // fail on the current implementation. It guards FUTURE DRIFT in what
        // `build` consumes; the POSITIVE direction above is what would catch a
        // widened build. A green 6.11 must not be read as "the signature is
        // verified complete".
        var n1 = net; n1.buses[7].pLoadPu += 0.5
        var n2 = net; n2.generators[0].pPu += 0.25
        var n3 = net; n3.branches[3].shiftRad = 0.04
        var n4 = net; n4.branches[3].ratingMva = 999
        var n5 = net; n5.branches[3].r *= 2
        var n6 = net
        if let i = (0..<n6.busCount).first(where: { n6.buses[$0].type == .pv }) {
            n6.buses[i].type = .pq
        }
        for (label, mutated) in [("load P", n1), ("gen P", n2), ("shiftRad", n3),
                                 ("ratingMva", n4), ("r", n5), ("pv->pq", n6)] {
            XCTAssertEqual(try FactorsSignature.of(mutated), base,
                           "mutating \(label) must NOT move the signature")
        }
        // The ENCODING half of §4's ruling, which the mutation directions do not
        // reach: not `Hasher`, bit pattern, -0.0 normalised, non-finite rejected.

        // (a) NOT Swift's `Hasher`. A pinned constant IS the test: `Hasher` is
        //     per-process seeded, so a Hasher-derived digest could not be
        //     pinned across runs at all.
        let case14 = try ReferenceCase.load("case14").network()
        let pinned = try FactorsSignature.of(case14).digest
        XCTAssertEqual(pinned, "f7b38a951eea9c7caaaadfceb9be7f9e",
                       "case14 signature must be STABLE across processes")

        // (b) non-finite rejected rather than hashed.
        var bad = net; bad.branches[3].x = .nan
        XCTAssertThrowsError(try FactorsSignature.of(bad)) { e in
            guard case SensitivityError.invalidNetworkParameter = e else {
                return XCTFail("expected .invalidNetworkParameter, got \(e)")
            }
        }

        // (c) -0.0 normalised to 0.0. An out-of-service branch contributes
        //     bSeries 0; forcing the sign bit must not move the digest.
        var negZero = net; negZero.branches[3].inService = false
        var posZero = net; posZero.branches[3].inService = false
        negZero.branches[3].x = -0.0
        posZero.branches[3].x = 0.0
        XCTAssertEqual(try FactorsSignature.of(negZero), try FactorsSignature.of(posZero),
                       "-0.0 must normalise to 0.0 before hashing")

        note("6.11: 6 positive mutations moved the signature; 6 negative did not "
             + "(negative direction holds BY CONSTRUCTION — guards future drift)")
    }

    // MARK: 6.19 — control 2 refuses a garbage solve rather than returning it
    func test6_19_residualControlRefusesGarbage() throws {
        let singular = BusBranchNetwork(
            baseMVA: 100,
            buses: [.init(type: .slack, baseKv: 138), .init(type: .pq, baseKv: 138, pLoadPu: 0.5),
                    .init(type: .pq, baseKv: 138, pLoadPu: 0.3), .init(type: .pq, baseKv: 138, pLoadPu: 0.2)],
            branches: [.init(from: 0, to: 1, r: 0.01, x: 0.10),
                       .init(from: 2, to: 3, r: 0.01, x: 0.10)],
            generators: [.init(bus: 0, pPu: 1.0, vSetPu: 1.0)])
        // The payload: QR SUCCEEDS here (D65 §0), so a status check would never
        // fire. The residual is the only thing that sees it.
        let f = DistributionFactors.build(singular)
        let r = SensitivityEngine.nodalBalanceResidual(singular, factors: f)
        XCTAssertGreaterThan(r, 1e-3, "the fixture must actually be garbage")
        XCTAssertThrowsError(try SensitivityEngine().ptdf(singular)) { e in
            guard case SensitivityError.singularAdmittanceMatrix(let got) = e else {
                return XCTFail("expected .singularAdmittanceMatrix, got \(e)")
            }
            XCTAssertEqual(got, r)
        }
        // And healthy fixtures are NOT refused — the inversion, without which a
        // control that refuses everything would pass.
        for name in cases {
            let net = try ReferenceCase.load(name).network()
            XCTAssertNoThrow(try SensitivityEngine().ptdf(net), "\(name) must not be refused")
        }
        note("6.19: singular fixture residual \(r), refused; case14/39/118 all accepted")
    }

    // MARK: 6.20 — the conditioning annotation is present and large
    func test6_20_conditioningAnnotationIsExposedAndLarge() throws {
        // PROVENANCE, required in the gate itself (E3): this constructed
        // family's single-tie reactance ratio is 1e6 against neighbours at 0.10,
        // i.e. 1e7 — which EXCEEDS case9241's whole-network spread of 4.07e5.
        // A green 6.20 is therefore evidence about CONSTRUCTED networks only.
        // The real-network maximum is reported alongside as calibration.
        //
        // FIXTURE CORRECTION, recorded because the gate caught it. The first
        // draft was a weak tie inside a loop with a STRONG parallel path — for
        // which h -> 0, so |1-h| -> 1. That is an ANTI-near-bridge: the gate
        // measured 0.99999970 and failed. A near-bridge is the DOMINANT path:
        // removing it nearly disconnects, so h -> 1 and |1-h| -> 0.
        //
        // Correct construction: two 2-bus halves joined by a PARALLEL PAIR of
        // ties, one strong and one very weak. Neither is a graph bridge (a
        // parallel pair never is), so connectivity correctly reports non-bridge
        // and control 1 does not fire — which is precisely the regime where the
        // annotation is the only signal. Branch 2 is the strong tie:
        // h ~ b_strong / (b_strong + b_weak) -> 1 as the weak tie vanishes.
        func tie(_ weakX: Double) -> BusBranchNetwork {
            BusBranchNetwork(
                baseMVA: 100,
                buses: [.init(type: .slack, baseKv: 138), .init(type: .pq, baseKv: 138, pLoadPu: 0.5),
                        .init(type: .pq, baseKv: 138, pLoadPu: 0.3), .init(type: .pq, baseKv: 138, pLoadPu: 0.2)],
                branches: [.init(from: 0, to: 1, r: 0.01, x: 0.10),
                           .init(from: 2, to: 3, r: 0.01, x: 0.10),
                           .init(from: 1, to: 2, r: 0.01, x: 0.10),      // strong tie
                           .init(from: 1, to: 2, r: 0.01, x: weakX)],    // weak parallel tie
                generators: [.init(bus: 0, pPu: 0.9, vSetPu: 1.0)])
        }
        let engine = SensitivityEngine()
        let near = tie(1e6)
        let lodf = try engine.lodf(near, from: try engine.ptdf(near))
        let cond = try lodf.conditioning(outaging: BranchID(2))
        XCTAssertFalse(lodf.isOutageIslanding(BranchID(2)),
                       "a parallel pair is never a bridge — control 1 must NOT fire")
        XCTAssertTrue(cond.isFinite, "the annotation must be present, not NaN")
        XCTAssertLessThan(cond, 1e-3, "a near-bridge must annotate as ill-conditioned")
        // Deliberately NOT asserting the LODF VALUE: at this conditioning it is
        // not meaningfully correct, and asserting one would be a ghost.

        let healthy = tie(0.10)
        let hc = try engine.lodf(healthy, from: try engine.ptdf(healthy))
            .conditioning(outaging: BranchID(2))
        XCTAssertGreaterThan(hc, 1e-2, "a healthy branch must not annotate as near-bridge")

        note("6.20: constructed near-bridge |1-h| = \(cond) (ratio 1e7 > case9241's "
             + "4.07e5 whole-network spread); healthy control |1-h| = \(hc); "
             + "real-network worst is 5.348e-04 at case9241 branch 8013")
    }
}
