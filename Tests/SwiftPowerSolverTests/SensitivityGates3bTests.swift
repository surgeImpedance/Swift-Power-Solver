import Foundation
import XCTest
@testable import SwiftPowerSolver

/// Unit 3b — the gates that needed a fixture built before they could be
/// written honestly.
///
/// WHY THEY ARE A SEPARATE FILE FROM 3a. Every empty-payload failure this
/// milestone found came from a fixture that did not contain the thing under
/// test — `nshift = 0` on all six IEEE cases, `isolated = 0` and
/// `outOfService = 0` across all seven fixtures, zero near-bridges in a
/// 231-outage sweep. Those gates are the ones most likely to be quietly
/// wrong, and burying them among the straightforward ones guarantees they get
/// less scrutiny, not more.
///
/// EVERY GATE HERE ANSWERS THE PAYLOAD QUESTION WITH A MEASURED VALUE, never
/// with the intent behind building the fixture. "I built a shifter case" is a
/// belief; "the shift term moves base flows by up to 43.633 MW" is an answer.
/// (A fixture built in this same milestone to contain a near-bridge contained
/// its inverse, and only an assertion on the measured value caught it.)
final class SensitivityGates3bTests: XCTestCase {

    private func note(_ m: String) {
        FileHandle.standardError.write(Data((m + "\n").utf8))
    }

    // MARK: - Fixtures

    /// `Reference/shifter.json` — gates 6.2 and 6.15.
    struct ShifterFixtures: Decodable {
        struct Net: Decodable {
            struct B: Decodable { var i: Int; var type: Int
                                  var pdMw, qdMvar, gsMw, bsMvar, baseKv: Double }
            struct R: Decodable { var f, t: Int
                                  var r, x, b, g, tap, shiftDeg: Double; var status: Int }
            struct G: Decodable { var bus: Int
                                  var pgMw, qmaxMvar, qminMvar, vgPu, vaDeg: Double
                                  var status: Int }
            struct DC: Decodable { var vaDeg, branchPFromMw, branchPToMw: [Double] }
            var name: String
            var nShift: Int
            var baseMva: Double
            var buses: [B]
            var branches: [R]
            var gens: [G]
            var dc: DC
            /// THE PAYLOAD, measured by the dumper: the largest base-flow change
            /// produced by zeroing every phase shift and re-solving.
            var shiftTermMaxFlowDeltaMw: Double
            var shiftDegMaxAbs: Double

            func network() -> BusBranchNetwork {
                BusBranchNetwork(
                    baseMVA: baseMva,
                    buses: buses.map {
                        .init(type: BusBranchNetwork.BusType(rawValue: $0.type) ?? .pq,
                              baseKv: $0.baseKv,
                              pLoadPu: $0.pdMw / baseMva, qLoadPu: $0.qdMvar / baseMva,
                              gsPu: $0.gsMw / baseMva, bsPu: $0.bsMvar / baseMva)
                    },
                    branches: branches.map {
                        .init(from: $0.f, to: $0.t, r: $0.r, x: $0.x, b: $0.b, g: $0.g,
                              tap: $0.tap <= 0 ? 1.0 : $0.tap,
                              shiftRad: $0.shiftDeg * .pi / 180,
                              inService: $0.status == 1)
                    },
                    generators: gens.map {
                        .init(bus: $0.bus, pPu: $0.pgMw / baseMva, vSetPu: $0.vgPu,
                              vaRefRad: $0.vaDeg * .pi / 180,
                              qMinPu: $0.qminMvar / baseMva, qMaxPu: $0.qmaxMvar / baseMva,
                              inService: $0.status == 1)
                    })
            }

            /// Physical bus injections in pu: generation − load. The DC model
            /// carries no shunt term.
            func injections() -> [BusID: Double] {
                var p = [Double](repeating: 0, count: buses.count)
                for b in buses { p[b.i] -= b.pdMw / baseMva }
                for g in gens where g.status == 1 { p[g.bus] += g.pgMw / baseMva }
                // The slack absorbs the imbalance; take its injection from the
                // DC solution rather than from the gen table, which for these
                // cases carries the pre-solve schedule.
                return Dictionary(uniqueKeysWithValues:
                    (0..<buses.count).map { (BusID($0), p[$0]) })
            }
        }
        var pandapowerVersion: String
        var networks: [Net]
    }

    /// `Reference/ptdf.json` — gate 6.7.
    struct PTDFReference: Decodable {
        struct Case: Decodable {
            var nBus, nBranch: Int
            var slackBuses: [Int]
            var nonzeroCount: Int
            /// Entries at or above 6.7's own tolerance. `nonzeroCount` counts
            /// numerical dust as well — see the gate.
            var significantCount: Int
            var maxAbs: Double
            var ptdf: [[Double]]
        }
        var pandapowerVersion: String
        var cases: [String: Case]
    }

    /// FAILS rather than skips when a reference is missing (C3): a control that
    /// abstains when its inputs are absent has the failure mode of the thing it
    /// was built to prevent.
    private func load<T: Decodable>(_ type: T.Type, _ name: String) throws -> T {
        guard let url = Bundle.module.url(forResource: name, withExtension: "json",
                                          subdirectory: "Reference") else {
            XCTFail("""
                missing Reference/\(name).json — this gate cannot run, and an \
                absent fixture must NOT read as a pass. Regenerate:
                  python Tools/dump_reference.py ptdf shifter
                """)
            throw SensitivityError.invalidNetworkParameter(
                "unreachable past XCTFail — fixture missing")
        }
        let d = JSONDecoder()
        d.keyDecodingStrategy = .convertFromSnakeCase
        return try d.decode(T.self, from: Data(contentsOf: url))
    }

    private func shifters() throws -> [ShifterFixtures.Net] {
        let f = try load(ShifterFixtures.self, "shifter")
        XCTAssertEqual(f.pandapowerVersion, "3.2.1",
                       "oracle version moved; a disagreement here is the ORACLE, "
                       + "not the code (D68 §3)")
        return f.networks
    }

    // MARK: 6.1 — the ISOLATED arm
    //
    // 1b's coverage table measured isolated = 0 across all seven fixtures, so
    // this arm was split out of 3a rather than reported as covered. The
    // network is constructed because no fixture supplies one.
    func test6_1_zeroColumnForEveryIsolatedBus() throws {
        // Three live buses in a path, plus one isolated bus carrying a branch
        // to nowhere — the shape an agent produces by opening the last breaker
        // on a bus, which is an ordinary Grid2Op/RL2Grid action, not an exotic
        // case (D64 §12).
        let net = BusBranchNetwork(
            baseMVA: 100,
            buses: [.init(type: .slack, baseKv: 138),
                    .init(type: .pq, baseKv: 138),
                    .init(type: .pq, baseKv: 138),
                    .init(type: .isolated, baseKv: 138)],
            branches: [.init(from: 0, to: 1, r: 0.01, x: 0.10),
                       .init(from: 1, to: 2, r: 0.01, x: 0.10),
                       .init(from: 2, to: 0, r: 0.01, x: 0.10),
                       .init(from: 2, to: 3, r: 0.01, x: 0.10)],
            generators: [.init(bus: 0, pPu: 0.5, vSetPu: 1.0)])

        let isolated = (0..<net.busCount).filter { net.buses[$0].type == .isolated }
        XCTAssertEqual(isolated.count, 1,
                       "no isolated bus — this gate's entire payload is absent")

        let ptdf = try SensitivityEngine().ptdf(net)
        var worst = 0.0
        var inspected = 0
        for k in 0..<net.branches.count {
            for s in isolated {
                worst = max(worst, abs(try ptdf[BranchID(k), BusID(s)]))
                inspected += 1
            }
        }
        // WRONG-QUANTITY GUARD. A column of zeros agrees with zero trivially,
        // so the gate also pins that the matrix is NOT globally zero — the
        // isolated column has to be zero against a background that is not.
        var liveMax = 0.0
        for k in 0..<net.branches.count {
            for j in 0..<net.busCount where !isolated.contains(j) {
                liveMax = max(liveMax, abs(try ptdf[BranchID(k), BusID(j)]))
            }
        }
        note("6.1 isolated: \(isolated.count) isolated bus, \(inspected) entries "
             + "inspected, max |PTDF[l,isolated]| = \(worst); "
             + "live-bus max |PTDF| = \(liveMax)")
        XCTAssertEqual(worst, 0.0, "isolated column must be EXACTLY zero")
        XCTAssertGreaterThan(liveMax, 0.1,
                             "the live part of the matrix is ~zero too — this gate "
                             + "would agree with anything")
    }

    // MARK: 6.2 — base flow reconstruction, WITH a shifter case
    //
    // PTDF x injections + the phase-shift terms must reproduce the DC solve.
    // D64 §12/A9: no IEEE case in this repo has a shifter, so run against
    // those alone this gate validates the shift term against nothing.
    func test6_2_baseFlowReconstruction() throws {
        var ranWithShifter = 0
        for fx in try shifters() {
            let net = fx.network()
            let ptdf = try SensitivityEngine().ptdf(net)
            let shift = try PhaseShiftTerms.of(net)

            // PAYLOAD, MEASURED — not "this fixture has a shifter".
            XCTAssertGreaterThan(fx.nShift, 0, "\(fx.name): nshift = 0, empty payload")
            XCTAssertGreaterThan(
                fx.shiftTermMaxFlowDeltaMw, 1e-3,
                "\(fx.name): deleting every phase shift moves base flows by only "
                + "\(fx.shiftTermMaxFlowDeltaMw) MW — the term is numerically "
                + "absent and 6.15's mutation could not fire")
            let shiftMagnitude = shift.branchFlowPu.map(abs).max() ?? 0
            XCTAssertGreaterThan(shiftMagnitude, 1e-6,
                                 "\(fx.name): every pFinj is ~0")
            ranWithShifter += 1

            let flows = try ptdf.completeDCBranchFlows(
                injections: fx.injections(), phaseShift: shift, network: net)

            var worst = 0.0, worstK = -1
            for k in 0..<net.branches.count {
                let want = fx.dc.branchPFromMw[k] / fx.baseMva
                let got = flows[BranchID(k)] ?? .nan
                if abs(got - want) > worst { worst = abs(got - want); worstK = k }
            }
            note("6.2 \(fx.name): nshift=\(fx.nShift), max|shift|="
                 + "\(fx.shiftDegMaxAbs)deg, shift term worth up to "
                 + "\(fx.shiftTermMaxFlowDeltaMw)MW, max|Δflow| = \(worst) pu "
                 + "at branch \(worstK) over \(net.branches.count) branches")
            XCTAssertLessThanOrEqual(worst, 1e-9,
                                     "\(fx.name): reconstruction differs from the DC solve")
        }
        XCTAssertGreaterThanOrEqual(ranWithShifter, 2,
                                    "6.2 must run on at least the two shifter fixtures")
    }

    // MARK: 6.15 — the MUTATION CHECK, which is 6.2's acceptance criterion
    //
    // A9: "The acceptance criterion is a MUTATION TEST, not a passing
    // assertion: zero out the pFinj/pBusInj contribution and confirm both
    // cases go RED. A gate that passes with the term deleted is not testing
    // the term."
    func test6_15_zeroingTheShiftTermTurns6_2Red() throws {
        for fx in try shifters() {
            let net = fx.network()
            let ptdf = try SensitivityEngine().ptdf(net)
            let real = try PhaseShiftTerms.of(net)

            // The mutant keeps the REAL signature and zeroes only the values,
            // so it passes `completeDCBranchFlows`' signature check and the
            // arithmetic is what fails. Worth stating plainly: the signature
            // does NOT protect against a deleted term — only this gate does.
            let mutant = PhaseShiftTerms(
                signature: real.signature,
                branchOrder: real.branchOrder,
                busOrder: real.busOrder,
                branchFlowPu: [Double](repeating: 0, count: real.branchFlowPu.count),
                busInjectionPu: [Double](repeating: 0, count: real.busInjectionPu.count))

            let flows = try ptdf.completeDCBranchFlows(
                injections: fx.injections(), phaseShift: mutant, network: net)
            var worst = 0.0
            for k in 0..<net.branches.count {
                let want = fx.dc.branchPFromMw[k] / fx.baseMva
                worst = max(worst, abs((flows[BranchID(k)] ?? .nan) - want))
            }
            note("6.15 \(fx.name): with pFinj/pBusInj zeroed, max|Δflow| = \(worst) pu "
                 + "(6.2's gate is 1e-9 — exceeded by \(worst / 1e-9)x)")
            XCTAssertGreaterThan(worst, 1e-9,
                                 "\(fx.name): 6.2 still PASSES with the phase-shift term "
                                 + "deleted, so 6.2 is not testing the term")
        }
    }

    // MARK: 6.3 — the OUT-OF-SERVICE arm
    //
    // Split from 3a for the same reason as 6.1's isolated arm: oos = 0 across
    // every fixture, so it would have been an empty payload there.
    func test6_3_outOfServiceDiagonalIsZero() throws {
        let net = BusBranchNetwork(
            baseMVA: 100,
            buses: [.init(type: .slack, baseKv: 138),
                    .init(type: .pq, baseKv: 138),
                    .init(type: .pq, baseKv: 138)],
            branches: [.init(from: 0, to: 1, r: 0.01, x: 0.10),
                       .init(from: 1, to: 2, r: 0.01, x: 0.10),
                       .init(from: 2, to: 0, r: 0.01, x: 0.10),
                       // The payload: a branch that exists in the model and
                       // carries no flow.
                       .init(from: 0, to: 2, r: 0.01, x: 0.10, inService: false)],
            generators: [.init(bus: 0, pPu: 0.5, vSetPu: 1.0)])

        let oos = (0..<net.branches.count).filter { !net.branches[$0].inService }
        XCTAssertEqual(oos.count, 1, "no out-of-service branch — empty payload")

        let f = DistributionFactors.build(net)
        var active = 0
        for k in 0..<net.branches.count {
            let d = f.lodf(monitored: k, outaged: k)
            if net.branches[k].inService { active += 1
                XCTAssertEqual(d, -1.0, "branch \(k): active diagonal must be -1 EXACTLY")
            } else {
                XCTAssertEqual(d, 0.0, "branch \(k): out-of-service diagonal must be 0")
            }
        }
        note("6.3 oos: \(oos.count) out-of-service diagonal == 0, "
             + "\(active) active diagonals == -1 exactly")
        XCTAssertGreaterThan(active, 0, "no active branch to contrast against")
    }

    // MARK: 6.4 — reference-scheme invariance of LODF, on a NON-DEGENERATE w
    //
    // D64 §2's closure risk, stated when the marker was written: "No fixture in
    // the corpus sets slackWeight, so `.distributed` reproduces
    // `.networkDefined` exactly and gate 6.4 would pass while testing nothing."
    // The weight is therefore constructed here AND its non-degeneracy is
    // MEASURED — by showing the two schemes produce DIFFERENT PTDF matrices.
    // LODF's invariance only means something once that is established.
    func test6_4_lodfIsInvariantAcrossReferenceSchemes() throws {
        for name in ["case14", "case39", "case118"] {
            let net = try ReferenceCase.load(name).network()
            let refs = Set((0..<net.busCount).filter { net.buses[$0].type == .slack })

            // D5: the weight must NOT sit entirely on the existing reference
            // set, or `.distributed` collapses onto `.networkDefined`.
            // Participation requires an IN-SERVICE GENERATOR at the bus —
            // `.distributed` validates that on construction (D64 §2), and a
            // participation factor on a bus with nothing to redispatch is
            // physically meaningless. Non-reference generator buses only.
            let genBuses = Set(net.generators.filter { $0.inService }.map { $0.bus })
            let participants = (0..<net.busCount)
                .filter { genBuses.contains($0) && !refs.contains($0)
                          && net.buses[$0].type != .isolated }
                .prefix(4)
            XCTAssertGreaterThanOrEqual(participants.count, 2,
                                        "\(name): too few non-reference buses to build a "
                                        + "non-degenerate participation vector")
            var w: [BusID: Double] = [:]
            for (n, b) in participants.enumerated() { w[BusID(b)] = Double(n + 1) }
            XCTAssertTrue(w.keys.allSatisfy { !refs.contains($0.index) },
                          "\(name): participation sits on the reference set — DEGENERATE, "
                          + "and this gate would compare a scheme against itself")

            let engine = SensitivityEngine()
            let base = try engine.ptdf(net, slack: .networkDefined)
            let dist = try engine.ptdf(net, slack: .distributed(w))

            // NON-DEGENERACY, MEASURED. If this is zero the two schemes are the
            // same scheme and the invariance below is a tautology.
            var ptdfSpread = 0.0
            for k in 0..<net.branches.count {
                for j in 0..<net.busCount {
                    ptdfSpread = max(ptdfSpread,
                                     abs(try base[BranchID(k), BusID(j)]
                                         - dist[BranchID(k), BusID(j)]))
                }
            }
            XCTAssertGreaterThan(ptdfSpread, 1e-6,
                                 "\(name): the two reference schemes produce the SAME PTDF — "
                                 + "the participation vector is degenerate and 6.4 is vacuous")

            // THE CLAIM, AND WHERE IT ACTUALLY LIVES.
            //
            // ⚠️ FINDING (recorded in D64 §2's closure): this gate CANNOT be
            // written through `SensitivityEngine.lodf`. That method takes a
            // `PTDFResult` but uses its factors only when the storage is
            // `.base`; a `.distributed` result is `.dense`, so `lodf` falls
            // through and rebuilds `DistributionFactors` from the network —
            // which is reference-independent BY CONSTRUCTION. Both arms would
            // then be the same fresh build and agree bitwise while testing
            // nothing. That is the tautology mode, and it is invisible unless
            // you read the storage switch.
            //
            // So the LODF is derived HERE from each scheme's own matrix, by
            // the definition
            //     LODF[m,k] = (PTDF[m,f_k] - PTDF[m,t_k]) / (1 - h_k),
            //     h_k       =  PTDF[k,f_k] - PTDF[k,t_k]
            // which is where the invariance actually has content: the
            // post-shift subtracts a per-ROW offset, and that offset CANCELS in
            // every endpoint difference. If it did not cancel, D64 §2's whole
            // justification for implementing `.distributed` as a post-shift
            // rather than a second factorization would be void.
            func lodfFrom(_ m: PTDFResult) throws -> [[Double]] {
                var out = [[Double]](repeating: [], count: net.branches.count)
                for k in 0..<net.branches.count {
                    let br = net.branches[k]
                    let h = try m[BranchID(k), BusID(br.from)]
                           - m[BranchID(k), BusID(br.to)]
                    guard abs(1 - h) > 1e-9 else { continue }   // islanding
                    out[k] = try (0..<net.branches.count).map {
                        (try m[BranchID($0), BusID(br.from)]
                         - m[BranchID($0), BusID(br.to)]) / (1 - h)
                    }
                }
                return out
            }

            // A third scheme, so invariance is asserted across a FAMILY rather
            // than between two points (D64 §2's `.uniformlyDistributed` case).
            let single = try engine.ptdf(net, slack: .uniformlyDistributed(
                [BusID(participants.first ?? 0)]))

            let lBase = try lodfFrom(base)
            let lDist = try lodfFrom(dist)
            let lSingle = try lodfFrom(single)

            var worst = 0.0
            var compared = 0
            for k in 0..<net.branches.count where !lBase[k].isEmpty {
                for m in 0..<net.branches.count {
                    worst = max(worst, abs(lBase[k][m] - lDist[k][m]))
                    worst = max(worst, abs(lBase[k][m] - lSingle[k][m]))
                    compared += 2
                }
            }
            // WRONG-QUANTITY GUARD: an all-zero LODF agrees with an all-zero
            // LODF. Pin that the factors being compared are actually large.
            let lodfMax = lBase.flatMap { $0 }.map(abs).max() ?? 0
            note("6.4 \(name): w over \(w.count) non-reference buses, PTDF spread "
                 + "between schemes = \(ptdfSpread), max|LODF| = \(lodfMax), "
                 + "LODF max|Δ| across 3 schemes = \(worst) over \(compared) comparisons")
            XCTAssertGreaterThan(compared, 0, "\(name): no non-islanding pair compared")
            XCTAssertGreaterThan(lodfMax, 0.1,
                                 "\(name): the LODFs being compared are ~zero — "
                                 + "this gate would agree with anything")
            XCTAssertLessThanOrEqual(worst, 1e-10,
                                     "\(name): LODF moved with the reference scheme")
        }
    }

    // MARK: 6.7 — external PTDF reference, from pandapower's makePTDF
    //
    // Independent of 6.5: that validates LODF against a re-solve computed by
    // the SAME DC machinery, so it cannot see an error the two share.
    func test6_7_ptdfAgreesWithExternalReference() throws {
        let ref = try load(PTDFReference.self, "ptdf")
        XCTAssertEqual(ref.pandapowerVersion, "3.2.1",
                       "oracle version moved (D68 §3)")
        for name in ["case14", "case39", "case118"] {
            let expect = try XCTUnwrap(ref.cases[name], "\(name) missing from ptdf.json")
            let net = try ReferenceCase.load(name).network()
            XCTAssertEqual(expect.nBus, net.busCount)
            XCTAssertEqual(expect.nBranch, net.branches.count)

            // Ask for the SAME scheme makePTDF used, rather than comparing two
            // conventions and calling the difference an error.
            let mine = try SensitivityEngine().ptdf(net, slack: .networkDefined)

            var worst = 0.0, worstAt = (-1, -1)
            var nonzero = 0, significant = 0
            for k in 0..<net.branches.count {
                for j in 0..<net.busCount {
                    let a = try mine[BranchID(k), BusID(j)]
                    let b = expect.ptdf[k][j]
                    if abs(a - b) > worst { worst = abs(a - b); worstAt = (k, j) }
                    if a != 0 { nonzero += 1 }
                    if abs(a) > 1e-9 { significant += 1 }
                }
            }
            // WRONG-QUANTITY GUARD, and it took two attempts to point at the
            // right quantity. An all-zero matrix agrees with an all-zero matrix,
            // so the gate must pin that real sensitivities are present — but
            // the OBVIOUS pin, raw nonzero count, compares numerical litter:
            // makePTDF emits hundreds of entries down to |H| ~ 1e-22 (measured:
            // case118 has 2,425 entries below 1e-14 and its smallest nonzero is
            // 1.9e-22) where this implementation returns exact 0.0. Counting
            // those made mine look 197 entries short of the oracle for a
            // difference no decision depends on.
            //
            // So the pin is entries at or above the gate's OWN tolerance, and
            // there it is an EQUALITY, not a fraction — the two implementations
            // must agree on which sensitivities are real.
            XCTAssertEqual(significant, expect.significantCount,
                           "\(name): \(significant) entries above 1e-9 against the "
                           + "oracle's \(expect.significantCount) — the two disagree about "
                           + "which sensitivities exist, not merely about their values")
            XCTAssertGreaterThan(significant, net.branches.count,
                                 "\(name): fewer significant entries than branches — "
                                 + "this agreement would be trivial")
            note("6.7 \(name): \(expect.nBranch)x\(expect.nBus), \(significant) significant "
                 + "(oracle \(expect.significantCount)), \(nonzero) raw nonzero "
                 + "(oracle \(expect.nonzeroCount) — the difference is dust below 1e-14), "
                 + "oracle max|H| \(expect.maxAbs), "
                 + "max|Δ| = \(worst) at branch \(worstAt.0)/bus \(worstAt.1)")
            XCTAssertLessThanOrEqual(worst, 1e-9,
                                     "\(name): PTDF disagrees with makePTDF")
        }
    }

    // MARK: 6.18 — the three connectivity guards, each EXERCISED
    //
    // 1b's coverage table showed isolated buses, out-of-service branches and
    // self-loops exercised by no fixture, while `bridgeBranches` guards all
    // three explicitly. D64 §12: the gate asserts what makes each guard
    // CORRECT, not merely that it is reached.
    func test6_18_theThreeConnectivityGuards() throws {
        // (a) An OUT-OF-SERVICE branch is excluded from the bridge set because
        //     its flow is already zero — which is what makes A3's zero-column
        //     convention coherent rather than a wart.
        do {
            let net = BusBranchNetwork(
                baseMVA: 100,
                buses: [.init(type: .slack, baseKv: 138), .init(type: .pq, baseKv: 138)],
                branches: [.init(from: 0, to: 1, r: 0.01, x: 0.10),
                           .init(from: 0, to: 1, r: 0.01, x: 0.10, inService: false)],
                generators: [.init(bus: 0, pPu: 0.5, vSetPu: 1.0)])
            let bridges = NetworkConnectivity.bridgeBranches(net)
            XCTAssertFalse(net.branches[1].inService, "payload: no oos branch")
            XCTAssertFalse(bridges[1],
                           "an out-of-service branch carries no flow and cannot be a bridge")
            XCTAssertTrue(bridges[0],
                          "the ONLY in-service branch must be a bridge — without this the "
                          + "case above passes because nothing is a bridge")
            note("6.18(a) oos: branch 1 out of service -> not a bridge; "
                 + "branch 0 in service -> bridge. Both directions asserted.")
        }

        // (b) A SELF-LOOP can never disconnect anything, so it is never a bridge.
        do {
            let net = BusBranchNetwork(
                baseMVA: 100,
                buses: [.init(type: .slack, baseKv: 138), .init(type: .pq, baseKv: 138)],
                branches: [.init(from: 0, to: 1, r: 0.01, x: 0.10),
                           .init(from: 1, to: 1, r: 0.01, x: 0.10)],
                generators: [.init(bus: 0, pPu: 0.5, vSetPu: 1.0)])
            let bridges = NetworkConnectivity.bridgeBranches(net)
            XCTAssertEqual(net.branches[1].from, net.branches[1].to, "payload: no self-loop")
            XCTAssertFalse(bridges[1], "a self-loop can never disconnect anything")
            XCTAssertTrue(bridges[0], "the real bridge must still be found")
            note("6.18(b) self-loop: branch 1 (1->1) -> not a bridge; branch 0 -> bridge.")
        }

        // (c) An ISOLATED bus is not part of the live graph, so no branch
        //     incident on it can be a bridge OF THAT GRAPH.
        do {
            let net = BusBranchNetwork(
                baseMVA: 100,
                buses: [.init(type: .slack, baseKv: 138),
                        .init(type: .pq, baseKv: 138),
                        .init(type: .isolated, baseKv: 138)],
                branches: [.init(from: 0, to: 1, r: 0.01, x: 0.10),
                           .init(from: 1, to: 2, r: 0.01, x: 0.10)],
                generators: [.init(bus: 0, pPu: 0.5, vSetPu: 1.0)])
            let bridges = NetworkConnectivity.bridgeBranches(net)
            XCTAssertEqual(net.buses[2].type, .isolated, "payload: no isolated bus")
            XCTAssertFalse(bridges[1],
                           "a branch incident on an isolated bus is not a bridge of the "
                           + "live graph")
            XCTAssertTrue(bridges[0], "the live bridge must still be found")
            note("6.18(c) isolated: branch 1 (to isolated bus 2) -> not a bridge; "
                 + "branch 0 -> bridge.")
        }
    }
}
