import CryptoKit
import Foundation
import XCTest
@testable import SwiftPowerSolver

/// D2: does connectivity-based islanding agree with the `h`-based
/// classification? Since D73's closure (2026-08-29) the SHIPPED classifier is
/// structural — `FactorsIdentityTests`' islanding hashes pin the structural
/// vector now, and they held BITWISE across the switch because the two
/// methods agree on every corpus branch. The h-based side of this comparison
/// is a local REFERENCE implementation of the retired rule, kept so the
/// ratcheted `differing` counts keep measuring where the two methods part on
/// real data (the C2 regime), rather than comparing one array to itself.
///
/// D42 measured coextensivity app-side on 135 branches. The pegase cases carry
/// 16,049 branches and a far wider reactance spread, which is exactly where the
/// `h`-based method should be expected to break first.
final class ConnectivityIslandingTests: XCTestCase {

    private func note(_ m: String) {
        FileHandle.standardError.write(Data((m + "\n").utf8))
    }

    private static func sha(_ values: [Double]) -> String {
        values.withUnsafeBytes { raw in
            SHA256.hash(data: raw).map { String(format: "%02x", $0) }.joined().prefix(16).description
        }
    }

    /// RATCHETED BASELINE for the scale comparison — the shipped-vs-h-based
    /// `differing` count per case, pinned 2026-08-29 from a measured run over
    /// the committed fixtures (that run's margins: worst bridge |1−h| =
    /// 2.5e-13 against closest non-bridge 5.3e-4 on case9241 — not close).
    ///
    /// The scale test FAILS on a RISE. A FALL is a finding to report, never
    /// an auto-re-baseline. Re-baselining in EITHER direction requires a
    /// written justification recorded HERE, beside the pinned values — the
    /// same standing as the AG2 ratchet's corpus-size record. Hard equality
    /// was deliberately declined: C2 measured that the two classifiers can
    /// diverge legitimately at conditioning extremes, so a rise can be a
    /// CORRECT result — the ratchet's job is to make it a visible one
    /// instead of a note scrolling past in a log.
    private static let pinnedDiffering: [String: Int] = [
        "case300": 0, "case1354": 0, "case9241": 0,
    ]

    @discardableResult
    private func compare(_ name: String, _ net: BusBranchNetwork) -> Int {
        let factors = DistributionFactors.build(net)
        let nbr = net.branches.count
        // Since D73's closure (2026-08-29) `factors.isIslanding` IS the
        // structural array, so comparing it to `bridgeBranches` would be a
        // tautology — the same computation twice. The h-based side is
        // therefore computed HERE as a REFERENCE implementation (the exact
        // pre-D73 rule: in service, |1 − h| < 1e-9), which keeps this
        // comparison's subject — where the h-method and structure part on
        // real data — and keeps the ratchet a control rather than a mirror.
        let hBased = (0..<nbr).map { k -> Bool in
            let br = net.branches[k]
            guard br.inService, br.from != br.to, br.x != 0 else { return false }
            let h = factors.ptdf(branch: k, bus: br.from)
                  - factors.ptdf(branch: k, bus: br.to)
            return abs(1 - h) < 1e-9
        }
        // GATE 6.16 — the SHIPPED connectivity, not a probe (M1/D3).
        //
        // D2 established coextensivity using a probe Tarjan in this file. That
        // validated the classification CONCEPT, not the artifact that ships, so
        // unit 1b ran a THREE-WAY comparison — shipped vs probe vs h-based — on
        // all six cases while the probe still existed, then deleted it. Result:
        // every hash identical, shipped-vs-h = 0 AND shipped-vs-probe = 0 on
        // all six (case14/39/118/300/1354/9241, 16,049 branches at case9241).
        // D2's finding therefore now covers the shipped code, and the probe is
        // gone — D3 has ONE connectivity implementation in the package.
        // (An earlier paragraph here argued the probe must outlive the delete;
        // that was unit 1b's SEQUENCING rationale — compare three ways FIRST,
        // then delete — and it read as present-tense long after the delete
        // landed. Removed 2026-08-29; the history lives in `b8a956b`.)
        let structural = NetworkConnectivity.bridgeBranches(net)   // SHIPPED

        let hHash = Self.sha(hBased.map { $0 ? 1.0 : 0.0 })
        let cHash = Self.sha(structural.map { $0 ? 1.0 : 0.0 })
        let differing = (0..<nbr).filter { hBased[$0] != structural[$0] }

        note("D2 \(name): branches=\(nbr) h-based=\(hBased.filter { $0 }.count) "
             + "connectivity=\(structural.filter { $0 }.count) "
             + "hHash=\(hHash) shippedHash=\(cHash) differing=\(differing.count)")

        for k in differing.prefix(20) {
            let br = net.branches[k]
            let h = factors.ptdf(branch: k, bus: br.from) - factors.ptdf(branch: k, bus: br.to)
            note(String(format: "D2 %@:   branch %d (%d->%d) x=%.6g tap=%.4g  "
                        + "h-based=%@ connectivity=%@  |1-h| = %.6e",
                        name, k, br.from, br.to, br.x, br.tap,
                        hBased[k] ? "islanding" : "meshed",
                        structural[k] ? "BRIDGE" : "meshed", abs(1 - h)))
        }
        if differing.count > 20 {
            note("D2 \(name):   ... and \(differing.count - 20) more")
        }

        // How much headroom does the h-based method actually have on this
        // network? The classifier fires at |1-h| < 1e-9, so the margin is the
        // gap between the WORST bridge (which should sit at ~0) and the
        // CLOSEST non-bridge (which must stay well above 1e-9). C2 showed the
        // method breaking on constructed networks; this says whether real ones
        // come anywhere near that regime.
        var worstBridgeDev = 0.0, closestNonBridgeDev = Double.infinity
        var closestNonBridge = -1
        for k in 0..<nbr where net.branches[k].inService {
            let br = net.branches[k]
            let h = factors.ptdf(branch: k, bus: br.from) - factors.ptdf(branch: k, bus: br.to)
            let dev = abs(1 - h)
            if structural[k] {
                worstBridgeDev = max(worstBridgeDev, dev)
            } else if dev < closestNonBridgeDev {
                closestNonBridgeDev = dev
                closestNonBridge = k
            }
        }
        let xs = net.branches.filter { $0.inService && $0.x != 0 }.map { abs($0.x) }
        let spread = (xs.max() ?? 0) / (xs.min() ?? 1)
        note(String(format: "D2 %@:   margin — worst bridge |1-h| = %.3e, closest "
                    + "non-bridge |1-h| = %.3e (branch %d), epsilon = 1e-9, "
                    + "x spread = %.3e",
                    name, worstBridgeDev, closestNonBridgeDev, closestNonBridge, spread))
        return differing.count
    }

    /// STRUCTURAL COVERAGE (J1 mode 1, applied per structural case rather than
    /// per gate). `NetworkConnectivity` is new code whose first real exercise is
    /// unit 1b. Six cases and 16,049 branches is strong evidence about the
    /// topologies that happen to be in the corpus and SILENT about the rest, so
    /// this reports which structural cases any fixture actually witnesses.
    ///
    /// An unwitnessed case is NOT a defect — it is a scope statement, recorded
    /// so that "6.16 green" is not later read as more than it is.
    func testStructuralCoverageOfTheFixtureCorpus() throws {
        func report(_ name: String, _ net: BusBranchNetwork) {
            let live = net.buses.map { $0.type != .isolated }
            let isolated = net.buses.filter { $0.type == .isolated }.count
            let oos = net.branches.filter { !$0.inService }.count
            let selfLoops = net.branches.filter { $0.from == $0.to }.count
            var pairs: [String: Int] = [:]
            for b in net.branches where b.inService && b.from != b.to {
                pairs["\(min(b.from, b.to))-\(max(b.from, b.to))", default: 0] += 1
            }
            let parallel = pairs.values.filter { $0 > 1 }.count
            var visited = Set<Int>(); var islands = 0; var slackless = 0
            for v in 0..<net.busCount where live[v] && !visited.contains(v) {
                let comp = NetworkConnectivity.componentReachable(from: v, in: net, live: live)
                visited.formUnion(comp); islands += 1
                if !comp.contains(where: { net.buses[$0].type == .slack }) { slackless += 1 }
            }
            note("1b COVERAGE \(name): isolated=\(isolated) oos=\(oos) "
                 + "selfLoops=\(selfLoops) parallelPairs=\(parallel) "
                 + "islands=\(islands) slacklessIslands=\(slackless)")
        }
        for n in ["case14", "case39", "case118"] {
            report(n, try ReferenceCase.load(n).network())
        }
        let d = JSONDecoder(); d.keyDecodingStrategy = .convertFromSnakeCase
        for p in FactorsIdentityTests.factorsCasePaths() {
            let f = try d.decode(FactorsIdentityTests.NetworkFixture.self,
                                 from: Data(contentsOf: URL(fileURLWithPath: p)))
            report(f.name, f.network())
        }
        report("twoIslandOneSlack", BusBranchNetwork(
            baseMVA: 100,
            buses: [.init(type: .slack, baseKv: 138), .init(type: .pq, baseKv: 138),
                    .init(type: .pq, baseKv: 138), .init(type: .pq, baseKv: 138)],
            branches: [.init(from: 0, to: 1, r: 0.01, x: 0.10),
                       .init(from: 2, to: 3, r: 0.01, x: 0.10)],
            generators: [.init(bus: 0, pPu: 1.0, vSetPu: 1.0)]))
    }

    func testReferenceCasesAgree() throws {
        for name in ["case14", "case39", "case118"] {
            compare(name, try ReferenceCase.load(name).network())
        }
    }

    func testPegaseScaleFixtures() throws {
        let paths = FactorsIdentityTests.factorsCasePaths()
        guard !paths.isEmpty else {
            XCTFail("No scale fixtures (env unset AND committed copies missing) "
                    + "— D2 cannot compare at scale. "
                    + "Regenerate with Tools/dump_factors_fixture.py")
            return
        }
        let decoder = JSONDecoder()
        decoder.keyDecodingStrategy = .convertFromSnakeCase
        for path in paths {
            let f = try decoder.decode(FactorsIdentityTests.NetworkFixture.self,
                                       from: Data(contentsOf: URL(fileURLWithPath: path)))
            let differing = compare(f.name, f.network())

            // The ratchet (see `pinnedDiffering`): a rise FAILS; a fall is a
            // reported finding, never an auto-re-baseline.
            guard let pinned = Self.pinnedDiffering[f.name] else {
                XCTFail("\(f.name): no pinned `differing` baseline — pin one "
                        + "(with its justification) before this case joins the scale set")
                continue
            }
            XCTAssertLessThanOrEqual(
                differing, pinned,
                "\(f.name): shipped-vs-h `differing` ROSE above the pinned \(pinned). "
                + "A rise can be legitimate (C2's conditioning extremes) but is never "
                + "silent — re-baselining requires a written justification beside "
                + "`pinnedDiffering`.")
            if differing < pinned {
                note("D2 \(f.name): differing=\(differing) FELL below the pinned "
                     + "\(pinned) — a FINDING to report, not an auto-re-baseline")
            }
        }
    }
}
