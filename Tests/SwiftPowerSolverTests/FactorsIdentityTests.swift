import CryptoKit
import Foundation
import XCTest
@testable import SwiftPowerSolver

/// Bit-identity gate for `DistributionFactors.build`, driven by external
/// network fixtures (they are 1–20 MB and have no business in the bundle):
///
///   SPS_FACTORS_CASES=/abs/factors_case300.json,/abs/factors_case1354.json,…
///   swift test -c release --filter FactorsIdentityTests
///
/// Generate the fixtures with `Tools/dump_factors_fixture.py` (one command;
/// they are regenerable and deliberately not committed):
///
///     python Tools/dump_factors_fixture.py --out-dir /tmp case300 case1354pegase case9241pegase
///
/// Verified 2026-08-24: regenerated fixtures reproduce all three goldens
/// EXACTLY on pandapower 3.2.1. That version is now ASSERTED below rather than
/// asserted in this comment (D68 §3): the fixture carries `pandapower_version`
/// and the sentence above was a quantitative claim with no control behind it,
/// so a fixture from another pandapower would have moved a hash and read as
/// "the code moved". Absent the fixtures this test USED TO SKIP — which is how
/// a gate named never-regress in CLAUDE.md came to be silently unrunnable in a
/// fresh checkout. It now FAILS; see `testBuildIdentity`.
///
/// ⚠️ ACCELERATE EXPOSURE — this gate is one of the two that carry it.
/// DECISIONS.md D68: the goldens below are a claim about a closed third-party
/// implementation (`SparseFactor`/`SparseSolve`). If they move on a clean tree,
/// check `sw_vers` and `pandapower.__version__` against D68 §4 BEFORE hunting a
/// code defect — no diff can show that cause.
///
/// Each fixture is the network-only portion of tools/dump_reference.py output
/// (buses/branches/gens/base — the factors build never reads a solution).
/// The test builds the factors, hashes the FULL ptdf and lodf matrices plus
/// the islanding vector, prints one `FACTORS …` line per case, and compares
/// against the goldens below.
///
/// GOLDENS are the output of the ORIGINAL algorithm at commit 24895bc,
/// recorded before any optimization of the build. Every subsequent change to
/// the build (CSC radix assembly, batched multi-RHS solve, loop reordering,
/// parallel fill) must reproduce them EXACTLY — bit identity is the
/// acceptance bar for this code, not approximate agreement. Hashes are
/// arm64/macOS observations; a different ISA would need its own goldens.
final class FactorsIdentityTests: XCTestCase {

    /// SHA-256 over the row-major matrix bytes, read through the PUBLIC
    /// accessors so the hash is independent of internal storage layout.
    static let golden: [String: (ptdf: String, lodf: String, islanding: String)] = [
        // Recorded 2026-08-13 at 24895bc (pre-optimization), macOS arm64.
        "case300": (ptdf: "c7a532cf786cd021", lodf: "b615538c61ad7d78", islanding: "69fc9f47c69e7f0a"),
        "case1354": (ptdf: "b217d6c62ad20982", lodf: "c7544e4b3cd16ebe", islanding: "4ca9ad2e58a532c3"),
        "case9241": (ptdf: "62bb035e5223ad58", lodf: "eaed1f025bd042b4", islanding: "61526d6902be8eea"),
    ]

    /// The oracle version the goldens above were captured against, and the only
    /// one they are claims about (D68 §3). The golden path runs through TWO
    /// closed implementations — pandapower on the way in, Accelerate on the way
    /// through — and a change in either moves a hash with a clean tree. This
    /// pins the half that can be pinned.
    static let goldenPandapowerVersion = "3.2.1"

    /// The three scale-fixture paths. `SPS_FACTORS_CASES` still overrides
    /// (comma-separated, unchanged semantics); otherwise the COMMITTED copies
    /// in `FactorsFixtures/` are used, so a bare clone runs every gate
    /// (fixture home ruled 2026-08-29). Missing committed files return an
    /// empty array — callers FAIL on that, never skip (C3): with the fixtures
    /// committed, absence is repo corruption, not a fresh-clone state.
    static func factorsCasePaths() -> [String] {
        if let env = ProcessInfo.processInfo.environment["SPS_FACTORS_CASES"] {
            return env.split(separator: ",").map(String.init)
        }
        let names = ["factors_case300", "factors_case1354", "factors_case9241"]
        let urls = names.compactMap {
            Bundle.module.url(forResource: $0, withExtension: "json",
                              subdirectory: "FactorsFixtures")
        }
        return urls.count == names.count ? urls.map(\.path) : []
    }

    // Visibility widened (private -> internal) at unit 0 so the four probe
    // decoders could be deleted and repointed here. NOTHING ELSE about this
    // type changed: it is ON the golden path (fixture -> decode -> build ->
    // hash), and P1 chose it as the survivor precisely so that path stays
    // LITERALLY unchanged rather than merely equivalent.
    struct NetworkFixture: Decodable {
        struct B: Decodable { var i: Int; var type: Int; var pdMw, qdMvar, gsMw, bsMvar, baseKv: Double }
        struct R: Decodable { var f, t: Int; var r, x, b, g, tap, shiftDeg: Double; var status: Int }
        struct G: Decodable { var bus: Int; var pgMw, qmaxMvar, qminMvar, vgPu, vaDeg: Double; var status: Int }
        var name: String
        var baseMva: Double
        /// Provenance, added at D68 §3. Optional so fixtures written before the
        /// dumper emitted it still decode; `testBuildIdentity` requires it.
        var pandapowerVersion: String?
        var buses: [B]
        var branches: [R]
        var gens: [G]

        func network() -> BusBranchNetwork {
            BusBranchNetwork(
                baseMVA: baseMva,
                buses: buses.map {
                    BusBranchNetwork.Bus(type: BusBranchNetwork.BusType(rawValue: $0.type) ?? .pq,
                                         baseKv: $0.baseKv,
                                         pLoadPu: $0.pdMw / baseMva, qLoadPu: $0.qdMvar / baseMva,
                                         gsPu: $0.gsMw / baseMva, bsPu: $0.bsMvar / baseMva)
                },
                branches: branches.map {
                    BusBranchNetwork.Branch(from: $0.f, to: $0.t, r: $0.r, x: $0.x, b: $0.b, g: $0.g,
                                            tap: $0.tap <= 0 ? 1.0 : $0.tap,
                                            shiftRad: $0.shiftDeg * .pi / 180,
                                            inService: $0.status == 1)
                },
                generators: gens.map {
                    BusBranchNetwork.Generator(bus: $0.bus, pPu: $0.pgMw / baseMva, vSetPu: $0.vgPu,
                                               vaRefRad: $0.vaDeg * .pi / 180,
                                               qMinPu: $0.qminMvar / baseMva, qMaxPu: $0.qmaxMvar / baseMva,
                                               inService: $0.status == 1)
                })
        }
    }

    func testBuildIdentity() throws {
        // FAILS, does not skip (C3). A control that quietly abstains when its
        // inputs are missing has the failure mode of the thing it was built to
        // prevent: this gate is named never-regress in CLAUDE.md and was
        // silently unrunnable in a fresh checkout for an unknown span, because
        // a skip is indistinguishable from a pass in every summary view.
        let paths = Self.factorsCasePaths()
        guard !paths.isEmpty else {
            XCTFail("""
                No scale fixtures: SPS_FACTORS_CASES is unset AND the committed \
                FactorsFixtures/ copies are missing from the test bundle — \
                the bit-identity gate cannot run. This is a FAILURE, not a \
                skip: an absent fixture must not read as a pass. Regenerate:

                  python Tools/dump_factors_fixture.py \
                      --out-dir Tests/SwiftPowerSolverTests/FactorsFixtures \
                      case300 case1354pegase case9241pegase
                """)
            return
        }
        let decoder = JSONDecoder()
        decoder.keyDecodingStrategy = .convertFromSnakeCase

        for path in paths {
            let fixture = try decoder.decode(NetworkFixture.self,
                                             from: Data(contentsOf: URL(fileURLWithPath: path)))

            // D68 §3. An unprovenanced or differently-provenanced fixture must
            // FAIL NAMING THE VERSION, not sail through and move a hash: the
            // two failures are indistinguishable in every summary view, and
            // only one of them is a code defect.
            guard let ppVersion = fixture.pandapowerVersion else {
                XCTFail("""
                    \(fixture.name): fixture carries no `pandapower_version`.
                    The goldens are a claim about pandapower \
                    \(Self.goldenPandapowerVersion) (D68 §3); a fixture that \
                    cannot state its own provenance must not read as a pass. \
                    Regenerate with Tools/dump_factors_fixture.py.
                    """)
                continue
            }
            XCTAssertEqual(ppVersion, Self.goldenPandapowerVersion,
                           "\(fixture.name): fixture built on pandapower \(ppVersion), "
                           + "goldens recorded on \(Self.goldenPandapowerVersion). "
                           + "A moved hash here is the ORACLE moving, not the code (D68 §3).")

            let net = fixture.network()

            let t0 = ContinuousClock.now
            let factors = DistributionFactors.build(net)
            let dt = ContinuousClock.now - t0
            let ms = Double(dt.components.seconds) * 1000
                   + Double(dt.components.attoseconds) / 1e15

            let n = factors.busCount, nbr = factors.branchCount
            var buf = [Double]()
            buf.reserveCapacity(max(nbr * n, nbr * nbr))

            buf.removeAll(keepingCapacity: true)
            for k in 0..<nbr { for j in 0..<n { buf.append(factors.ptdf(branch: k, bus: j)) } }
            let ptdfHash = Self.sha(buf)

            buf.removeAll(keepingCapacity: true)
            for m in 0..<nbr { for k in 0..<nbr { buf.append(factors.lodf(monitored: m, outaged: k)) } }
            let lodfHash = Self.sha(buf)

            let islHash = Self.sha((0..<nbr).map { factors.isIslanding(outage: $0) ? 1.0 : 0.0 })

            print("FACTORS \(fixture.name) build=\(String(format: "%.1f", ms))ms "
                  + "ptdf=\(ptdfHash) lodf=\(lodfHash) islanding=\(islHash)")

            guard let want = Self.golden[fixture.name] else { continue }
            if want.ptdf == "RECORD" { continue }   // record mode: print only
            XCTAssertEqual(ptdfHash, want.ptdf, "\(fixture.name): PTDF diverged from 24895bc")
            XCTAssertEqual(lodfHash, want.lodf, "\(fixture.name): LODF diverged from 24895bc")
            XCTAssertEqual(islHash, want.islanding, "\(fixture.name): islanding set diverged")
        }
    }

    private static func sha(_ values: [Double]) -> String {
        values.withUnsafeBytes { raw in
            SHA256.hash(data: raw).map { String(format: "%02x", $0) }.joined().prefix(16).description
        }
    }
}
