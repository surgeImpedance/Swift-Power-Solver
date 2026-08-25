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
/// EXACTLY on pandapower 3.2.1, so a hash mismatch means the code moved, not
/// the fixture. Absent the fixtures this test SKIPS — which is how a gate named
/// never-regress in CLAUDE.md came to be silently unrunnable in a fresh
/// checkout.
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
        guard let paths = ProcessInfo.processInfo.environment["SPS_FACTORS_CASES"] else {
            XCTFail("""
                SPS_FACTORS_CASES unset — the bit-identity gate cannot run.
                This is a FAILURE, not a skip: an absent fixture must not read \
                as a pass. Regenerate (about 30 s) and re-run:

                  python Tools/dump_factors_fixture.py --out-dir /tmp \
                      case300 case1354pegase case9241pegase
                  SPS_FACTORS_CASES=/tmp/factors_case300.json,\
                /tmp/factors_case1354.json,/tmp/factors_case9241.json \
                      swift test -c release --filter FactorsIdentityTests
                """)
            return
        }
        let decoder = JSONDecoder()
        decoder.keyDecodingStrategy = .convertFromSnakeCase

        for path in paths.split(separator: ",").map(String.init) {
            let fixture = try decoder.decode(NetworkFixture.self,
                                             from: Data(contentsOf: URL(fileURLWithPath: path)))
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
