import CryptoKit
import Foundation
import XCTest
@testable import SwiftPowerSolver

/// §5's refactor gate: N-1 output captured BEFORE the sensitivity promotion,
/// so the post-refactor path can be shown to reproduce it rather than asserted
/// to.
///
/// Stored as a digest plus human-readable summary counts, not as the full
/// monitored x outaged product: case118 alone is 186 x 186 doubles, and a
/// checked-in table that large makes a bare clone expensive for no extra
/// power — the digest already fails on a single changed bit. The summary is
/// what tells a reader WHICH case moved when it does.
///
/// Recapture (deliberate act, never automatic):
///
///     SPS_CAPTURE_N1_GOLDEN=1 swift test -c release --filter N1GoldenTests
///
/// A recapture that changes a digest must be justified in the commit message
/// against D64 §1 — the arithmetic is not supposed to move.
final class N1GoldenTests: XCTestCase {

    private static let cases = ["case14", "case39", "case118"]

    /// Digests recorded 2026-08-24 at SwiftPowerSolver `abb5247`, BEFORE any
    /// part of the sensitivity promotion landed. macOS arm64.
    private static let golden: [String: String] = [
        "case14": "f1684c9c3c8162f4",
        "case39": "e0fd55f56e2c4c07",
        "case118": "c0e11ed376da3c7b",
    ]

    private struct Summary: Codable, Equatable {
        var monitoredCount: Int
        var caseCount: Int
        var solved: Int
        var islanding: Int
        var outOfService: Int
        var violations: Int
        var worstLoading: Double
        var worstLoadingOutage: Int
        var digest: String
    }

    /// Canonical serialization: every field that distinguishes one screening
    /// from another, in a fixed order, doubles by bit pattern. Anything the
    /// digest does not cover is a hole in the gate, so it covers all of it.
    private func summarize(_ s: ContingencyScreening) -> Summary {
        var hasher = SHA256()
        func absorb(_ v: Int) { withUnsafeBytes(of: Int64(v).littleEndian) { hasher.update(bufferPointer: $0) } }
        func absorb(_ v: Double) { withUnsafeBytes(of: v.bitPattern.littleEndian) { hasher.update(bufferPointer: $0) } }

        absorb(s.monitoredBranches.count)
        for m in s.monitoredBranches { absorb(m) }
        absorb(s.cases.count)

        var solved = 0, islanding = 0, oos = 0, violations = 0
        var worst = 0.0, worstOutage = -1
        for c in s.cases {
            absorb(c.outagedBranch)
            switch c.outcome {
            case .solved: absorb(0); solved += 1
            case .islandsNetwork: absorb(1); islanding += 1
            case .branchOutOfService: absorb(2); oos += 1
            }
            absorb(c.postFlowsPu.count)
            for f in c.postFlowsPu { absorb(f) }
            absorb(c.violations.count)
            for v in c.violations {
                absorb(v.monitoredBranch); absorb(v.postFlowPu)
                absorb(v.ratingMva); absorb(v.loading)
                violations += 1
                if v.loading > worst { worst = v.loading; worstOutage = c.outagedBranch }
            }
        }
        let digest = hasher.finalize().compactMap { String(format: "%02x", $0) }
            .joined().prefix(16)
        return Summary(monitoredCount: s.monitoredBranches.count,
                       caseCount: s.cases.count, solved: solved,
                       islanding: islanding, outOfService: oos,
                       violations: violations, worstLoading: worst,
                       worstLoadingOutage: worstOutage, digest: String(digest))
    }

    /// `ReferenceCase.network()` leaves `ratingMva` nil on every branch, so a
    /// screening over it can never report a violation — the violation half of
    /// the digest would be an EMPTY PAYLOAD, and this gate would pass whether
    /// or not the violation path worked at all (the CLAUDE.md test: would this
    /// pass if the payload were empty?).
    ///
    /// So the golden fixture assigns ratings deterministically from the base
    /// DC flow: `rating = |Pf|·baseMVA·1.05 + 1.0` MVA, which puts every loaded
    /// branch just under 95% of its rating in the base case and makes any
    /// contingency that raises a flow by more than ~5% a reported violation.
    /// Unloaded branches get a 1.0 MVA floor rather than nil, so they too can
    /// be violated rather than being silently unratable.
    private func ratedNetwork(_ name: String) throws -> BusBranchNetwork {
        var net = try ReferenceCase.load(name).network()
        let base = DCPowerFlowSolver().solve(net)
        XCTAssertTrue(base.converged, "\(name): base DC solve failed")
        for k in 0..<net.branches.count {
            let mw = abs(base.branchFlows[k].pFromPu) * net.baseMVA
            net.branches[k].ratingMva = mw * 1.05 + 1.0
        }
        return net
    }

    private func screen(_ name: String) throws -> Summary {
        let net = try ratedNetwork(name)
        let base = DCPowerFlowSolver().solve(net)
        XCTAssertTrue(base.converged, "\(name): base DC solve failed")
        let screening = N1ContingencyAnalyzer().screen(net, base: base,
                                                       options: ContingencyScreeningOptions())
        return summarize(screening)
    }

    func testN1OutputMatchesPreRefactorGolden() throws {
        let capturing = ProcessInfo.processInfo.environment["SPS_CAPTURE_N1_GOLDEN"] == "1"
        var captured: [String: Summary] = [:]

        for name in Self.cases {
            let s = try screen(name)
            captured[name] = s
            FileHandle.standardError.write(Data((
                "N1 GOLDEN \(name): digest=\(s.digest) monitored=\(s.monitoredCount) "
                + "cases=\(s.caseCount) solved=\(s.solved) islanding=\(s.islanding) "
                + "oos=\(s.outOfService) violations=\(s.violations) "
                + "worstLoading=\(s.worstLoading) @outage=\(s.worstLoadingOutage)\n").utf8))
        }

        if capturing {
            let lines = Self.cases.map { "        \"\($0)\": \"\(captured[$0]!.digest)\"," }
            FileHandle.standardError.write(Data((
                "\nN1 GOLDEN — paste into `golden`:\n"
                + lines.joined(separator: "\n") + "\n\n").utf8))
            return
        }

        for name in Self.cases {
            let expected = Self.golden[name] ?? ""
            guard !expected.isEmpty else {
                XCTFail("\(name): no golden recorded — run with SPS_CAPTURE_N1_GOLDEN=1")
                continue
            }
            XCTAssertEqual(captured[name]!.digest, expected,
                           "\(name): N-1 output changed against the pre-refactor golden")
        }
    }
}
