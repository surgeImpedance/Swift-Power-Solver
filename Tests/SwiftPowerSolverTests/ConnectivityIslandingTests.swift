import CryptoKit
import Foundation
import XCTest
@testable import SwiftPowerSolver

/// D2: does connectivity-based islanding agree with the `h`-based
/// classification the goldens pin?
///
/// This runs BEFORE any classification code changes, because
/// `FactorsIdentityTests` pins the islanding vector by hash
/// (case300 `69fc9f47c69e7f0a`, and the case1354 / case9241 equivalents). If
/// connectivity disagrees anywhere, the golden pins a MISCLASSIFICATION and has
/// since `24895bc` — a finding, not an obstacle.
///
/// D42 measured coextensivity app-side on 135 branches. The pegase cases carry
/// 16,049 branches and a far wider reactance spread, which is exactly where the
/// `h`-based method should be expected to break first.
final class ConnectivityIslandingTests: XCTestCase {

    private func note(_ m: String) {
        FileHandle.standardError.write(Data((m + "\n").utf8))
    }

    /// Bridges of the live, in-service graph — iterative Tarjan, so a 9,241-bus
    /// chain cannot overflow the stack.
    ///
    /// Parallel branches are handled by skipping the specific parent EDGE, not
    /// every edge back to the parent vertex: one of a parallel pair is never a
    /// bridge, which is the same rule `dump_reference.py:_islanding_outages`
    /// applies via networkx. Self-loops are never bridges. Branches that are
    /// out of service or touch a dead bus are inert — `false`, matching the
    /// `bSeries == 0` path in `DistributionFactors`.
    static func bridges(_ net: BusBranchNetwork) -> [Bool] {
        let n = net.busCount
        let nbr = net.branches.count
        let live = net.buses.map { $0.type != .isolated }

        var active = [Bool](repeating: false, count: nbr)
        var adj = [[(to: Int, edge: Int)]](repeating: [], count: n)
        for (k, br) in net.branches.enumerated() {
            guard br.inService, live[br.from], live[br.to], br.from != br.to else { continue }
            active[k] = true
            adj[br.from].append((br.to, k))
            adj[br.to].append((br.from, k))
        }

        var disc = [Int](repeating: -1, count: n)
        var low = [Int](repeating: 0, count: n)
        var isBridge = [Bool](repeating: false, count: nbr)
        var timer = 0

        for root in 0..<n where live[root] && disc[root] == -1 {
            // (node, incoming edge id, next adjacency index to examine)
            var stack: [(node: Int, parentEdge: Int, next: Int)] = []
            disc[root] = timer; low[root] = timer; timer += 1
            stack.append((root, -1, 0))

            while !stack.isEmpty {
                let frame = stack[stack.count - 1]
                if frame.next < adj[frame.node].count {
                    stack[stack.count - 1].next += 1
                    let (to, edge) = adj[frame.node][frame.next]
                    if edge == frame.parentEdge { continue }   // the edge we came in on
                    if disc[to] == -1 {
                        disc[to] = timer; low[to] = timer; timer += 1
                        stack.append((to, edge, 0))
                    } else {
                        low[frame.node] = min(low[frame.node], disc[to])
                    }
                } else {
                    stack.removeLast()
                    if let parent = stack.last {
                        low[parent.node] = min(low[parent.node], low[frame.node])
                        if low[frame.node] > disc[parent.node] {
                            isBridge[frame.parentEdge] = true
                        }
                    }
                }
            }
        }
        return (0..<nbr).map { isBridge[$0] && active[$0] }
    }

    private static func sha(_ values: [Double]) -> String {
        values.withUnsafeBytes { raw in
            SHA256.hash(data: raw).map { String(format: "%02x", $0) }.joined().prefix(16).description
        }
    }

    private func compare(_ name: String, _ net: BusBranchNetwork) {
        let factors = DistributionFactors.build(net)
        let nbr = net.branches.count
        let hBased = (0..<nbr).map { factors.isIslanding(outage: $0) }
        let structural = Self.bridges(net)

        let hHash = Self.sha(hBased.map { $0 ? 1.0 : 0.0 })
        let cHash = Self.sha(structural.map { $0 ? 1.0 : 0.0 })
        let differing = (0..<nbr).filter { hBased[$0] != structural[$0] }

        note("D2 \(name): branches=\(nbr) h-based=\(hBased.filter { $0 }.count) "
             + "connectivity=\(structural.filter { $0 }.count) "
             + "hHash=\(hHash) cHash=\(cHash) differing=\(differing.count)")

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
    }

    func testReferenceCasesAgree() throws {
        for name in ["case14", "case39", "case118"] {
            compare(name, try ReferenceCase.load(name).network())
        }
    }

    func testPegaseScaleFixtures() throws {
        guard let paths = ProcessInfo.processInfo.environment["SPS_FACTORS_CASES"] else {
            XCTFail("SPS_FACTORS_CASES unset — D2 cannot compare at scale. "
                    + "Regenerate with Tools/dump_factors_fixture.py")
            return
        }
        let decoder = JSONDecoder()
        decoder.keyDecodingStrategy = .convertFromSnakeCase
        for path in paths.split(separator: ",").map(String.init) {
            let f = try decoder.decode(FactorsIdentityTests.NetworkFixture.self,
                                       from: Data(contentsOf: URL(fileURLWithPath: path)))
            compare(f.name, f.network())
        }
    }
}
