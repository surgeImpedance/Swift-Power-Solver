import Foundation

/// Structural connectivity over the live, in-service graph.
///
/// **This is the single source (D3).** The app's D42 path consumes it after the
/// pin bump; until then the probe in `ConnectivityIslandingTests` coexists with
/// it, which is deliberate and temporary — 1b is what discharges the duplication.
///
/// **Why islanding is decided HERE and not from `1 − h_k` (D65 §4).** For a
/// bridge, `h_k = 1` EXACTLY in arithmetic at any reactance. The `h`-based
/// classifier flips when numerical error in `h` exceeds a threshold, and nothing
/// distinguishes "h is genuinely 1−δ because the branch is weakly coupled but
/// real" from "h is 1 and we resolved it to only 1e-9". A graded family measured
/// the `h` method misclassifying at residuals around 1e-8/1e-9, with the
/// distribution CONTINUOUS rather than bimodal — so no scalar threshold
/// separates them. Connectivity is exact and needs no threshold.
///
/// **What that is and is not evidence for (D66 §2).** On every real network in
/// the corpus the two methods agree exactly — zero differing branches across six
/// cases, 16,049 branches, every pinned golden reproduced. The `h` method is not
/// broken on real data; it is fragile BY CONSTRUCTION, demonstrated on synthetic
/// networks whose single-tie reactance ratio exceeds case9241's whole-network
/// spread. Connectivity is adopted for exactness and threshold-freedom, **not**
/// as a repair.
public enum NetworkConnectivity {

    /// Branches whose outage disconnects the network — graph bridges over the
    /// live, in-service subgraph.
    ///
    /// Iterative Tarjan, so a 9,241-bus chain cannot overflow the stack.
    /// Parallel branches are handled by skipping the specific parent EDGE rather
    /// than every edge back to the parent vertex: one of a parallel pair is
    /// never a bridge, matching `dump_reference.py:_islanding_outages` and
    /// networkx. Self-loops are never bridges. Branches out of service or
    /// touching a dead bus are inert and report `false`, matching the
    /// `bSeries == 0` path in `DistributionFactors`.
    public static func bridgeBranches(_ net: BusBranchNetwork) -> [Bool] {
        let live = net.buses.map { $0.type != .isolated }
        return bridgeBranches(net, live: live)
    }

    static func bridgeBranches(_ net: BusBranchNetwork, live: [Bool]) -> [Bool] {
        let n = net.busCount
        let nbr = net.branches.count

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
            var stack: [(node: Int, parentEdge: Int, next: Int)] = []
            disc[root] = timer; low[root] = timer; timer += 1
            stack.append((root, -1, 0))

            while !stack.isEmpty {
                let frame = stack[stack.count - 1]
                if frame.next < adj[frame.node].count {
                    stack[stack.count - 1].next += 1
                    let (to, edge) = adj[frame.node][frame.next]
                    if edge == frame.parentEdge { continue }
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

    /// Buses reachable from `start` over live, in-service branches. Used to
    /// reject an island-spanning participation set (D64 §2).
    static func componentReachable(from start: Int, in net: BusBranchNetwork,
                                   live: [Bool]) -> Set<Int> {
        var adj = [[Int]](repeating: [], count: net.busCount)
        for br in net.branches
        where br.inService && live[br.from] && live[br.to] {
            adj[br.from].append(br.to)
            adj[br.to].append(br.from)
        }
        var seen: Set<Int> = [start]
        var stack = [start]
        while let v = stack.popLast() {
            for w in adj[v] where !seen.contains(w) {
                seen.insert(w)
                stack.append(w)
            }
        }
        return seen
    }
}
