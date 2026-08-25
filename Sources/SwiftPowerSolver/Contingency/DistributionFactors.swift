import Foundation
import os

// PTDF / LODF linear sensitivity factors, built on the DC model (Piece 2).
//
//   PTDF[k, j] : change in branch k's flow per unit injection at bus j,
//                with the slack absorbing. PTDF = Bf · Bbus⁻¹, slack column 0.
//   LODF[m, k] : change in branch m's flow per unit of branch k's pre-outage
//                flow, when branch k is outaged:
//
//                  LODF[m, k] = (PTDF[m, f_k] − PTDF[m, t_k]) / (1 − h_k)
//                  h_k        =  PTDF[k, f_k] − PTDF[k, t_k]
//                  LODF[k, k] = −1
//
// This reuses DCModel (B′ assembly, tap, phase-shift injections) verbatim —
// no parallel topology construction — and the additive multi-RHS sparse solve
// so Bbus is factorized once for all bus columns.
//
// PLUG POINTS, deliberately not built here:
//   // N-1-1           multi-element outages: compose LODF pairs with the
//                      Woodbury update over the outaged pair.
//   // GENERATOR OUTAGE generator contingencies: an injection shift handled by
//                      PTDF columns rather than LODF.

// `Sendable` added at unit 1a so `PTDFResult` / `LODFResult` can hold factors
// without copying 1.19 GB at case9241 scale. Genuinely Sendable: every stored
// property is a `let` over `Int` / `[Double]` / `[Bool]`, so no `@unchecked` is
// needed. A1 froze this type's ARITHMETIC; a protocol conformance is not
// arithmetic, and the bit-identity gate is the check that proves it.
public struct DistributionFactors: Sendable {

    /// Below this, the LODF denominator (1 − h_k) counts as zero: outaging k
    /// disconnects the network, so no finite redistribution factor exists.
    static let islandingEpsilon = 1e-9

    public let busCount: Int
    public let branchCount: Int

    /// Row-major PTDF, branchCount × busCount.
    private let ptdfValues: [Double]
    /// Row-major LODF, branchCount × branchCount (monitored × outaged).
    /// Columns for islanding outages are left zero — see `isIslanding`.
    private let lodfValues: [Double]
    private let islanding: [Bool]

    public func ptdf(branch k: Int, bus j: Int) -> Double {
        ptdfValues[k * busCount + j]
    }

    /// Redistribution factor for `monitored` when `outaged` is removed.
    /// Only meaningful when `isIslanding(outage: outaged)` is false.
    public func lodf(monitored m: Int, outaged k: Int) -> Double {
        lodfValues[m * branchCount + k]
    }

    /// True when outaging branch `k` splits the network, making LODF for that
    /// outage undefined (1 − h_k → 0).
    ///
    /// NOTE — deliberate divergence from pandapower: pypower's `makeLODF`
    /// divides by zero here and returns a column of `inf`/`nan`. We classify
    /// the outage instead and leave the column zero, so callers get a reported
    /// outcome rather than silent garbage. The validation tests assert this
    /// classification and diff only the finite columns — a future reader
    /// should NOT "fix" this back toward emitting inf/nan.
    public func isIslanding(outage k: Int) -> Bool { islanding[k] }

    // MARK: - Build

    public static func build(_ net: BusBranchNetwork) -> DistributionFactors {
        // The nil-hook build cannot be cancelled, so the force-unwrap is total.
        buildCancellable(net, isCancelled: nil)!
    }

    /// `build`, with a cooperative cancellation hook. `isCancelled` is polled
    /// between stages and once per parallel row chunk; a true return abandons
    /// the build and yields nil. Additive API: `build(_:)` forwards here, and
    /// existing callers are untouched.
    public static func buildCancellable(_ net: BusBranchNetwork,
                                        isCancelled: (@Sendable () -> Bool)?)
                                        -> DistributionFactors? {
        let n = net.busCount
        let nbr = net.branches.count
        let cancelled = { isCancelled?() ?? false }

        // Live / slack classification, matching DCPowerFlowSolver: only
        // `.isolated` buses are dead, and `.slack` buses are the references.
        var live = [Bool](repeating: false, count: n)
        var isSlack = [Bool](repeating: false, count: n)
        for (i, bus) in net.buses.enumerated() {
            switch bus.type {
            case .slack: isSlack[i] = true; live[i] = true
            case .pv, .pq: live[i] = true
            case .isolated: break
            }
        }

        let model = DCModel(net: net, live: live)
        let pvpq = (0..<n).filter { live[$0] && !isSlack[$0] }
        var reducedIndex = [Int](repeating: -1, count: n)
        for (r, bus) in pvpq.enumerated() { reducedIndex[bus] = r }

        let signposter = OSSignposter(subsystem: "SwiftPowerSolver", category: "factors")

        // X = Bbus[pvpq,pvpq]⁻¹, stored ROW-major and flat: xFlat[i*n + j] is
        // X[bus i, injection-column j]. Row-major because the PTDF stage reads
        // X along rows f_k and t_k — flat sequential runs instead of the
        // former array-of-columns pointer chase (which was ~10% of a 9241
        // build by itself). Slack and dead buses keep zero rows/columns
        // (PTDF is measured relative to the slack).
        var xFlat = [Double](repeating: 0, count: n * n)
        if !pvpq.isEmpty {
            let spSolve = signposter.beginInterval("solve")
            var entries: [(row: Int, col: Int, value: Double)] = []
            entries.reserveCapacity(pvpq.count * 4)
            for (r, i) in pvpq.enumerated() {
                for (j, val) in model.row(i) where live[j] && !isSlack[j] {
                    entries.append((r, reducedIndex[j], val))
                }
            }
            // One unit-injection RHS per non-slack live bus.
            var rhs = [[Double]]()
            rhs.reserveCapacity(pvpq.count)
            for r in 0..<pvpq.count {
                var e = [Double](repeating: 0, count: pvpq.count)
                e[r] = 1
                rhs.append(e)
            }
            if let solved = SparseLinearSolver.solve(
                n: pvpq.count, entries: entries, rhsColumns: rhs) {
                for (c, bus) in pvpq.enumerated() {
                    for (r, i) in pvpq.enumerated() {
                        xFlat[i * n + bus] = solved[c][r]
                    }
                }
            }
            signposter.endInterval("solve", spSolve)
        }
        if cancelled() { return nil }

        // PTDF[k, j] = b_k · (X[f_k, j] − X[t_k, j]), branch rows in parallel.
        // Each row k writes only its own slice and each element is the same
        // two loads, subtract, multiply as the sequential original, in the
        // same j order — fixed partitioning, no accumulation, so parallel
        // execution is bit-identical (pinned by FactorsIdentityTests).
        let spPtdf = signposter.beginInterval("ptdf")
        var ptdf = [Double](repeating: 0, count: nbr * n)
        let branches = net.branches
        let bSeries = model.bSeries
        ptdf.withUnsafeMutableBufferPointer { pp in
            let pBase = pp.baseAddress!
            xFlat.withUnsafeBufferPointer { xp in
                let xBase = xp.baseAddress!
                DispatchQueue.concurrentPerform(iterations: nbr) { k in
                    guard !cancelled() else { return }
                    let b = bSeries[k]
                    guard b != 0 else { return }
                    let fRow = xBase + branches[k].from * n
                    let tRow = xBase + branches[k].to * n
                    let out = pBase + k * n
                    for j in 0..<n {
                        out[j] = b * (fRow[j] - tRow[j])
                    }
                }
            }
        }
        signposter.endInterval("ptdf", spPtdf)
        if cancelled() { return nil }

        // LODF from PTDF, with the islanding guard. Per-outage terms (f, t,
        // 1/(1−h) as the ORIGINAL division — see below) are precomputed in
        // branch order, then monitored ROWS fill in parallel: row-major
        // sequential writes instead of the former column-strided fill
        // (stride nbr·8 bytes ≈ 128 KB on case9241 — the cache-hostile
        // pattern that made this stage 21% of the build).
        //
        // Arithmetic note: each element is computed as
        // (ptdf[m,f] − ptdf[m,t]) / den — the division kept per element, NOT
        // hoisted to a reciprocal multiply, because x/den and x·(1/den) can
        // differ in the last bit and identity is the bar.
        let spLodf = signposter.beginInterval("lodf")
        var lodf = [Double](repeating: 0, count: nbr * nbr)
        var islanding = [Bool](repeating: false, count: nbr)
        struct Outage { var f = 0; var t = 0; var den = 0.0; var active = false }
        var outages = [Outage](repeating: Outage(), count: nbr)
        for (k, branch) in net.branches.enumerated() {
            guard bSeries[k] != 0 else { continue }         // out of service / dead
            let f = branch.from, t = branch.to
            let h = ptdf[k * n + f] - ptdf[k * n + t]
            let den = 1 - h
            if abs(den) < islandingEpsilon {
                islanding[k] = true                          // column stays zero
                continue
            }
            outages[k] = Outage(f: f, t: t, den: den, active: true)
        }
        lodf.withUnsafeMutableBufferPointer { lp in
            let lBase = lp.baseAddress!
            ptdf.withUnsafeBufferPointer { pp in
                let pBase = pp.baseAddress!
                outages.withUnsafeBufferPointer { op in
                    let oBase = op.baseAddress!
                    DispatchQueue.concurrentPerform(iterations: nbr) { m in
                        guard !cancelled() else { return }
                        let row = lBase + m * nbr
                        let pRow = pBase + m * n
                        for k in 0..<nbr {
                            let o = oBase[k]
                            guard o.active else { continue }
                            row[k] = (pRow[o.f] - pRow[o.t]) / o.den
                        }
                        if oBase[m].active { row[m] = -1 }
                    }
                }
            }
        }
        signposter.endInterval("lodf", spLodf)
        if cancelled() { return nil }

        return DistributionFactors(
            busCount: n, branchCount: nbr,
            ptdfValues: ptdf, lodfValues: lodf, islanding: islanding)
    }
}
