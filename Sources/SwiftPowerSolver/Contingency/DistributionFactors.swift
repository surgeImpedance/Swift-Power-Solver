import Foundation

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

public struct DistributionFactors {

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
        let n = net.busCount
        let nbr = net.branches.count

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

        // X = Bbus[pvpq,pvpq]⁻¹, one column per non-slack live bus. Slack and
        // dead buses keep zero columns (PTDF is measured relative to the slack).
        var xColumns = [[Double]](repeating: [Double](repeating: 0, count: n),
                                  count: n)
        if !pvpq.isEmpty {
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
                    var column = [Double](repeating: 0, count: n)
                    for (r, i) in pvpq.enumerated() { column[i] = solved[c][r] }
                    xColumns[bus] = column
                }
            }
        }

        // PTDF[k, j] = b_k · (X[f_k, j] − X[t_k, j]).
        var ptdf = [Double](repeating: 0, count: nbr * n)
        for (k, branch) in net.branches.enumerated() {
            let b = model.bSeries[k]
            guard b != 0 else { continue }
            for j in 0..<n {
                let column = xColumns[j]
                ptdf[k * n + j] = b * (column[branch.from] - column[branch.to])
            }
        }

        // LODF from PTDF, with the islanding guard.
        var lodf = [Double](repeating: 0, count: nbr * nbr)
        var islanding = [Bool](repeating: false, count: nbr)
        for (k, branch) in net.branches.enumerated() {
            let f = branch.from, t = branch.to
            guard model.bSeries[k] != 0 else { continue }   // out of service / dead
            let h = ptdf[k * n + f] - ptdf[k * n + t]
            let den = 1 - h
            if abs(den) < islandingEpsilon {
                islanding[k] = true
                continue                                    // column stays zero
            }
            for m in 0..<nbr {
                lodf[m * nbr + k] = (ptdf[m * n + f] - ptdf[m * n + t]) / den
            }
            lodf[k * nbr + k] = -1
        }

        return DistributionFactors(
            busCount: n, branchCount: nbr,
            ptdfValues: ptdf, lodfValues: lodf, islanding: islanding)
    }
}
