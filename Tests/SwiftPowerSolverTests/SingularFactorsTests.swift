import Foundation
import XCTest
@testable import SwiftPowerSolver

/// D65's measurements, run BEFORE the fix is designed, as the entry requires.
///
/// Two questions, in order:
///
/// 1. When `B_red` is genuinely rank-deficient, does Accelerate's sparse QR
///    report a status failure — so `SparseLinearSolver.solve` returns nil and
///    `DistributionFactors.build` silently yields all-zero factors — or does it
///    SUCCEED with a least-squares answer, which is silent garbage that no
///    status check can see?
/// 2. What is the residual spread across HEALTHY fixtures? D65 §3 forbids
///    choosing a threshold before that spread is measured.
final class SingularFactorsTests: XCTestCase {

    private func note(_ m: String) {
        FileHandle.standardError.write(Data((m + "\n").utf8))
    }

    /// Two islands, a slack in only one — D65 §3's fixture. Island A is
    /// {bus 0 slack, bus 1}, island B is {bus 2, bus 3}, no branch between.
    /// `pvpq` is [1, 2, 3] and island B contributes the block [[b,-b],[-b,b]],
    /// whose zero row sums make `B_red` singular.
    private func twoIslandOneSlack() -> BusBranchNetwork {
        BusBranchNetwork(
            baseMVA: 100,
            buses: [
                .init(type: .slack, baseKv: 138),
                .init(type: .pq, baseKv: 138, pLoadPu: 0.5),
                .init(type: .pq, baseKv: 138, pLoadPu: 0.3),
                .init(type: .pq, baseKv: 138, pLoadPu: 0.2),
            ],
            branches: [
                .init(from: 0, to: 1, r: 0.01, x: 0.10),
                .init(from: 2, to: 3, r: 0.01, x: 0.10),
            ],
            generators: [.init(bus: 0, pPu: 1.0, vSetPu: 1.0)])
    }

    /// `max_c ||B_red·x_c − e_c||inf` over every solved column — the control
    /// D65 §3 rules is the real one, because it sees both the nil path and the
    /// rank-deficient-success path. nil when the solve itself reported failure.
    static func worstColumnResidual(_ net: BusBranchNetwork) -> Double? {
        let live = net.buses.map { $0.type != .isolated }
        let isSlack = net.buses.map { $0.type == .slack }
        let model = DCModel(net: net, live: live)
        let pvpq = (0..<net.busCount).filter { live[$0] && !isSlack[$0] }
        guard !pvpq.isEmpty else { return 0 }
        var reducedIndex = [Int](repeating: -1, count: net.busCount)
        for (r, bus) in pvpq.enumerated() { reducedIndex[bus] = r }

        var entries: [(row: Int, col: Int, value: Double)] = []
        for (r, i) in pvpq.enumerated() {
            for (j, val) in model.row(i) where live[j] && !isSlack[j] {
                entries.append((r, reducedIndex[j], val))
            }
        }
        var rhs = [[Double]]()
        for r in 0..<pvpq.count {
            var e = [Double](repeating: 0, count: pvpq.count)
            e[r] = 1
            rhs.append(e)
        }
        guard let solved = SparseLinearSolver.solve(
            n: pvpq.count, entries: entries, rhsColumns: rhs) else { return nil }

        var worst = 0.0
        for (c, x) in solved.enumerated() {
            var ax = [Double](repeating: 0, count: pvpq.count)
            for e in entries { ax[e.row] += e.value * x[e.col] }
            for r in 0..<pvpq.count {
                worst = max(worst, abs(ax[r] - (r == c ? 1 : 0)))
            }
        }
        return worst
    }

    /// Question 1, at the solver.
    func testRankDeficientSolveReportsSuccessNotFailure() throws {
        let residual = Self.worstColumnResidual(twoIslandOneSlack())
        guard let residual else {
            note("D65 Q1: QR reported a STATUS FAILURE — solve returned nil.")
            return
        }
        note("D65 Q1: QR returned SparseStatusOK on a rank-deficient B_red; "
             + "worst||B_red·x − e||inf = \(residual)")
        XCTAssertGreaterThan(residual, 1e-6,
            "a rank-deficient solve that reports success must be caught by the residual")
    }

    /// Question 1, at the factors — what the N-1 screen would consume.
    func testSingularNetworkProducesPlausibleButWrongFactors() throws {
        let net = twoIslandOneSlack()
        let factors = DistributionFactors.build(net)
        let nbr = net.branches.count

        var maxAbsPTDF = 0.0
        for k in 0..<nbr {
            for j in 0..<net.busCount {
                maxAbsPTDF = max(maxAbsPTDF, abs(factors.ptdf(branch: k, bus: j)))
            }
        }
        let diagonals = (0..<nbr).map { factors.lodf(monitored: $0, outaged: $0) }
        let islanding = (0..<nbr).map { factors.isIslanding(outage: $0) }
        var anyNonFinite = false
        for m in 0..<nbr {
            for k in 0..<nbr where !factors.lodf(monitored: m, outaged: k).isFinite {
                anyNonFinite = true
            }
        }
        note("D65 Q1b: max|PTDF| = \(maxAbsPTDF)  diagonals = \(diagonals)  "
             + "islanding = \(islanding)  anyNonFinite = \(anyNonFinite)")

        // BOTH branches are bridges — each is the only path in its island — so
        // a correct build classifies both as islanding. Branch 1 is not, and
        // that misclassification is produced by the garbage solve.
        XCTAssertTrue(islanding[0], "branch 0 is a bridge")
        XCTAssertFalse(islanding[1],
            "PINS TODAY'S DEFECT: branch 1 is also a bridge, but the "
            + "least-squares solve makes it look like a healthy meshed branch. "
            + "D65's fix inverts this assertion.")
        XCTAssertFalse(anyNonFinite, "no inf/nan reaches the public accessors")
    }

    /// Question 2: the healthy spread, measured before any threshold is chosen.
    func testHealthyResidualSpread() throws {
        var rows: [(name: String, buses: Int, residual: Double)] = []
        for name in ["case14", "case39", "case118"] {
            let net = try ReferenceCase.load(name).network()
            guard let residual = Self.worstColumnResidual(net) else {
                XCTFail("\(name): solve reported failure on a healthy fixture")
                continue
            }
            rows.append((name, net.busCount, residual))
        }
        guard let singular = Self.worstColumnResidual(twoIslandOneSlack()) else {
            note("D65 Q2: singular fixture returned nil")
            return
        }
        for r in rows {
            note("D65 Q2 healthy  \(r.name)  buses=\(r.buses)  "
                 + "worst||B_red·x − e||inf = \(r.residual)")
        }
        note("D65 Q2 singular twoIslandOneSlack  buses=4  "
             + "worst||B_red·x − e||inf = \(singular)")
        let worstHealthy = rows.map { $0.residual }.max() ?? 0
        note("D65 Q2 SPREAD: worst healthy = \(worstHealthy), "
             + "singular = \(singular), separation = \(singular / worstHealthy)x")

        XCTAssertGreaterThan(rows.count, 0, "no healthy fixture was measured")
    }
}
