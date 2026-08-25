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

    // MARK: - C2: is the residual distribution BIMODAL or CONTINUOUS?
    //
    // Everything measured so far is either healthy (~1e-14) or fully
    // rank-deficient (1.0), and a 9.0e13x gap makes the threshold VALUE
    // insensitive. The design question is therefore not which number to pick,
    // it is whether anything lands in the gap. If nearly-singular systems sit
    // in the gap, one threshold separates cleanly forever. If they smear
    // continuously across it, a scalar threshold is the wrong control and the
    // classification has to key on something structural instead.

    /// Two 2-bus islands joined by ONE tie branch whose reactance is swept.
    /// As `xTie` grows, `b_tie = 1/x` goes to zero and the two islands decouple
    /// continuously — the network approaches, without ever reaching, the
    /// rank-deficient `twoIslandOneSlack` fixture. `nil` omits the tie entirely,
    /// which IS that fixture.
    private func gradedTie(xTie: Double?) -> BusBranchNetwork {
        var branches: [BusBranchNetwork.Branch] = [
            .init(from: 0, to: 1, r: 0.01, x: 0.10),
            .init(from: 2, to: 3, r: 0.01, x: 0.10),
        ]
        if let xTie { branches.append(.init(from: 1, to: 2, r: 0.01, x: xTie)) }
        return BusBranchNetwork(
            baseMVA: 100,
            buses: [
                .init(type: .slack, baseKv: 138),
                .init(type: .pq, baseKv: 138, pLoadPu: 0.5),
                .init(type: .pq, baseKv: 138, pLoadPu: 0.3),
                .init(type: .pq, baseKv: 138, pLoadPu: 0.2),
            ],
            branches: branches,
            generators: [.init(bus: 0, pPu: 1.0, vSetPu: 1.0)])
    }

    func testResidualSpreadIsBimodalOrContinuous() throws {
        note("D65 C2: tie reactance sweep — weak coupling approaching rank deficiency")
        note("D65 C2: xTie          b_tie        worst residual   islanding   max|PTDF|")

        var residuals: [(x: Double, residual: Double)] = []
        let sweep: [Double] = [1e-9, 1e-6, 1e-3, 1e-1, 1e0, 1e2, 1e4, 1e6,
                               1e8, 1e10, 1e12, 1e14, 1e16]
        for x in sweep {
            let net = gradedTie(xTie: x)
            guard let residual = Self.worstColumnResidual(net) else {
                note("D65 C2: xTie=\(x) -> solve reported FAILURE (nil)")
                continue
            }
            let f = DistributionFactors.build(net)
            var maxAbs = 0.0
            for k in 0..<net.branches.count {
                for j in 0..<net.busCount {
                    maxAbs = max(maxAbs, abs(f.ptdf(branch: k, bus: j)))
                }
            }
            let isl = (0..<net.branches.count).map { f.isIslanding(outage: $0) }
            residuals.append((x, residual))
            note(String(format: "D65 C2: %-12.0e  %-11.3e  %-15.6e  %@  %.4f",
                        x, 1.0 / x, residual, String(describing: isl), maxAbs))
        }

        // The fully rank-deficient endpoint, for the gap it has to be compared to.
        if let disconnected = Self.worstColumnResidual(gradedTie(xTie: nil)) {
            note(String(format: "D65 C2: (no tie)      0            %-15.6e  "
                        + "<- fully rank-deficient endpoint", disconnected))
        }

        // Where does the distribution actually sit? Report the largest jump
        // between consecutive points, which is what a gap looks like.
        var worstJump = 0.0
        var jumpAt = ""
        for i in 1..<residuals.count {
            let a = max(residuals[i - 1].residual, 1e-300)
            let b = max(residuals[i].residual, 1e-300)
            let ratio = b / a
            if ratio > worstJump {
                worstJump = ratio
                jumpAt = "\(residuals[i - 1].x) -> \(residuals[i].x)"
            }
        }
        note(String(format: "D65 C2: largest consecutive jump = %.3e x, at xTie %@",
                    worstJump, jumpAt))
        let maxResidual = residuals.map(\.residual).max() ?? 0
        note(String(format: "D65 C2: worst residual anywhere in the graded family = %.6e",
                    maxResidual))

        XCTAssertFalse(residuals.isEmpty, "the sweep measured nothing")
    }

    /// The scale measurement D65 §0 deferred, now decision-relevant: C2 showed
    /// the first islanding MISCLASSIFICATION appears around a residual of
    /// 3.7e-9. A scalar threshold can only be the control if healthy networks
    /// AT SCALE stay well below that. Reads the same fixtures as
    /// FactorsIdentityTests.
    func testHealthyResidualAtScale() throws {
        guard let paths = ProcessInfo.processInfo.environment["SPS_FACTORS_CASES"] else {
            XCTFail("SPS_FACTORS_CASES unset — cannot measure the at-scale residual. "
                    + "Regenerate with Tools/dump_factors_fixture.py")
            return
        }
        let decoder = JSONDecoder()
        decoder.keyDecodingStrategy = .convertFromSnakeCase
        for path in paths.split(separator: ",").map(String.init) {
            let data = try Data(contentsOf: URL(fileURLWithPath: path))
            let fixture = try decoder.decode(FactorsIdentityTests.NetworkFixture.self, from: data)
            let net = fixture.network()
            let t0 = ContinuousClock.now
            let residual = Self.worstColumnResidual(net)
            let dt = ContinuousClock.now - t0
            let ms = Double(dt.components.seconds) * 1000
                   + Double(dt.components.attoseconds) / 1e15
            note(String(format: "D65 SCALE %@: buses=%d branches=%d "
                        + "worst||B_red·x − e||inf = %@  (%.0f ms)",
                        fixture.name, net.busCount, net.branches.count,
                        residual.map { String(format: "%.6e", $0) } ?? "nil (solve failed)",
                        ms))
        }
    }
    /// Locate the misclassification ONSET precisely, and separate the two
    /// quantities that C2's decade sweep conflated:
    ///
    ///   - the RESIDUAL, a proxy for how badly conditioned the solve was;
    ///   - `|1 − h_k|`, which is what `islandingEpsilon` (1e-9) actually tests.
    ///
    /// Every branch of the 0-1-2-3 path is a bridge, so `h_k = 1` EXACTLY in
    /// arithmetic, for every tie reactance. Any drift is numerical error, and
    /// the classification flips the moment that error exceeds 1e-9. The
    /// threshold question is whether the residual can see that flip coming.
    func testMisclassificationOnset() throws {
        note("D65 ONSET: xTie        residual        |1-h_tie|       tie classified")
        var lastGood: (x: Double, residual: Double, h: Double)?
        var firstBad: (x: Double, residual: Double, h: Double)?

        var x = 1e3
        while x <= 1e8 {
            let net = gradedTie(xTie: x)
            let residual = Self.worstColumnResidual(net) ?? .nan
            let f = DistributionFactors.build(net)
            // tie is branch index 2, from bus 1 to bus 2
            let h = f.ptdf(branch: 2, bus: 1) - f.ptdf(branch: 2, bus: 2)
            let dev = abs(1 - h)
            let islanding = f.isIslanding(outage: 2)
            note(String(format: "D65 ONSET: %-10.2e  %-14.6e  %-14.6e  %@",
                        x, residual, dev, islanding ? "islanding (correct)" : "MESHED (WRONG)"))
            if islanding { lastGood = (x, residual, dev) }
            else if firstBad == nil { firstBad = (x, residual, dev) }
            x *= pow(10, 0.5)
        }

        if let g = lastGood, let b = firstBad {
            note(String(format: "D65 ONSET: last correct  xTie=%.2e residual=%.6e |1-h|=%.6e",
                        g.x, g.residual, g.h))
            note(String(format: "D65 ONSET: first WRONG   xTie=%.2e residual=%.6e |1-h|=%.6e",
                        b.x, b.residual, b.h))
            note(String(format: "D65 ONSET: worst healthy at scale = 9.734435e-13 (case9241); "
                        + "margin to first wrong residual = %.1f x", b.residual / 9.734435e-13))
        }
        XCTAssertNotNil(firstBad, "the sweep never reached a misclassification")
    }
}
