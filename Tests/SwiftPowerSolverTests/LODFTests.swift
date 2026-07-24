import XCTest
@testable import SwiftPowerSolver

// Piece 3 oracle, part 1: our LODF columns must match pandapower's own
// makeLODF matrix on the IEEE cases.
//
// Islanding outages are the deliberate exception. pypower divides by
// (1 - h) with h = 1 there and returns inf/nan; we classify the outage as
// islanding instead. So the test asserts the CLASSIFICATION matches the
// reference's `islands` flag, and diffs numbers only for the finite columns.
final class LODFTests: XCTestCase {

    private let lodfTol = 1e-6

    private func validate(case name: String) throws {
        let ref = try ReferenceCase.load(name)
        guard let cg = ref.contingency else {
            throw XCTSkip("\(name).json has no contingency block — run Tools/dump_reference.py")
        }
        let net = ref.network()
        let factors = DistributionFactors.build(net)

        var worst = ErrorStats(reference: [], computed: [])
        var worstOutage = -1
        var comparedColumns = 0
        var islandingChecked = 0

        for outage in cg.outages {
            let k = outage.branch

            // Classification must agree with the reference topology.
            XCTAssertEqual(factors.isIslanding(outage: k), outage.islands,
                           "\(name): islanding classification differs for outage \(k)")
            if outage.islands {
                islandingChecked += 1
                continue        // pandapower's column is inf/nan — nothing to diff
            }

            guard let column = outage.lodfColumn else {
                XCTFail("\(name): non-islanding outage \(k) has no LODF column")
                continue
            }
            // Reference entries are finite for non-islanding outages.
            let refValues = column.map { $0 ?? .nan }
            let ourValues = (0..<net.branches.count).map {
                factors.lodf(monitored: $0, outaged: k)
            }
            let stats = ErrorStats(reference: refValues, computed: ourValues)
            comparedColumns += 1
            if stats.maxError > worst.maxError { worst = stats; worstOutage = k }
        }

        XCTAssertGreaterThan(comparedColumns, 0, "\(name): no LODF columns compared")
        XCTAssertGreaterThan(islandingChecked, 0,
                             "\(name): no islanding outage exercised the degenerate path")
        XCTAssertLessThanOrEqual(worst.maxError, lodfTol,
                                 "\(name): worst " + worst.description("LODF", unit: ""))

        print(String(format: "%@ LODF vs pandapower: %d columns (%d islanding "
                     + "classified, not diffed), max |ΔLODF| %.3e (outage %d, "
                     + "branch %d), mean %.3e",
                     name, comparedColumns, islandingChecked, worst.maxError,
                     worstOutage, worst.worstIndex, worst.meanError))
    }

    func testCase14() throws { try validate(case: "case14") }
    func testCase39() throws { try validate(case: "case39") }
    func testCase118() throws { try validate(case: "case118") }

    /// The self-sensitivity of a radial branch is exactly 1, which is what
    /// makes its LODF undefined. Guards the epsilon test in
    /// DistributionFactors against silently drifting.
    func testIslandingOutageHasUnitSelfSensitivity() throws {
        let ref = try ReferenceCase.load("case14")
        let net = ref.network()
        let factors = DistributionFactors.build(net)
        let islanders = (0..<net.branches.count).filter { factors.isIslanding(outage: $0) }
        XCTAssertFalse(islanders.isEmpty, "case14 should have a radial branch")

        for k in islanders {
            let branch = net.branches[k]
            let h = factors.ptdf(branch: k, bus: branch.from)
                - factors.ptdf(branch: k, bus: branch.to)
            XCTAssertEqual(h, 1.0, accuracy: 1e-9,
                           "islanding branch \(k) self-sensitivity should be 1")
            // And the column must be left zero, never inf/nan.
            for m in 0..<net.branches.count {
                XCTAssertTrue(factors.lodf(monitored: m, outaged: k).isFinite,
                              "LODF[\(m),\(k)] must not be inf/nan")
            }
        }
    }
}
