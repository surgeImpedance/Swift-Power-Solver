import XCTest
@testable import SwiftPowerSolver

// Piece 3 oracle, part 2 — the end-result check: LODF-predicted
// post-contingency flows must match a real pandapower DC re-solve with the
// branch out of service (rundcpp per outage). This validates the answer, not
// just the intermediate matrix.
final class N1ScreeningTests: XCTestCase {

    private let flowTol = 1e-4     // MW

    private func validate(case name: String) throws {
        let ref = try ReferenceCase.load(name)
        guard let cg = ref.contingency else {
            throw XCTSkip("\(name).json has no contingency block — run Tools/dump_reference.py")
        }
        let net = ref.network()
        let base = DCPowerFlowSolver().solve(net)
        XCTAssertTrue(base.converged, "\(name): base DC solve failed")

        var options = ContingencyScreeningOptions()
        options.outageBranches = cg.outages.map(\.branch)
        let screening = try N1ContingencyAnalyzer().screen(net, base: base, options: options)

        let byOutage = Dictionary(uniqueKeysWithValues:
            screening.cases.map { ($0.outagedBranch, $0) })
        let monitored = screening.monitoredBranches
        let baseMVA = net.baseMVA

        var worst = ErrorStats(reference: [], computed: [])
        var worstOutage = -1
        var compared = 0
        var islanded = 0

        for outage in cg.outages {
            guard let result = byOutage[outage.branch] else {
                XCTFail("\(name): outage \(outage.branch) missing from screening")
                continue
            }
            if outage.islands {
                XCTAssertEqual(result.outcome, .islandsNetwork,
                               "\(name): outage \(outage.branch) should report islanding")
                XCTAssertTrue(result.postFlowsPu.isEmpty,
                              "\(name): islanding outage must not report flows")
                islanded += 1
                continue
            }

            XCTAssertEqual(result.outcome, .solved,
                           "\(name): outage \(outage.branch) should have solved")
            // Compare on the monitored branches, in MW.
            var refFlows: [Double] = []
            var ourFlows: [Double] = []
            for (idx, m) in monitored.enumerated() {
                guard let r = outage.postPFromMw[m] else { continue }  // NaN in reference
                refFlows.append(r)
                ourFlows.append(result.postFlowsPu[idx] * baseMVA)
            }
            let stats = ErrorStats(reference: refFlows, computed: ourFlows)
            compared += 1
            if stats.maxError > worst.maxError { worst = stats; worstOutage = outage.branch }
        }

        XCTAssertGreaterThan(compared, 0, "\(name): no outages compared")
        XCTAssertGreaterThan(islanded, 0, "\(name): islanding path not exercised")
        XCTAssertLessThanOrEqual(worst.maxError, flowTol,
                                 "\(name): worst " + worst.description("post-flow", unit: "MW"))

        print(String(format: "%@ N-1 post-contingency vs pandapower re-solve: "
                     + "%d outages (%d islanding reported), max |ΔP| %.3e MW "
                     + "(outage %d), mean %.3e MW",
                     name, compared, islanded, worst.maxError, worstOutage,
                     worst.meanError))
    }

    func testCase14() throws { try validate(case: "case14") }
    func testCase39() throws { try validate(case: "case39") }
    func testCase118() throws { try validate(case: "case118") }

    // MARK: - Synthetic network: phase shift + off-nominal tap

    /// The IEEE cases have off-nominal taps but no phase shifters, so LODF with
    /// a phase shift is structurally unvalidated by them. This is
    /// self-validating: the LODF-predicted post-contingency flow must equal an
    /// actual DC re-solve of the same network with the branch out of service.
    /// Both a phase shifter and an off-nominal tap are present, on the same
    /// network but different branches.
    private func shifterNetwork() -> BusBranchNetwork {
        // Ring 0-1-2-3-0 plus a 0-2 chord, so no single outage islands it.
        let buses = [
            BusBranchNetwork.Bus(type: .slack, baseKv: 110),
            BusBranchNetwork.Bus(type: .pq, baseKv: 110, pLoadPu: 0.6, qLoadPu: 0.2),
            BusBranchNetwork.Bus(type: .pq, baseKv: 110, pLoadPu: 0.9, qLoadPu: 0.3),
            BusBranchNetwork.Bus(type: .pq, baseKv: 110, pLoadPu: 0.4, qLoadPu: 0.1),
        ]
        let branches = [
            BusBranchNetwork.Branch(from: 0, to: 1, r: 0.01, x: 0.10, b: 0.02),
            // phase shifter
            BusBranchNetwork.Branch(from: 1, to: 2, r: 0.012, x: 0.12, b: 0.02,
                                    shiftRad: 0.05),
            BusBranchNetwork.Branch(from: 2, to: 3, r: 0.008, x: 0.09, b: 0.02),
            BusBranchNetwork.Branch(from: 3, to: 0, r: 0.015, x: 0.14, b: 0.02),
            // off-nominal tap, and a second shifter with the opposite sign
            BusBranchNetwork.Branch(from: 0, to: 2, r: 0.02, x: 0.18, b: 0.01,
                                    tap: 0.975, shiftRad: -0.03),
        ]
        let gens = [BusBranchNetwork.Generator(bus: 0, pPu: 0, vSetPu: 1.0)]
        return BusBranchNetwork(baseMVA: 100, buses: buses, branches: branches,
                                generators: gens)
    }

    func testPhaseShifterAndTapAgainstReSolve() throws {
        let net = shifterNetwork()
        let base = DCPowerFlowSolver().solve(net)
        XCTAssertTrue(base.converged)

        let screening = try N1ContingencyAnalyzer().screen(net, base: base)
        var worstAll = 0.0
        var worstOutage = -1

        for result in screening.cases {
            XCTAssertEqual(result.outcome, .solved,
                           "meshed network: outage \(result.outagedBranch) should solve")
            // Ground truth: re-solve DC with the branch actually removed.
            var outaged = net
            outaged.branches[result.outagedBranch].inService = false
            let resolved = DCPowerFlowSolver().solve(outaged)
            XCTAssertTrue(resolved.converged)

            let refFlows = screening.monitoredBranches.map { resolved.branchFlows[$0].pFromPu }
            let stats = ErrorStats(reference: refFlows, computed: result.postFlowsPu)
            XCTAssertLessThanOrEqual(
                stats.maxError, 1e-9,
                "outage \(result.outagedBranch): " + stats.description("post-flow", unit: "pu"))
            if stats.maxError > worstAll {
                worstAll = stats.maxError
                worstOutage = result.outagedBranch
            }
        }

        print(String(format: "phase-shifter+tap network: LODF vs DC re-solve, "
                     + "%d outages, max |ΔP| %.3e pu (outage %d)",
                     screening.cases.count, worstAll, worstOutage))
    }

    // MARK: - Screening semantics

    func testRatingsAndEdgeCases() throws {
        var net = shifterNetwork()
        net.branches[0].ratingMva = 15          // deliberately tight -> violation
        net.branches[1].ratingMva = 10_000      // never violated
        // branches 2..4 stay unrated (nil)
        net.branches[3].inService = false       // out-of-service outage

        let base = DCPowerFlowSolver().solve(net)
        let screening = try N1ContingencyAnalyzer().screen(
            net, base: base,
            options: ContingencyScreeningOptions(outageBranches: [0, 1, 2, 3, 4]))
        let byOutage = Dictionary(uniqueKeysWithValues:
            screening.cases.map { ($0.outagedBranch, $0) })

        // Out-of-service branch: reported, not screened.
        XCTAssertEqual(byOutage[3]?.outcome, .branchOutOfService)
        XCTAssertTrue(byOutage[3]?.violations.isEmpty ?? false)

        // Unrated branches never appear as violations.
        let flagged = Set(screening.cases.flatMap { $0.violations.map(\.monitoredBranch) })
        XCTAssertFalse(flagged.contains(2), "unrated branch reported as violated")
        XCTAssertFalse(flagged.contains(4), "unrated branch reported as violated")
        XCTAssertFalse(flagged.contains(1), "branch under rating reported as violated")
        XCTAssertTrue(flagged.contains(0), "tightly-rated branch should violate")

        // Violations are sorted worst-first and carry a consistent loading.
        for result in screening.cases {
            XCTAssertEqual(result.violations, result.violations.sorted { $0.loading > $1.loading })
            for v in result.violations {
                XCTAssertEqual(v.loading, abs(v.postFlowPu) * net.baseMVA / v.ratingMva,
                               accuracy: 1e-12)
                XCTAssertGreaterThan(v.loading, 1.0)
            }
        }
        XCTAssertFalse(screening.casesWithViolations.isEmpty)
    }

    /// A radial branch must be reported as islanding, not silently solved.
    func testRadialBranchReportsIslanding() throws {
        // 0 (slack) - 1 - 2, with 2 hanging off a single branch.
        let buses = [
            BusBranchNetwork.Bus(type: .slack, baseKv: 110),
            BusBranchNetwork.Bus(type: .pq, baseKv: 110, pLoadPu: 0.5),
            BusBranchNetwork.Bus(type: .pq, baseKv: 110, pLoadPu: 0.3),
        ]
        let branches = [
            BusBranchNetwork.Branch(from: 0, to: 1, r: 0.01, x: 0.1),
            BusBranchNetwork.Branch(from: 1, to: 2, r: 0.01, x: 0.1),   // radial
        ]
        let net = BusBranchNetwork(
            baseMVA: 100, buses: buses, branches: branches,
            generators: [BusBranchNetwork.Generator(bus: 0, pPu: 0, vSetPu: 1.0)])

        let base = DCPowerFlowSolver().solve(net)
        let screening = try N1ContingencyAnalyzer().screen(net, base: base)
        for result in screening.cases {
            XCTAssertEqual(result.outcome, .islandsNetwork,
                           "every branch of a radial feeder islands the network")
            XCTAssertTrue(result.postFlowsPu.isEmpty)
        }
    }
}
