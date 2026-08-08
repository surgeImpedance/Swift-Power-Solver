import XCTest
@testable import SwiftPowerSolver

// Distributed slack with generator P limits (the regulating range).
//
// WHY THE ORACLE IS BUILT THE WAY IT IS. pandapower does not enforce P limits:
// `runpp(distributed_slack=True)` ignores `min_p_mw` / `max_p_mw` entirely —
// verified on 3.2.1 empirically (a gen driven to 60.03 MW against a 21 MW max)
// and by source (`pypower/newtonpf.py`, the distributed-slack implementation,
// contains no P-limit code path). So unlike the angle reference, the T->pi
// conversion or the short-circuit κ, there is NO upstream convention to match
// here: the pin-and-redistribute rule is this package's own specification.
//
// It is still validated against pandapower, by NETWORK EQUIVALENCE. A converged
// pinned solution is identical to an unlimited distributed-slack problem where
// each pinned unit is fixed at its limit with zero participation and the
// survivors renormalize — a network pandapower CAN solve. `Tools/dump_reference.py`
// drives the cascade one level at a time and solves every level in pandapower,
// so the reference is pandapower's arithmetic throughout; only the pin decision
// is ours.
//
// What each test covers:
//   1. agreement with that oracle at machine precision, on a genuine TWO-level
//      cascade and on a lower-limit pin;
//   2. that the cascade really is two-level — g2 is inside its limit until g1
//      pins — so the case is not a batch pin wearing a cascade's name;
//   3. the additive gate: strip the limits and the same network reproduces
//      pandapower's unlimited distributed slack, bit for bit;
//   4. pinned units sit EXACTLY on their bound, not near it;
//   5. saturation — when every participant is past its range, one keeps
//      regulating and is reported, rather than the solve going singular.
final class DistributedSlackPLimitTests: XCTestCase {

    private struct Reference: Decodable {
        struct Gen: Decodable {
            var bus: Int
            var pSetMw: Double
            var vmPu: Double
            var slackWeight: Double
            var pMinMw: Double?
            var pMaxMw: Double?
        }
        struct Solution: Decodable {
            var vmPu: [Double]; var vaDeg: [Double]; var genPMw: [Double]
        }
        struct Network: Decodable {
            var name: String
            var baseMva: Double
            var buses: [ReferenceCase.RefBus]
            var branches: [ReferenceCase.RefBranch]
            var gens: [Gen]
            var cascadeLevels: Int
            var pinnedGenIndices: [Int]
            var solution: Solution
            var unlimitedSolution: Solution

            /// - Parameter limits: false strips every P limit, which is the
            ///   additive gate — the network must then solve exactly as it did
            ///   before these fields existed.
            /// - Parameter onlyLimitGen: keep the limit on one generator and
            ///   drop the rest, used to show the cascade's second level is real.
            func network(limits: Bool = true, onlyLimitGen: Int? = nil) -> BusBranchNetwork {
                let base = baseMva
                let nb = buses.map { b in
                    BusBranchNetwork.Bus(
                        type: BusBranchNetwork.BusType(rawValue: b.type) ?? .pq,
                        baseKv: b.baseKv, pLoadPu: b.pdMw / base, qLoadPu: b.qdMvar / base,
                        gsPu: b.gsMw / base, bsPu: b.bsMvar / base)
                }
                let br = branches.map { b in
                    BusBranchNetwork.Branch(from: b.f, to: b.t, r: b.r, x: b.x, b: b.b, g: b.g,
                                            tap: b.tap <= 0 ? 1.0 : b.tap,
                                            shiftRad: b.shiftDeg * .pi / 180,
                                            inService: b.status == 1)
                }
                let gen = gens.enumerated().map { (k, g) -> BusBranchNetwork.Generator in
                    let keep = limits && (onlyLimitGen == nil || onlyLimitGen == k)
                    return BusBranchNetwork.Generator(
                        bus: g.bus, pPu: g.pSetMw / base, vSetPu: g.vmPu, vaRefRad: 0,
                        slackWeight: g.slackWeight,
                        pMinPu: keep ? g.pMinMw.map { $0 / base } : nil,
                        pMaxPu: keep ? g.pMaxMw.map { $0 / base } : nil)
                }
                return BusBranchNetwork(baseMVA: base, buses: nb, branches: br, generators: gen)
            }
        }

        var networks: [Network]

        static func load() throws -> Reference {
            guard let url = Bundle.module.url(forResource: "distributed_slack_plimits",
                                              withExtension: "json",
                                              subdirectory: "Reference") else {
                throw XCTSkip("missing distributed_slack_plimits.json — "
                              + "run Tools/dump_reference.py")
            }
            let d = JSONDecoder(); d.keyDecodingStrategy = .convertFromSnakeCase
            return try d.decode(Reference.self, from: Data(contentsOf: url))
        }
    }

    private func options() -> PowerFlowOptions {
        var o = PowerFlowOptions(); o.tolerancePu = 1e-12; return o
    }

    /// Angles de-referenced against the angle-reference bus, so a pure reference
    /// rotation cancels instead of reading as a mismatch.
    private func deref(_ va: [Double]) -> [Double] { va.map { $0 - va[0] } }

    private func assertMatches(_ sol: PowerFlowSolution,
                               _ expected: Reference.Solution,
                               base: Double, name: String,
                               tol: Double = 1e-9,
                               file: StaticString = #filePath, line: UInt = #line) {
        XCTAssertTrue(sol.converged, "\(name) did not converge", file: file, line: line)
        for (i, want) in expected.vmPu.enumerated() {
            XCTAssertEqual(sol.vmPu[i], want, accuracy: tol,
                           "\(name) vm[\(i)]", file: file, line: line)
        }
        let gotVa = deref(sol.vaRad).map { $0 * 180 / .pi }
        let wantVa = expected.vaDeg.map { $0 - expected.vaDeg[0] }
        for (i, want) in wantVa.enumerated() {
            XCTAssertEqual(gotVa[i], want, accuracy: tol,
                           "\(name) va[\(i)]", file: file, line: line)
        }
        for (g, want) in expected.genPMw.enumerated() {
            XCTAssertEqual(sol.genPPu[g] * base, want, accuracy: tol * base,
                           "\(name) genP[\(g)]", file: file, line: line)
        }
    }

    // MARK: - 1. Agreement with the equivalent-network oracle

    func testAgreementVsPandapowerEquivalentNetwork() throws {
        let ref = try Reference.load()
        XCTAssertFalse(ref.networks.isEmpty)
        for net in ref.networks {
            let sol = NewtonRaphsonSolver().solve(net.network(), options: options())
            assertMatches(sol, net.solution, base: net.baseMva, name: net.name)
            XCTAssertEqual(sol.pLimitedGenIndices.sorted(), net.pinnedGenIndices,
                           "\(net.name): the cascade must pin exactly the units "
                           + "the oracle pinned")
            XCTAssertTrue(sol.pSaturatedGenIndices.isEmpty,
                          "\(net.name): no unit should be saturated here")
        }
    }

    /// At least one reference network must exercise more than one cascade level,
    /// or the cascade is untested however many units pin.
    func testAtLeastOneReferenceIsAMultiLevelCascade() throws {
        let ref = try Reference.load()
        XCTAssertTrue(ref.networks.contains { $0.cascadeLevels >= 2 },
                      "no reference network exercises a multi-level cascade")
        XCTAssertTrue(ref.networks.contains { $0.pinnedGenIndices.count >= 2 },
                      "no reference network pins more than one unit")
    }

    // MARK: - 2. The cascade's second level is real, not a batch pin

    /// With ONLY g1 limited, g2 lands above what its own limit would be — so in
    /// the full case g2 is pushed over the edge BY g1 pinning, not by the
    /// original dispatch. That is what makes it a cascade rather than two
    /// independent violations discovered in the same pass.
    func testSecondCascadeLevelIsCausedByTheFirstPin() throws {
        let ref = try Reference.load()
        let net = try XCTUnwrap(ref.networks.first { $0.name == "cascade_two_pin" })
        let base = net.baseMva
        let g2Max = try XCTUnwrap(net.gens[2].pMaxMw)

        // Unlimited: g2 is comfortably INSIDE its limit.
        let unlimited = NewtonRaphsonSolver().solve(net.network(limits: false),
                                                    options: options())
        XCTAssertLessThan(unlimited.genPPu[2] * base, g2Max,
                          "g2 must start inside its range, or there is no cascade")

        // Only g1 limited: pinning g1 pushes g2 OVER its limit.
        let afterG1 = NewtonRaphsonSolver().solve(net.network(onlyLimitGen: 1),
                                                  options: options())
        XCTAssertEqual(afterG1.pLimitedGenIndices, [1])
        XCTAssertGreaterThan(afterG1.genPPu[2] * base, g2Max,
                             "g1 pinning must be what drives g2 past its limit")

        // Full case: both pinned.
        let full = NewtonRaphsonSolver().solve(net.network(), options: options())
        XCTAssertEqual(full.pLimitedGenIndices.sorted(), [1, 2])
    }

    // MARK: - 3. Additive gate

    /// Strip the limits and the network must reproduce pandapower's ordinary
    /// unlimited distributed slack — the same answer it gave before P limits
    /// existed. This is the "AGC off changes nothing" guarantee at network level.
    func testNoLimitsReproducesUnlimitedDistributedSlack() throws {
        let ref = try Reference.load()
        for net in ref.networks {
            let sol = NewtonRaphsonSolver().solve(net.network(limits: false),
                                                  options: options())
            assertMatches(sol, net.unlimitedSolution, base: net.baseMva,
                          name: "\(net.name) [unlimited]")
            XCTAssertTrue(sol.pLimitedGenIndices.isEmpty,
                          "nothing may pin when no limits are set")
            XCTAssertTrue(sol.pSaturatedGenIndices.isEmpty)
        }
    }

    /// The limits must actually be doing something — otherwise the agreement
    /// test above would pass on a solver that ignored them entirely.
    func testLimitsChangeTheAnswer() throws {
        let ref = try Reference.load()
        for net in ref.networks {
            let limited = NewtonRaphsonSolver().solve(net.network(), options: options())
            let unlimited = NewtonRaphsonSolver().solve(net.network(limits: false),
                                                        options: options())
            let moved = zip(limited.genPPu, unlimited.genPPu)
                .map { abs($0 - $1) }.max() ?? 0
            XCTAssertGreaterThan(moved * net.baseMva, 1.0,
                                 "\(net.name): limits must move the dispatch")
        }
    }

    // MARK: - 4. Pinned units sit exactly on the bound

    func testPinnedUnitsSitExactlyOnTheirLimit() throws {
        let ref = try Reference.load()
        for net in ref.networks {
            let sol = NewtonRaphsonSolver().solve(net.network(), options: options())
            for g in sol.pLimitedGenIndices {
                let delivered = sol.genPPu[g] * net.baseMva
                let lo = net.gens[g].pMinMw, hi = net.gens[g].pMaxMw
                let bound = (hi.map { abs(delivered - $0) < abs(delivered - (lo ?? .infinity)) }
                             ?? false) ? hi! : (lo ?? hi!)
                XCTAssertEqual(delivered, bound, accuracy: 1e-9,
                               "\(net.name) gen \(g) must sit ON its limit")
            }
        }
    }

    // MARK: - 5. Saturation

    /// Every participant past its range. Something still has to balance the
    /// island, so the largest-participation unit keeps regulating beyond its
    /// limit and is reported — rather than the slack column going identically
    /// zero and the solve failing as "singular Jacobian", which would describe
    /// the arithmetic instead of the physical condition.
    func testSaturationKeepsTheLargestParticipantRegulating() throws {
        let ref = try Reference.load()
        let net = try XCTUnwrap(ref.networks.first { $0.name == "cascade_two_pin" })
        let base = net.baseMva

        // Squeeze every contributor to a tiny range around its setpoint, so no
        // unit can absorb the imbalance.
        var bbn = net.network()
        for g in bbn.generators.indices {
            bbn.generators[g].pMinPu = bbn.generators[g].pPu - 0.001
            bbn.generators[g].pMaxPu = bbn.generators[g].pPu + 0.001
        }
        let sol = NewtonRaphsonSolver().solve(bbn, options: options())

        XCTAssertTrue(sol.converged, "saturation must still produce an answer")
        XCTAssertEqual(sol.pSaturatedGenIndices.count, 1,
                       "exactly one unit keeps regulating")
        let saturated = try XCTUnwrap(sol.pSaturatedGenIndices.first)
        // Largest participation wins: ext_grid carries weight 3, the largest.
        XCTAssertEqual(saturated, 0, "the largest-participation unit keeps regulating")
        XCTAssertFalse(sol.pLimitedGenIndices.contains(saturated),
                       "a saturated unit is not also reported as pinned")
        XCTAssertEqual(sol.pLimitedGenIndices.count, bbn.generators.count - 1,
                       "every other participant pins")
        // And it really is outside its declared range.
        let delivered = sol.genPPu[saturated] * base
        let hi = (bbn.generators[saturated].pMaxPu ?? .infinity) * base
        let lo = (bbn.generators[saturated].pMinPu ?? -.infinity) * base
        XCTAssertTrue(delivered > hi + 1e-9 || delivered < lo - 1e-9,
                      "saturated means genuinely beyond the range")
    }

    /// Power balance still holds when units pin: total generation equals total
    /// load plus losses. Checked independently of the oracle, so a systematic
    /// error in the reference could not hide here.
    func testPowerBalanceHoldsWithPinnedUnits() throws {
        let ref = try Reference.load()
        for net in ref.networks {
            let bbn = net.network()
            let sol = NewtonRaphsonSolver().solve(bbn, options: options())
            let gen = sol.genPPu.reduce(0, +)
            let load = bbn.buses.reduce(0.0) { $0 + $1.pLoadPu }
            let losses = sol.branchFlows.reduce(0.0) { $0 + $1.pFromPu + $1.pToPu }
            XCTAssertEqual(gen, load + losses, accuracy: 1e-9,
                           "\(net.name): generation must equal load + losses")
        }
    }
}
