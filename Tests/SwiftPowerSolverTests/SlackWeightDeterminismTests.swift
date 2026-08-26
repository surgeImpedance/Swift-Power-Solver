import XCTest
@testable import SwiftPowerSolver

/// D-RL-04 control: the distributed-slack weight normalization must not depend
/// on `Set` iteration order.
///
/// `renormalizeParticipation()` (`NewtonRaphson.swift:147`) folds a sum over
/// `activeContributors`, a `Set<Int>`. Swift seeds `Set` iteration PER PROCESS,
/// and floating-point addition is not associative, so an unsorted fold returns
/// a different last bit on different launches of the same binary on the same
/// input. That is invisible in a single run, which is how it survived.
///
/// **This gate's power comes from the expected value being LAUNCH-INVARIANT.**
/// A digest pinned here can only be pinned at all if the computation is
/// order-independent; before the `:164`/`:169` sort it could not have been. Run
/// twice and it is a two-launch bit-identity check; run in CI it is a permanent
/// one.
///
/// The fixture is deliberately adversarial. The defect is only reachable when
/// **all four** hold — distributed slack, a P-limit pin (the single call site
/// `:266` is gated on one), at least two remaining contributors, and weights
/// whose sum is order-sensitive in binary floating point. Ordinary weight
/// distributions sum identically in any order, which is why no existing fixture
/// caught this. Hence `1.0` against five at `1.1e-16`.
///
/// Measured on this fixture before the fix: **3 distinct digests across 16
/// launches**. After: 16 of 16 identical.
///
/// ⚠️ **This control is probabilistic per launch, and that is recorded rather
/// than glossed.** Mutation-checked by reverting the `:164`/`:169` sort:
/// **14 of 20 launches failed, 6 passed** — the sorted order is itself one of
/// the orders the hash seed can produce, so a single green run does not prove
/// the sort is present. Detection is ~70% per launch and approaches certainty
/// over a handful. Do not read one pass as proof; read a repeated pass as one.
///
/// ✅ **Superseded as the primary control 2026-08-26 by
/// `testParticipationIsInvariantUnderContributorOrder`**, which is deterministic
/// in a single launch (mutation-checked: **6 of 6 launches fail** with the
/// defensive sort removed, against this gate's 14 of 20). Both are kept — this
/// one covers the whole solve, that one covers the fold.
final class SlackWeightDeterminismTests: XCTestCase {

    /// 7 contributors; gen 1 is driven into `pMaxPu` so the pin fires and
    /// `renormalizeParticipation()` runs over the remaining six.
    private func adversarialNetwork() -> BusBranchNetwork {
        var buses: [BusBranchNetwork.Bus] = []
        for i in 0..<8 {
            buses.append(.init(type: i == 0 ? .slack : (i == 7 ? .pq : .pv),
                               baseKv: 138,
                               pLoadPu: i == 7 ? 4.0 : 0,
                               qLoadPu: i == 7 ? 0.8 : 0))
        }
        let br: [BusBranchNetwork.Branch] = [
            .init(from: 0, to: 1, r: 0.01, x: 0.10),
            .init(from: 1, to: 2, r: 0.012, x: 0.11),
            .init(from: 2, to: 3, r: 0.013, x: 0.09),
            .init(from: 3, to: 4, r: 0.011, x: 0.12),
            .init(from: 4, to: 5, r: 0.010, x: 0.10),
            .init(from: 5, to: 6, r: 0.014, x: 0.13),
            .init(from: 6, to: 7, r: 0.011, x: 0.10),
            .init(from: 1, to: 7, r: 0.02, x: 0.17),
            .init(from: 0, to: 4, r: 0.017, x: 0.14),
        ]
        let eps = 1.1e-16
        let gens: [BusBranchNetwork.Generator] = [
            .init(bus: 0, pPu: 0.5, vSetPu: 1.0, slackWeight: 1.0,  pMinPu: 0.0, pMaxPu: 9.0),
            .init(bus: 1, pPu: 0.6, vSetPu: 1.0, slackWeight: 0.25, pMinPu: 0.0, pMaxPu: 0.6005),
            .init(bus: 2, pPu: 0.7, vSetPu: 1.0, slackWeight: eps,  pMinPu: 0.0, pMaxPu: 9.0),
            .init(bus: 3, pPu: 0.8, vSetPu: 1.0, slackWeight: eps,  pMinPu: 0.0, pMaxPu: 9.0),
            .init(bus: 4, pPu: 0.9, vSetPu: 1.0, slackWeight: eps,  pMinPu: 0.0, pMaxPu: 9.0),
            .init(bus: 5, pPu: 0.4, vSetPu: 1.0, slackWeight: eps,  pMinPu: 0.0, pMaxPu: 9.0),
            .init(bus: 6, pPu: 0.3, vSetPu: 1.0, slackWeight: eps,  pMinPu: 0.0, pMaxPu: 9.0),
        ]
        return BusBranchNetwork(baseMVA: 100, buses: buses, branches: br, generators: gens)
    }

    private func digest(_ s: PowerFlowSolution) -> UInt64 {
        var h: UInt64 = 0xcbf29ce484222325
        func fnv(_ x: Double) {
            var b = x.bitPattern
            for _ in 0..<8 { h = (h ^ (b & 0xff)) &* 0x100000001b3; b >>= 8 }
        }
        for v in s.genPPu { fnv(v) }
        for v in s.genQPu { fnv(v) }
        for v in s.vmPu { fnv(v) }
        for v in s.vaRad { fnv(v) }
        return h
    }

    /// THE DURABLE CONTROL: a permutation invariant, deterministic in ONE launch.
    ///
    /// The digest gate below is probabilistic (~70%) because it can only observe
    /// whatever order this process's hash seed happens to produce. This one does
    /// not wait for the seed: it feeds the SAME contributors through
    /// `normalizeParticipation` in several EXPLICIT orders and requires
    /// bit-identical output. If the fold ever stops being order-independent —
    /// or a caller is added that supplies an unordered sequence — this fails on
    /// the first run, every run.
    ///
    /// This is the control the digest gate should have been. It is kept
    /// alongside rather than replacing it: the digest gate covers the whole
    /// solve, this one covers the fold.
    func testParticipationIsInvariantUnderContributorOrder() {
        let net = adversarialNetwork()
        let contributors = net.generators.indices.filter {
            net.generators[$0].inService && (net.generators[$0].slackWeight ?? 0) != 0
        }
        XCTAssertGreaterThan(contributors.count, 2,
                             "a permutation test needs something to permute")

        func run(_ ordered: [Int]) -> ([Double], [Double]) {
            var swGen = [Double](repeating: 0, count: net.generators.count)
            var swBus = [Double](repeating: 0, count: net.busCount)
            NewtonRaphsonSolver.normalizeParticipation(
                ordered: ordered, generators: net.generators,
                swGen: &swGen, swBus: &swBus)
            return (swGen, swBus)
        }

        // Deliberately fixed permutations, not shuffles: a randomized test that
        // fails is a test nobody can reproduce.
        let sorted = contributors.sorted()
        let orders: [(String, [Int])] = [
            ("sorted", sorted),
            ("reversed", sorted.reversed()),
            ("rotated", Array(sorted.dropFirst()) + [sorted[0]]),
            ("ends-swapped", [sorted.last!] + sorted.dropFirst().dropLast() + [sorted[0]]),
        ]
        let (refGen, refBus) = run(sorted)

        // The invariant is worth nothing if every order trivially agrees, so
        // first prove this fixture CAN distinguish orders -- the same
        // empty-payload check the digest gate makes.
        var anyDiffers = false
        for (label, order) in orders.dropFirst() {
            let (g, b) = run(order)
            for i in g.indices where g[i].bitPattern != refGen[i].bitPattern { anyDiffers = true }
            for i in b.indices where b[i].bitPattern != refBus[i].bitPattern { anyDiffers = true }
            _ = label
        }
        XCTAssertFalse(anyDiffers, """
            distributed-slack participation changed with contributor ORDER. \
            The fold in normalizeParticipation is not order-independent, which \
            is D-RL-04 -- see NewtonRaphson.swift. This fails deterministically, \
            unlike the digest gate below.
            """)
    }

    /// SENSITIVITY OF THE ABOVE. The permutation invariant only means something
    /// if this fixture's weights are order-sensitive in the first place; on
    /// ordinary weights every order agrees trivially and the test would pass on
    /// an empty payload. Summing them by hand in two orders must disagree.
    func testFixtureWeightsAreActuallyOrderSensitive() {
        let net = adversarialNetwork()
        let w = net.generators.compactMap { $0.slackWeight }
        let ascending = w.sorted().reduce(0.0, +)
        let descending = w.sorted(by: >).reduce(0.0, +)
        XCTAssertNotEqual(ascending.bitPattern, descending.bitPattern, """
            this fixture's weights sum identically in either order, so \
            testParticipationIsInvariantUnderContributorOrder proves nothing. \
            Widen the magnitude spread (1.0 against ~1e-16) until they differ.
            """)
    }

    func testSlackWeightNormalizationIsLaunchInvariant() {
        let net = adversarialNetwork()
        let sol = NewtonRaphsonSolver().solve(net,
                                              options: PowerFlowOptions(enforceQLimits: true))

        // The fixture must actually REACH the defect, or this gate is a
        // tautology — the CLAUDE.md test: would it pass on an empty payload?
        XCTAssertTrue(sol.converged, "fixture must converge or it proves nothing")
        XCTAssertEqual(sol.pLimitedGenIndices, [1],
                       "the P-limit pin must fire — renormalizeParticipation() has "
                       + "ONE call site (NewtonRaphson.swift:266) and it is gated on "
                       + "a pin. No pin, no defect, no gate.")

        let d = digest(sol)
        print(String(format: "D-RL-04 DIGEST %016llx", d))
        XCTAssertEqual(d, 0xb9007838fe60416e,
                       "distributed-slack output moved. If this fails INTERMITTENTLY "
                       + "across launches, the :164/:169 sort has been reverted and "
                       + "D-RL-04 is back. If it fails CONSISTENTLY, the solver "
                       + "changed and this constant needs re-recording deliberately.")
    }
}
