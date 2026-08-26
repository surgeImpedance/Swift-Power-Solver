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
