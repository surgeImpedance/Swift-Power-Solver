import Foundation

// N-1 thermal contingency screening: for each single-branch outage, the
// post-contingency flow on every monitored branch, plus any rating violations.
//
//   P_m_post = P_m_pre + LODF[m, k] · P_k_pre
//
// Exact for DC: injections (including the slack's share) are unchanged by a
// branch outage, so superposition reproduces a full DC re-solve.
//
// Results are structured data — branch indices, flows, loadings. Formatting
// for display is the consuming application's job.
//
// PLUG POINTS, deliberately not built here:
//   // N-1-1            multi-element outages (branch pairs, common-mode).
//   // GENERATOR OUTAGE generator/injection contingencies via PTDF columns.

public struct ContingencyScreeningOptions: Sendable {
    /// Branches whose post-contingency flows are evaluated. nil = every
    /// in-service branch.
    public var monitoredBranches: [Int]?
    /// Outages to screen. nil = every in-service branch.
    public var outageBranches: [Int]?
    /// Fraction of rating above which a flow is reported as a violation
    /// (1.0 = 100% of rating).
    public var violationThreshold: Double

    public init(monitoredBranches: [Int]? = nil,
                outageBranches: [Int]? = nil,
                violationThreshold: Double = 1.0) {
        self.monitoredBranches = monitoredBranches
        self.outageBranches = outageBranches
        self.violationThreshold = violationThreshold
    }
}

public struct ContingencyScreening: Sendable {

    public enum Outcome: Equatable, Sendable {
        /// Post-contingency flows were computed.
        case solved
        /// Outaging this branch disconnects the network; LODF is undefined, so
        /// no post-contingency flows are reported for it. (A full re-solve of
        /// the surviving island would be needed — deliberately out of scope.)
        case islandsNetwork
        /// The branch is already out of service (or sits on a dead bus), so
        /// there is no contingency to screen.
        case branchOutOfService
    }

    public struct Violation: Equatable, Sendable {
        public let monitoredBranch: Int
        /// Signed post-contingency flow, pu on the system base.
        public let postFlowPu: Double
        public let ratingMva: Double
        /// |flow| / rating, as a fraction (1.0 = at rating).
        public let loading: Double
    }

    public struct OutageCase: Sendable {
        public let outagedBranch: Int
        public let outcome: Outcome
        /// Post-contingency flow per monitored branch, parallel to
        /// `monitoredBranches`; empty unless `outcome == .solved`.
        public let postFlowsPu: [Double]
        /// Violations among the monitored branches, worst loading first.
        public let violations: [Violation]
    }

    public let monitoredBranches: [Int]
    public let cases: [OutageCase]

    /// Every case that produced a rating violation.
    public var casesWithViolations: [OutageCase] {
        cases.filter { !$0.violations.isEmpty }
    }
}

public struct N1ContingencyAnalyzer {

    public init() {}

    /// Screen single-branch outages against a DC base solution.
    ///
    /// `base` must come from solving `net` (its `branchFlows` supply the
    /// pre-contingency flows). Branches whose `ratingMva` is nil are still
    /// evaluated for flow, but never reported as violations.
    public func screen(_ net: BusBranchNetwork,
                       base: PowerFlowSolution,
                       options: ContingencyScreeningOptions
                        = ContingencyScreeningOptions()) throws -> ContingencyScreening {
        let nbr = net.branches.count

        // UNIT 2 — the interface change the brief's §5 actually asks for.
        //
        // §5 was written on the premise that a duplicate implementation existed
        // to delete. Step 0 established there never was one: `DistributionFactors`
        // was already the single public implementation. What §5 wanted is this —
        // N-1 consuming the PUBLIC types instead of reaching into the factors
        // with raw `Int` indices.
        //
        // Two couplings become load-bearing at this line and were informational
        // before it:
        //
        //  1. ISLANDING CHANGES SOURCE. `DistributionFactors` classifies via
        //     `h`; `LODFResult` classifies via CONNECTIVITY. Gate 6.16 proved
        //     these coextensive on six cases and 16,049 branches — but that was
        //     evidence about two implementations agreeing, not about N-1's
        //     output. Routing N-1 through `LODFResult` makes it load-bearing,
        //     and it is the first place to look if the goldens ever move.
        //
        //  2. THE RESIDUAL CONTROL CAN NOW REFUSE. `engine.ptdf` applies
        //     control 2, so this call can `throw` where the old path always
        //     returned. Healthy fixtures measure 2.2e-15 to 8.8e-15 against a
        //     1e-6 tolerance, so a refusal here is a finding, not a tuning
        //     problem — the tolerance is not to be loosened to make a screen
        //     pass, which would invert the control into a formality.
        //
        // The ARITHMETIC is untouched: the superposition below is the same
        // expression in the same evaluation order, reading the same stored LODF
        // values through the public subscript. A1 froze that and E4 is watching.
        let engine = SensitivityEngine()
        let ptdf = try engine.ptdf(net)
        let redistribution = try engine.lodf(net, from: ptdf)

        func isScreenable(_ k: Int) -> Bool {
            let branch = net.branches[k]
            return branch.inService
                && net.buses[branch.from].type != .isolated
                && net.buses[branch.to].type != .isolated
        }

        let monitored = options.monitoredBranches ?? (0..<nbr).filter(isScreenable)
        let outages = options.outageBranches ?? (0..<nbr).filter(isScreenable)
        let prePu = base.branchFlows.map(\.pFromPu)

        var cases: [ContingencyScreening.OutageCase] = []
        cases.reserveCapacity(outages.count)

        for k in outages {
            guard isScreenable(k) else {
                cases.append(.init(outagedBranch: k, outcome: .branchOutOfService,
                                   postFlowsPu: [], violations: []))
                continue
            }
            // Undefined LODF: the outage splits the network. Reported as an
            // outcome rather than inf/nan. Classification is now STRUCTURAL —
            // `LODFResult.isOutageIslanding` is a graph-bridge test, not a
            // threshold on `1 − h_k` (D65 §4).
            guard !redistribution.isOutageIslanding(BranchID(k)) else {
                cases.append(.init(outagedBranch: k, outcome: .islandsNetwork,
                                   postFlowsPu: [], violations: []))
                continue
            }

            let pk = prePu[k]
            var flows = [Double](repeating: 0, count: monitored.count)
            var violations: [ContingencyScreening.Violation] = []
            for (idx, m) in monitored.enumerated() {
                // The outaged branch itself carries nothing afterwards.
                let post = (m == k) ? 0 : prePu[m]
                    + (try redistribution[BranchID(m), BranchID(k)]) * pk
                flows[idx] = post
                guard let rating = net.branches[m].ratingMva, rating > 0 else {
                    continue   // unrated: flow reported, never a violation
                }
                let loading = abs(post) * net.baseMVA / rating
                if loading > options.violationThreshold {
                    violations.append(.init(monitoredBranch: m, postFlowPu: post,
                                            ratingMva: rating, loading: loading))
                }
            }
            violations.sort { $0.loading > $1.loading }
            cases.append(.init(outagedBranch: k, outcome: .solved,
                               postFlowsPu: flows, violations: violations))
        }

        return ContingencyScreening(monitoredBranches: monitored, cases: cases)
    }
}
