import Foundation

public struct PowerFlowOptions: Sendable {
    /// Convergence threshold on the infinity norm of the power mismatch, pu.
    /// (pandapower's tolerance_mva / baseMVA.)
    public var tolerancePu: Double
    public var maxIterations: Int
    /// Enforce generator reactive limits by PV -> PQ switching after each
    /// converged solve (pandapower enforce_q_lims).
    public var enforceQLimits: Bool
    /// Optional warm-start voltages (per bus, parallel to `network.buses`). When
    /// set, the Newton-Raphson solve seeds free-variable voltages from these
    /// instead of a flat start; non-finite or non-positive entries fall back to
    /// flat per bus, and slack/PV setpoints are re-pinned regardless. `nil`
    /// (the default) is an ordinary flat start — bit-identical to before this
    /// field existed. Used by `TimeSeriesSweep` to warm-start each step.
    public var initialVmPu: [Double]?
    public var initialVaRad: [Double]?
    /// Which solver runs (`PowerFlowEngine` dispatch). The default is plain
    /// Newton-Raphson, bit-identical to before this field existed;
    /// `NewtonRaphsonSolver` and `FastDecoupledSolver` called directly ignore
    /// it. `.fastDecoupled` with `enforceQLimits` (or distributed slack) is
    /// routed through the warm-start path so NR provides those features.
    public var method: SolverMethod
    /// B′/B″ construction variant for FDPF. BX (Van Amerongen) by default —
    /// the more robust choice on high-r/x networks, which is the malformed-
    /// network regime the fallback exists for.
    public var fdpfVariant: FDPFVariant
    /// When the selected method's primary attempt diverges, retry through the
    /// FDPF recovery chain (warm start → NR, then full FDPF where feature-
    /// equivalent). Off by default; never silent — see
    /// `PowerFlowSolution.solutionPath`.
    public var autoFallback: Bool
    /// Bounds for the FDPF warm-start stage: it stops at whichever of the
    /// loose tolerance / round cap comes first and hands its iterate to NR.
    public var fdpfWarmStartMaxIterations: Int
    public var fdpfWarmStartTolerancePu: Double

    public init(tolerancePu: Double = 1e-8,
                maxIterations: Int = 30,
                enforceQLimits: Bool = false,
                initialVmPu: [Double]? = nil,
                initialVaRad: [Double]? = nil,
                method: SolverMethod = .newtonRaphson,
                fdpfVariant: FDPFVariant = .bx,
                autoFallback: Bool = false,
                fdpfWarmStartMaxIterations: Int = 5,
                fdpfWarmStartTolerancePu: Double = 1e-2) {
        self.tolerancePu = tolerancePu
        self.maxIterations = maxIterations
        self.enforceQLimits = enforceQLimits
        self.initialVmPu = initialVmPu
        self.initialVaRad = initialVaRad
        self.method = method
        self.fdpfVariant = fdpfVariant
        self.autoFallback = autoFallback
        self.fdpfWarmStartMaxIterations = fdpfWarmStartMaxIterations
        self.fdpfWarmStartTolerancePu = fdpfWarmStartTolerancePu
    }
}

/// Per-branch terminal flows, pu on the system base. Receiving convention of
/// pandapower: positive P flows INTO the branch at that terminal.
public struct BranchFlow: Equatable, Sendable {
    public var pFromPu: Double
    public var qFromPu: Double
    public var pToPu: Double
    public var qToPu: Double

    public static let zero = BranchFlow(pFromPu: 0, qFromPu: 0, pToPu: 0, qToPu: 0)

    public init(pFromPu: Double, qFromPu: Double, pToPu: Double, qToPu: Double) {
        self.pFromPu = pFromPu
        self.qFromPu = qFromPu
        self.pToPu = pToPu
        self.qToPu = qToPu
    }
}

public struct PowerFlowSolution: Sendable {
    public var converged: Bool
    public var failureReason: String?
    /// Total NR iterations, summed across Q-limit restarts.
    public var iterations: Int
    /// Voltage magnitude per bus, pu. NaN for de-energized (isolated) buses.
    public var vmPu: [Double]
    /// Voltage angle per bus, radians. NaN for de-energized buses.
    public var vaRad: [Double]
    /// Parallel to network.branches; .zero for out-of-service branches and
    /// branches inside de-energized islands.
    public var branchFlows: [BranchFlow]
    /// Parallel to network.generators (pu): P including the slack share,
    /// Q as dispatched (pinned gens sit at their limit).
    public var genPPu: [Double]
    public var genQPu: [Double]
    /// Generators switched PV -> PQ because they hit a reactive limit.
    public var pinnedGenIndices: Set<Int>
    /// Distributed-slack contributors held AT a regulating (P) limit: they have
    /// stopped following the imbalance and their `genPPu` is exactly `pMinPu` or
    /// `pMaxPu`. Empty whenever no generator carries a P limit, so this is inert
    /// on every network that predates the field.
    public var pLimitedGenIndices: Set<Int> = []
    /// Contributors still regulating BEYOND their range because every
    /// participant was past its limit and something had to balance the island.
    /// Non-empty means the distribution has run out of room — the answer is
    /// converged and self-consistent, but the unit is outside the range it was
    /// declared able to regulate over. See the saturation note in
    /// `NewtonRaphsonSolver.solve`.
    public var pSaturatedGenIndices: Set<Int> = []
    /// Which route produced this answer. Stamped by `PowerFlowEngine` (and by
    /// `FastDecoupledSolver` for its own results); a `NewtonRaphsonSolver`
    /// called directly leaves the inert default `.nr`. A fallback path here is
    /// the signal that the primary method did not hold — no silent fallbacks.
    public var solutionPath: SolutionPath = .nr
    /// Per-stage iteration counts and exit mismatches for every stage the
    /// engine attempted, in order. Empty for direct `NewtonRaphsonSolver` use.
    public var stages: [SolveStage] = []
}

public protocol PowerFlowSolver {
    func solve(_ net: BusBranchNetwork, options: PowerFlowOptions) -> PowerFlowSolution
}
