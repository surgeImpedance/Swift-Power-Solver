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

    public init(tolerancePu: Double = 1e-8,
                maxIterations: Int = 30,
                enforceQLimits: Bool = false,
                initialVmPu: [Double]? = nil,
                initialVaRad: [Double]? = nil) {
        self.tolerancePu = tolerancePu
        self.maxIterations = maxIterations
        self.enforceQLimits = enforceQLimits
        self.initialVmPu = initialVmPu
        self.initialVaRad = initialVaRad
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
}

public protocol PowerFlowSolver {
    func solve(_ net: BusBranchNetwork, options: PowerFlowOptions) -> PowerFlowSolution
}
