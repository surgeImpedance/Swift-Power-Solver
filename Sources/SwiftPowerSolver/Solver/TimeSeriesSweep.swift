import Foundation

// Quasi-static time-series sweep: solve a sequence of network conditions
// (a base network + per-step load / generation profiles), warm-starting each
// step's Newton-Raphson from the previous *converged* step's voltages.
//
// This is the // TIME SERIES SWEEP seam: the solver owns the efficient stepping
// loop (warm-start + per-step convergence handling) instead of every caller
// reimplementing a flat-started for-loop. Consecutive steps are similar, so
// warm-starting typically halves the iterations per step (pandapower's
// time-series does the same via runpp(init="results")).
//
// Warm-start is a speed optimization only: a warm-started step reaches the same
// solution a flat start would (the TimeSeriesTests assert this at machine
// precision). The single-solve NR path is unchanged — warm-start rides on the
// opt-in initialVmPu/initialVaRad options.

/// One step's conditions, applied on top of the base network.
public struct LoadStep: Sendable, Equatable {
    public struct BusLoad: Sendable, Equatable {
        public var pPu: Double
        public var qPu: Double
        public init(pPu: Double, qPu: Double) { self.pPu = pPu; self.qPu = qPu }
    }

    /// Absolute per-bus load for this step, pu on the system base. Buses not
    /// listed keep the base network's load (supports both uniform profiles —
    /// list every bus scaled — and non-uniform / RL per-bus variation).
    public var busLoads: [Int: BusLoad]
    /// Optional generator active-power setpoint overrides for this step, pu,
    /// keyed by generator index. Buses/gens not listed keep the base setpoint.
    public var genPOverridesPu: [Int: Double]

    public init(busLoads: [Int: BusLoad] = [:],
                genPOverridesPu: [Int: Double] = [:]) {
        self.busLoads = busLoads
        self.genPOverridesPu = genPOverridesPu
    }
}

public struct StepResult: Sendable {
    public let converged: Bool
    public let failureReason: String?
    /// NR iterations for this step (lower when warm-started).
    public let iterations: Int
    public let vmPu: [Double]
    public let vaRad: [Double]
    public let branchFlows: [BranchFlow]
    /// True if this step was warm-started from a previous converged step.
    public let warmStarted: Bool
}

public struct TimeSeriesSweep {

    /// What to do when a step fails to converge.
    public enum NonConvergencePolicy: Sendable {
        /// Record the failed step and keep going (RL episodes hit odd states).
        /// The next step warm-starts from the last *converged* step, never from
        /// the divergent iterates, so one bad step does not cascade.
        case continueSweep
        /// Stop the sweep at the first failure (planning use).
        case halt
    }

    public init() {}

    /// Solve each step against `base`, warm-starting from the previous
    /// converged step. `initialGuess` seeds step 0 (otherwise step 0 flat-starts).
    public func run(base: BusBranchNetwork,
                    steps: [LoadStep],
                    options: PowerFlowOptions = PowerFlowOptions(),
                    onNonConvergence: NonConvergencePolicy = .continueSweep,
                    initialGuess: (vmPu: [Double], vaRad: [Double])? = nil)
        -> [StepResult] {
        let solver = NewtonRaphsonSolver()
        var results: [StepResult] = []
        results.reserveCapacity(steps.count)

        // The warm-start source: the last converged step's voltages (or the
        // caller's initial guess for step 0). Deliberately NOT updated on a
        // non-converged step, so its divergent iterates never seed a successor.
        var warm: (vm: [Double], va: [Double])? = initialGuess.map { ($0.vmPu, $0.vaRad) }

        for step in steps {
            let net = apply(step, to: base)
            var opts = options
            let warmStarted = warm != nil
            opts.initialVmPu = warm?.vm
            opts.initialVaRad = warm?.va

            let sol = solver.solve(net, options: opts)
            results.append(StepResult(
                converged: sol.converged, failureReason: sol.failureReason,
                iterations: sol.iterations, vmPu: sol.vmPu, vaRad: sol.vaRad,
                branchFlows: sol.branchFlows, warmStarted: warmStarted))

            if sol.converged {
                warm = (sol.vmPu, sol.vaRad)            // advance the warm source
            } else if onNonConvergence == .halt {
                break
            }
            // On continue: leave `warm` at the last converged step (or nil).
        }
        return results
    }

    /// Apply a step's load / generation overrides to a copy of the base network.
    private func apply(_ step: LoadStep, to base: BusBranchNetwork) -> BusBranchNetwork {
        var net = base
        for (bus, load) in step.busLoads where bus >= 0 && bus < net.buses.count {
            net.buses[bus].pLoadPu = load.pPu
            net.buses[bus].qLoadPu = load.qPu
        }
        for (g, p) in step.genPOverridesPu where g >= 0 && g < net.generators.count {
            net.generators[g].pPu = p
        }
        return net
    }
}
