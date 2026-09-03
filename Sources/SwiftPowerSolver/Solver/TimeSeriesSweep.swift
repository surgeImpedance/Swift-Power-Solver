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
        /// D80: the step's absolute Z / I components of `pPu` / `qPu`, pu at
        /// V = 1 (see `Bus.pLoadZPu`). Default 0 — every pre-D80 caller passes
        /// a constant-power step, bit-identically.
        public var pZPu: Double = 0
        public var pIPu: Double = 0
        public var qZPu: Double = 0
        public var qIPu: Double = 0
        public init(pPu: Double, qPu: Double,
                    pZPu: Double = 0, pIPu: Double = 0,
                    qZPu: Double = 0, qIPu: Double = 0) {
            self.pPu = pPu; self.qPu = qPu
            self.pZPu = pZPu; self.pIPu = pIPu
            self.qZPu = qZPu; self.qIPu = qIPu
        }
    }

    /// Absolute per-bus shunt admittance for a step (switched reactive
    /// support: capacitor banks in on the ramp, reactors in at the valley).
    /// Constant-impedance semantics are preserved — these land in `Bus.gsPu`
    /// / `Bus.bsPu` and thence the Ybus diagonal, so injection stays V²·B.
    public struct BusShunt: Sendable, Equatable {
        public var gsPu: Double
        public var bsPu: Double
        public init(gsPu: Double, bsPu: Double) { self.gsPu = gsPu; self.bsPu = bsPu }
    }

    /// Absolute per-bus load for this step, pu on the system base. Buses not
    /// listed keep the base network's load (supports both uniform profiles —
    /// list every bus scaled — and non-uniform / RL per-bus variation).
    public var busLoads: [Int: BusLoad]
    /// Optional generator active-power setpoint overrides for this step, pu,
    /// keyed by generator index. Buses/gens not listed keep the base setpoint.
    public var genPOverridesPu: [Int: Double]
    /// Optional per-bus shunt overrides for this step, pu at V = 1. Buses not
    /// listed keep the base network's shunts — an empty map (the default) is
    /// exactly the pre-existing fixed-shunt behavior. NOTE for FDPF sweeps:
    /// B″ depends on bus susceptance, so shunt-varying steps invalidate the
    /// `FDPFFactorizationCache` per step (its key already covers `bsPu`);
    /// load-only steps keep the factor-once behavior unchanged.
    public var busShuntsPu: [Int: BusShunt]

    public init(busLoads: [Int: BusLoad] = [:],
                genPOverridesPu: [Int: Double] = [:],
                busShuntsPu: [Int: BusShunt] = [:]) {
        self.busLoads = busLoads
        self.genPOverridesPu = genPOverridesPu
        self.busShuntsPu = busShuntsPu
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
    /// Which route produced this step's answer (see `SolutionPath`). `.nr`
    /// for every step of a default (Newton-Raphson, no-fallback) sweep.
    public var solutionPath: SolutionPath = .nr
    /// Q-limit re-pin cycles for this step (see
    /// `PowerFlowSolution.qLimitRestarts`).
    public var qLimitRestarts: Int = 0
    /// True when this step ran with the recovery chain suppressed because the
    /// previous step's full chain had already failed — see
    /// `run(...)`'s `skipRecoveryAfterFailedStep`. A converged step with this
    /// set is an ordinary primary-method solve; a failed one did NOT retry the
    /// fallback ladder, and its `failureReason` says so.
    public var recoverySkipped: Bool = false
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
    ///
    /// When `options.method` involves FDPF (or `autoFallback` is on), steps go
    /// through `PowerFlowEngine` with one shared `FDPFFactorizationCache`, so
    /// B′/B″ are factorized exactly once for the whole sweep while topology
    /// holds still — load and generator-P changes never invalidate them.
    /// `fdpfCache` lets a caller inject (and afterwards inspect) that cache;
    /// the default builds one internally.
    ///
    /// FALLBACK ECONOMY (`skipRecoveryAfterFailedStep`, default ON). Measured
    /// on case9241pegase (2026-08-14): when a sweep enters an infeasible
    /// regime — e.g. uniformly scaled loads that no single slack can balance —
    /// EVERY step re-runs the full recovery ladder (primary NR, FDPF warm
    /// start → NR, cold retry) only to rediscover its neighbor's divergence,
    /// at ~65 wasted iterations a step. With economy on, a step whose
    /// PREDECESSOR failed the full chain runs the primary method alone
    /// (Newton-Raphson — or standalone FDPF when that IS the method — with
    /// `autoFallback` off); the moment any step converges, full behavior
    /// resumes. Consequences, stated plainly:
    ///   - Steps the primary method can solve are BIT-IDENTICAL with economy
    ///     on or off — the economized attempt is the same first attempt the
    ///     full chain would have made, from the same warm start.
    ///   - A step that only the fallback ladder could recover, sitting
    ///     immediately after a failed step, reports failure instead (its
    ///     `failureReason` and `recoverySkipped` say the ladder was skipped).
    ///     Regime recovery is still detected through the primary attempt.
    ///   - The first step never economizes, and neither does any step whose
    ///     predecessor converged. Pass `false` to diagnose with the full
    ///     chain on every step.
    public func run(base: BusBranchNetwork,
                    steps: [LoadStep],
                    options: PowerFlowOptions = PowerFlowOptions(),
                    onNonConvergence: NonConvergencePolicy = .continueSweep,
                    initialGuess: (vmPu: [Double], vaRad: [Double])? = nil,
                    fdpfCache: FDPFFactorizationCache? = nil,
                    skipRecoveryAfterFailedStep: Bool = true)
        -> [StepResult] {
        let solver = NewtonRaphsonSolver()
        // The pure-NR default takes the exact pre-FDPF path through
        // NewtonRaphsonSolver below; the engine is only in the loop when the
        // options ask for something FDPF-shaped.
        let usesFDPF = options.method != .newtonRaphson || options.autoFallback
        let engine = PowerFlowEngine()
        let cache = fdpfCache ?? (usesFDPF ? FDPFFactorizationCache() : nil)
        var results: [StepResult] = []
        results.reserveCapacity(steps.count)

        // The warm-start source: the last converged step's voltages (or the
        // caller's initial guess for step 0). Deliberately NOT updated on a
        // non-converged step, so its divergent iterates never seed a successor.
        var warm: (vm: [Double], va: [Double])? = initialGuess.map { ($0.vmPu, $0.vaRad) }

        // True after a step failed its FULL chain (never set by an economized
        // failure alone — economy only ever follows a full-chain verdict, so
        // consecutive skips are all anchored to one real diagnosis).
        var previousChainFailed = false

        for step in steps {
            let net = apply(step, to: base)
            var opts = options
            let warmStarted = warm != nil
            opts.initialVmPu = warm?.vm
            opts.initialVaRad = warm?.va

            // Fallback economy: predecessor's chain failed, so run the primary
            // attempt only. Identical to the chain's own first attempt for
            // NR-shaped methods; standalone FDPF keeps its method.
            let economize = skipRecoveryAfterFailedStep && previousChainFailed && usesFDPF
            if economize {
                opts.method = options.method == .fastDecoupled ? .fastDecoupled : .newtonRaphson
                opts.autoFallback = false
            }

            var sol = usesFDPF
                ? engine.solve(net, options: opts, fdpfCache: cache)
                : solver.solve(net, options: opts)
            if economize && !sol.converged {
                sol.failureReason = (sol.failureReason ?? "did not converge")
                    + " (recovery chain skipped: previous step's chain already failed)"
            }
            results.append(StepResult(
                converged: sol.converged, failureReason: sol.failureReason,
                iterations: sol.iterations, vmPu: sol.vmPu, vaRad: sol.vaRad,
                branchFlows: sol.branchFlows, warmStarted: warmStarted,
                solutionPath: sol.solutionPath,
                qLimitRestarts: sol.qLimitRestarts,
                recoverySkipped: economize))

            if sol.converged {
                warm = (sol.vmPu, sol.vaRad)            // advance the warm source
                previousChainFailed = false
            } else {
                if !economize { previousChainFailed = true }
                if onNonConvergence == .halt { break }
            }
            // On continue: leave `warm` at the last converged step (or nil).
        }
        return results
    }

    /// Apply a step's load / generation / shunt overrides to a copy of the
    /// base network.
    private func apply(_ step: LoadStep, to base: BusBranchNetwork) -> BusBranchNetwork {
        var net = base
        for (bus, load) in step.busLoads where bus >= 0 && bus < net.buses.count {
            net.buses[bus].pLoadPu = load.pPu
            net.buses[bus].qLoadPu = load.qPu
            // D80: the step carries its own absolute Z/I components, exactly
            // as it carries its absolute total — a scaled hour scales the
            // components with the schedule. Zero on every pre-D80 step, which
            // writes 0 over 0 and is bit-neutral.
            net.buses[bus].pLoadZPu = load.pZPu
            net.buses[bus].pLoadIPu = load.pIPu
            net.buses[bus].qLoadZPu = load.qZPu
            net.buses[bus].qLoadIPu = load.qIPu
        }
        for (g, p) in step.genPOverridesPu where g >= 0 && g < net.generators.count {
            net.generators[g].pPu = p
        }
        for (bus, sh) in step.busShuntsPu where bus >= 0 && bus < net.buses.count {
            net.buses[bus].gsPu = sh.gsPu
            net.buses[bus].bsPu = sh.bsPu
        }
        return net
    }
}
