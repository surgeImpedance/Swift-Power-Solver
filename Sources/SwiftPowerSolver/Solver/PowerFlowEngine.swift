import Foundation

// Method dispatch and the auto-fallback policy, in one place.
//
// `PowerFlowEngine` is the front door for method-aware solving. The individual
// solvers stay single-purpose — `NewtonRaphsonSolver` is bit-identical to its
// pre-FDPF self, `FastDecoupledSolver` is standalone FDPF — and everything
// about choosing, sequencing, and REPORTING them lives here:
//
//   .newtonRaphson          NR, exactly as before. With `autoFallback` set, a
//                           divergence triggers the recovery chain below.
//   .fastDecoupledWarmStart FDPF for a few loose-tolerance rounds to produce a
//                           seed, then NR finishes. Every NR feature (Q-limits,
//                           distributed slack, AVR semantics) is available
//                           because NR does the finishing. If the seeded NR
//                           fails, NR is retried from the caller's own start —
//                           a loose seed can land outside a basin the plain
//                           start was inside — so this method never loses to
//                           plain Newton-Raphson.
//   .fastDecoupled          Standalone FDPF. Requests that need NR-only
//                           features (Q-limits, distributed slack) are routed
//                           through the warm-start path instead — documented,
//                           not silent: the solutionPath says .fdpfWarmStartNR.
//
// The fallback chain (NR diverged, `autoFallback` on):
//   1. FDPF warm start → NR retry.
//   2. Full FDPF to convergence — only when the request needs no NR-only
//      feature, so a fallback answer never silently drops Q-limit
//      enforcement or distributed slack.
//   3. The original divergence error, enriched with every stage attempted.
//
// NO SILENT FALLBACKS: `PowerFlowSolution.solutionPath` says which route
// produced the answer, and `stages` carries per-stage iteration counts and
// exit mismatches. A `.fdpfWarmStartNRFallback` or `.fdpfFallback` result
// means the primary method did not hold — surface it to the operator.

public enum SolverMethod: String, Sendable, Equatable, CaseIterable {
    case newtonRaphson
    case fastDecoupled
    /// FDPF seed → NR finish (the app's "Auto").
    case fastDecoupledWarmStart
}

/// Which route actually produced the reported solution.
public enum SolutionPath: String, Sendable, Equatable {
    /// Newton-Raphson, as requested.
    case nr
    /// Standalone FDPF, as requested.
    case fdpf
    /// FDPF warm start → NR, as requested (method .fastDecoupledWarmStart, or
    /// a .fastDecoupled request that needed NR-only features).
    case fdpfWarmStartNR
    /// NR diverged; recovered by the FDPF warm start → NR retry.
    case fdpfWarmStartNRFallback
    /// NR (and the warm-started retry) failed; full FDPF produced the answer.
    case fdpfFallback
    /// Nothing converged; `failureReason` lists every stage attempted.
    case failed
}

/// One attempted stage of a solve, for result metadata.
public struct SolveStage: Sendable, Equatable {
    public enum Kind: String, Sendable, Equatable {
        case newtonRaphson
        case fdpfWarmStart
        case fdpf
    }
    public var kind: Kind
    /// NR iterations, or FDPF full P+Q rounds.
    public var iterations: Int
    public var converged: Bool
    /// Exit mismatch norm, pu — FDPF's scaled max(|ΔP/V|, |ΔQ/V|) for FDPF
    /// stages. nil for NR stages (a failed NR's own `failureReason` string
    /// carries its mismatch; a converged one is below tolerance by definition).
    public var finalMismatchPu: Double?

    public init(kind: Kind, iterations: Int, converged: Bool,
                finalMismatchPu: Double? = nil) {
        self.kind = kind
        self.iterations = iterations
        self.converged = converged
        self.finalMismatchPu = finalMismatchPu
    }
}

public struct PowerFlowEngine: PowerFlowSolver {

    public init() {}

    public func solve(_ net: BusBranchNetwork,
                      options: PowerFlowOptions = PowerFlowOptions()) -> PowerFlowSolution {
        solve(net, options: options, fdpfCache: nil)
    }

    /// Method-aware solve. Pass an `FDPFFactorizationCache` to reuse B′/B″
    /// factorizations across repeated solves of one topology (time-series
    /// sweeps); `TimeSeriesSweep` does this automatically.
    public func solve(_ net: BusBranchNetwork,
                      options: PowerFlowOptions,
                      fdpfCache: FDPFFactorizationCache?) -> PowerFlowSolution {
        switch options.method {
        case .newtonRaphson:
            var sol = NewtonRaphsonSolver().solve(net, options: options)
            let nrStage = SolveStage(kind: .newtonRaphson, iterations: sol.iterations,
                                     converged: sol.converged)
            if sol.converged {
                sol.solutionPath = .nr
                sol.stages = [nrStage]
                return sol
            }
            guard options.autoFallback else {
                sol.solutionPath = .failed
                sol.stages = [nrStage]
                return sol
            }
            return fallbackChain(net, options: options, fdpfCache: fdpfCache,
                                 nrStage: nrStage,
                                 nrReason: sol.failureReason ?? "did not converge")

        case .fastDecoupledWarmStart:
            return warmStartSolve(net, options: options, fdpfCache: fdpfCache)

        case .fastDecoupled:
            let needsNRFeatures = options.enforceQLimits
                || net.generators.contains { $0.inService && ($0.slackWeight ?? 0) != 0 }
            if needsNRFeatures {
                // Documented reroute (v1 has no FDPF Q-limits / distributed
                // slack): NR finishes so those semantics hold. The path in the
                // result says so.
                return warmStartSolve(net, options: options, fdpfCache: fdpfCache)
            }
            // FastDecoupledSolver stamps .fdpf / .failed and its own stage.
            return FastDecoupledSolver().solve(net, options: options, cache: fdpfCache)
        }
    }

    // MARK: - FDPF warm start → NR

    private func warmStartSolve(_ net: BusBranchNetwork,
                                options: PowerFlowOptions,
                                fdpfCache: FDPFFactorizationCache?,
                                asFallback: Bool = false,
                                priorStages: [SolveStage] = [],
                                priorReasons: [String] = []) -> PowerFlowSolution {
        let seed = FastDecoupledSolver().iterate(
            net, options: options,
            tolerancePu: options.fdpfWarmStartTolerancePu,
            maxRounds: options.fdpfWarmStartMaxIterations,
            cache: fdpfCache)
        let warmStage = SolveStage(kind: .fdpfWarmStart, iterations: seed.rounds,
                                   converged: seed.converged,
                                   finalMismatchPu: seed.finalMismatchPu)

        // Hand NR the seed even when the loose tolerance was not reached — a
        // partial FDPF iterate is still a better start than flat. NR's own
        // warm-start guard drops any non-finite per-bus entries.
        var opts = options
        opts.initialVmPu = seed.vm
        opts.initialVaRad = seed.va
        var sol = NewtonRaphsonSolver().solve(net, options: opts)
        let nrStage = SolveStage(kind: .newtonRaphson, iterations: sol.iterations,
                                 converged: sol.converged)
        let stages = priorStages + [warmStage, nrStage]

        if sol.converged {
            sol.solutionPath = asFallback ? .fdpfWarmStartNRFallback : .fdpfWarmStartNR
            sol.stages = stages
            return sol
        }

        var reasons = priorReasons
            + ["FDPF warm start → NR failed (\(sol.failureReason ?? "did not converge"))"]
        var allStages = stages

        // A loose FDPF seed is not guaranteed to beat the caller's own start —
        // near a nose point a partial iterate can land OUTSIDE the NR basin
        // that the plain start was inside. So before any change of method, and
        // only when plain NR has not already been tried (asFallback means it
        // has), retry NR from the caller's own start. This is what keeps the
        // warm-start method from ever losing to plain Newton-Raphson.
        if !asFallback {
            var coldSol = NewtonRaphsonSolver().solve(net, options: options)
            let coldStage = SolveStage(kind: .newtonRaphson, iterations: coldSol.iterations,
                                       converged: coldSol.converged)
            allStages.append(coldStage)
            if coldSol.converged {
                coldSol.solutionPath = .nr        // plain NR produced the answer
                coldSol.stages = allStages
                return coldSol
            }
            reasons.append("Newton-Raphson failed (\(coldSol.failureReason ?? "did not converge"))")
        }

        if asFallback || options.autoFallback {
            return fullFDPFFallback(net, options: options, fdpfCache: fdpfCache,
                                    stages: allStages, reasons: reasons)
        }
        sol.solutionPath = .failed
        sol.stages = allStages
        sol.failureReason = reasons.joined(separator: "; ")
        return sol
    }

    // MARK: - Fallback chain

    private func fallbackChain(_ net: BusBranchNetwork,
                               options: PowerFlowOptions,
                               fdpfCache: FDPFFactorizationCache?,
                               nrStage: SolveStage,
                               nrReason: String) -> PowerFlowSolution {
        warmStartSolve(net, options: options, fdpfCache: fdpfCache,
                       asFallback: true,
                       priorStages: [nrStage],
                       priorReasons: ["Newton-Raphson failed (\(nrReason))"])
    }

    /// Last resort: full FDPF to convergence — but never at the cost of
    /// silently dropping a requested NR-only feature.
    private func fullFDPFFallback(_ net: BusBranchNetwork,
                                  options: PowerFlowOptions,
                                  fdpfCache: FDPFFactorizationCache?,
                                  stages: [SolveStage],
                                  reasons: [String]) -> PowerFlowSolution {
        let needsNRFeatures = options.enforceQLimits
            || net.generators.contains { $0.inService && ($0.slackWeight ?? 0) != 0 }
        if needsNRFeatures {
            return enrichedFailure(net, stages: stages, reasons: reasons
                + ["full-FDPF fallback skipped (request needs Q-limit enforcement "
                   + "or distributed slack, which standalone FDPF v1 does not provide)"])
        }
        var fdpfOptions = options
        fdpfOptions.enforceQLimits = false
        var sol = FastDecoupledSolver().solve(net, options: fdpfOptions, cache: fdpfCache)
        guard sol.converged else {
            let detail = sol.failureReason ?? "did not converge"
            return enrichedFailure(net, stages: stages + sol.stages,
                                   reasons: reasons + ["full FDPF failed (\(detail))"])
        }
        sol.solutionPath = .fdpfFallback
        sol.stages = stages + sol.stages
        return sol
    }

    /// The original divergence error, enriched: no crash, no partial state —
    /// NaN voltages and zero flows exactly like any other failed solve — and a
    /// failureReason that names every stage attempted.
    private func enrichedFailure(_ net: BusBranchNetwork,
                                 stages: [SolveStage],
                                 reasons: [String]) -> PowerFlowSolution {
        PowerFlowSolution(
            converged: false,
            failureReason: reasons.joined(separator: "; "),
            iterations: stages.reduce(0) { $0 + $1.iterations },
            vmPu: [Double](repeating: .nan, count: net.busCount),
            vaRad: [Double](repeating: .nan, count: net.busCount),
            branchFlows: [BranchFlow](repeating: .zero, count: net.branches.count),
            genPPu: [Double](repeating: 0, count: net.generators.count),
            genQPu: [Double](repeating: 0, count: net.generators.count),
            pinnedGenIndices: [],
            solutionPath: .failed,
            stages: stages)
    }
}
