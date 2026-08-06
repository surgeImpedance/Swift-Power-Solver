# SwiftPowerSolver

[![CI](https://github.com/surgeImpedance/Swift-Power-Solver/actions/workflows/ci.yml/badge.svg)](https://github.com/surgeImpedance/Swift-Power-Solver/actions/workflows/ci.yml)

A native Swift power flow core: Ybus assembly, polar Newton-Raphson AC power
flow, DC power flow, N-1 thermal contingency screening (PTDF/LODF), and IEC
60909 short-circuit calculation, built on Apple's Accelerate framework (sparse
solves). Runs on macOS 14+ and iOS 17+ with no third-party dependencies.

**Validated against [pandapower](https://www.pandapower.org) at machine
precision** on the IEEE 14, 39, and 118 bus systems — the validation harness
and full-precision reference solutions ship in this repo, so `swift test`
verifies every number from a bare clone with no Python installed.

## Scope: this is the solver core

SwiftPowerSolver operates on a **reduced bus-branch network**
(`BusBranchNetwork`): dense bus indices, unified pi-model branches (lines and
transformers, with off-nominal tap ratio and phase shift), generators, and
shunts — all per-unit. That type is the API seam.

**Node-breaker reduction is deliberately not in this package.** Collapsing
switches, migrating editor schemas, and converting ohmic/nameplate data to
per-unit are application concerns; the consuming application performs its own
reduction and hands the solver a `BusBranchNetwork`. (This package was
extracted from a substation simulator that does exactly that.)

## Features

- **Piece 0 — Ybus assembly** (`YbusBuilder`): line pi-models, transformer
  tap ratio and phase shift (matpower convention), bus shunts, and branch
  charging conductance (pandapower's `BR_G`, e.g. transformer magnetizing
  after the t→pi conversion).
- **Piece 1 — AC Newton-Raphson** (`NewtonRaphsonSolver`): polar form,
  PV/PQ/slack classification, convergence on max power mismatch, sparse QR
  linear solve via Accelerate, generator reactive limits (PV→PQ switching,
  matching pandapower `enforce_q_lims`), multiple slack buses, non-zero slack
  reference angles, de-energized island handling (NaN voltages).
- **Piece 2 — DC power flow** (`DCPowerFlowSolver`): MATPOWER `makeBdc`
  formulation (B′ = 1/(x·tap), phase-shift injections, bus shunts), a single
  Accelerate sparse solve reusing the AC path's linear solver. Shares the AC
  solver's slack / classification / island conventions and returns the same
  `PowerFlowSolution` (flat voltage, P-only flows), so AC and DC results are
  directly comparable on the same network.
- **Piece 3 — N-1 thermal contingency** (`DistributionFactors`,
  `N1ContingencyAnalyzer`): PTDF (`Bf·Bbus⁻¹`) and LODF built on the DC model,
  post-contingency flows `P_post = P_pre + LODF·P_pre` for every single-branch
  outage, and violations against branch thermal ratings. Outages that
  disconnect the network are classified as islanding rather than producing the
  `inf`/`nan` that the textbook formula yields there. Results are structured
  data (flows, loadings, outcomes) for the application to render.
- **Piece 4 — IEC 60909 short-circuit** (`ShortCircuitAnalyzer`): standardized
  bus-fault currents — `Ik''` (initial symmetrical), `Skss`, `ip` (peak, via
  the IEC method-C κ factor), and `Ith` (thermal), from the positive-sequence
  fault impedance `Zk = Zbus[f,f]` built with source shunts `1/(c·Zsource)`.
  Three-phase and two-phase (line-to-line) faults; external-grid-type sources
  and lines. Structured per-bus outcomes (`.solved` / `.isolatedBus` /
  `.noSourceFeeding`). Sources are `Generator`s carrying a subtransient
  impedance (`scSubtransientRPu` / `scSubtransientXPu`).
- **Time-series / quasi-static sweep** (`TimeSeriesSweep`): steps a base network
  through a sequence of per-step conditions (`[LoadStep]` — absolute per-bus
  loads, plus optional generator setpoint overrides), **warm-starting** each
  step's Newton-Raphson from the previous *converged* step's voltages. Per-step
  convergence status is reported; a diverging step does not seed its successor
  (no cascade), with continue-and-report (default) or halt-on-first-failure
  policies. This is the stepping loop a time-varying profile or an RL
  environment drives. Warm-starting rides on the opt-in
  `PowerFlowOptions.initialVmPu` / `initialVaRad` — a single solve can be
  warm-started too.
- **Planned**: zero-sequence networks (line-to-ground faults) and transformer /
  generator impedance-correction factors (K_T / K_G), multi-element and
  generator contingencies (the contingency and short-circuit code mark those
  plug points) — all consuming the same `BusBranchNetwork` and Ybus. This is an
  early, piece-by-piece project; every piece lands with its oracle tests.

## Validation results

Each reference JSON carries pandapower's own ppc-level inputs, its internal
Ybus, and its full-precision solution. The tests enforce 1e-6 pu / 1e-4 MVA
tolerances; actual agreement is machine precision:

| Oracle | Quantity | Max error |
|---|---|---|
| IEEE 14/39/118 Ybus vs `net._ppc.internal.Ybus` | admittance | 1.1e-13 pu |
| IEEE NR, with and without Q-limits | Vm | 9.3e-15 pu |
| IEEE NR, with and without Q-limits | Va | 2.1e-12 rad |
| IEEE NR | branch P/Q flows | 2.7e-09 MVA |
| case118 with Q-limits | pinned-generator set | exact match (6 gens) |
| IEEE 14/39/118 DC vs `rundcpp` | bus angle Va | 7.3e-15 rad |
| IEEE 14/39/118 DC vs `rundcpp` | branch P flow | 3.9e-12 MW |
| IEEE 14/39/118 LODF vs `makeLODF` | outage distribution factor | 5.0e-14 |
| IEEE 14/39/118 N-1 vs repeated `rundcpp` | post-contingency P | 1.5e-11 MW |
| IEC 60909 short-circuit vs `calc_sc` (3ph/2ph) | Ik'' / ip / Ith | 1.5e-13 kA |
| case14 time-series (10-step profile) vs `run_timeseries` | per-step Vm/Va/flow | 4.8e-11 MVA |
| warm-start vs flat-start, same step | Vm/Va (same root) | 1.2e-13 |

DC agreement per case (angle Va / branch P), against pandapower `rundcpp`:

| Case | Max \|ΔVa\| | Max \|ΔP\| |
|---|---|---|
| case14 | 9.4e-16 rad | 2.8e-13 MW |
| case39 | 2.3e-15 rad | 3.9e-12 MW |
| case118 (30° slack + taps) | 7.3e-15 rad | 3.9e-12 MW |

N-1 agreement per case. LODF is compared against pandapower's own
`makeLODF`; post-contingency flows against a full DC re-solve (`rundcpp`
with the branch out of service), which validates the result rather than just
the intermediate matrix. Islanding outages are classified, not diffed —
pypower emits `inf`/`nan` for those columns:

| Case | Outages | Islanding | Max \|ΔLODF\| | Max \|ΔP\| post-contingency |
|---|---|---|---|---|
| case14 | 19 | 1 | 4.0e-15 | 4.8e-13 MW |
| case39 | 35 | 11 | 5.0e-14 | 1.5e-11 MW |
| case118 | 23 | 9 | 2.1e-14 | 5.4e-12 MW |

A synthetic phase-shifter + off-nominal-tap network additionally checks LODF
against a DC re-solve at **1.2e-15 pu** — a case the IEEE networks cannot
cover, since none of them contains a phase shifter.

Short-circuit is validated against pandapower's `calc_sc` (IEC 60909) on
purpose-built networks — the IEEE cases carry no short-circuit data. The
meshed two-source network is the key one: it resolves multi-source fault
contribution through the Zbus inversion, which a radial network cannot.
Currents agree to machine precision:

| Fault | Network | Max \|ΔIk''\| | Max \|Δip\| | Max \|ΔIth\| |
|---|---|---|---|---|
| 3-phase | radial | 2.0e-14 kA | 4.4e-14 kA | 2.0e-14 kA |
| 3-phase | meshed two-source | 6.4e-14 kA | 1.5e-13 kA | 6.9e-14 kA |
| 2-phase | meshed two-source | 5.6e-14 kA | 1.3e-13 kA | 6.0e-14 kA |

Edge cases are validated as reported outcomes, not numbers: a fault at an
isolated bus reports `.isolatedBus`, and one in a source-less component
reports `.noSourceFeeding` (pandapower returns NaN for both).

The time-series sweep is validated two ways. Per-step, case14 driven by a
10-step load-multiplier profile matches pandapower's `run_timeseries`
(`ConstControl` load scaling + `runpp` per step) at machine precision. And a
distinct guard confirms **warm-start == flat-start** for the same step to
1.2e-13 — warm-starting reaches the same root, so it is a speed optimization
and never a different answer. Across the profile, warm-starting cut total
iterations from 40 (flat) to 32.

## Usage

```swift
import SwiftPowerSolver

// Your application reduces its model to a BusBranchNetwork (all per-unit):
let network = BusBranchNetwork(
    baseMVA: 100,
    buses: [
        .init(type: .slack, baseKv: 110),
        .init(type: .pq, baseKv: 110, pLoadPu: 0.4, qLoadPu: 0.12),
    ],
    // `ratingMva` is how a branch gets a thermal limit — N-1 screening reports
    // violations only against rated branches. Unrated (nil) branches still get
    // post-contingency flows, they are just never flagged.
    branches: [.init(from: 0, to: 1, r: 0.01, x: 0.1, b: 0.04, ratingMva: 120)],
    generators: [.init(bus: 0, pPu: 0, vSetPu: 1.02)])

var options = PowerFlowOptions()
options.enforceQLimits = true
let solution = NewtonRaphsonSolver().solve(network, options: options)

if solution.converged {
    print(solution.vmPu, solution.vaRad)      // bus voltages
    print(solution.branchFlows)               // per-branch P/Q at both ends
    print(solution.genPPu, solution.genQPu)   // dispatch incl. slack share
}
```

N-1 thermal screening runs off a DC base solution:

```swift
let base = DCPowerFlowSolver().solve(network)
let screening = N1ContingencyAnalyzer().screen(network, base: base)

for result in screening.casesWithViolations {
    print("outage", result.outagedBranch)
    for v in result.violations {                 // worst loading first
        print("  branch \(v.monitoredBranch) at \(v.loading * 100)% of \(v.ratingMva) MVA")
    }
}

// Outages that split the network are reported, not silently wrong:
let islanding = screening.cases.filter { $0.outcome == .islandsNetwork }
```

IEC 60909 short-circuit needs fault-current sources — generators carrying a
subtransient impedance. `scSubtransientRPu` / `scSubtransientXPu` are how a
generator (or the slack, standing in for the external grid) contributes fault
current; a generator with neither is not a source:

```swift
let scNetwork = BusBranchNetwork(
    baseMVA: 100,
    buses: [.init(type: .slack, baseKv: 110), .init(type: .pq, baseKv: 110)],
    branches: [.init(from: 0, to: 1, r: 0.008, x: 0.033)],
    // raw source impedance in pu (Un²/S″k for a grid), before the voltage factor c:
    generators: [.init(bus: 0, pPu: 0, vSetPu: 1.0,
                       scSubtransientRPu: 0.0099, scSubtransientXPu: 0.0995)])

var scOptions = ShortCircuitOptions()
scOptions.faultType = .threePhase          // or .twoPhase (line-to-line)
let sc = ShortCircuitAnalyzer().faults(scNetwork, options: scOptions)

for r in sc.buses where r.outcome == .solved {
    print("bus \(r.bus): Ik'' \(r.ikssKa) kA, ip \(r.ipKa) kA, Ith \(r.ithKa) kA")
}
```

A time-series sweep steps a base network through per-step conditions, warm-
starting each step from the previous converged one. Each `LoadStep` gives
absolute per-bus loads (a uniform profile scales the base loads; RL can vary
buses independently):

```swift
// A daily-ish load profile: scale the base loads per step.
let steps = [0.8, 1.0, 1.25, 1.1].map { m -> LoadStep in
    var busLoads: [Int: LoadStep.BusLoad] = [:]
    for (i, bus) in network.buses.enumerated() where bus.pLoadPu != 0 {
        busLoads[i] = .init(pPu: bus.pLoadPu * m, qPu: bus.qLoadPu * m)
    }
    return LoadStep(busLoads: busLoads)
}

let results = TimeSeriesSweep().run(base: network, steps: steps)
                                                    // .halt to stop at first failure
for (k, step) in results.enumerated() {
    guard step.converged else { print("step \(k): \(step.failureReason ?? "?")"); continue }
    print("step \(k): \(step.iterations) iters (warm: \(step.warmStarted)), Vm \(step.vmPu)")
}
```

A single solve can also be warm-started directly via
`PowerFlowOptions.initialVmPu` / `initialVaRad` (nil ⇒ ordinary flat start).

`YbusBuilder.build(network)` is also public if you only need the admittance
matrix.

## Building and testing

```sh
swift build
swift test     # runs the full pandapower validation harness offline
```

Requires Xcode 16+ / Swift 6 toolchain (the package builds in Swift 5
language mode) on macOS 14+.

## Regenerating the reference solutions

The checked-in references (`Tests/SwiftPowerSolverTests/Reference/*.json`)
were produced by pandapower 3.2.1. To regenerate — only needed when changing
the dump script or bumping pandapower — use any Python with pandapower
installed:

```sh
pip install pandapower
python Tools/dump_reference.py
swift test
```

The dump script asserts the modeling assumptions it relies on (identity
bus mapping, purely real branch parameters, no asymmetric pi-models) and
fails loudly if a pandapower upgrade ever changes them.

## API notes

- The complex number type is named `ComplexD` to avoid colliding with
  `Complex` from swift-numerics in consuming projects.
- `PowerFlowSolver` is a protocol with two conformers today,
  `NewtonRaphsonSolver` (AC) and `DCPowerFlowSolver` (DC), over the same
  network type; further solvers plug in the same way.

## License

[Apache-2.0](LICENSE)
