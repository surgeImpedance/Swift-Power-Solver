# SwiftPowerSolver

[![CI](https://github.com/surgeImpedance/Swift-Power-Solver/actions/workflows/ci.yml/badge.svg)](https://github.com/surgeImpedance/Swift-Power-Solver/actions/workflows/ci.yml)

A native Swift power flow core: Ybus assembly, polar Newton-Raphson AC power
flow, DC power flow, N-1 thermal contingency screening (PTDF/LODF), and IEC
60909 short-circuit calculation, built on Apple's Accelerate framework (sparse
solves). Runs on macOS 14+ and iOS 17+ with no third-party dependencies.

**Validated against [pandapower](https://www.pandapower.org) at machine
precision** on the IEEE 14, 39, and 118 bus systems — the validation harness
and full-precision reference solutions ship in this repo, so those cases verify
from a bare clone with no Python installed.

The large-case gates read their fixtures from the committed
`Tests/SwiftPowerSolverTests/FactorsFixtures/` copies (3.9 MB total, since
2026-08-29), so **a bare clone runs the full suite with no Python installed**.
`SPS_FACTORS_CASES` overrides the committed copies; if both are absent the
gates **fail rather than skip**, so a checkout cannot mistake an unrun gate
for a passing one — see [Running the full suite](#running-the-full-suite).

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
  data (flows, loadings, outcomes) for the application to render. Since
  2026-08 the factors are also **first-class, queryable outputs** —
  `SensitivityEngine` / `PTDFResult` / `LODFResult`, see
  [Sensitivity factors](#sensitivity-factors-ptdflodfotdf) below.
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
- **Distributed slack** (`Generator.slackWeight`): instead of one slack bus
  absorbing all imbalance, participating generators share it by participation
  factor (pandapower `slack_weight`, normalized to sum 1). An extra scalar
  unknown enters the Newton-Raphson solve and each contributor delivers
  `setpoint − share`, while the angle reference is preserved (one bus still
  fixes θ). Opt-in: with no weights the solve is the ordinary single-slack NR,
  unchanged.
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
| distributed slack vs `distributed_slack=True` | Vm / Va (de-ref) / gen P | 4.4e-11 MW |

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

Distributed slack is validated against pandapower `distributed_slack=True` on
purpose-built networks (the IEEE cases are single-slack): per-bus voltages and
**per-generator P shares** agree to machine precision. Angles are compared
**de-referenced** against the slack bus, so a reference rotation cancels rather
than reading as a false mismatch — the residual (3.1e-17 rad on the 2:1:1 case)
is genuine physics, not a hidden reference offset. A negative control confirms
the match is not vacuous: perturbing one participation weight (1→5) drives the
generator dispatch 22.77 MW away from pandapower. And a uniform-weight case is
checked independently of the oracle — equal weights split the imbalance evenly
(spread 3.6e-15 MW).

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

N-1 thermal screening runs off a DC base solution. `screen` throws — the
residual control refuses a numerically hollow solve rather than returning a
plausible all-clear (`SensitivityError.singularAdmittanceMatrix`):

```swift
let base = DCPowerFlowSolver().solve(network)
let screening = try N1ContingencyAnalyzer().screen(network, base: base)

for result in screening.casesWithViolations {
    print("outage", result.outagedBranch)
    for v in result.violations {                 // worst loading first
        print("  branch \(v.monitoredBranch) at \(v.loading * 100)% of \(v.ratingMva) MVA")
    }
}

// Outages that split the network are reported, not silently wrong:
let islanding = screening.cases.filter { $0.outcome == .islandsNetwork }
```

### Sensitivity factors (PTDF/LODF/OTDF)

The factors behind the screen are a public, queryable surface of their own
(`Sources/SwiftPowerSolver/Contingency/`):

```swift
let engine = SensitivityEngine()                  // residual guard ON (1e-6)
let ptdf = try engine.ptdf(network)               // ∂F/∂P, signature-keyed
// Cancellable form, same guard on the same core — nil means CANCELLED:
// try engine.ptdf(network, isCancelled: { Task.isCancelled })
// A consumer can recompute the shift signature to check terms for currency:
// try PhaseShiftSignature.of(network) == terms.signature
let sens = try ptdf[BranchID(4), BusID(17)]       // one factor
let lodf = try engine.lodf(network, from: ptdf)   // reuses the build

if lodf.isOutageIslanding(BranchID(2)) { /* branchable, non-throwing */ }
let redistribution = try lodf[BranchID(0), BranchID(7)]
let kappa = try lodf.conditioning(outaging: BranchID(7))  // annotation only

// Complete DC flow needs the phase-shift terms — a separate, cheap value
// with its own superset signature, so a shifter retune can never serve
// stale flows from a signature-keyed cache:
let shift = try PhaseShiftTerms.of(network)
let flows = try ptdf.completeDCBranchFlows(
    injections: [BusID(17): 1.0], phaseShift: shift, network: network)
```

- **Identity:** `FactorsSignature.of(network)` — SHA-256 over exactly what
  the factors consume (`bSeries`, branch endpoints, bus classification);
  `shiftRad` is deliberately excluded. `PhaseShiftTerms.signature` embeds it
  as a strict superset. Both are the intended cache keys; **no cache ships
  in this package.**
- **Failure surface:** `SensitivityError` — `.signatureMismatch` /
  `.shiftSignatureMismatch` (stale object vs live network),
  `.islandingOutage`, `.singularAdmittanceMatrix(worstResidual:)` (control 2;
  disable with `SensitivityEngine(residualTolerance: nil)`), and the input
  validation cases.
- **Islanding is structural** (bridge-finding via `NetworkConnectivity`),
  never a numerical threshold; `conditioning` is exposed and deliberately
  unthresholded.

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

Distributed slack is opt-in: give participating generators a `slackWeight` and
the solve shares the imbalance across them by normalized weight instead of
piling it on one slack bus. With no weights the solve is ordinary single-slack.

```swift
let network = BusBranchNetwork(
    baseMVA: 100,
    buses: [.init(type: .slack, baseKv: 110),   // angle reference
            .init(type: .pv, baseKv: 110),
            .init(type: .pq, baseKv: 110, pLoadPu: 1.4, qLoadPu: 0.4)],
    branches: [.init(from: 0, to: 1, r: 0.01, x: 0.1),
               .init(from: 1, to: 2, r: 0.01, x: 0.1),
               .init(from: 0, to: 2, r: 0.02, x: 0.2)],
    generators: [
        .init(bus: 0, pPu: 0,   vSetPu: 1.02, slackWeight: 2),   // shares 50%
        .init(bus: 1, pPu: 0.5, vSetPu: 1.01, slackWeight: 1),   // shares 25%
    ])

let sol = NewtonRaphsonSolver().solve(network)
print(sol.genPPu)   // each contributor = setpoint − its share of the imbalance
```

`YbusBuilder.build(network)` is also public if you only need the admittance
matrix.

## Building and testing

```sh
swift build
swift test     # full suite, Python-free — fixtures are committed
```

Requires Xcode 16+ / Swift 6 toolchain (the package builds in Swift 5
language mode) on macOS 14+.

### Running the full suite

Six tests read the large-case fixtures. Since 2026-08-29 those are
**committed** — `Tests/SwiftPowerSolverTests/FactorsFixtures/`,
`factors_case300.json` (92 KB) / `factors_case1354.json` (447 KB) /
`factors_case9241.json` (3.3 MB), dumped by pandapower 3.2.1 — so a bare
clone runs all of them:

| test | what it gates |
|---|---|
| `FactorsIdentityTests` | the PTDF/LODF/islanding **bit-identity goldens** |
| `ConnectivityIslandingTests.testPegaseScaleFixtures` | islanding at pegase scale |
| `FootprintTests.testCase9241FactorsFootprint` | residency at 9,241 buses |
| `NearBridgeAccuracyTests.testAtScale` | near-bridge accuracy at scale |
| `SensitivityAPITests.testUnit2_enginePathFootprintAtScale` | engine-path residency |
| `SingularFactorsTests.testMeasureColumnResidualAtScale` | column-residual **measurement** at scale (a superseded quantity — the shipped 1e-6 guard is calibrated by `SensitivityAPITests.testResidualCalibration` and by nothing else) |

`SPS_FACTORS_CASES` (comma-separated paths) **overrides** the committed
copies, unchanged semantics. If it is unset *and* the committed copies are
missing, the gates **fail rather than skip on purpose**: a skipped gate and a
passing gate are indistinguishable in every summary view, and only one of
them is a guarantee — so absence is reported as a failure naming the cause.

To regenerate the fixtures (only needed when the dump schema or the oracle
version changes):

```sh
pip install pandapower  # goldens are a claim about 3.2.1 — see below
python Tools/dump_factors_fixture.py \
    --out-dir Tests/SwiftPowerSolverTests/FactorsFixtures \
    case300 case1354pegase case9241pegase
```

Measured 2026-08-29: the dump takes **6.4 s** and the bare-clone suite runs
**107 tests, 0 failures, 0 skips** in 39.5 s (release).

⚠️ `FactorsIdentityTests` **rejects a fixture whose `pandapower_version` does not
match the version its goldens were recorded against**, and says so in the failure
rather than moving a hash. A digest that moves because the oracle moved and a
digest that moves because the code moved are indistinguishable in a summary; that
guard is what separates them.

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
