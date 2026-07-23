# SwiftPowerSolver

[![CI](https://github.com/surgeImpedance/Swift-Power-Solver/actions/workflows/ci.yml/badge.svg)](https://github.com/surgeImpedance/Swift-Power-Solver/actions/workflows/ci.yml)

A native Swift power flow core: Ybus assembly, polar Newton-Raphson AC power
flow, and DC power flow, built on Apple's Accelerate framework (sparse solves).
Runs on macOS 14+ and iOS 17+ with no third-party dependencies.

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
- **Planned**: N-1 contingency screening (PTDF/LODF — the DC solver already
  marks the plug points), short-circuit — all consuming the same
  `BusBranchNetwork` and Ybus. This is an early, piece-by-piece project;
  every piece lands with its oracle tests.

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

DC agreement per case (angle Va / branch P), against pandapower `rundcpp`:

| Case | Max \|ΔVa\| | Max \|ΔP\| |
|---|---|---|
| case14 | 9.4e-16 rad | 2.8e-13 MW |
| case39 | 2.3e-15 rad | 3.9e-12 MW |
| case118 (30° slack + taps) | 7.3e-15 rad | 3.9e-12 MW |

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
    branches: [.init(from: 0, to: 1, r: 0.01, x: 0.1, b: 0.04)],
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
