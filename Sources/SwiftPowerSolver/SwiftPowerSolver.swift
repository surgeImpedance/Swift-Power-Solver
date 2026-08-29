// SwiftPowerSolver — a native Swift AC power flow core.
//
// The package operates on a reduced bus-branch network (`BusBranchNetwork`,
// per-unit): consuming applications do their own node-breaker reduction and
// hand the solver buses, pi-model branches, and generators.
//
// Pieces, built and validated one at a time against pandapower
// (Tools/dump_reference.py + the reference-diff tests):
//   0. Ybus assembly (`YbusBuilder`) — line pi-models, transformer tap ratio
//      and phase shift, bus and branch shunts.
//   1. AC Newton-Raphson power flow (`NewtonRaphsonSolver`) — polar form,
//      PV/PQ/slack classification, generator Q-limit switching, multiple
//      slack buses, Accelerate sparse QR each iteration.
//   2. DC power flow (`DCPowerFlowSolver`) — makeBdc formulation, one sparse
//      solve, same slack/island conventions as the AC path.
//   3. N-1 thermal contingency (`DistributionFactors`, `N1ContingencyAnalyzer`)
//      — and, since 2026-08, PTDF/LODF/OTDF as first-class public outputs
//      (`SensitivityEngine`, `PTDFResult`, `LODFResult`, `FactorsSignature`).
//   4. IEC 60909 short-circuit (`ShortCircuitAnalyzer`).
//   All consume the same `BusBranchNetwork` and Ybus.
//
// Fast-decoupled power flow (`FastDecoupledSolver`, XB/BX per MATPOWER makeB;
// verified against pypower's makeB and the NR solutions) rides alongside as a
// standalone method, a warm-start stage, and an auto-fallback for NR
// divergence — all dispatched through `PowerFlowEngine`, which reports the
// route taken in `PowerFlowSolution.solutionPath`/`stages` (no silent
// fallbacks). B′/B″ are factorized once and cached across time-series steps
// (`FDPFFactorizationCache`). The NR path is bit-identical to before FDPF
// existed.

import Foundation
