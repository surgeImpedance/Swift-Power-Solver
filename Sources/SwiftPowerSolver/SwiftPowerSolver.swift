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
//   Planned: DC power flow, N-1 contingency screening, short-circuit — all
//   consuming the same `BusBranchNetwork` and Ybus.

import Foundation
