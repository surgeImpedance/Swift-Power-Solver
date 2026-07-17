"""
Dump pandapower reference solutions for validating SwiftPowerSolver.

For each IEEE case we dump, at full precision:
  - ppc-level bus / branch / gen parameters (matpower-style, already the
    per-unit pi-model the solver operates on — transformer t-model, tap and
    shift conversion has been done by pandapower)
  - pandapower's own Ybus as sparse triplets (the Ybus-assembly oracle)
  - solved bus vm/va and branch P/Q flows, with and without generator
    Q-limit enforcement (the Newton-Raphson oracle)

Run with any Python that has pandapower installed (pip install pandapower):

  python Tools/dump_reference.py

Output goes to Tests/SwiftPowerSolverTests/Reference/*.json and is checked
in, so `swift test` works from a bare clone with no Python installed.
Regenerate only when changing this script or bumping pandapower.
"""
from __future__ import annotations

import json
import pathlib
import sys

import numpy as np
import pandapower as pp
import pandapower.networks as pn
from pandapower.pypower.idx_bus import (
    BUS_I, BUS_TYPE, PD, QD, GS, BS, VM, VA, BASE_KV,
)
from pandapower.pypower.idx_gen import (
    GEN_BUS, PG, QG, QMAX, QMIN, VG, GEN_STATUS,
)
from pandapower.pypower.idx_brch import (
    F_BUS, T_BUS, BR_R, BR_X, BR_B, BR_G, TAP, SHIFT, BR_STATUS, PF, QF, PT, QT,
    BR_R_ASYM, BR_X_ASYM, BR_G_ASYM, BR_B_ASYM,
)

HERE = pathlib.Path(__file__).resolve().parent
OUT_DIR = HERE.parent / "Tests" / "SwiftPowerSolverTests" / "Reference"

CASES = {
    "case14": pn.case14,
    "case39": pn.case39,
    "case118": pn.case118,
}


def _check_ppc_is_identity_mapped(net) -> None:
    """The Swift side indexes buses 0..n-1 in ppc order. For the IEEE cases
    (no switches, everything in service) the pandapower->ppc bus lookup must
    be the identity; bail loudly if that ever changes."""
    lookup = net._pd2ppc_lookups["bus"]
    n = len(net.bus)
    if not np.array_equal(lookup[:n], np.arange(n)):
        raise AssertionError("ppc bus lookup is not the identity mapping")


def dump_params(net) -> dict:
    """Input-side ppc data: buses, branches, gens — everything Ybus/NR needs."""
    ppc = net._ppc
    buses = []
    for row in ppc["bus"]:
        buses.append({
            "i": int(row[BUS_I]),
            "type": int(row[BUS_TYPE]),        # 1=PQ 2=PV 3=slack 4=isolated
            "pd_mw": float(row[PD]),
            "qd_mvar": float(row[QD]),
            "gs_mw": float(row[GS]),           # shunt P at V=1 pu, MW
            "bs_mvar": float(row[BS]),         # shunt Q at V=1 pu, MVAr
            "base_kv": float(row[BASE_KV]),
        })
    # pandapower's ppc branch matrix may be complex-valued; the parameter
    # columns we dump must be purely real or the Swift pi-model would
    # silently diverge.
    par = ppc["branch"][:, [BR_R, BR_X, BR_B, BR_G, TAP, SHIFT]]
    if np.max(np.abs(np.imag(par))) > 1e-12:
        raise AssertionError("ppc branch parameters have imaginary parts")
    # The Swift model has one symmetric pi per branch; pandapower can emit
    # asymmetric from/to parameters for some trafo models — bail loudly.
    asym = ppc["branch"][:, [BR_R_ASYM, BR_X_ASYM, BR_G_ASYM, BR_B_ASYM]]
    if np.max(np.abs(asym)) > 1e-12:
        raise AssertionError("ppc has asymmetric branch parameters (unsupported)")
    branches = []
    for row in ppc["branch"]:
        branches.append({
            "f": int(row[F_BUS].real),
            "t": int(row[T_BUS].real),
            "r": float(row[BR_R].real),
            "x": float(row[BR_X].real),
            "b": float(row[BR_B].real),        # total charging susceptance, pu
            "g": float(row[BR_G].real),        # total charging conductance, pu
                                               # (trafo magnetizing after t->pi)
            "tap": float(row[TAP].real),       # 0 means 1.0 (matpower convention)
            "shift_deg": float(row[SHIFT].real),
            "status": int(row[BR_STATUS].real),
        })
    # Slack reference angle is an input (e.g. case118 pins its slack at 30°);
    # take it from the pandapower-level ext_grid table.
    slack_va = {int(row["bus"]): float(row["va_degree"])
                for _, row in net.ext_grid.iterrows()}
    gens = []
    for row in ppc["gen"]:
        gens.append({
            "bus": int(row[GEN_BUS]),
            "pg_mw": float(row[PG]),           # input setpoint (pre-solve column use)
            "qmax_mvar": float(row[QMAX]),
            "qmin_mvar": float(row[QMIN]),
            "vg_pu": float(row[VG]),
            "va_deg": slack_va.get(int(row[GEN_BUS]), 0.0),
            "status": int(row[GEN_STATUS]),
        })
    return {
        "base_mva": float(ppc["baseMVA"]),
        "buses": buses,
        "branches": branches,
        "gens": gens,
    }


def dump_ybus(net) -> dict:
    ybus = net._ppc["internal"]["Ybus"].tocoo()
    return {
        "n": int(ybus.shape[0]),
        "row": [int(i) for i in ybus.row],
        "col": [int(j) for j in ybus.col],
        "g": [float(v.real) for v in ybus.data],
        "b": [float(v.imag) for v in ybus.data],
    }


def dump_solution(net) -> dict:
    """Result-side ppc data after runpp: vm/va per bus, branch end flows,
    solved gen P/Q. All full precision."""
    ppc = net._ppc
    flows = []
    for row in ppc["branch"]:
        flows.append({
            "pf_mw": float(row[PF].real),
            "qf_mvar": float(row[QF].real),
            "pt_mw": float(row[PT].real),
            "qt_mvar": float(row[QT].real),
        })
    return {
        "vm_pu": [float(row[VM]) for row in ppc["bus"]],
        "va_deg": [float(row[VA]) for row in ppc["bus"]],
        "branch_flows": flows,
        "gen_p_mw": [float(row[PG]) for row in ppc["gen"]],
        "gen_q_mvar": [float(row[QG]) for row in ppc["gen"]],
        "iterations": int(ppc.get("iterations", -1)),
        # Bus types AFTER the solve: with enforce_q_lims, PV buses that hit a
        # limit have been switched to PQ — lets the Swift test check switching.
        "bus_type": [int(row[BUS_TYPE]) for row in ppc["bus"]],
    }


def dump_case(name: str, make_net) -> dict:
    doc = {"name": name, "pandapower_version": pp.__version__}

    # Run 1: plain NR, no Q-limit enforcement.
    net = make_net()
    pp.runpp(net, enforce_q_lims=False, calculate_voltage_angles=True,
             init="flat", tolerance_mva=1e-10)
    _check_ppc_is_identity_mapped(net)
    doc.update(dump_params(net))
    doc["ybus"] = dump_ybus(net)
    solutions = {"default": dump_solution(net)}

    # Run 2: with generator Q-limits (PV->PQ switching).
    net_q = make_net()
    pp.runpp(net_q, enforce_q_lims=True, calculate_voltage_angles=True,
             init="flat", tolerance_mva=1e-10)
    _check_ppc_is_identity_mapped(net_q)
    solutions["q_lims"] = dump_solution(net_q)

    doc["solutions"] = solutions
    return doc


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for name, make_net in CASES.items():
        doc = dump_case(name, make_net)
        out = OUT_DIR / f"{name}.json"
        out.write_text(json.dumps(doc, indent=1))
        nb = len(doc["buses"])
        nbr = len(doc["branches"])
        nnz = len(doc["ybus"]["g"])
        it = doc["solutions"]["default"]["iterations"]
        print(f"{name}: {nb} buses, {nbr} branches, Ybus nnz={nnz}, "
              f"NR iterations={it} -> {out.relative_to(HERE.parent)}")


if __name__ == "__main__":
    sys.exit(main())
