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
from pandapower.pypower.makePTDF import makePTDF
from pandapower.pypower.makeLODF import makeLODF

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


def dump_dc(net) -> dict:
    """Result-side ppc data after rundcpp: bus angles and DC branch P (both
    ends). DC is flat-voltage/lossless, so vm is 1.0 and Q is 0 — the Piece-2
    oracle only compares angles and branch P."""
    ppc = net._ppc
    va = [float(row[VA]) for row in ppc["bus"]]
    pf = [float(row[PF].real) for row in ppc["branch"]]
    pt = [float(row[PT].real) for row in ppc["branch"]]
    if not all(np.isfinite(va)) or not all(np.isfinite(pf)):
        raise AssertionError("rundcpp did not populate ppc VA / branch PF")
    return {
        "va_deg": va,
        "branch_p_from_mw": pf,
        "branch_p_to_mw": pt,
    }


def _islanding_outages(ppc) -> set:
    """Branch indices whose outage disconnects the network (graph bridges,
    counting parallel branches as non-bridges)."""
    import networkx as nx
    nb = len(ppc["bus"])
    ends = [(int(row[F_BUS].real), int(row[T_BUS].real)) for row in ppc["branch"]]
    g = nx.Graph()
    g.add_nodes_from(range(nb))
    for f, t in ends:
        g.add_edge(f, t)
    bridges = set()
    for u, v in nx.bridges(g):
        keys = [k for k, (f, t) in enumerate(ends) if {f, t} == {u, v}]
        if len(keys) == 1:          # a parallel pair is not a bridge
            bridges.add(keys[0])
    return bridges


def _outage_selection(name: str, ppc) -> list:
    """Which outages to dump, deterministically.

    case14 / case39: every branch (exhaustive).
    case118: a documented subset — ALL islanding branches plus every 8th
    branch index — so the reference stays small while still covering scale,
    off-nominal taps and the non-zero slack angle. Regenerating this script
    reproduces exactly the same set.
    """
    nbr = len(ppc["branch"])
    islanding = _islanding_outages(ppc)
    if name == "case118":
        return sorted(islanding | set(range(0, nbr, 8)))
    return list(range(nbr))


def dump_contingency(net, name: str) -> dict:
    """Piece-3 oracle. Per selected outage, two independent references:

    1. `lodf_column`: column k of pandapower's own makeLODF matrix — the
       redistribution factors for outaging branch k. For islanding outages
       pypower divides by (1 - h) with h = 1 and emits inf/nan; those are
       dumped as null so the Swift side asserts its own `islandsNetwork`
       classification instead of matching garbage.
    2. `post_p_from_mw`: post-contingency DC branch flows from a real
       re-solve (rundcpp with the branch out of service) — validates the end
       result, not just the intermediate matrix.

    Only the `_outage_selection` columns are dumped (all branches for
    case14/case39, the documented subset for case118) so the checked-in
    references stay small enough to keep a bare clone cheap.
    """
    import copy

    ppc = net._ppc
    H = makePTDF(ppc["baseMVA"], ppc["bus"], ppc["branch"])
    L = makeLODF(ppc["branch"], H)
    islanding = _islanding_outages(ppc)
    nbr = len(ppc["branch"])

    selection = _outage_selection(name, ppc)
    nl = len(net.line)
    outages = []
    for k in selection:
        islands = bool(k in islanding)
        column = None
        if not islands:
            column = [None if not np.isfinite(L[m, k]) else float(L[m, k])
                      for m in range(nbr)]

        # ppc branch order is lines first, then transformers.
        net_k = copy.deepcopy(net)
        if k < nl:
            net_k.line.loc[net_k.line.index[k], "in_service"] = False
        else:
            net_k.trafo.loc[net_k.trafo.index[k - nl], "in_service"] = False
        pp.rundcpp(net_k)
        pk = net_k._ppc["branch"][:, PF]

        outages.append({
            "branch": int(k),
            "islands": islands,
            "lodf_column": column,
            "post_p_from_mw": [None if not np.isfinite(v.real) else float(v.real)
                               for v in pk],
        })

    return {
        "islanding_branches": sorted(int(k) for k in islanding),
        "outages": outages,
    }


# --------------------------------------------------------------------------- #
# Piece 4 oracle: IEC 60909 short-circuit.                                      #
#                                                                              #
# The IEEE cases carry no short-circuit data (calc_sc fails on them), so the   #
# references are purpose-built networks, authored here in per-unit on a 100    #
# MVA base. Each is fed to pandapower in the equivalent engineering units and  #
# the SAME per-unit definition is dumped for the Swift side, so both solve     #
# identical data. Scope matches the solver: external-grid-type sources + lines #
# (no transformers -> no K_T correction), three-phase and two-phase faults.    #
# --------------------------------------------------------------------------- #
SC_BASE_MVA = 100.0
SC_C = 1.1          # c_max for HV (>1 kV); pandapower uses the same for 110 kV
SC_TK_S = 0.1
SC_FREQ_HZ = 50.0

# type codes match BusBranchNetwork.BusType: 1=pq, 3=slack, 4=isolated.
SC_NETWORKS = [
    {
        "name": "radial",
        "buses": [{"base_kv": 110.0, "type": 3}, {"base_kv": 110.0, "type": 1},
                  {"base_kv": 110.0, "type": 1}],
        "branches": [{"f": 0, "t": 1, "r_pu": 0.008264, "x_pu": 0.033058},
                     {"f": 1, "t": 2, "r_pu": 0.006612, "x_pu": 0.026446}],
        # one external-grid source; raw (pre-c) subtransient impedance, pu.
        "sources": [{"bus": 0, "sc_r_pu": 0.0099504, "sc_x_pu": 0.0995037}],
    },
    {
        # The key network: two sources feeding a mesh, so the fault current at
        # each bus is a multi-source contribution resolved through the Zbus
        # inversion — a radial test would not exercise that.
        "name": "meshed_two_source",
        "buses": [{"base_kv": 110.0, "type": 3}, {"base_kv": 110.0, "type": 1},
                  {"base_kv": 110.0, "type": 1}, {"base_kv": 110.0, "type": 3}],
        "branches": [{"f": 0, "t": 1, "r_pu": 0.007934, "x_pu": 0.034711},
                     {"f": 1, "t": 2, "r_pu": 0.005289, "x_pu": 0.023141},
                     {"f": 2, "t": 3, "r_pu": 0.009917, "x_pu": 0.043388},
                     {"f": 0, "t": 2, "r_pu": 0.013223, "x_pu": 0.057851},
                     {"f": 1, "t": 3, "r_pu": 0.006612, "x_pu": 0.028926}],
        "sources": [{"bus": 0, "sc_r_pu": 0.006633, "sc_x_pu": 0.066334},
                    {"bus": 3, "sc_r_pu": 0.014799, "sc_x_pu": 0.123328}],
    },
    {
        # Edge cases: a fed pair, an isolated-type bus, and a source-less
        # component. pandapower returns NaN for the last two; the Swift side
        # must report `.isolatedBus` / `.noSourceFeeding`, not a number.
        "name": "edge_cases",
        "buses": [{"base_kv": 110.0, "type": 3}, {"base_kv": 110.0, "type": 1},
                  {"base_kv": 110.0, "type": 4},                       # isolated
                  {"base_kv": 110.0, "type": 1}, {"base_kv": 110.0, "type": 1}],
        "branches": [{"f": 0, "t": 1, "r_pu": 0.004132, "x_pu": 0.016529},
                     {"f": 3, "t": 4, "r_pu": 0.004132, "x_pu": 0.016529}],  # unfed
        "sources": [{"bus": 0, "sc_r_pu": 0.0099504, "sc_x_pu": 0.0995037}],
    },
]


def _sc_build_pp_net(spec: dict):
    net = pp.create_empty_network(sn_mva=SC_BASE_MVA)
    bus_ids = []
    for b in spec["buses"]:
        bus_ids.append(pp.create_bus(net, vn_kv=b["base_kv"]))
    for src in spec["sources"]:
        b = spec["buses"][src["bus"]]
        zbase = b["base_kv"] ** 2 / SC_BASE_MVA
        zmag_pu = (src["sc_r_pu"] ** 2 + src["sc_x_pu"] ** 2) ** 0.5
        # raw (pre-c) source impedance Un^2/Ssc = zmag_pu * zbase -> Ssc, rx.
        s_sc = SC_BASE_MVA / zmag_pu
        rx = src["sc_r_pu"] / src["sc_x_pu"]
        pp.create_ext_grid(net, bus_ids[src["bus"]], s_sc_max_mva=s_sc, rx_max=rx)
    for br in spec["branches"]:
        fb, tb = spec["buses"][br["f"]], spec["buses"][br["t"]]
        zbase = fb["base_kv"] ** 2 / SC_BASE_MVA
        pp.create_line_from_parameters(
            net, bus_ids[br["f"]], bus_ids[br["t"]], length_km=1.0,
            r_ohm_per_km=br["r_pu"] * zbase, x_ohm_per_km=br["x_pu"] * zbase,
            c_nf_per_km=0.0, max_i_ka=1.0)
    return net, bus_ids


def _sc_results(net, bus_ids, fault: str) -> list:
    from pandapower.shortcircuit import calc_sc
    calc_sc(net, fault=fault, case="max", ip=True, ith=True, tk_s=SC_TK_S,
            kappa_method="C")
    df = net.res_bus_sc
    out = []
    for b in bus_ids:
        if b not in df.index or not np.isfinite(df.ikss_ka.at[b]):
            out.append(None)
            continue
        row = {
            "ikss_ka": float(df.ikss_ka.at[b]),
            "ip_ka": float(df.ip_ka.at[b]) if np.isfinite(df.ip_ka.at[b]) else None,
            "ith_ka": float(df.ith_ka.at[b]) if np.isfinite(df.ith_ka.at[b]) else None,
            "rk_ohm": float(df.rk_ohm.at[b]),
            "xk_ohm": float(df.xk_ohm.at[b]),
        }
        # S″k = √3·Un·I″k is a three-phase quantity; pandapower reports a
        # different convention for 2ph, so only carry it for the 3ph oracle.
        if fault == "3ph" and "skss_mw" in df.columns and np.isfinite(df.skss_mw.at[b]):
            row["skss_mva"] = float(df.skss_mw.at[b])
        out.append(row)
    return out


def dump_shortcircuit() -> list:
    networks = []
    for spec in SC_NETWORKS:
        net, bus_ids = _sc_build_pp_net(spec)
        net2, _ = _sc_build_pp_net(spec)      # 2ph on a fresh net
        entry = dict(spec)
        entry["base_mva"] = SC_BASE_MVA
        entry["c"] = SC_C
        entry["tk_s"] = SC_TK_S
        entry["freq_hz"] = SC_FREQ_HZ
        entry["results"] = {
            "3ph": _sc_results(net, bus_ids, "3ph"),
            "2ph": _sc_results(net2, bus_ids, "2ph"),
        }
        networks.append(entry)
    return networks


# --------------------------------------------------------------------------- #
# Time-series sweep oracle (quasi-static). A base IEEE case driven by an        #
# N-step load-multiplier profile: each step scales all loads and solves — the   #
# numerically identical per-step calculation pandapower's time-series module    #
# (ConstControl load scaling + run_timeseries) performs. Warm- vs flat-start is #
# an iteration-count difference only, so the per-step converged solution is the #
# oracle regardless of start. The Swift sweep reads case14's base loads,        #
# applies the same multipliers, warm-starts each step, and must match these.    #
# --------------------------------------------------------------------------- #
TIMESERIES_BASE = "case14"
TIMESERIES_MULTIPLIERS = [0.65, 0.80, 0.95, 1.00, 1.10, 1.25, 1.40, 1.20, 0.90, 0.70]


def dump_timeseries() -> dict:
    make_net = CASES[TIMESERIES_BASE]
    steps = []
    for m in TIMESERIES_MULTIPLIERS:
        net = make_net()
        net.load.p_mw = net.load.p_mw * m
        net.load.q_mvar = net.load.q_mvar * m
        pp.runpp(net, enforce_q_lims=False, calculate_voltage_angles=True,
                 init="flat", tolerance_mva=1e-10)
        _check_ppc_is_identity_mapped(net)
        sol = dump_solution(net)
        sol["multiplier"] = m
        steps.append(sol)
    return {"base_case": TIMESERIES_BASE,
            "multipliers": list(TIMESERIES_MULTIPLIERS),
            "steps": steps}


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

    # Run 3: DC power flow (Piece 2 oracle). Additive — the AC keys above are
    # untouched. Same ppc-level, identity-mapped dump as the AC runs.
    net_dc = make_net()
    pp.rundcpp(net_dc)
    _check_ppc_is_identity_mapped(net_dc)
    doc["dc"] = dump_dc(net_dc)

    # Run 4: N-1 contingency (Piece 3 oracle) — LODF plus post-contingency DC
    # flows from repeated rundcpp. Additive; the AC/DC keys above are untouched.
    doc["contingency"] = dump_contingency(net_dc, name)

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
        cg = doc["contingency"]
        print(f"{name}: {nb} buses, {nbr} branches, Ybus nnz={nnz}, "
              f"NR iterations={it}, DC dumped, "
              f"{len(cg['outages'])} outages ({len(cg['islanding_branches'])} islanding) "
              f"-> {out.relative_to(HERE.parent)}")

    # Piece 4: short-circuit reference networks (separate file — the IEEE case
    # JSONs carry no short-circuit data).
    sc = {"pandapower_version": pp.__version__, "networks": dump_shortcircuit()}
    sc_out = OUT_DIR / "shortcircuit.json"
    sc_out.write_text(json.dumps(sc, indent=1))
    print(f"shortcircuit: {len(sc['networks'])} networks "
          f"({', '.join(n['name'] for n in sc['networks'])}) "
          f"-> {sc_out.relative_to(HERE.parent)}")

    # Time-series sweep reference (separate file; additive).
    ts = {"pandapower_version": pp.__version__, **dump_timeseries()}
    ts_out = OUT_DIR / "timeseries.json"
    ts_out.write_text(json.dumps(ts, indent=1))
    print(f"timeseries: base {ts['base_case']}, {len(ts['steps'])} steps "
          f"-> {ts_out.relative_to(HERE.parent)}")


if __name__ == "__main__":
    sys.exit(main())
