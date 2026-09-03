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


# --------------------------------------------------------------------------- #
# Distributed-slack oracle. Purpose-built networks (the IEEE cases are single-  #
# slack): an ext_grid (angle reference) plus generators, each carrying a        #
# slack_weight, solved with distributed_slack=True. The imbalance is shared     #
# across contributors proportional to normalized weight. We dump the ppc-level  #
# per-unit inputs, the raw per-generator weights, and the solution (per-bus     #
# vm/va and per-generator P) so the Swift solve sees identical data.            #
# --------------------------------------------------------------------------- #
DSLACK_BASE_MVA = 100.0

# name, buses [(vn_kv,)], ext_grid (bus, vm, weight), gens [(bus, p_mw, vm,
# weight)], loads [(bus, p_mw, q_mvar)], lines [(f, t, len_km)].
DSLACK_NETWORKS = [
    {
        "name": "grid_two_gen",           # ext_grid weight 2, gens weight 1,1 (non-uniform)
        "buses": [110.0, 110.0, 110.0, 110.0],
        "ext_grid": (0, 1.02, 2.0),
        "gens": [(1, 50.0, 1.01, 1.0), (2, 30.0, 1.00, 1.0)],
        "loads": [(3, 120.0, 40.0), (1, 20.0, 5.0)],
        "lines": [(0, 1), (1, 2), (2, 3), (0, 3)],
    },
    {
        "name": "uniform_three",          # all contributors equal weight -> even split
        "buses": [220.0, 220.0, 220.0, 220.0],
        "ext_grid": (0, 1.03, 1.0),
        "gens": [(1, 40.0, 1.02, 1.0), (3, 40.0, 1.02, 1.0)],
        "loads": [(2, 150.0, 45.0)],
        "lines": [(0, 1), (1, 2), (2, 3), (3, 0), (0, 2)],
    },
]


def _build_dslack_net(spec: dict):
    net = pp.create_empty_network(sn_mva=DSLACK_BASE_MVA)
    b = [pp.create_bus(net, vn_kv=vn) for vn in spec["buses"]]
    eg_bus, eg_vm, eg_w = spec["ext_grid"]
    pp.create_ext_grid(net, b[eg_bus], vm_pu=eg_vm, slack_weight=eg_w)
    for gb, p, vm, w in spec["gens"]:
        pp.create_gen(net, b[gb], p_mw=p, vm_pu=vm, slack_weight=w)
    for lb, p, q in spec["loads"]:
        pp.create_load(net, b[lb], p_mw=p, q_mvar=q)
    for f, t in spec["lines"]:
        pp.create_line_from_parameters(net, b[f], b[t], length_km=10,
                                       r_ohm_per_km=0.05, x_ohm_per_km=0.30,
                                       c_nf_per_km=0.0, max_i_ka=1.0)
    return net


def dump_distributed_slack() -> list:
    networks = []
    for spec in DSLACK_NETWORKS:
        net = _build_dslack_net(spec)
        pp.runpp(net, distributed_slack=True, calculate_voltage_angles=True,
                 init="flat", tolerance_mva=1e-10)
        _check_ppc_is_identity_mapped(net)
        params = dump_params(net)   # buses / branches (ppc-level pu inputs)

        # Generators in ppc order (ext_grid first, then gens). Setpoints come
        # from the net inputs, NOT ppc PG (which holds the distributed result).
        gens = [{"bus": int(net.ext_grid.bus.iloc[0]), "p_set_mw": 0.0,
                 "vm_pu": float(net.ext_grid.vm_pu.iloc[0]),
                 "slack_weight": float(net.ext_grid.slack_weight.iloc[0])}]
        for i in range(len(net.gen)):
            gens.append({"bus": int(net.gen.bus.iloc[i]),
                         "p_set_mw": float(net.gen.p_mw.iloc[i]),
                         "vm_pu": float(net.gen.vm_pu.iloc[i]),
                         "slack_weight": float(net.gen.slack_weight.iloc[i])})
        # Per-generator solved P, same order.
        gen_p = [float(net.res_ext_grid.p_mw.iloc[0])] + \
                [float(net.res_gen.p_mw.iloc[i]) for i in range(len(net.gen))]

        networks.append({
            "name": spec["name"],
            "base_mva": params["base_mva"],
            "buses": params["buses"],
            "branches": params["branches"],
            "gens": gens,
            "solution": {
                "vm_pu": [float(v) for v in net._ppc["bus"][:, VM]],
                "va_deg": [float(v) for v in net._ppc["bus"][:, VA]],
                "gen_p_mw": gen_p,
            },
        })
    return networks


# --------------------------------------------------------------------------- #
# Distributed slack WITH generator P limits (regulating range).                 #
#                                                                               #
# pandapower CANNOT be called directly for this: runpp ignores min_p_mw /       #
# max_p_mw entirely. Verified two ways on pandapower 3.2.1 —                    #
#   * empirically: distributed_slack=True drives a gen to 60.03 MW against a    #
#     max_p_mw of 21;                                                           #
#   * by source: pypower/newtonpf.py, the distributed-slack implementation,     #
#     contains no reference to max_p_mw, min_p_mw or any enforce_p flag.        #
#                                                                               #
# So the pin-and-redistribute convention is the Swift package's own spec, and   #
# pandapower is used as the oracle by NETWORK EQUIVALENCE instead: a converged  #
# pinned solution is identical to an UNLIMITED distributed-slack problem in     #
# which each pinned unit is fixed at its limit with zero participation and the  #
# survivors' weights renormalize. Every line below is therefore a genuine       #
# pandapower solve — the cascade is driven here, but each level is solved by    #
# pandapower, not by us.                                                        #
#                                                                               #
# The cascade rule mirrored here (and specified in NewtonRaphson.solve):        #
#   1. solve unlimited distributed slack over the active participants;          #
#   2. pin EVERY participant outside its range, at the violated bound;          #
#   3. renormalize the survivors and re-solve; repeat until no violation;       #
#   4. never pin the last participant — it saturates instead.                   #
# --------------------------------------------------------------------------- #
PLIMIT_NETWORKS = [
    {
        # TWO units pin in TWO cascade levels — the limits are chosen so this
        # cannot collapse into one batch pin:
        #   unlimited      -> g1 76.086 (over its 55), g2 48.043 (UNDER its 50)
        #   after g1 pins  -> g2 52.245 (now over its 50)  => second level
        # g2's max sits strictly between 48.043 and 52.245, so g2 is only pushed
        # over the edge BY g1 pinning. That is the cascade, not a batch.
        "name": "cascade_two_pin",
        "buses": [110.0, 110.0, 110.0, 110.0, 110.0],
        "ext_grid": (0, 1.02, 3.0, None, None),
        "gens": [(1, 40.0, 1.01, 2.0, None, 55.0),
                 (2, 30.0, 1.00, 1.0, None, 50.0),
                 (3, 25.0, 1.00, 1.0, None, None)],
        "loads": [(4, 190.0, 55.0), (2, 30.0, 8.0)],
        "lines": [(0, 1), (1, 2), (2, 3), (3, 4), (0, 4)],
    },
    {
        # A LOWER-limit pin: load falls well below the scheduled dispatch, so
        # the distribution drives units down and g1 hits pMin. The same rule has
        # to work in the decreasing direction — that is AGC backing off.
        "name": "lower_limit_pin",
        "buses": [220.0, 220.0, 220.0, 220.0],
        "ext_grid": (0, 1.03, 1.0, None, None),
        "gens": [(1, 90.0, 1.02, 2.0, 70.0, None),
                 (3, 80.0, 1.02, 1.0, None, None)],
        "loads": [(2, 60.0, 15.0)],
        "lines": [(0, 1), (1, 2), (2, 3), (3, 0), (0, 2)],
    },
]


def _build_plimit_net(spec: dict, fixed: dict):
    """`fixed` maps generator index (0 = ext_grid, 1.. = gens) to the MW it is
    pinned at; a pinned unit gets slack_weight 0 so pandapower renormalizes the
    survivors exactly as the Swift cascade does."""
    net = pp.create_empty_network(sn_mva=DSLACK_BASE_MVA)
    b = [pp.create_bus(net, vn_kv=vn) for vn in spec["buses"]]
    eg_bus, eg_vm, eg_w, _, _ = spec["ext_grid"]
    pp.create_ext_grid(net, b[eg_bus], vm_pu=eg_vm,
                       slack_weight=0.0 if 0 in fixed else eg_w)
    for k, (gb, p, vm, w, _, _) in enumerate(spec["gens"], start=1):
        pp.create_gen(net, b[gb], p_mw=fixed.get(k, p), vm_pu=vm,
                      slack_weight=0.0 if k in fixed else w)
    for lb, p, q in spec["loads"]:
        pp.create_load(net, b[lb], p_mw=p, q_mvar=q)
    for f, t in spec["lines"]:
        pp.create_line_from_parameters(net, b[f], b[t], length_km=10,
                                       r_ohm_per_km=0.05, x_ohm_per_km=0.30,
                                       c_nf_per_km=0.0, max_i_ka=1.0)
    return net


def _plimit_ranges(spec: dict) -> list:
    """(pmin, pmax) per generator in ppc order: ext_grid first, then gens."""
    return [(spec["ext_grid"][3], spec["ext_grid"][4])] + \
           [(g[4], g[5]) for g in spec["gens"]]


def dump_plimit_distributed_slack() -> list:
    networks = []
    for spec in PLIMIT_NETWORKS:
        ranges = _plimit_ranges(spec)
        fixed: dict = {}
        levels = 0
        while True:
            net = _build_plimit_net(spec, fixed)
            pp.runpp(net, distributed_slack=True, calculate_voltage_angles=True,
                     init="flat", tolerance_mva=1e-10)
            solved = [float(net.res_ext_grid.p_mw.iloc[0])] + \
                     [float(net.res_gen.p_mw.iloc[i]) for i in range(len(net.gen))]
            active = [k for k in range(len(solved)) if k not in fixed]
            violators = {}
            for k in active:
                lo, hi = ranges[k]
                if hi is not None and solved[k] > hi + 1e-9:
                    violators[k] = hi
                elif lo is not None and solved[k] < lo - 1e-9:
                    violators[k] = lo
            # Never pin the last participant — it saturates instead.
            if violators and len(violators) == len(active):
                keep = max(active, key=lambda k: (_weight(spec, k), -k))
                violators.pop(keep, None)
            if not violators:
                break
            fixed.update(violators)
            levels += 1
            if levels > len(solved) + 1:
                raise RuntimeError("P-limit cascade did not settle")

        # The SAME network with every limit ignored, solved by pandapower.
        # Two jobs: it is the additive gate (the Swift solve with limits
        # stripped must reproduce it at machine precision), and it proves the
        # limits are what move the answer rather than some other difference.
        unl = _build_plimit_net(spec, {})
        pp.runpp(unl, distributed_slack=True, calculate_voltage_angles=True,
                 init="flat", tolerance_mva=1e-10)
        unlimited = {
            "vm_pu": [float(v) for v in unl._ppc["bus"][:, VM]],
            "va_deg": [float(v) for v in unl._ppc["bus"][:, VA]],
            "gen_p_mw": [float(unl.res_ext_grid.p_mw.iloc[0])] +
                        [float(unl.res_gen.p_mw.iloc[i]) for i in range(len(unl.gen))],
        }

        _check_ppc_is_identity_mapped(net)
        params = dump_params(net)
        gens = [{"bus": int(spec["ext_grid"][0]), "p_set_mw": 0.0,
                 "vm_pu": float(spec["ext_grid"][1]),
                 "slack_weight": float(spec["ext_grid"][2]),
                 "p_min_mw": spec["ext_grid"][3], "p_max_mw": spec["ext_grid"][4]}]
        for gb, p, vm, w, lo, hi in spec["gens"]:
            gens.append({"bus": int(gb), "p_set_mw": float(p), "vm_pu": float(vm),
                         "slack_weight": float(w), "p_min_mw": lo, "p_max_mw": hi})

        networks.append({
            "name": spec["name"],
            "base_mva": params["base_mva"],
            "buses": params["buses"],
            "branches": params["branches"],
            "gens": gens,
            "cascade_levels": levels,
            "pinned_gen_indices": sorted(fixed.keys()),
            "solution": {
                "vm_pu": [float(v) for v in net._ppc["bus"][:, VM]],
                "va_deg": [float(v) for v in net._ppc["bus"][:, VA]],
                "gen_p_mw": solved,
            },
            "unlimited_solution": unlimited,
        })
    return networks


def _weight(spec: dict, k: int) -> float:
    return float(spec["ext_grid"][2]) if k == 0 else float(spec["gens"][k - 1][3])


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


# ---------------------------------------------------------------------------
# Unit 3b — the two fixture-building gates' oracles (6.7, and 6.2/6.15).
#
# BOTH LAND IN NEW FILES. The existing case14/39/118.json are NOT regenerated
# by these: every N-1 golden and bit-identity digest in this repo is keyed to
# their exact bytes (DECISIONS.md D68 §3 — the golden path runs through
# pandapower as well as Accelerate), so re-emitting them to add a key would put
# an unrelated oracle version on the critical path of four gates. Additive
# files cost one extra `Bundle.module.url` and risk nothing.
# ---------------------------------------------------------------------------


def dump_ptdf_reference() -> dict:
    """Gate 6.7 — an EXTERNAL PTDF reference, from pandapower's own makePTDF.

    Why this is not redundant with 6.5's brute-force re-solve: 6.5 validates
    LODF against a re-solve computed by the SAME DC machinery, so it cannot
    see an error shared by both. makePTDF is an independent implementation of
    the matrix itself.

    The mandatory corpus only (D64 §12). case118 is 186x118 = 21,948 doubles,
    which is the largest that keeps a bare clone cheap.
    """
    out = {}
    for name, make_net in CASES.items():
        net = make_net()
        pp.rundcpp(net)
        _check_ppc_is_identity_mapped(net)
        ppc = net._ppc
        H = makePTDF(ppc["baseMVA"], ppc["bus"], ppc["branch"])
        H = np.asarray(H, dtype=float)
        if not np.all(np.isfinite(H)):
            raise AssertionError(f"{name}: makePTDF produced non-finite entries")
        # The reference slack makePTDF used, so the Swift side can ask for the
        # SAME scheme rather than comparing two different conventions and
        # calling the difference an error.
        slacks = [int(b[BUS_I].real) for b in ppc["bus"] if int(b[BUS_TYPE].real) == 3]
        out[name] = {
            "n_bus": int(H.shape[1]),
            "n_branch": int(H.shape[0]),
            "slack_buses": slacks,
            # Reported so the Swift gate can assert a MINIMUM nonzero count and
            # refuse to agree trivially with an all-zero matrix.
            "nonzero_count": int(np.count_nonzero(H)),
            # The count that MEANS something. makePTDF carries hundreds of
            # entries of numerical dust — down to |H| ~ 1e-22 on case118 —
            # where an exact-zero implementation returns 0.0. Comparing raw
            # nonzero counts therefore compares litter, not sensitivities.
            # This counts entries at or above the gate's own tolerance.
            "significant_count": int((np.abs(H) > 1e-9).sum()),
            "max_abs": float(np.abs(H).max()),
            "ptdf": [[float(v) for v in row] for row in H],
        }
        print(f"  ptdf/{name}: {H.shape[0]}x{H.shape[1]}, "
              f"{int(np.count_nonzero(H))} nonzero "
              f"({int((np.abs(H) > 1e-9).sum())} significant), "
              f"max|H| {float(np.abs(H).max()):.6f}, slack {slacks}")
    return out


def _build_shifter_loop():
    """A9 item 1 — the hand-checkable shifter oracle.

    Two buses joined by TWO parallel branches, one of them a transformer with
    a phase shift, and NO load and NO generation anywhere. Net injection is
    zero at every bus, so any flow that exists is a pure circulating flow
    driven by the shift alone — which makes the shift term the ONLY thing the
    fixture measures, rather than a small correction on top of load flow.

    Hand-check: for a DC model the loop carries
        c = shift_rad / (x_line + x_trafo)   [pu on baseMVA]
    in one direction and -c in the other. The gate reports the measured value
    beside this, so "the fixture contains a shifter" is a NUMBER and not a
    belief about how it was built. (A fixture constructed to contain a
    near-bridge in this same milestone turned out to contain its inverse, and
    only an assertion on the measured value caught it.)
    """
    net = pp.create_empty_network(sn_mva=100.0)
    b0 = pp.create_bus(net, vn_kv=110.0, name="A")
    b1 = pp.create_bus(net, vn_kv=110.0, name="B")
    pp.create_ext_grid(net, bus=b0, vm_pu=1.0, va_degree=0.0)
    # Branch 1: a plain line.
    pp.create_line_from_parameters(net, from_bus=b0, to_bus=b1, length_km=1.0,
                                   r_ohm_per_km=0.0, x_ohm_per_km=12.1,
                                   c_nf_per_km=0.0, max_i_ka=10.0)
    # Branch 2: an ideal-ratio transformer carrying the phase shift. 5 degrees
    # is deliberately LARGE — three orders above the pegase shifts — so the
    # circulating flow is unmistakable and the 6.15 mutation cannot be lost in
    # rounding.
    pp.create_transformer_from_parameters(
        net, hv_bus=b0, lv_bus=b1, sn_mva=100.0, vn_hv_kv=110.0, vn_lv_kv=110.0,
        vkr_percent=0.0, vk_percent=10.0, pfe_kw=0.0, i0_percent=0.0,
        shift_degree=5.0)
    return net


def dump_shifter_reference() -> list:
    """Gates 6.2 and 6.15 — the shifter-bearing fixtures.

    NO IEEE CASE IN THIS REPO HAS A PHASE SHIFTER: nshift = 0 on all six, so a
    base-flow-reconstruction gate written against them validates the shift term
    against an empty payload and passes vacuously (D64 §12, A9). Two networks,
    for two different reasons:

      shifter_loop   — one shifter, zero injection, hand-checkable, and the
                       shift term is 100% of the answer.
      case1354pegase — 6 shifters at realistic scale, where the shift term is a
                       correction on top of real load flow. NOT case9241
                       (66 shifters) — too slow for a routine gate (D64 §12).

    Each network reports the MEASURED size of its shift term — the DC flows
    re-solved with every shift zeroed, differenced against the real solve — so
    the payload is a number in the reference itself.
    """
    import copy
    out = []
    for name, make_net in (("shifter_loop", _build_shifter_loop),
                           ("case1354pegase", pn.case1354pegase)):
        net = make_net()
        pp.rundcpp(net)
        _check_ppc_is_identity_mapped(net)
        ppc = net._ppc
        shift = np.asarray([float(r[SHIFT].real) for r in ppc["branch"]])
        nshift = int((shift != 0).sum())
        if nshift == 0:
            raise AssertionError(f"{name}: nshift = 0 — this fixture exists to "
                                 "carry a phase shifter and does not")

        doc = {"name": name, "n_shift": nshift}
        doc.update(dump_params(net))
        doc["dc"] = dump_dc(net)

        # THE PAYLOAD MEASUREMENT. Re-solve with every shift set to zero; the
        # difference IS the shift term's contribution to base flows. A gate
        # whose fixture cannot move when the term is deleted is testing
        # nothing, and this is the number that proves it can.
        # Zeroed at the MODEL level and re-solved as a real network, not by
        # editing ppc in place: a ppc edit measures what the ppc would do, and
        # what the gate cares about is what the network does.
        net0 = make_net()
        if len(net0.trafo):
            net0.trafo["shift_degree"] = 0.0
        if hasattr(net0, "trafo3w") and len(net0.trafo3w):
            for c in ("shift_mv_degree", "shift_lv_degree"):
                if c in net0.trafo3w:
                    net0.trafo3w[c] = 0.0
        pp.rundcpp(net0)
        shift0 = np.asarray([float(r[SHIFT].real) for r in net0._ppc["branch"]])
        if int((shift0 != 0).sum()) != 0:
            raise AssertionError(f"{name}: zeroing shift_degree left "
                                 f"{int((shift0 != 0).sum())} shifted branches — "
                                 "the control did not do what it claims")
        pf = np.asarray([float(r[PF].real) for r in ppc["branch"]])
        pf0 = np.asarray([float(r[PF].real) for r in net0._ppc["branch"]])
        delta = float(np.abs(pf - pf0).max())
        doc["shift_term_max_flow_delta_mw"] = delta
        doc["shift_deg_max_abs"] = float(np.abs(shift).max())
        out.append(doc)
        print(f"  shifter/{name}: {len(doc['buses'])} buses, "
              f"{len(doc['branches'])} branches, n_shift={nshift}, "
              f"max|shift| {float(np.abs(shift).max()):.6f} deg, "
              f"shift term moves base flows by up to {delta:.3f} MW")
    return out


def main() -> None:
    """Sections are selectable so a run that only needs the unit-3b oracles
    cannot rewrite the checked-in case references as a side effect.

        python Tools/dump_reference.py                # everything (default)
        python Tools/dump_reference.py ptdf shifter   # unit 3b only

    Why the selector exists (D68 §3): case14/39/118.json are the input half of
    the N-1 golden and the bit-identity digests. Re-emitting them is a
    deliberate act with a justification owed in the commit message, never a
    side effect of adding an unrelated oracle.
    """
    known = {"cases", "shortcircuit", "timeseries", "distributed_slack",
             "plimits", "ptdf", "shifter"}
    want = set(sys.argv[1:]) or known
    unknown = want - known
    if unknown:
        raise SystemExit(f"unknown section(s): {sorted(unknown)}; "
                         f"known: {sorted(known)}")

    OUT_DIR.mkdir(parents=True, exist_ok=True)

    if "ptdf" in want:
        print("gate 6.7 — external PTDF reference (makePTDF):")
        doc = {"pandapower_version": pp.__version__, "cases": dump_ptdf_reference()}
        out = OUT_DIR / "ptdf.json"
        out.write_text(json.dumps(doc, indent=1))
        print(f"ptdf: {len(doc['cases'])} cases -> {out.relative_to(HERE.parent)}")

    if "shifter" in want:
        print("gates 6.2 / 6.15 — shifter-bearing fixtures:")
        doc = {"pandapower_version": pp.__version__,
               "networks": dump_shifter_reference()}
        out = OUT_DIR / "shifter.json"
        out.write_text(json.dumps(doc, indent=1))
        print(f"shifter: {len(doc['networks'])} networks -> "
              f"{out.relative_to(HERE.parent)}")

    if "cases" not in want:
        return

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

    if "shortcircuit" not in want:
        return

    # Piece 4: short-circuit reference networks (separate file — the IEEE case
    # JSONs carry no short-circuit data).
    sc = {"pandapower_version": pp.__version__, "networks": dump_shortcircuit()}
    sc_out = OUT_DIR / "shortcircuit.json"
    sc_out.write_text(json.dumps(sc, indent=1))
    print(f"shortcircuit: {len(sc['networks'])} networks "
          f"({', '.join(n['name'] for n in sc['networks'])}) "
          f"-> {sc_out.relative_to(HERE.parent)}")

    if "timeseries" not in want:
        return

    # Time-series sweep reference (separate file; additive).
    ts = {"pandapower_version": pp.__version__, **dump_timeseries()}
    ts_out = OUT_DIR / "timeseries.json"
    ts_out.write_text(json.dumps(ts, indent=1))
    print(f"timeseries: base {ts['base_case']}, {len(ts['steps'])} steps "
          f"-> {ts_out.relative_to(HERE.parent)}")

    if "distributed_slack" not in want:
        return

    # Distributed-slack reference networks (separate file; additive).
    ds = {"pandapower_version": pp.__version__, "networks": dump_distributed_slack()}
    ds_out = OUT_DIR / "distributed_slack.json"
    ds_out.write_text(json.dumps(ds, indent=1))
    print(f"distributed_slack: {len(ds['networks'])} networks "
          f"({', '.join(n['name'] for n in ds['networks'])}) "
          f"-> {ds_out.relative_to(HERE.parent)}")

    if "plimits" not in want:
        return

    # Distributed slack WITH generator P limits (separate file; additive).
    # Driven as a cascade of pandapower solves — see the note above
    # PLIMIT_NETWORKS for why pandapower cannot be called directly.
    pl = {"pandapower_version": pp.__version__,
          "networks": dump_plimit_distributed_slack()}
    pl_out = OUT_DIR / "distributed_slack_plimits.json"
    pl_out.write_text(json.dumps(pl, indent=1))
    for netdoc in pl["networks"]:
        print(f"plimits/{netdoc['name']}: {netdoc['cascade_levels']} cascade level(s), "
              f"pinned gens {netdoc['pinned_gen_indices']}")
    print(f"distributed_slack_plimits: {len(pl['networks'])} networks "
          f"-> {pl_out.relative_to(HERE.parent)}")


# ---------------------------------------------------------------------------
# D80 step 3 — the ZIP (voltage-dependent load) oracle.
#
# AN ADDITIVE FILE (`Reference/zip.json`), never a re-emission of
# case14/39/118.json: those bytes are on the critical path of four gates
# (D68 §3), so the ZIP blocks live beside them. Run as
#     python Tools/dump_reference.py zip
#
# Two known asymmetries between the reference tools, measured 2026-09-02
# (substation-lab docs/investigations/zip-oracle-control.md):
#   * pandapower re-evaluates the load each iteration but its Jacobian has no
#     load term (pf/create_jacobian.py: Ibus = zeros), so its ITERATION COUNT
#     is a partial-Newton count (8/13/8 here vs MATPOWER's full-Newton 4/5/5).
#     `iterations` is recorded as provenance and must never be a gate.
#   * pandapower count-averages the coefficients of several loads on one bus
#     (build_bus.py); the mix below is applied UNIFORMLY and one-load-per-bus
#     is asserted, so the bus polynomial is exact either way.
# Converged voltages agree with MATPOWER 8.0 on identical data to ≤ 1e-11 pu
# (case14/39) and to the constant-power export residual on case118.
# ---------------------------------------------------------------------------

# Fractions of P0 / Q0 at V = 1 pu. P = 0.3 Z + 0.3 I + 0.4 P; Q = 0.5 Z + 0.3 I + 0.2 Q.
ZIP_MIX = {"z_p": 0.3, "i_p": 0.3, "z_q": 0.5, "i_q": 0.3}

# Pinned. `max_iteration=60` is REQUIRED for the ZIP runs: pandapower's
# partial Newton needs 13 iterations on case39, above its default of 10.
ZIP_OPTIONS = dict(algorithm="nr", init="flat", tolerance_mva=1e-10, max_iteration=60,
                   calculate_voltage_angles=True, voltage_depend_loads=True)


def _apply_zip(net) -> None:
    assert net.load.bus.value_counts().max() == 1, (
        "one load per bus is the fixture rule: pandapower count-averages the "
        "coefficients of several loads on one bus (build_bus.py)")
    net.load["const_z_p_percent"] = 100.0 * ZIP_MIX["z_p"]
    net.load["const_i_p_percent"] = 100.0 * ZIP_MIX["i_p"]
    net.load["const_z_q_percent"] = 100.0 * ZIP_MIX["z_q"]
    net.load["const_i_q_percent"] = 100.0 * ZIP_MIX["i_q"]


def _consumed_load(net) -> dict:
    """pandapower's own consumed load at the solved voltage (res_load), keyed
    by ppc bus index — the oracle for `PowerFlowSolution.loadPPu`."""
    lookup = net._pd2ppc_lookups["bus"]
    return {
        "bus": [int(lookup[b]) for b in net.load.bus],
        "p_mw": [float(v) for v in net.res_load.p_mw],
        "q_mvar": [float(v) for v in net.res_load.q_mvar],
        "scheduled_p_mw": [float(v) for v in net.load.p_mw],
        "scheduled_q_mvar": [float(v) for v in net.load.q_mvar],
    }


def dump_zip_reference() -> dict:
    doc = {
        "pandapower_version": pp.__version__,
        "coefficients": {**ZIP_MIX,
                         "p_p": 1.0 - ZIP_MIX["z_p"] - ZIP_MIX["i_p"],
                         "q_q": 1.0 - ZIP_MIX["z_q"] - ZIP_MIX["i_q"],
                         "applied_as": "uniform const_{z,i}_{p,q}_percent on every load; "
                                       "one load per bus asserted"},
        "options": {k: v for k, v in ZIP_OPTIONS.items()},
        "cases": {},
    }
    for name, make_net in CASES.items():
        entry = {"constant_power": {}, "zip": {}, "max_abs_dvm_zip_minus_constant_power": {}}
        for q in (False, True):
            key = "q_lims" if q else "default"
            # Constant-power control: EXACTLY the committed dump's call, so the
            # Swift loader can assert this block equals the committed fixture
            # (the regeneration control) — a drift in options shows up as a
            # moved control, not as a moved ZIP answer.
            net = make_net()
            pp.runpp(net, enforce_q_lims=q, calculate_voltage_angles=True,
                     init="flat", tolerance_mva=1e-10)
            _check_ppc_is_identity_mapped(net)
            entry["constant_power"][key] = dump_solution(net)

            netz = make_net()
            _apply_zip(netz)
            pp.runpp(netz, enforce_q_lims=q, **ZIP_OPTIONS)
            _check_ppc_is_identity_mapped(netz)
            sol = dump_solution(netz)
            sol["consumed_load"] = _consumed_load(netz)
            entry["zip"][key] = sol

            # Anti-vacuity at generation time (the Swift loader asserts it
            # again): a ZIP fixture indistinguishable from constant power is
            # a fixture of the container, not the content.
            dv = max(abs(a - b) for a, b in
                     zip(entry["constant_power"][key]["vm_pu"], sol["vm_pu"]))
            entry["max_abs_dvm_zip_minus_constant_power"][key] = dv
            assert dv > 1e-6, f"{name}/{key}: ZIP == constant power ({dv:.3e}); vacuous"
            print(f"zip {name}/{key}: constant-power it={entry['constant_power'][key]['iterations']} "
                  f"zip it={sol['iterations']} max|dVm|={dv:.3e} pu")
        doc["cases"][name] = entry
    return doc


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "zip":
        out = OUT_DIR / "zip.json"
        out.write_text(json.dumps(dump_zip_reference(), indent=1))
        print(f"-> {out}")
        sys.exit(0)
    sys.exit(main())
