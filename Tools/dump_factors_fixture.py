"""Emit the network-only fixture `FactorsIdentityTests` consumes.

The bit-identity gate reads 1-20 MB fixtures through SPS_FACTORS_CASES and
SKIPS when they are absent — which is how the gate CLAUDE.md names as
never-regress came to be unrunnable in a fresh checkout. This regenerates them.

Field conventions are lifted verbatim from `dump_reference.py:dump_params`;
the two must not drift, because the goldens were recorded against that shape.

    python Tools/dump_factors_fixture.py case300 case1354pegase > /tmp/x.json
    python Tools/dump_factors_fixture.py --out-dir /tmp case300
"""
import argparse
import json
import os
import sys

import numpy as np
import pandapower as pp
import pandapower.networks as pn
from pandapower.pypower.idx_bus import BUS_I, BUS_TYPE, PD, QD, GS, BS, BASE_KV
from pandapower.pypower.idx_brch import (F_BUS, T_BUS, BR_R, BR_X, BR_B, BR_STATUS,
                                         TAP, SHIFT)
from pandapower.pypower.idx_gen import GEN_BUS, PG, QMAX, QMIN, VG, GEN_STATUS

try:
    from pandapower.pypower.idx_brch import BR_G
except ImportError:
    BR_G = None


def build_ppc(net):
    """Populate net._ppc. AC first (what dump_reference.py's params come from);
    fall back to DC and SAY SO, because a DC-built ppc is not guaranteed to
    carry the same R/B columns."""
    try:
        pp.runpp(net)
        return "runpp"
    except Exception as exc:            # noqa: BLE001 - reported, not swallowed
        print(f"  runpp failed ({type(exc).__name__}: {exc}); falling back to rundcpp",
              file=sys.stderr)
        pp.rundcpp(net)
        return "rundcpp"


def dump(name):
    net = getattr(pn, name)()
    how = build_ppc(net)
    ppc = net._ppc

    lookup = net._pd2ppc_lookups["bus"]
    n_pp = len(net.bus)
    identity = np.array_equal(lookup[:n_pp], np.arange(n_pp))

    buses = [{
        "i": int(row[BUS_I]),
        "type": int(row[BUS_TYPE]),
        "pd_mw": float(row[PD]),
        "qd_mvar": float(row[QD]),
        "gs_mw": float(row[GS]),
        "bs_mvar": float(row[BS]),
        "base_kv": float(row[BASE_KV]),
    } for row in ppc["bus"]]

    cols = [BR_R, BR_X, BR_B, TAP, SHIFT] + ([BR_G] if BR_G is not None else [])
    par = ppc["branch"][:, cols]
    if np.max(np.abs(np.imag(par))) > 1e-12:
        raise AssertionError("ppc branch parameters have imaginary parts")

    branches = [{
        "f": int(row[F_BUS].real),
        "t": int(row[T_BUS].real),
        "r": float(row[BR_R].real),
        "x": float(row[BR_X].real),
        "b": float(row[BR_B].real),
        "g": float(row[BR_G].real) if BR_G is not None else 0.0,
        "tap": float(row[TAP].real),
        "shift_deg": float(row[SHIFT].real),
        "status": int(row[BR_STATUS].real),
    } for row in ppc["branch"]]

    slack_va = {int(r["bus"]): float(r["va_degree"]) for _, r in net.ext_grid.iterrows()}
    gens = [{
        "bus": int(row[GEN_BUS]),
        "pg_mw": float(row[PG]),
        "qmax_mvar": float(row[QMAX]),
        "qmin_mvar": float(row[QMIN]),
        "vg_pu": float(row[VG]),
        "va_deg": slack_va.get(int(row[GEN_BUS]), 0.0),
        "status": int(row[GEN_STATUS]),
    } for row in ppc["gen"]]

    nshift = sum(1 for b in branches if abs(b["shift_deg"]) > 0)
    print(f"  {name}: via {how}, buses={len(buses)} branches={len(branches)} "
          f"gens={len(gens)} shifters={nshift} ppc_identity={identity} "
          f"pandapower={pp.__version__}", file=sys.stderr)

    # The Swift fixture name keys the golden table; strip the pegase suffix the
    # golden table does not use.
    short = name.replace("pegase", "")
    return {
        "name": short,
        "pandapower_version": pp.__version__,
        "base_mva": float(ppc["baseMVA"]),
        "buses": buses,
        "branches": branches,
        "gens": gens,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cases", nargs="+")
    ap.add_argument("--out-dir", default=None)
    args = ap.parse_args()
    for case in args.cases:
        doc = dump(case)
        if args.out_dir:
            path = os.path.join(args.out_dir, f"factors_{doc['name']}.json")
            with open(path, "w") as fh:
                json.dump(doc, fh)
            print(path)
        else:
            json.dump(doc, sys.stdout)


if __name__ == "__main__":
    main()
