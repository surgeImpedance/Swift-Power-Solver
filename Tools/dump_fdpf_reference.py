"""
Dump the FDPF (fast-decoupled power flow) references for SwiftPowerSolver.

Two kinds of fixture, both additive to the existing dump_reference.py output:

1. SLIM case dumps (fdpf_case30.json, fdpf_case300.json): ppc-level
   parameters plus the plain solved voltages only — no Ybus triplets, no
   contingency section. These exist because the FDPF correctness gate wants
   case30/case300 coverage without paying the full dump price in checked-in
   bytes (case300's exhaustive contingency dump alone would dwarf every other
   fixture). The Swift FDPF gate compares FDPF against the Swift NR solution;
   the pandapower voltages here guard the fixture conversion itself.

2. makeB oracles (fdpf_makeb.json): pypower's own B′/B″ matrices for case14
   and case118, both algorithms (XB and BX), as sparse triplets. The Swift
   B-matrix construction is verified entry-for-entry against these — makeB.m
   semantics are the reference, per the FDPF spec.

Run with the same environment as dump_reference.py:

  python Tools/dump_fdpf_reference.py

Output goes to Tests/SwiftPowerSolverTests/Reference/ and is checked in.
"""
from __future__ import annotations

import json
import pathlib
import sys

import pandapower as pp
import pandapower.networks as pn
from pandapower.pypower.makeB import makeB

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from dump_reference import _check_ppc_is_identity_mapped, dump_params, dump_solution  # noqa: E402

OUT_DIR = HERE.parent / "Tests" / "SwiftPowerSolverTests" / "Reference"

SLIM_CASES = {
    "case30": pn.case30,
    "case300": pn.case300,
}

MAKEB_CASES = {
    "case14": pn.case14,
    "case118": pn.case118,
}

# pypower PF_ALG codes: 2 = XB (fdxb), 3 = BX (fdbx).
ALGS = {"xb": 2, "bx": 3}


def dump_slim(name: str, make_net) -> dict:
    net = make_net()
    pp.runpp(net, enforce_q_lims=False, calculate_voltage_angles=True,
             init="flat", tolerance_mva=1e-10)
    _check_ppc_is_identity_mapped(net)
    doc = {"name": name, "pandapower_version": pp.__version__}
    doc.update(dump_params(net))
    sol = dump_solution(net)
    doc["solution"] = {
        "vm_pu": sol["vm_pu"],
        "va_deg": sol["va_deg"],
        "iterations": sol["iterations"],
    }
    return doc


def triplets(mat) -> dict:
    coo = mat.tocoo()
    coo.sum_duplicates()
    return {
        "row": [int(i) for i in coo.row],
        "col": [int(j) for j in coo.col],
        "v": [float(v) for v in coo.data],
    }


def dump_makeb(name: str, make_net) -> dict:
    net = make_net()
    pp.runpp(net, enforce_q_lims=False, calculate_voltage_angles=True,
             init="flat", tolerance_mva=1e-10)
    _check_ppc_is_identity_mapped(net)
    ppc = net._ppc
    entry = {"name": name, "n": int(ppc["bus"].shape[0])}
    for key, alg in ALGS.items():
        bp, bpp = makeB(ppc["baseMVA"], ppc["bus"], ppc["branch"], alg)
        entry[key] = {"bp": triplets(bp), "bpp": triplets(bpp)}
    return entry


def fdpf_sanity(name: str, make_net) -> None:
    """Informational only (not stored): does pandapower's own FDPF converge on
    this case from flat start? Sets expectations for the Swift gate."""
    for alg in ("fdxb", "fdbx"):
        net = make_net()
        try:
            pp.runpp(net, algorithm=alg, enforce_q_lims=False,
                     calculate_voltage_angles=True, init="flat",
                     tolerance_mva=1e-6, max_iteration=60)
            its = net._ppc.get("iterations", -1)
            print(f"  pandapower {alg} on {name}: converged, iterations={its}")
        except Exception as e:  # noqa: BLE001
            print(f"  pandapower {alg} on {name}: DID NOT CONVERGE ({type(e).__name__})")


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for name, make_net in SLIM_CASES.items():
        doc = dump_slim(name, make_net)
        out = OUT_DIR / f"fdpf_{name}.json"
        out.write_text(json.dumps(doc, indent=1))
        print(f"fdpf_{name}: {len(doc['buses'])} buses, {len(doc['branches'])} branches, "
              f"NR iterations={doc['solution']['iterations']} -> {out.relative_to(HERE.parent)}")
        fdpf_sanity(name, make_net)

    mb = {"pandapower_version": pp.__version__,
          "cases": [dump_makeb(name, make_net) for name, make_net in MAKEB_CASES.items()]}
    out = OUT_DIR / "fdpf_makeb.json"
    out.write_text(json.dumps(mb, indent=1))
    for c in mb["cases"]:
        print(f"makeB {c['name']}: n={c['n']}, "
              f"xb bp/bpp nnz={len(c['xb']['bp']['v'])}/{len(c['xb']['bpp']['v'])}, "
              f"bx bp/bpp nnz={len(c['bx']['bp']['v'])}/{len(c['bx']['bpp']['v'])}")
    print(f"-> {out.relative_to(HERE.parent)}")


if __name__ == "__main__":
    sys.exit(main())
