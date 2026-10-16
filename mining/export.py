"""T7: export mined scenes to the shared JSONL schema (Approach 1 <-> Kent / Andy / Victor).

Every exported scene is re-labelled with the FULL recovery profile R(h) (no pruning), so no entry is 'unresolved'
unless a backend failure leaves it so.  'Unrecoverable' is relative to the library, worlds, horizon and eps recorded in
each record - never a physical-impossibility claim.  Cosmos appearance variants must set parent_id and
variant_kind='appearance' and inherit theta and R unchanged.

Usage:  python export.py --run ../results/mock_run --algos sobol,straddle,bisect --out ../results/dataset
"""
import argparse
import hashlib
import json
import os
from dataclasses import asdict

import numpy as np
import pandas as pd

import core

SCHEMA_VERSION = 1
KEEP_CATS = ("unrecoverable", "edge", "planner_failure")


def scene_id(family, theta):
    key = json.dumps({k: round(float(v), 6) for k, v in sorted(theta.items())}) + family
    return "s_" + hashlib.sha1(key.encode()).hexdigest()[:12]


def evidence(profile):
    """verified_witness: r_j >= eps (a policy that succeeds in ALL worlds).  counterexample: r_j < eps (some world
    defeats the policy).  unresolved: r_j missing/NaN (never invented)."""
    out = {}
    for name, r in profile.items():
        if r is None or not np.isfinite(r):
            out[name] = "unresolved"
        else:
            out[name] = "verified_witness" if r >= core.EPS else "counterexample"
    return out


def trace_npz(backend, theta, row, path):
    """Ego/other traces of the best policy in every world (observation-side reconstruction aid)."""
    best = next(P for P in core.LIBRARY if P.name == row["best_policy"])
    arrays = {}
    for w in core.WORLDS:
        tr = backend.rollout(theta, w, best())
        arrays[f"{w}__ego"] = np.array(tr["ego"], dtype=float)
        for n, st in tr["others"].items():
            arrays[f"{w}__{n}"] = np.array(st, dtype=float)
    np.savez_compressed(path, **arrays)


def make_record(backend, u, cfg, family, seed, pair=None, source="sim_mining"):
    theta = core.decode(np.asarray(u))
    row = core.oracle(backend, theta, full=True, cfg=cfg)
    profile = {P.name: row[f"r_{P.name}"] for P in core.LIBRARY}
    sid = scene_id(family, theta)
    rec = {
        "schema_version": SCHEMA_VERSION, "id": sid, "source": source, "backend": backend.name,
        "family": family, "theta": theta, "u": [float(x) for x in u], "seed": seed,
        "world_set": list(core.WORLDS), "policy_library": [P.name for P in core.LIBRARY],
        "horizon_s": core.HORIZON_S, "eps": core.EPS, "edge_band": core.EDGE_BAND,
        "R": profile, "V": row["V"], "best_policy": row["best_policy"], "planner_margin": row["planner_margin"],
        "category": row["category"], "evidence": evidence(profile),
        "evidence_class": row["evidence"], "V_hind": row["V_hind"], "info_gap": row["info_gap"],
        "R_matrix": row["R"], "worlds": row["worlds"],
        "t_reveal": theta.get("t_reveal"), "margin_config": asdict(cfg),
        "descriptors": {"m1": row["m1"], "m2": row["m2"]},
        "pair_id": pair["pair_id"] if pair else None, "pair_role": pair["role"] if pair else None,
        "varied_factor": pair["varied"] if pair else None,
        "parent_id": None, "variant_kind": None,
        "trace_path": f"traces/{sid}.npz",
        "note": "unrecoverable = relative to policy_library, world_set, horizon_s, eps; not physical impossibility",
    }
    return rec, theta


def _key(u):
    return json.dumps([round(float(x), 9) for x in u])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True, help="run.py output dir with samples_<algo>.csv (+ matched_pairs.json)")
    ap.add_argument("--algos", default="sobol,cem,straddle,bisect,qd_boundary")
    ap.add_argument("--out", default="dataset")
    ap.add_argument("--family", choices=list(core.FAMILIES), default="two_lane")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--all", action="store_true", help="also export safe scenes")
    a = ap.parse_args()
    core.use_family(a.family)
    os.makedirs(f"{a.out}/traces", exist_ok=True)
    be, cfg = core.MockBackend(), core.DEFAULT_CFG
    us = {}
    for al in a.algos.split(","):
        p = f"{a.run}/samples_{al}.csv"
        if os.path.exists(p):
            for r in pd.read_csv(p).to_dict("records"):
                if a.all or r["category"] in KEEP_CATS:
                    u = json.loads(r["u"])
                    us[_key(u)] = u
    pair_of = {}
    pp = f"{a.run}/matched_pairs.json"
    if os.path.exists(pp):
        for i, p in enumerate(json.load(open(pp))):
            pid = f"pair_{i:03d}"
            for role, key in (("unrecoverable", "u_unrecoverable"), ("recoverable", "u_recoverable")):
                if key in p:
                    us[_key(p[key])] = p[key]
                    pair_of[_key(p[key])] = {"pair_id": pid, "role": role, "varied": core.NAMES[p["varied_index"]]}
    # distinct u can decode to the same theta (reaction_delay / t_reveal are quantised): one record per scene id,
    # keeping pair metadata if any member of the collision carries it
    recs = {}
    for k, u in us.items():
        rec, theta = make_record(be, u, cfg, a.family, a.seed, pair_of.get(k))
        if rec["id"] not in recs or (rec["pair_id"] and not recs[rec["id"]][0]["pair_id"]):
            recs[rec["id"]] = (rec, theta)
    n = 0
    with open(f"{a.out}/scenes.jsonl", "w") as f:
        for rec, theta in recs.values():
            trace_npz(be, theta, rec, f"{a.out}/{rec['trace_path']}")
            f.write(json.dumps(rec) + "\n")
            n += 1
    print(f"exported {n} scenes ({len(pair_of)} pair members) to {a.out}/scenes.jsonl")


if __name__ == "__main__":
    main()
