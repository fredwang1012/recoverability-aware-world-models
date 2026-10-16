"""
Phase D: boundary mining INSIDE a physical case family (ask/tell algorithms over theta in [0,1]^d).

    python scripts/mine_cases.py --family corridor --backend mock  --algos sobol,straddle,bisect --budget 60 --seeds 3
    python scripts/mine_cases.py --family rc03     --backend carla --port 2000 ...          (needs a live, bring-up-passed server)

Search sees G_search (finite, clipped); labels come from the full ExecutionRecord.  Every row is checkpointed to
results/<backend>/mining/<family>/records_<algo>_s<seed>.jsonl and a rerun resumes (skips finished scenes).
Mock and CARLA outputs never share a directory.  Matched pairs (bisect) keep ONE varied factor, the same split_group, and both
members are re-labelled with the full record.

Summary: per-algorithm unrecoverable-scene count, near-boundary count (|V_search| < 0.3), boundary distance (analytic where
the family has an anchor), pairs, evidence mix.
"""
import argparse
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "mining"))
import numpy as np
import pandas as pd

import algos
import margin as M
from families import FAMILIES, mock_ctx

EDGE = 0.3


def search_value(rec):
    """V_search = max_j min_w G_search (finite).  Missing entries are skipped (never treated as 0)."""
    best = -np.inf
    for row in rec.R_norm:
        vals = [x for x in row if x is not None]
        if len(vals) == len(row):
            best = max(best, min(vals))
    return float(best) if np.isfinite(best) else -1.0


def make_ctx(args):
    cfg = M.DEFAULT_V2
    if args.backend == "mock":
        return mock_ctx(cfg)
    from sim.bringup import connect, make_plant
    prof = json.load(open("results/carla/physics_profile.json"))
    client, world, site = connect(args.host, args.port, args.town)
    return {"backend": "carla", "scope": "carla_finite_replay", "cfg": cfg,
            "factory": lambda th: make_plant(client, world, site, decel=th.get("b_mps2")),
            "runtime": {"physics_profile": prof["id"], "map": world.get_map().name, "port": args.port}}


def run_one(fam, name, budget, seed, ctx, outdir, batch=4):
    path = f"{outdir}/records_{name}_s{seed}.jsonl"
    done = {}
    if os.path.exists(path):
        for line in open(path):
            r = json.loads(line)
            done[json.dumps(r["_u"])] = r
    rng = np.random.default_rng(seed)
    algo = algos.REGISTRY[name](rng, d=fam.d)
    rows, out = [], open(path, "a")
    cache = dict(done)
    while len(rows) < budget:
        n = min(batch, budget - len(rows))
        U = np.clip(algo.ask(n), 0, 1)[:n]
        new = []
        for u in U:
            key = json.dumps([float(x) for x in u])
            if key in cache:
                r = cache[key]
            else:
                th = fam.decode(u)
                rec = fam.run(th, ctx)
                a = fam.anchor(th, ctx)
                r = rec.to_json()
                r["_u"] = [float(x) for x in u]
                r["_Vsearch"] = search_value(rec)
                r["_anchor_G_m"] = a
                r["_algo"], r["_seed"] = name, seed
                out.write(json.dumps(r) + "\n"); out.flush()
                cache[key] = r
            m1, m2 = fam.descriptors(r["theta"])
            new.append({"u": r["_u"], "V": r["_Vsearch"], "m1": m1, "m2": m2, "rec": r})
        algo.tell(U, new)
        rows += new
    out.close()
    return rows, algo


def relabel_pairs(fam, algo, ctx, outdir, name, seed):
    pairs = getattr(algo, "pairs", [])
    res = []
    for i, p in enumerate(pairs):
        pid, split = f"{fam.name}_{name}_s{seed}_pair{i:03d}", f"{fam.name}_{name}_s{seed}_pair{i:03d}"
        members = {}
        for role, key in (("unrecoverable_side", "u_unrecoverable"), ("recoverable_side", "u_recoverable")):
            th = fam.decode(p[key])
            rec = fam.run(th, ctx)
            rec.parent_family_id, rec.varied_factor, rec.split_group = pid, fam.names[p["varied_index"]], split
            rec.finalize(ctx["cfg"].eps)
            members[role] = rec.to_json()
        res.append({"pair_id": pid, "varied": fam.names[p["varied_index"]], **members})
    if res:
        with open(f"{outdir}/pairs_{name}_s{seed}.jsonl", "w") as f:
            for r in res:
                f.write(json.dumps(r) + "\n")
    return res


def summarise(fam, ctx, args, outdir):
    L = []
    for name in args.algos.split(","):
        for seed in range(args.seeds):
            rows, algo = run_one(fam, name, args.budget, seed, ctx, outdir)
            pairs = relabel_pairs(fam, algo, ctx, outdir, name, seed) if name == "bisect" else []
            recs = [r["rec"] for r in rows]
            ev = [r["evidence_class"] for r in recs]
            anch = [abs(r["_anchor_G_m"]) for r in recs if r.get("_anchor_G_m") is not None]
            pair_ok = sum(1 for p in pairs if p["unrecoverable_side"]["evidence_class"] != "R_REF"
                          and p["recoverable_side"]["evidence_class"] == "R_REF")
            L.append({"algo": name, "seed": seed, "unrecoverable(U_*)": sum(e.startswith("U_") for e in ev),
                      "R_REF": ev.count("R_REF"), "UNRESOLVED": ev.count("UNRESOLVED"), "INVALID_RUN": ev.count("INVALID_RUN"),
                      "near_boundary": sum(abs(r["V"]) < EDGE for r in rows),
                      "boundary_dist_m": float(np.mean(sorted(anch)[:20])) if anch else np.nan,
                      "pairs": len(pairs), "pairs_verified": pair_ok})
    return pd.DataFrame(L)


def md(df, fam, ctx, args):
    g = df.drop(columns="seed").groupby("algo")
    m, n = g.mean(numeric_only=True), df.seed.nunique()
    ci = 1.96 * g.std(numeric_only=True) / math.sqrt(n) if n > 1 else m * 0
    cols = list(m.columns)
    head = (f"## {fam.name} mining - backend: {ctx['backend'].upper()}"
            f"{' (MOCK plant: kinematic stand-in, NOT CARLA)' if ctx['backend'] == 'mock' else ''}\n\n"
            f"{n} seed(s) x {args.budget} scenes per algorithm; mean +/- 95% CI. near_boundary = |V_search| < {EDGE}; "
            f"boundary_dist_m = mean |analytic G| of the 20 scenes closest to the boundary (corridor only).\n\n")
    L = ["| algo | " + " | ".join(cols) + " |", "|---|" + "---|" * len(cols)]
    for a in m.index:
        L.append(f"| {a} | " + " | ".join("n/a" if np.isnan(m.loc[a, c]) else f"{m.loc[a, c]:.2f} +/- {0 if np.isnan(ci.loc[a, c]) else ci.loc[a, c]:.2f}" for c in cols) + " |")
    return head + "\n".join(L) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--family", choices=list(FAMILIES), required=True)
    ap.add_argument("--backend", choices=["mock", "carla"], default="mock")
    ap.add_argument("--algos", default="sobol,straddle,bisect")
    ap.add_argument("--budget", type=int, default=60)
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--town", default="custom_straight")
    args = ap.parse_args()
    fam, ctx = FAMILIES[args.family], make_ctx(args)
    outdir = f"results/{args.backend}/mining/{fam.name}"
    os.makedirs(outdir, exist_ok=True)
    t0 = time.time()
    df = summarise(fam, ctx, args, outdir)
    df.to_csv(f"{outdir}/summary.csv", index=False)
    text = md(df, fam, ctx, args)
    open(f"{outdir}/summary.md", "w", encoding="utf-8").write(text)
    print(text, f"\n({time.time() - t0:.0f} s)")


if __name__ == "__main__":
    main()
