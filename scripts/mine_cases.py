"""
Phase D: boundary mining INSIDE a physical case family (ask/tell algorithms over theta in [0,1]^d).

    python scripts/mine_cases.py --family corridor --backend mock  --algos sobol,straddle,bisect --budget 60 --seeds 3
    cd mining && python ../scripts/mine_cases.py --family rc03 --backend carla --port 2000 ...   (needs a live, bring-up-passed server)

Search sees G_search (finite, clipped); labels come from the full ExecutionRecord.  Every row is checkpointed to
results/<backend>/mining/<family>/records_<algo>_s<seed>.jsonl (cwd-relative: run CARLA from mining/ so outputs land in
mining/results/carla/mining/<family>/) and a rerun resumes (skips finished scenes).  Mock and CARLA outputs never share a directory.
On CARLA the family is the CARLA variant (families.FAMILIES_CARLA: calibrated fixed b, a_lat_max, wider rc03 space).
Matched pairs (bisect) keep ONE varied factor, the same split_group, and both members are re-labelled with the full record.

Summary: per-algorithm unrecoverable-scene count, near-boundary count (|V_search| < 0.3), boundary distance (analytic where
the family has an anchor), pairs (pairs.md), evidence mix.  scripts/mining_post.py adds D4 repeats and the figures.
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
from families import FAMILIES, FAMILIES_CARLA, mock_ctx

EDGE = 0.3
FALSIFIERS = ("cem", "bo_lcb", "qd_fail", "qd_boundary")
UNITS = {"_mps2": "m/s^2", "_mps": "m/s", "_m": "m", "_s": "s"}


# ----------------------------------------------------------------------------- pure helpers (mock-tested)
def unit_of(name):
    for suf, u in UNITS.items():
        if name.endswith(suf):
            return u
    return ""


def short(name):
    for suf in UNITS:
        if name.endswith(suf):
            return name[: -len(suf)]
    return name


def describe_pair(varied, th_u, th_r):
    """Human-readable matched-pair description: 'same scene except gap 15.2 m vs 15.6 m' plus the held-fixed factors."""
    u, r, un = th_u[varied], th_r[varied], unit_of(varied)
    others = [k for k in th_u if k != varied and th_u[k] == th_r.get(k)]
    differ = [k for k in th_u if k != varied and th_u[k] != th_r.get(k)]
    fixed = ", ".join(f"{short(k)} {th_u[k]:.4g} {unit_of(k)}".strip() for k in others)
    if abs(u - r) < 1e-12:
        head = f"same scene except {short(varied)}: both members equal {u:.4g} {un} after tick quantisation"
    else:
        head = f"same scene except {short(varied)} {u:.4g} {un} (unrecoverable side) vs {r:.4g} {un} (recoverable side)"
    if differ:
        head += f"  [WARNING: also differs: {differ}]"
    return f"{head}; held fixed: {fixed}"


def gv(x):
    """R_raw JSON value -> float; '-inf' -> -inf; None stays None (unrun, never 0)."""
    return -math.inf if x == "-inf" else (None if x is None else float(x))


def v_raw(rec):
    """Label-grade scene value V = max over policies of min over worlds of R_raw (policy rows with an unrun entry are skipped)."""
    best = None
    for row in rec["R_raw"]:
        vals = [gv(x) for x in row]
        if any(x is None for x in vals):
            continue
        m = min(vals)
        best = m if best is None else max(best, m)
    return best


def label_kind(ev):
    return "R" if ev == "R_REF" else ("U" if ev.startswith("U_") else ev)


def search_value(rec):
    """V_search = max_j min_w G_search (finite).  Missing entries are skipped (never treated as 0)."""
    best = -np.inf
    for row in rec.R_norm:
        vals = [x for x in row if x is not None]
        if len(vals) == len(row):
            best = max(best, min(vals))
    return float(best) if np.isfinite(best) else -1.0


# ----------------------------------------------------------------------------- context
def make_ctx(args):
    cfg = M.DEFAULT_V2
    if args.backend == "mock":
        return mock_ctx(cfg)
    from sim.bringup import connect, make_plant
    from sim.plant_carla import runtime_info
    prof = json.load(open("results/carla/physics_profile.json"))        # run from mining/
    client, world, site = connect(args.host, args.port, args.town)
    rt = dict(runtime_info(client, world, site), physics_profile=prof["id"], port=args.port)
    return {"backend": "carla", "scope": "carla_finite_replay", "cfg": cfg, "pid": prof["id"],
            "factory": lambda th: make_plant(client, world, site, decel=th.get("b_mps2")),
            "runtime": rt, "run_kw": {"physics_profile_id": prof["id"], "runtime": rt}}


def get_family(args):
    return (FAMILIES_CARLA if args.backend == "carla" else FAMILIES)[args.family]


def check_profile(r, ctx, path):
    if ctx["backend"] == "carla" and r.get("physics_profile_id") != ctx["pid"]:
        raise RuntimeError(f"{path}: cached record has physics_profile_id {r.get('physics_profile_id')} != current {ctx['pid']}; "
                           "refusing to mix profiles (move the old file aside)")


# ----------------------------------------------------------------------------- run
def run_one(fam, name, budget, seed, ctx, outdir, batch=4, bisect_iters=None):
    path = f"{outdir}/records_{name}_s{seed}.jsonl"
    done = {}
    if os.path.exists(path):
        for line in open(path):
            r = json.loads(line)
            check_profile(r, ctx, path)
            done[json.dumps(r["_u"])] = r
    rng = np.random.default_rng(seed)
    kw = {"iters": bisect_iters} if (name == "bisect" and bisect_iters) else {}
    algo = algos.REGISTRY[name](rng, d=fam.d, **kw)
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
                sv = getattr(fam, "search_value", None)             # family-specific search surrogate (D5); default = V_search
                r["_Vsearch"] = float(sv(rec, name)) if sv else search_value(rec)
                r["_anchor_G_m"] = a
                r["_algo"], r["_seed"] = name, seed
                r.update(fam.meta())
                out.write(json.dumps(r) + "\n"); out.flush()
                cache[key] = r
            m1, m2 = fam.descriptors(r["theta"])
            new.append({"u": r["_u"], "V": r["_Vsearch"], "m1": m1, "m2": m2, "rec": r})
        algo.tell(U, new)
        rows += new
        print(f"  {fam.name} {name} s{seed}: {len(rows)}/{budget}", flush=True)
    out.close()
    return rows, algo


def relabel_pairs(fam, algo, ctx, outdir, name, seed):
    pairs = getattr(algo, "pairs", [])
    path = f"{outdir}/pairs_{name}_s{seed}.jsonl"
    old = {}
    if os.path.exists(path):
        for line in open(path):
            r = json.loads(line)
            old[r["pair_id"]] = r
    res = []
    for i, p in enumerate(pairs):
        pid = f"{fam.name}_{name}_s{seed}_pair{i:03d}"
        split = pid
        ths = {"unrecoverable_side": fam.decode(p["u_unrecoverable"]), "recoverable_side": fam.decode(p["u_recoverable"])}
        prev = old.get(pid)
        if prev and all(prev[k]["theta"] == ths[k] for k in ths):
            for k in ths:
                check_profile(prev[k], ctx, path)
            res.append(prev)
            continue
        members = {}
        for role in ("unrecoverable_side", "recoverable_side"):
            rec = fam.run(ths[role], ctx)
            rec.parent_family_id, rec.varied_factor, rec.split_group = pid, fam.names[p["varied_index"]], split
            rec.finalize(ctx["cfg"].eps)
            members[role] = rec.to_json()
            members[role].update(fam.meta())                 # library_id etc., same as the plain scene records
        varied = fam.names[p["varied_index"]]
        umid = [float((a + b) / 2) for a, b in zip(p["u_unrecoverable"], p["u_recoverable"])]
        res.append({"pair_id": pid, "varied": varied, "u_mid": umid,
                    "description": describe_pair(varied, ths["unrecoverable_side"], ths["recoverable_side"]),
                    "anchor_mid_m": fam.anchor(fam.decode(umid), ctx), **members})
    if res:
        with open(path, "w") as f:
            for r in res:
                f.write(json.dumps(r) + "\n")
    return res


def pair_ok(p):
    return p["unrecoverable_side"]["evidence_class"].startswith("U_") and p["recoverable_side"]["evidence_class"] == "R_REF"


def pair_key(p):
    return json.dumps([p["unrecoverable_side"]["theta"], p["recoverable_side"]["theta"]], sort_keys=True)


def pairs_md(fam, all_pairs):
    L = [f"# {fam.name}: matched pairs (bisect, both members re-labelled with the full R matrix)\n",
         "Each pair changes ONE factor (same split_group, same other factors).  `verified` = unrecoverable side is U_* and the recoverable "
         "side is R_REF after the full-record re-label.  anchor@mid = analytic signed margin (m) at the pair midpoint (corridor only): "
         "0 means the CARLA flip sits on the analytic boundary.\n",
         "| pair | description | unrecoverable side | recoverable side | verified | anchor@mid (m) |", "|---|---|---|---|---|---|"]
    for _, ps in all_pairs:
        seen = set()
        for p in ps:
            if pair_key(p) in seen:                      # bisect can re-pick the same chain; list each distinct pair once per seed
                continue
            seen.add(pair_key(p))
            a = p.get("anchor_mid_m")
            L.append(f"| {p['pair_id']} | {p['description']} | {p['unrecoverable_side']['evidence_class']} | "
                     f"{p['recoverable_side']['evidence_class']} | {pair_ok(p)} | {'n/a' if a is None else f'{a:+.2f}'} |")
    return "\n".join(L) + "\n"


def summarise(fam, ctx, args, outdir):
    L, mix, all_pairs = [], {}, []
    fb = args.falsifier_budget or args.budget
    for name in args.algos.split(","):
        budget = fb if name in FALSIFIERS else args.budget
        for seed in range(args.seeds):
            rows, algo = run_one(fam, name, budget, seed, ctx, outdir, bisect_iters=args.bisect_iters)
            pairs = relabel_pairs(fam, algo, ctx, outdir, name, seed) if name == "bisect" else []
            all_pairs.append((seed, pairs))
            recs = [r["rec"] for r in rows]
            ev = [r["evidence_class"] for r in recs]
            for e in ev:
                mix.setdefault(name, {}).setdefault(e, 0)
                mix[name][e] += 1
            anch = [abs(r["_anchor_G_m"]) for r in recs if r.get("_anchor_G_m") is not None]
            agree = [(r["_anchor_G_m"] > 0) == (r["evidence_class"] == "R_REF") for r in recs
                     if r.get("_anchor_G_m") is not None and label_kind(r["evidence_class"]) in ("R", "U")]
            amid = [p["anchor_mid_m"] for p in pairs if p.get("anchor_mid_m") is not None and pair_ok(p)]
            L.append({"algo": name, "seed": seed, "scenes": len(recs), "unrecoverable(U_*)": sum(e.startswith("U_") for e in ev),
                      "R_REF": ev.count("R_REF"), "UNRESOLVED": ev.count("UNRESOLVED"), "INVALID_RUN": ev.count("INVALID_RUN"),
                      "near_boundary": sum(abs(r["V"]) < EDGE for r in rows),
                      "boundary_dist_m": float(np.mean(sorted(anch)[:20])) if anch else np.nan,
                      "label_vs_anchor_agree": float(np.mean(agree)) if agree else np.nan,
                      "pairs": len(pairs), "distinct_pairs": len({pair_key(p) for p in pairs}), "pairs_verified": sum(pair_ok(p) for p in pairs),
                      "pair_mid_anchor_abs_m": float(np.mean(np.abs(amid))) if amid else np.nan,
                      "pair_mid_anchor_signed_m": float(np.mean(amid)) if amid else np.nan})
    return pd.DataFrame(L), mix, all_pairs


def md(df, fam, ctx, args, mix=None):
    g = df.drop(columns="seed").groupby("algo")
    m, n = g.mean(numeric_only=True), df.seed.nunique()
    ci = 1.96 * g.std(numeric_only=True) / math.sqrt(n) if n > 1 else m * 0
    cols = list(m.columns)
    fb = args.falsifier_budget or args.budget
    fals = ", ".join(a for a in m.index if a in FALSIFIERS) or "-"
    head = (f"## {fam.name} mining - backend: {ctx['backend'].upper()}"
            f"{' (MOCK plant: kinematic stand-in, NOT CARLA)' if ctx['backend'] == 'mock' else ''}\n\n"
            f"bisect: {args.bisect_iters or 4} bisection iterations per chain; "
            f"{n} seed(s) x {args.budget} scenes per algorithm ({fb} for falsifiers: {fals}); mean +/- 95% CI (n = {n} seeds, CIs are rough). "
            f"near_boundary = |V_search| < {EDGE}; boundary_dist_m = mean |analytic G| of the 20 scenes closest to the boundary "
            f"(corridor only; reflects sampling density near the boundary, not a localisation error); "
            f"label_vs_anchor_agree = fraction of R/U-labelled scenes whose label matches the sign of the analytic margin; "
            f"pair_mid_anchor_*_m = analytic margin at the midpoint of verified bisect pairs (localisation error of the CARLA flip vs the anchor).\n\n")
    L = ["| algo | " + " | ".join(cols) + " |", "|---|" + "---|" * len(cols)]
    for a in m.index:
        L.append(f"| {a} | " + " | ".join("n/a" if np.isnan(m.loc[a, c]) else f"{m.loc[a, c]:.2f} +/- {0 if np.isnan(ci.loc[a, c]) else ci.loc[a, c]:.2f}" for c in cols) + " |")
    out = head + "\n".join(L) + "\n"
    if mix:
        cls = sorted({c for d in mix.values() for c in d})
        out += "\n### Evidence mix (total scenes over seeds)\n\n| algo | " + " | ".join(cls) + " |\n|---|" + "---|" * len(cls) + "\n"
        for a, d in mix.items():
            out += f"| {a} | " + " | ".join(str(d.get(c, 0)) for c in cls) + " |\n"
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--family", choices=list(FAMILIES), required=True)
    ap.add_argument("--backend", choices=["mock", "carla"], default="mock")
    ap.add_argument("--algos", default="sobol,straddle,bisect")
    ap.add_argument("--budget", type=int, default=60)
    ap.add_argument("--falsifier-budget", type=int, default=None, help="scenes per run for cem/bo_lcb/qd_* (default: --budget)")
    ap.add_argument("--bisect-iters", type=int, default=None, help="bisection steps per chain (default: algos.Bisect default 4)")
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--town", default="custom_straight")
    args = ap.parse_args()
    fam, ctx = get_family(args), make_ctx(args)
    outdir = f"results/{args.backend}/mining/{fam.name}"
    os.makedirs(outdir, exist_ok=True)
    t0 = time.time()
    df, mix, all_pairs = summarise(fam, ctx, args, outdir)
    df.to_csv(f"{outdir}/summary.csv", index=False)
    text = md(df, fam, ctx, args, mix)
    open(f"{outdir}/summary.md", "w", encoding="utf-8", newline="\n").write(text)
    if any(ps for _, ps in all_pairs):
        open(f"{outdir}/pairs.md", "w", encoding="utf-8", newline="\n").write(pairs_md(fam, all_pairs))
    print(text, f"\n({time.time() - t0:.0f} s)")


if __name__ == "__main__":
    main()
