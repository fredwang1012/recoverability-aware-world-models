"""
run.py - drive the search algorithms against a backend, checkpoint every batch, report metrics.

  # real thing: one CARLA server per port, algorithms share the same oracle + budget
  python run.py --backend carla --ports 2000,2002,2004,2006 --algos straddle,qd_boundary,bisect --budget 150 --out out_carla

  # fast algorithm check on the numpy mock (NOT a CARLA result)
  python run.py --backend mock --algos sobol,cem,bo_lcb,straddle,qd_fail,qd_boundary,bisect --budget 100 --out out_mock
  python run.py --backend mock --bench --seeds 3 --budget 100 --out bench       # multi-seed comparison

  python run.py --report --out out_mock                                         # summary + plots from saved CSVs
"""
import argparse
import json
import os
import queue
import time
from concurrent.futures import ThreadPoolExecutor

import numpy as np
import pandas as pd

import core
from algos import REGISTRY, MapElites
from core import EDGE_BAND, EPS, decode


# ------------------------------------------------------------------ oracle pool
class Pool:
    def __init__(self, backends):
        self.q = queue.Queue()
        for b in backends:
            self.q.put(b)
        self.ex = ThreadPoolExecutor(len(backends))

    def _one(self, u, full):
        b = self.q.get()
        try:
            row = core.oracle(b, decode(u), full=full)
        finally:
            self.q.put(b)
        row["u"] = [float(x) for x in u]
        return row

    def evaluate(self, U, full=False):
        return list(self.ex.map(lambda u: self._one(u, full), U))


def make_pool(args):
    if args.backend == "mock":
        return Pool([core.MockBackend() for _ in range(args.workers)])
    from carla_backend import CarlaBackend
    ports = [int(p) for p in args.ports.split(",")]
    return Pool([CarlaBackend(host=args.host, port=p, town=args.town) for p in ports])


# ------------------------------------------------------------------ search loop
def search(name, pool, budget, batch, seed, out_csv=None, log=print, vtransform="none"):
    rng = np.random.default_rng(seed)
    algo = REGISTRY[name](rng, vtransform=vtransform)
    rows, t0 = [], time.time()
    while len(rows) < budget:
        n = min(batch, budget - len(rows))
        U = np.clip(algo.ask(n), 0, 1)[:n]
        new = pool.evaluate(U)
        algo.tell(U, new)
        rows += new
        if out_csv:
            pd.DataFrame([{**r, "u": json.dumps(r["u"]), "algo": name, "i": i} for i, r in enumerate(rows)]).to_csv(out_csv, index=False)
        c = pd.Series([r["category"] for r in rows]).value_counts().to_dict()
        log(f"  {name:12s} {len(rows):4d}/{budget}  {c}  ({time.time() - t0:.0f}s)")
    return rows, algo


# ------------------------------------------------------------------ metrics
def diversity(rows):
    pts = np.array([r["u"] for r in rows if r["category"] == "unrecoverable"])
    if len(pts) < 2:
        return 0.0
    d = np.linalg.norm(pts[:, None] - pts[None], axis=-1)
    np.fill_diagonal(d, np.inf)
    return float(d.min(1).mean())


def qd_coverage(rows, bins=(10, 10), ranges=((0, 8), (0, 10))):
    me = MapElites(np.random.default_rng(0), bins=bins, ranges=ranges)
    cells = {me.cell(r) for r in rows if r["category"] == "unrecoverable"}
    return len(cells)


def summarize(name, rows, pairs=0):
    cats = pd.Series([r["category"] for r in rows]).value_counts()
    hit = next((i + 1 for i, r in enumerate(rows) if r["V"] < EPS), np.nan)
    return {"algorithm": name, "sims": len(rows),
            "unrecoverable": int(cats.get("unrecoverable", 0)),
            "planner_failure": int(cats.get("planner_failure", 0)),
            "edge": int(cats.get("edge", 0)),
            "sims_to_first_unrec": hit,
            "near_boundary": int(sum(abs(r["V"] - EPS) < EDGE_BAND for r in rows)),
            "diversity": round(diversity(rows), 3),
            "qd_cells_unrec": qd_coverage(rows),
            "matched_pairs": pairs}


def levelset_f1(rows, ref_u, ref_unrec):
    """Ground-truth level-set quality WITHOUT any algorithm-specific surrogate: 1-NN label from the
    algorithm's own samples, scored on a reference Sobol set (mock backend only)."""
    X = np.array([r["u"] for r in rows]); y = np.array([r["V"] < EPS for r in rows])
    d = np.linalg.norm(ref_u[:, None] - X[None], axis=-1)
    pred = y[d.argmin(1)]
    tp = (pred & ref_unrec).sum(); fp = (pred & ~ref_unrec).sum(); fn = (~pred & ref_unrec).sum()
    return float(2 * tp / max(2 * tp + fp + fn, 1))


# ------------------------------------------------------------------ plots
def plot_run(out, algos):
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    col = {"unrecoverable": "#c0392b", "planner_failure": "#e67e22", "edge": "#f1c40f", "safe": "#27ae60"}
    fig, axes = plt.subplots(1, len(algos), figsize=(3.6 * len(algos), 3.8), sharex=True, sharey=True, squeeze=False)
    for ax, a in zip(axes[0], algos):
        df = pd.read_csv(f"{out}/samples_{a}.csv")
        for cat, c in col.items():
            m = df.category == cat
            ax.scatter(df.m1[m], df.m2[m], s=14, c=c, alpha=.8, label=cat)
        ax.set(title=a, xlabel="m1: time to obstacle (s)")
    axes[0][0].set_ylabel("m2: time until left car catches ego (s)")
    axes[0][0].legend(fontsize=7)
    fig.tight_layout(); fig.savefig(f"{out}/descriptor_space.png", dpi=140)


def plot_bench(df, out):
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    cols = ["unrecoverable", "near_boundary", "qd_cells_unrec", "diversity", "levelset_f1"]
    fig, axes = plt.subplots(1, len(cols), figsize=(3.4 * len(cols), 3.6))
    for ax, c in zip(axes, cols):
        g = df.groupby("algorithm")[c]
        m, s = g.mean(), g.std().fillna(0)
        ax.bar(m.index, m.values, yerr=s.values, color="#4c72b0"); ax.set_title(c, fontsize=10)
        ax.tick_params(axis="x", rotation=60)
    fig.tight_layout(); fig.savefig(f"{out}/benchmark.png", dpi=140)


# ------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--family", choices=list(core.FAMILIES), default="two_lane")
    ap.add_argument("--backend", choices=["mock", "carla"], default="mock")
    ap.add_argument("--algos", default="sobol,cem,bo_lcb,straddle,qd_fail,qd_boundary,bisect")
    ap.add_argument("--budget", type=int, default=100, help="oracle calls (scenes) per algorithm")
    ap.add_argument("--batch", type=int, default=None, help="scenes per round (default = number of servers)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--seeds", type=int, default=3, help="for --bench")
    ap.add_argument("--workers", type=int, default=1)
    ap.add_argument("--ports", default="2000"); ap.add_argument("--host", default="localhost")
    ap.add_argument("--town", default="Town04")
    ap.add_argument("--out", default="out")
    ap.add_argument("--bench", action="store_true"); ap.add_argument("--report", action="store_true")
    ap.add_argument("--vtransform", choices=["none", "clip", "tanh"], default="none",
                    help="transform of V before the GP fit (bo_lcb / straddle only)")
    ap.add_argument("--profiles", action="store_true", help="re-label finds with the full recovery profile R(h)")
    args = ap.parse_args()
    core.use_family(args.family)
    os.makedirs(args.out, exist_ok=True)
    algos = args.algos.split(",")

    if args.report:
        rows = {a: pd.read_csv(f"{args.out}/samples_{a}.csv").assign(u=lambda d: d.u.map(json.loads)).to_dict("records")
                for a in algos if os.path.exists(f"{args.out}/samples_{a}.csv")}
        s = pd.DataFrame([summarize(a, r) for a, r in rows.items()])
        print(s.to_string(index=False)); s.to_csv(f"{args.out}/summary.csv", index=False)
        plot_run(args.out, list(rows)); return

    pool = make_pool(args)
    batch = args.batch or (len(args.ports.split(",")) if args.backend == "carla" else 8)

    if args.bench:
        ref_u = np.random.default_rng(123).random((600, core.D))
        ref = pool.evaluate(ref_u)
        ref_unrec = np.array([r["V"] < EPS for r in ref])
        print(f"reference set: {ref_unrec.mean():.0%} unrecoverable of {len(ref)}")
        res = []
        for seed in range(args.seed, args.seed + args.seeds):
            for a in algos:
                tag = f"{args.out}/bench_{a}_s{seed}.csv"
                rows, algo = search(a, pool, args.budget, batch, seed, tag, log=lambda *_: None, vtransform=args.vtransform)
                m = summarize(a, rows, len(getattr(algo, "pairs", [])))
                m.update(seed=seed, levelset_f1=levelset_f1(rows, ref_u, ref_unrec))
                res.append(m); print(m)
        df = pd.DataFrame(res); df.to_csv(f"{args.out}/bench_results.csv", index=False)
        print(df.groupby("algorithm").mean(numeric_only=True).round(3).to_string())
        plot_bench(df, args.out); return

    summaries, mined = [], []
    for a in algos:
        csv = f"{args.out}/samples_{a}.csv"
        print(f"== {a}")
        rows, algo = search(a, pool, args.budget, batch, args.seed, csv, vtransform=args.vtransform)
        pairs = getattr(algo, "pairs", [])
        if pairs:
            for p in pairs:
                p["varied"] = core.NAMES[p["varied_index"]]
                p["unrecoverable"] = decode(np.array(p["u_unrecoverable"])); p["recoverable"] = decode(np.array(p["u_recoverable"]))
            json.dump(pairs, open(f"{args.out}/matched_pairs.json", "w"), indent=2)
        summaries.append(summarize(a, rows, len(pairs)))
        mined += [r for r in rows if r["category"] in ("unrecoverable", "edge", "planner_failure")]
    s = pd.DataFrame(summaries); print(s.to_string(index=False)); s.to_csv(f"{args.out}/summary.csv", index=False)

    if args.profiles:                                       # full R(h) for the scenes worth reconstructing
        uniq = {json.dumps(r["u"]): r["u"] for r in mined}
        full = pool.evaluate([np.array(u) for u in uniq.values()], full=True)
    else:
        full = list({json.dumps(r["u"]): r for r in mined}.values())
    scenes = [{k: v for k, v in r.items() if k != "u"} for r in full]
    json.dump(scenes, open(f"{args.out}/mined_scenes.json", "w"), indent=1)
    plot_run(args.out, algos)
    print(f"\nwrote {len(scenes)} mined scenes to {args.out}/mined_scenes.json")


if __name__ == "__main__":
    main()
