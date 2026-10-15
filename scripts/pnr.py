"""T5 (mock): point-of-no-return / time-to-react scan, approach (i) 'replay the nominal prefix, then branch'.

On the mock, holding the nominal KEEP command for t seconds and then handing control to the library is exactly
theta with reaction_delay = t, so V(h_t) = oracle(theta | reaction_delay = t). t* = first t where V < eps.
This is a simulation-based Time-To-React over the library (max over maneuvers).  NOT a CARLA result.
Usage: python scripts/pnr.py --n 6 --out results/pnr
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "mining"))
import numpy as np
import pandas as pd

import core

T_GRID = np.round(np.arange(0.0, 3.01, 0.2), 2)


def scan(be, th):
    rows = [{"t_s": float(t), "V": core.oracle(be, dict(th, reaction_delay=float(t)))["V"]} for t in T_GRID]
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=6)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default="results/pnr")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    be, rng, found = core.MockBackend(), np.random.default_rng(a.seed), []
    while len(found) < a.n:
        th = core.decode(rng.random(core.D)); th["reaction_delay"] = 0.0
        rows = scan(be, th)
        if rows[0]["V"] >= core.EPS > rows[-1]["V"]:                 # recoverable now, not recoverable if we wait
            t_star = next(r["t_s"] for r in rows if r["V"] < core.EPS)
            found.append((th, rows, t_star))
    recs = []
    for i, (th, rows, ts) in enumerate(found):
        for r in rows:
            recs.append({"scene": i, **{k: round(v, 3) for k, v in th.items() if k != "reaction_delay"}, **r, "t_star": ts})
    df = pd.DataFrame(recs); df.to_csv(f"{a.out}/pnr_scan.csv", index=False)
    print(df.groupby("scene").agg(t_star=("t_star", "first"), V0=("V", "first")).to_string())
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(6.5, 4))
    for i, g in df.groupby("scene"):
        ax.plot(g.t_s, g.V, marker="o", ms=3, label=f"scene {i}  t*={g.t_star.iloc[0]:.1f}s")
    ax.axhline(core.EPS, color="k", lw=1)
    ax.set(xlabel="time spent on the nominal (keep-speed) path before reacting, t (s)", ylabel="V(h_t)",
           title="Point of no return (mock): V(h_t) crosses eps at t*\n(unrecoverable relative to library, worlds, horizon, eps)")
    ax.legend(fontsize=7); fig.tight_layout(); fig.savefig(f"{a.out}/pnr_curves.png", dpi=150)


if __name__ == "__main__":
    main()
