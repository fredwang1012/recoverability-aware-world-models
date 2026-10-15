"""T1: sensitivity of scene categories to each constant of the margin G (mock backend, NOT CARLA).

Rollouts do not depend on MarginConfig, so traces are cached and only the margin is recomputed.
Usage: python scripts/g_sensitivity.py --n 400 --out results/g_sensitivity.md
"""
import argparse
import os
import sys
from dataclasses import fields, replace

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "mining"))
import numpy as np
from scipy.stats import qmc

import core


class CachedBackend:
    def __init__(self):
        self.be, self.cache = core.MockBackend(), {}

    def rollout(self, th, world, policy):
        key = (tuple(sorted(th.items())), world, policy.name)
        if key not in self.cache:
            self.cache[key] = self.be.rollout(th, world, policy)
        return self.cache[key]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=400)
    ap.add_argument("--out", default="results/g_sensitivity.md")
    a = ap.parse_args()
    U = qmc.Sobol(core.D, scramble=True, seed=0).random(a.n)
    be = CachedBackend()
    cats = lambda cfg: [core.oracle(be, core.decode(u), cfg=cfg)["category"] for u in U]
    base = cats(core.DEFAULT_CFG)
    lines = ["# Sensitivity of scene category to the constants of G (mock backend, not CARLA)", "",
             f"{a.n} scrambled-Sobol scenes. Baseline: " +
             ", ".join(f"{k}={base.count(k)}" for k in ("unrecoverable", "planner_failure", "edge", "safe")) + ".", "",
             "| constant | default | factor | scenes changing category | to unrecoverable | from unrecoverable |",
             "|---|---|---|---|---|---|"]
    for f in fields(core.MarginConfig):
        d = getattr(core.DEFAULT_CFG, f.name)
        for fac in (0.5, 1.5):
            c = cats(replace(core.DEFAULT_CFG, **{f.name: d * fac}))
            ch = [(b, x) for b, x in zip(base, c) if b != x]
            lines.append(f"| {f.name} | {d} | x{fac} | {len(ch)} ({len(ch) / a.n:.1%}) | "
                         f"{sum(x == 'unrecoverable' for _, x in ch)} | {sum(b == 'unrecoverable' for b, _ in ch)} |")
    lines += ["", "'Unrecoverable' is relative to the library, worlds, horizon and eps of `mining/core.py`."]
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    open(a.out, "w").write("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
