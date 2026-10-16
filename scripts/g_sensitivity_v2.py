"""A1 acceptance: how many corridor-family scenes change class when each MarginConfigV2 constant moves +/-50%.
Scenes: 150 Sobol points of (gap, v, tau, b); label = R_REF / not (via margin.py, the CARLA code path).
Run: python scripts/g_sensitivity_v2.py   -> results/g_sensitivity_v2.md  (REFERENCE model, not CARLA)"""
import dataclasses
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "mining"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "mining", "reference"))
import numpy as np
from scipy.stats import qmc

import corridor_margin as CM
import margin as M

LO, HI = np.array([8.0, 6.0, 0.0, 4.0]), np.array([40.0, 20.0, 0.6, 9.0])
U = qmc.Sobol(4, scramble=True, seed=5).random(128)
TH = LO + U * (HI - LO)


def labels(cfg):
    out = []
    for gap, v, tau, b in TH:
        G, _ = CM.score(v, tau, b, gap, cfg=cfg, dt=0.05)
        out.append(G >= cfg.eps)
    return np.array(out)


def main():
    base = M.DEFAULT_V2
    ref = labels(base)
    rows = [f"| constant | x0.5 changes | x1.5 changes |", "|---|---|---|"]
    for f in ("reserve_m", "q_clear", "hold_s", "stop_speed"):
        ch = []
        for k in (0.5, 1.5):
            cfg = dataclasses.replace(base, **{f: getattr(base, f) * k})
            ch.append(f"{int((labels(cfg) != ref).sum())} / {len(ref)}")
        rows.append(f"| {f} | {ch[0]} | {ch[1]} |")
    os.makedirs("results", exist_ok=True)
    open("results/g_sensitivity_v2.md", "w", encoding="utf-8").write(
        "## G sensitivity - corridor REFERENCE model through margin.py (not CARLA)\n\n"
        f"{len(ref)} Sobol scenes; {int(ref.sum())} R_REF at defaults. Entries = scenes whose R_REF / not-R_REF class flips.\n"
        "q_clear only rescales G (eps = 0), so it cannot flip a class; reserve_m moves the boundary by its change in metres; "
        "hold_s / stop_speed change the terminal test.\n\n" + "\n".join(rows) + "\n")
    print("\n".join(rows))


if __name__ == "__main__":
    main()
