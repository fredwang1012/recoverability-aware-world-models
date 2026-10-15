"""T9: build proposal assets from SAVED results (mock backend only; every file says so).

Inputs : results/bench10/bench_results.csv, results/mock_run/matched_pairs.json, results/pnr/pnr_scan.csv
Outputs: results/proposal_assets/{definition.md, algorithm_table.md, matched_pairs.md,
         recoverability_map.png, point_of_no_return.png, ref_sobol.csv}
Run    : python scripts/proposal_assets.py
"""
import json
import os
import shutil
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "mining"))
import numpy as np
import pandas as pd
from scipy.stats import qmc

import core

OUT = "results/proposal_assets"
UNITS = {"obstacle_gap": "m", "adjacent_offset": "m", "adjacent_speed": "m/s", "ego_speed": "m/s",
         "reaction_delay": "s", "brake_limit": "m/s^2"}
NICE = {"obstacle_gap": "distance to the stopped obstacle", "adjacent_offset": "position of the left-lane car relative to ego",
        "adjacent_speed": "speed of the left-lane car", "ego_speed": "ego speed",
        "reaction_delay": "ego reaction delay", "brake_limit": "available braking deceleration"}


def reference_set(n=2048):
    path = f"{OUT}/ref_sobol.csv"
    if os.path.exists(path):
        return pd.read_csv(path)
    be = core.MockBackend()
    U = qmc.Sobol(core.D, scramble=True, seed=11).random(n)
    df = pd.DataFrame([core.oracle(be, core.decode(u)) for u in U])
    df.to_csv(path, index=False)
    return df


def recoverability_map(df):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    col = {"unrecoverable": "#c0392b", "planner_failure": "#e67e22", "edge": "#f1c40f", "safe": "#27ae60"}
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.4))
    for cat, c in col.items():
        m = df.category == cat
        axes[0].scatter(df.m1[m], df.m2[m], s=7, c=c, alpha=0.7, label=f"{cat} ({m.sum()})")
    axes[0].set(xlabel="m1: time to reach the obstacle at current speed (s)",
                ylabel="m2: time until the left-lane car catches ego (s)",
                title="2048 Sobol scenes by category")
    axes[0].legend(fontsize=7)
    b1, b2 = np.linspace(0, 8, 17), np.linspace(0, 10, 21)
    num, _, _ = np.histogram2d(df.m1, df.m2, bins=[b1, b2], weights=(df.V < core.EPS).astype(float))
    den, _, _ = np.histogram2d(df.m1, df.m2, bins=[b1, b2])
    frac = np.where(den >= 3, num / np.maximum(den, 1), np.nan)
    im = axes[1].pcolormesh(b1, b2, frac.T, cmap="RdYlGn_r", vmin=0, vmax=1)
    axes[1].set(xlabel="m1 (s)", ylabel="m2 (s)", title="fraction unrecoverable per cell (cells with >= 3 scenes)")
    fig.colorbar(im, ax=axes[1])
    fig.suptitle("MOCK backend (not CARLA). Unrecoverable = relative to library, worlds, horizon 8 s, eps = 0", fontsize=9)
    fig.tight_layout()
    fig.savefig(f"{OUT}/recoverability_map.png", dpi=150)


def algorithm_table():
    d = pd.read_csv("results/bench10/bench_results.csv")
    n = d.seed.nunique()
    cols = ["unrecoverable", "edge", "near_boundary", "sims_to_first_unrec", "qd_cells_unrec", "diversity", "matched_pairs"]
    L = [f"## Algorithm comparison - MOCK backend (not CARLA)", "",
         f"{n} seeds x 100 scenes per algorithm, same oracle. Mean +/- 95% CI half-width over seeds. "
         f"near_boundary = scenes with |V - eps| < {core.EDGE_BAND}. qd_cells_unrec = distinct unrecoverable cells of a 10x10 (m1, m2) grid.",
         "", "| algorithm | " + " | ".join(cols) + " |", "|---|" + "---|" * len(cols)]
    for a, g in d.groupby("algorithm"):
        L.append(f"| {a} | " + " | ".join(f"{g[c].mean():.{2 if c == 'diversity' else 1}f} +/- {1.96 * g[c].std() / np.sqrt(n):.{2 if c == 'diversity' else 1}f}" for c in cols) + " |")
    L += ["", "Reading: straddle targets the boundary; cem / bo_lcb find failures but little near the boundary; bisect is the only source of "
          "one-factor matched pairs; MAP-Elites (qd_*) is under-budgeted at 100 scenes. The level-set F1 metric is omitted because it "
          "rewards space-filling, not boundary focus."]
    open(f"{OUT}/algorithm_table.md", "w", encoding="utf-8").write("\n".join(L) + "\n")


def pnr_figure():
    shutil.copy("results/pnr/pnr_curves.png", f"{OUT}/point_of_no_return.png")


def matched_pairs_md():
    pairs = json.load(open("results/mock_run/matched_pairs.json"))
    be, L = core.MockBackend(), ["## Matched pairs - MOCK backend (not CARLA)", "",
                                 "Same scene except ONE factor; one member unrecoverable, the other recoverable "
                                 "(relative to the library, worlds, horizon, eps). Bisection stops after 4 halvings, "
                                 "so the factor gap is the localisation precision.", ""]
    shown, seen = 0, set()
    for p in pairs:
        f = p["varied"] if "varied" in p else core.NAMES[p["varied_index"]]
        lo, hi = core.decode(np.array(p["u_unrecoverable"])), core.decode(np.array(p["u_recoverable"]))
        key = (f, round(lo[f], 3), round(hi[f], 3), round(lo["ego_speed"], 3), round(lo["obstacle_gap"], 3))
        if lo[f] == hi[f] or key in seen:
            continue
        seen.add(key)
        Vlo, Vhi = core.oracle(be, lo)["V"], core.oracle(be, hi)["V"]
        rest = ", ".join(f"{k} {lo[k]:.1f} {UNITS[k]}" for k in core.NAMES if k != f)
        L.append(f"{shown + 1}. **{NICE[f]}**: {lo[f]:.2f} {UNITS[f]} -> V = {Vlo:+.2f} (unrecoverable); "
                 f"{hi[f]:.2f} {UNITS[f]} -> V = {Vhi:+.2f} (recoverable). Held fixed: {rest}.")
        shown += 1
        if shown == 10:
            break
    open(f"{OUT}/matched_pairs.md", "w", encoding="utf-8").write("\n".join(L) + "\n")
    return shown


DEFINITION = """## Definition: unrecoverable relative to a library (Approach 1)

For a scene h (here: scenario parameters theta and the shared observation history), a finite library of causal recovery
policies Pi = {pi_1..pi_N} (observation -> command, never given the hidden world) and a finite set of hidden worlds Omega,
let G(pi_j, omega; h) be a normalised margin computed from privileged simulator state (collision clearance, lane keeping, a
time-headway continuation term; used for labels only). The per-policy reference margin is r_j(h) = min over omega in Omega of
G(pi_j, omega; h), the recovery profile is R(h) = [r_1(h), ..., r_N(h)], and V(h) = max_j r_j(h). A scene is **unrecoverable
relative to (Pi, Omega, horizon, eps)** iff V(h) < eps. It is a *finite-library, finite-world approximation of an inevitable
collision state* (cf. Time-To-React as the max over maneuvers; Brake Threat Number >= 1; Hamilton-Jacobi collision-avoidance
reachable sets). It is never a claim of physical impossibility.

**Direction of errors.** A too-small Pi makes a scene look unrecoverable when a better recovery exists (false
"unrecoverable"). A too-small Omega makes a scene look recoverable when a worse world exists (false "recoverable"). A horizon
that is too short also makes scenes look recoverable. Evidence is kept per policy: verified witness (r_j >= eps),
verified counterexample (r_j < eps), or unresolved (never invented).

**Classes.** unrecoverable (V < eps); planner_failure (V >= eps but the planner under test fails: a recovery existed);
edge (eps <= V < eps + 0.3); safe. All numbers in these assets come from the numpy MOCK backend, not CARLA.
"""


def main():
    os.makedirs(OUT, exist_ok=True)
    recoverability_map(reference_set())
    algorithm_table()
    pnr_figure()
    n = matched_pairs_md()
    open(f"{OUT}/definition.md", "w", encoding="utf-8").write(DEFINITION)
    print(f"wrote proposal assets to {OUT} ({n} matched pairs)")


if __name__ == "__main__":
    main()
