"""A3: mining on ANALYTIC reference families (no CARLA, no mock).  Ground truth is closed form, so every metric is checkable.

Families
  corridor5  RC01/02/12/17 : theta = (gap, v, tau, b, prefix);  G_m = gap - v*(tau+prefix) - v^2/(2b) - c  [m];  Rec <=> G_m >= 0
  info_tree  RC10/11       : theta = (D, t_obs, T_lat, tau_plan);  label via info_tree.build_matrix + contract.classify
Search sees only V = clip(G/q, -1, 1) (search-grade).  Labels come from the exact quantity.

Algorithms: sobol, cem, bo_lcb, straddle, qd_boundary, bisect  (+ info-gap mining on info_tree: cem on -(V_hind - V), straddle+filter)
Metrics: unrec, near (|V|<0.3), boundary distance (mean |G| of the 20 closest to the boundary), surrogate F1, pairs, pair error.
For info_tree: U_INFORMATION count, and the error of the mined R_REF -> U_INFORMATION boundary vs the analytic t_obs*.

Run: python scripts/reference_mining.py [--seeds 10] [--budget 80]    -> results/reference_mining.md (+ csv)
"""
import argparse
import math
import os
import sys
import warnings

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "mining"))
sys.path.insert(0, os.path.join(HERE, "..", "mining", "reference"))
import numpy as np
import pandas as pd

import algos
import corridor as C
import info_tree as T
from contract import classify, R_REF, U_INFORMATION, U_LIBRARY

warnings.filterwarnings("ignore")
CRES, Q = 1.0, 5.0


# ----------------------------------------------------------------------------- corridor5
class Corridor5:
    names = ["gap", "v", "tau", "b", "prefix"]
    lo = np.array([8.0, 6.0, 0.0, 4.0, 0.0])
    hi = np.array([40.0, 20.0, 0.6, 9.0, 0.6])
    d = 5

    def decode(self, u):
        return self.lo + np.clip(u, 0, 1) * (self.hi - self.lo)

    def G_m(self, th):
        gap, v, tau, b, pre = th
        return gap - v * (tau + pre) - v * v / (2 * b) - CRES

    def row(self, u):
        th = self.decode(u)
        g = self.G_m(th)
        V = float(np.clip(g / Q, -1, 1))
        return {"u": [float(x) for x in u], "V": V, "G_m": g, "m1": float(th[0] / th[1]), "m2": float(th[3]),
                "evidence": R_REF if g >= 0 else U_LIBRARY}

    def anchors(self):
        """Exact boundary in gap for given others - used for pair error."""
        return None


# ----------------------------------------------------------------------------- info tree
class InfoTree:
    names = ["D", "t_obs", "T_lat", "tau_plan"]
    lo = np.array([14.0, 0.0, 0.8, 0.0])
    hi = np.array([30.0, 1.2, 1.6, 0.3])
    d = 4
    v, tau, b, reserve = 15.0, 0.25, 6.0, 0.05
    p_grid = np.arange(0.0, 2.55, 0.1)          # coarser than the unit test: speed; threshold comparison allows one grid step
    ts_grid = np.arange(0.0, 2.05, 0.1)

    def decode(self, u):
        return self.lo + np.clip(u, 0, 1) * (self.hi - self.lo)

    def tstar(self, D, T_lat, tau_plan):
        arr = [T.t_arrival(D, self.v, self.tau, self.b, p) for p in self.p_grid]
        arr = [a for a in arr if math.isfinite(a)]
        return (max(arr) - tau_plan - self.tau - T_lat - self.reserve) if arr else float("nan")

    def row(self, u):
        D, t_obs, T_lat, tau_plan = self.decode(u)
        names, R = T.build_matrix(D, v=self.v, tau=self.tau, b=self.b, T_lat=T_lat, tau_plan=tau_plan, t_obs=t_obs,
                                  reserve=self.reserve, p_grid=self.p_grid, ts_grid=self.ts_grid)
        c = classify(R)
        V = float(np.clip(c["V"], -1, 1))
        return {"u": [float(x) for x in u], "V": V, "V_hind": float(c["V_hind"]), "info_gap": float(c["info_gap"]),
                "evidence": c["evidence"], "m1": float(D / self.v), "m2": float(t_obs * 8.0),
                "tstar": self.tstar(D, T_lat, tau_plan), "t_obs": float(t_obs),
                "stop_ok": bool((D - (self.v * self.tau + self.v ** 2 / (2 * self.b)) - 1.0) >= 0)}   # a common stop works


# ----------------------------------------------------------------------------- drivers
def run(fam, name, budget, seed, batch=4, info_gap_mode=False):
    rng = np.random.default_rng(seed)
    algo = algos.REGISTRY[name](rng, d=fam.d)
    rows = []
    while len(rows) < budget:
        n = min(batch, budget - len(rows))
        U = np.clip(algo.ask(n), 0, 1)[:n]
        new = [fam.row(u) for u in U]
        if info_gap_mode:                       # CEM minimises V: feed it -(info_gap) when V_hind >= eps, +1 otherwise
            fed = [dict(r, V=(-r["info_gap"] if r["V_hind"] >= 0 else 1.0)) for r in new]
            algo.tell(U, fed)
        else:
            algo.tell(U, new)
        rows += new
    return rows, algo


def gp_f1(fam, rows, ref_u, ref_unrec):
    from sklearn.gaussian_process import GaussianProcessRegressor as GPR
    from sklearn.gaussian_process.kernels import ConstantKernel as Ck, Matern, WhiteKernel
    X = np.array([r["u"] for r in rows]); y = np.array([r["V"] for r in rows])
    gp = GPR(Ck(1.0) * Matern(np.full(fam.d, .3), (.05, 5), nu=2.5) + WhiteKernel(1e-4, (1e-8, 1e-1)), normalize_y=True,
             n_restarts_optimizer=1, random_state=0).fit(X, y)
    pred = gp.predict(ref_u) < 0
    tp = (pred & ref_unrec).sum(); fp = (pred & ~ref_unrec).sum(); fn = (~pred & ref_unrec).sum()
    return float(2 * tp / max(2 * tp + fp + fn, 1))


def corridor_table(budget, seeds, algo_names):
    fam = Corridor5()
    ref_u = np.random.default_rng(999).random((4000, fam.d))
    ref_unrec = np.array([fam.G_m(fam.decode(u)) < 0 for u in ref_u])
    out = []
    for a in algo_names:
        for s in range(seeds):
            rows, algo = run(fam, a, budget, s)
            pairs = getattr(algo, "pairs", [])
            perr = np.nanmean([abs(fam.G_m((fam.decode(np.array(p["u_unrecoverable"])) + fam.decode(np.array(p["u_recoverable"]))) / 2))
                               for p in pairs]) if pairs else np.nan
            out.append({"algo": a, "seed": s, "unrec": sum(r["V"] < 0 for r in rows), "near": sum(abs(r["V"]) < .3 for r in rows),
                        "boundary_dist_m": float(np.mean(sorted(abs(r["G_m"]) for r in rows)[:20])),
                        "f1": gp_f1(fam, rows, ref_u, ref_unrec), "pairs": len(pairs), "pair_err_m": perr})
    return pd.DataFrame(out)


def info_table(budget, seeds, algo_names):
    fam = InfoTree()
    out, step = [], 0.1
    for a in algo_names + ["cem_infogap"]:
        for s in range(seeds):
            gap_mode = a == "cem_infogap"
            rows, algo = run(fam, "cem" if gap_mode else a, budget, s, info_gap_mode=gap_mode)
            n_info = sum(r["evidence"] == U_INFORMATION for r in rows)
            near = [r for r in rows if abs(r["V"]) < 0.1 and r["V_hind"] >= 0 and not r["stop_ok"]]   # |V| < 0.1 == one grid step in t_obs (dV/dt_obs = -1)
            terr = float(np.mean([abs(r["t_obs"] - r["tstar"]) for r in near])) if near else np.nan
            pairs = getattr(algo, "pairs", [])
            out.append({"algo": a, "seed": s, "U_INFORMATION": n_info, "R_REF": sum(r["evidence"] == R_REF for r in rows),
                        "U_LIBRARY": sum(r["evidence"] == U_LIBRARY for r in rows), "near_boundary": len(near),
                        "tobs_err_s": terr, "within_grid_step": float(np.mean([abs(r["t_obs"] - r["tstar"]) <= step for r in near])) if near else np.nan,
                        "max_info_gap": max((r["info_gap"] for r in rows if r["V_hind"] >= 0), default=np.nan), "pairs": len(pairs)})
    return pd.DataFrame(out)


def md(df, title, note):
    g = df.drop(columns="seed").groupby("algo")
    m, ci = g.mean(numeric_only=True), 1.96 * g.std(numeric_only=True) / math.sqrt(df.seed.nunique())
    cols = list(m.columns)
    L = [f"### {title}", "", note, "", "| algo | " + " | ".join(cols) + " |", "|---|" + "---|" * len(cols)]
    for a in m.index:
        L.append(f"| {a} | " + " | ".join("n/a" if np.isnan(m.loc[a, c]) else f"{m.loc[a, c]:.2f} +/- {0 if np.isnan(ci.loc[a, c]) else ci.loc[a, c]:.2f}" for c in cols) + " |")
    return "\n".join(L) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=10)
    ap.add_argument("--budget", type=int, default=80)
    ap.add_argument("--out", default="results/reference")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    names = ["sobol", "cem", "bo_lcb", "straddle", "qd_boundary", "bisect"]
    c5 = corridor_table(a.budget, a.seeds, names)
    c5.to_csv(f"{a.out}/corridor5.csv", index=False)
    it = info_table(a.budget, a.seeds, names)
    it.to_csv(f"{a.out}/info_tree.csv", index=False)
    head = (f"## Reference-family mining (analytic ground truth; NOT CARLA) - {a.seeds} seeds x {a.budget} scenes, mean +/- 95% CI\n\n")
    text = head + md(c5, "corridor5 (RC01/02/12/17): theta = gap, v, tau, b, prefix",
                     "boundary_dist_m = mean |G| in metres of the 20 samples closest to the boundary (smaller = better); "
                     "f1 = GP-surrogate F1 for the unrecoverable class on 4000 reference points; pair_err_m = |G| at pair midpoints.") + "\n"
    text += md(it, "info_tree (RC10/RC11): theta = D, t_obs, T_lat, tau_plan",
               "near_boundary = |V| < 0.1 with V_hind >= eps and no common stop (so the R_REF/U_INFORMATION flip is the reveal-time boundary); tobs_err_s / within_grid_step compare mined boundary points to the analytic "
               "t_obs* (grid step 0.1 s); cem_infogap = CEM maximising V_hind - V subject to V_hind >= eps.") + "\n"
    open("results/reference_mining.md", "w", encoding="utf-8").write(text)
    print(text)


if __name__ == "__main__":
    main()
