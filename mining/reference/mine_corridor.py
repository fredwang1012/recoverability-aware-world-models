"""
reference/mine_corridor.py - can the mining algorithms rediscover a boundary we know analytically?

Family RC01/RC02/RC12/RC17 in the PDF's exact 1-D model. theta = (gap, v, tau, b), reserve c = 1 m.
Ground truth (closed form):  G_m = gap - D_stop - c   [metres];  recoverable  <=>  G_m >= 0.
The search sees only V = clip(G_m/5, -1, 1), like it would see a CARLA margin.

Metrics (all against the analytic truth, so they are checkable):
  unrec          # samples with G_m < 0
  near           # samples within +-1.5 m of the boundary (|V| < 0.3)
  mean_abs_G_m   mean |G_m| over the 20 samples closest to the boundary (smaller = better localisation)
  f1             F1 of the unrecoverable class from a GP surrogate fitted on the samples, scored on 4000 reference points
  pairs, pair_err_m   one-factor matched pairs found and mean |G_m| of their midpoints
"""
import json, os, sys, warnings
import numpy as np, pandas as pd
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))
from algos import REGISTRY  # noqa: E402
import corridor as C  # noqa: E402

warnings.filterwarnings("ignore")
LO = np.array([8.0, 6.0, 0.0, 4.0]); HI = np.array([40.0, 20.0, 0.6, 9.0]); NAMES = ["gap", "v", "tau", "b"]
CRES, Q = 1.0, 5.0


def decode(u): return LO + np.clip(u, 0, 1) * (HI - LO)
def G_m(th): gap, v, tau, b = th; return gap - C.d_stop(v, tau, b) - CRES
def row(u):
    th = decode(u); g = G_m(th); V = float(np.clip(g / Q, -1, 1))
    return {"u": [float(x) for x in u], "V": V, "G_m": g, "m1": float(th[0] / th[1]), "m2": float(th[3]),
            "category": "unrecoverable" if V < 0 else "edge" if V < 0.3 else "safe"}


def run(name, budget, seed, batch=4):
    rng = np.random.default_rng(seed); algo = REGISTRY[name](rng, d=4); rows = []
    while len(rows) < budget:
        n = min(batch, budget - len(rows)); U = np.clip(algo.ask(n), 0, 1)[:n]
        new = [row(u) for u in U]; algo.tell(U, new); rows += new
    return rows, algo


def gp_f1(rows, ref_u, ref_unrec):
    from sklearn.gaussian_process import GaussianProcessRegressor as GPR
    from sklearn.gaussian_process.kernels import ConstantKernel as Ck, Matern, WhiteKernel
    X = np.array([r["u"] for r in rows]); y = np.array([r["V"] for r in rows])
    gp = GPR(Ck(1.0) * Matern(np.full(4, .3), (.05, 5), nu=2.5) + WhiteKernel(1e-4, (1e-8, 1e-1)), normalize_y=True,
             n_restarts_optimizer=1, random_state=0).fit(X, y)
    pred = gp.predict(ref_u) < 0
    tp = (pred & ref_unrec).sum(); fp = (pred & ~ref_unrec).sum(); fn = (~pred & ref_unrec).sum()
    return float(2 * tp / max(2 * tp + fp + fn, 1))


def main(budget=80, seeds=5, algos=("sobol", "cem", "bo_lcb", "straddle", "qd_boundary", "bisect")):
    ref_u = np.random.default_rng(999).random((4000, 4)); ref_unrec = np.array([G_m(decode(u)) < 0 for u in ref_u])
    print(f"reference: {ref_unrec.mean():.0%} unrecoverable of {len(ref_u)}; budget {budget}, seeds {seeds}")
    out = []
    for a in algos:
        for s in range(seeds):
            rows, algo = run(a, budget, s)
            absg = sorted(abs(r["G_m"]) for r in rows)[:20]
            pairs = getattr(algo, "pairs", [])
            perr = np.nanmean([abs(G_m((decode(np.array(p["u_unrecoverable"])) + decode(np.array(p["u_recoverable"]))) / 2)) for p in pairs]) if pairs else np.nan
            out.append({"algo": a, "seed": s, "unrec": sum(r["V"] < 0 for r in rows), "near": sum(abs(r["V"]) < .3 for r in rows),
                        "mean_abs_G_m": float(np.mean(absg)), "f1": gp_f1(rows, ref_u, ref_unrec),
                        "pairs": len(pairs), "pair_err_m": perr})
    df = pd.DataFrame(out); df.to_csv(os.path.join(os.path.dirname(__file__), "corridor_mining_results.csv"), index=False)
    print(df.groupby("algo").mean(numeric_only=True).drop(columns="seed").round(3).to_string())


if __name__ == "__main__":
    main()
