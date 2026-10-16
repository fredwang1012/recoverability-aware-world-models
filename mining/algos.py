"""
algos.py - search algorithms for mining unrecoverable / edge scenes, as ask/tell optimizers.

All work in the unit cube [0,1]^d and see only the scalar margin V(theta) (+ the row dict), so they
are simulator-agnostic.  Each class names the paper it implements.  Nothing here assumes CARLA.

  sobol     Quasi-random baseline. Corso et al., JAIR 2021 (survey) frame random/QMC as the baseline
            every falsification method must beat.
  cem       Cross-entropy falsification: refit a Gaussian to the lowest-V elites each round.
            Corso et al. 2021 (CE under importance sampling/optimization); the sampler VerifAI ships.
  bo_lcb    Bayesian-optimization falsification: minimize a GP lower confidence bound of V.
            Corso et al. 2021 (BO for falsification); VerifAI (Dreossi et al., CAV 2019).
  straddle  Active level-set estimation: sample where a GP is uncertain AND near V = EPS.
            Bryan et al., NIPS 2005 (straddle heuristic); randomized-beta variant: Inatsu et al., TMLR 2024.
  qd_fail / qd_boundary
            MAP-Elites over descriptors (m1, m2) = time budgets of the two escape routes.
            Mouret & Clune 2015 (MAP-Elites); applied to scenario generation by Fontaine et al.
            (HRI scenarios, 2021) and CaDRE (arXiv 2403.13208, 2024).  qd_fail fills each cell with its
            most-unrecoverable scene; qd_boundary with the scene closest to V = EPS.
  bisect    One-factor boundary bisection (project-specific, not from a paper): move ONE parameter
            of an unrecoverable scene until it flips, then bisect -> matched pairs (proposal Sec. 5.1).
"""
import warnings

import numpy as np
from scipy.stats import qmc

warnings.filterwarnings("ignore", message="The balance properties of Sobol")

import core
from core import EPS, EDGE_BAND

# ---------------------------------------------------------------- baselines
class Sobol:
    name = "sobol"
    def __init__(self, rng, d=None, **_):
        self.d = d or core.D
        self.s = qmc.Sobol(self.d, scramble=True, seed=int(rng.integers(2**31)))
    def ask(self, n): return self.s.random(n)
    def tell(self, U, rows): pass


class CEM:
    name = "cem"
    def __init__(self, rng, d=None, elite_frac=0.25, explore=0.1, **_):
        self.d = d or core.D
        self.rng, self.ef, self.explore = rng, elite_frac, explore
        self.mu, self.sd = np.full(self.d, 0.5), np.full(self.d, 0.3)
        self.hist = []
    def ask(self, n):
        U = np.clip(self.mu + self.sd * self.rng.standard_normal((n, self.d)), 0, 1)
        flip = self.rng.random(n) < self.explore             # keep some uniform exploration (mode collapse)
        U[flip] = self.rng.random((int(flip.sum()), self.d))
        return U
    def tell(self, U, rows):
        self.hist += [(r["V"], np.asarray(r["u"])) for r in rows]
        self.hist.sort(key=lambda t: t[0])
        elite = np.array([u for _, u in self.hist[: max(3, int(self.ef * len(self.hist)))]])
        self.mu = 0.7 * elite.mean(0) + 0.3 * self.mu
        self.sd = np.maximum(0.7 * elite.std(0) + 0.3 * self.sd, 0.05)


# ---------------------------------------------------------------- GP-based
def apply_vtransform(kind, v):
    """Monotone, zero-preserving transform of V for GP fitting, so the boundary (V = EPS = 0) dominates the fit
    instead of the +1 plateau and the failure region.  'none' keeps V unchanged."""
    if kind in (None, "none"):
        return float(v)
    if kind == "clip":
        return float(np.clip(v, -0.5, 0.5))
    if kind == "tanh":
        return float(np.tanh(2.0 * v))
    raise ValueError(f"unknown vtransform {kind!r}")


class GPSearch:
    """mode='lcb'      minimize mu - kappa*sigma          (BO falsification)
       mode='straddle' maximize max(0, sqrt(beta)*sigma - |mu - EPS|), beta ~ chi2(2)   (randomized straddle)"""
    def __init__(self, rng, d=None, mode="straddle", n_init=None, n_cand=3000, kappa=2.0, vtransform="none", **_):
        self.d = d or core.D
        from sklearn.gaussian_process.kernels import ConstantKernel as C, Matern, WhiteKernel
        self.rng, self.mode, self.n_cand, self.kappa, self.vt = rng, mode, n_cand, kappa, vtransform
        self.name = "bo_lcb" if mode == "lcb" else "straddle"
        self.n_init = n_init or 2 * self.d
        self.sob = qmc.Sobol(self.d, scramble=True, seed=int(rng.integers(2**31)))
        self.X, self.y = [], []
        self.kernel = C(1.0) * Matern(np.full(self.d, 0.3), (0.05, 5.0), nu=2.5) + WhiteKernel(1e-3, (1e-6, 1e-1))

    def _fit(self, X, y, optimize=True):
        from sklearn.gaussian_process import GaussianProcessRegressor
        gp = GaussianProcessRegressor(self.kernel, normalize_y=True, alpha=1e-6,
                                      optimizer="fmin_l_bfgs_b" if optimize else None,
                                      n_restarts_optimizer=1 if optimize else 0,
                                      random_state=int(self.rng.integers(2**31)))
        gp.fit(np.asarray(X), np.asarray(y))
        if optimize:
            self.kernel = gp.kernel_
        return gp

    def _acq(self, mu, sd):
        if self.mode == "lcb":
            return -(mu - self.kappa * sd)
        beta = self.rng.chisquare(2)
        a = np.maximum(0.0, np.sqrt(beta) * sd - np.abs(mu - EPS))
        return a if a.max() > 0 else sd

    def ask(self, n):
        if len(self.X) < self.n_init:
            return self.sob.random(max(n, self.n_init - len(self.X)))[: max(n, 1)]
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            gp = self._fit(self.X, self.y)
            cand, Xf, yf, picks = self.sob.random(self.n_cand), list(self.X), list(self.y), []
            for _ in range(n):
                mu, sd = gp.predict(cand, return_std=True)
                j = int(np.argmax(self._acq(mu, sd)))
                picks.append(cand[j]); Xf.append(cand[j]); yf.append(float(mu[j]))   # kriging believer
                cand = np.delete(cand, j, 0)
                gp = self._fit(Xf, yf, optimize=False)
        return np.array(picks)

    def tell(self, U, rows):
        self.X += [np.asarray(r["u"]) for r in rows]
        self.y += [apply_vtransform(self.vt, r["V"]) for r in rows]


# ---------------------------------------------------------------- quality diversity
class MapElites:
    def __init__(self, rng, d=None, objective="fail", bins=(10, 10), ranges=((0, 8), (0, 10)),
                 n_init=20, sigma=0.12, **_):
        self.d = d or core.D
        self.rng, self.obj, self.bins, self.ranges = rng, objective, bins, ranges
        self.name = "qd_fail" if objective == "fail" else "qd_boundary"
        self.n_init, self.sigma, self.archive, self.seen = n_init, sigma, {}, 0

    def cell(self, row):
        c = []
        for v, b, (lo, hi) in zip((row["m1"], row["m2"]), self.bins, self.ranges):
            c.append(int(np.clip((v - lo) / (hi - lo) * b, 0, b - 1)))
        return tuple(c)

    def fitness(self, row):
        return -row["V"] if self.obj == "fail" else -abs(row["V"] - EPS)

    def ask(self, n):
        if self.seen < self.n_init or not self.archive:
            return self.rng.random((n, self.d))
        keys = list(self.archive)
        parents = [self.archive[keys[i]][1] for i in self.rng.integers(len(keys), size=n)]
        return np.clip(np.array(parents) + self.sigma * self.rng.standard_normal((n, self.d)), 0, 1)

    def tell(self, U, rows):
        for r in rows:
            self.seen += 1
            c, f = self.cell(r), self.fitness(r)
            if c not in self.archive or f > self.archive[c][0]:
                self.archive[c] = (f, np.asarray(r["u"]), r)


# ---------------------------------------------------------------- one-factor boundary bisection
class Bisect:
    name = "bisect"
    def __init__(self, rng, d=None, iters=4, **_):
        self.d = d or core.D
        self.rng, self.iters = rng, iters
        self.rows, self.chains, self.pairs, self.pending = [], [], [], []
        self.sob = qmc.Sobol(self.d, scramble=True, seed=int(rng.integers(2**31)))

    def _new_chain(self):
        bad = [r for r in self.rows if r["V"] < EPS]
        if not bad:
            return None
        u = np.asarray(bad[self.rng.integers(len(bad))]["u"])
        f = int(self.rng.integers(self.d))
        ends = [0.0, 1.0] if self.rng.random() < 0.5 else [1.0, 0.0]
        return {"u": u, "f": f, "ends": ends, "stage": "anchor", "lo": u.copy(), "hi": None, "k": 0}

    def _next_point(self, ch):
        if ch["stage"] == "anchor":
            p = ch["u"].copy(); p[ch["f"]] = ch["ends"][0]; return p
        return (ch["lo"] + ch["hi"]) / 2

    def ask(self, n):
        pts, self.pending = [], []
        active = [c for c in self.chains if c["stage"] != "done"]
        while len(pts) < n:
            ch = active.pop() if active else self._new_chain()
            if ch is None:                                    # nothing failing yet: seed with QMC
                pts.append(self.sob.random(1)[0]); self.pending.append(None); continue
            if not any(c is ch for c in self.chains):
                self.chains.append(ch)
            pts.append(self._next_point(ch)); self.pending.append(ch)
        return np.array(pts)

    def tell(self, U, rows):
        self.rows += rows
        for ch, r in zip(self.pending, rows):
            if ch is None:
                continue
            if ch["stage"] == "anchor":
                if r["V"] >= EPS:
                    ch["hi"], ch["stage"] = np.asarray(r["u"]), "bisect"
                else:
                    ch["ends"].pop(0)
                    if not ch["ends"]:
                        ch["stage"] = "done"                  # this factor alone cannot rescue the scene
            else:
                ch["k"] += 1
                if r["V"] < EPS: ch["lo"] = np.asarray(r["u"])
                else: ch["hi"] = np.asarray(r["u"])
                if ch["k"] >= self.iters:
                    ch["stage"] = "done"
                    self.pairs.append({"varied_index": ch["f"], "u_unrecoverable": ch["lo"].tolist(),
                                       "u_recoverable": ch["hi"].tolist(),
                                       "gap_unit": float(abs(ch["hi"][ch["f"]] - ch["lo"][ch["f"]]))})


REGISTRY = {
    "sobol": lambda rng, **kw: Sobol(rng, **kw),
    "cem": lambda rng, **kw: CEM(rng, **kw),
    "bo_lcb": lambda rng, **kw: GPSearch(rng, mode="lcb", **kw),
    "straddle": lambda rng, **kw: GPSearch(rng, mode="straddle", **kw),
    "qd_fail": lambda rng, **kw: MapElites(rng, objective="fail", **kw),
    "qd_boundary": lambda rng, **kw: MapElites(rng, objective="boundary", **kw),
    "bisect": lambda rng, **kw: Bisect(rng, **kw),
}
