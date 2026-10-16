"""
families.py - parameter spaces theta in [0,1]^d for the physical case families, so the ask/tell algorithms in algos.py can
search INSIDE a case family (spec phase D1).  Search uses the finite search-grade margin G_search (never labels); every
sampled scene is then labelled from the FULL evidence record (all policies x worlds, contract.classify).

A family provides
    names, lo, hi            theta axes (the case sweep blocks)
    decode(u) -> theta       dict with SI units
    run(theta, plant_ctx)    -> ExecutionRecord (label-grade evidence)
    descriptors(theta)       (m1, m2) for MAP-Elites
    anchor(theta)            analytic G in metres where one exists (corridor), else None
plant_ctx = {"backend": "mock"|"carla", "factory": callable(theta) -> plant, "scope": evidence_scope, "cfg": MarginConfigV2}
"""
import math

import numpy as np

import margin as M
from cases import corridor as CO
from cases import two_lane as TL
from sim.types import DT


def _q(x, dt=DT):
    return round(x / dt) * dt


class Family:
    name = "family"
    names, lo, hi = [], np.array([]), np.array([])

    @property
    def d(self):
        return len(self.names)

    def decode(self, u):
        u = np.clip(np.asarray(u, float), 0.0, 1.0)
        v = self.lo + u * (self.hi - self.lo)
        return self._theta(v)

    def _theta(self, v):
        return dict(zip(self.names, map(float, v)))

    def descriptors(self, th):
        return 0.0, 0.0

    def anchor(self, th, ctx):
        return None

    def run(self, th, ctx):                                  # pragma: no cover
        raise NotImplementedError


class Corridor(Family):
    """RC01 / RC02 / RC12 / RC17: theta = (gap, v, tau, prefix, b).  b is the braking authority (plant parameter)."""
    name = "corridor"
    names = ["gap_m", "v_mps", "tau_s", "prefix_s", "b_mps2"]
    lo = np.array([8.0, 6.0, 0.0, 0.0, 4.0])
    hi = np.array([40.0, 20.0, 0.6, 0.6, 9.0])

    def _theta(self, v):
        th = dict(zip(self.names, map(float, v)))
        th["tau_s"], th["prefix_s"] = _q(th["tau_s"]), math.ceil(th["prefix_s"] / DT - 1e-9) * DT
        return th

    def descriptors(self, th):
        return th["gap_m"] / th["v_mps"], th["b_mps2"]

    def anchor(self, th, ctx):
        """Analytic signed margin in metres: gap - (v*(tau+prefix) + v^2/(2b)) - reserve (quantised delays, plant b)."""
        c = ctx["cfg"].reserve_m
        return th["gap_m"] - th["v_mps"] * (th["tau_s"] + th["prefix_s"]) - th["v_mps"] ** 2 / (2 * th["b_mps2"]) - c

    def run(self, th, ctx):
        return CO.run(lambda: ctx["factory"](th), th, ctx["cfg"], ctx["backend"], ctx["scope"], case_id="RC01",
                      family_id="corridor", with_bypass=True)


class Rc03(Family):
    """RC03: front blocker in the ego lane, adjacent lane free: theta = (gap, v, tau, b)."""
    name = "rc03"
    names = ["gap_m", "v_mps", "tau_s", "b_mps2"]
    lo = np.array([12.0, 8.0, 0.0, 3.0])
    hi = np.array([45.0, 16.0, 0.5, 8.0])

    def _theta(self, v):
        th = dict(zip(self.names, map(float, v)))
        th["tau_s"] = _q(th["tau_s"])
        return th

    def descriptors(self, th):
        return th["gap_m"] / th["v_mps"], th["b_mps2"]

    def run(self, th, ctx):
        return TL.run("RC03", lambda: ctx["factory"](th), th, ctx["cfg"], ctx["backend"], ctx["scope"], family="rc03")


class Rc07(Family):
    """RC07: theta = (gap, rear_gap, rear_v, v); the rear actor arrives after the first avoidance."""
    name = "rc07"
    names = ["gap_m", "rear_gap_m", "rear_v_mps", "v_mps"]
    lo = np.array([20.0, 30.0, 16.0, 8.0])
    hi = np.array([45.0, 110.0, 28.0, 16.0])

    def descriptors(self, th):
        return th["gap_m"] / th["v_mps"], th["rear_gap_m"] / max(th["rear_v_mps"] - th["v_mps"], 1.0)

    def run(self, th, ctx):
        th = dict(th, tau_s=0.25, b_mps2=6.0)
        return TL.run("RC07", lambda: ctx["factory"](th), th, ctx["cfg"], ctx["backend"], ctx["scope"], family="rc07")


FAMILIES = {f.name: f for f in (Corridor(), Rc03(), Rc07())}


def mock_factory(th):
    from sim.plant_kinematic import KinematicPlant
    return KinematicPlant(b_max=th.get("b_mps2", 6.0))


def mock_ctx(cfg=M.DEFAULT_V2):
    return {"backend": "mock", "factory": mock_factory, "scope": "mock_replay", "cfg": cfg}
