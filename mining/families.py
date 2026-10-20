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

    def meta(self):
        """Extra provenance written into every record row (library id, fixed plant parameters)."""
        return {}

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
                      family_id="corridor", with_bypass=True, **ctx.get("run_kw", {}))


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
        return TL.run("RC03", lambda: ctx["factory"](th), th, ctx["cfg"], ctx["backend"], ctx["scope"], family="rc03",
                      **ctx.get("run_kw", {}))


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
        return TL.run("RC07", lambda: ctx["factory"](th), th, ctx["cfg"], ctx["backend"], ctx["scope"], family="rc07",
                      **ctx.get("run_kw", {}))


class InfoGap(Family):
    """D5 info-gap family = RC10/RC11 (cases.occlusion): theta = (D, v, t_reveal).  D = bumper gap to the hidden stopped vehicle, which sits in
    the ego lane or the adjacent lane (two worlds); the feed withholds it until t_reveal.  tau 0.25 s, braking authority fixed at the
    calibrated decel target, library lc_time 2.0, a_lat_max 8 (same declared scene as the Task 7 sweep).  Records keep the delivered
    observation history (E1).  Search value per algorithm: see infogap.py; labels and the objective come from the full record."""
    name = "infogap"
    names = ["gap_m", "v_mps", "t_reveal_s"]
    lo = np.array([14.0, 12.0, 0.0])
    hi = np.array([34.0, 18.0, 1.2])
    TAU = 0.25

    def _theta(self, v):
        th = dict(zip(self.names, map(float, v)))
        th["gap_m"], th["v_mps"] = round(th["gap_m"], 1), round(th["v_mps"], 1)
        th["t_reveal_s"] = round(_q(th["t_reveal_s"]), 2)
        th["tau_s"], th["b_mps2"] = self.TAU, CARLA_DECEL
        return th

    def descriptors(self, th):
        return th["gap_m"] / th["v_mps"], th["t_reveal_s"]

    def meta(self):
        return {"library_id": "occ_lc2.0_al8lc2", "lc_time": 2.0, "a_lat_max": 8.0, "b_fixed_mps2": CARLA_DECEL}

    def search_value(self, rec, algo):
        import infogap as IG
        f = IG.cem_surrogate if algo == "cem" else IG.straddle_surrogate
        return float(f(rec.R_norm, rec.margin_config.get("eps", 0.0)))

    def run(self, th, ctx):
        from cases import occlusion as OC
        case = "RC10" if th["t_reveal_s"] <= 0.5 else "RC11"
        return OC.run(case, lambda: ctx["factory"](th), th, ctx["cfg"], ctx["backend"], ctx["scope"], family="infogap",
                      split_group=f"infogap_D{th['gap_m']:g}_v{th['v_mps']:g}", keep_history=True, **ctx.get("run_kw", {}))


FAMILIES = {f.name: f for f in (Corridor(), Rc03(), Rc07())}
INFOGAP = InfoGap()          # D5 family: driven by scripts/mine_infogap.py (its search value is per algorithm), not by mine_cases.summarise


# ----------------------------------------------------------------------------- CARLA variants (Task 5; mock families above unchanged)
CARLA_DECEL = 6.19            # the only decel target with a distance-equivalent calibration (profile floor is 4.77)
B_CAL, TAU_PLANT = 6.211, 0.05      # Task 3 calibration: measured brake_hold decel, one-tick plant onset (shared trace, not independent)
CARLA_A_LAT_MAX = 8.0
CARLA_LC_TIME = 2.0


class CorridorCarla(Corridor):
    """Corridor on CARLA: theta = (gap, v, tau, prefix); b is FIXED at the calibrated decel target (RULING 1), not searched.
    Anchor uses the calibrated b_cal and tau_eff = tau + one-tick plant onset (constants shared with the calibration trace)."""
    names = ["gap_m", "v_mps", "tau_s", "prefix_s"]
    lo = np.array([8.0, 6.0, 0.0, 0.0])
    hi = np.array([40.0, 20.0, 0.6, 0.6])

    def _theta(self, v):
        th = super()._theta(v)
        th["b_mps2"] = CARLA_DECEL
        return th

    def descriptors(self, th):
        return th["gap_m"] / th["v_mps"], th["prefix_s"]

    def anchor(self, th, ctx):
        c = ctx["cfg"].reserve_m
        return corridor_anchor_m(th["gap_m"], th["v_mps"], th["tau_s"] + TAU_PLANT, th["prefix_s"], B_CAL, c)

    def meta(self):
        return {"library_id": "corridor_v1", "b_fixed_mps2": CARLA_DECEL}


def corridor_anchor_m(gap, v, tau_eff, prefix, b, c):
    """Signed analytic margin in metres: gap - v*(tau_eff+prefix) - v^2/(2b) - c."""
    return gap - v * (tau_eff + prefix) - v ** 2 / (2 * b) - c


class Rc03Carla(Rc03):
    """RC03 on CARLA (RULING 2): lc_time 2.0 library, spec a_lat_max = 8 (dynamic-limit margin), b fixed at 6.19.  Under that limit a
    bypass window exists only at higher speeds, hence v up to 24 and gap up to 60."""
    names = ["gap_m", "v_mps", "tau_s"]
    lo = np.array([10.0, 8.0, 0.0])
    hi = np.array([60.0, 24.0, 0.5])

    def _theta(self, v):
        th = super()._theta(v)
        th["b_mps2"] = CARLA_DECEL
        return th

    def descriptors(self, th):
        return th["gap_m"] / th["v_mps"], th["tau_s"]

    def meta(self):
        return {"library_id": f"rc03_carla_lc{CARLA_LC_TIME:g}_full_alat{CARLA_A_LAT_MAX:g}", "lc_time": CARLA_LC_TIME,
                "a_lat_max": CARLA_A_LAT_MAX, "b_fixed_mps2": CARLA_DECEL}

    def run(self, th, ctx):
        spec = dict(TL.rc03_spec(th, ctx["cfg"]), a_lat_max=CARLA_A_LAT_MAX)
        return TL.run("RC03", lambda: ctx["factory"](th), th, ctx["cfg"], ctx["backend"], ctx["scope"], family="rc03",
                      library=TL.rc03_library(True, CARLA_LC_TIME), spec=spec, **ctx.get("run_kw", {}))


FAMILIES_CARLA = {f.name: f for f in (CorridorCarla(), Rc03Carla())}


def mock_factory(th):
    from sim.plant_kinematic import KinematicPlant
    return KinematicPlant(b_max=th.get("b_mps2", 6.0))


def mock_ctx(cfg=M.DEFAULT_V2):
    return {"backend": "mock", "factory": mock_factory, "scope": "mock_replay", "cfg": cfg}
