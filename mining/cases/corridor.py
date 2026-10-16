"""
cases/corridor.py - RC01 / RC02 / RC12 / RC17 on the 'corridor' site: a straight road section closed by a full-width
stationary blocker (two static vehicles abreast, one per lane).  Intentions live in the docstrings only; labels come from
evidence (sim.harness.evaluate -> contract.classify).

theta = {gap_m, v_mps, tau_s, prefix_s}
    gap_m    bumper-to-blocker clearance at t0 (ego front bumper at x = 0, blocker rear face at x = gap_m)
    tau_s    observation-to-effective-command delay (RC17), configured in the command pipeline (act delay)
    prefix_s baseline 'keep speed' time before handing over to the recovery policy (RC12)
"""
from sim import policies as P
from sim.harness import evaluate

LANE_W = 3.5
EGO_HL, EGO_HW = 2.3, 0.95
BLK_HL, BLK_HW = 2.3, 0.95
WORLD = {"name": "stationary_blocker_no_rear", "actors": {}}
LIBRARY_ID = "corridor_v1"


def spec(theta, hold_s=4.0, post_s=4.0, obs_hz=20.0):
    gap, v, tau = theta["gap_m"], theta["v_mps"], theta["tau_s"]
    prefix = theta.get("prefix_s", 0.0)
    horizon = prefix + tau + v / 4.0 + hold_s + 1.0            # slowest plausible stop (4 m/s^2) + hold
    return {
        "ego": {"x": -EGO_HL, "lat": 0.0, "v": v, "hl": EGO_HL, "hw": EGO_HW},
        "actors": {"blk_r": {"x": gap + BLK_HL, "lat": 0.0, "v": 0.0, "hl": BLK_HL, "hw": BLK_HW, "static": True},
                   "blk_l": {"x": gap + BLK_HL, "lat": -LANE_W, "v": 0.0, "hl": BLK_HL, "hw": BLK_HW, "static": True}},
        "road": (-1.5 * LANE_W, 0.5 * LANE_W), "lane_w": LANE_W, "horizon_s": horizon,
        "delay": {"obs": 0.0, "plan": 0.0, "act": tau}, "obs_hz": obs_hz, "policy_hz": obs_hz,
        "terminal": {"hold_s": hold_s, "post_maneuver_s": post_s},
    }


def library(theta, with_bypass=True):
    prefix = theta.get("prefix_s", 0.0)
    wrap = (lambda inner: (lambda: P.BaselineThenHandover(inner(), prefix_s=prefix))) if prefix > 0 else (lambda f: f)
    lib = [wrap(lambda: P.BrakeHold())]
    if with_bypass:
        lib += [wrap(lambda: P.Bypass(side=-1, lc_time=2.5)), lambda: P.LaneFollow()]
    return lib


def run(plant_factory, theta, cfg, backend, evidence_scope, case_id="RC01", family_id=None, with_bypass=True, **kw):
    fam = family_id or f"corridor_{case_id}"
    return evaluate(plant_factory, spec(theta, cfg.hold_s, cfg.post_maneuver_s), library(theta, with_bypass), [WORLD], theta, case_id, fam,
                    fam, backend, evidence_scope, cfg=cfg, **kw)
