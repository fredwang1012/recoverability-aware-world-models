"""
cases/two_lane.py - RC03, RC07, RC16, RC19, RC20 on the 'two_lane' site (two same-direction lanes; ego in the right lane).

Intentions are documented here only; labels always come from evidence (sim.harness.evaluate -> contract.classify).
    RC03  front blocker in ego lane only; braking cannot stop in time but the adjacent lane is free (bypass witness).
          paired negative: close the escape lane with a second blocker (NOT pre-labelled U_MODEL).
    RC07  front blocker + fast rear actor in the left lane arriving after the first avoidance; stopping in the left lane
          (premature hand-back) is refuted by the rear actor; bypass-then-return is the continuation.
    RC16  a thin post beside the lane: the centre line clears it, the body (plus reserve) may not.
    RC19  benign anchor (clear road) and a label-preserving nuisance twin (distant non-interacting actor).
    RC20  same scene, restricted / budget-limited / expanded policy libraries: UNRESOLVED / U_LIBRARY -> R_REF.
"""
from sim import policies as P
from sim.harness import evaluate

LANE_W = 3.5
EGO_HL, EGO_HW = 2.3, 0.95
BLK_HL, BLK_HW = 2.3, 0.95
POST_HL, POST_HW = 0.9, 0.3                      # bicycle-like thin obstacle
ROAD = (-1.5 * LANE_W, 0.5 * LANE_W)
CLEAN = {"name": "static_blocker", "actors": {}}


def _ego(v):
    return {"x": -EGO_HL, "lat": 0.0, "v": v, "hl": EGO_HL, "hw": EGO_HW}


def _blk(x_rear, lat=0.0):
    return {"x": x_rear + BLK_HL, "lat": lat, "v": 0.0, "hl": BLK_HL, "hw": BLK_HW, "static": True}


def _base(theta, horizon, actors, lane_tol=0.5, hold_s=4.0, post_s=4.0):
    return {"ego": _ego(theta["v_mps"]), "actors": actors, "road": ROAD, "lane_w": LANE_W, "horizon_s": horizon,
            "delay": {"obs": 0.0, "plan": 0.0, "act": theta.get("tau_s", 0.25)}, "obs_hz": 20.0, "policy_hz": 20.0,
            "terminal": {"hold_s": hold_s, "post_maneuver_s": post_s}, "lane_tol": lane_tol}


# ----------------------------------------------------------------------------- RC03
def rc03_spec(theta, cfg):
    acts = {"blk": _blk(theta["gap_m"])}
    if theta.get("closed", False):
        acts["blk_l"] = _blk(theta["gap_m"], -LANE_W)
    horizon = theta.get("tau_s", 0.25) + max(theta["v_mps"] / 3.0, 3.0) + cfg.hold_s + 1.0
    return _base(theta, horizon, acts, hold_s=cfg.hold_s, post_s=cfg.post_maneuver_s)


def rc03_library(with_bypass=True, lc_time=2.0):
    """lc_time=2.0 is the mock default; CARLA's measured braking (b ~ 6.2) beats a 2.0 s lane change at every tested speed, so the
    CARLA runs pass an emergency lc_time (scripts/carla_core.py, library id rc03_carla_lc1.0)."""
    lib = [lambda: P.BrakeHold(), lambda: P.LaneFollow()]
    if with_bypass:
        lib += [lambda: P.Bypass(side=-1, lc_time=lc_time), lambda: P.Bypass(side=-1, lc_time=lc_time, brake_s=0.4)]
    return lib


# ----------------------------------------------------------------------------- RC07
def rc07_spec(theta, cfg, horizon=None):
    acts = {"blk": _blk(theta["gap_m"]),
            "rear": {"x": -theta["rear_gap_m"], "lat": -LANE_W, "v": theta["rear_v_mps"], "hl": 2.3, "hw": 0.95}}
    h = horizon or (theta.get("tau_s", 0.25) + 12.0)
    return _base(theta, h, acts, hold_s=cfg.hold_s, post_s=cfg.post_maneuver_s)


RC07_WORLD = {"name": "rear_cruise", "actors": {"rear": {"mode": "cruise"}}}


def rc07_library():
    return [lambda: P.BypassThenStop(lc_time=2.0), lambda: P.BypassThenReturn(lc_time=2.0), lambda: P.BrakeHold()]


# ----------------------------------------------------------------------------- RC16
def rc16_spec(theta, cfg):
    post = {"x": 60.0, "lat": theta["offset_m"], "v": 0.0, "hl": POST_HL, "hw": POST_HW, "static": True}
    return _base(theta, 60.0 / theta["v_mps"] + 7.0, {"post": post}, lane_tol=1.2, hold_s=cfg.hold_s,
                 post_s=cfg.post_maneuver_s)


def rc16_library():
    return [lambda: P.LaneFollow(), lambda: P.Nudge(lat_offset=-1.0, lc_time=1.5), lambda: P.BrakeHold()]


# ----------------------------------------------------------------------------- RC19
def rc19_spec(theta, cfg, nuisance=False):
    acts = {"far": {"x": 250.0, "lat": -LANE_W, "v": 0.0, "hl": 2.3, "hw": 0.95, "static": True}} if nuisance else {}
    return _base(theta, 8.0, acts, hold_s=cfg.hold_s, post_s=cfg.post_maneuver_s)


# ----------------------------------------------------------------------------- runner
def run(case, plant_factory, theta, cfg, backend, scope, world=None, library=None, spec=None, family=None, **kw):
    maker = {"RC03": (rc03_spec, rc03_library, CLEAN), "RC07": (rc07_spec, rc07_library, RC07_WORLD),
             "RC16": (rc16_spec, rc16_library, CLEAN), "RC19": (rc19_spec, lambda: [lambda: P.LaneFollow()], CLEAN)}[case]
    sp = spec or maker[0](theta, cfg)
    lib = library or maker[1]()
    fam = family or f"two_lane_{case}"
    return evaluate(plant_factory, sp, lib, [world or maker[2]], theta, case, fam, fam, backend, scope, cfg=cfg, **kw)
