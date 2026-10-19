"""
cases/compositional.py - RC04, RC05, RC06, RC08, RC15, RC18 (C3 compositional cases) on the two-lane site (ego in the right lane,
usable left lane).  Intentions are documented here only; labels always come from evidence (sim.harness.evaluate -> contract.classify).

    RC04  front blocker; braking alone cannot stop in time and an IMMEDIATE lane change breaks the lateral-acceleration authority
          (a_lat_max) or clearance; braking first and then changing lanes (StagedBrakeChange) is the supported continuation.
    RC05  abort an unsafe lane change: every policy starts DURING a physically executed lane-change prefix (the episode itself runs
          it; no lateral teleport).  A hidden rear vehicle in the target lane is revealed at t_reveal; two worlds: its gap closes
          (cruise) or stays open (it brakes).  commit vs abort_and_stabilize vs abort_if_closing (observation-conditioned).
    RC06  yield, then use a time-varying gap: a faster vehicle passes in the target lane; wait_observe_enter vs immediate_entry.
    RC08  protected waiting with route non-completion: safety is the label; route_goal_completed is stored SEPARATELY per run
          (traces[...]["route_goal_completed"]) and never enters the margin / terminal test / label.
    RC15  yielding must be observed: two worlds (the adjacent vehicle yields from trigger_t / does not); always_merge vs
          observe_yield_else_fallback.
    RC18  requested control exceeds authority: spec a_lat_max (m/s^2) and a_comb_max (friction circle sqrt(a_long^2 + a_lat^2),
          measured accelerations) as dynamic-limit margins; the requested brake+steer command is refuted by the limit margin.
          The CARLA tyre friction of the physics profile is non-physical (friction_fixed 5.0), so CARLA's own tyre limit is NOT the
          authority; the declared 8.0 m/s^2 limits are.
"""
from sim import policies as P
from sim.harness import evaluate

from cases import two_lane as TL

LANE_W, EGO_HL, EGO_HW = TL.LANE_W, TL.EGO_HL, TL.EGO_HW
A_LAT_MAX = 8.0                  # m/s^2, controller ruling (same value as the core cases)
A_COMB_MAX = 8.0                 # m/s^2, friction-circle limit for RC18
CLEAN = {"name": "static_blocker", "actors": {}}


def _named(factory, name):
    """Policy factory whose instance carries a parameter-specific name (so R_raw rows are unambiguous in the record)."""
    def mk():
        p = factory()
        p.name = name
        return p
    return mk


def _base(theta, horizon, actors, cfg, **extra):
    sp = TL._base(theta, horizon, actors, hold_s=cfg.hold_s, post_s=cfg.post_maneuver_s)
    sp["a_lat_max"] = A_LAT_MAX
    sp.update(extra)
    return sp


def _stop_horizon(theta, cfg):
    return theta.get("tau_s", 0.25) + max(theta["v_mps"] / 3.0, 3.0) + cfg.hold_s + 1.0


# ----------------------------------------------------------------------------- RC04
def rc04_spec(theta, cfg):
    return _base(theta, _stop_horizon(theta, cfg), {"blk": TL._blk(theta["gap_m"])}, cfg)


def rc04_library(lc_time=1.6, ps=(0.4, 0.8)):
    lib = [lambda: P.BrakeHold(), lambda: P.LaneFollow(),
           _named(lambda: P.Bypass(side=-1, lc_time=lc_time), "immediate_lane_change")]
    lib += [_named((lambda p=p: P.StagedBrakeChange(p=p, lc_time=lc_time)), f"staged_brake_{p:g}_change") for p in ps]
    return lib


# ----------------------------------------------------------------------------- RC05
def rc05_spec(theta, cfg):
    acts = {"blk": TL._blk(theta["gap_m"]),
            "rear": {"x": -theta["rear_gap_m"], "lat": -LANE_W, "v": theta["rear_v_mps"], "hl": 2.3, "hw": 0.95}}
    return _base(theta, theta.get("tau_s", 0.25) + 11.0, acts, cfg)


def rc05_worlds(t_reveal, b_open=5.0):
    """Mode chosen once per world: the rear vehicle keeps its speed (gap closes) or brakes from t = 0 (gap stays open)."""
    return [{"name": "gap_closes", "actors": {"rear": {"mode": "cruise"}}, "reveal": {"rear": t_reveal}},
            {"name": "gap_open", "actors": {"rear": {"mode": "delayed_brake", "t_brake": 0.0, "b": b_open}}, "reveal": {"rear": t_reveal}}]


def rc05_library(prefix_s=1.0, lc_time=3.0, abort_lc=1.5, ttc_s=4.0):
    return [lambda: P.LaneChangePrefix(prefix_s=prefix_s, lc_time=lc_time, abort_lc=abort_lc),
            lambda: P.AbortAndStabilize(prefix_s=prefix_s, lc_time=lc_time, abort_lc=abort_lc),
            lambda: P.AbortIfClosing(prefix_s=prefix_s, lc_time=lc_time, abort_lc=abort_lc, ttc_s=ttc_s)]


# ----------------------------------------------------------------------------- RC06
def rc06_spec(theta, cfg):
    acts = {"blk": TL._blk(theta["gap_m"]),
            "pass": {"x": -theta["pass_gap_m"], "lat": -LANE_W, "v": theta["pass_v_mps"], "hl": 2.3, "hw": 0.95}}
    return _base(theta, theta.get("tau_s", 0.25) + 12.0, acts, cfg)


RC06_WORLD = {"name": "passer_cruise", "actors": {"pass": {"mode": "cruise"}}}


def rc06_library(lc_time=2.0):
    return [lambda: P.ImmediateEntry(lc_time=lc_time), lambda: P.WaitObserveEnter(lc_time=lc_time),
            lambda: P.WaitObserveEnter(lc_time=lc_time, wait_brake_frac=0.3), lambda: P.BrakeHold(), lambda: P.LaneFollow()]


# ----------------------------------------------------------------------------- RC08
def rc08_spec(theta, cfg):
    acts = {"blk": TL._blk(theta["gap_m"])}
    if theta.get("closed", False):
        acts["blk_l"] = TL._blk(theta["gap_m"], -LANE_W)
    goal_x = theta["gap_m"] + 2 * TL.BLK_HL + theta.get("goal_beyond_m", 15.0)
    horizon = theta.get("tau_s", 0.25) + max(goal_x / theta["v_mps"], theta["v_mps"] / 3.0) + cfg.hold_s + 3.0
    return _base(theta, horizon, acts, cfg, route_goal={"x": goal_x})


def rc08_library(lc_time=2.0):
    return [lambda: P.BrakeHold(), lambda: P.Bypass(side=-1, lc_time=lc_time), lambda: P.LaneFollow()]


# ----------------------------------------------------------------------------- RC15
def rc15_spec(theta, cfg):
    acts = {"blk": TL._blk(theta["gap_m"]),
            "adj": {"x": -theta["adj_gap_m"], "lat": -LANE_W, "v": theta["adj_v_mps"], "hl": 2.3, "hw": 0.95}}
    return _base(theta, theta.get("tau_s", 0.25) + 11.0, acts, cfg)


def rc15_worlds(trigger_t=0.5, react_s=0.5, v_yield=5.0, b=3.0):
    """Per-world adjacent-vehicle mode, never re-drawn per tick: yields from trigger_t (+react_s), or explicitly does not."""
    return [{"name": "adj_yields", "actors": {"adj": {"mode": "yield", "trigger_t": trigger_t, "react_s": react_s,
                                                       "v_yield": v_yield, "b": b}}},
            {"name": "adj_nonyield", "actors": {"adj": {"mode": "nonyield"}}}]


def rc15_library(lc_time=2.0, wait_s=2.0, drop=1.5):
    return [lambda: P.AlwaysMerge(lc_time=lc_time), lambda: P.ObserveYieldElseFallback(lc_time=lc_time, wait_s=wait_s, drop=drop),
            lambda: P.BrakeHold(), lambda: P.LaneFollow()]


# ----------------------------------------------------------------------------- RC18
def rc18_spec(theta, cfg, comb=True):
    sp = _base(theta, _stop_horizon(theta, cfg), {"blk": TL._blk(theta["gap_m"])}, cfg)
    if comb:
        sp["a_comb_max"] = A_COMB_MAX
    return sp


def rc18_library(lc_times=(1.2, 2.0), lc_feasible=2.0):
    """requested brake+steer commands (several lateral-offset times), the feasible keep-speed alternative, and braking only."""
    lib = [lambda: P.BrakeHold(), lambda: P.LaneFollow(), _named(lambda: P.Bypass(side=-1, lc_time=lc_feasible), "bypass_left")]
    lib += [(lambda lc=lc: P.BrakeSteer(side=-1, lc_time=lc, brake_frac=1.0)) for lc in lc_times]
    return lib


# ----------------------------------------------------------------------------- runner
def run(case, plant_factory, theta, cfg, backend, scope, worlds=None, library=None, spec=None, family=None, **kw):
    maker = {"RC04": (rc04_spec, rc04_library, [CLEAN]),
             "RC05": (rc05_spec, rc05_library, None),
             "RC06": (rc06_spec, rc06_library, [RC06_WORLD]),
             "RC08": (rc08_spec, rc08_library, [CLEAN]),
             "RC15": (rc15_spec, rc15_library, None),
             "RC18": (rc18_spec, rc18_library, [CLEAN])}[case]
    sp = spec or maker[0](theta, cfg)
    lib = library or maker[1]()
    ws = worlds or maker[2]
    if ws is None:
        ws = rc05_worlds(theta.get("t_reveal_s", 1.25)) if case == "RC05" else rc15_worlds()
    fam = family or f"compositional_{case}"
    return evaluate(plant_factory, sp, lib, ws, theta, case, fam, fam, backend, scope, cfg=cfg, **kw)
