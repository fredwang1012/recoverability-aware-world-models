"""
cases/occlusion.py - RC10, RC11, RC14 (C4 partial observation) on the straight two-lane map (ego in the RIGHT lane, usable LEFT lane).
Observation gate first (the feed withholds the hidden vehicle; CARLA itself has no occluder).  Intentions are documented here only; labels
always come from evidence (sim.harness.evaluate -> contract.classify).

Site / corridor mapping (controller ruling; info_tree's LEFT/RIGHT corridors):
    one STOPPED hidden vehicle `hid`, ahead of the ego by the bumper gap D, sits in exactly ONE of the two escape corridors.
    right corridor = the ego's own lane (keep lane / stop in lane)         <->  info_tree `fixed_right`, world "B blocks RIGHT"
    left  corridor = the adjacent lane (change lane)                       <->  info_tree `fixed_left`,  world "A blocks LEFT"
    world `hid_left_lane`  (A): hid in the left lane  (lat = -LANE_W)  -> staying / stopping is safe, a left change collides
    world `hid_own_lane`   (B): hid in the ego lane   (lat = 0)        -> only the left change (with braking) works; stopping cannot (D < D_stop)
    lat is positive to the RIGHT, so the left lane is lat = -LANE_W.  The world's `spawn` override places the vehicle; the feed withholds it
    until t_reveal, so both worlds deliver IDENTICAL observation histories before the reveal.

    RC10  early evidence (t_reveal at or before the analytic threshold): R_REF only through shared_slowdown_then_branch
          (stop_in_place, fixed_left, fixed_right are each refuted in one world)
    RC11  late evidence (t_reveal after the threshold): U_INFORMATION - each world is individually solvable (hindsight) but no causal policy of the
          library survives both; if a common stop prefix survives (D >= stopping distance) the label is R_REF, never U_INFORMATION
    RC14  two histories with an identical CURRENT frame (hid occluded again after t_forget) but different earlier observations (hid seen in the
          own lane vs seen in the left lane): different history-conditioned profiles; the current-frame-only policy fails where the
          memory policy succeeds.  "Decisive evidence before t0" is implemented as: decisive evidence at the start of the episode
          (t < t_forget), after which it stays causally available in the delivered history.
"""
from sim import policies as P
from sim.harness import evaluate

from cases import two_lane as TL

LANE_W, EGO_HL, EGO_HW = TL.LANE_W, TL.EGO_HL, TL.EGO_HW
HID_HL, HID_HW = TL.BLK_HL, TL.BLK_HW
A_LAT_MAX = 8.0                       # m/s^2, controller ruling (same as the core / compositional cases)
HID = "hid"
LC_TIME = 2.0                         # emergency lane-change duration (controller ruling)
LEFT_LAT, OWN_LAT = -LANE_W, 0.0


def _named(factory, name):
    def mk():
        p = factory()
        p.name = name
        return p
    return mk


def spec(theta, cfg):
    """The hid placement here is a default (ego lane); every world overrides it through `spawn`."""
    sp = TL._base(theta, theta.get("tau_s", 0.25) + 12.0, {HID: TL._blk(theta["gap_m"], OWN_LAT)}, hold_s=cfg.hold_s, post_s=cfg.post_maneuver_s)
    sp["a_lat_max"] = A_LAT_MAX
    return sp


def _spawn(theta, lat):
    return {HID: {"x": theta["gap_m"] + HID_HL, "lat": lat}}


def worlds(t_reveal, theta, forget=None):
    """(A) hid_left_lane, (B) hid_own_lane; both reveal the hidden vehicle at t_reveal (a tick multiple)."""
    kw = {"reveal": {HID: t_reveal}, **({"forget": {HID: forget}} if forget is not None else {})}
    return [{"name": "hid_left_lane", "actors": {}, "spawn": _spawn(theta, LEFT_LAT), **kw},
            {"name": "hid_own_lane", "actors": {}, "spawn": _spawn(theta, OWN_LAT), **kw}]


def library(lc_time=LC_TIME):
    return [_named(lambda: P.BrakeHold(), "stop_in_place"),
            _named(lambda: P.SharedSlowdownThenBranch(lc_time=lc_time, blind="change_left"), "fixed_left"),
            _named(lambda: P.LaneFollow(), "fixed_right"),
            lambda: P.SharedSlowdownThenBranch(lc_time=lc_time)]


def rc14_worlds(t_forget, theta):
    """Both hid placements are SEEN from t = 0 and occluded again from t_forget on (t_forget is the sweep parameter; the time since the last observation at decision time t is t - t_forget, there is no separate field)."""
    kw = {"reveal": {HID: 0.0}, "forget": {HID: t_forget}}
    return [{"name": "seen_left_then_occluded", "actors": {}, "spawn": _spawn(theta, LEFT_LAT), **kw},
            {"name": "seen_own_then_occluded", "actors": {}, "spawn": _spawn(theta, OWN_LAT), **kw}]


def rc14_library(lc_time=LC_TIME):
    return library(lc_time) + [lambda: P.SharedSlowdownThenBranch(lc_time=lc_time, memory=False)]


def run(case, plant_factory, theta, cfg, backend, scope, worlds_=None, library_=None, spec_=None, family=None, split_group=None, **kw):
    """case RC10 / RC11 use worlds(theta['t_reveal_s']); RC14 uses rc14_worlds(theta['t_forget_s'])."""
    sp = spec_ or spec(theta, cfg)
    if case == "RC14":
        ws = worlds_ or rc14_worlds(theta.get("t_forget_s", 0.6), theta)
        lib = library_ or rc14_library()
    else:
        ws = worlds_ or worlds(theta["t_reveal_s"], theta)
        lib = library_ or library()
    fam = family or f"occlusion_{case}"
    return evaluate(plant_factory, sp, lib, ws, theta, case, fam, split_group or fam, backend, scope, cfg=cfg, **kw)
