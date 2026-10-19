"""
cases/negatives.py - RC09 and RC13 (C5 difficult negatives).  Intentions are documented here only; labels always come from evidence
(sim.harness.evaluate -> contract.classify).

    RC09  front/rear conflict with no escape.  Site: the two-lane straight (ego in the right lane; the right edge is the road edge).
          Front: a stationary blocker `blk` in the ego lane at bumper gap `gap_m`.  Rear: a vehicle `rear` in the ego lane, `rear_gap_m`
          (bumper to bumper) behind the ego, cruising `dv_mps` faster than the ego (world `rear_cruise`; it does not brake: that is the
          declared world, not an assumption about all rear vehicles).  Left lane: a QUEUE of frozen vehicles `adj0..adjN` alongside the
          ego's start and all the way to the blocker, so no lane change can complete (the adjacent lane is blocked alongside and ahead).
          The label is U_LIBRARY relative to the FINITE policy class RC09_CLASS_ID only (RC09_MEMBERS lists every member with its
          parameters).  It is not a claim about every conceivable controller.
    RC13  over-approximated prediction.  `EnvelopePredictor` is a documented, simple function over DELIVERED observations: constant-velocity
          prediction of each observed object, its box inflated by (lat_m, lon_m) plus growth_mps per second of look-ahead, tested against
          the nominal swept footprint of one action.  It is NOT part of the evidence pipeline (it never produces a label or a margin);
          it is compared AGAINST the evidence (the margins of the coherent worlds) for the same actions.
"""
import math
from dataclasses import asdict, dataclass

from sim import policies as P
from sim.trackers import quintic
from sim.types import EgoCommand

from cases import two_lane as TL
from cases import occlusion as OC

LANE_W, EGO_HL, EGO_HW = TL.LANE_W, TL.EGO_HL, TL.EGO_HW
BLK_HL = TL.BLK_HL
A_LAT_MAX = 8.0                       # m/s^2, same declared authority as the other cases
RC09_CLASS_ID = "rc09_noescape_v1"
RC09_WORLD = {"name": "rear_cruise", "actors": {"rear": {"mode": "cruise"}}}
CAR_LEN, QUEUE_PITCH = 2 * TL.BLK_HL, 5.5          # frozen-queue vehicle length, centre-to-centre spacing (0.9 m bumper gaps)


# ----------------------------------------------------------------------------- RC09: policies
class Accelerate(P.Policy):
    """Forward progress at throttle (EgoCommand 'accel'): the 'full throttle' member of the class."""
    name, terminal = "accelerate", "lane_follow"

    def decide(self, history, t):
        return EgoCommand(long="accel")


class BrakeThenKeep(P.Policy):
    """Brake at `frac` of full brake for `dur_s`, then resume the speed held at that moment (forward progress after a slowdown)."""
    terminal = "lane_follow"

    def __init__(self, frac=0.5, dur_s=1.0, **kw):
        super().__init__(frac=frac, dur_s=dur_s, **kw)
        self.name = f"brake_{frac:g}_for_{dur_s:g}s_then_keep"

    def decide(self, history, t):
        if self._since(t) < self.params["dur_s"]:
            return EgoCommand(long="brake", brake_frac=self.params["frac"])
        return EgoCommand(long="keep")


def _named(factory, name):
    def mk():
        p = factory()
        p.name = name
        return p
    return mk


RC09_BRAKE_FRACS = tuple(round(0.1 * i, 1) for i in range(11))          # 0.0 (coast) .. 1.0 (full brake), constant until the end
RC09_STAGED = ((0.3, 1.0), (0.3, 2.0), (0.6, 1.0), (0.6, 2.0))         # (brake_frac, duration s) then keep
RC09_LCS = (1.0, 2.0)                                                   # lane-change durations (s); 1.0 is the emergency one
RC09_STEER_BRAKES = (0.5, 1.0)


def rc09_library():
    """The FINITE class.  Order is the record's policy order."""
    lib = [_named(lambda: P.LaneFollow(), "keep_speed"), lambda: Accelerate()]
    lib += [_named((lambda f=f: P.BrakeHold(brake_frac=f)), f"brake_frac_{f:.1f}") for f in RC09_BRAKE_FRACS]
    lib += [(lambda f=f, d=d: BrakeThenKeep(f, d)) for f, d in RC09_STAGED]
    lib += [_named((lambda lc=lc: P.Bypass(side=-1, lc_time=lc)), f"bypass_left_lc{lc:g}") for lc in RC09_LCS]
    lib += [_named(lambda: P.Bypass(side=+1, lc_time=2.0), "bypass_right_lc2")]
    lib += [_named((lambda lc=lc, b=b: P.BrakeSteer(side=-1, lc_time=lc, brake_frac=b)), f"brake_steer_left_lc{lc:g}_b{b:g}")
            for lc in RC09_LCS for b in RC09_STEER_BRAKES]
    lib += [_named(lambda: P.StagedBrakeChange(p=0.4, lc_time=1.0), "staged_brake_0.4_change_lc1")]
    return lib


def rc09_members():
    """[{id, name, params, terminal}] of the class, from the policies themselves (nothing hand-copied)."""
    out = []
    for i, mk in enumerate(rc09_library()):
        d = mk().descriptor()
        out.append({"index": i, "name": d["name"], "params": d["params"], "terminal": d["terminal"], "class": type(mk()).__name__})
    return out


# ----------------------------------------------------------------------------- RC09: scene
def queue_positions(gap_m):
    """Rear-face x of the frozen vehicles filling the left lane from alongside the ego's start (x = -2 EGO_HL) to the blocker, ending
    with one exactly beside the blocker (no overlaps between neighbours)."""
    pos, x = [], -2 * EGO_HL
    while x + CAR_LEN <= gap_m - 0.5:
        pos.append(x)
        x += QUEUE_PITCH
    return pos + [gap_m]


def rc09_spec(theta, cfg, rear=True, left_queue=True):
    """theta = {gap_m, v_mps, tau_s, rear_gap_m (bumper-to-bumper), rear_dv_mps}."""
    acts = {"blk": TL._blk(theta["gap_m"])}
    if left_queue:
        for k, x in enumerate(queue_positions(theta["gap_m"])):
            acts[f"adj{k}"] = TL._blk(x, -LANE_W)
    if rear:
        acts["rear"] = {"x": -2 * EGO_HL - theta["rear_gap_m"] - TL.BLK_HL, "lat": 0.0, "v": theta["v_mps"] + theta["rear_dv_mps"],
                        "hl": TL.BLK_HL, "hw": TL.BLK_HW}
    horizon = theta.get("tau_s", 0.25) + 12.0
    sp = TL._base(theta, horizon, acts, hold_s=cfg.hold_s, post_s=cfg.post_maneuver_s)
    sp["a_lat_max"] = A_LAT_MAX
    return sp


def rc09_world(rear=True):
    return RC09_WORLD if rear else {"name": "no_rear", "actors": {}}


def rc09_run(plant_factory, theta, cfg, backend, scope, rear=True, left_queue=True, library=None, **kw):
    from sim.harness import evaluate
    fam = "negatives_RC09"
    return evaluate(plant_factory, rc09_spec(theta, cfg, rear, left_queue), library or rc09_library(), [rc09_world(rear)], theta, "RC09",
                    fam, fam, backend, scope, cfg=cfg, **kw)


def counterexamples(rec):
    """For every policy of an RC09 record: {policy: {world, G, refuted, termination, collided_with, collision_tick}} from the record's
    evidence (R_raw, events) only.  `refuted` = a resolved world with G < eps."""
    eps = rec["margin_config"].get("eps", 0.0) if isinstance(rec, dict) else rec.margin_config.get("eps", 0.0)
    get = (lambda k: rec[k]) if isinstance(rec, dict) else (lambda k: getattr(rec, k))
    out = {}
    for pi, p in enumerate(get("policies")):
        for wi, w in enumerate(get("worlds")):
            g = get("R_raw")[pi][wi]
            g = -math.inf if g == "-inf" else (None if g is None else float(g))
            ev = get("events")[f"{p}/{w}"]
            c = [e for e in ev["events"] if e.get("type") == "collision"]
            out[p] = {"world": w, "G": g, "refuted": g is not None and g < eps, "termination": ev["termination"],
                      "collided_with": c[0]["other"] if c else None, "collision_tick": c[0]["k"] if c else None}
    return out


def decel_between(vs, dt, hi=0.9, lo=0.3):
    """Mean deceleration (m/s^2) of a speed series between the first crossing below hi*v0 and the first crossing below lo*v0 (or the last
    tick if lo is never reached).  Returns (decel, t_hi, t_lo, v_lo_reached) or None when the series never drops below hi*v0."""
    v0 = vs[0]
    i_hi = next((i for i, v in enumerate(vs) if v <= hi * v0), None)
    if i_hi is None:
        return None
    i_lo = next((i for i, v in enumerate(vs) if v <= lo * v0), len(vs) - 1)
    if i_lo <= i_hi:
        return None
    return ((vs[i_hi] - vs[i_lo]) / ((i_lo - i_hi) * dt), i_hi * dt, i_lo * dt, vs[i_lo])


# ----------------------------------------------------------------------------- RC13: predictor
@dataclass(frozen=True)
class Envelope:
    """Parameters of the inflated-envelope predictor.

    lat_m, lon_m   constant inflation of each observed object's half-width / half-length (m)
    growth_mps     extra inflation per second of look-ahead, both axes (m/s): the usual 'uncertainty grows with time' term
    horizon_s      look-ahead (s);   dt_s  time resolution of the check (s)
    clearance_m    required box separation between the ego footprint and an inflated envelope (m)
    """
    lat_m: float = 0.0
    lon_m: float = 0.0
    growth_mps: float = 0.0
    horizon_s: float = 4.0
    dt_s: float = 0.1
    clearance_m: float = 0.0


# nominal ego motion of an action (declared dynamics: the calibrated plant values of Task 3 / 7)
B_NOM, TAU_NOM, V_FLOOR, LC_TIME = 6.211, 0.30, 8.0, OC.LC_TIME


def nominal_ego(action, v0, t, b=B_NOM, tau=TAU_NOM, v_floor=V_FLOOR, lc_time=LC_TIME, lane_w=LANE_W):
    """(x, lat) of the ego relative to its pose at the decision time, t seconds ahead.  Both actions keep v0 until the command takes
    effect (tau), then brake at b.  change_left: brake only while v > v_floor (then hold that speed) and move laterally -lane_w with a
    quintic of duration lc_time starting at tau.  stay: brake to a stop in the lane (no lateral motion)."""
    t1 = min(t, tau)
    x = v0 * t1
    tb = max(t - tau, 0.0)
    if action == "stay":
        tstop = v0 / b
        x += v0 * min(tb, tstop) - 0.5 * b * min(tb, tstop) ** 2
        return x, 0.0
    if action == "change_left":
        tfl = max(v0 - v_floor, 0.0) / b
        tb1 = min(tb, tfl)
        x += v0 * tb1 - 0.5 * b * tb1 ** 2
        v1 = min(v0, v_floor)
        x += v1 * max(tb - tfl, 0.0)
        return x, -lane_w * quintic(tb / lc_time)
    raise ValueError(action)


def action_rejected(ob, action, env: Envelope, ego_hw=EGO_HW):
    """Reject the action if, at any look-ahead step, the ego's nominal footprint comes closer than env.clearance_m to the inflated
    constant-velocity box of any object of the delivered observation `ob` (box separation: negative = overlap).  Returns
    (rejected, min_separation_m, t_of_min_s).  Uses only the observation's own fields (dx, dlat, dv, hl, hw, v, ego_hl)."""
    best, t_best = math.inf, None
    n = int(round(env.horizon_s / env.dt_s))
    for k in range(n + 1):
        t = k * env.dt_s
        xe, le = nominal_ego(action, ob.v, t)
        infl = env.growth_mps * t
        for o in ob.objs:
            xo = o["dx"] + (ob.v + o["dv"]) * t                      # object x in the ego's t0 frame (constant velocity, no lateral motion)
            sep_x = abs(xo - xe) - (ob.ego_hl + o["hl"] + env.lon_m + infl)
            sep_y = abs(o["dlat"] - le) - (ego_hw + o["hw"] + env.lat_m + infl)
            sep = max(sep_x, sep_y)                                     # rectangles overlap iff both separations are negative
            if sep < best:
                best, t_best = sep, t
    return best < env.clearance_m, best, t_best


def predictor_threshold(ob, action, grid, **kw):
    """Smallest uniform inflation m in `grid` (lat_m = lon_m = m, no growth) at which `action` is rejected; None if it never is."""
    for m in grid:
        if action_rejected(ob, action, Envelope(lat_m=m, lon_m=m, **kw))[0]:
            return m
    return None


class Recording(P.Policy):
    """Transparent wrapper: runs `inner` and keeps the first delivered observation that contains an object (the predictor's input).
    Adds no behaviour: decide/reset/name/terminal/descriptor are the inner policy's."""

    def __init__(self, inner):
        super().__init__()
        self.inner, self.name, self.terminal = inner, inner.name, inner.terminal
        self.first_obs = None

    def descriptor(self):
        return self.inner.descriptor()

    def reset(self):
        super().reset()
        self.inner.reset()
        self.first_obs = None

    def decide(self, history, t):
        if self.first_obs is None and history and history[-1].objs:
            self.first_obs = history[-1]
        return self.inner.decide(history, t)


def envelope_dict(env):
    return asdict(env)
