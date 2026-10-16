"""
core.py - backend-agnostic pieces of Approach 1 (simulation mining of unrecoverable scenes).

A *backend* (CARLA or the numpy mock) only has to implement
    rollout(theta, world, policy) -> trace
and everything else (policies, margin G, oracle V, categories, QD measures) is shared,
so results from the cheap mock and from CARLA are labelled by exactly the same code.

Oracle (proposal Sec. 4):
    r_j(theta) = min_{omega in WORLDS} G(pi_j, omega; theta)
    V(theta)   = max_j r_j(theta)
    unrecoverable  <=>  V < EPS      (relative to THIS library, worlds, horizon - never "physically impossible")
"""
from dataclasses import dataclass

import numpy as np

from contract import classify

# ------------------------------------------------------------------ config
POLICY_HZ, HORIZON_S, DT = 5, 8.0, 0.05
LANE_W = 3.5
EPS, EDGE_BAND = 0.0, 0.30          # V<EPS unrecoverable; EPS<=V<EPS+EDGE_BAND "edge"


@dataclass(frozen=True)
class MarginConfig:
    """Every constant of the margin G in one place (change G here; Andy/Victor own the final values)."""
    lat_clear: float = 0.5          # m   lateral gap above which an actor in another lane is ignored (margin 1.0)
    headway_s: float = 1.0          # s   time headway charged on the closing speed: gap - closing*headway
    margin_scale: float = 5.0       # m   normalisation of the longitudinal / lateral margin to [-1, 1]
    collision_speed_scale: float = 30.0   # m/s  severity: -(0.1 + impact_speed/scale), clipped to [-1, -0.1]
    offroad_speed_scale: float = 30.0     # m/s  severity: -(0.1 + ego_speed/scale), clipped to [-1, -0.1]
    fail_floor: float = 0.1         # -    smallest failure magnitude (any collision/off-road is at most -fail_floor)


DEFAULT_CFG = MarginConfig()
MARGIN_SCALE, HEADWAY_S, LAT_CLEAR = DEFAULT_CFG.margin_scale, DEFAULT_CFG.headway_s, DEFAULT_CFG.lat_clear  # legacy aliases
T_LANE_CHANGE, A_ACC = 3.0, 3.0

# Search space theta (units: m, m/s, s, m/s^2).  Algorithms work in [0,1]^d.
SPACE = {
    "obstacle_gap":    (25.0, 110.0),    # stationary obstacle ahead in ego lane (centre-to-centre)
    "adjacent_offset": (-45.0, 15.0),   # left-lane car relative to ego (negative = behind)
    "adjacent_speed":  (15.0, 32.0),
    "ego_speed":       (12.0, 30.0),
    "reaction_delay":  (0.0, 1.4),      # ego holds speed this long before any recovery starts
    "brake_limit":     (5.0, 9.0),      # max achievable decel (wet .. dry road)
}
# Scenario families.  "occluded" adds t_reveal: the left-lane car is hidden (by an occluder) from every
# policy's observation until t_reveal seconds; the label contract (G, Pi, Omega, V) is unchanged.
FAMILIES = {
    "two_lane": dict(SPACE),
    "occluded": {**SPACE, "t_reveal": (0.0, 4.0)},     # s; 0 = visible from the start
}


def use_family(name):
    """Select the scenario family; rebinds SPACE/NAMES/LO/HI/D (modules must read them as core.<name>)."""
    global SPACE, NAMES, LO, HI, D, FAMILY
    FAMILY = name
    SPACE = dict(FAMILIES[name])
    NAMES = list(SPACE)
    LO = np.array([SPACE[k][0] for k in NAMES])
    HI = np.array([SPACE[k][1] for k in NAMES])
    D = len(NAMES)


use_family("two_lane")


def decode(u):
    th = {k: float(v) for k, v in zip(NAMES, LO + np.clip(u, 0, 1) * (HI - LO))}
    th["reaction_delay"] = round(th["reaction_delay"] * POLICY_HZ) / POLICY_HZ
    if "t_reveal" in th:
        th["t_reveal"] = round(th["t_reveal"] * POLICY_HZ) / POLICY_HZ
    return th


def measures(th):
    """Behaviour descriptors for quality-diversity: the time budget of each escape route.
    m1: time until the ego reaches the obstacle at current speed (brake route).
    m2: time until the left-lane car catches the ego (lane-change route); 0 if it is alongside/ahead."""
    m1 = th["obstacle_gap"] / th["ego_speed"]
    off, closing = th["adjacent_offset"], th["adjacent_speed"] - th["ego_speed"]
    if off >= 0:
        m2 = 0.0
    elif closing <= 0.1:
        m2 = 10.0
    else:
        m2 = min(-off / closing, 10.0)
    return float(np.clip(m1, 0, 8)), float(m2)


# ------------------------------------------------------------------ worlds Omega
# Hidden mechanisms of the left-lane car that look identical in the initial observation.
WORLDS = ("constant", "idm_reactive", "accelerating")

# ------------------------------------------------------------------ causal policies Pi
KEEP, BRAKE, ACCEL, LEFT = "KEEP", "BRAKE", "ACCEL", "LANE_LEFT"


def left_gap_ok(obs, min_front=12.0, min_rear=10.0, ttc_rear=2.5):
    """Observed-gap predicate for a left lane change (reads observation rows only)."""
    w = obs["lane_w"]
    for o in obs["objs"]:
        if abs(o["dlat"] + w) > w / 2:       # left lane has negative lateral coordinate
            continue
        if 0 <= o["dx"] < min_front:
            return False
        if o["dx"] < 0 and (-o["dx"] < min_rear or (o["dv"] > 0 and -o["dx"] / o["dv"] < ttc_rear)):
            return False
    return True


def ttc_ahead(obs):
    best = np.inf
    for o in obs["objs"]:
        if abs(o["dlat"]) < obs["lane_w"] / 2 and o["dx"] > 0 and o["dv"] < 0:
            best = min(best, (o["dx"] - 4.8) / -o["dv"])
    return best


class Policy:
    name = "base"
    def reset(self): self.t, self.changed = 0, False
    def act(self, obs):
        a = self._act(obs); self.t += 1; return a


class Keep(Policy):
    name = "keep"
    def _act(self, obs): return KEEP


class EmergencyBrake(Policy):
    name = "emergency_brake"
    def _act(self, obs): return BRAKE


class LeftNow(Policy):
    name = "left_now"
    def _act(self, obs): return LEFT if self.t == 0 else KEEP


class LeftIfGapElseBrake(Policy):
    name = "left_if_gap_else_brake"
    def _act(self, obs):
        if self.changed: return KEEP
        if left_gap_ok(obs): self.changed = True; return LEFT
        return BRAKE


class BrakeThenLeft(Policy):
    name = "brake_then_left_if_gap"
    def _act(self, obs):
        if self.changed: return KEEP
        if self.t >= 3 and left_gap_ok(obs): self.changed = True; return LEFT
        return BRAKE


class AccelThenLeft(Policy):
    name = "accel_then_left_if_gap"
    def _act(self, obs):
        if self.changed: return KEEP
        if left_gap_ok(obs): self.changed = True; return LEFT
        return ACCEL if self.t < 4 else BRAKE


class TTCBrakePlanner(Policy):
    """Stand-in for the planner under test (later: Autoware). Brakes only, and late."""
    name = "planner_under_test"
    def _act(self, obs): return BRAKE if ttc_ahead(obs) < 2.0 else KEEP


LIBRARY = [Keep, EmergencyBrake, LeftNow, LeftIfGapElseBrake, BrakeThenLeft, AccelThenLeft]

# ------------------------------------------------------------------ margin G (privileged state, labels only)
def _fail(score_speed, scale, floor=DEFAULT_CFG.fail_floor):
    return -float(np.clip(floor + score_speed / scale, floor, 1.0))


def actor_margin(e, o, cfg=DEFAULT_CFG):
    """Normalized margin between ego and one actor at one instant; None means box overlap (collision).
    Actors in a clearly different lane contribute 1.0, so a car alongside does not cap the margin."""
    dx, dl = o["x"] - e["x"], o["lat"] - e["lat"]
    long_gap = abs(dx) - (e["hl"] + o["hl"])
    lat_gap = abs(dl) - (e["hw"] + o["hw"])
    if lat_gap < 0 and long_gap < 0:
        return None
    if lat_gap >= cfg.lat_clear:
        return 1.0
    if long_gap < 0:
        return (lat_gap - cfg.lat_clear) / cfg.margin_scale
    closing = (e["v"] - o["v"]) if dx > 0 else (o["v"] - e["v"])
    return (long_gap - max(closing, 0.0) * cfg.headway_s) / cfg.margin_scale


def margin(tr, cfg=DEFAULT_CFG):
    """tr = {'ego': [(x,lat,v,onroad)...], 'others': {name: [(x,lat,v)...]}, 'half': {name: (hl,hw)}}.
    G = min over time and actors; collisions/off-road give a continuous negative severity score."""
    hl, hw = tr["half"]["ego"]
    g = 1.0
    for k, (x, lat, v, on) in enumerate(tr["ego"]):
        if not on:
            return _fail(v, cfg.offroad_speed_scale, cfg.fail_floor)
        e = {"x": x, "lat": lat, "v": v, "hl": hl, "hw": hw}
        for name, states in tr["others"].items():
            ox, ol, ov = states[k]
            ohl, ohw = tr["half"][name]
            m = actor_margin(e, {"x": ox, "lat": ol, "v": ov, "hl": ohl, "hw": ohw}, cfg)
            if m is None:
                return _fail(abs(v - ov), cfg.collision_speed_scale, cfg.fail_floor)
            g = min(g, m)
    return float(np.clip(g, -1.0, 1.0))


# ------------------------------------------------------------------ oracle
def oracle(backend, theta, full=False, cfg=DEFAULT_CFG):
    """V = max_j min_w r_j (exact pruning when full=False).  Keeps the policy x world margin matrix R (NaN = not run).
    full=True  -> every entry is evaluated, and contract.classify() assigns R_REF / U_LIBRARY / U_INFORMATION and the hindsight gap.
    full=False -> only R_REF vs 'not R_REF' can be decided (pruned entries are unresolved, never negative evidence)."""
    best = -np.inf
    R = np.full((len(LIBRARY), len(WORLDS)), np.nan)
    names = [P.name for P in LIBRARY]
    for j, P in enumerate(LIBRARY):
        worst = np.inf
        for k, w in enumerate(WORLDS):
            g = margin(backend.rollout(theta, w, P()), cfg)
            R[j, k] = g
            worst = min(worst, g)
            if not full and worst <= best:
                break
        best = max(best, worst)
    planner = min(margin(backend.rollout(theta, w, TTCBrakePlanner()), cfg) for w in WORLDS)
    cat = ("unrecoverable" if best < EPS else "planner_failure" if planner < EPS
           else "edge" if best < EPS + EDGE_BAND else "safe")
    m1, m2 = measures(theta)
    resolved = ~np.isnan(R).any(axis=1)
    best_j = int(np.argmax(np.where(resolved, np.nanmin(np.where(np.isnan(R), np.inf, R), axis=1), -np.inf)))
    row = {**theta, "V": float(best), "planner_margin": float(planner), "category": cat, "m1": m1, "m2": m2,
           "best_policy": names[best_j]}
    if full:
        c = classify(R, EPS)
        row.update({"evidence": c["evidence"], "V_hind": c["V_hind"], "info_gap": c["info_gap"],
                    "R": {names[j]: [None if np.isnan(x) else float(x) for x in R[j]] for j in range(len(names))},
                    "worlds": list(WORLDS)})
        row.update({f"r_{names[j]}": float(np.nanmin(R[j])) for j in range(len(names))})
    else:
        row["evidence"] = "R_REF" if best >= EPS else "NOT_R_REF(U_LIBRARY_or_U_INFORMATION; run full=True)"
    return row


# ------------------------------------------------------------------ fast mock backend (NOT CARLA)
def _quintic(tau):
    tau = min(max(tau, 0.0), 1.0)
    return 10 * tau**3 - 15 * tau**4 + 6 * tau**5


class MockBackend:
    """Point-mass longitudinal + scripted lane-change model, ~100x faster than CARLA.
    Only for testing/benchmarking the search algorithms; it is not a claim about CARLA dynamics."""
    name = "mock"
    HALF = (2.4, 0.95)

    def rollout(self, th, world, policy):
        n = int(HORIZON_S / DT)
        every = int(round(1 / (POLICY_HZ * DT)))
        delay = int(round(th["reaction_delay"] * POLICY_HZ))
        w = LANE_W
        ego = {"x": 0.0, "lat": 0.0, "v": th["ego_speed"]}
        adj = {"x": th["adjacent_offset"], "lat": -w, "v": th["adjacent_speed"]}
        obst = {"x": th["obstacle_gap"], "lat": 0.0, "v": 0.0}
        tr = {"ego": [], "others": {"adj": [], "obst": []},
              "half": {"ego": self.HALF, "adj": self.HALF, "obst": self.HALF}}
        policy.reset()
        cmd, prev, v_hold, t_lc, lat0, k_dec = KEEP, KEEP, ego["v"], None, 0.0, 0
        for k in range(n):
            t = k * DT
            if k % every == 0:
                visible = (adj, obst) if t >= th.get("t_reveal", 0.0) else (obst,)   # occluded until t_reveal
                obs = {"t": t, "v": ego["v"], "lane_w": w, "objs": [
                    {"dx": o["x"] - ego["x"], "dlat": o["lat"] - ego["lat"], "dv": o["v"] - ego["v"]}
                    for o in visible]}
                cmd = KEEP if k_dec < delay else policy.act(obs)
                k_dec += 1
                if cmd == KEEP and prev != KEEP:
                    v_hold = ego["v"]
                if cmd == LEFT and t_lc is None:
                    t_lc, lat0 = t, ego["lat"]
                prev = cmd
            # ego longitudinal
            a = {BRAKE: -th["brake_limit"], ACCEL: A_ACC}.get(cmd, float(np.clip(2.0 * (v_hold - ego["v"]), -3, 3)))
            ego["v"] = max(ego["v"] + a * DT, 0.0)
            ego["x"] += ego["v"] * DT
            if t_lc is not None:
                ego["lat"] = lat0 - w * _quintic((t - t_lc) / T_LANE_CHANGE)
            # adjacent car mechanism (hidden world)
            if world == "accelerating":
                adj["v"] = min(adj["v"] + 1.5 * DT, 40.0)
            elif world == "idm_reactive":
                lead = None
                for o in (ego,):
                    if o["x"] > adj["x"] and abs(o["lat"] - adj["lat"]) < 0.75 * w:
                        lead = o
                v0 = th["adjacent_speed"]
                gap = (lead["x"] - adj["x"] - 4.8) if lead else 1e3
                dv = (adj["v"] - lead["v"]) if lead else 0.0
                s_star = 2.0 + adj["v"] * 1.2 + adj["v"] * dv / (2 * np.sqrt(1.5 * 2.0))
                acc = 1.5 * (1 - (adj["v"] / v0) ** 4 - (s_star / max(gap, 0.1)) ** 2)
                adj["v"] = max(adj["v"] + float(np.clip(acc, -9, 1.5)) * DT, 0.0)
            adj["x"] += adj["v"] * DT
            on = -1.5 * w <= ego["lat"] <= 0.5 * w
            tr["ego"].append((ego["x"], ego["lat"], ego["v"], on))
            tr["others"]["adj"].append((adj["x"], adj["lat"], adj["v"]))
            tr["others"]["obst"].append((obst["x"], obst["lat"], 0.0))
            # early exit on collision (the margin function will find it)
            if abs(adj["x"] - ego["x"]) < 4.8 and abs(adj["lat"] - ego["lat"]) < 1.9: break
            if abs(obst["x"] - ego["x"]) < 4.8 and abs(obst["lat"] - ego["lat"]) < 1.9: break
        return tr
