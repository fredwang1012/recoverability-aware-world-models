"""
harness.py - one scenario harness for every plant (PDF 7.4, 9.2; tasks B3-B7).

    run_episode(plant, spec, policy, world)  -> EpisodeResult   (one FRESH matched episode, nothing restored from a save state)
    evaluate(plant_factory, spec, library, worlds, ...) -> ExecutionRecord   (policy x world matrix, contract.classify evidence)

spec (what is built; intent lives elsewhere):
    ego {x, lat, v, hl, hw}                 actors {name: {x, lat, v, hl, hw, static?}}
    road (lat_lo, lat_hi)  lane_w           horizon_s
    delay {obs, plan, act} seconds          obs_hz, policy_hz
    terminal {hold_s, post_maneuver_s}      a_lat_max (dynamic limit, optional)
    a_comb_max (optional): friction-circle limit sqrt(a_long^2 + a_lat^2) <= a_comb_max, from MEASURED accelerations
        (a_long = ego speed difference per tick, smoothed over A_LONG_SMOOTH_TICKS to suppress finite-difference / ABS noise;
        a_lat = |v * yaw_rate| per tick, unsmoothed as for a_lat_max).  A dynamic-limit margin like a_lat_max.
    route_goal {x} (optional): reported SEPARATELY as route_goal_completed (ego front bumper reached x without contact);
        it never enters the margin, the terminal test or the label.
world (a mechanism, chosen once per episode):
    name, actors {actor: {mode, ...}}, reveal {actor: t_reveal}, forget {actor: t_forget}

Policies receive only delivered Obs.  Trackers and the evaluator read ground truth.  Margins come from margin.py.
"""
import math
from dataclasses import dataclass, field

import numpy as np

import margin as M
from records import ExecutionRecord

from .actors import ScriptedActor
from .queues import CommandPipeline, ObservationFeed
from .trackers import EgoController
from .types import DT, Snapshot

A_LONG_SMOOTH_TICKS = 5          # 0.25 s centred moving average of the measured longitudinal acceleration (a_comb_max only)


def smooth(x, n=A_LONG_SMOOTH_TICKS):
    """Centred moving average with edge replication (same length as x)."""
    x = np.asarray(x, float)
    if n <= 1 or len(x) == 0:
        return x
    h = n // 2
    xp = np.concatenate([np.full(h, x[0]), x, np.full(h, x[-1])])
    return np.convolve(xp, np.ones(n) / n, mode="valid")[:len(x)]


@dataclass
class EpisodeResult:
    G: float
    G_search: float
    terminal_ok: bool
    termination: str
    invalid: list
    margin: dict
    events: list
    ego_t: list = field(default_factory=list)
    ego: list = field(default_factory=list)            # (t, x, lat, yaw, v)
    timestamps: dict = field(default_factory=dict)
    geometry: dict = field(default_factory=dict)
    route_goal_completed: object = None                  # None = no route goal declared; else bool (never part of the label)


def _poses(states):
    return (states.x, states.lat, states.yaw)


def terminal_test(mode, ego_hist, speeds, dt, cfg, spec):
    """Boolean terminal continuation, evaluated over the whole post-maneuver interval (never 'speed == 0' alone)."""
    if mode == "protected_hold":
        return M.protected_hold_ok(speeds, dt, cfg)
    if mode == "lane_follow":
        n = max(int(round(cfg.post_maneuver_s / dt)), 1)
        if len(ego_hist) < n:
            return False
        w = spec["lane_w"]
        centres = np.array(spec.get("lane_centres", [0.0, -w]))
        tail = ego_hist[-n:]
        tol = spec.get("lane_tol", 0.5)
        return all(min(abs(lat - centres)) <= tol and abs(yaw) <= 0.05 for (_, lat, yaw) in tail)
    raise ValueError(f"unknown terminal mode {mode}")


def run_episode(plant, spec, policy, world, cfg=M.DEFAULT_V2, seed=0):
    dt = plant.dt
    ego_s = dict(spec["ego"], static=False)
    states = plant.build({"actors": {"ego": ego_s, **spec["actors"]}, "road": spec["road"]})
    invalid = []
    E0 = M.obb_corners(states["ego"].x, states["ego"].lat, states["ego"].yaw, states["ego"].hl, states["ego"].hw)
    for n, a in states.items():
        if n != "ego" and M.signed_distance(E0, M.obb_corners(a.x, a.lat, a.yaw, a.hl, a.hw)) < 0:
            invalid.append(f"spawn_overlap:{n}")

    ki = getattr(plant, "speed_ki", 0.0)                       # >0 only for the drag-limited CARLA plant
    hold_i = getattr(plant, "hold_i", {})                      # throttle feed-forward reached at the end of warm-up
    ctl = EgoController(plant.max_steer, ki_speed=ki, dt=dt, i0=hold_i.get("ego", 0.0))
    ctl.reset(states["ego"])
    scripted = {}
    for n, a in spec["actors"].items():
        if a.get("static", False):
            continue
        scripted[n] = ScriptedActor(n, world.get("actors", {}).get(n, {"mode": "cruise"}), a["lat"], a["v"], plant.max_steer, dt,
                                  ki=ki, i0=hold_i.get(n, 0.0))
    d = spec.get("delay", {})
    feed = ObservationFeed(spec["lane_w"], spec.get("obs_hz", 20.0), d.get("obs", 0.0), world.get("reveal"), world.get("forget"), dt)
    pipe = CommandPipeline(d.get("plan", 0.0), d.get("act", 0.0), dt)
    policy.reset()
    every = max(int(round(1.0 / (spec.get("policy_hz", 20.0) * dt))), 1)

    snap = Snapshot(0, 0.0, states)
    names = ["ego"] + list(spec["actors"])
    rec = {n: [_poses(snap.actors[n])] for n in names}
    speeds, ego_hist, ax_hist = [snap.actors["ego"].v], [_poses(snap.actors["ego"])], []
    history, last_cmd, brake_eff_t, collision = [], None, None, False
    termination = "horizon"
    for k in range(int(round(spec["horizon_s"] / dt))):
        t = k * dt
        feed.capture(snap)
        history += feed.deliver(k)
        if history and k % every == 0:
            cmd = policy.decide(history, t)
            if cmd is not None and cmd != last_cmd:
                pipe.submit(cmd, k)
                last_cmd = cmd
        for c in pipe.due(k):
            ctl.apply(c, snap.actors["ego"], t)
            if c.long == "brake" and brake_eff_t is None:
                brake_eff_t = t
        controls = {"ego": ctl.control(snap.actors["ego"], t)}
        for n, sa in scripted.items():
            c = sa.control(snap.actors[n], snap.actors["ego"], t)
            if c is not None:
                controls[n] = c
        snap = plant.step(controls)
        for n in names:
            rec[n].append(_poses(snap.actors[n]))
        speeds.append(snap.actors["ego"].v)
        ego_hist.append(_poses(snap.actors["ego"]))
        ax_hist.append(snap.ax if snap.ax is not None else 0.0)
        if snap.events:
            collision, termination = True, "collision"
            break

    half = {n: (snap.actors[n].hl, snap.actors[n].hw) for n in names}
    trace = {"dt": dt, "ego": rec["ego"], "others": {n: rec[n] for n in names if n != "ego"}, "half": {**half}}
    yaw = np.array([p[2] for p in ego_hist])
    a_lat = np.abs(np.array(speeds) * np.gradient(yaw, dt)) if len(yaw) > 2 else np.zeros(len(speeds))
    dyn = [("a_lat", a_lat, spec["a_lat_max"])] if "a_lat_max" in spec else []
    ax_s = smooth([0.0] + list(ax_hist))                 # aligned to the state index (ax_hist[i] is measured during step i -> i+1)
    a_comb = np.sqrt(ax_s[:len(a_lat)] ** 2 + a_lat ** 2)
    if "a_comb_max" in spec:
        dyn.append(("a_comb", a_comb, spec["a_comb_max"]))
    dyn = dyn or None
    term_ok = False if collision else terminal_test(policy.terminal, ego_hist, np.array(speeds), dt, cfg, spec)
    out = M.compute(trace, cfg, terminal_ok=term_ok, road=spec["road"], dyn=dyn)
    out["dyn_peaks"] = {"a_lat": float(np.max(a_lat)), "a_long_smoothed": float(np.max(np.abs(ax_s))), "a_comb": float(np.max(a_comb)),
                         "dyn_margin": {n: float(np.min((lim - np.abs(ser)) / lim)) for n, ser, lim in (dyn or [])}}
    goal = spec.get("route_goal")
    route_done = None if goal is None else bool((not collision) and max(p[0] + states["ego"].hl for p in ego_hist) >= goal["x"])

    measured = None
    if brake_eff_t is not None:
        k0 = int(round(brake_eff_t / dt))
        idx = [i for i in range(k0, len(ax_hist)) if ax_hist[i] < -1.0]
        measured = (idx[0] * dt - brake_eff_t) if idx else None
    return EpisodeResult(
        G=out["G"], G_search=out["G_search"], terminal_ok=bool(term_ok), termination=termination, invalid=invalid,
        margin={k: v for k, v in out.items() if k in ("G_raw", "parts", "min_clearance_m", "dyn_peaks")}, events=snap.events,
        route_goal_completed=route_done,
        ego=[(i * dt, *p, v) for i, (p, v) in enumerate(zip(ego_hist, speeds))],
        timestamps={"observations": feed.log[-3:], "commands": pipe.log, "brake_effective_t": brake_eff_t,
                    "measured_response_delay_s": measured, "configured_delay_s": d.get("obs", 0) + d.get("plan", 0) + d.get("act", 0)},
        geometry={"initial": {n: {"x": a.x, "lat": a.lat, "v": a.v} for n, a in states.items()},
                  "initial_min_clearance_m": {n: M.signed_distance(E0, M.obb_corners(a.x, a.lat, a.yaw, a.hl, a.hw))
                                              for n, a in states.items() if n != "ego"}})


def evaluate(plant_factory, spec, library, worlds, theta, case_id, family_id, split_group, backend, evidence_scope,
             cfg=M.DEFAULT_V2, seed=0, physics_profile_id=None, runtime=None, parent_family_id=None, varied_factor=None,
             run_mask=None):
    """PDF 9.2: for each policy, for each compatible world, a fresh matched episode; store witness/counterexample/unresolved
    evidence; classify with contract.classify.  Any invalid episode makes the whole record INVALID_RUN.
    run_mask: optional set of (policy_idx, world_idx) pairs that are actually executed; every other entry stays None
    (unresolved, never 0 / failure) - used for query-budget-limited searches (RC20)."""
    R_raw, R_norm, T, runs, invalid, descr = [], [], [], {}, [], []
    for mk in library:
        p0 = mk()
        descr.append(p0.descriptor())
        rr, rn, tt = [], [], []
        for wi, w in enumerate(worlds):
            if run_mask is not None and (len(R_raw), wi) not in run_mask:
                rr.append(None); rn.append(None); tt.append(None)
                continue
            plant = plant_factory()
            try:
                res = run_episode(plant, spec, mk(), w, cfg, seed)
            finally:
                plant.close()
            if res.invalid:
                invalid += [f"{p0.name}/{w['name']}:{x}" for x in res.invalid]
            rr.append(res.G if math.isfinite(res.G) or res.G == -math.inf else None)
            rn.append(res.G_search)
            tt.append(res.terminal_ok)
            runs[f"{p0.name}/{w['name']}"] = {"termination": res.termination, "events": res.events, "margin": res.margin,
                                              "route_goal_completed": res.route_goal_completed,
                                              "timestamps": res.timestamps, "final": res.ego[-1]}
        R_raw.append(rr); R_norm.append(rn); T.append(tt)
        rec = ExecutionRecord(
        case_id=case_id, family_id=family_id, split_group=split_group, backend=backend, theta=theta,
        policies=[p["name"] for p in descr], worlds=[w["name"] for w in worlds], R_raw=R_raw, R_norm=R_norm, terminal_ok=T,
        margin_config=dict(M.asdict(cfg)), evidence_scope=evidence_scope, seed=seed, policy_descriptors=descr,
        runtime=runtime or {}, physics_profile_id=physics_profile_id,
        geometry={"spec": {k: spec[k] for k in ("ego", "actors", "road", "lane_w", "horizon_s", "delay", "a_lat_max", "a_comb_max", "route_goal") if k in spec}},
        timestamps={k: v["timestamps"] for k, v in runs.items()},
        events={k: {"termination": v["termination"], "events": v["events"]} for k, v in runs.items()},
        traces={k: {"final": v["final"], "margin": v["margin"], **({"route_goal_completed": v["route_goal_completed"]}
                                                               if v["route_goal_completed"] is not None else {})} for k, v in runs.items()},
        invalid_reasons=invalid, parent_family_id=parent_family_id, varied_factor=varied_factor)
    return rec.finalize(cfg.eps)
