"""
bringup.py - CARLA bring-up gate (spec tasks B1-B4).  NOTHING downstream may run until `selftest` and `determinism` pass.

    python -m sim.bringup runtime     --port 2000 [--town Town04]     -> results/carla/runtime.json
    python -m sim.bringup calibrate   --port 2000                      -> results/carla/physics_profile.json, calibration.md
    python -m sim.bringup selftest    --port 2000                      -> results/carla/selftest.txt
    python -m sim.bringup determinism --ports 2000[,2002]              -> results/carla/determinism.md
    python -m sim.bringup timing      --port 2000                      -> results/carla/timing.md

Run from mining/ with the carla312 conda env (Python 3.12, carla==0.9.16) and a server started with e.g.
    CarlaUE4.exe -RenderOffScreen -nosound -quality-level=Low -carla-rpc-port=2000

STATUS: written without a live server; every claim it prints comes from a run, nothing is assumed.
"""
import argparse
import hashlib
import json
import math
import os
import subprocess
import sys
import time

import numpy as np

import margin as M
from cases import corridor as CO
from cases import two_lane as TL
from sim import policies as P
from sim.harness import evaluate, run_episode
from sim.plant_carla import CarlaPlant, runtime_info
from sim.sites import find_straight_site
from sim.trackers import EgoController
from sim.types import DT

OUT = "results/carla"
CFG = M.DEFAULT_V2


XODR = os.path.join(os.path.dirname(__file__), "assets", "straight_2lane.xodr")
CUSTOM = "custom_straight"


def connect(host, port, town):
    """town == 'custom_straight' (default): generate the project's own straight 2-lane OpenDRIVE world ONCE per server
    session (no repeated map loads - repeated load_world crashed the server).  Otherwise load the named CARLA town."""
    import carla
    client = carla.Client(host, port)
    client.set_timeout(120.0)
    world = client.get_world()
    if town == CUSTOM:
        if "straight_2lane" not in world.get_map().to_opendrive():
            client.generate_opendrive_world(open(XODR, encoding="utf-8").read(),
                                            carla.OpendriveGenerationParameters(vertex_distance=2.0, max_road_length=50.0,
                                                                                wall_height=0.0, additional_width=0.6,
                                                                                smooth_junctions=True, enable_mesh_visibility=True))
            world = client.get_world()
        site = find_straight_site(world, length=350.0, behind=250.0)
    else:
        if town not in world.get_map().name:
            world = client.load_world(town)
        site = find_straight_site(world)
    CarlaPlant.configure_world(world)
    world.tick()
    return client, world, site


def make_plant(client, world, site, decel=None):
    return CarlaPlant(client, world, site, decel=decel)


def _write(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    open(path, "w", encoding="utf-8").write(text)


def _gpu():
    try:
        return subprocess.check_output(["nvidia-smi", "--query-gpu=name,memory.total,driver_version", "--format=csv,noheader"],
                                       text=True).strip()
    except Exception:
        return "unknown"


# ----------------------------------------------------------------------------- B1
def cmd_runtime(a):
    import carla
    client, world, site = connect(a.host, a.port, a.town)
    info = runtime_info(client, world, site)
    info.update({"python": sys.version.split()[0], "numpy": np.__version__, "carla_client_wheel": getattr(carla, "__version__", "n/a"),
                 "gpu": _gpu(), "blueprints": {"ego": "vehicle.tesla.model3", "static": "vehicle.audi.a2", "mover": "vehicle.mini.cooper"},
                 "physics_profile_id": (json.load(open(f"{OUT}/physics_profile.json"))["id"]
                                        if os.path.exists(f"{OUT}/physics_profile.json") else None),
                 "margin_config": M.asdict(CFG), "dt": DT, "port": a.port})
    _write(f"{OUT}/runtime.json", json.dumps(info, indent=2))
    print(json.dumps(info, indent=2))


# ----------------------------------------------------------------------------- B2
def _one_ego(plant, v0, lat=0.0):
    spec = {"actors": {"ego": {"x": 60.0, "lat": lat, "v": v0, "hl": 2.3, "hw": 0.95}}, "road": TL.ROAD}
    return plant.build(spec)


def set_wheels(plant, friction=None, torque=None):
    pc = plant.actors["ego"].get_physics_control()
    ws = pc.wheels
    for w in ws:
        if friction is not None:
            w.tire_friction = float(friction)
        if torque is not None:
            w.max_brake_torque = float(torque)
    pc.wheels = ws
    plant.actors["ego"].apply_physics_control(pc)


def measure_brake(plant, friction, torque, v0=20.0, steer=0.0):
    """Full brake from v0: decel = -slope of a linear fit of v(t) on 0.9*v0 .. 0.2*v0; also onset delay and residual RMS."""
    plant.wheels_override = (friction, torque)                   # applied right after spawn, before warm-up
    _one_ego(plant, v0)
    t, v, ay = [], [], []
    ctl_k0 = len(t)
    prev_yaw, a_lat = None, []
    for k in range(int(12.0 / DT)):
        snap = plant.step({"ego": (0.0, 1.0, steer)})
        e = snap.actors["ego"]
        t.append(k * DT); v.append(e.v)
        if prev_yaw is not None:
            a_lat.append(abs(e.v * (e.yaw - prev_yaw) / DT))
        prev_yaw = e.yaw
        if e.v < 0.5:
            break
    t, v = np.array(t), np.array(v)
    sel = (v <= 0.9 * v0) & (v >= 0.2 * v0)
    if sel.sum() < 5:
        return None
    slope, icpt = np.polyfit(t[sel], v[sel], 1)
    resid = float(np.sqrt(np.mean((v[sel] - (slope * t[sel] + icpt)) ** 2)))
    onset = next((i for i in range(1, len(v)) if (v[i - 1] - v[i]) / DT > 0.5), None)
    return {"decel": float(-slope), "resid_rms": resid, "onset_delay_s": None if onset is None else onset * DT,
            "a_lat_max": float(max(a_lat)) if a_lat else 0.0}


def measure_steer(plant, v0=12.0, steer=0.3):
    _one_ego(plant, v0)
    yaw_rates, prev = [], None
    for k in range(int(2.5 / DT)):
        snap = plant.step({"ego": (0.15, 0.0, steer)})
        yaw = snap.actors["ego"].yaw
        if prev is not None:
            yaw_rates.append((yaw - prev) / DT)
        prev = yaw
    yr = np.array(yaw_rates)
    ss = float(np.mean(yr[-10:]))
    t90 = next((i * DT for i, x in enumerate(yr) if abs(x) >= 0.9 * abs(ss)), None)
    return {"steer_cmd": steer, "yaw_rate_ss": ss, "t90_s": t90, "yaw_rate_per_steer": ss / steer}


def measure_tracking(plant, v0=12.0, lc_time=3.0):
    st = _one_ego(plant, v0)["ego"]
    ctl = EgoController(plant.max_steer)
    ctl.reset(st)
    from sim.types import EgoCommand
    ctl.apply(EgoCommand(long="keep", lat_target=-TL.LANE_W, lc_time=lc_time), st, 0.0)
    errs = []
    for k in range(int((lc_time + 3.0) / DT)):
        snap_t = k * DT
        e = plant._state(plant.actors["ego"])
        ref, _ = ctl.lat_ref(snap_t)
        errs.append(e.lat - ref)
        plant.step({"ego": ctl.control(e, snap_t)})
    e = plant._state(plant.actors["ego"])
    return {"max_abs_err_m": float(np.max(np.abs(errs))), "rms_err_m": float(np.sqrt(np.mean(np.square(errs)))),
            "final_lat_m": float(e.lat), "target_lat_m": -TL.LANE_W}


def cmd_calibrate(a):
    client, world, site = connect(a.host, a.port, a.town)
    plant = make_plant(client, world, site)
    try:
        _one_ego(plant, 20.0)
        default_torque = plant.actors["ego"].get_physics_control().wheels[0].max_brake_torque
        torques = [default_torque * f for f in (1.0, 2.0)]
        rows = []
        for torque in torques:
            for mu in (0.6, 1.0, 1.5, 2.5, 3.5, 5.0):
                m = measure_brake(plant, mu, torque)
                if m:
                    rows.append({"friction": mu, "max_brake_torque": float(torque), **m})
                    print(rows[-1])
        base = sorted([r for r in rows if abs(r["max_brake_torque"] - default_torque) < 1e-6], key=lambda r: r["decel"])
        dec = np.array([r["decel"] for r in base])
        steer = measure_steer(plant)
        track = measure_tracking(plant)
        combined = measure_brake(plant, base[len(base) // 2]["friction"], default_torque, steer=0.15)
        prof = {"blueprint": "vehicle.tesla.model3", "dt": DT, "default_max_brake_torque": float(default_torque),
                "friction": [r["friction"] for r in base], "decel": [r["decel"] for r in base],
                "max_brake_torque": float(default_torque), "table": rows, "monotone": bool(np.all(np.diff(dec) >= 0)),
                "decel_range": [float(dec.min()), float(dec.max())], "steer_response": steer, "lateral_tracking": track,
                "combined_brake_steer": combined, "response_delay_s": next((r["onset_delay_s"] for r in base if r["onset_delay_s"] is not None), None),
                "covers_5_to_9": bool(dec.min() <= 5.0 and dec.max() >= 9.0)}
        prof["id"] = "pp_" + hashlib.sha256(json.dumps({k: v for k, v in prof.items() if k != "id"}, sort_keys=True).encode()).hexdigest()[:10]
        _write(f"{OUT}/physics_profile.json", json.dumps(prof, indent=2))
        L = [f"## CARLA physics profile {prof['id']} (Tesla Model 3, dt={DT})", "",
             f"decel range {prof['decel_range'][0]:.2f}..{prof['decel_range'][1]:.2f} m/s^2; monotone in friction: {prof['monotone']}; "
             f"covers 5-9 m/s^2: {prof['covers_5_to_9']}", "", "| friction | brake torque | decel | resid RMS (m/s) | onset delay (s) |", "|---|---|---|---|---|"]
        L += [f"| {r['friction']} | {r['max_brake_torque']:.0f} | {r['decel']:.2f} | {r['resid_rms']:.3f} | {r['onset_delay_s']} |" for r in rows]
        L += ["", f"steer response: {steer}", f"lateral tracking (3 s lane change @12 m/s): {track}", f"combined brake+steer: {combined}"]
        _write(f"{OUT}/calibration.md", "\n".join(L) + "\n")
        print("\n".join(L))
    finally:
        plant.close()


# ----------------------------------------------------------------------------- B3
def _clear_spec(v, horizon=8.0):
    th = {"v_mps": v, "tau_s": 0.0}
    return TL.rc19_spec(th, CFG) | {"horizon_s": horizon}


def _ep(plant_factory, spec, policy, world=None):
    plant = plant_factory()
    try:
        return run_episode(plant, spec, policy, world or {"name": "w", "actors": {}}, CFG)
    finally:
        plant.close()


def cmd_selftest(a):
    client, world, site = connect(a.host, a.port, a.town)
    prof = json.load(open(f"{OUT}/physics_profile.json")) if os.path.exists(f"{OUT}/physics_profile.json") else None
    target = float(np.median(prof["decel"])) if prof else None
    mk = lambda: make_plant(client, world, site, decel=target)
    lines, ok_all = [f"CARLA selftest {time.strftime('%Y-%m-%d %H:%M:%S')}  map={world.get_map().name}  lane_w={site.lane_w:.2f}  "
                     f"physics_profile={prof['id'] if prof else 'NONE (run calibrate first)'}"], True

    def check(name, cond, detail):
        nonlocal ok_all
        ok_all &= bool(cond)
        lines.append(f"[{'PASS' if cond else 'FAIL'}] {name}: {detail}")

    r = _ep(mk, _clear_spec(12.0), P.LaneFollow())
    v = [e[4] for e in r.ego]
    check("keep holds speed", abs(v[-1] - v[0]) < 0.5 and max(abs(x - v[0]) for x in v) < 0.5,
          f"v0={v[0]:.2f} v_end={v[-1]:.2f} max|dv|={max(abs(x - v[0]) for x in v):.2f} m/s over {len(v) * DT:.1f} s")
    spec = CO.spec({"gap_m": 60.0, "v_mps": 20.0, "tau_s": 0.0})
    r = _ep(mk, spec, P.BrakeHold(), CO.WORLD)
    t = np.array([e[0] for e in r.ego]); vv = np.array([e[4] for e in r.ego])
    sel = (vv <= 0.9 * 20) & (vv >= 0.2 * 20)
    dec = -np.polyfit(t[sel], vv[sel], 1)[0] if sel.sum() > 5 else float("nan")
    if target:
        check("emergency brake decel within 10% of calibrated target", abs(dec - target) <= 0.10 * target,
              f"measured {dec:.2f} m/s^2, target {target:.2f}")
    else:
        lines.append(f"[INFO] emergency brake decel measured {dec:.2f} m/s^2 (no calibration, no target)")
    r = _ep(mk, _clear_spec(12.0, 9.0), P.Bypass(side=-1, lc_time=3.0))
    lat = r.ego[-1][2]
    check("left lane change ends at -lane_w +/- 0.3 m", abs(lat + site.lane_w) < 0.3, f"lat_end={lat:.2f} m (target {-site.lane_w:.2f})")
    th = {"gap_m": 18.0, "v_mps": 12.0, "tau_s": 0.25}
    r = _ep(mk, CO.spec(th), P.BrakeHold(), CO.WORLD)
    g = r.geometry["initial_min_clearance_m"]
    check("measured bumper gap at t0 equals the declared gap (0.1 m)", all(abs(c - 18.0) < 0.1 for c in g.values()),
          f"measured clearances {{{', '.join(f'{k}: {c:.3f}' for k, c in g.items())}}} m (declared 18.0)")
    check("no spawn overlap / no invalid reasons", not r.invalid, f"invalid={r.invalid}")
    lines.append("\nRESULT: " + ("PASS" if ok_all else "FAIL - diagnose before running any search (settle ticks, spawn z, substepping, controller state)"))
    _write(f"{OUT}/selftest.txt", "\n".join(lines) + "\n")
    print("\n".join(lines))
    sys.exit(0 if ok_all else 1)


# ----------------------------------------------------------------------------- B3 determinism
def _trace(plant_factory, spec, policy, world):
    r = _ep(plant_factory, spec, policy, world)
    return np.array([[e[1], e[2], e[4]] for e in r.ego])


def cmd_determinism(a):
    ports = [int(p) for p in a.ports.split(",")]
    cases = {
        "corridor gap 18 brake": (CO.spec({"gap_m": 18.0, "v_mps": 12.0, "tau_s": 0.25}), lambda: P.BrakeHold(), CO.WORLD),
        "two_lane bypass": (TL.rc03_spec({"gap_m": 40.0, "v_mps": 12.0, "tau_s": 0.25}, CFG), lambda: P.Bypass(side=-1, lc_time=2.0), TL.CLEAN),
    }
    conns = [connect(a.host, p, a.town) for p in ports]
    prof = json.load(open(f"{OUT}/physics_profile.json")) if os.path.exists(f"{OUT}/physics_profile.json") else None
    target = float(np.median(prof["decel"])) if prof else None
    rows, ok = [], True
    for name, (spec, mkpol, world) in cases.items():
        traces = []
        for (client, w, site) in conns:
            for _ in range(2):
                traces.append(_trace(lambda: make_plant(client, w, site, decel=target), spec, mkpol(), world))
        n = min(len(t) for t in traces)
        base = traces[0][:n]
        for i, t in enumerate(traces[1:], 1):
            d = np.abs(t[:n] - base)
            same_srv = i < 2
            rows.append((name, "same server, repeat" if same_srv else f"server {ports[(i) // 2]}", d[:, 0].max(), d[:, 1].max(), d[:, 2].max()))
            ok &= d[:, 0].max() < 0.05 and d[:, 1].max() < 0.05 and d[:, 2].max() < 0.05
    L = ["## CARLA determinism (same case run repeatedly; tolerance 0.05 m / 0.05 m/s)", "",
         "| case | comparison | max dx (m) | max dlat (m) | max dv (m/s) |", "|---|---|---|---|---|"]
    L += [f"| {r[0]} | {r[1]} | {r[2]:.4f} | {r[3]:.4f} | {r[4]:.4f} |" for r in rows]
    L += ["", "RESULT: " + ("PASS" if ok else "FAIL - do not run any search")]
    _write(f"{OUT}/determinism.md", "\n".join(L) + "\n")
    print("\n".join(L))
    sys.exit(0 if ok else 1)


# ----------------------------------------------------------------------------- B4
def cmd_timing(a):
    client, world, site = connect(a.host, a.port, a.town)
    prof = json.load(open(f"{OUT}/physics_profile.json")) if os.path.exists(f"{OUT}/physics_profile.json") else None
    target = float(np.median(prof["decel"])) if prof else None
    mk = lambda: make_plant(client, world, site, decel=target)
    t0 = time.time()
    for _ in range(200):
        world.tick()
    ms_tick = (time.time() - t0) / 200 * 1000
    th = {"gap_m": 18.0, "v_mps": 12.0, "tau_s": 0.25}
    t0 = time.time()
    r = _ep(mk, CO.spec(th), P.BrakeHold(), CO.WORLD)
    s_roll = time.time() - t0
    t0 = time.time()
    rec = CO.run(mk, th, CFG, "carla", "carla_finite_replay")
    s_rec = time.time() - t0
    L = ["## CARLA timing (this machine, headless, no sensors)", "", f"- empty tick: {ms_tick:.1f} ms",
         f"- one rollout (corridor, brake_hold, {len(r.ego)} ticks incl. warm-up/spawn): {s_roll:.2f} s",
         f"- one full record (3 policies x 1 world): {s_rec:.2f} s", f"- audit batch estimate (864 policy/world runs): {864 * s_roll / 60:.1f} min on one server"]
    _write(f"{OUT}/timing.md", "\n".join(L) + "\n")
    print("\n".join(L))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["runtime", "calibrate", "selftest", "determinism", "timing"])
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--ports", default="2000")
    ap.add_argument("--town", default=CUSTOM)
    a = ap.parse_args()
    {"runtime": cmd_runtime, "calibrate": cmd_calibrate, "selftest": cmd_selftest, "determinism": cmd_determinism,
     "timing": cmd_timing}[a.cmd](a)


if __name__ == "__main__":
    main()
