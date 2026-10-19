"""
plant_carla.py - CARLA 0.9.16 PLANT with the same interface as KinematicPlant (build / step / close).

STATUS: written against the CARLA 0.9.16 Python API; NOT run against a live server until `results/carla/selftest.txt` says
otherwise.  Nothing from this file may be reported as a CARLA result before B1-B3 (runtime pin, calibration, selftest,
determinism) pass.

Design (PDF 7-8):
  * synchronous mode, fixed 0.05 s step, substepping 0.01 x 10 (fixed_delta <= max_substep_delta * max_substeps), no Traffic Manager
  * road frame: x forward, lat positive RIGHT (= CARLA +y), yaw rad relative to the road, positive = turning right
  * t0 is defined AFTER a warm-up approach from MEASURED oriented-box geometry; static blockers are re-placed relative to the
    measured ego bumper (they are static - this is not teleporting a decisive vehicle along a trajectory)
  * poses are box CENTRES: actor.get_transform().transform(actor.bounding_box.location)
  * contact is checked on oriented boxes every tick AND corroborated by the collision sensor (impulse + other actor logged)
  * every episode is a fresh matched start (CARLA has no state snapshot)
"""
import hashlib
import json
import math
import os

import numpy as np

from .trackers import SpeedPI, hold_lane_control
from .types import ActorState, DT, Snapshot

DEFAULT_BP = {"ego": "vehicle.tesla.model3"}
STATIC_BP, MOVER_BP = "vehicle.audi.a2", "vehicle.mini.cooper"


def _wrap(deg):
    return (deg + 180.0) % 360.0 - 180.0


def wheel_params(profile, decel):
    """(tire_friction, max_brake_torque) realising `decel` (m/s^2) under a calibrated profile.  New profiles (knob ==
    'max_brake_torque') interpolate torque from decel at the fixed friction; targets outside the measured range are clipped
    to it.  Old profiles (no knob) interpolate friction from decel at the single recorded torque."""
    if profile.get("knob") == "max_brake_torque":
        dec = np.maximum.accumulate(np.asarray(profile["decel"], float))
        torque = float(np.interp(decel, dec, np.asarray(profile["torque"], float)))
        return float(profile["friction_fixed"]), torque
    mu = float(np.interp(decel, profile["decel"], profile["friction"]))
    return mu, profile.get("max_brake_torque")


class CarlaPlant:
    name = "carla"

    def __init__(self, client, world, site, decel=None, profile_path="results/carla/physics_profile.json", dt=DT,
                 warm_s=2.0, bp_ego="vehicle.tesla.model3", speed_ki=0.3):
        import carla
        self.C, self.client, self.world, self.site, self.dt = carla, client, world, site, dt
        self.map, self.lib = world.get_map(), world.get_blueprint_library()
        self.warm_s, self.bp_ego = warm_s, bp_ego
        self.speed_ki, self.hold_i = speed_ki, {}
        self.decel_target = decel
        self.profile_data = json.load(open(profile_path)) if os.path.exists(profile_path) else None
        self.actors, self.byid, self.k = {}, {}, 0
        self.sensor, self._collisions, self.static = None, [], set()
        self.max_steer = math.radians(70.0)
        self._prev_v = None
        self.wheels_override = None      # (tire_friction, max_brake_torque) applied BEFORE warm-up (calibration sweeps)

    # ------------------------------------------------------------------ settings (call once per world)
    @staticmethod
    def configure_world(world, dt=DT):
        s = world.get_settings()
        s.synchronous_mode, s.fixed_delta_seconds, s.no_rendering_mode = True, dt, True
        s.substepping, s.max_substep_delta_time, s.max_substeps = True, 0.01, 10
        assert dt <= s.max_substep_delta_time * s.max_substeps + 1e-12
        world.apply_settings(s)

    def profile(self):
        d = {"plant": self.name, "dt": self.dt, "decel_target": self.decel_target, "bp_ego": self.bp_ego}
        if self.profile_data:
            d["physics_profile"] = self.profile_data.get("id")
        return d

    # ------------------------------------------------------------------ frames and boxes
    def _loc(self, x, lat):
        return self.site.to_world(x, lat, self.map)

    def _spawn(self, bp_id, x, lat, v, tag):
        C = self.C
        bps = self.lib.filter(bp_id)
        bp = bps[0] if bps else [b for b in self.lib.filter("vehicle.*") if int(b.get_attribute("number_of_wheels")) == 4][0]
        bp.set_attribute("role_name", tag)
        a = self.world.try_spawn_actor(bp, C.Transform(self._loc(x, lat), C.Rotation(yaw=self.site.road_yaw)))
        if a is None:
            raise RuntimeError(f"spawn failed for {tag} at x={x:.1f} lat={lat:.2f}")
        if v:
            a.set_target_velocity(C.Vector3D(self.site.f[0] * v, self.site.f[1] * v, 0.0))
        return a

    def _state(self, a):
        tf = a.get_transform()
        bbl = a.bounding_box.location                              # local offset of the box centre
        c = tf.transform(self.C.Location(bbl.x, bbl.y, bbl.z))     # copy: Transform.transform mutates its argument (verified on 0.9.16)
        x, lat = self.site.to_road(c)
        yaw = math.radians(_wrap(tf.rotation.yaw - self.site.road_yaw))
        vel = a.get_velocity()
        fwd = tf.get_forward_vector()
        v = vel.x * fwd.x + vel.y * fwd.y
        e = a.bounding_box.extent
        return ActorState(x, lat, yaw, v, e.x, e.y)

    def _states(self):
        return {n: self._state(a) for n, a in self.actors.items()}

    # ------------------------------------------------------------------ physics profile
    def _apply_physics(self, a):
        """Set tire friction and max brake torque.  apply_physics_control RESETS the vehicle dynamics, so this must be called
        right after spawn (before the warm-up approach), never on a moving vehicle.
        wheels_override=(mu, torque) wins (calibration); else decel_target is mapped through the calibrated profile."""
        if self.wheels_override is not None:
            mu, torque = self.wheels_override
            pc = a.get_physics_control()
            ws = pc.wheels
            for w in ws:
                if mu is not None:
                    w.tire_friction = float(mu)
                if torque is not None:
                    w.max_brake_torque = float(torque)
            pc.wheels = ws
            a.apply_physics_control(pc)
            return
        if self.decel_target is None or not self.profile_data:
            return
        mu, torque = wheel_params(self.profile_data, self.decel_target)
        pc = a.get_physics_control()
        ws = pc.wheels
        for w in ws:
            w.tire_friction = mu
            if torque:
                w.max_brake_torque = float(torque)
        pc.wheels = ws
        a.apply_physics_control(pc)

    # ------------------------------------------------------------------ build / step
    def build(self, spec):
        C = self.C
        self.close()
        self.k, self.static, self._prev_v = 0, set(), None
        ego, others = spec["actors"]["ego"], {n: a for n, a in spec["actors"].items() if n != "ego"}
        warm_back = lambda v: v * self.warm_s
        self.actors["ego"] = self._spawn(self.bp_ego, ego["x"] - warm_back(ego["v"]), ego["lat"], ego["v"], "ego")
        self._apply_physics(self.actors["ego"])
        if ego["v"] and (self.wheels_override is not None or self.decel_target is not None):
            # apply_physics_control also zeroes the velocity, and applies it on the NEXT tick (racy if the speed is re-imposed
            # first): tick once to let the control take effect, then re-impose the initial speed
            self.world.tick()
            self.actors["ego"].set_target_velocity(C.Vector3D(self.site.f[0] * ego["v"], self.site.f[1] * ego["v"], 0.0))
        pc = self.actors["ego"].get_physics_control()
        self.max_steer = math.radians(pc.wheels[0].max_steer_angle)
        sens_bp = self.lib.find("sensor.other.collision")
        self.sensor = self.world.spawn_actor(sens_bp, C.Transform(), attach_to=self.actors["ego"])
        self.sensor.listen(lambda ev: self._collisions.append({"frame": ev.frame, "other_id": ev.other_actor.id,
                                                               "impulse": float(math.sqrt(ev.normal_impulse.x ** 2 + ev.normal_impulse.y ** 2 + ev.normal_impulse.z ** 2))}))
        for n, a in others.items():
            if a.get("static"):
                self.static.add(n)
                self.actors[n] = self._spawn(a.get("bp", STATIC_BP), a["x"], a["lat"], 0.0, n)
                if a.get("frozen"):
                    # unstable two-wheelers (RC16 post) tip over and drift under hand-brake; freeze the body in place (opt-in per actor)
                    self.actors[n].set_simulate_physics(False)
            else:
                self.actors[n] = self._spawn(a.get("bp", MOVER_BP), a["x"] - warm_back(a["v"]), a["lat"], a["v"], n)
        self.byid = {a.id: n for n, a in self.actors.items()}
        # warm-up approach with simple cruise + lane hold; static actors braked
        ref = {"ego": (ego["lat"], ego["v"]), **{n: (a["lat"], a.get("v", 0.0)) for n, a in others.items()}}
        pis = {n: SpeedPI(0.6, self.speed_ki, self.dt) for n in self.actors if n not in self.static}
        v_prev = ego["v"]
        for wk in range(int(round(self.warm_s / self.dt))):
            ctl = {}
            for n, a in self.actors.items():
                if n in self.static:
                    a.apply_control(C.VehicleControl(hand_brake=True, brake=1.0))
                    continue
                st = self._state(a)
                if n == "ego":
                    v_prev = st.v
                t, b, s = hold_lane_control(st, ref[n][0], ref[n][1], self.max_steer, pi=pis[n])
                if n != "ego" and ref[n][1] > 0:
                    # Moving non-ego actors: the clutch engagement / engine spin-up drags them far below their declared speed (a Mini at 20 m/s
                    # was 16.8 m/s at t0) and a PI hold cannot recover that in the warm-up.  The ego is re-imposed after its physics control; do
                    # the same here every warm-up tick, AFTER reading the measured speed: the PI therefore integrates the per-tick drag deficit and
                    # learns the hold throttle (hold_i), and the actor starts the episode at its declared speed.
                    self.actors[n].set_target_velocity(C.Vector3D(self.site.f[0] * ref[n][1], self.site.f[1] * ref[n][1], 0.0))
                # the single-gear drivetrain starts in neutral (gear 0): the first throttle would engage the clutch on a stationary
                # engine and drag the car down by ~1/3 of its speed.  Engage gear 1 on the first warm-up tick so the engine spin-up
                # happens (and is absorbed by the cruise controller) before t0.
                a.apply_control(C.VehicleControl(throttle=t, brake=b, steer=s, **({"manual_gear_shift": True, "gear": 1} if wk == 0 else {})))
            self.world.tick()
        self.hold_i = {n: p.i for n, p in pis.items()}           # hand the learned feed-forward throttle to the episode controllers
        # re-place STATIC blockers relative to the MEASURED ego bumper so the declared bumper gap holds exactly
        me = self._state(self.actors["ego"])
        a_ego = (me.v - v_prev) / self.dt
        for n in self.static:
            a = others[n]
            gap = (a["x"] - a["hl"]) - (ego["x"] + ego["hl"])
            sa = self._state(self.actors[n])
            # the ego advances one more tick before t0: constant-acceleration extrapolation from the last two warm-up speeds
            x_new = me.x + me.v * self.dt + 0.5 * a_ego * self.dt ** 2 + me.hl + gap + sa.hl
            loc = self._loc(x_new, a["lat"])
            self.actors[n].set_transform(C.Transform(C.Location(loc.x, loc.y, self.actors[n].get_transform().location.z),
                                                    C.Rotation(yaw=self.site.road_yaw)))
        for n, (_, rv) in ref.items():                          # the same pin before the extra tick, so movers are not left one drag-tick low at t0
            if n != "ego" and n not in self.static and rv > 0:
                self.actors[n].set_target_velocity(C.Vector3D(self.site.f[0] * rv, self.site.f[1] * rv, 0.0))
        self.world.tick()                                       # let the transform take effect, then t0
        self._collisions.clear()
        st = self._states()
        self._prev_v = st["ego"].v
        return st

    def step(self, controls):
        C = self.C
        for n, a in self.actors.items():
            if n in self.static:
                a.apply_control(C.VehicleControl(hand_brake=True, brake=1.0))
            elif n in controls:
                t, b, s = controls[n]
                a.apply_control(C.VehicleControl(throttle=float(t), brake=float(b), steer=float(s)))
        self.world.tick()
        self.k += 1
        st = self._states()
        v = st["ego"].v
        ax = (v - self._prev_v) / self.dt if self._prev_v is not None else None
        self._prev_v = v
        return Snapshot(self.k, self.k * self.dt, st, ax, self._events(st))

    def _events(self, st):
        from margin import obb_corners, signed_distance
        ev, e = [], st["ego"]
        E = obb_corners(e.x, e.lat, e.yaw, e.hl, e.hw)
        for n, o in st.items():
            if n != "ego" and abs(o.x - e.x) < e.hl + o.hl + 1.0 and abs(o.lat - e.lat) < e.hw + o.hw + 1.0:
                if signed_distance(E, obb_corners(o.x, o.lat, o.yaw, o.hl, o.hw)) < 0:
                    ev.append({"type": "collision", "other": n, "k": self.k, "source": "obb"})
        for c in self._collisions:
            if self.byid.get(c["other_id"]):
                ev.append({"type": "collision", "other": self.byid[c["other_id"]], "k": self.k, "source": "sensor",
                           "impulse": c["impulse"]})
        self._collisions.clear()
        return ev

    def close(self):
        ids = [a.id for a in self.actors.values()]
        if self.sensor is not None:
            try:
                self.sensor.stop()
                ids.append(self.sensor.id)
            except Exception:
                pass
        if ids:
            self.client.apply_batch([self.C.command.DestroyActor(i) for i in ids])
            self.world.tick()
        self.actors, self.byid, self.sensor = {}, {}, None


def map_hash(world):
    return hashlib.sha256(world.get_map().to_opendrive().encode()).hexdigest()[:16]


def runtime_info(client, world, site):
    s = world.get_settings()
    return {"server_version": client.get_server_version(), "client_version": client.get_client_version(),
            "map": world.get_map().name, "map_hash": map_hash(world),
            "settings": {"synchronous_mode": s.synchronous_mode, "fixed_delta_seconds": s.fixed_delta_seconds,
                         "substepping": s.substepping, "max_substep_delta_time": s.max_substep_delta_time,
                         "max_substeps": s.max_substeps, "no_rendering_mode": s.no_rendering_mode},
            "site": {"origin_xyz": [site.origin.location.x, site.origin.location.y, site.origin.location.z],
                     "road_yaw": site.road_yaw, "lane_w": site.lane_w, "n_lanes_same_dir": site.n_lanes}}
