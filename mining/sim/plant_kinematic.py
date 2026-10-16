"""
plant_kinematic.py - a deterministic kinematic-bicycle PLANT with the same interface as the CARLA plant.

STATUS: MOCK.  Results from this plant are never CARLA results; they exist so that the harness, trackers, delay queues,
policies, cases and labels can be unit-tested now.  Its constants (b_max, brake lag, steering limits) are declared inputs,
not measurements of any CARLA vehicle.

Interface (shared with plant_carla.CarlaPlant):
    build(spec) -> {name: ActorState}          spawn + (instant) warm start; spec = {"actors": {name: {...}}, "road": (lo, hi)}
    step(controls) -> Snapshot                 controls: {name: (throttle, brake, steer)} for every actor that is driven
    close()
Actors without controls hold their state (static blockers).
"""
import math

import numpy as np

from .types import ActorState, DT, Snapshot


class KinematicPlant:
    name = "kinematic_mock"

    def __init__(self, b_max=6.0, a_max=3.0, brake_lag_s=0.0, wheelbase=2.9, max_steer=0.55, steer_rate=1.2,
                 drag=0.0, dt=DT):
        self.b_max, self.a_max, self.brake_lag, self.L = b_max, a_max, brake_lag_s, wheelbase
        self.max_steer, self.steer_rate, self.drag, self.dt = max_steer, steer_rate, drag, dt
        self.k = 0
        self.s = {}
        self.static = set()

    def profile(self):
        return {"plant": self.name, "b_max": self.b_max, "a_max": self.a_max, "brake_lag_s": self.brake_lag,
                "wheelbase": self.L, "max_steer": self.max_steer, "steer_rate": self.steer_rate, "dt": self.dt}

    def build(self, spec):
        self.k, self.s, self.static = 0, {}, set()
        for name, a in spec["actors"].items():
            self.s[name] = {"x": a["x"], "lat": a["lat"], "yaw": a.get("yaw", 0.0), "v": a.get("v", 0.0),
                            "hl": a["hl"], "hw": a["hw"], "steer": 0.0, "brake": 0.0}
            if a.get("static", False):
                self.static.add(name)
        return self._states()

    def _states(self):
        return {n: ActorState(s["x"], s["lat"], s["yaw"], s["v"], s["hl"], s["hw"]) for n, s in self.s.items()}

    def step(self, controls):
        dt = self.dt
        ax_ego = None
        for name, s in self.s.items():
            if name in self.static:
                continue
            thr, brk, steer = controls.get(name, (0.0, 0.0, 0.0))
            # brake pressure first-order lag (0 = instantaneous)
            s["brake"] = brk if self.brake_lag <= 0 else s["brake"] + (brk - s["brake"]) * min(dt / self.brake_lag, 1.0)
            a = thr * self.a_max - s["brake"] * self.b_max - self.drag * s["v"]
            v_new = max(s["v"] + a * dt, 0.0)
            a_eff = (v_new - s["v"]) / dt
            # steering slew limit
            d = float(np.clip(steer, -1, 1)) - s["steer"]
            s["steer"] += float(np.clip(d, -self.steer_rate * dt, self.steer_rate * dt))
            yaw_rate = s["v"] / self.L * math.tan(s["steer"] * self.max_steer)
            s["yaw"] += yaw_rate * dt
            s["x"] += 0.5 * (s["v"] + v_new) * math.cos(s["yaw"]) * dt
            s["lat"] += 0.5 * (s["v"] + v_new) * math.sin(s["yaw"]) * dt
            s["v"] = v_new
            if name == "ego":
                ax_ego = a_eff
        self.k += 1
        return Snapshot(self.k, self.k * dt, self._states(), ax_ego, self._events())

    def _events(self):
        from margin import obb_corners, signed_distance
        ev = []
        e = self.s.get("ego")
        if e is None:
            return ev
        E = obb_corners(e["x"], e["lat"], e["yaw"], e["hl"], e["hw"])
        for n, o in self.s.items():
            if n != "ego" and abs(o["x"] - e["x"]) < e["hl"] + o["hl"] + 1.0 and abs(o["lat"] - e["lat"]) < e["hw"] + o["hw"] + 1.0:
                if signed_distance(E, obb_corners(o["x"], o["lat"], o["yaw"], o["hl"], o["hw"])) < 0:
                    ev.append({"type": "collision", "other": n, "k": self.k})
        return ev

    def close(self):
        pass
