"""
carla_backend.py - CARLA backend for core.oracle().

STATUS: written against the CARLA 0.9.16 Python API (all class/field/method names were checked against
the 0.9.16 client wheel) but NOT run against a live server by the author.  Run `python carla_backend.py
--selftest` first; it prints the numbers you need to sanity-check (speed hold, braking, lane change).

Design choices (all for determinism + cheap headless runs):
  * synchronous mode, fixed 0.05 s step, no_rendering_mode (no cameras needed), substepping on
  * NO Traffic Manager / autopilot: every actor (ego and scripted) is driven by our own controllers
  * a *road frame* (x forward, lat positive to the RIGHT = CARLA's +y) anchored at a straight 2+ lane stretch
  * there is no state snapshot in CARLA, so every rollout re-spawns from theta (deterministic) - the
    oracle therefore costs ~len(LIBRARY)*len(WORLDS) episodes per scene; run several servers (--ports).
Start a headless server e.g.:  ./CarlaUE4.sh -RenderOffScreen -nosound -carla-rpc-port=2000
"""
import json
import math
import os
import numpy as np

import core
from core import (ACCEL, BRAKE, DT, HORIZON_S, KEEP, LANE_W, LEFT, POLICY_HZ, T_LANE_CHANGE, _quintic)


def _wrap(deg):
    return (deg + 180.0) % 360.0 - 180.0


class CarlaBackend:
    name = "carla"
    CAL_FILE = "friction_calibration.json"

    def __init__(self, host="localhost", port=2000, town="Town04", timeout=60.0, straight_len=345.0):
        import carla
        self.carla = carla
        self.client = carla.Client(host, port)
        self.client.set_timeout(timeout)
        world = self.client.get_world()
        if town not in world.get_map().name:
            world = self.client.load_world(town)
        self.world = world
        s = world.get_settings()
        s.synchronous_mode, s.fixed_delta_seconds = True, DT
        s.no_rendering_mode = True
        s.substepping, s.max_substep_delta_time, s.max_substeps = True, 0.01, 10
        world.apply_settings(s)
        self.map = world.get_map()
        self.lib = world.get_blueprint_library()
        self._find_road(straight_len)
        self.cal = json.load(open(self.CAL_FILE)) if os.path.exists(self.CAL_FILE) else None
        self._maxsteer = {}

    # ------------------------------------------------------------ road frame
    def _find_road(self, length):
        C = self.carla
        for wp in self.map.generate_waypoints(20.0):
            if wp.lane_type != C.LaneType.Driving or wp.is_junction:
                continue
            left = wp.get_left_lane()
            if left is None or left.lane_type != C.LaneType.Driving or left.lane_id * wp.lane_id <= 0:
                continue
            right = wp.get_right_lane()
            if right is not None and right.lane_type == C.LaneType.Driving and right.lane_id * wp.lane_id > 0:
                continue                                          # want ego in the right-most lane
            ok = True
            for s in range(10, int(length) + 1, 15):
                nx = wp.next(float(s))
                if (not nx or nx[0].is_junction or nx[0].get_left_lane() is None
                        or abs(_wrap(nx[0].transform.rotation.yaw - wp.transform.rotation.yaw)) > 1.0):
                    ok = False
                    break
            if ok:
                self.origin = wp.transform
                yaw = math.radians(wp.transform.rotation.yaw)
                self.f = (math.cos(yaw), math.sin(yaw))
                self.r = (-math.sin(yaw), math.cos(yaw))          # right-hand direction (CARLA y is right)
                self.lane_w = float(wp.lane_width)
                self.road_yaw = wp.transform.rotation.yaw
                self.ego_x0 = 60.0                                  # leaves 60 m behind for the adjacent car
                return
        raise RuntimeError("no straight multi-lane stretch found; try --town Town06")

    def to_road(self, loc):
        dx, dy = loc.x - self.origin.location.x, loc.y - self.origin.location.y
        return dx * self.f[0] + dy * self.f[1], dx * self.r[0] + dy * self.r[1]

    def to_world(self, x, lat, z=0.5):
        o = self.origin.location
        return self.carla.Location(o.x + x * self.f[0] + lat * self.r[0],
                                   o.y + x * self.f[1] + lat * self.r[1], o.z + z)

    # ------------------------------------------------------------ actors
    def _bp(self, name):
        found = self.lib.filter(name)
        if found:
            return found[0]
        return [b for b in self.lib.filter("vehicle.*") if int(b.get_attribute("number_of_wheels")) == 4][0]

    def _spawn(self, bp_name, x, lat, v, role):
        C = self.carla
        tf = C.Transform(self.to_world(x, lat), C.Rotation(yaw=self.road_yaw))
        a = self.world.try_spawn_actor(self._bp(bp_name), tf)
        if a is None:
            raise RuntimeError(f"spawn failed for {role} at x={x:.1f} lat={lat:.1f}")
        a.set_target_velocity(C.Vector3D(self.f[0] * v, self.f[1] * v, 0.0))
        return a

    def _set_friction(self, ego, brake_limit):
        pc = ego.get_physics_control()
        mu = self._friction_for(brake_limit)
        ws = pc.wheels
        for w in ws:
            w.tire_friction = mu
        pc.wheels = ws
        ego.apply_physics_control(pc)

    def _friction_for(self, brake_limit):
        if self.cal:                                                # measured table: decel -> friction
            return float(np.interp(brake_limit, self.cal["decel"], self.cal["friction"]))
        return float(np.clip(brake_limit / 2.5, 0.5, 5.0))          # UNCALIBRATED guess; run --calibrate

    # ------------------------------------------------------------ controllers
    def _lat_cmd(self, a, lat_ref, vlat_ref, v):
        """Stanley-style lane tracker in the road frame. steer>0 turns right, lat>0 is right."""
        tf = a.get_transform()
        _, lat = self.to_road(tf.location)
        psi = math.radians(_wrap(tf.rotation.yaw - self.road_yaw))   # >0: pointing right of the road
        psi_ref = math.atan2(vlat_ref, max(v, 1.0))
        delta = -((psi - psi_ref) + math.atan(1.2 * (lat - lat_ref) / (max(v, 1.0) + 1.0)))
        if a.id not in self._maxsteer:
            self._maxsteer[a.id] = math.radians(a.get_physics_control().wheels[0].max_steer_angle)
        max_steer = self._maxsteer[a.id]
        return float(np.clip(delta / max_steer, -1.0, 1.0))

    @staticmethod
    def _long_cmd(v, v_target):
        e = v_target - v
        return (float(np.clip(0.6 * e, 0, 1)), float(np.clip(-0.6 * e, 0, 1)))

    def _speed(self, a):
        vel = a.get_velocity()
        return vel.x * self.f[0] + vel.y * self.f[1]

    # ------------------------------------------------------------ one episode
    def rollout(self, th, world_mode, policy):
        C = self.carla
        w = self.lane_w
        ex = self.ego_x0
        actors = []
        try:
            ego = self._spawn("vehicle.tesla.model3", ex, 0.0, th["ego_speed"], "ego")
            adj = self._spawn("vehicle.audi.a2", ex + th["adjacent_offset"], -w, th["adjacent_speed"], "adj")
            obs_a = self._spawn("vehicle.mini.cooper", ex + th["obstacle_gap"], 0.0, 0.0, "obst")
            actors = [ego, adj, obs_a]
            self._set_friction(ego, th["brake_limit"])
            half = {n: (a.bounding_box.extent.x, a.bounding_box.extent.y)
                    for n, a in (("ego", ego), ("adj", adj), ("obst", obs_a))}
            tr = {"ego": [], "others": {"adj": [], "obst": []}, "half": half}

            every = int(round(1 / (POLICY_HZ * DT)))
            delay = int(round(th["reaction_delay"] * POLICY_HZ))
            policy.reset()
            cmd, prev, v_hold, t_lc, lat0, k_dec = KEEP, KEEP, th["ego_speed"], None, 0.0, 0
            adj_v_ref = th["adjacent_speed"]
            for _ in range(3):                                      # settle suspension
                ego.apply_control(C.VehicleControl(throttle=0.3)); adj.apply_control(C.VehicleControl(throttle=0.3))
                obs_a.apply_control(C.VehicleControl(hand_brake=True, brake=1.0)); self.world.tick()

            for k in range(int(HORIZON_S / DT)):
                t = k * DT
                st = {n: (*self.to_road(a.get_transform().location), self._speed(a))
                      for n, a in (("ego", ego), ("adj", adj), ("obst", obs_a))}
                if k % every == 0:
                    seen = ("adj", "obst") if t >= th.get("t_reveal", 0.0) else ("obst",)   # occluded family (UNTESTED on CARLA)
                    ob = {"t": t, "v": st["ego"][2], "lane_w": w, "objs": [
                        {"dx": st[n][0] - st["ego"][0], "dlat": st[n][1] - st["ego"][1], "dv": st[n][2] - st["ego"][2]}
                        for n in seen]}
                    cmd = KEEP if k_dec < delay else policy.act(ob)
                    k_dec += 1
                    if cmd == KEEP and prev != KEEP:
                        v_hold = st["ego"][2]
                    if cmd == LEFT and t_lc is None:
                        t_lc, lat0 = t, st["ego"][1]
                    prev = cmd

                # ---- ego
                ev = st["ego"][2]
                if t_lc is None:
                    lat_ref, vlat = lat0, 0.0
                else:
                    tau = (t - t_lc) / T_LANE_CHANGE
                    lat_ref = lat0 - w * _quintic(tau)
                    vlat = -w * (_quintic(tau + 1e-3) - _quintic(tau)) / (1e-3 * T_LANE_CHANGE) if 0 <= tau < 1 else 0.0
                steer = self._lat_cmd(ego, lat_ref, vlat, ev)
                if cmd == BRAKE:
                    thr, brk = 0.0, 1.0
                elif cmd == ACCEL:
                    thr, brk = 0.7, 0.0
                else:
                    thr, brk = self._long_cmd(ev, v_hold)
                ego.apply_control(C.VehicleControl(throttle=thr, brake=brk, steer=steer))

                # ---- hidden-world adjacent car (scripted longitudinal profile, same lane tracker)
                ax, al, av = st["adj"]
                if world_mode == "accelerating":
                    adj_v_ref = min(adj_v_ref + 1.5 * DT, 40.0)
                elif world_mode == "idm_reactive":
                    ex_, el_, ev_ = st["ego"]
                    gap = (ex_ - ax - 4.8) if (ex_ > ax and abs(el_ - al) < 0.75 * w) else 1e3
                    dv = av - ev_ if gap < 1e3 else 0.0
                    s_star = 2.0 + av * 1.2 + av * dv / (2 * math.sqrt(3.0))
                    acc = 1.5 * (1 - (av / th["adjacent_speed"]) ** 4 - (s_star / max(gap, 0.1)) ** 2)
                    adj_v_ref = max(av + float(np.clip(acc, -9, 1.5)) * DT * 4, 0.0)
                a_thr, a_brk = self._long_cmd(av, adj_v_ref)
                adj.apply_control(C.VehicleControl(throttle=a_thr, brake=a_brk, steer=self._lat_cmd(adj, -w, 0.0, av)))
                obs_a.apply_control(C.VehicleControl(hand_brake=True, brake=1.0))

                self.world.tick()
                st = {n: (*self.to_road(a.get_transform().location), self._speed(a))
                      for n, a in (("ego", ego), ("adj", adj), ("obst", obs_a))}
                on = self.map.get_waypoint(ego.get_transform().location, project_to_road=False,
                                           lane_type=C.LaneType.Driving) is not None
                tr["ego"].append((st["ego"][0], st["ego"][1], st["ego"][2], on))
                tr["others"]["adj"].append(st["adj"]); tr["others"]["obst"].append(st["obst"])
                # stop early only on a real box overlap (a worse score cannot happen after impact)
                ex_, el_, _ = st["ego"]
                if any(abs(st[n][0] - ex_) < half["ego"][0] + half[n][0] and
                       abs(st[n][1] - el_) < half["ego"][1] + half[n][1] for n in ("adj", "obst")):
                    break
            return tr
        finally:
            self.client.apply_batch([C.command.DestroyActor(a) for a in actors])
            self.world.tick()

    # ------------------------------------------------------------ calibration / selftest
    def calibrate_friction(self, frictions=(1.0, 2.0, 3.5, 5.0), v0=20.0):
        """Measure achieved hard-brake decel for several tire frictions; saves friction_calibration.json."""
        C = self.carla
        out = {"friction": [], "decel": []}
        for mu in frictions:
            ego = self._spawn("vehicle.tesla.model3", self.ego_x0, 0.0, v0, "ego")
            try:
                pc = ego.get_physics_control(); ws = pc.wheels
                for wl in ws: wl.tire_friction = mu
                pc.wheels = ws; ego.apply_physics_control(pc)
                for _ in range(3): ego.apply_control(C.VehicleControl(throttle=0.3)); self.world.tick()
                v_start, t = self._speed(ego), 0
                while self._speed(ego) > 2.0 and t < 400:
                    ego.apply_control(C.VehicleControl(brake=1.0)); self.world.tick(); t += 1
                out["friction"].append(mu); out["decel"].append((v_start - self._speed(ego)) / (t * DT))
            finally:
                ego.destroy(); self.world.tick()
        order = np.argsort(out["decel"])
        out = {k: [out[k][i] for i in order] for k in out}
        json.dump(out, open(self.CAL_FILE, "w"), indent=2)
        self.cal = out
        return out

    def selftest(self):
        th = core.decode(np.full(core.D, 0.5))
        print("road frame: lane_w=%.2f m, yaw=%.1f deg" % (self.lane_w, self.road_yaw))
        print("calibration:", self.cal or "none (run --calibrate)")
        for P in (core.Keep, core.EmergencyBrake, core.LeftNow):
            tr = self.rollout(dict(th, reaction_delay=0.0), "constant", P())
            e = tr["ego"]
            print(f"{P.name:18s} steps={len(e):3d} v0={e[0][2]:5.1f} v_end={e[-1][2]:5.1f} "
                  f"lat_end={e[-1][1]:5.2f} (expect ~{-self.lane_w:.1f} for left_now) margin={core.margin(tr):+.2f}")
        row = core.oracle(self, th)
        print("oracle:", {k: row[k] for k in ("V", "category", "best_policy")})


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--town", default="Town04")
    ap.add_argument("--calibrate", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    b = CarlaBackend(port=a.port, town=a.town)
    if a.calibrate:
        print(b.calibrate_friction())
    if a.selftest or not a.calibrate:
        b.selftest()
