"""Actuator-level controllers shared by every plant (kinematic mock and CARLA).

They turn a high-level EgoCommand into (throttle, brake, steer) using ground-truth state - exactly like a low-level
controller would.  DECISIONS never see ground truth; only these trackers do.
"""
import math

import numpy as np

from .types import ActorState, EgoCommand


def quintic(tau):
    """Minimum-jerk 0 -> 1 transition, tau in [0, 1]."""
    tau = min(max(tau, 0.0), 1.0)
    return 10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5


class SpeedPI:
    """Speed hold: throttle/brake from u = kp*e + i.  The integral term (ki > 0) supplies the feed-forward throttle that a
    drag-limited plant needs to hold speed (a P-only hold droops: CARLA's Model 3 needs ~0.4 throttle at 11 m/s).  ki = 0
    reproduces the old P-only law exactly (kinematic mock plant)."""

    def __init__(self, kp=0.6, ki=0.0, dt=0.05, i0=0.0):
        self.kp, self.ki, self.dt, self.i = kp, ki, dt, i0

    def __call__(self, e):
        self.i = float(np.clip(self.i + self.ki * e * self.dt, 0.0, 1.0))
        u = self.kp * e + self.i
        return float(np.clip(u, 0, 1)), float(np.clip(-u, 0, 1))


class EgoController:
    def __init__(self, max_steer_rad, k_lat=1.2, kp_speed=0.6, accel_throttle=0.7, ki_speed=0.0, dt=0.05, i0=0.0):
        self.max_steer, self.k_lat, self.kp, self.accel_thr = max_steer_rad, k_lat, kp_speed, accel_throttle
        self.pi = SpeedPI(kp_speed, ki_speed, dt, i0)

    def reset(self, st: ActorState):
        self.long, self.brake_frac = "keep", 1.0
        self.v_hold = st.v
        self.lat0 = self.lat_target = st.lat
        self.t_lc, self.lc_time = None, 3.0

    def lat_ref(self, t):
        if self.t_lc is None:
            return self.lat_target, 0.0
        tau = (t - self.t_lc) / self.lc_time
        if tau >= 1.0:
            return self.lat_target, 0.0
        d = self.lat_target - self.lat0
        return self.lat0 + d * quintic(tau), d * (quintic(tau + 1e-3) - quintic(tau)) / (1e-3 * self.lc_time)

    def apply(self, cmd: EgoCommand, st: ActorState, t: float):
        """Make a command effective NOW (after planning + actuation delay)."""
        if cmd.long == "keep" and self.long != "keep":
            self.v_hold = st.v
            self.pi.i = 0.0
        self.long, self.brake_frac = cmd.long, cmd.brake_frac
        if cmd.lat_target is not None and abs(cmd.lat_target - self.lat_target) > 1e-9:
            self.lat0, self.lat_target, self.t_lc, self.lc_time = st.lat, cmd.lat_target, t, cmd.lc_time

    def control(self, st: ActorState, t: float):
        ref, vref = self.lat_ref(t)
        psi_ref = math.atan2(vref, max(st.v, 1.0))
        delta = -((st.yaw - psi_ref) + math.atan(self.k_lat * (st.lat - ref) / (max(st.v, 1.0) + 1.0)))
        steer = float(np.clip(delta / self.max_steer, -1.0, 1.0))
        if self.long == "brake":
            thr, brk = 0.0, float(self.brake_frac)
        elif self.long == "hold":
            thr, brk = 0.0, 1.0
        elif self.long == "accel":
            thr, brk = self.accel_thr, 0.0
        else:
            thr, brk = self.pi(self.v_hold - st.v)
        return thr, brk, steer


def hold_lane_control(st: ActorState, lat_ref, v_ref, max_steer_rad, k_lat=1.2, kp=0.6, pi=None):
    """Tracker for scripted actors: hold a lateral reference and a speed reference."""
    delta = -(st.yaw + math.atan(k_lat * (st.lat - lat_ref) / (max(st.v, 1.0) + 1.0)))
    steer = float(np.clip(delta / max_steer_rad, -1.0, 1.0))
    e = v_ref - st.v
    if pi is not None:
        return (*pi(e), steer)
    return float(np.clip(kp * e, 0, 1)), float(np.clip(-kp * e, 0, 1)), steer
