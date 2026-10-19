"""
actors.py - project-owned finite-state controllers for decisive actors (PDF 8.3, task B7).

NO Traffic Manager / autopilot.  A world fixes each actor's MODE before the episode; transitions are keyed to simulation
time or observed geometry, never re-drawn per tick.  Modes:
    static          : never driven (blockers, parked cars)
    cruise          : hold speed v and lane
    delayed_brake   : cruise, then brake at b after t_brake
    yield           : cruise; once ego is alongside/ahead within trigger_gap, slow to v_yield after react_s
    nonyield        : explicit non-yielding (same as cruise, named so world sets are self-documenting)
    accelerate      : constant acceleration a
trigger_t (optional, yield): start reacting at that sim time instead of the geometric trigger.
Parameters ride in the world, e.g. {"adj": {"mode": "yield", "v_yield": 8.0, "react_s": 0.5}}.
"""
from .trackers import SpeedPI, hold_lane_control
from .types import ActorState


class ScriptedActor:
    def __init__(self, name, params, lane_lat, v0, max_steer_rad, dt, ki=0.0, i0=0.0):
        self.name, self.p, self.lane_lat, self.v_ref = name, params, lane_lat, v0
        self.mode = params.get("mode", "cruise")
        self.max_steer, self.dt = max_steer_rad, dt
        self.t_trigger = None
        self.pi = SpeedPI(0.6, ki, dt, i0) if ki else None

    def control(self, st: ActorState, ego: ActorState, t: float):
        m = self.mode
        if m == "static":
            return None
        if m == "accelerate":
            self.v_ref = min(self.v_ref + self.p.get("a", 1.5) * self.dt, self.p.get("v_max", 40.0))
        elif m == "delayed_brake" and t >= self.p.get("t_brake", 0.0):
            self.v_ref = max(self.v_ref - self.p.get("b", 4.0) * self.dt, 0.0)
        elif m == "yield":
            if self.t_trigger is None and "trigger_t" in self.p:        # time-keyed yield (chosen per world)
                if t >= self.p["trigger_t"]:
                    self.t_trigger = t
            elif self.t_trigger is None and ego.x > st.x - self.p.get("trigger_gap", 0.0) and abs(ego.lat - st.lat) < self.p.get("trigger_lat", 4.0):
                self.t_trigger = t
            if self.t_trigger is not None and t >= self.t_trigger + self.p.get("react_s", 0.5):
                self.v_ref = max(self.v_ref - self.p.get("b", 3.0) * self.dt, self.p.get("v_yield", 0.0))
        return hold_lane_control(st, self.lane_lat, self.v_ref, self.max_steer, pi=self.pi)
