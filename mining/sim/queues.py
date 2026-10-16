"""
queues.py - explicit observation delivery and command pipelines (PDF 8.1-8.2, task B6).

Timeline of one decision (all on the tick grid, realized values logged):
    capture (t_capture) -> delivery (+obs ticks) -> decision (policy tick) -> planning (+plan ticks)
    -> command actuation (+act ticks) -> command effective (applied) -> measured response (from the trace)
While a command is pending the PREVIOUS control keeps being applied (the world is never paused, nothing is backdated).
A frame captured earlier but delivered later is NOT available retrospectively.
"""
import math
from dataclasses import dataclass, field

from .types import DT, Obs


def ticks(seconds, dt=DT):
    """Quantise a configured delay to the tick grid (round to nearest tick)."""
    return int(round(seconds / dt))


class ObservationFeed:
    """Structured simulator-derived observations with a visibility mask and a delivery delay.

    reveal: {actor_name: t_reveal_s} - the actor is visible only for t >= t_reveal (occluded family / RC10-RC14).
    forget_after: {actor_name: t_s} - the actor stops being observed for t >= t_s (RC14 occlude-after-seen)."""

    def __init__(self, lane_w, obs_hz=20.0, delivery_s=0.0, reveal=None, forget_after=None, dt=DT):
        self.lane_w, self.dt = lane_w, dt
        self.every = max(int(round(1.0 / (obs_hz * dt))), 1)
        self.delay_k = ticks(delivery_s, dt)
        self.reveal, self.forget = reveal or {}, forget_after or {}
        self.pending, self.log = [], []

    def visible(self, name, t):
        return t >= self.reveal.get(name, -math.inf) and t < self.forget.get(name, math.inf)

    def capture(self, snap):
        if snap.k % self.every:
            return
        ego = snap.actors["ego"]
        objs = []
        for name, a in snap.actors.items():
            if name == "ego" or not self.visible(name, snap.t):
                continue
            objs.append({"dx": a.x - ego.x, "dlat": a.lat - ego.lat, "dv": a.v - ego.v, "hl": a.hl, "hw": a.hw})
        objs.sort(key=lambda o: (o["dx"], o["dlat"]))            # anonymous: order carries no actor identity
        ob = Obs(t_capture=snap.t, t_available=snap.t + self.delay_k * self.dt, v=ego.v, lane_w=self.lane_w, objs=objs)
        self.pending.append((snap.k + self.delay_k, ob))

    def deliver(self, k):
        due = [o for (kk, o) in self.pending if kk <= k]
        self.pending = [(kk, o) for (kk, o) in self.pending if kk > k]
        for o in due:
            self.log.append({"t_capture": o.t_capture, "t_available": o.t_available})
        return due


@dataclass
class CommandPipeline:
    plan_s: float = 0.0
    act_s: float = 0.0
    dt: float = DT
    queue: list = field(default_factory=list)
    log: list = field(default_factory=list)

    def submit(self, cmd, decision_k):
        eff = decision_k + ticks(self.plan_s, self.dt) + ticks(self.act_s, self.dt)
        self.queue.append((eff, cmd))
        self.log.append({"decision_t": decision_k * self.dt, "effective_t": eff * self.dt, "long": cmd.long,
                         "lat_target": cmd.lat_target})

    def due(self, k):
        out = [c for (e, c) in self.queue if e <= k]
        self.queue = [(e, c) for (e, c) in self.queue if e > k]
        return out
