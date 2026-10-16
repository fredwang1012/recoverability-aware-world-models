"""
policies.py - the finite, calibrated reference policy library (PDF 7.3).  Observation-conditioned, deterministic given
history.  A policy sees ONLY delivered Obs objects (no world id, no future, no scenario name, no evaluator state).

decide(history, t) is called on every decision tick with ALL delivered observations so far and returns an EgoCommand
(or None = keep the previous command).  Parameters, version and terminal mode are stored with each policy so records can
describe exactly what was run.
"""
from .types import EgoCommand

VERSION = "0.1"


class Policy:
    name = "policy"
    terminal = "protected_hold"          # which terminal test this continuation is judged by

    def __init__(self, **params):
        self.params = params

    def descriptor(self):
        return {"name": self.name, "version": VERSION, "params": dict(self.params), "terminal": self.terminal}

    def reset(self):
        self.t0 = None

    def _since(self, t):
        if self.t0 is None:
            self.t0 = t
        return t - self.t0

    def decide(self, history, t):               # pragma: no cover - overridden
        return None


class LaneFollow(Policy):
    """Nominal lane following at the current speed (benign anchor, RC19)."""
    name, terminal = "lane_follow", "lane_follow"

    def decide(self, history, t):
        return None


class BrakeHold(Policy):
    """Full brake, then hold the stop (RC01/RC02/RC12/RC17 witness)."""
    name = "brake_hold"

    def decide(self, history, t):
        return EgoCommand(long="brake", brake_frac=self.params.get("brake_frac", 1.0))


class Bypass(Policy):
    """Lane change to `side` (-1 = left, +1 = right) after `brake_s` seconds of braking, then stabilise (RC03/RC04)."""
    terminal = "lane_follow"

    def __init__(self, side=-1, lc_time=2.5, brake_s=0.0, **kw):
        super().__init__(side=side, lc_time=lc_time, brake_s=brake_s, **kw)
        self.name = ("bypass_left" if side < 0 else "bypass_right") if brake_s == 0 else \
                    ("brake_steer_left" if side < 0 else "brake_steer_right")

    def decide(self, history, t):
        s = self._since(t)
        p = self.params
        lane_w = history[-1].lane_w if history else 3.5
        if p["brake_s"] > 0 and s < p["brake_s"]:
            return EgoCommand(long="brake")
        return EgoCommand(long="keep", lat_target=p["side"] * lane_w, lc_time=p["lc_time"])


class StagedBrakeChange(Policy):
    """Brake for `p` seconds, then change lanes (RC04)."""
    name, terminal = "staged_brake_change", "lane_follow"

    def __init__(self, p=0.6, lc_time=2.5, side=-1, **kw):
        super().__init__(p=p, lc_time=lc_time, side=side, **kw)

    def decide(self, history, t):
        s = self._since(t)
        lane_w = history[-1].lane_w if history else 3.5
        if s < self.params["p"]:
            return EgoCommand(long="brake")
        return EgoCommand(long="keep", lat_target=self.params["side"] * lane_w, lc_time=self.params["lc_time"])


class BaselineThenHandover(Policy):
    """RC12: keep speed (the baseline prefix) for `prefix_s`, then hand over to `inner`.  The prefix itself is checked by
    the margin because the whole rollout is scored."""

    def __init__(self, inner, prefix_s=0.0, **kw):
        super().__init__(prefix_s=prefix_s, **kw)
        self.inner = inner
        self.name, self.terminal = f"{inner.name}@prefix", inner.terminal

    def descriptor(self):
        return {**super().descriptor(), "inner": self.inner.descriptor()}

    def reset(self):
        super().reset(); self.inner.reset()

    def decide(self, history, t):
        if self._since(t) < self.params["prefix_s"]:
            return None
        return self.inner.decide(history, t)
