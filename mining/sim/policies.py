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


def _obstacle_passed(policy, ob):
    """True once an obstacle in the ego/adjacent lane has been SEEN and none remains ahead or alongside (observations only)."""
    ahead = [o for o in ob.objs if o["dx"] > -(o["hl"] + ob.ego_hl + 1.0) and abs(o["dlat"]) <= 1.6 * ob.lane_w]
    if ahead:
        policy.seen = True
    return bool(getattr(policy, "seen", False) and not ahead)


class Nudge(Policy):
    """Shift laterally to `lat_offset` (m, road frame, + = right) and stay there: the 'wider path' witness for RC16."""
    terminal = "lane_follow"

    def __init__(self, lat_offset=-0.8, lc_time=1.5, **kw):
        super().__init__(lat_offset=lat_offset, lc_time=lc_time, **kw)
        self.name = f"nudge_{'left' if lat_offset < 0 else 'right'}"

    def decide(self, history, t):
        return EgoCommand(long="keep", lat_target=self.params["lat_offset"], lc_time=self.params["lc_time"])


class BypassThenStop(Policy):
    """RC07 premature-hand-back policy: lane change left, then stop in the left lane once the obstacle is passed.
    Decision uses only delivered observations (obstacle behind us => passed)."""
    name, terminal = "bypass_then_stop", "protected_hold"

    def __init__(self, lc_time=2.0, **kw):
        super().__init__(lc_time=lc_time, **kw)

    def reset(self):
        super().reset(); self.passed, self.seen = False, False

    def decide(self, history, t):
        ob = history[-1]
        if not self.passed and _obstacle_passed(self, ob):
            self.passed = True
        if self.passed:
            return EgoCommand(long="brake", lat_target=-ob.lane_w)
        return EgoCommand(long="keep", lat_target=-ob.lane_w, lc_time=self.params["lc_time"])


class BypassThenReturn(Policy):
    """RC07 supported continuation: lane change left, then once the obstacle is passed return to the original lane and keep
    driving (hand back and continue), clearing the left lane for the fast rear actor."""
    name, terminal = "bypass_then_return", "lane_follow"

    def __init__(self, lc_time=2.0, **kw):
        super().__init__(lc_time=lc_time, **kw)

    def reset(self):
        super().reset(); self.passed, self.seen = False, False

    def decide(self, history, t):
        ob = history[-1]
        if not self.passed and _obstacle_passed(self, ob):
            self.passed = True
        if self.passed:
            return EgoCommand(long="keep", lat_target=0.0, lc_time=self.params["lc_time"])
        return EgoCommand(long="keep", lat_target=-ob.lane_w, lc_time=self.params["lc_time"])


# ----------------------------------------------------------------------------- compositional cases (RC05, RC06, RC15, RC18)
def _target_lane_objs(ob, behind_only=True):
    """Anonymous objects seen in the left-neighbour lane band while the ego is still in its own lane (dlat in (-1.5, -0.5) lanes)."""
    return [o for o in ob.objs if -1.5 * ob.lane_w < o["dlat"] < -0.5 * ob.lane_w and (o["dx"] < 0 or not behind_only)]


class BrakeSteer(Policy):
    """RC18: brake and steer AT THE SAME TIME (a combined brake-steer command; brake_frac of full brake)."""
    terminal = "lane_follow"

    def __init__(self, side=-1, lc_time=1.5, brake_frac=1.0, **kw):
        super().__init__(side=side, lc_time=lc_time, brake_frac=brake_frac, **kw)
        self.name = f"brake_steer_cmd_lc{lc_time:g}_b{brake_frac:g}"

    def decide(self, history, t):
        lane_w = history[-1].lane_w if history else 3.5
        p = self.params
        return EgoCommand(long="brake", brake_frac=p["brake_frac"], lat_target=p["side"] * lane_w, lc_time=p["lc_time"])


class ImmediateEntry(Policy):
    """RC06 / RC15: enter the left lane at once (no observation of the neighbour lane)."""
    name, terminal = "immediate_entry", "lane_follow"

    def __init__(self, lc_time=2.0, **kw):
        super().__init__(lc_time=lc_time, **kw)

    def decide(self, history, t):
        lane_w = history[-1].lane_w if history else 3.5
        return EgoCommand(long="keep", lat_target=-lane_w, lc_time=self.params["lc_time"])


class AlwaysMerge(ImmediateEntry):
    name = "always_merge"


class WaitObserveEnter(Policy):
    """RC06: yield to the vehicle passing in the target lane, then enter once the observed gap is usable.  While waiting the ego
    keeps its speed (or brakes at wait_brake_frac > 0).  A target-lane object is blocking when it is behind by less than
    hl_sum + back_m + back_t * closing_speed, or ahead by less than hl_sum + front_m.  Entry is latched."""
    terminal = "lane_follow"

    def __init__(self, lc_time=2.0, back_m=5.0, back_t=1.5, front_m=5.0, wait_brake_frac=0.0, **kw):
        super().__init__(lc_time=lc_time, back_m=back_m, back_t=back_t, front_m=front_m, wait_brake_frac=wait_brake_frac, **kw)
        self.name = "wait_observe_enter" if not wait_brake_frac else "wait_brake_observe_enter"

    def reset(self):
        super().reset(); self.entered = False

    def _blocked(self, ob):
        p = self.params
        for o in _target_lane_objs(ob, behind_only=False):
            hs = o["hl"] + ob.ego_hl
            if o["dx"] < 0 and -o["dx"] < hs + p["back_m"] + p["back_t"] * max(o["dv"], 0.0):
                return True
            if o["dx"] >= 0 and o["dx"] < hs + p["front_m"]:
                return True
        return False

    def decide(self, history, t):
        ob = history[-1]
        if not self.entered and not self._blocked(ob):
            self.entered = True
        if self.entered:
            return EgoCommand(long="keep", lat_target=-ob.lane_w, lc_time=self.params["lc_time"])
        if self.params["wait_brake_frac"]:
            return EgoCommand(long="brake", brake_frac=self.params["wait_brake_frac"])
        return EgoCommand(long="keep")


class ObserveYieldElseFallback(Policy):
    """RC15: merge left only after the neighbour's YIELD has been observed (its closing speed dv fell by `drop` m/s from the
    largest value seen); if no yield is observed within `wait_s`, fall back to full braking in the lane.  Both latch.
    The terminal test follows the branch taken (a stopped ego in its lane centre also satisfies lane_follow)."""
    name = "observe_yield_else_fallback"

    def __init__(self, lc_time=2.0, wait_s=2.0, drop=1.5, **kw):
        super().__init__(lc_time=lc_time, wait_s=wait_s, drop=drop, **kw)

    terminal = "lane_follow"

    def reset(self):
        super().reset(); self.dv_peak, self.merged, self.fell_back = None, False, False

    def decide(self, history, t):
        ob, s, p = history[-1], self._since(t), self.params
        if not self.merged and not self.fell_back:
            objs = _target_lane_objs(ob)
            if objs:
                dv = max(o["dv"] for o in objs)
                self.dv_peak = dv if self.dv_peak is None else max(self.dv_peak, dv)
                if dv <= self.dv_peak - p["drop"]:
                    self.merged = True
            if not self.merged and s >= p["wait_s"]:
                self.fell_back = True
        if self.merged:
            return EgoCommand(long="keep", lat_target=-ob.lane_w, lc_time=p["lc_time"])
        if self.fell_back:
            return EgoCommand(long="brake")
        return EgoCommand(long="keep")


class LaneChangePrefix(Policy):
    """RC05: the ego is DURING a lane change: for prefix_s it commands the left lane change (physically executed in the episode,
    never a lateral teleport); `after` then decides.  Base class = commit (keep going)."""
    name, terminal = "commit_lane_change", "lane_follow"

    def __init__(self, prefix_s=1.0, lc_time=3.0, abort_lc=1.5, **kw):
        super().__init__(prefix_s=prefix_s, lc_time=lc_time, abort_lc=abort_lc, **kw)

    def _commit(self, ob):
        return EgoCommand(long="keep", lat_target=-ob.lane_w, lc_time=self.params["lc_time"])

    def _abort(self, t):
        """Steer back to the original lane at the current speed, then brake once the return transition has settled (braking while
        still steering leaves a residual yaw at standstill).  The abort start time is latched."""
        if getattr(self, "t_abort", None) is None:
            self.t_abort = t
        if t - self.t_abort < self.params["abort_lc"] + self.params.get("settle_s", 0.3):
            return EgoCommand(long="keep", lat_target=0.0, lc_time=self.params["abort_lc"])
        return EgoCommand(long="brake", lat_target=0.0, lc_time=self.params["abort_lc"])

    def after(self, history, t):
        return self._commit(history[-1])

    def decide(self, history, t):
        if self._since(t) < self.params["prefix_s"]:
            return self._commit(history[-1])
        return self.after(history, t)


class AbortAndStabilize(LaneChangePrefix):
    """RC05: after the prefix, abort unconditionally: steer back to the original lane and brake."""
    name = "abort_and_stabilize"

    def reset(self):
        super().reset(); self.t_abort = None

    def after(self, history, t):
        return self._abort(t)


class AbortIfClosing(LaneChangePrefix):
    """RC05 (observation-conditioned): keep changing lanes until a target-lane object behind the ego is seen closing with time to
    contact below ttc_s, then abort (latched).  Sees only delivered observations."""
    name = "abort_if_closing"

    def __init__(self, ttc_s=4.0, **kw):
        super().__init__(ttc_s=ttc_s, **kw)

    def reset(self):
        super().reset(); self.aborted = False; self.t_abort = None

    def _closing(self, ob):
        for o in ob.objs:
            if o["dx"] < 0 and abs(o["dlat"]) <= 1.6 * ob.lane_w and o["dv"] > 0.5:
                if (-o["dx"] - o["hl"] - ob.ego_hl) / o["dv"] < self.params["ttc_s"]:
                    return True
        return False

    def decide(self, history, t):
        if not self.aborted and self._closing(history[-1]):
            self.aborted = True
        if self.aborted:
            return self._abort(t)
        return self._commit(history[-1])


# ----------------------------------------------------------------------------- partial observation (RC10, RC11, RC14)
def _hidden_candidates(ob):
    """Anonymous objects ahead of (or alongside) the ego within +-1.6 lane widths, nearest first (delivered observation only)."""
    return sorted((o for o in ob.objs if o["dx"] > -(o["hl"] + ob.ego_hl + 1.0) and abs(o["dlat"]) <= 1.6 * ob.lane_w), key=lambda o: o["dx"])


class SharedSlowdownThenBranch(Policy):
    """RC10 witness (contingency-MPC idea on a finite tree): a SHARED slowdown trunk (full brake, in the ego lane) that is the same in
    every world, then a BRANCH chosen from the delivered observation of the stopped vehicle: if it is in the ego lane, change to the
    free adjacent (left) lane while braking continues (the trunk keeps buying time); if it is in the adjacent lane, stay in the lane
    and keep braking.  Braking during the lane change is released once the ego speed is below `v_floor` (m/s): a stop in the middle of a
    steered transition leaves a residual yaw, which the lane_follow terminal test would reject, so the slow ego completes the change at
    constant speed.  memory=True latches the branch the first time the object has been delivered (decisive evidence stays causally
    available); memory=False decides from the CURRENT frame only (RC14 foil: it forgets the object once it is occluded and returns to
    the lane).  blind="change_left" ignores observations altogether (the info_tree blind(side=left, p) policy; named fixed_left)."""
    terminal = "lane_follow"

    def __init__(self, lc_time=2.0, trunk_brake=1.0, branch_brake=1.0, memory=True, v_floor=8.0, blind=None, **kw):
        super().__init__(lc_time=lc_time, trunk_brake=trunk_brake, branch_brake=branch_brake, memory=memory, v_floor=v_floor, blind=blind, **kw)
        self.name = "shared_slowdown_then_branch" if memory else "branch_current_frame_only"
        if blind:
            self.name = f"blind_{blind}"

    def reset(self):
        super().reset(); self.branch, self.released = None, False

    @staticmethod
    def _classify(ob):
        c = _hidden_candidates(ob)
        if not c:
            return None
        return "change_left" if abs(c[0]["dlat"]) < 0.5 * ob.lane_w else "stay"

    def decide(self, history, t):
        p, ob = self.params, history[-1]
        if p["blind"]:
            b = p["blind"]
        elif p["memory"]:
            if self.branch is None:
                for h in history:
                    b = self._classify(h)
                    if b:
                        self.branch = b
                        break
            b = self.branch
        else:
            b = self._classify(ob)
        if b == "change_left":
            if ob.v < p["v_floor"]:
                self.released = True
            if self.released:
                return EgoCommand(long="keep", lat_target=-ob.lane_w, lc_time=p["lc_time"])
            return EgoCommand(long="brake", brake_frac=p["branch_brake"], lat_target=-ob.lane_w, lc_time=p["lc_time"])
        if b == "stay":
            return EgoCommand(long="brake", brake_frac=p["branch_brake"], lat_target=0.0, lc_time=p["lc_time"])
        return EgoCommand(long="brake", brake_frac=p["trunk_brake"], lat_target=None if p["memory"] else 0.0)
