import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "mining"))
from sim.trackers import SpeedPI  # noqa: E402


def test_ki_zero_is_the_old_p_law():
    pi = SpeedPI(0.6, 0.0)
    for e in (-2.0, -0.3, 0.0, 0.4, 3.0):
        assert pi(e) == (float(np.clip(0.6 * e, 0, 1)), float(np.clip(-0.6 * e, 0, 1)))


def test_integral_builds_feedforward_and_is_bounded():
    pi = SpeedPI(0.6, 0.3, 0.05)
    for _ in range(2000):
        pi(0.5)
    assert pi.i == 1.0                                  # anti-windup clip
    thr, brk = pi(-0.1)
    assert thr > 0 and brk == 0.0                       # feed-forward keeps throttle on for a small overspeed


def test_brake_to_keep_keeps_rescaled_feedforward():
    from sim.trackers import EgoController
    from sim.types import ActorState, EgoCommand
    st = ActorState(0.0, 0.0, 0.0, 10.0, 2.3, 0.95)
    ctl = EgoController(0.5, ki_speed=0.3, i0=0.4)
    ctl.reset(st)
    ctl.apply(EgoCommand(long="brake"), st, 0.0)
    slow = ActorState(0.0, 0.0, 0.0, 5.0, 2.3, 0.95)
    ctl.apply(EgoCommand(long="keep"), slow, 1.0)
    assert abs(ctl.pi.i - 0.2) < 1e-9                     # 0.4 * 5/10
    assert ctl.v_hold == 5.0
    thr, brk, _ = ctl.control(slow, 1.0)
    assert thr > 0.15 and brk == 0.0                      # no deceleration kick on resume


def test_brake_to_keep_ki_zero_matches_old_behaviour():
    from sim.trackers import EgoController
    from sim.types import ActorState, EgoCommand
    st = ActorState(0.0, 0.0, 0.0, 10.0, 2.3, 0.95)
    ctl = EgoController(0.5)
    ctl.reset(st)
    ctl.apply(EgoCommand(long="brake"), st, 0.0)
    ctl.apply(EgoCommand(long="keep"), st, 1.0)
    assert ctl.control(st, 1.0)[:2] == (0.0, 0.0)
