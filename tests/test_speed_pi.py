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
