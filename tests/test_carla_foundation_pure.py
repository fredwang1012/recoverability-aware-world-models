"""Pure helpers of scripts/carla_foundation.py (no CARLA server needed)."""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import carla_foundation as CF


def test_interp_zero_ignores_inf_and_finds_crossing():
    assert abs(CF.interp_zero([14, 15, 16, 18], [-math.inf, -0.2, 0.2, 0.7]) - 15.5) < 1e-9
    assert CF.interp_zero([1, 2], [-math.inf, -math.inf]) is None


def test_g_of_decodes_serialised_minus_inf():
    r = {"policies": ["a", "b"], "R_raw": [["-inf"], [0.5]]}
    assert CF.G_of(r, "a") == -math.inf and CF.G_of(r, "b") == 0.5
