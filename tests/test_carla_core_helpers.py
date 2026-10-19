"""Pure helpers of scripts/carla_core.py (no CARLA server needed)."""
import importlib.util
import math
import os

import margin as M
import validate_records as VR
from cases import two_lane as TL
from sim.plant_kinematic import KinematicPlant

spec = importlib.util.spec_from_file_location("carla_core", os.path.join(os.path.dirname(__file__), "..", "scripts", "carla_core.py"))
cc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cc)


def test_feasible_set_and_downset():
    s = cc.feasible_set_summary([0, .25, .5, .75], [True, True, True, False])
    assert s["upper"] == 0.5 and s["downset"]
    assert not cc.feasible_set_summary([0, .25, .5], [True, False, True])["downset"]


def test_tp_star_and_window_pick():
    assert abs(cc.t_p_star(40.0, 12.0) - (40 - 144 / (2 * 6.211) - 12 * 0.3 - 1) / 12) < 1e-12
    assert cc.middle_of_window([1, 2, 3, 4], [False, True, True, True]) == 3
    assert cc.middle_of_window([1], [False]) is None


def test_profile_diff_and_decode():
    assert cc.profile_max_abs_diff([0.5], [0.5]) == 0
    assert cc.profile_max_abs_diff([-math.inf], [0.5]) == math.inf
    assert cc.profile_max_abs_diff([None], [0.5]) == math.inf
    assert cc.gval("-inf") == -math.inf and cc.gval(None) is None


def test_contrast_helpers_on_mock_record():
    th = {"gap_m": 26.0, "v_mps": 14.0, "tau_s": 0.25}
    rec = TL.run("RC03", lambda: KinematicPlant(b_max=4.0), th, M.DEFAULT_V2, "mock", "mock_replay")
    import json
    r = json.loads(json.dumps(rec.to_json()))
    assert cc.rc03_contrast(r) and VR.check_record(r) == []
    assert TL.rc03_library(True)[2]().params["lc_time"] == 2.0           # mock default unchanged
    assert TL.rc03_library(True, 1.0)[2]().params["lc_time"] == 1.0


def test_rc07_contrast_requires_all_three_and_peak_alat():
    def rec(a, b, c):
        return {"policies": ["bypass_left", "bypass_then_stop", "bypass_then_return"], "R_raw": [[a], [b], [c]]}
    assert cc.rc07_contrast(rec(0.1, "-inf", 0.1))
    assert not cc.rc07_contrast(rec("-inf", "-inf", 0.1))          # primary-only collides: not the full contrast
    assert not cc.rc07_contrast(rec(0.1, 0.1, 0.1))
    assert not cc.rc07_contrast(rec(0.1, "-inf", None))            # unrun entry never counts
    r = {"traces": {"bypass_left/w": {"margin": {"parts": {"dyn": 0.25}}}}}
    assert abs(cc.peak_alat(r, "bypass_left") - 6.0) < 1e-12
    assert cc.with_limit({"a": 1})["a_lat_max"] == cc.A_LAT_MAX
    key, full = cc.rc07_select({(26, 80, 18): rec(0.1, "-inf", 0.1), (26, 80, 24): rec("-inf", "-inf", 0.1)})
    assert key == (26, 80, 18) and full == [(26, 80, 18)]
