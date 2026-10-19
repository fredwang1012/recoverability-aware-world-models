"""Pure / mock-only tests for Task 5 (CARLA mining families, pair descriptions, D4 helpers).  No CARLA server."""
import importlib.util
import math
import os

import numpy as np
import pytest

import margin as M
from families import B_CAL, TAU_PLANT, FAMILIES, FAMILIES_CARLA, CorridorCarla, Rc03Carla, corridor_anchor_m, mock_ctx


def _load(name):
    spec = importlib.util.spec_from_file_location(name, os.path.join(os.path.dirname(__file__), "..", "scripts", name + ".py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


MC = _load("mine_cases")
MP = _load("mining_post")


def test_corridor_carla_fixed_b_and_axes():
    f = FAMILIES_CARLA["corridor"]
    assert f.names == ["gap_m", "v_mps", "tau_s", "prefix_s"] and f.d == 4 and f.name == "corridor"
    th = f.decode([0.5, 0.5, 0.5, 0.5])
    assert th["b_mps2"] == 6.19 and abs(th["gap_m"] - 24.0) < 1e-9 and abs(th["v_mps"] - 13.0) < 1e-9
    assert abs(th["tau_s"] / 0.05 - round(th["tau_s"] / 0.05)) < 1e-9          # tick-quantised delay
    # mock families are untouched (5 axes with searched b)
    assert FAMILIES["corridor"].d == 5 and "b_mps2" in FAMILIES["corridor"].names


def test_corridor_carla_anchor_uses_calibration():
    f, ctx = FAMILIES_CARLA["corridor"], mock_ctx()
    th = {"gap_m": 30.0, "v_mps": 12.0, "tau_s": 0.25, "prefix_s": 0.1, "b_mps2": 6.19}
    want = 30.0 - 12.0 * (0.25 + 0.05 + 0.1) - 144.0 / (2 * 6.211) - M.DEFAULT_V2.reserve_m
    assert abs(f.anchor(th, ctx) - want) < 1e-12
    assert abs(corridor_anchor_m(30.0, 12.0, 0.30, 0.1, B_CAL, 1.0) - want) < 1e-12 and TAU_PLANT == 0.05
    # the mock anchor (plant b, no plant onset) differs, i.e. the CARLA variant really changes it
    assert abs(FAMILIES["corridor"].anchor(th, ctx) - want) > 0.1


def test_rc03_carla_space_covers_bypass_window():
    f = FAMILIES_CARLA["rc03"]
    assert f.hi[1] >= 24 and f.hi[0] >= 60 and f.names == ["gap_m", "v_mps", "tau_s"]
    th = f.decode([1, 1, 1])
    assert th["b_mps2"] == 6.19 and th["v_mps"] == 24 and th["gap_m"] == 60
    assert f.meta()["a_lat_max"] == 8.0 and f.meta()["lc_time"] == 2.0


def test_carla_variants_run_on_mock_plant():
    ctx = mock_ctx()
    rec = FAMILIES_CARLA["corridor"].run(FAMILIES_CARLA["corridor"].decode([0.9, 0.3, 0.2, 0.0]), ctx)
    assert rec.evidence_class in ("R_REF", "U_LIBRARY", "U_INFORMATION", "UNRESOLVED") and rec.family_id == "corridor"
    r3 = FAMILIES_CARLA["rc03"].run(FAMILIES_CARLA["rc03"].decode([0.5, 0.5, 0.2]), ctx)
    assert r3.geometry["spec"]["a_lat_max"] == 8.0 and "bypass_left" in r3.policies


def test_describe_pair_text():
    a = {"gap_m": 15.2, "v_mps": 12.0, "tau_s": 0.25, "prefix_s": 0.0}
    b = dict(a, gap_m=15.6)
    d = MC.describe_pair("gap_m", a, b)
    assert d.startswith("same scene except gap 15.2 m (unrecoverable side) vs 15.6 m (recoverable side)")
    assert "v 12 m/s" in d and "tau 0.25 s" in d and "WARNING" not in d
    assert "WARNING" in MC.describe_pair("gap_m", a, dict(b, v_mps=13.0))
    assert "after tick quantisation" in MC.describe_pair("tau_s", a, dict(a))


def test_v_raw_handles_inf_and_unrun():
    r = {"R_raw": [["-inf"], [0.2], [None]]}
    assert MC.v_raw(r) == 0.2
    assert MC.v_raw({"R_raw": [["-inf"]]}) == -math.inf
    assert MC.v_raw({"R_raw": [[None]]}) is None
    assert MC.v_raw({"R_raw": [[0.4, -0.1], [0.05, 0.3]]}) == 0.05


def test_d4_flag_and_near_threshold():
    assert not MP.d4_flag(0.004, "R", [(0.004, "R")] * 3)
    assert MP.d4_flag(0.004, "R", [(0.004, "R"), (-0.001, "U"), (0.004, "R")])
    assert MP.d4_flag(-0.2, "U", [(-0.2, "U"), (-0.2, "UNRESOLVED"), (-0.2, "U")])
    assert MP.near_threshold([0.5, 0.004, -0.009, None, -math.inf, 0.02], 0.01) == [1, 2]


def test_profile_mix_refused():
    ctx = {"backend": "carla", "pid": "pp_new"}
    with pytest.raises(RuntimeError):
        MC.check_profile({"physics_profile_id": "pp_old"}, ctx, "x.jsonl")
    MC.check_profile({"physics_profile_id": "pp_new"}, ctx, "x.jsonl")
    MC.check_profile({"physics_profile_id": None}, {"backend": "mock"}, "x.jsonl")      # mock: no check
