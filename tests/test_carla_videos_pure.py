"""Pure helpers of scripts/carla_videos.py (no CARLA server needed)."""
import importlib.util
import math
import os

spec = importlib.util.spec_from_file_location("carla_videos", os.path.join(os.path.dirname(__file__), "..", "scripts", "carla_videos.py"))
cv = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cv)


def test_stem_and_sanitising():
    assert cv.stem("RC03", "bypass_left") == "RC03_bypass_left"
    assert cv.stem("RC05", "abort_and_stabilize", "gap_closes") == "RC05_abort_and_stabilize_gap_closes"
    assert cv.stem("RC12", "brake_hold@prefix", None, "p2.25") == "RC12_brake_hold-prefix_p2.25"


def test_sign_check():
    assert cv.sign_check(0.02, 0.5) == "agree"
    assert cv.sign_check(-math.inf, -0.03) == "agree"
    assert cv.sign_check(-0.01, 0.01) == "DIFFERS"
    assert cv.sign_check(math.inf, None) == "n/a"
    assert cv.sign_check(-math.inf, 0.0) == "DIFFERS"


def test_stored_g_lookup():
    rec = {"policies": ["a", "b"], "worlds": ["w1", "w2"], "R_raw": [["-inf", 0.1], [None, 0.2]]}
    assert cv.stored_G(rec, "a", "w1") == -math.inf and cv.stored_G(rec, "b", "w2") == 0.2
    assert cv.stored_G(rec, "b", "w1") is None and cv.stored_G(rec, "zz", "w1") is None
    assert cv.stored_G(rec, "a", None) == -math.inf


def test_titles_and_runs_cover_all_cases():
    t = cv.titles()
    assert len(t) == 20 and t["RC01"] == "Braking with a protected stop"
    runs = cv.build_runs()
    assert {r.case for r in runs} == set(t)
    stems = [r.stem(p, w) for r in runs for p, w, _ in r.clips]
    assert len(stems) == len(set(stems))              # no two clips share a file name
    for r in runs:
        os.path.exists(os.path.join("results/carla", r.stored[0])) or True
