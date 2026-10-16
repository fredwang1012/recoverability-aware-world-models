"""Core cases (RC03/07/16/19/20) through the harness on the KINEMATIC MOCK plant: the mechanism contrasts the PDF requires.
Not a CARLA result."""
import json

import margin as M
import validate_records as VR
from cases import two_lane as TL
from sim.plant_kinematic import KinematicPlant

CFG = M.DEFAULT_V2


def plant(b=4.0):
    return lambda: KinematicPlant(b_max=b)


def row(rec):
    return {p: rec.R_raw[i][0] for i, p in enumerate(rec.policies)}


def test_rc03_braking_fails_but_bypass_is_a_witness():
    th = {"gap_m": 26.0, "v_mps": 14.0, "tau_s": 0.25}
    rec = TL.run("RC03", plant(4.0), th, CFG, "mock", "mock_replay")
    r = row(rec)
    assert r["brake_hold"] < CFG.eps and r["bypass_left"] >= CFG.eps           # PDF required profile contrast
    assert rec.evidence_class == "R_REF" and rec.best_policy in ("bypass_left", "brake_steer_left")
    assert VR.check_record(json.loads(json.dumps(rec.to_json()))) == []


def test_rc03_closing_the_escape_corridor_is_negative_but_only_U_LIBRARY():
    th = {"gap_m": 26.0, "v_mps": 14.0, "tau_s": 0.25, "closed": True}
    rec = TL.run("RC03", plant(4.0), th, CFG, "mock", "mock_replay")
    assert rec.evidence_class == "U_LIBRARY"                                    # never U_MODEL from finite replay
    assert rec.evidence_scope == "mock_replay"


def test_rc07_handback_refuted_continuation_passes():
    th = {"gap_m": 30.0, "v_mps": 12.0, "tau_s": 0.25, "rear_gap_m": 70.0, "rear_v_mps": 22.0}
    rec = TL.run("RC07", plant(6.0), th, CFG, "mock", "mock_replay")
    r = row(rec)
    assert r["bypass_then_stop"] < CFG.eps and r["bypass_then_return"] >= CFG.eps
    assert rec.evidence_class == "R_REF"                                        # scene is NOT declared unrecoverable
    ev = rec.events["bypass_then_stop/rear_cruise"]
    assert ev["termination"] == "collision" and ev["events"][0]["other"] == "rear"
    # advance the rear actor (paired contrast): the hand-back continuation is refuted too, yet braking in place still
    # witnesses the scene, so it stays recoverable - one refuted policy never makes a scene unrecoverable
    near = TL.run("RC07", plant(6.0), dict(th, rear_gap_m=60.0), CFG, "mock", "mock_replay")
    assert row(near)["bypass_then_return"] < CFG.eps and near.evidence_class == "R_REF" and near.best_policy == "brake_hold"


def test_rc16_point_clear_body_conflict_and_wider_path_witness():
    base = {"v_mps": 10.0, "tau_s": 0.0}
    r_contact = row(TL.run("RC16", plant(6.0), dict(base, offset_m=1.2), CFG, "mock", "mock_replay"))
    assert r_contact["lane_follow"] == float("-inf")                 # centre line clears the post; the body overlaps it
    rec = TL.run("RC16", plant(6.0), dict(base, offset_m=1.4), CFG, "mock", "mock_replay")
    r = row(rec)
    assert -1.0 < r["lane_follow"] < CFG.eps                          # no contact (finite margin) but the reserve is violated
    assert r["nudge_left"] >= CFG.eps and rec.evidence_class == "R_REF"   # wider path witnesses the scene
    assert r["brake_hold"] >= CFG.eps                                 # earlier braking is also a supported alternative
    rec_w = TL.run("RC16", plant(6.0), dict(base, offset_m=2.6), CFG, "mock", "mock_replay")
    assert row(rec_w)["lane_follow"] >= CFG.eps                       # wide gate: lane following is a witness


def test_rc19_benign_anchor_and_nuisance_pair_agree():
    th = {"v_mps": 12.0}
    a = TL.run("RC19", plant(), th, CFG, "mock", "mock_replay")
    b = TL.run("RC19", plant(), th, CFG, "mock", "mock_replay", spec=TL.rc19_spec(th, CFG, nuisance=True))
    assert a.evidence_class == "R_REF" and a.R_raw[0][0] > 0.5                  # benign: no intervention, large margin
    assert abs(a.R_raw[0][0] - b.R_raw[0][0]) < 1e-9                             # nuisance twin: profile agrees
    assert a.events["lane_follow/static_blocker"]["events"] == []


def test_rc20_incomplete_library_unresolved_then_witness_with_physics_unchanged():
    th = {"gap_m": 26.0, "v_mps": 14.0, "tau_s": 0.25}
    spec = TL.rc03_spec(th, CFG)
    restricted = TL.rc03_library(with_bypass=False)
    full = TL.run("RC03", plant(4.0), th, CFG, "mock", "mock_replay", spec=spec, library=restricted)
    assert full.evidence_class == "U_LIBRARY"                                    # exhaustively refuted restricted library
    budget = TL.run("RC03", plant(4.0), th, CFG, "mock", "mock_replay", spec=spec, library=restricted, run_mask={(0, 0)})
    assert budget.evidence_class == "UNRESOLVED" and budget.R_raw[1][0] is None  # missing entry is None, never 0 / failure
    expanded = TL.run("RC03", plant(4.0), th, CFG, "mock", "mock_replay", spec=spec)
    assert expanded.evidence_class == "R_REF"
    assert expanded.geometry["spec"] == full.geometry["spec"]                    # same physical scene, only the library changed
