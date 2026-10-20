"""Difficult negatives (RC09, RC13) through the harness on the KINEMATIC MOCK plant, plus the pure predictor / counterexample helpers.
Not a CARLA result.  Labels by evidence only."""
import json
import math
import os
import sys

import pytest

import margin as M
import validate_records as VR
from cases import negatives as NG
from cases import occlusion as OC
from sim.plant_kinematic import KinematicPlant
from sim.types import Obs

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import carla_negatives as CN

CFG = M.DEFAULT_V2
TH = {"gap_m": 27.0, "v_mps": 14.0, "tau_s": 0.25, "rear_gap_m": 25.0, "rear_dv_mps": 6.0}


def plant():
    return KinematicPlant(b_max=6.2)


def run9(th=TH, **kw):
    rec = NG.rc09_run(plant, th, CFG, "mock", "mock_replay", **kw)
    assert VR.check_record(json.loads(json.dumps(rec.to_json()))) == []
    return rec


@pytest.fixture(scope="module")
def base_rec():
    """The default RC09 mock record, run once for the module."""
    return run9()


def rows(rec):
    return {p: rec.R_raw[i][0] for i, p in enumerate(rec.policies)}


def test_rc09_class_is_exactly_the_declared_finite_set():
    members = NG.rc09_members()
    names = [m["name"] for m in members]
    assert len(names) == len(set(names)) == 25
    assert [f"brake_frac_{f:.1f}" for f in NG.RC09_BRAKE_FRACS] == [n for n in names if n.startswith("brake_frac_")]
    assert {"keep_speed", "accelerate", "bypass_left_lc1", "bypass_left_lc2", "bypass_right_lc2", "staged_brake_0.4_change_lc1"} <= set(names)
    byname = {m["name"]: m for m in members}
    assert byname["brake_frac_0.3"]["params"]["brake_frac"] == 0.3 and byname["bypass_left_lc1"]["params"]["lc_time"] == 1.0


def test_rc09_every_member_refuted_with_a_counterexample(base_rec):
    rec = base_rec
    assert rec.evidence_class == "U_LIBRARY"
    ce = NG.counterexamples(rec)
    assert set(ce) == {m["name"] for m in NG.rc09_members()}
    assert all(c["refuted"] for c in ce.values())
    assert all(c["world"] == "rear_cruise" for c in ce.values())
    # every refutation is a contact except the right-side move, which leaves the road (road-edge margin)
    assert {p for p, c in ce.items() if c["collided_with"] is None} <= {"bypass_right_lc2"}
    assert {c["collided_with"] for c in ce.values()} >= {"blk", "rear"} and any(str(c["collided_with"]).startswith("adj") for c in ce.values())


def test_rc09_controls_the_rear_and_the_left_queue_are_what_close_the_scene():
    no_rear = rows(run9(rear=False))
    assert no_rear["brake_frac_1.0"] >= CFG.eps            # without the rear vehicle, stopping is a witness
    open_left = rows(run9(left_queue=False))
    assert max(open_left[k] for k in ("bypass_left_lc1", "brake_steer_left_lc1_b1", "bypass_left_lc2")) >= CFG.eps


def test_rc09_scene_geometry():
    pos = NG.queue_positions(27.0)
    assert pos[0] == -4.6 and pos[-1] == 27.0
    assert all(b - a >= NG.CAR_LEN - 1e-9 for a, b in zip(pos, pos[1:]))          # no overlaps
    sp = NG.rc09_spec(TH, CFG)
    assert sum(k.startswith("adj") for k in sp["actors"]) == len(pos)
    assert all(sp["actors"][k]["static"] and sp["actors"][k]["lat"] == -3.5 for k in sp["actors"] if k.startswith("adj"))
    r = sp["actors"]["rear"]
    assert r["v"] == 20.0 and math.isclose((-2 * NG.EGO_HL) - (r["x"] + r["hl"]), 25.0)      # bumper-to-bumper gap
    assert sp["a_lat_max"] == 8.0


def test_counterexamples_follow_the_record_numbers_not_intent(base_rec):
    r = json.loads(json.dumps(base_rec.to_json()))
    r["R_raw"][3][0] = 0.5
    assert NG.counterexamples(r)[r["policies"][3]]["refuted"] is False


def test_decel_between_linear_and_degenerate():
    dt = 0.05
    vs = [14.0 - 5.0 * i * dt for i in range(60)]
    d = NG.decel_between(vs, dt)
    assert abs(d[0] - 5.0) < 0.1 and d[3] <= 0.3 * 14.0
    assert NG.decel_between([14.0] * 20, dt) is None


# ----------------------------------------------------------------------------- RC13
def obs_hid(dx, dlat, v=14.0):
    return Obs(0.0, 0.0, v, 3.5, [{"dx": dx, "dlat": dlat, "dv": -v, "hl": 2.3, "hw": 0.95}], 2.3)


def test_nominal_ego_motion():
    assert NG.nominal_ego("stay", 14.0, 0.0) == (0.0, 0.0)
    x, lat = NG.nominal_ego("stay", 14.0, 10.0)
    assert abs(x - (14 * 0.3 + 14.0 ** 2 / (2 * NG.B_NOM))) < 1e-6 and lat == 0.0
    x, lat = NG.nominal_ego("change_left", 14.0, 10.0)
    assert abs(lat + 3.5) < 1e-9 and x > 14 * 0.3 + (14 ** 2 - 8 ** 2) / (2 * NG.B_NOM)
    x0, _ = NG.nominal_ego("change_left", 6.0, 3.0)
    assert abs(x0 - 6.0 * 3.0) < 1e-9                                       # already below the floor: constant speed


def test_predictor_exact_is_right_and_inflation_closes_the_corridor():
    own = obs_hid(22.2, 0.0)        # D = 19 m scene at t_reveal 0.1: stopped vehicle in the ego lane, left lane free
    left = obs_hid(22.2, -3.5)      # stopped vehicle in the left lane
    exact = NG.Envelope()
    assert NG.action_rejected(own, "stay", exact)[0]
    assert not NG.action_rejected(own, "change_left", exact)[0]
    assert NG.action_rejected(left, "change_left", exact)[0]
    assert not NG.action_rejected(left, "stay", exact)[0]
    assert NG.action_rejected(own, "change_left", NG.Envelope(lat_m=2.0, lon_m=2.0))[0]       # false closure
    grid = [i * 0.1 for i in range(30)]
    m = NG.predictor_threshold(own, "change_left", grid)
    assert m is not None and 0.5 < m < 2.0
    assert not NG.action_rejected(own, "change_left", NG.Envelope(lat_m=m - 0.1, lon_m=m - 0.1))[0]


def test_predictor_growth_and_resolution_are_parameters():
    own = obs_hid(22.2, 0.0)
    assert not NG.action_rejected(own, "change_left", NG.Envelope(lat_m=0.5, lon_m=0.5))[0]
    assert NG.action_rejected(own, "change_left", NG.Envelope(lat_m=0.5, lon_m=0.5, growth_mps=1.0))[0]
    assert NG.envelope_dict(NG.Envelope(lat_m=1.0))["dt_s"] == 0.1


def test_predictor_input_is_an_anonymous_observation():
    """Behavioural: the object fields are only the anonymous ones, and the same Obs gives identical output whatever world produced it."""
    assert set(obs_hid(1, 0).objs[0]) == {"dx", "dlat", "dv", "hl", "hw"}
    ob_a, ob_b = obs_hid(22.2, 0.0), obs_hid(22.2, 0.0)          # 'from world A' and 'from world B': the same delivered frame
    env = NG.Envelope(lat_m=0.5, lon_m=0.5)
    for act in ("stay", "change_left"):
        assert NG.action_rejected(ob_a, act, env) == NG.action_rejected(ob_b, act, env)
    # an extra, non-anonymous field on the object is ignored (the result depends only on the anonymous fields)
    tagged = Obs(0.0, 0.0, 14.0, 3.5, [dict(ob_a.objs[0], world="hid_own_lane", name="hid")], 2.3)
    assert NG.action_rejected(tagged, "change_left", env) == NG.action_rejected(ob_a, "change_left", env)


def test_exact_envelope_at_the_evidence_reserve_rejects_a_witness_with_small_box_separation():
    own = obs_hid(22.2, 0.0)
    sep = NG.action_rejected(own, "change_left", NG.Envelope())[1]
    assert sep > 0 and not NG.action_rejected(own, "change_left", NG.Envelope())[0]
    assert NG.action_rejected(own, "change_left", NG.Envelope(clearance_m=sep + 0.01))[0]      # a clearance above the nominal separation closes it


def test_repeat_status_marginal_rule():
    eps, sg = 0.0, 0.0167
    assert CN.repeat_status([0.105, 0.110, 0.100], eps, sg)[1] is False
    assert CN.repeat_status([0.018, 0.012, 0.030], eps, sg)[1] is True                           # one repeat within sigma of eps
    r, marg, flips = CN.repeat_status([0.030, -0.005, 0.02], eps, sg)
    assert marg and flips and r == (-0.005, 0.030)
    assert CN.repeat_status([-math.inf, -math.inf], eps, sg)[1] is False                         # consistently refuted is not marginal
    assert CN.repeat_status([0.05, None], eps, sg)[1] is True


def test_scene_robust_uses_both_worlds_and_evidence_class():
    def rec(ga, gb, ev="R_REF"):
        return {"evidence_class": ev, "policies": ["shared_slowdown_then_branch"], "R_raw": [[ga, gb]]}
    assert CN.scene_robust([rec(0.1, 0.05), rec(0.1, 0.06)])[0]
    assert not CN.scene_robust([rec(0.1, 0.05), rec(0.1, 0.012)])[0]
    assert not CN.scene_robust([rec(0.1, 0.05), rec(0.1, 0.05, "U_INFORMATION")])[0]


def test_script_takes_dt_from_the_types_module():
    import sim.types as T
    assert CN.DT == T.DT == 0.05


def test_recording_wrapper_is_transparent_and_captures_the_first_object_frame():
    th = {"gap_m": 20.0, "v_mps": 14.0, "tau_s": 0.25, "t_reveal_s": 0.2}
    plain = OC.run("RC10", plant, th, CFG, "mock", "mock_replay")
    inst = []

    def wrap(mk):
        def f():
            p = NG.Recording(mk())
            inst.append(p)
            return p
        return f
    rec = OC.run("RC10", plant, th, CFG, "mock", "mock_replay", library_=[wrap(mk) for mk in OC.library()])
    assert rec.R_raw == plain.R_raw and rec.policies == plain.policies
    n_w = len(rec.worlds)
    ob_own = CN.recorded(inst, 3, 1, n_w).first_obs                 # shared_slowdown_then_branch, world hid_own_lane
    ob_left = CN.recorded(inst, 3, 0, n_w).first_obs
    assert abs(ob_own.objs[0]["dlat"]) < 0.1 and ob_own.t_capture >= 0.2 - 1e-9
    assert abs(ob_left.objs[0]["dlat"] + 3.5) < 0.1 and ob_left.objs[0]["dx"] == ob_own.objs[0]["dx"]


def test_rc13_mock_witness_is_rejected_only_by_the_inflated_predictor():
    th = {"gap_m": 20.0, "v_mps": 14.0, "tau_s": 0.25, "t_reveal_s": 0.2}
    sc = CN.rc13_scene(plant, th, CFG, "mock", "mock_replay", None)
    rec = sc["rec"]
    g = CN.mpol(rec)["shared_slowdown_then_branch"]
    assert rec["evidence_class"] == "R_REF" and g[0] >= CFG.eps and g[1] >= CFG.eps
    t = CN.rc13_table(sc, [NG.Envelope(), NG.Envelope(lat_m=2.0, lon_m=2.0)])
    exact, big = t[0], t[1]
    assert not exact["false_rejections"] and exact["agree"] == exact["n"]       # exact CV agrees with the evidence on every cell
    assert big["false_rejections"]                                              # inflation: witness cell rejected although r >= eps


def test_script_helpers():
    assert CN.fm(None) == "None" and CN.fm(-math.inf) == "-inf" and CN.fm(0.12345) == "0.123"
