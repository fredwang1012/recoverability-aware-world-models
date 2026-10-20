"""Compositional cases (RC04/05/06/08/15/18) through the harness on the KINEMATIC MOCK plant: mechanism contrasts, labels by
evidence only.  Not a CARLA result."""
import json
import math
import os
import sys

import numpy as np

import margin as M
import validate_records as VR
from cases import compositional as CP
from sim import actors as A
from sim import harness as H
from sim import policies as P
from sim.plant_kinematic import KinematicPlant

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import carla_compositional as CCP

CFG = M.DEFAULT_V2


def plant(b=6.2):
    return lambda: KinematicPlant(b_max=b)


def rows(rec):
    return {p: rec.R_raw[i] for i, p in enumerate(rec.policies)}


def run(case, th, b=6.2, **kw):
    rec = CP.run(case, plant(b), th, CFG, "mock", "mock_replay", **kw)
    assert VR.check_record(json.loads(json.dumps(rec.to_json()))) == []
    return rec


def test_rc04_mock_staged_passes_where_brake_hold_and_immediate_change_fail():
    rec = run("RC04", {"gap_m": 30.0, "v_mps": 18.0, "tau_s": 0.25})
    r = rows(rec)
    assert r["brake_hold"][0] < CFG.eps                                       # cannot stop in time
    assert r["immediate_lane_change"][0] < CFG.eps                            # lc 1.6 from v = 18 breaks the a_lat limit
    assert r["staged_brake_0.4_change"][0] >= CFG.eps and rec.evidence_class == "R_REF"
    assert rec.geometry["spec"]["a_lat_max"] == 8.0


def test_rc05_physical_prefix_then_abort_vs_commit_two_worlds():
    th = {"gap_m": 60.0, "v_mps": 14.0, "tau_s": 0.25, "rear_gap_m": 35.0, "rear_v_mps": 20.0, "t_reveal_s": 1.25}
    rec = run("RC05", th)
    r = rows(rec)
    assert rec.worlds == ["gap_closes", "gap_open"]
    assert r["commit_lane_change"][0] < CFG.eps and r["commit_lane_change"][1] >= CFG.eps     # unsafe only when the gap closes
    assert all(g >= CFG.eps for g in r["abort_and_stabilize"]) and rec.evidence_class == "R_REF"


def test_rc05_prefix_is_executed_not_teleported():
    th = {"gap_m": 60.0, "v_mps": 14.0, "tau_s": 0.25, "rear_gap_m": 35.0, "rear_v_mps": 20.0, "t_reveal_s": 1.25}
    sp, w = CP.rc05_spec(th, CFG), CP.rc05_worlds(1.25)[0]
    res = H.run_episode(KinematicPlant(b_max=6.2), sp, CP.rc05_library()[1](), w, CFG)
    lat = np.array([e[2] for e in res.ego]); v = np.array([e[4] for e in res.ego])
    assert lat[0] == 0.0 and lat.min() < -0.5                                  # the lane change was physically started and moved the ego
    assert np.all(np.abs(np.diff(lat)) <= v[:-1] * 0.05 + 1e-9)                # lateral step bounded by v*dt: no teleport
    assert abs(lat[-1]) < 0.5                                                  # and the abort returned to the original lane


def test_rc06_wait_then_enter_vs_immediate_entry():
    rec = run("RC06", {"gap_m": 60.0, "v_mps": 10.0, "tau_s": 0.25, "pass_gap_m": 10.0, "pass_v_mps": 20.0})
    r = rows(rec)
    assert r["immediate_entry"][0] < CFG.eps and r["wait_observe_enter"][0] >= CFG.eps
    assert -1.0 < r["immediate_entry"][0] < CFG.eps                                 # reserve violated by the passer (finite margin)


def test_rc08_route_goal_is_stored_separately_from_safety():
    open_ = run("RC08", {"gap_m": 45.0, "v_mps": 14.0, "tau_s": 0.25})
    closed = run("RC08", {"gap_m": 45.0, "v_mps": 14.0, "tau_s": 0.25, "closed": True})
    g = lambda rec: {k.split("/")[0]: v["route_goal_completed"] for k, v in rec.traces.items()}
    assert g(open_) == {"brake_hold": False, "bypass_left": True, "lane_follow": False}
    assert g(closed)["brake_hold"] is False and not any(g(closed).values())     # safe waiting, route not completed
    assert rows(closed)["brake_hold"][0] >= CFG.eps and closed.evidence_class == "R_REF"
    # the goal never changes the label: same scene without a route goal gives the same margins
    sp = CP.rc08_spec({"gap_m": 45.0, "v_mps": 14.0, "tau_s": 0.25}, CFG)
    sp.pop("route_goal")
    plain = CP.run("RC08", plant(), {"gap_m": 45.0, "v_mps": 14.0, "tau_s": 0.25}, CFG, "mock", "mock_replay", spec=sp)
    assert plain.R_raw == open_.R_raw and all("route_goal_completed" not in t for t in plain.traces.values())


def test_rc15_yield_must_be_observed_two_worlds_mode_fixed_per_world():
    th = {"gap_m": 60.0, "v_mps": 10.0, "tau_s": 0.25, "adj_gap_m": 25.0, "adj_v_mps": 16.0}
    rec = run("RC15", th)
    r = rows(rec)
    assert rec.worlds == ["adj_yields", "adj_nonyield"]
    assert r["always_merge"][0] >= CFG.eps and r["always_merge"][1] < CFG.eps          # merging works only if the neighbour yields
    assert all(g >= CFG.eps for g in r["observe_yield_else_fallback"]) and rec.evidence_class == "R_REF"
    assert CCP.rc15_contrast(json.loads(json.dumps(rec.to_json()))) == "strong"
    assert CP.rc15_worlds()[0]["actors"]["adj"]["mode"] == "yield" and CP.rc15_worlds()[1]["actors"]["adj"]["mode"] == "nonyield"


def test_yield_actor_trigger_t_is_time_keyed():
    from sim.types import ActorState
    a = A.ScriptedActor("adj", {"mode": "yield", "trigger_t": 0.5, "react_s": 0.0, "b": 2.0, "v_yield": 5.0}, -3.5, 16.0, 0.5, 0.05)
    st, ego = ActorState(-30, -3.5, 0, 16.0, 2.3, 0.95), ActorState(0, 0, 0, 10.0, 2.3, 0.95)
    for k in range(10):
        a.control(st, ego, k * 0.05)
    assert a.v_ref == 16.0                                                         # nothing before trigger_t
    for k in range(10, 30):
        a.control(st, ego, k * 0.05)
    assert a.v_ref < 16.0


def test_rc18_combined_friction_circle_limit_refutes_brake_plus_steer():
    th = {"gap_m": 38.0, "v_mps": 18.0, "tau_s": 0.25}
    alat = run("RC18", th, spec=CP.rc18_spec(th, CFG, comb=False))
    comb = run("RC18", th, spec=CP.rc18_spec(th, CFG, comb=True))
    name = "brake_steer_cmd_lc2_b1"
    assert rows(alat)[name][0] >= CFG.eps                                       # passes the lateral-only authority
    assert rows(comb)[name][0] < CFG.eps                                        # refuted by sqrt(a_long^2 + a_lat^2) <= 8
    assert CCP.rc18_refuted_by_comb(json.loads(json.dumps(alat.to_json())), json.loads(json.dumps(comb.to_json()))) == [name]
    assert rows(comb)["bypass_left"][0] >= CFG.eps and comb.evidence_class == "R_REF"     # a feasible alternative exists
    dp = comb.traces[f"{name}/static_blocker"]["margin"]["dyn_peaks"]
    assert dp["a_comb"] > 8.0 and dp["a_lat"] < 8.0 and comb.geometry["spec"]["a_comb_max"] == 8.0


def test_rc18_requested_brake_9_alone_exceeds_authority():
    th = {"gap_m": 38.0, "v_mps": 18.0, "tau_s": 0.25}
    rec = run("RC18", th, b=9.0, spec=CP.rc18_spec(th, CFG, comb=True))
    assert rows(rec)["brake_hold"][0] < CFG.eps                                 # decel 9 > 8: outside the admissible set
    ok = run("RC18", th, b=9.0, spec=CP.rc18_spec(th, CFG, comb=False))
    assert rows(ok)["brake_hold"][0] >= CFG.eps                                 # same braking passes when only a_lat is limited


def test_a_long_smoothing_is_centred_and_length_preserving():
    x = np.array([0, 0, 10.0, 0, 0, 0, 0])
    s = H.smooth(x, 5)
    assert len(s) == len(x) and abs(s.sum() - 10.0) < 1e-9 and s[2] == 2.0


def test_script_helpers_median_and_contrast_on_synthetic_rows():
    r = {"policies": ["brake_hold", "immediate_lane_change", "staged_brake_0.4_change"], "R_raw": [["-inf"], [-0.1], [0.2]]}
    assert CCP.rc04_contrast(r) and CCP.median_key([3, 1, 2]) == 2 and CCP.median_key([]) is None
    r2 = {"policies": ["commit_lane_change", "abort_and_stabilize"], "R_raw": [[0.1, "-inf"], [0.1, 0.2]]}
    assert CCP.rc05_contrast(r2)


def test_accel_series_mask_excludes_whole_smoothing_window_and_vmin_overrides_default():
    n = 40
    speeds = np.linspace(10.0, 0.0, n)                         # crosses 5 m/s between index 19 and 20
    ax = np.full(n - 1, -6.0)
    ax[24:28] = -30.0                                          # wheel-lock spike below 5 m/s
    a_lat = np.zeros(n)
    acc = H.accel_series(speeds, ax, a_lat, 5.0)
    h = H.A_LONG_SMOOTH_TICKS // 2
    first_slow = int(np.argmax(speeds < 5.0))
    assert np.all(acc["a_comb"][first_slow - h:] == 0.0) and np.all(acc["a_long_masked"][first_slow - h:] == 0.0)   # within half a window of the crossing: excluded
    assert acc["a_comb"][first_slow - h - 1] > 0.0                                          # the last full-window tick is still counted
    assert acc["a_comb_unmasked"].max() > 20.0                                              # unmasked still sees the spike
    assert acc["a_comb"].max() <= 6.0 + 1e-9                                                # no leak of the spike into the counted ticks
    low = H.accel_series(speeds, ax, a_lat, 0.5)               # a spec vmin overrides the default
    assert low["a_comb"].max() > 20.0 and low["peaks"]["a_comb"]["v"] < 5.0


def test_harness_uses_spec_a_comb_vmin_and_records_peak_times():
    th = {"gap_m": 38.0, "v_mps": 18.0, "tau_s": 0.25}
    sp = CP.rc18_spec(th, CFG, comb=True)
    assert sp["a_comb_vmin"] == H.A_COMB_VMIN
    res = H.run_episode(KinematicPlant(b_max=6.2), sp, P.BrakeHold(), CP.CLEAN, CFG)
    pt = res.margin["dyn_peaks"]["peak_times"]
    assert pt["a_long"]["v"] >= H.A_COMB_VMIN and "a_comb_unmasked" in res.margin["dyn_peaks"]


def test_harness_spec_a_comb_vmin_override_changes_the_margin():
    th = {"gap_m": 38.0, "v_mps": 18.0, "tau_s": 0.25}
    out = {}
    for vmin in (0.0, 50.0):
        sp = dict(CP.rc18_spec(th, CFG, comb=True), a_comb_vmin=vmin)
        res = H.run_episode(KinematicPlant(b_max=9.0), sp, P.BrakeHold(), CP.CLEAN, CFG)
        out[vmin] = res.margin["parts"]["dyn"]
    assert out[0.0] < 0.0 <= out[50.0]       # vmin=0 counts the 9 m/s^2 braking (> 8); vmin=50 excludes every tick
    assert out[50.0] > out[0.0]


def test_rc04_path_length_lane_change_gives_the_staged_witness():
    """Time-defined lane changes cannot show RC04 (see cases/compositional.rc04_path_library); over a fixed path length the immediate
    change breaks a_lat_max at 28 m/s while braking first and then changing passes, and the gap is too short for brake_hold alone."""
    from families import mock_ctx
    ctx = mock_ctx()
    th = {"gap_m": 66.0, "v_mps": 28.0, "tau_s": 0.25, "b_mps2": 6.19}
    r = CP.run("RC04", lambda: ctx["factory"](th), th, M.DEFAULT_V2, "mock", "mock", library=CP.rc04_path_library(40.0))
    G = {p: row[0] for p, row in zip(r.policies, r.R_raw)}
    assert G["brake_hold"] < 0 and G["immediate_lane_change"] < 0
    assert max(g for p, g in G.items() if p.startswith("staged")) >= 0
    assert r.evidence_class == "R_REF"
