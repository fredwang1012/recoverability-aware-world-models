"""Partial-observation cases (RC10 / RC11 / RC14) through the harness on the KINEMATIC MOCK plant.  Not a CARLA result.
Labels by evidence only; the identical-history and leakage checks are structural."""
import dataclasses
import json
import os
import sys

import numpy as np

import margin as M
import validate_records as VR
from cases import occlusion as OC
from reference import info_tree as IT
from sim import harness as H
from sim import policies as P
from sim.plant_kinematic import KinematicPlant

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import carla_occlusion as CO

CFG = M.DEFAULT_V2
B_MOCK, TAU = 6.2, 0.25
TH = {"gap_m": 19.0, "v_mps": 14.0, "tau_s": TAU}


def plant():
    return KinematicPlant(b_max=B_MOCK)


def th(t_reveal, **kw):
    return {**TH, "t_reveal_s": t_reveal, **kw}


def run(case, theta, **kw):
    rec = OC.run(case, plant, theta, CFG, "mock", "mock_replay", **kw)
    assert VR.check_record(json.loads(json.dumps(rec.to_json()))) == []
    return rec


def rows(rec):
    return {p: rec.R_raw[i] for i, p in enumerate(rec.policies)}


class Spy(P.BrakeHold):
    """Records every history the policy is given (it is the policy's ONLY input)."""
    name = "spy"

    def reset(self):
        super().reset(); self.seen = []

    def decide(self, history, t):
        self.seen.append((t, list(history)))
        return super().decide(history, t)


def spy_histories(theta, world_list):
    out = []
    for w in world_list:
        spy = Spy()
        H.run_episode(plant(), OC.spec(theta, CFG), spy, w, CFG)
        out.append(spy.seen)
    return out


# ----------------------------------------------------------------------------- observation contract
def test_identical_delivered_histories_before_reveal_in_the_two_worlds():
    t_rev = 0.6
    theta = th(t_rev)
    ha, hb = spy_histories(theta, OC.worlds(t_rev, theta))
    n_pre = [i for i, (t, h) in enumerate(ha) if t < t_rev - 1e-9]
    assert len(n_pre) >= 10 and len(ha) >= len(n_pre) + 2
    for i in n_pre:
        assert ha[i][1] == hb[i][1]                                     # the delivered Obs sequences are equal, element by element
        assert all(o.objs == [] for o in ha[i][1])                      # and the hidden vehicle is simply absent
    tr = [i for i, (t, h) in enumerate(ha) if abs(t - t_rev) < 1e-9][0]
    assert ha[tr][1] != hb[tr][1]                                       # non-vacuous: the reveal tick DOES separate the worlds
    assert ha[tr][1][-1].objs[0]["dlat"] == -OC.LANE_W and hb[tr][1][-1].objs[0]["dlat"] == 0.0
    assert ha[tr][1][-1].objs[0]["dx"] == hb[tr][1][-1].objs[0]["dx"]   # same range, only the lane differs


def test_no_hidden_identity_in_policy_inputs_before_the_reveal():
    t_rev = 0.8
    theta = th(t_rev)
    ws = OC.worlds(t_rev, theta)
    sp = OC.spec(theta, CFG)
    secrets = [OC.HID, "hid_left_lane", "hid_own_lane", "audi", "a2", "static", "colour", "color", "blueprint", "spawn"]
    x_hid = theta["gap_m"] + OC.HID_HL
    for w, hist in zip(ws, spy_histories(theta, ws)):
        for t, h in hist:
            if t >= t_rev - 1e-9:
                continue
            for ob in h:
                assert type(ob).__name__ == "Obs" and {f.name for f in dataclasses.fields(ob)} == \
                    {"t_capture", "t_available", "v", "lane_w", "objs", "ego_hl"}
                blob = json.dumps(dataclasses.asdict(ob)).lower()
                assert not any(s.lower() in blob for s in secrets)
                assert ob.objs == []
                assert not any(f"{x}" in blob for x in (x_hid, x_hid - OC.HID_HL, OC.LEFT_LAT))
    # after the reveal the object is delivered ANONYMOUSLY: relative geometry only, no id / name / mode keys
    h = spy_histories(theta, ws[:1])[0]
    late = [o for t, hh in h if t > t_rev + 0.1 for o in hh[-1].objs]
    assert late and all(set(o) == {"dx", "dlat", "dv", "hl", "hw"} for o in late)
    # the policy API gives it nothing else
    import inspect
    assert list(inspect.signature(P.SharedSlowdownThenBranch.decide).parameters) == ["self", "history", "t"]


def test_world_spawn_override_places_the_vehicle_and_is_recorded():
    theta = th(0.0)
    rec = run("RC10", theta)
    assert rec.worlds == ["hid_left_lane", "hid_own_lane"]
    ws = {w["name"]: w for w in rec.geometry["world_spec"]}
    assert ws["hid_left_lane"]["spawn"]["hid"]["lat"] == -OC.LANE_W and ws["hid_own_lane"]["spawn"]["hid"]["lat"] == 0.0
    assert rec.geometry["spec"]["actors"]["hid"]["static"] is True


# ----------------------------------------------------------------------------- RC10 / RC11 labels
def test_rc10_early_reveal_is_r_ref_only_through_the_observation_branch():
    rec = run("RC10", th(0.2))
    r = rows(rec)
    assert rec.evidence_class == "R_REF"
    assert r["shared_slowdown_then_branch"][0] >= CFG.eps and r["shared_slowdown_then_branch"][1] >= CFG.eps
    assert r["stop_in_place"][1] < CFG.eps                              # cannot stop (D < D_stop): refuted in the own-lane world
    assert r["fixed_left"][0] < CFG.eps and r["fixed_left"][1] >= CFG.eps   # the blind left change collides with the left-lane vehicle
    assert r["fixed_right"][1] < CFG.eps and r["fixed_right"][0] >= CFG.eps  # the blind stay collides with the own-lane vehicle


def test_rc11_late_reveal_is_u_information():
    rec = run("RC11", th(1.0))
    r = rows(rec)
    assert rec.evidence_class == "U_INFORMATION"
    assert all(min(row) < CFG.eps for row in r.values())               # every causal policy refuted in at least one world
    assert r["shared_slowdown_then_branch"][1] < CFG.eps               # the branch is too late in the own-lane world
    assert max(r["fixed_left"]) >= CFG.eps and max(r["stop_in_place"]) >= CFG.eps   # each world has its own witness (hindsight feasible)


def test_reveal_sweep_is_a_single_transition_r_ref_to_u_information():
    ev = [run("RC10", th(round(t, 2))).evidence_class for t in np.arange(0.0, 1.21, 0.1)]
    first_u = ev.index("U_INFORMATION")
    assert ev[:first_u] == ["R_REF"] * first_u and set(ev[first_u:]) == {"U_INFORMATION"} and 0 < first_u < len(ev)


def test_common_stop_prefix_means_never_u_information():
    for t in (0.0, 0.6, 1.2):
        rec = run("RC11", th(t, gap_m=26.0))                            # D > stopping distance + reserve: a common stop survives both worlds
        assert rec.evidence_class == "R_REF" and all(g >= CFG.eps for g in rows(rec)["stop_in_place"])


def test_info_tree_reconciliation_and_threshold_crosscheck():
    # (1) closed-form t_obs* vs the discrete tree on the two-lane geometry and vs the symmetric two-lateral-corridor tree
    D, v, b, tau, T_lat = 19.0, 14.0, B_MOCK, TAU, 1.3
    ts = IT.t_obs_star(D, v, tau, b, T_lat, tau_plan=0.0, reserve=0.0)
    sweep = np.round(np.arange(0.0, 1.2, 0.01), 2)
    kw = dict(v=v, tau=tau, b=b, T_lat=T_lat, tau_plan=0.0, reserve=0.0, p_grid=np.arange(0.0, 3.01, 0.01), ts_grid=np.arange(0.0, 0.05, 0.05))
    cross = lambda two: max(t for t in sweep if IT.label_scene(D, t, two_lane=two, **kw)["evidence"] == "R_REF")
    assert abs(cross(True) - ts) <= 0.02 and abs(cross(False) - ts) <= 0.02          # same threshold: the lateral world is binding in both
    assert IT.label_scene(D, ts + 0.05, two_lane=True, **kw)["evidence"] == "U_INFORMATION"
    # (2) a common stop (D >= D_stop) means there is no finite threshold
    assert IT.t_obs_star(26.0, v, tau, b, T_lat) is None


def test_mock_threshold_matches_analytic_within_declared_tolerance():
    for D in (19.0, 20.0):
        recs = {t: {"evidence_class": run("RC10", th(round(t, 2), gap_m=D)).evidence_class} for t in np.arange(0.0, 1.01, 0.1)}
        tr = CO.transition(recs)
        ts = CO.analytic_threshold(D, 14.0, b=B_MOCK, tau=TAU)
        cc = CO.crosscheck(tr, ts)
        assert tr is not None and cc["ok"], (D, tr, ts, cc)


def test_script_pure_helpers():
    assert abs(CO.t_lat() - 1.3875) < 1e-3
    assert CO.analytic_threshold(26.0, 14.0) is None and CO.analytic_threshold(19.0, 14.0) is not None      # common stop => no gate
    assert CO.transition({0.0: {"evidence_class": "R_REF"}, 0.1: {"evidence_class": "R_REF"}, 0.2: {"evidence_class": "U_INFORMATION"}}) == (0.1, 0.2)
    assert CO.transition({0.0: {"evidence_class": "R_REF"}, 0.1: {"evidence_class": "R_REF"}}) is None
    assert CO.transition({0.0: {"evidence_class": "U_INFORMATION"}, 0.1: {"evidence_class": "R_REF"}}) is None
    log = [{"ego": (-2.3 + 0.7 * k, -0.1 * k, 14.0 - k), "hid": (21.3, 0.0, 0.0)} for k in range(40)]
    a = CO.arrival_stats(log)
    assert a["t_arrival"] == next(k for k in range(40) if -2.3 + 0.7 * k + 2.3 >= 21.3 - 2.3) * 0.05


# ----------------------------------------------------------------------------- RC14
def test_rc14_identical_current_frame_different_earlier_observations():
    t_f = 0.6
    theta = {**TH, "t_forget_s": t_f}
    ha, hb = spy_histories(theta, OC.rc14_worlds(t_f, theta))
    i = [k for k, (t, h) in enumerate(ha) if t >= t_f + 0.3][0]
    assert ha[i][1][-1] == hb[i][1][-1] and ha[i][1][-1].objs == []         # identical current frame (hid occluded again)
    assert ha[i][1] != hb[i][1]                                             # different earlier observations
    assert ha[i][1][0].objs != hb[i][1][0].objs and ha[i][1][0].objs[0]["dlat"] != hb[i][1][0].objs[0]["dlat"]


def test_rc14_history_conditioned_profiles_differ_and_memory_is_needed():
    rec = run("RC14", {**TH, "t_forget_s": 0.6})
    r = rows(rec)
    col = lambda w: [r[p][w] for p in rec.policies]
    assert rec.worlds == ["seen_left_then_occluded", "seen_own_then_occluded"]
    assert col(0) != col(1)                                                 # different profiles for the same current frame
    assert all(g >= CFG.eps for g in r["shared_slowdown_then_branch"]) and rec.evidence_class == "R_REF"
    assert min(r["branch_current_frame_only"]) < CFG.eps                    # the current-frame-only policy forgets the vehicle and fails
