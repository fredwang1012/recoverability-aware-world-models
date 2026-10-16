"""Foundation cases (RC01/02/12/17) through the full harness on the KINEMATIC MOCK plant.  Verifies the machinery:
trackers, delay pipeline, observation feed, OBB margin, terminal hold and evidence classification reproduce the PDF's
analytic anchors after substituting the plant's own b and delay.  Not a CARLA result."""
import math

import pytest

import margin as M
from cases import corridor as CO
from sim.plant_kinematic import KinematicPlant

CFG = M.DEFAULT_V2
B = 6.0


def rec_for(gap, tau=0.25, v=12.0, prefix=0.0, b=B, full=True):
    theta = {"gap_m": gap, "v_mps": v, "tau_s": tau, "prefix_s": prefix}
    return CO.run(lambda: KinematicPlant(b_max=b), theta, CFG, "mock", "mock_replay", with_bypass=full)


def label(r):
    G = {p: r.R_raw[i][0] for i, p in enumerate(r.policies)}
    return r.evidence_class, G


def d_req(v, tau, b, c=CFG.reserve_m, prefix=0.0):
    return v * (tau + prefix) + v * v / (2 * b) + c


def test_rc01_witness_and_rc02_negative_through_harness():
    r = rec_for(18.0)
    assert r.evidence_class == "R_REF" and r.best_policy == "brake_hold"
    assert r.R_raw[0][0] == pytest.approx((3.0 - CFG.reserve_m) / CFG.q_clear, abs=1e-3)       # PDF: 3 m final clearance
    r = rec_for(12.0)                                                                           # contact
    assert r.evidence_class == "U_LIBRARY" and r.V is not None and r.V < 0
    r = rec_for(15.5)                                                                           # no contact, reserve violated
    assert r.evidence_class == "U_LIBRARY"
    G = r.R_raw[0][0]
    assert -0.2 < G < 0                                                                         # clearance 0.5 m < reserve 1 m


def test_gap_boundary_matches_analytic_d_required():
    v, tau = 12.0, 0.25
    lo, hi = 10.0, 25.0
    for _ in range(11):                                   # R_REF holds for LARGER gaps
        mid = 0.5 * (lo + hi)
        lo, hi = (lo, mid) if rec_for(mid, tau, v, full=False).evidence_class == "R_REF" else (mid, hi)
    assert 0.5 * (lo + hi) == pytest.approx(d_req(v, tau, B), abs=0.05)


def test_rc17_delay_boundary_one_tick():
    v, gap = 12.0, 18.0
    tau_star = (gap - v * v / (2 * B) - CFG.reserve_m) / v            # 0.4167 s
    k_star = math.floor(tau_star / 0.05)                               # largest admissible delay on the tick grid
    assert rec_for(gap, tau=k_star * 0.05, full=False).evidence_class == "R_REF"
    assert rec_for(gap, tau=(k_star + 1) * 0.05, full=False).evidence_class == "U_LIBRARY"


def test_rc12_prefix_boundary_and_feasible_set():
    v, tau, gap = 12.0, 0.25, 18.0
    tp_star = (gap - v * tau - v * v / (2 * B) - CFG.reserve_m) / v     # 0.1667 s
    assert rec_for(gap, prefix=0.10, full=False).evidence_class == "R_REF"
    assert rec_for(gap, prefix=0.25, full=False).evidence_class == "U_LIBRARY"
    assert tp_star == pytest.approx(0.1667, abs=1e-3)


def test_timing_is_logged_and_measured_response_matches_configured_delay():
    r = rec_for(18.0, tau=0.25)
    ts = r.timestamps["brake_hold/stationary_blocker_no_rear"]
    assert ts["configured_delay_s"] == pytest.approx(0.25)
    assert ts["brake_effective_t"] == pytest.approx(0.25, abs=1e-9)
    assert ts["measured_response_delay_s"] == pytest.approx(0.0, abs=0.06)     # decel appears the tick the command is effective
    assert ts["commands"][0]["decision_t"] == 0.0 and ts["commands"][0]["effective_t"] == pytest.approx(0.25)


def test_deterministic_repeat():
    a, b = rec_for(16.5), rec_for(16.5)
    assert a.R_raw == b.R_raw and a.checksum == b.checksum


def test_record_validates():
    import json
    import validate_records as VR
    r = json.loads(json.dumps(rec_for(18.0).to_json()))
    assert VR.check_record(r) == []
