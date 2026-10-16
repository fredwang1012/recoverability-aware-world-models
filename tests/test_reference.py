import math, os, sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "mining"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "mining", "reference"))
import corridor as C
import info_tree as T
from contract import classify, R_REF, U_LIBRARY, U_INFORMATION, UNRESOLVED


def test_pdf_worked_example():
    v, tau, b, c = 12.0, 0.25, 6.0, 1.0
    assert abs(C.d_stop(v, tau, b) - 15.0) < 1e-12
    assert abs(C.d_stop(v, tau, b) + c - 16.0) < 1e-12
    assert C.label(v, tau, b, 18.0, c) == "R_REF_witness"
    assert C.label(v, tau, b, 15.5, c) == "no_contact_but_reserve_violated"
    assert C.label(v, tau, b, 12.0, c) == "contact"
    G, info = C.margin(v, tau, b, 18.0, c)
    assert abs(info["final_clearance"] - 3.0) < 2e-2           # the PDF's "3 m final clearance"


def test_certificate_is_not_a_witness():
    # a comfortable braking value must not be used to certify impossibility: the certificate needs b_max
    assert C.model_certificate(12.0, 12.0, 0.25, 6.0, 1.0) == "collision_inevitable_in_model"
    assert C.model_certificate(12.0, 12.0, 0.25, 10.0, 1.0) != "collision_inevitable_in_model"   # larger true b_max: no proof


def _bisect(f, lo, hi, n=60):
    for _ in range(n):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if f(mid) else (lo, mid)
    return 0.5 * (lo + hi)


def test_boundaries_match_numeric_simulation():
    v, tau, b, c, gap = 12.0, 0.25, 6.0, 1.0, 18.0
    ok = lambda **kw: C.label(v, kw.get("tau", tau), b, gap, c, prefix=kw.get("prefix", 0.0)) == "R_REF_witness"
    tp = _bisect(lambda x: ok(prefix=x), 0.0, 1.0, 30)                      # RC12 largest admissible prefix
    assert abs(tp - (gap - C.d_stop(v, tau, b) - c) / v) < 5e-3
    ts = _bisect(lambda x: ok(tau=x), 0.0, 1.0, 30)                         # RC17 largest admissible delay
    assert abs(ts - (gap - v * v / (2 * b) - c) / v) < 5e-3


def test_monotone_in_delay_for_corridor():
    v, b, c, gap = 12.0, 6.0, 1.0, 18.0
    taus = np.linspace(0, 0.8, 17)
    res = [C.label(v, t, b, gap, c) == "R_REF_witness" for t in taus]
    assert res == sorted(res, reverse=True)                                  # once lost, never regained


def test_classify_duality_and_hindsight_inequality():
    rng = np.random.default_rng(0)
    for _ in range(300):
        R = rng.normal(size=(rng.integers(1, 6), rng.integers(1, 5)))
        o = classify(R)
        rec = (R.min(axis=1) >= 0).any()                                     # exists pi forall w
        unrec = (R.min(axis=0) < 0).any() and (R.min(axis=1) < 0).all()      # forall pi exists w
        assert rec == (o["evidence"] == R_REF) and rec != (R.min(axis=1) < 0).all()
        assert o["V"] <= o["V_hind"] + 1e-12                                 # max-min <= min-max
        if o["evidence"] == U_INFORMATION:
            assert (R.max(axis=0) >= 0).all() and (R.min(axis=1) < 0).all()


def test_masked_entries_never_become_negative_labels():
    R = np.array([[1.0, np.nan], [np.nan, np.nan]])
    assert classify(R)["evidence"] == UNRESOLVED                            # missing world => no witness, no full refutation
    R = np.array([[-1.0, np.nan], [-0.5, np.nan]])
    assert classify(R)["evidence"] == U_LIBRARY                             # one resolved violating world refutes a policy
    R = np.array([[-1.0, 1.0], [1.0, -1.0]])
    assert classify(R)["evidence"] == U_INFORMATION                         # each world solvable, no common policy


def test_info_tree_reveal_threshold_is_analytic():
    D, v, tau, b, T_lat, tau_plan, reserve = 20.0, 15.0, 0.25, 6.0, 1.2, 0.1, 0.05
    t_arr_max = max(T.t_arrival(D, v, tau, b, p) for p in np.arange(0, 2.55, 0.05) if math.isfinite(T.t_arrival(D, v, tau, b, p)))
    t_star = t_arr_max - tau_plan - tau - T_lat - reserve
    assert 0.2 < t_star < 0.3
    before = T.label_scene(D, t_star - 0.03)["evidence"]
    after = T.label_scene(D, t_star + 0.03)["evidence"]
    assert (before, after) == (R_REF, U_INFORMATION)


def test_info_tree_physically_impossible_is_not_information_infeasible():
    assert T.label_scene(10.0, 0.0)["evidence"] == U_LIBRARY               # no world has a witness
    assert T.label_scene(40.0, 5.0)["evidence"] == R_REF                   # a common stop succeeds regardless of reveal time
