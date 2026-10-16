"""A1: the PDF-faithful margin (margin.py) reproduces the section 4.1 worked example and basic geometry."""
import math

import numpy as np
import pytest

import margin as M
from reference import corridor as C
from reference import corridor_margin as CM


def test_obb_signed_distance_basic():
    A = M.obb_corners(0, 0, 0, 2.0, 1.0)
    assert M.signed_distance(A, M.obb_corners(10, 0, 0, 2.0, 1.0)) == pytest.approx(6.0)        # 10 - 2 - 2
    assert M.signed_distance(A, M.obb_corners(3, 0, 0, 2.0, 1.0)) == pytest.approx(-1.0)        # overlap 1 m
    assert M.signed_distance(A, M.obb_corners(0, 5, 0, 2.0, 1.0)) == pytest.approx(3.0)         # lateral gap
    # rotated box: corner-to-edge distance
    B = M.obb_corners(6, 3, math.pi / 4, 2.0, 1.0)
    assert M.signed_distance(A, B) == pytest.approx(M.signed_distance(B, A))
    assert M.signed_distance(A, B) > 0


def test_point_clear_but_body_overlaps():
    """RC16 mechanism: centre-line clears a post, the body does not."""
    ego = M.obb_corners(0, 0, 0, 2.3, 0.95)
    post = M.obb_corners(0, 1.0, 0, 0.1, 0.1)             # post face at 0.9 m, ego half-width 0.95 m
    assert 1.0 - 0.1 > 0                                    # centre line (lat 0) is clear of the post
    assert M.signed_distance(ego, post) < 0                 # body penetrates by 0.05 m


def test_pdf_worked_example_through_margin_path():
    v, tau, b, c = 12.0, 0.25, 6.0, 1.0
    assert C.d_stop(v, tau, b) == pytest.approx(15.0) and C.d_stop(v, tau, b) + c == pytest.approx(16.0)
    assert CM.label(v, tau, b, 18.0, c) == "R_REF_witness"
    assert CM.label(v, tau, b, 15.5, c) == "no_contact_but_reserve_violated"
    assert CM.label(v, tau, b, 12.0, c) == "contact"
    G, o = CM.score(v, tau, b, 18.0, c)
    assert o["clear_m"] == pytest.approx(3.0, abs=1e-6)      # the PDF's 3 m final clearance
    assert G == pytest.approx((3.0 - c) / M.DEFAULT_V2.q_clear, abs=1e-6)
    assert o["hold_ok"] and o["terminal_ok"]


def test_boolean_terminal_failure_is_minus_inf_but_search_grade_finite():
    tr, speeds = CM.trace(12.0, 0.25, 6.0, 18.0)
    out = M.compute(tr, terminal_ok=False)
    assert out["G"] == -math.inf and math.isfinite(out["G_search"]) and out["G_search"] < 0
    assert out["G_raw"] > 0                                  # path margin kept separately


def test_boundaries_through_margin_path_match_analytic():
    v, tau, b, c, gap = 12.0, 0.25, 6.0, 1.0, 18.0
    def bis(f, lo, hi, n=22):
        for _ in range(n):
            mid = 0.5 * (lo + hi); lo, hi = (mid, hi) if f(mid) else (lo, mid)
        return 0.5 * (lo + hi)
    ok = lambda **kw: CM.label(v, kw.get("tau", tau), b, gap, c, prefix=kw.get("prefix", 0.0), dt=0.02) == "R_REF_witness"
    assert bis(lambda x: ok(prefix=x), 0.0, 1.0) == pytest.approx((gap - C.d_stop(v, tau, b) - c) / v, abs=6e-3)   # RC12
    assert bis(lambda x: ok(tau=x), 0.0, 1.0) == pytest.approx((gap - v * v / (2 * b) - c) / v, abs=6e-3)         # RC17


def test_swept_check_catches_between_tick_tunnelling():
    """Fast ego, coarse ticks: poses at ticks straddle a thin post; the swept check must still see the overlap."""
    dt, hl, hw = 0.5, 2.0, 1.0
    ego = [(-6.0, 0.0, 0.0), (6.0, 0.0, 0.0)]            # jumps 12 m between ticks, passes x=0
    post = [(0.0, 0.0, 0.0)] * 2
    tr = {"dt": dt, "ego": ego, "others": {"post": post}, "half": {"ego": (hl, hw), "post": (0.1, 0.1)}}
    coarse = M.compute(tr, M.MarginConfigV2(swept_substeps=1))
    swept = M.compute(tr, M.MarginConfigV2(swept_substeps=20))
    assert coarse["min_clearance_m"]["post"] > 0 > swept["min_clearance_m"]["post"]


def test_road_margin_sign():
    tr = {"dt": 0.1, "ego": [(0, 0, 0)] * 3, "others": {}, "half": {"ego": (2.3, 0.95)}}
    assert M.compute(tr, road=(-1.75, 1.75))["parts"]["road"] > 0              # 1.75 - 0.95 = 0.8 m inside
    assert M.compute(tr, road=(-0.5, 0.5))["parts"]["road"] < 0                # box sticks out of the road
