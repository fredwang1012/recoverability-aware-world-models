"""T0 tests: margin unit tests, analytic BTN cross-check, determinism, algorithm contract."""
import numpy as np
import pytest

import core
from algos import REGISTRY
from core import EPS, MockBackend, actor_margin, decode

E = {"x": 0.0, "lat": 0.0, "v": 20.0, "hl": 2.4, "hw": 0.95}


def O(x, lat, v, hl=2.4, hw=0.95):
    return {"x": x, "lat": lat, "v": v, "hl": hl, "hw": hw}


# ---------------------------------------------------------------- 1. actor_margin
def test_collision_returns_none():
    assert actor_margin(E, O(3.0, 0.0, 0.0)) is None


def test_other_lane_is_one():
    assert actor_margin(E, O(0.0, -core.LANE_W, 25.0)) == 1.0


def test_stopped_behind_obstacle_gap_over_scale():
    e = dict(E, v=0.0)
    gap = 10.0
    m = actor_margin(e, O(gap + 4.8, 0.0, 0.0))
    assert m == pytest.approx(gap / core.MARGIN_SCALE)


def test_squeeze_alongside_negative():
    # same longitudinal position, lateral gap positive but below LAT_CLEAR
    lat = 2 * 0.95 + 0.2
    m = actor_margin(E, O(0.0, lat, 20.0))
    assert m is not None and m < 0


# ---------------------------------------------------------------- 2. analytic BTN cross-check
class BrakeOnly(core.Policy):
    name = "brake_only"

    def _act(self, obs):
        return core.BRAKE


def _btn_scene(rng):
    th = decode(rng.random(core.D))
    th["adjacent_offset"] = -200.0          # keep the left-lane car out of the picture
    th["adjacent_speed"] = 15.0
    return th


def _collision_free_analytic(th):
    v, a, d = th["ego_speed"], th["brake_limit"], th["reaction_delay"]
    return v * v / (2 * a) + v * d < th["obstacle_gap"] - 4.8


def _margin_analytic(th, H=core.HEADWAY_S):
    """Exact G>=0 criterion for brake-only: headway margin min_u G0 + u^2/(2a) - u*H over u in [0, v]."""
    v, a, d = th["ego_speed"], th["brake_limit"], th["reaction_delay"]
    # remaining long gap at speed u while braking: gap_eff - (v^2-u^2)/(2a); margin numerator = that - u*H.
    # It is convex in u, minimised at u* = a*H (clipped to [0, v]); at u=0 it equals the stopped gap.
    u_star = a * H
    g_end = th["obstacle_gap"] - 4.8 - v * v / (2 * a) - v * d
    gap_eff = th["obstacle_gap"] - 4.8 - v * d
    u = np.clip(u_star, 0.0, v)
    g_min = gap_eff - (v * v - u * u) / (2 * a) - u * H
    return min(g_min, g_end) >= 0


def test_btn_analytic_agreement(capsys):
    rng = np.random.default_rng(0)
    be, n = MockBackend(), 300
    dis_col, dis_marg = [], []
    for _ in range(n):
        th = _btn_scene(rng)
        g = min(core.margin(be.rollout(th, w, BrakeOnly())) for w in core.WORLDS)
        oracle_rec = g >= EPS
        if oracle_rec != _collision_free_analytic(th):
            dis_col.append((th, g))
        if oracle_rec != _margin_analytic(th):
            dis_marg.append((th, g))
    agree_col, agree_marg = 1 - len(dis_col) / n, 1 - len(dis_marg) / n
    with capsys.disabled():
        print(f"\nBTN agreement (collision-free criterion v^2/2a+v*d < gap-L): {agree_col:.1%} ({len(dis_col)} disagree)")
        print(f"BTN agreement (exact G>=0 headway-aware criterion):          {agree_marg:.1%} ({len(dis_marg)} disagree)")
        for th, g in dis_marg[:5]:
            print("  disagreement:", {k: round(v, 2) for k, v in th.items()}, "G=%.3f" % g)
    assert agree_marg >= 0.99


# ---------------------------------------------------------------- 3. determinism
def test_mock_deterministic():
    th = decode(np.full(core.D, 0.4))
    a = MockBackend().rollout(th, "idm_reactive", core.LeftIfGapElseBrake())
    b = MockBackend().rollout(th, "idm_reactive", core.LeftIfGapElseBrake())
    assert a["ego"] == b["ego"] and a["others"] == b["others"]


# ---------------------------------------------------------------- 4/5. algorithm contract
@pytest.mark.parametrize("name", list(REGISTRY))
def test_algorithm_smoke(name):
    rng = np.random.default_rng(1)
    algo, be, rows = REGISTRY[name](rng), MockBackend(), []
    while len(rows) < 24:
        U = np.clip(algo.ask(8), 0, 1)
        assert U.ndim == 2 and U.shape[1] == core.D and U.min() >= 0 and U.max() <= 1
        new = []
        for u in U[:8]:
            r = core.oracle(be, decode(u)); r["u"] = [float(x) for x in u]; new.append(r)
        algo.tell(U[:8], new)
        rows += new
    assert len(rows) >= 24


def test_ask_shape():
    for name in REGISTRY:
        algo = REGISTRY[name](np.random.default_rng(2))
        for n in (1, 5):
            U = np.asarray(algo.ask(n))
            assert U.shape[1] == core.D and 1 <= len(U) <= max(n, 12)
            assert (U >= 0).all() and (U <= 1).all()


def test_bisect_pairs():
    rng = np.random.default_rng(3)
    algo, be, rows = REGISTRY["bisect"](rng), MockBackend(), []
    for _ in range(25):
        U = np.clip(algo.ask(8), 0, 1)
        new = []
        for u in U:
            r = core.oracle(be, decode(u)); r["u"] = [float(x) for x in u]; new.append(r)
        algo.tell(U, new)
    assert algo.pairs, "bisect produced no pairs"
    for p in algo.pairs:
        lo, hi = np.array(p["u_unrecoverable"]), np.array(p["u_recoverable"])
        diff = np.flatnonzero(~np.isclose(lo, hi))
        assert list(diff) == [p["varied_index"]]
        assert core.oracle(be, decode(lo))["V"] < EPS <= core.oracle(be, decode(hi))["V"]


# ---------------------------------------------------------------- policy isolation
def test_policies_see_only_observation():
    import inspect
    for P in core.LIBRARY + [core.TTCBrakePlanner]:
        assert list(inspect.signature(P.act).parameters) == ["self", "obs"]
    seen = {}

    class Spy(core.Policy):
        def _act(self, obs):
            seen.update(obs); return core.KEEP

    th = decode(np.full(core.D, 0.5))
    MockBackend().rollout(th, "accelerating", Spy())
    assert set(seen) == {"t", "v", "lane_w", "objs"}
    assert all(set(o) == {"dx", "dlat", "dv"} for o in seen["objs"])


# ---------------------------------------------------------------- T1: MarginConfig regression
def test_default_margin_config_matches_golden():
    import json, os
    gold = json.load(open(os.path.join(os.path.dirname(__file__), "golden_oracle.json")))
    for u, v, cat in gold:
        r = core.oracle(MockBackend(), decode(np.array(u)), full=True)
        assert r["V"] == pytest.approx(v, abs=1e-12) and r["category"] == cat


def test_margin_config_changes_margin():
    th = decode(np.full(core.D, 0.5))
    tr = MockBackend().rollout(th, "constant", core.Keep())
    from dataclasses import replace
    a = core.margin(tr)
    b = core.margin(tr, replace(core.DEFAULT_CFG, offroad_speed_scale=1.0, collision_speed_scale=1.0))
    assert a != b or a >= 0


def test_vtransform_preserves_sign():
    from algos import apply_vtransform
    for k in ("none", "clip", "tanh"):
        for v in (-1.0, -0.1, 0.0, 0.2, 1.0):
            assert np.sign(apply_vtransform(k, v)) == np.sign(v)
