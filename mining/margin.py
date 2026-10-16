"""
margin.py - label-grade margin G from the PDF (section 2.1, 3), backend independent.

    G(pi, w; h) = min{ min_{s in [t, t+H)} min_i m_i(x_s)/q_i ,  m_term/q_term }

Inputs are a trace of oriented-box poses (ego + other actors) on a fixed tick grid, in a road frame
(x forward, lat positive to the RIGHT, yaw in radians, 0 = along +x, positive = toward +lat).  Everything the margin
needs comes from poses, so the same code scores the analytic corridor, the numpy mock and CARLA
(CARLA: BoundingBox.get_world_vertices gives the same polygons).

Hard constraints (all signed, + = accepted, positive reserve subtracted so 0 means 'meets the reserve'):
    clearance   : signed distance between oriented boxes (negative = penetration) minus reserve_m, per actor, per tick,
                  refined between ticks (swept check) by interpolating poses
    road        : distance from every ego corner to the permitted road edges
    dynamics    : optional signed margins supplied by the backend (|a_lat|, steering rate, ...) as (value, limit) pairs
Terminal continuation is a Boolean computed by the caller (e.g. protected hold over the post-maneuver interval) and stored
separately: if it fails, G = -inf (label-grade).  Route progress / comfort are never part of G.

Two scalars are returned and must never be confused:
    G      label grade  (may be -inf)             -> used for R, V, evidence
    G_srch search grade (finite, clipped to +-1)  -> used ONLY by the mining algorithms (they need a gradient)
"""
import math
from dataclasses import dataclass, asdict

import numpy as np


@dataclass(frozen=True)
class MarginConfigV2:
    reserve_m: float = 1.0          # m, required residual clearance; margin 0 == just meeting it
    q_clear: float = 5.0            # m, normalisation of clearance margins
    q_road: float = 1.0             # m, normalisation of road-edge margins
    q_dyn: float = 1.0              # normalisation of dynamic-limit margins ((limit - |value|)/limit already unit-free)
    q_term: float = 1.0             # normalisation of the terminal margin (only used if a terminal margin is supplied)
    eps: float = 0.0                # label threshold
    horizon_s: float = 8.0          # s, evaluation horizon including emergency, stabilisation and post-maneuver
    hold_s: float = 4.0             # s, protected-hold duration (starting test value, not indefinite safety)
    post_maneuver_s: float = 4.0    # s, interval checked after the maneuver finishes
    swept_substeps: int = 20        # max pose-interpolation substeps between ticks
    swept_step_m: float = 0.25      # m, adaptive: one substep per this much relative motion per tick
    search_clip: float = 1.0        # G_srch = clip(G_raw, -clip, +clip)
    stop_speed: float = 0.1         # m/s, 'stopped' threshold for protected hold


DEFAULT_V2 = MarginConfigV2()


# ----------------------------------------------------------------------------- geometry
def obb_corners(x, y, yaw, hl, hw):
    """Corners (4,2) of an oriented box: centre (x, y), heading yaw, half length hl (along heading), half width hw."""
    c, s = math.cos(yaw), math.sin(yaw)
    fwd, side = np.array([c, s]), np.array([-s, c])
    p = np.array([x, y], float)
    return np.array([p + hl * fwd + hw * side, p + hl * fwd - hw * side,
                     p - hl * fwd - hw * side, p - hl * fwd + hw * side])


def _axes(poly):
    e = np.roll(poly, -1, axis=0) - poly
    n = np.stack([-e[:, 1], e[:, 0]], axis=1)
    return n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)


def _seg_seg_dist(p, q, r, s):
    """Minimum distance between segments pq and rs (2-D)."""
    def pt_seg(a, b, c):
        d = b - a
        t = 0.0 if not d.any() else float(np.clip(np.dot(c - a, d) / np.dot(d, d), 0.0, 1.0))
        return float(np.linalg.norm(c - (a + t * d)))
    d1, d2 = q - p, s - r
    den = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(den) > 1e-12:                       # possible proper crossing
        t = ((r[0] - p[0]) * d2[1] - (r[1] - p[1]) * d2[0]) / den
        u = ((r[0] - p[0]) * d1[1] - (r[1] - p[1]) * d1[0]) / den
        if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0:
            return 0.0
    return min(pt_seg(p, q, r), pt_seg(p, q, s), pt_seg(r, s, p), pt_seg(r, s, q))


def signed_distance(A, B):
    """Signed distance between convex polygons A, B (n,2): >0 separation distance, <0 minus penetration depth."""
    depth = math.inf
    for ax in np.vstack([_axes(A), _axes(B)]):
        pa, pb = A @ ax, B @ ax
        ov = min(pa.max(), pb.max()) - max(pa.min(), pb.min())
        if ov <= 0:                            # separating axis exists -> disjoint
            best = math.inf
            for i in range(len(A)):
                for j in range(len(B)):
                    best = min(best, _seg_seg_dist(A[i], A[(i + 1) % len(A)], B[j], B[(j + 1) % len(B)]))
            return best
        depth = min(depth, ov)
    return -depth


def _lerp_pose(p0, p1, a):
    yaw = p0[2] + a * ((p1[2] - p0[2] + math.pi) % (2 * math.pi) - math.pi)
    return (p0[0] + a * (p1[0] - p0[0]), p0[1] + a * (p1[1] - p0[1]), yaw)


# ----------------------------------------------------------------------------- the margin
def compute(trace, cfg=DEFAULT_V2, terminal_ok=None, road=None, dyn=None):
    """Label-grade and search-grade margins for ONE (policy, world) rollout.

    trace = {"dt": float,
             "ego":    [(x, y, yaw), ...]                    one pose per tick
             "others": {name: [(x, y, yaw), ...]},           same length
             "half":   {"ego": (hl, hw), name: (hl, hw)}}
    road  = (lat_min, lat_max) permitted lateral interval in the road frame, or None (no road constraint)
    dyn   = [(name, value_series, limit)] signed margin (limit - |v|)/limit per tick, or None
    terminal_ok = Boolean from the caller's terminal test (None = not evaluated -> treated as unresolved by caller)

    Returns dict(G, G_raw, G_search, path_margin, terminal_ok, parts{clearance, road, dyn}, argmin{...}).
    G_raw ignores the terminal Boolean; G = G_raw if terminal_ok else -inf."""
    ego, others, half = trace["ego"], trace["others"], trace["half"]
    hl, hw = half["ego"]
    n = len(ego)
    far = 4.0 * cfg.q_clear + cfg.reserve_m           # clearance saturates here: far actors never set the minimum
    rad = {name: math.hypot(*half[name]) for name in others}
    r_ego = math.hypot(hl, hw)
    clear = {name: math.inf for name in others}
    when = {name: 0 for name in others}
    for k in range(n):
        last = k == n - 1
        for name, poses in others.items():
            o0, e0 = poses[k], ego[k]
            gap_c = math.hypot(o0[0] - e0[0], o0[1] - e0[1]) - r_ego - rad[name]
            move = 0.0 if last else (math.hypot(ego[k + 1][0] - e0[0], ego[k + 1][1] - e0[1])
                                     + math.hypot(poses[k + 1][0] - o0[0], poses[k + 1][1] - o0[1]))
            if gap_c - move > far:                       # even after relative motion this tick it stays far
                clear[name] = min(clear[name], far)
                continue
            k_sub = 1 if last else int(min(max(cfg.swept_substeps, 1), max(1, math.ceil(move / cfg.swept_step_m))))
            for i in range(k_sub):
                a = i / k_sub
                e = e0 if a == 0.0 else _lerp_pose(e0, ego[k + 1], a)
                o = o0 if a == 0.0 else _lerp_pose(o0, poses[k + 1], a)
                d = signed_distance(obb_corners(e[0], e[1], e[2], hl, hw), obb_corners(o[0], o[1], o[2], half[name][0], half[name][1]))
                if d < clear[name]:
                    clear[name], when[name] = d, k
    m_clear = {name: (d - cfg.reserve_m) / cfg.q_clear for name, d in clear.items()}

    m_road = math.inf
    if road is not None:
        lo, hi = road
        for k in range(n):
            E = obb_corners(ego[k][0], ego[k][1], ego[k][2], hl, hw)
            m_road = min(m_road, (E[:, 1].min() - lo) / cfg.q_road, (hi - E[:, 1].max()) / cfg.q_road)
    m_dyn = math.inf
    for _name, series, limit in (dyn or []):
        m_dyn = min(m_dyn, float(np.min((limit - np.abs(series)) / limit)) / cfg.q_dyn)

    parts = {"clearance": m_clear, "road": m_road, "dyn": m_dyn}
    G_raw = min([*m_clear.values(), m_road, m_dyn]) if (m_clear or road is not None or dyn) else math.inf
    G = G_raw if terminal_ok in (True, None) else -math.inf
    G_search = float(np.clip(G_raw if math.isfinite(G_raw) else cfg.search_clip, -cfg.search_clip, cfg.search_clip))
    if terminal_ok is False:                    # search grade keeps a finite penalty so algorithms still see a gradient
        G_search = min(G_search, -0.05)
    return {"G": float(G), "G_raw": float(G_raw), "G_search": G_search, "terminal_ok": terminal_ok,
            "parts": parts, "argmin_tick": when, "min_clearance_m": clear, "cfg": asdict(cfg)}


def protected_hold_ok(speeds, dt, cfg=DEFAULT_V2, rear_clear=None):
    """Terminal 'protected hold': ego speed stays <= stop_speed for the last hold_s seconds of the rollout and (if given)
    the rear clearance series (m, minus reserve already applied by the caller) stays >= 0.  Zero speed alone is not enough
    when a rear actor can still arrive - pass rear_clear for such worlds."""
    n_hold = int(round(cfg.hold_s / dt))
    if len(speeds) < n_hold:
        return False
    if not np.all(np.asarray(speeds[-n_hold:]) <= cfg.stop_speed):
        return False
    return True if rear_clear is None else bool(np.all(np.asarray(rear_clear[-n_hold:]) >= 0.0))
