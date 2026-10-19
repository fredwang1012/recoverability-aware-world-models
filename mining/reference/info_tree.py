"""
reference/info_tree.py - finite information-tree reference for RC10 / RC11 (PDF sections 2.4, 4.2).

Scene (analytical, NOT CARLA): ego at speed v must clear a blocker plane at distance D. It cannot stop in time
(D < D_stop with actuation delay). A hidden actor blocks ONE of two escape corridors: world A blocks LEFT, world B blocks RIGHT.
The mode is delivered to the policy at time t_obs (already including sensing/delivery delay).

A lateral escape issued at t_s takes effect after tau and needs T_lat to complete before the ego reaches the plane at t_arr(p),
where p is the duration of a shared braking prefix (issued at t=0, effective after tau):
        slack = t_arr(p) - (t_s + tau + T_lat) - reserve          (G, in seconds; >= eps means success)
Causal policies may only choose the corridor using the mode AFTER t_obs + tau_plan; before that they must commit blindly.

Library Pi: stop | stay | blind(side, p, t_s) | observe(p)   (observe: branch at t_s = t_obs + tau_plan, pick the free corridor)
Worlds Omega = {A, B}.  contract.classify() then yields R_REF / U_INFORMATION / U_LIBRARY exactly as in the PDF.
"""
import math

import numpy as np

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from contract import classify, R_REF, U_INFORMATION, U_LIBRARY  # noqa: E402


def t_arrival(D, v, tau, b, p):
    """Time at which the ego reaches distance D: speed v until tau; brake at b for p seconds (starting at tau); then constant."""
    s_tau = v * tau
    if s_tau >= D:
        return D / v
    t_brake_stop = v / b
    pb = min(p, t_brake_stop)
    s_brake = v * pb - 0.5 * b * pb * pb
    v_after = v - b * pb
    if s_tau + s_brake >= D:                          # reaches the plane during braking
        # solve v*t - 0.5 b t^2 = D - s_tau
        rem = D - s_tau
        disc = v * v - 2 * b * rem
        return tau + (v - math.sqrt(max(disc, 0.0))) / b
    rem = D - s_tau - s_brake
    if v_after <= 1e-9:
        return math.inf                                # stopped before the plane
    return tau + pb + rem / v_after


def build_matrix(D, v=15.0, tau=0.25, b=6.0, T_lat=1.2, tau_plan=0.1, t_obs=0.0, reserve=0.05, c_stop=1.0,
                 p_grid=None, ts_grid=None):
    p_grid = np.arange(0.0, 2.55, 0.05) if p_grid is None else p_grid
    ts_grid = np.arange(0.0, 2.05, 0.05) if ts_grid is None else ts_grid
    names, rows = [], []
    # stop: succeeds iff the plane is farther than the stopping distance + reserve (margin in seconds at speed v)
    dstop = v * tau + v * v / (2 * b)
    g_stop = (D - dstop - c_stop) / v
    names.append("stop"); rows.append([g_stop, g_stop])
    names.append("stay"); rows.append([-1.0, -1.0])
    for p in p_grid:
        t_arr = t_arrival(D, v, tau, b, p)
        if not math.isfinite(t_arr):                    # braked to a stop before the plane = same as 'stop' outcome
            continue
        for ts in ts_grid:                              # blind commitments
            slack = t_arr - (ts + tau + T_lat) - reserve
            names.append(f"blind_L(p={p:.2f},ts={ts:.2f})"); rows.append([-1.0, slack])   # A blocks left, B blocks right
            names.append(f"blind_R(p={p:.2f},ts={ts:.2f})"); rows.append([slack, -1.0])
        ts = t_obs + tau_plan                           # observation-conditioned branch
        slack = t_arr - (ts + tau + T_lat) - reserve
        names.append(f"observe(p={p:.2f})"); rows.append([slack, slack])
    return names, np.array(rows)


def build_matrix_two_lane(D, v=15.0, tau=0.25, b=6.0, T_lat=1.2, tau_plan=0.1, t_obs=0.0, reserve=0.05, c_stop=1.0,
                          p_grid=None, ts_grid=None):
    """Same finite tree on the two-lane straight site (CARLA cases RC10/RC11; Task 7).  RECONCILIATION with build_matrix:
    build_matrix has two LATERAL corridors that both need T_lat (a blocker plane with a free corridor on each side).  On the two-lane
    site the corridors are the ego's own (RIGHT) lane and the adjacent LEFT lane, and the stopped hidden vehicle sits in ONE of them:
        world A (hid_left_lane) : hidden vehicle in the LEFT lane  -> the ego-lane corridor is free: staying / stopping works, no lateral move
        world B (hid_own_lane)  : hidden vehicle in the ego lane   -> only the left-lane change works (needs T_lat, as before)
    Rows: stop, stay (fixed_right), blind_L(p, ts) (fixed_left = brake p and change left at ts), observe(p).  The binding world is B
    (the lateral escape), whose slack is the same expression as in build_matrix, so the analytic threshold t_obs* is unchanged
    (t_obs_star below); the ego-lane corridor only adds non-binding positive entries in world A (+1 = clear).  Columns: [A, B]."""
    p_grid = np.arange(0.0, 2.55, 0.05) if p_grid is None else p_grid
    ts_grid = np.arange(0.0, 2.05, 0.05) if ts_grid is None else ts_grid
    names, rows = [], []
    dstop = v * tau + v * v / (2 * b)
    g_stop = (D - dstop - c_stop) / v
    names.append("stop"); rows.append([1.0, g_stop])                   # stopping in the ego lane is harmless when the vehicle is in the left lane
    names.append("stay"); rows.append([1.0, -1.0])                    # fixed_right: keep lane at speed
    for p in p_grid:
        t_arr = t_arrival(D, v, tau, b, p)
        if not math.isfinite(t_arr):
            continue
        for ts in ts_grid:
            slack = t_arr - (ts + tau + T_lat) - reserve
            names.append(f"blind_L(p={p:.2f},ts={ts:.2f})"); rows.append([-1.0, slack])   # left change collides with the left-lane vehicle
        ts = t_obs + tau_plan
        slack = t_arr - (ts + tau + T_lat) - reserve
        names.append(f"observe(p={p:.2f})"); rows.append([1.0, slack])
    return names, np.array(rows)


def t_arrival_max(D, v, tau, b):
    """Latest possible arrival time at distance D over braking-prefix durations p (continuous braking; inf excluded = a full stop).
    Returns (t_arr, v_arr); None if D >= the stopping distance (the ego stops before the plane)."""
    s_tau = v * tau
    if s_tau >= D:
        return D / v, v
    rem = D - s_tau
    disc = v * v - 2 * b * rem
    if disc <= 0:
        return None
    return tau + (v - math.sqrt(disc)) / b, math.sqrt(disc)


def t_obs_star(D, v, tau, b, T_lat, tau_plan=0.0, reserve=0.0, p=None):
    """Analytic latest evidence time at which the contingent branch still has slack >= 0 (closed form of the info_tree observe row:
    slack = t_arr(p) - (t_obs + tau_plan + tau + T_lat) - reserve).  p = braking-prefix duration of the branch policy; None = the supremum
    over p (continuous braking).  None when the ego stops before the plane (a common stop exists)."""
    if p is None:
        ta = t_arrival_max(D, v, tau, b)
        ta = None if ta is None else ta[0]
    else:
        ta = t_arrival(D, v, tau, b, p)
        ta = None if not math.isfinite(ta) else ta
    return None if ta is None else ta - tau - T_lat - tau_plan - reserve


def label_scene(D, t_obs, two_lane=False, **kw):
    names, R = (build_matrix_two_lane if two_lane else build_matrix)(D, t_obs=t_obs, **kw)
    out = classify(R)
    out["best"] = names[out["best_policy"]] if out["best_policy"] is not None else None
    return out


if __name__ == "__main__":
    print("t_obs sweep at D=20 m (cannot stop: D_stop with delay = 22.5 m)")
    print(f"{'t_obs':>6} {'evidence':>14} {'V':>7} {'V_hind':>7} {'gap':>6}  best")
    for t_obs in np.arange(0.0, 0.61, 0.05):
        o = label_scene(20.0, t_obs)
        print(f"{t_obs:6.2f} {o['evidence']:>14} {o['V']:7.3f} {o['V_hind']:7.3f} {o['info_gap']:6.3f}  {o['best']}")
    print("\nD sweep at t_obs=0.5 (late evidence) vs 0.0 (early)")
    for D in (10, 14, 18, 20, 22, 24, 27):
        e = label_scene(D, 0.0)["evidence"], label_scene(D, 0.5)["evidence"]
        print(f"D={D:4.1f}  early:{e[0]:>14}  late:{e[1]:>14}")
