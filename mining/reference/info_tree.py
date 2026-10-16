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


def label_scene(D, t_obs, **kw):
    names, R = build_matrix(D, t_obs=t_obs, **kw)
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
