"""
reference/corridor.py - the PDF's exact 1-D reference model (section 4.1) and its derived boundaries.

Assumptions (all analytical, NOT CARLA properties): level corridor, stationary full-width obstacle, no lateral escape,
no rear threat, speed constant during the actuation delay tau, then constant achievable deceleration b.
gap = bumper-to-obstacle clearance at t0.  c = required clearance reserve after stopping.

    D_stop     = v*tau + v^2/(2b)
    d_required = D_stop + c
Boundaries (all in the same units):
    contact boundary   gap  = D_stop
    reserve boundary   gap  = D_stop + c
    max prefix (RC12)  T_p* = (gap - D_stop - c)/v         intervention may be delayed this long at speed v
    max delay  (RC17)  tau* = (gap - v^2/(2b) - c)/v
"""
import math

import numpy as np


def d_stop(v, tau, b):
    return v * tau + v * v / (2.0 * b)


def simulate(v, tau, b, gap, prefix=0.0, hold_s=4.0, dt=1e-3):
    """Numerically integrate: constant speed for (prefix + tau), then brake at b until stopped, then hold.
    Returns dict(min_clearance, final_clearance, contact, stopped, v_end)."""
    t_free = prefix + tau
    s, x_v, t = 0.0, v, 0.0
    min_clear = gap
    contact = False
    t_end = t_free + v / b + hold_s
    while t < t_end:
        a = 0.0 if t < t_free else (-b if x_v > 0 else 0.0)
        x_v = max(x_v + a * dt, 0.0)
        s += x_v * dt
        t += dt
        clear = gap - s
        min_clear = min(min_clear, clear)
        if clear <= 0:
            contact = True
            break
    return {"min_clearance": min_clear, "final_clearance": gap - s, "contact": contact,
            "stopped": (x_v == 0.0) and not contact, "v_end": x_v}


def margin(v, tau, b, gap, c=1.0, q=1.0, prefix=0.0):
    """G = path margin (clearance - reserve)/q, with terminal 'protected hold' Boolean: fail => -inf (stored separately)."""
    r = simulate(v, tau, b, gap, prefix=prefix)
    path = (r["min_clearance"] - c) / q
    terminal_ok = r["stopped"]
    return (path if terminal_ok else -math.inf), {**r, "terminal_ok": terminal_ok}


def label(v, tau, b, gap, c=1.0, eps=0.0, prefix=0.0):
    """Three distinct outcomes the PDF insists on keeping apart."""
    G, info = margin(v, tau, b, gap, c=c, prefix=prefix)
    if info["contact"]:
        return "contact"
    if G >= eps:
        return "R_REF_witness"
    return "no_contact_but_reserve_violated"


def model_certificate(gap, v, tau_min, b_max, c):
    """Sound statement for the REFERENCE MODEL only: needs genuine bounds tau_min (smallest possible delay) and b_max
    (largest possible deceleration). Never use a comfortable or typical value here."""
    lb = v * tau_min + v * v / (2.0 * b_max)
    if gap < lb:
        return "collision_inevitable_in_model"
    if gap < lb + c:
        return "reserve_infeasible_in_model"
    return "no_impossibility_proof"


if __name__ == "__main__":
    v, tau, b, c = 12.0, 0.25, 6.0, 1.0
    print(f"D_stop = {d_stop(v,tau,b):.3f} m,  d_required = {d_stop(v,tau,b)+c:.3f} m")
    for gap in (18.0, 15.5, 12.0):
        print(gap, label(v, tau, b, gap, c), model_certificate(gap, v, tau, b, c))
    print("T_p* (RC12, gap 18) =", (18 - d_stop(v, tau, b) - c) / v, "s")
    print("tau* (RC17, gap 18) =", (18 - v*v/(2*b) - c) / v, "s")
