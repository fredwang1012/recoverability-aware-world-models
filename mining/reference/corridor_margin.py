"""reference/corridor_margin.py - the PDF section 4.1 corridor scored through margin.py (oriented boxes, swept check,
reserve, protected-hold terminal), i.e. the SAME code path CARLA traces will use.

Ego front bumper starts at x=0 (centre at -hl).  A full-width blocker's rear face sits at x=gap.
Kinematics are the PDF's: speed constant for prefix+tau, then constant deceleration b to rest, then hold."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import margin as M  # noqa: E402

EGO_HL, EGO_HW = 2.3, 0.95
BLOCK_HL, BLOCK_HW = 1.0, 5.0          # 'full width' within the corridor


def trace(v, tau, b, gap, prefix=0.0, dt=0.02, cfg=M.DEFAULT_V2):
    """Exact (closed-form) kinematics sampled on the tick grid - no integration error."""
    t_free = prefix + tau
    n = int(round((t_free + v / b + cfg.hold_s + 1.0) / dt)) + 1
    t = np.arange(n) * dt
    td = np.maximum(t - t_free, 0.0)
    tb = np.minimum(td, v / b)
    front = v * np.minimum(t, t_free) + v * tb - 0.5 * b * tb ** 2     # bumper position
    speed = np.where(t < t_free, v, np.maximum(v - b * td, 0.0))
    ego = [(float(f - EGO_HL), 0.0, 0.0) for f in front]
    block = [(gap + BLOCK_HL, 0.0, 0.0)] * n
    return {"dt": dt, "ego": ego, "others": {"blocker": block},
            "half": {"ego": (EGO_HL, EGO_HW), "blocker": (BLOCK_HL, BLOCK_HW)}}, speed


def score(v, tau, b, gap, c=None, prefix=0.0, dt=0.02, cfg=M.DEFAULT_V2):
    """Returns (G, details).  c overrides cfg.reserve_m."""
    if c is not None:
        cfg = M.MarginConfigV2(**{**cfg.__dict__, "reserve_m": c})
    tr, speeds = trace(v, tau, b, gap, prefix, dt, cfg)
    hold_ok = M.protected_hold_ok(speeds, dt, cfg)
    out = M.compute(tr, cfg, terminal_ok=hold_ok)
    out["clear_m"] = out["min_clearance_m"]["blocker"]
    out["hold_ok"] = hold_ok
    return out["G"], out


def label(v, tau, b, gap, c=1.0, eps=0.0, prefix=0.0, dt=0.02):
    G, o = score(v, tau, b, gap, c, prefix, dt)
    if o["clear_m"] <= 0.0:
        return "contact"
    return "R_REF_witness" if G >= eps else "no_contact_but_reserve_violated"
