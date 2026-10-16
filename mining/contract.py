"""
contract.py - the recoverability contract from "Recoverability: definitions, formulations and a constructed
CARLA case catalogue" (sections 2-3), as code.

Input: margin matrix R[j, w] = G(pi_j, omega_w; h)   (policies x worlds), NaN = unresolved (never run / pruned).

    r_j      = min_w R[j, w]                       worst-world margin of policy j
    V        = max_j r_j                           best causal-policy margin (robust)
    V_hind   = min_w max_j R[j, w]                 best per-world margin (hindsight; each world may use its own policy)
    V <= V_hind always (max-min <= min-max); the gap V_hind - V is the price of not knowing the world.

Evidence classes (section 3):
    R_REF         some policy has every world resolved and r_j >= eps          (positive witness)
    U_LIBRARY     every policy has a resolved refutation (a world with G < eps) (negative, scope = this library only)
    U_INFORMATION U_LIBRARY, and every world individually has a witness         (hindsight-feasible, not causal-feasible)
    UNRESOLVED    anything else (missing entries, no witness, no full refutation)
    (U_MODEL needs an external sound proof; INVALID_RUN is set by the run harness. Neither can be derived here.)

A refutation needs only ONE resolved world with G < eps for that policy; a witness needs ALL worlds resolved.
"""
import numpy as np

R_REF, U_LIBRARY, U_INFORMATION, UNRESOLVED = "R_REF", "U_LIBRARY", "U_INFORMATION", "UNRESOLVED"


def _gap(V_hind, V):
    """V_hind - V; NaN if either is unresolved or both are -inf (terminal failures; no finite price of ignorance)."""
    if np.isnan(V) or np.isnan(V_hind) or (np.isneginf(V) and np.isneginf(V_hind)):
        return np.nan
    return float(V_hind - V)


def classify(R, eps=0.0):
    R = np.asarray(R, float)
    if R.ndim != 2:
        raise ValueError("R must be (policies, worlds)")
    n, w = R.shape
    known = ~np.isnan(R)
    refuted = np.array([(R[j][known[j]] < eps).any() for j in range(n)])
    witness = np.array([known[j].all() and (R[j] >= eps).all() for j in range(n)])

    # robust value: only defined for policies fully resolved or already refuted (upper bound = min over resolved)
    r_upper = np.array([np.nanmin(R[j]) if known[j].any() else np.inf for j in range(n)])
    r_exact = np.where(known.all(axis=1), r_upper, np.nan)
    V_upper = float(np.max(r_upper)) if n else -np.inf                    # optimistic bound on V
    V = float(np.nanmax(r_exact)) if (~np.isnan(r_exact)).any() else np.nan  # exact over fully-resolved policies (-inf allowed)

    # hindsight per world (needs the column resolved for at least the best policy)
    world_best = np.array([np.nanmax(R[:, k]) if known[:, k].any() else np.nan for k in range(w)])
    world_feasible = world_best >= eps
    V_hind = float(np.nanmin(world_best)) if (~np.isnan(world_best)).any() else np.nan

    if witness.any():
        ev = R_REF
    elif refuted.all():
        ev = U_INFORMATION if (known.all() and world_feasible.all()) else U_LIBRARY
    else:
        ev = UNRESOLVED
    return {"evidence": ev, "V": V, "V_upper": V_upper, "V_hind": V_hind,
            "info_gap": _gap(V_hind, V),
            "n_witness": int(witness.sum()), "n_refuted": int(refuted.sum()),
            "n_unresolved_entries": int((~known).sum()),
            "world_feasible": world_feasible.tolist(),
            "best_policy": int(np.nanargmax(r_exact)) if (~np.isnan(r_exact)).any() else None}
