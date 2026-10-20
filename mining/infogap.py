"""
infogap.py - D5 information-gap mining on the RC10/RC11 family: pure helpers (mock-tested, no simulator).

Info-gap scene (spec D5, contract section 2):  V_hind >= eps  and  V < eps.   Every world is individually solvable (hindsight), but no
causal policy of the library survives all of them.  The mining objective is   maximise  V_hind - V   subject to   V_hind >= eps.

Two grades, never mixed:
    search signal   R_norm (finite, clipped G_search).  Used ONLY to steer the ask/tell algorithms.
    label / objective   R_raw of the full ExecutionRecord (label-grade G, -inf allowed), through contract.classify.
When V = -inf (every policy collides in some world) the label-grade gap V_hind - V is +inf: reported as such, never clipped, never
ranked as if it were a finite margin.
"""
import math

import numpy as np

import contract


def _arr(R):
    """Matrix with None -> NaN, '-inf' -> -inf (unrun entries stay unresolved, never 0)."""
    def f(x):
        if x is None:
            return np.nan
        return -np.inf if x == "-inf" else float(x)
    return np.array([[f(x) for x in row] for row in R], float)


def search_signals(R_norm, eps=0.0):
    """(V_search, V_hind_search, gap_search) from the finite search-grade matrix; NaN where unresolved."""
    c = contract.classify(_arr(R_norm), eps)
    return c["V"], c["V_hind"], c["info_gap"]


def cem_surrogate(R_norm, eps=0.0):
    """Value handed to CEM, which MINIMISES it: feasible scenes (V_hind_search >= eps) get -(V_hind - V) so a bigger gap is better;
    infeasible scenes get a positive penalty that grows with the hindsight shortfall, so every feasible scene ranks ahead of every infeasible one."""
    V, Vh, gap = search_signals(R_norm, eps)
    if not (math.isfinite(Vh) and math.isfinite(V)) or Vh < eps:
        shortfall = eps - Vh if math.isfinite(Vh) else 1.0
        return 1.0 + float(shortfall)
    return -float(gap)


def straddle_surrogate(R_norm, eps=0.0):
    """Value handed to the level-set (straddle) search: V_search itself.  Its eps-level set is the edge of the unrecoverable region, which is
    where the info-gap scenes live; the GP sees nothing but the finite search margin."""
    V, _, _ = search_signals(R_norm, eps)
    return float(V) if math.isfinite(V) else -1.0


def full_objective(rec, eps=0.0):
    """Label-grade D5 quantities from an ExecutionRecord JSON (R_raw), recomputed with contract.classify.
    objective = V_hind - V when feasible (V_hind >= eps) else None; may be +inf when V = -inf."""
    c = contract.classify(_arr(rec["R_raw"]), eps)
    V, Vh = c["V"], c["V_hind"]
    ok = math.isfinite(Vh) and Vh >= eps
    resolved = not math.isnan(V) and not math.isnan(Vh)
    return {"V": V, "V_hind": Vh, "feasible": bool(ok and resolved),
            "is_info_gap": bool(ok and resolved and V < eps),
            "V_is_neg_inf": bool(V == -np.inf),
            "objective": (float(Vh - V) if (ok and resolved) else None),
            "evidence_class": rec.get("evidence_class"), "recomputed_evidence": c["evidence"]}


def rank_key(obj):
    """Sort key, best first: finite gaps by size, then V = -inf scenes (infinite gap) by V_hind.  Non-info-gap scenes last."""
    if not obj["is_info_gap"]:
        return (2, 0.0)
    if obj["V_is_neg_inf"]:
        return (1, -obj["V_hind"])
    return (0, -obj["objective"])


def sweep_transition_from_class_string(classes):
    """Task 7 sweep helper: classes = [(t_reveal, evidence)] sorted by t -> (last_R_REF, first_non_R_REF) or None."""
    flags = [e == "R_REF" for _, e in classes]
    if all(flags) or not any(flags) or flags != sorted(flags, reverse=True):
        return None
    k = flags.index(False)
    return classes[k - 1][0], classes[k][0]
