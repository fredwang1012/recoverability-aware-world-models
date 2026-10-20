"""
G sensitivity on CARLA: how many CARLA records change evidence class when each MarginConfigV2 constant moves x0.5 / x1.5.

    python scripts/g_sensitivity_carla.py        (repo root; no simulator, no new runs)  -> mining/results/carla/g_sensitivity.md

Re-scored from the stored records (mining/results/carla/<group>/*.jsonl and mining/results/carla/mining/*/records_*.jsonl;
`*superseded*` files skipped).  Every episode stores its margin parts before the min: per-actor minimum clearance in metres
(`min_clearance_m`), the normalised road-edge margin and the normalised dynamic-limit margin.  So for the constants that only
enter after the rollout (reserve_m, q_clear, q_road, q_dyn, eps) G can be recomputed exactly:
    clearance_i = (d_i - reserve) / q_clear,  road = road_0 * q_road_0 / q_road,  dyn = dyn_0 * q_dyn_0 / q_dyn,
    G = min(...) if terminal_ok else -inf,    then contract.classify(R, eps) as in records.finalize.
Exactness check: at the default constants the recomputed matrix must reproduce the stored R_raw (|diff| <= 1e-9) entry by entry;
an entry that does not (two library members with the same name share one trace key) is not reconstructable and its record is
left out of the counts (reported).  Clearance saturates at far = 4 q_clear + reserve in margin.compute; a saturated actor whose
true distance is unknown is taken at the new saturation value when it shrinks, and at the old one (a lower bound) when it grows -
a far actor never sets the minimum, so this cannot flip a class unless every other part is also far (counted, reported).
Constants that change the rollout or the terminal test (horizon_s, hold_s, post_maneuver_s, stop_speed, swept_*) are NOT
recomputable from records and need re-runs; they are listed as NOT EXECUTED.  eps = 0 has no relative scale: it is moved +/-0.05.
"""
import glob
import json
import math
import os
import sys
from dataclasses import replace
from collections import Counter, defaultdict

import numpy as np

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, os.path.join(ROOT, "mining"))

import contract  # noqa: E402
import margin as M  # noqa: E402

RES = os.path.join(ROOT, "mining", "results", "carla")
OUT = os.path.join(RES, "g_sensitivity.md")
D = M.DEFAULT_V2
TOL = 1e-9


def fnum(x):
    if x is None:
        return None
    if isinstance(x, str):
        return {"-inf": -math.inf, "inf": math.inf, "nan": math.nan}[x]
    return float(x)


def load():
    files = sorted(glob.glob(os.path.join(RES, "*", "*.jsonl")) + glob.glob(os.path.join(RES, "mining", "*", "records_*.jsonl")))
    out = []
    for f in files:
        if "superseded" in os.path.basename(f) or "old_pp_" in f:
            continue
        group = os.path.relpath(f, RES).replace("\\", "/").split("/")[0]
        if group == "mining":
            group = "mining/" + os.path.relpath(f, RES).replace("\\", "/").split("/")[1]
        for line in open(f, encoding="utf-8"):
            r = json.loads(line)
            if "R_raw" in r and "traces" in r and r.get("backend") == "carla":
                out.append((group, os.path.basename(f), r))
    return out


def episode_G(tr, term_ok, cfg, cfg0):
    """Recompute one episode's G under cfg from its stored parts (cfg0 = the constants the record was scored with).
    Returns (G, saturated_lower_bound_used)."""
    mg = tr.get("margin") or {}
    parts, dmin = mg.get("parts") or {}, mg.get("min_clearance_m") or {}
    far0 = 4.0 * cfg0["q_clear"] + cfg0["reserve_m"]
    far1 = 4.0 * cfg.q_clear + cfg.reserve_m
    vals, lb = [], False
    for name, d in dmin.items():
        d = fnum(d)
        if d >= far0 - 1e-9:                       # saturated in the stored run: true distance >= far0
            if far1 <= far0:
                d = far1
            else:
                d, lb = far0, True
        vals.append((d - cfg.reserve_m) / cfg.q_clear)
    road = fnum(parts.get("road"))
    if road is not None and math.isfinite(road):
        vals.append(road * cfg0["q_road"] / cfg.q_road)
    dyn = fnum(parts.get("dyn"))
    if dyn is not None and math.isfinite(dyn):
        vals.append(dyn * cfg0["q_dyn"] / cfg.q_dyn)
    g_raw = min(vals) if vals else math.inf
    return (g_raw if term_ok in (True, None) else -math.inf), lb


def matrix(r, cfg):
    cfg0 = r["margin_config"]
    R, lb = [], False
    for j, p in enumerate(r["policies"]):
        row = []
        for k, w in enumerate(r["worlds"]):
            stored = fnum(r["R_raw"][j][k])
            if stored is None:
                row.append(np.nan)
                continue
            tr = r["traces"].get(f"{p}/{w}")
            if tr is None:
                return None, False
            g, l = episode_G(tr, r["terminal_ok"][j][k], cfg, cfg0)
            lb |= l
            row.append(g)
        R.append(row)
    return np.array(R, float), lb


def reconstructable(r):
    R, _ = matrix(r, M.MarginConfigV2(**{k: r["margin_config"][k] for k in ("reserve_m", "q_clear", "q_road", "q_dyn", "q_term", "eps")}))
    if R is None:
        return False
    S = np.array([[np.nan if fnum(x) is None else fnum(x) for x in row] for row in r["R_raw"]], float)
    with np.errstate(invalid="ignore"):
        same = (np.isnan(R) & np.isnan(S)) | (R == S) | (np.abs(R - S) <= TOL)
    return bool(same.all())


def evidence(R, eps, r):
    return "INVALID_RUN" if r.get("invalid_reasons") else contract.classify(R, eps)["evidence"]


def main():
    recs = load()
    ok, bad = [], Counter()
    for g, f, r in recs:
        (ok.append((g, f, r)) if reconstructable(r) else bad.update([g]))
    base = {}
    for i, (g, f, r) in enumerate(ok):
        cfg = M.MarginConfigV2(**{k: r["margin_config"][k] for k in ("reserve_m", "q_clear", "q_road", "q_dyn", "q_term", "eps")})
        R, _ = matrix(r, cfg)
        base[i] = evidence(R, cfg.eps, r)
        assert base[i] == r["evidence_class"], (f, r.get("_key"), base[i], r["evidence_class"])

    perturb = []
    for name in ("reserve_m", "q_clear", "q_road", "q_dyn"):
        for fac in (0.5, 1.5):
            perturb.append((name, f"x{fac}", lambda c, n=name, f=fac: replace(c, **{n: getattr(c, n) * f})))
    for de in (-0.05, 0.05):
        perturb.append(("eps", f"{de:+.2f}", lambda c, d=de: replace(c, eps=c.eps + d)))

    rows, detail = [], defaultdict(Counter)
    for name, lab, fn in perturb:
        n_chg, n_lb, by_group, trans = 0, 0, Counter(), Counter()
        for i, (g, f, r) in enumerate(ok):
            cfg = M.MarginConfigV2(**{k: r["margin_config"][k] for k in ("reserve_m", "q_clear", "q_road", "q_dyn", "q_term", "eps")})
            cfg = fn(cfg)
            R, lb = matrix(r, cfg)
            ev = evidence(R, cfg.eps, r)
            if ev != base[i]:
                n_chg += 1
                n_lb += lb
                by_group[g] += 1
                trans[f"{base[i]} -> {ev}"] += 1
                detail[(name, lab)][f"{r['case_id']} ({g}/{f})"] += 1
        rows.append((name, lab, n_chg, n_lb, by_group, trans))

    groups = Counter(g for g, _, _ in ok)
    L = ["# G sensitivity on CARLA (re-scored stored records; no new runs)\n",
         f"Script `scripts/g_sensitivity_carla.py` (method in its docstring).  Records: {len(recs)} CARLA records loaded "
         f"(superseded files skipped); {len(ok)} reconstructable and used; {sum(bad.values())} left out because a duplicated policy "
         f"name shares one trace key (by group: {dict(bad) or 'none'}).  Self-check: at the stored constants every used record reproduces "
         "its stored R_raw entry by entry (|diff| <= 1e-9) and its stored evidence class (asserted).\n",
         "Records used per group: " + ", ".join(f"{g} {n}" for g, n in sorted(groups.items())) + ".\n",
         "Defaults (margin.DEFAULT_V2): reserve_m 1.0, q_clear 5.0, q_road 1.0, q_dyn 1.0, eps 0.0.  With eps = 0 a positive rescaling "
         "(q_*) cannot move G across 0, so q_* rows are expected to show 0 changes; reserve_m shifts every clearance margin by its change "
         "in metres; eps moves the label threshold itself.  Read the rates with the sampling in mind: the mining records (straddle, bisect) "
         "are concentrated near the label boundary by design and the case sweeps bracket it, so the share of records that flip is far "
         "higher than for scenes drawn uniformly.  The q_* rows are the scale-invariance check; the reserve_m / eps rows show how much of "
         "the found boundary sits within 0.5 m / 0.05 of it.\n",
         "| constant | change | records changing class | of which used a saturated lower bound | by group | transitions |",
         "|---|---|---|---|---|---|"]
    for name, lab, n, nlb, bg, tr in rows:
        L.append(f"| {name} | {lab} | {n} / {len(ok)} | {nlb} | {', '.join(f'{g} {c}' for g, c in sorted(bg.items())) or '-'} | "
                 f"{', '.join(f'{t} {c}' for t, c in sorted(tr.items())) or '-'} |")
    L.append("\n### Changing records by case\n")
    for (name, lab), c in detail.items():
        L.append(f"- {name} {lab}: " + "; ".join(f"{k} x{v}" for k, v in sorted(c.items())))
    if not detail:
        L.append("- none")
    L.append("\n**NOT EXECUTED:** horizon_s, hold_s, post_maneuver_s, stop_speed, swept_substeps, swept_step_m.  These change the "
             "rollout or the terminal test, which the stored parts cannot replay; they need new CARLA runs.\n")
    text = "\n".join(L) + "\n"
    open(OUT, "w", encoding="utf-8", newline="\n").write(text)
    print(text)


if __name__ == "__main__":
    main()
