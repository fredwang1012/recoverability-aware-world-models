"""
Task 9 fix round: re-flag the Task 5 mining claims at the max observed CARLA repeat spread (0.028 margin units) instead of the Task 5 sigma (0.01666).

    cd mining && python ../scripts/spread_reflag.py        -> results/carla/mining/sigma028.md

Reads the (git-ignored, local) records_*.jsonl, pairs_*.jsonl and d4_repeats.jsonl of results/carla/mining/{corridor,rc03}.  Pure counting; no simulator.
Nothing is re-run: scenes that fall inside the wider band but have no repeats are reported as NOT REPEATED, never as resolved.
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "mining"))
sys.path.insert(0, HERE)

import mine_cases as MC
import mining_post as MP

SIG_OLD, SIG_NEW, Q_CLEAR = 0.01666, 0.028, 5.0
ROOT = "results/carla/mining"


def count_band(vals, sigma):
    return sum(v is not None and math.isfinite(v) and abs(v) <= sigma for v in vals)


def family_block(fam):
    d = f"{ROOT}/{fam}"
    recs, members = MP.load_records(d)
    pool, seen = [], set()
    for r in recs + members:
        k = json.dumps(r["theta"], sort_keys=True)
        if k not in seen:
            seen.add(k)
            pool.append(r)
    vals = [MC.v_raw(r) for r in pool]
    d4 = {x["key"]: x for x in MP.load_d4(d)}
    keys = [json.dumps(r["theta"], sort_keys=True) for r in pool]
    L = [f"### {fam}\n", f"Pool: {len(pool)} distinct scenes (records + pair members), label-grade V = max_j min_w R_raw.\n",
         "| sigma (margin units) | scenes with finite V, abs(V - eps) <= sigma | of which with D4 repeats | of which NOT REPEATED | repeated rows UNRESOLVED (label side/kind changed) |", "|---|---|---|---|---|"]
    for s in (SIG_OLD, SIG_NEW):
        idx = [i for i, v in enumerate(vals) if v is not None and math.isfinite(v) and abs(v) <= s]
        rep = [i for i in idx if keys[i] in d4]
        unres = sum(MP.d4_flag(d4[keys[i]]["V0"], MC.label_kind(d4[keys[i]]["label0"]),
                                [(x["V"], MC.label_kind(x["label"])) for x in d4[keys[i]]["reps"]]) for i in rep)
        L.append(f"| {s:g} | {len(idx)} | {len(rep)} | {len(idx) - len(rep)} | {unres} |")
    # all D4 rows regardless of band: the largest observed spread
    sp = [MP.spread_of(x) for x in d4.values()]
    L.append(f"\nD4 rows (all, including the extended set): {len(d4)}; rows with spread > 0: {sum(x > 0 for x in sp)}; max spread {max(sp or [0]):.5f}; "
             f"rows UNRESOLVED over all repeats: {sum(MP.d4_flag(x['V0'], MC.label_kind(x['label0']), [(y['V'], MC.label_kind(y['label'])) for y in x['reps']]) for x in d4.values())}.\n")
    # all scene records of all algorithms (as in summary.md)
    allr = []
    for fn in sorted(os.listdir(d)):
        if fn.startswith("records_") and fn.endswith(".jsonl"):
            allr += [json.loads(l) for l in open(f"{d}/{fn}")]
    nb = sum(abs(r["_Vsearch"]) < 0.3 for r in allr)
    nb_edge = sum(abs(r["_Vsearch"]) <= SIG_NEW for r in allr)
    L.append(f"near_boundary (summary.md definition, abs(V_search) < 0.3): {nb} of {len(allr)} scene records; the window is about 10 times the spread, so it is "
             f"insensitive to sigma; {nb_edge} records have abs(V_search) <= {SIG_NEW} (V_search is the finite search grade, so this is only an indicator).\n")
    anc = [(r["_anchor_G_m"], r["evidence_class"]) for r in allr if r.get("_anchor_G_m") is not None and MC.label_kind(r["evidence_class"]) in ("R", "U")]
    if anc:
        ag = lambda lo: [(a > 0) == (e == "R_REF") for a, e in anc if abs(a) > lo]
        for s in (SIG_OLD, SIG_NEW):
            lo = s * Q_CLEAR
            sub = ag(lo)
            L.append(f"label_vs_anchor_agree excluding scenes with abs(analytic margin) <= {lo:.3f} m (= {s:g} x q_clear {Q_CLEAR:g}): "
                     f"{sum(sub)}/{len(sub)} = {sum(sub) / len(sub):.3f}; scenes excluded as inside the noise band: {len(anc) - len(sub)}")
        allag = ag(-1)
        L.append(f"label_vs_anchor_agree over all {len(anc)} R/U-labelled scene records: {sum(allag)}/{len(allag)} = {sum(allag) / len(allag):.3f}.\n")
    else:
        L.append("label_vs_anchor_agree: no analytic anchor for this family (n/a).\n")
    return "\n".join(L)


def main():
    text = ("# Mining claims re-flagged at the max observed repeat spread (0.028)\n\n"
            "Task 5 used sigma = 0.01666 (observed corridor and rc03 repeats). Task 8 later observed a spread of 0.028 (RC13 scene 14/20/0.2). "
            "This file recomputes the near-threshold sets from the local records at both values. Only scenes that already have D4 repeats can be checked "
            "for a changed label; the others are listed as NOT REPEATED and nothing is assumed about them. Margin units: clearance metres / q_clear = 5.0, "
            "so 0.028 = 0.14 m.\n\n" + "\n".join(family_block(f) for f in ("corridor", "rc03")))
    open(f"{ROOT}/sigma028.md", "w", encoding="utf-8", newline="\n").write(text)
    print(text)


if __name__ == "__main__":
    main()
