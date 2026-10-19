"""
Repeat-spread measurement for D4 (Task 5), run from mining/ against the live server:

    cd mining && PYTHONNOUSERSITE=1 python ../scripts/carla_repeat_check.py

Re-runs RC12 corridor scenes (gap 40, v 12, brake_hold at several prefixes) whose records in results/carla/core/rc12_prefix.jsonl were
produced BEFORE the CARLA server restart, twice in the current session.  Reports
    same-session spread  : rep1 vs rep2 (current session)
    cross-session spread : old record vs rep1 (server restarted in between; same physics_profile_id)
in margin units and metres (clearance margins are metres / q_clear).  Writes results/carla/mining/repeat_spread.{json,md}.
Only the scenes actually re-run here are covered; this is a small sample, not a systematic cross-session study.
"""
import importlib.util
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "mining"))
spec = importlib.util.spec_from_file_location("carla_core", os.path.join(HERE, "carla_core.py"))
cc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cc)

OUT = "results/carla/mining"


def spread(a, b, q):
    """(max |dG|, in metres) between two R_raw matrices (finite entries); inf if a finite/-inf mismatch."""
    d = cc.profile_max_abs_diff([cc.gval(x) for row in a for x in row], [cc.gval(x) for row in b for x in row])
    return d, d * q


def main():
    os.makedirs(OUT, exist_ok=True)
    ctx = cc.Ctx("localhost", 2000)
    q = cc.CFG.q_clear
    old = {}
    for line in open("results/carla/core/rc12_prefix.jsonl"):
        r = json.loads(line)
        if r["physics_profile_id"] != ctx.pid:
            raise RuntimeError("profile mismatch")
        old[round(r["theta"]["prefix_s"], 2)] = r
    finite = sorted(p for p, r in old.items() if all(x != "-inf" for row in r["R_raw"] for x in row))
    pick = finite[:: max(1, len(finite) // 10)][:12]
    rows = []
    try:
        for p in pick:
            r1, r2 = (cc.rc12_rec(ctx, p).to_json() for _ in range(2))
            fin = lambda r: next(iter(r["traces"].values()))["final"]
            f0, f1, f2 = fin(old[p]), fin(r1), fin(r2)
            same = spread(r1["R_raw"], r2["R_raw"], q)
            cross = spread(old[p]["R_raw"], r1["R_raw"], q)
            rows.append({"prefix_s": p, "G_old": old[p]["R_raw"][0][0], "G_rep1": r1["R_raw"][0][0], "G_rep2": r2["R_raw"][0][0],
                         "same_session_dG": same[0], "cross_session_dG": cross[0], "same_session_m": same[1], "cross_session_m": cross[1],
                         "same_final": [f1, f2], "old_final": f0,
                         "label_old": old[p]["evidence_class"], "label_new": r1["evidence_class"]})
            print(rows[-1]["prefix_s"], rows[-1]["same_session_dG"], rows[-1]["cross_session_dG"], flush=True)
    finally:
        pass
    sm = max(r["same_session_m"] for r in rows)
    cm = max(r["cross_session_m"] for r in rows)
    out = {"n_scenes": len(rows), "same_session_max_m": sm, "cross_session_max_m": cm, "q_clear": q,
           "sigma_m": max(0.05, sm, cm), "rows": rows, "physics_profile_id": ctx.pid, "runtime": ctx.runtime}
    json.dump(out, open(f"{OUT}/repeat_spread.json", "w"), indent=1)
    L = ["## Repeat spread of the same CARLA scene (input to D4)\n",
         f"{len(rows)} RC12 corridor scenes (gap 40, v 12, tau 0.25, brake_hold, prefix varied) re-run twice in the current server session; the old records "
         "in `results/carla/core/rc12_prefix.jsonl` were produced before the server restart (same physics_profile_id). Margin G is normalised "
         f"(clearance metres / q_clear = {q}); metres = |dG| * {q}.\n",
         "| prefix_s | G (pre-restart) | G (rep 1) | G (rep 2) | same-session |dG| | cross-session |dG| | label old / new |", "|---|---|---|---|---|---|---|"]
    for r in rows:
        L.append(f"| {r['prefix_s']:.2f} | {r['G_old']:.5f} | {r['G_rep1']:.5f} | {r['G_rep2']:.5f} | {r['same_session_dG']:.5f} | "
                 f"{r['cross_session_dG']:.5f} | {r['label_old']} / {r['label_new']} |")
    L.append(f"\nMax same-session spread: {sm:.4f} m-equivalent.  Max cross-session spread (restart in between): {cm:.4f} m-equivalent "
             f"(small sample of {len(rows)} scenes, one family).  sigma used for D4 = max(0.05 m, observed) = {out['sigma_m']:.4f} m "
             f"= {out['sigma_m'] / q:.5f} in margin units.")
    open(f"{OUT}/repeat_spread.md", "w", encoding="utf-8", newline="\n").write("\n".join(L) + "\n")
    print("\n".join(L))


if __name__ == "__main__":
    main()
