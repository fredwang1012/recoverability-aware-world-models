"""
Task 9 / D5: information-gap mining on the RC10/RC11 family.   maximise V_hind - V   subject to   V_hind >= eps   (info-gap = V < eps too).

    python scripts/mine_infogap.py --backend mock  --algos sobol,cem,straddle --budget 36 --seeds 2
    cd mining && python ../scripts/mine_infogap.py --backend carla --algos cem,straddle --budget 9 --seeds 2 --batch 3

theta = (D, v, t_reveal) (families.InfoGap).  The ask/tell algorithms see ONLY the finite search signal (infogap.cem_surrogate for cem,
infogap.straddle_surrogate for straddle / sobol); every label, the objective and the found-scene list come from the full ExecutionRecord
(R_raw through contract.classify).  Records are checkpointed to results/<backend>/mining/infogap/records_<algo>_s<seed>.jsonl and a rerun
skips finished scenes.  Output summary: results/<backend>/mining/infogap/infogap.md.  CARLA is run from mining/; mock from the repo root.
The Task 7 sweep transition (mining/results/carla/occlusion.md) is the comparison reference: an info-gap scene found here is checked against the
analytic reveal threshold of its (D, v).
"""
import argparse
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "mining"))
sys.path.insert(0, HERE)

import infogap as IG
import mine_cases as MC
from families import INFOGAP


def analytic_t_star(D, v):
    """Policy-matched analytic reveal threshold from the Task 7 convention (None = a common stop exists)."""
    import carla_occlusion as CO
    return CO.analytic_threshold(D, v)


def stop_distance(v):
    import carla_occlusion as CO
    return CO.stop_distance(v)


def scene_row(r, eps):
    th = r["theta"]
    o = IG.full_objective(r, eps)
    ts = analytic_t_star(th["gap_m"], th["v_mps"])
    return {"algo": r["_algo"], "seed": r["_seed"], "gap_m": th["gap_m"], "v_mps": th["v_mps"], "t_reveal_s": th["t_reveal_s"],
            "evidence": r["evidence_class"], "V": o["V"], "V_hind": o["V_hind"], "objective": o["objective"],
            "is_info_gap": o["is_info_gap"], "V_is_neg_inf": o["V_is_neg_inf"], "t_star_analytic": ts,
            "stop_dist_m": stop_distance(th["v_mps"]), "search_value": r["_Vsearch"]}


def fnum(x, nd=3):
    if x is None:
        return "-"
    if isinstance(x, float) and math.isinf(x):
        return "-inf" if x < 0 else "inf"
    return f"{x:.{nd}f}"


def report(rows, args, ctx, budget, wall):
    algos = args.algos.split(",")
    L = [f"## D5 info-gap mining - backend: {ctx['backend'].upper()}"
         f"{' (MOCK plant: kinematic stand-in, NOT CARLA)' if ctx['backend'] == 'mock' else ''}\n",
         f"Family: RC10/RC11 (`cases/occlusion`), theta = (D, v, t_reveal) in D [{INFOGAP.lo[0]:g}, {INFOGAP.hi[0]:g}] m, v [{INFOGAP.lo[1]:g}, {INFOGAP.hi[1]:g}] m/s, "
         f"t_reveal [{INFOGAP.lo[2]:g}, {INFOGAP.hi[2]:g}] s on the 0.05 s tick grid (D to 0.1 m, v to 0.1 m/s).  Objective: maximise V_hind - V subject to "
         f"V_hind >= eps (eps = 0); an info-gap scene is V_hind >= eps and V < eps.  Search algorithms see only the finite search signal "
         f"(cem: -(V_hind - V) from R_norm for feasible scenes, penalty otherwise; straddle/sobol: V_search from R_norm); labels, V, V_hind and the "
         f"objective come from the full record (R_raw, contract.classify).  When V = -inf the label-grade gap is +inf; those scenes are listed "
         f"separately and never mixed with finite gaps.  {args.seeds} seed(s) x {budget} scenes per algorithm, batch {args.batch}; wall {wall:.0f} s.\n"]
    L += ["### Per algorithm\n", "| algo | scenes | distinct (D,v) | feasible (V_hind >= eps) | info-gap (V < eps) | of which V finite | of which V = -inf | "
          "R_REF | max finite gap | evidence mix |", "|---|---|---|---|---|---|---|---|---|---|"]
    for a in algos:
        rs = [r for r in rows if r["algo"] == a]
        ig = [r for r in rs if r["is_info_gap"]]
        fin = [r for r in ig if not r["V_is_neg_inf"]]
        mix = {}
        for r in rs:
            mix[r["evidence"]] = mix.get(r["evidence"], 0) + 1
        feas = [r for r in rs if r["objective"] is not None]
        L.append(f"| {a} | {len(rs)} | {len({(r['gap_m'], r['v_mps']) for r in rs})} | {len(feas)} | {len(ig)} | {len(fin)} | {len(ig) - len(fin)} | "
                 f"{sum(r['evidence'] == 'R_REF' for r in rs)} | {fnum(max((r['objective'] for r in fin), default=None))} | "
                 f"{', '.join(f'{k}={v}' for k, v in sorted(mix.items()))} |")
    ig_all = sorted((r for r in rows if r["is_info_gap"]), key=lambda r: IG.rank_key({"is_info_gap": True, "V_is_neg_inf": r["V_is_neg_inf"],
                                                                                      "V_hind": r["V_hind"], "objective": r["objective"]}))
    L += ["\n### Found info-gap scenes (best first: finite gaps by size, then V = -inf scenes by V_hind)\n",
          "t* = Task 7 policy-matched analytic reveal threshold for this (D, v) (the reveal time above which the shared-slowdown policy "
          "loses its witness); `t_reveal - t*` is the distance after it.  D_stop = common-stop distance at this v (D >= D_stop: a common stop exists, no info gap expected).\n",
          "| algo | seed | D (m) | v (m/s) | t_reveal (s) | V | V_hind | V_hind - V | t* (s) | t_reveal - t* | D_stop (m) | evidence |", "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in ig_all:
        ts = r["t_star_analytic"]
        L.append(f"| {r['algo']} | {r['seed']} | {r['gap_m']:g} | {r['v_mps']:g} | {r['t_reveal_s']:g} | {fnum(r['V'])} | {fnum(r['V_hind'])} | "
                 f"{fnum(r['objective'])} | {fnum(ts)} | {fnum(None if ts is None else r['t_reveal_s'] - ts, 2)} | {r['stop_dist_m']:.2f} | {r['evidence']} |")
    if not ig_all:
        L.append("| (none found) | | | | | | | | | | | |")
    inside = [r for r in ig_all if r["t_star_analytic"] is not None and r["gap_m"] < r["stop_dist_m"]]
    early = [r for r in ig_all if r["t_star_analytic"] is not None and r["t_reveal_s"] < r["t_star_analytic"]]
    L.append(f"\nComparison with the Task 7 transition: {len(ig_all)} info-gap scene(s); {len(inside)} lie below the analytic common-stop distance "
             f"(a necessary condition); {len(early)} have t_reveal earlier than the analytic t* (Task 7 measured the threshold 0.05 to 0.22 s AFTER t*, "
             f"so t_reveal slightly below t* is not by itself a contradiction).")
    nonig = [r for r in rows if not r["is_info_gap"] and r["evidence"] == "U_INFORMATION"]
    L.append(f"Scenes labelled U_INFORMATION that are not info-gap by the objective: {len(nonig)} (must be 0; U_INFORMATION implies V_hind >= eps and V < eps).")
    return "\n".join(L) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--backend", choices=["mock", "carla"], default="mock")
    ap.add_argument("--algos", default="cem,straddle")
    ap.add_argument("--budget", type=int, default=9)
    ap.add_argument("--batch", type=int, default=3)
    ap.add_argument("--seeds", type=int, default=2)
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--town", default="custom_straight")
    args = ap.parse_args()
    ctx = MC.make_ctx(args)
    outdir = f"results/{args.backend}/mining/infogap"
    os.makedirs(outdir, exist_ok=True)
    eps = ctx["cfg"].eps
    t0, rows = time.time(), []
    try:
        for a in args.algos.split(","):
            for s in range(args.seeds):
                if args.backend == "carla":
                    import carla_compositional as CCP
                    CCP.gpu_wait()
                got, _ = MC.run_one(INFOGAP, a, args.budget, s, ctx, outdir, batch=args.batch)
                rows += [scene_row(g["rec"], eps) for g in got]
    except KeyboardInterrupt:
        print("interrupted; checkpoints kept", flush=True)
    text = report(rows, args, ctx, args.budget, time.time() - t0)
    open(f"{outdir}/infogap.md", "w", encoding="utf-8", newline="\n").write(text)
    print(text)


if __name__ == "__main__":
    main()
