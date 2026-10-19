"""
Task 5 post-processing for CARLA mining (D4 repeats + recoverability maps), run from mining/:

    cd mining && PYTHONNOUSERSITE=1 python ../scripts/mining_post.py --family corridor --d4 --figures
    ... --family rc03 ...

D4: scenes whose label-grade value V (max over policies of min over worlds of R_raw) is within 1 sigma of the threshold eps get 3 matched
repeats (same scene, fresh episodes).  If the sign of V - eps (or the R/U label) changes among original + repeats, the scene is marked
UNRESOLVED.  sigma = max(0.05 m, observed repeat spread) / q_clear (from results/carla/mining/repeat_spread.json).  The strict set
(|V - eps| <= sigma) is expected to be tiny for a deterministic simulator, so the closest-to-boundary scenes and all pair members are also
repeated (extended set, reported separately, not part of the D4 rule).
"""
import argparse
import importlib.util
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np

import mine_cases as MC

spec = importlib.util.spec_from_file_location("carla_core", os.path.join(HERE, "carla_core.py"))
cc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cc)

REPEATS = 3
EXTENDED_NEAREST = 12
COL = {"brake": "#009E73", "bypass": "#0072B2", "U": "#D55E00", "UNRESOLVED": "#7f7f7f", "INVALID_RUN": "#000000"}


# ----------------------------------------------------------------------------- pure helpers (mock-tested)
def d4_flag(v0, label0, reps, eps=0.0):
    """reps: list of (V, label_kind).  UNRESOLVED if the side of V relative to eps or the R/U kind changes anywhere in {original}+reps."""
    sides = {(v >= eps) if v is not None else None for v in [v0] + [r[0] for r in reps]}
    kinds = {label0} | {r[1] for r in reps}
    return len(sides) > 1 or len(kinds) > 1


def near_threshold(vals, sigma, eps=0.0):
    """Indices of finite values with |V - eps| <= sigma, ordered by closeness."""
    idx = [i for i, v in enumerate(vals) if v is not None and np.isfinite(v) and abs(v - eps) <= sigma]
    return sorted(idx, key=lambda i: abs(vals[i] - eps))


def load_records(outdir):
    """(records, pair members) from the mining directory, de-duplicated by theta."""
    recs, seen = [], set()
    for fn in sorted(os.listdir(outdir)):
        if fn.startswith("records_") and fn.endswith(".jsonl"):
            for line in open(f"{outdir}/{fn}"):
                r = json.loads(line)
                k = json.dumps(r["theta"], sort_keys=True)
                if k not in seen:
                    seen.add(k)
                    recs.append(r)
    pair_members = []
    for fn in sorted(os.listdir(outdir)):
        if fn.startswith("pairs_") and fn.endswith(".jsonl"):
            for line in open(f"{outdir}/{fn}"):
                p = json.loads(line)
                for role in ("unrecoverable_side", "recoverable_side"):
                    pair_members.append(dict(p[role], _pair=p["pair_id"], _role=role))
    return recs, pair_members


# ----------------------------------------------------------------------------- D4
def run_d4(fam, ctx, outdir, sigma_m, q_clear):
    sigma = sigma_m / q_clear
    recs, members = load_records(outdir)
    pool, seen = [], set()
    for r in recs + members:
        k = json.dumps(r["theta"], sort_keys=True)
        if k in seen:
            continue
        seen.add(k)
        pool.append(r)
    vals = [MC.v_raw(r) for r in pool]
    strict_idx = set(near_threshold(vals, sigma, ctx["cfg"].eps))
    fin = [i for i, v in enumerate(vals) if v is not None and np.isfinite(v)]
    nearest = sorted(fin, key=lambda i: abs(vals[i] - ctx["cfg"].eps))[:EXTENDED_NEAREST]
    member_keys = {json.dumps(m["theta"], sort_keys=True) for m in members}
    ext_idx = set(nearest) | {i for i, r in enumerate(pool) if json.dumps(r["theta"], sort_keys=True) in member_keys}
    todo = sorted(strict_idx | ext_idx, key=lambda i: abs(vals[i] - ctx["cfg"].eps) if vals[i] is not None and np.isfinite(vals[i]) else 99)
    path = f"{outdir}/d4_repeats.jsonl"
    done = {}
    if os.path.exists(path):
        for line in open(path):
            d = json.loads(line)
            done[d["key"]] = d
    out = open(path, "a")
    rows = []
    for i in todo:
        r = pool[i]
        key = json.dumps(r["theta"], sort_keys=True)
        if key not in done:
            reps = []
            for _ in range(REPEATS):
                rec = fam.run(r["theta"], ctx).finalize(ctx["cfg"].eps).to_json()
                reps.append({"V": MC.v_raw(rec), "label": rec["evidence_class"]})
            d = {"key": key, "theta": r["theta"], "V0": vals[i], "label0": r["evidence_class"], "reps": reps,
                 "physics_profile_id": ctx.get("pid")}
            out.write(json.dumps(d) + "\n"); out.flush()
            done[key] = d
        d = done[key]
        flag = d4_flag(d["V0"], MC.label_kind(d["label0"]), [(x["V"], MC.label_kind(x["label"])) for x in d["reps"]], ctx["cfg"].eps)
        rows.append({**d, "strict": i in strict_idx, "unresolved": flag})
    out.close()
    return rows, sigma, len(pool)


def d4_md(fam, rows, sigma_m, sigma, q, npool, rep):
    n_strict = sum(r["strict"] for r in rows)
    L = [f"## D4 repeats - {fam.name} (CARLA)\n",
         f"sigma = max(0.05 m, observed repeat spread) = {sigma_m:.4f} m = {sigma:.5f} in margin units (clearance metres / q_clear = {q}). "
         f"Repeat spread source: `results/carla/mining/repeat_spread.json`: same-session max {rep['same_session_max_m']:.4f} m-equivalent, "
         f"cross-session (server restart in between) max {rep['cross_session_max_m']:.4f} m-equivalent over {rep['n_scenes']} re-run RC12 scenes; "
         "cross-session repeats were NOT measured systematically (small sample, one family).\n",
         f"Pool: {npool} distinct scenes (all algorithms + pair members). Strict D4 set (|V - eps| <= sigma): **{n_strict}** scenes. "
         f"Extended set (the {EXTENDED_NEAREST} closest scenes plus all pair members, repeated for evidence; not part of the D4 rule): {len(rows) - n_strict} more scenes. "
         f"Each scene: original + {REPEATS} matched repeats (same scene, fresh episodes, same session).  "
         f"**UNRESOLVED** (sign of V - eps or R/U label changes): {sum(r['unresolved'] for r in rows)} of {len(rows)}.\n",
         "| set | theta | V (orig) | V (repeats) | label | UNRESOLVED |", "|---|---|---|---|---|---|"]
    for r in sorted(rows, key=lambda r: (not r["strict"], abs(r["V0"]) if r["V0"] is not None and np.isfinite(r["V0"]) else 99)):
        th = ", ".join(f"{MC.short(k)} {v:.3f}" for k, v in r["theta"].items())
        f = lambda x: "None" if x is None else ("-inf" if x == -np.inf else f"{x:.5f}")
        L.append(f"| {'strict' if r['strict'] else 'extended'} | {th} | {f(r['V0'])} | {', '.join(f(x['V']) for x in r['reps'])} | {r['label0']} | {r['unresolved']} |")
    return "\n".join(L) + "\n"


# ----------------------------------------------------------------------------- figures
def figures(fam, outdir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    recs, members = load_records(outdir)
    allr = recs + [m for m in members]
    seen, pts = set(), []
    for r in allr:
        k = json.dumps(r["theta"], sort_keys=True)
        if k not in seen:
            seen.add(k)
            pts.append(r)
    if fam.name == "corridor":
        fig, ax = plt.subplots(1, 2, figsize=(12, 4.8))
        for lab, name, mk in (("R_REF", "brake", "o"), ("U", "U", "x"), ("UNRESOLVED", "UNRESOLVED", "s"), ("INVALID_RUN", "INVALID_RUN", "d")):
            sel = [r for r in pts if MC.label_kind(r["evidence_class"]) == (lab if lab != "R_REF" else "R")]
            ax[0].scatter([r["theta"]["gap_m"] for r in sel], [r["theta"]["v_mps"] for r in sel], c=COL[name], marker=mk, s=28,
                          label=f"{'R_REF (recoverable)' if lab == 'R_REF' else lab + (' (U_*)' if lab == 'U' else '')} n={len(sel)}")
        from families import B_CAL, TAU_PLANT
        v = np.linspace(6, 20, 100)
        c = 1.0
        lo = [v * TAU_PLANT + v ** 2 / (2 * B_CAL) + c, v * (TAU_PLANT + 1.2) + v ** 2 / (2 * B_CAL) + c]
        ax[0].plot(lo[0], v, "k--", lw=1, label="analytic boundary, tau + prefix = 0")
        ax[0].plot(lo[1], v, "k:", lw=1, label="analytic boundary, tau + prefix = 1.2 s (max)")
        ax[0].fill_betweenx(v, lo[0], lo[1], color="0.9", zorder=0)
        ax[0].set_xlim(8, 40); ax[0].set_ylim(6, 20)
        ax[0].set_xlabel("gap (m)"); ax[0].set_ylabel("speed (m/s)")
        ax[0].set_title("Recoverability map (corridor, CARLA): gap x v\nall mined scenes, tau/prefix projected out")
        ax[0].legend(fontsize=7, loc="upper left")
        for lab, name in (("R", "brake"), ("U", "U"), ("UNRESOLVED", "UNRESOLVED")):
            sel = [r for r in pts if MC.label_kind(r["evidence_class"]) == lab and r.get("_anchor_G_m") is not None and MC.v_raw(r) is not None
                   and np.isfinite(MC.v_raw(r))]
            ax[1].scatter([r["_anchor_G_m"] for r in sel], [MC.v_raw(r) for r in sel], c=COL[name], s=22, label=lab)
        ax[1].axhline(0, c="k", lw=.6); ax[1].axvline(0, c="k", lw=.6)
        ax[1].set_xlabel("analytic margin gap - v(tau_eff+prefix) - v^2/(2 b_cal) - c   (m)")
        ax[1].set_ylabel("CARLA label-grade margin V (normalised)")
        ax[1].set_title("CARLA margin vs analytic anchor\n(b_cal 6.211, tau_eff = tau + 0.05 s)")
        ax[1].legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(f"{outdir}/map_corridor_gap_v.png", dpi=140)
        plt.close(fig)
    elif fam.name == "rc03":
        fig, ax = plt.subplots(figsize=(7.5, 5.5))
        groups = {"U": ([], "x", "U_* (no policy passes)"), "brake": ([], "o", "R_REF, brake_hold passes"),
                  "bypass": ([], "*", "R_REF, bypass only (brake_hold fails): two-lane witness"), "UNRESOLVED": ([], "s", "UNRESOLVED")}
        for r in pts:
            k = MC.label_kind(r["evidence_class"])
            if k == "R":
                m = cc.margins(r)
                k = "bypass" if cc.rc03_contrast(r) else "brake"
            if k in groups:
                groups[k][0].append(r)
        for k, (sel, mk, lab) in groups.items():
            ax.scatter([r["theta"]["gap_m"] for r in sel], [r["theta"]["v_mps"] for r in sel], c=COL[k], marker=mk, s=70 if k == "bypass" else 30,
                       label=f"{lab} n={len(sel)}")
        from families import B_CAL, TAU_PLANT
        v = np.linspace(8, 24, 100)
        for tau, ls in ((0.0, "--"), (0.5, ":")):
            ax.plot(v * (tau + TAU_PLANT) + v ** 2 / (2 * B_CAL) + 1.0, v, "k" + ls, lw=1, label=f"brake-only analytic boundary, tau {tau:g} s")
        ax.set_xlim(10, 60); ax.set_ylim(8, 24)
        ax.set_xlabel("gap (m)"); ax.set_ylabel("speed (m/s)")
        ax.set_title("Recoverability map (two-lane RC03, CARLA, a_lat_max 8, lc_time 2.0)\nall mined scenes, tau projected out")
        ax.legend(fontsize=7, loc="upper left")
        fig.tight_layout()
        fig.savefig(f"{outdir}/map_rc03_gap_v.png", dpi=140)
        plt.close(fig)
    return len(pts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--family", choices=["corridor", "rc03"], required=True)
    ap.add_argument("--d4", action="store_true")
    ap.add_argument("--figures", action="store_true")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--town", default="custom_straight")
    a = ap.parse_args()
    a.backend = "carla"
    fam = MC.get_family(a)
    outdir = f"results/carla/mining/{fam.name}"
    if a.figures:
        print("figure scenes:", figures(fam, outdir))
    if a.d4:
        rep = json.load(open("results/carla/mining/repeat_spread.json"))
        sigma_m = max(0.05, rep["same_session_max_m"], rep["cross_session_max_m"])
        ctx = MC.make_ctx(a)
        q = ctx["cfg"].q_clear
        rows, sigma, npool = run_d4(fam, ctx, outdir, sigma_m, q)
        text = d4_md(fam, rows, sigma_m, sigma, q, npool, rep)
        open(f"{outdir}/d4.md", "w", encoding="utf-8", newline="\n").write(text)
        print(text)


if __name__ == "__main__":
    main()
