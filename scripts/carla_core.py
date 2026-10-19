"""
Task C2: core cases on CARLA (RC03, RC07, RC12, RC19, RC20).

    cd mining && PYTHONNOUSERSITE=1 python ../scripts/carla_core.py [--host localhost --port 2000] [--case RC03 ...]

Needs a live, bring-up-passed server (never started here).  Every point is a full ExecutionRecord checkpointed to
mining/results/carla/core/<sweep>.jsonl (a rerun skips finished points); summary mining/results/carla/core.md.
Labels come from evidence only (sim.harness.evaluate -> contract.classify); eps/reserve are margin.DEFAULT_V2, untouched.
"""
import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "mining"))

import margin as M
from cases import corridor as CO
from cases import two_lane as TL

OUT = "results/carla/core"
DECEL_TARGET = 6.19
CFG = M.DEFAULT_V2
C = CFG.reserve_m
TAU = 0.25
B_CAL, TAU_EFF = 6.211, 0.300          # Task 3 measurements (shared calibration trace, not an independent prediction)
SPEEDS = (8.0, 11.0, 14.0)
RC03_LC = 1.0
LIB_FULL, LIB_RESTRICTED = "rc03_carla_lc1.0_full", "rc03_carla_lc1.0_restricted"
RC07_LIBRARY_ID = "rc07_v1"


# ----------------------------------------------------------------------------- pure helpers (mock-tested)
def gval(x):
    """Decode an R_raw/V JSON value: '-inf' -> -inf, None -> None (unrun), else float."""
    return -math.inf if x == "-inf" else (None if x is None else float(x))


def margins(r):
    return {p: gval(row[0]) for p, row in zip(r["policies"], r["R_raw"])}


def rc03_contrast(r, eps=CFG.eps):
    """brake_hold below eps and at least one bypass-type policy at/above eps (RC03 positive)."""
    m = margins(r)
    bh = m.get("brake_hold")
    byp = [v for k, v in m.items() if k.startswith(("bypass", "brake_steer")) and v is not None]
    return bh is not None and bh < eps and any(v >= eps for v in byp)


def rc07_contrast(r, eps=CFG.eps):
    m = margins(r)
    return all(m.get(k) is not None for k in ("bypass_then_stop", "bypass_then_return", "brake_hold")) and \
        m["bypass_then_stop"] < eps and m["bypass_then_return"] >= eps


def middle_of_window(keys, flags):
    """Median key among the keys whose flag is True (None if empty)."""
    ok = sorted(k for k, f in zip(keys, flags) if f)
    return ok[len(ok) // 2] if ok else None


def feasible_set_summary(taus, feas):
    """Feasible-delay set, its upper end, and whether it is a down-set (feasible iff delay <= upper)."""
    s = sorted(t for t, f in zip(taus, feas) if f)
    up = max(s) if s else None
    contiguous = all(f == (t <= up) for t, f in zip(taus, feas)) if s else not any(feas)
    return {"set": s, "upper": up, "downset": contiguous}


def t_p_star(gap, v, b=B_CAL, tau=TAU_EFF, c=C):
    """Latest baseline-keep time before braking can still stop with reserve: (gap - D_stop - c)/v, D_stop = v^2/(2b) + v*tau."""
    d_stop = v * v / (2 * b) + v * tau
    return (gap - d_stop - c) / v


def profile_max_abs_diff(a, b):
    """max |a_i - b_i| over matching finite margins; -inf must match -inf; None entries must match None."""
    d = 0.0
    for x, y in zip(a, b):
        if x is None or y is None:
            if x is not y:
                return math.inf
        elif math.isinf(x) or math.isinf(y):
            if x != y:
                return math.inf
        else:
            d = max(d, abs(x - y))
    return d


# ----------------------------------------------------------------------------- CARLA context
class Ctx:
    def __init__(self, host, port):
        from sim.bringup import connect, make_plant
        from sim.plant_carla import runtime_info
        self.prof = json.load(open("results/carla/physics_profile.json"))
        self.client, self.world, self.site = connect(host, port, "custom_straight")
        self.runtime = dict(runtime_info(self.client, self.world, self.site), physics_profile=self.prof["id"], port=port)
        self.factory = lambda: make_plant(self.client, self.world, self.site, decel=DECEL_TARGET)
        self.pid = self.prof["id"]
        self.cache = {}

    def load(self, sweep):
        path = f"{OUT}/{sweep}.jsonl"
        d = {}
        if os.path.exists(path):
            for line in open(path):
                r = json.loads(line)
                if r.get("physics_profile_id") != self.pid:
                    raise RuntimeError(f"{path}: cached record has physics_profile_id {r.get('physics_profile_id')} != current "
                                       f"{self.pid}; refusing to mix profiles (move the old file aside)")
                d[r["_key"]] = r
        self.cache[sweep] = d
        return d

    def point(self, sweep, key, fn, **extra):
        d = self.cache.get(sweep) or self.load(sweep)
        if key in d:
            return d[key]
        rec = fn()
        r = rec.to_json()
        r["_key"] = key
        r.update(extra)
        with open(f"{OUT}/{sweep}.jsonl", "a") as f:
            f.write(json.dumps(r) + "\n")
        d[key] = r
        return r

    def tl(self, case, theta, **kw):
        return TL.run(case, self.factory, theta, CFG, "carla", "carla_finite_replay", physics_profile_id=self.pid,
                      runtime=self.runtime, **kw)


# ----------------------------------------------------------------------------- RC03
def rc03_scan(ctx, v, gaps):
    name = f"rc03_v{v:g}"
    out = {}
    for g in gaps:
        th = {"gap_m": float(g), "v_mps": v, "tau_s": TAU}
        out[g] = ctx.point(name, f"{g:.3f}", lambda th=th: ctx.tl("RC03", th, library=TL.rc03_library(True, RC03_LC)))
    return out


def rc03_pick(scan):
    """Window of gaps with the RC03 contrast; pick the middle one (robust to the window edges)."""
    gaps = sorted(scan)
    flags = [rc03_contrast(scan[g]) for g in gaps]
    return middle_of_window(gaps, flags), [g for g, f in zip(gaps, flags) if f]


def rc03_closed(ctx, v, g):
    th = {"gap_m": float(g), "v_mps": v, "tau_s": TAU, "closed": True}
    return ctx.point(f"rc03_closed_v{v:g}", f"{g:.3f}", lambda: ctx.tl("RC03", th, library=TL.rc03_library(True, RC03_LC)))


def run_rc03(ctx, gaps_by_v=None):
    res = {}
    for v in SPEEDS:
        gaps = (gaps_by_v or {}).get(v) or [float(g) for g in range(6, 23)]
        scan = rc03_scan(ctx, v, gaps)
        g, window = rc03_pick(scan)
        closed = rc03_closed(ctx, v, g) if g is not None else None
        res[v] = {"scan": scan, "gap": g, "window": window, "closed": closed}
        print(f"RC03 v={v}: window {window} pick {g}", flush=True)
    return res


# ----------------------------------------------------------------------------- RC07
def rc07_library():
    from sim import policies as P
    return TL.rc07_library() + [lambda: P.Bypass(side=-1, lc_time=2.0)]      # + primary-only avoidance (no hand-back)


class SpyPlant:
    """Transparent plant wrapper that logs the realised speed of every actor each tick (measured rear speed)."""
    def __init__(self, plant, log):
        self._p, self.log = plant, log

    def __getattr__(self, k):
        return getattr(self._p, k)

    def step(self, controls):
        snap = self._p.step(controls)
        self.log.append({n: (a.x, a.lat, a.v) for n, a in snap.actors.items()})
        return snap


def rc07_rec(ctx, th, spy=None):
    fac = ctx.factory if spy is None else (lambda: SpyPlant(ctx.factory(), spy))
    return TL.run("RC07", fac, th, CFG, "carla", "carla_finite_replay", library=rc07_library(),
                  physics_profile_id=ctx.pid, runtime=ctx.runtime)


def rc07_theta(gap, rg, rv):
    return {"gap_m": float(gap), "v_mps": 12.0, "tau_s": TAU, "rear_gap_m": float(rg), "rear_v_mps": float(rv)}


def rc07_scan(ctx, name, grid):
    out = {}
    for gap, rg, rv in grid:
        th = rc07_theta(gap, rg, rv)
        out[(gap, rg, rv)] = ctx.point(name, f"{gap}_{rg}_{rv}", lambda th=th: rc07_rec(ctx, th))
        m = margins(out[(gap, rg, rv)])
        print("RC07", gap, rg, rv, out[(gap, rg, rv)]["evidence_class"],
              {k: (None if x is None else round(x, 2)) for k, x in m.items()}, flush=True)
    return out


def run_rc07(ctx):
    res = {"scan": rc07_scan(ctx, "rc07_scan", [(g, rg, rv) for g in (26, 34) for rg in (40, 60, 80, 100) for rv in (18, 24)])}
    sel = (26, 60, 24)                      # first coarse-scan setting with the full contrast (see core.md)
    th = rc07_theta(*sel)
    spy = []
    r = ctx.point("rc07_sel", "sel", lambda: rc07_rec(ctx, th, spy), library_id=RC07_LIBRARY_ID)
    if not spy:                              # cached: re-measure the rear speed with one episode of bypass_then_return
        from sim import policies as P
        TL.run("RC07", lambda: SpyPlant(ctx.factory(), spy), th, CFG, "carla", "carla_finite_replay",
               library=[lambda: P.BypassThenReturn(lc_time=2.0)], physics_profile_id=ctx.pid, runtime=ctx.runtime)
    rear = [x["rear"][2] for x in spy if "rear" in x]
    res["sel"], res["sel_theta"] = r, th
    res["rear_v"] = {"nominal": th["rear_v_mps"], "start": rear[0], "max": max(rear), "mean": sum(rear) / len(rear), "n": len(rear)}
    res["no_rear"] = ctx.point("rc07_norear", "sel", lambda: rc07_rec(ctx, rc07_theta(26, 100, 18)))
    return res


# ----------------------------------------------------------------------------- RC12
def rc12_rec(ctx, prefix):
    th = {"gap_m": 40.0, "v_mps": 12.0, "tau_s": TAU, "prefix_s": float(prefix)}
    return CO.run(ctx.factory, th, CFG, "carla", "carla_finite_replay", case_id="RC12", with_bypass=False,
                  physics_profile_id=ctx.pid, runtime=ctx.runtime)


def run_rc12(ctx):
    name = "rc12_prefix"
    feas = lambda r: r["evidence_class"] == "R_REF"
    pre = [round(0.25 * i, 2) for i in range(0, 13)]
    for p in pre:
        ctx.point(name, f"{p:.3f}", lambda p=p: rc12_rec(ctx, p))
    d = ctx.cache[name]
    up = max(p for p in pre if feas(d[f"{p:.3f}"]))
    for p in [round(up + 0.05 * i, 2) for i in range(1, 6)]:        # refine on the 0.05 s tick grid above the last feasible coarse point
        ctx.point(name, f"{p:.3f}", lambda p=p: rc12_rec(ctx, p))
    rows = sorted(d.values(), key=lambda r: r["theta"]["prefix_s"])
    return {"rows": rows, "summary": feasible_set_summary([r["theta"]["prefix_s"] for r in rows], [feas(r) for r in rows])}


# ----------------------------------------------------------------------------- RC19
def run_rc19(ctx):
    out = {}
    for v in (8.0, 12.0):
        th = {"v_mps": v}
        a = ctx.point("rc19", f"anchor_{v:g}", lambda th=th: ctx.tl("RC19", th))
        b = ctx.point("rc19", f"nuisance_{v:g}", lambda th=th: ctx.tl("RC19", th, spec=TL.rc19_spec(th, CFG, nuisance=True)))
        ga, gb = [gval(x[0]) for x in a["R_raw"]], [gval(x[0]) for x in b["R_raw"]]
        out[v] = {"anchor": a, "nuisance": b, "dV": profile_max_abs_diff(ga, gb), "ga": ga, "gb": gb}
    return out


# ----------------------------------------------------------------------------- RC20
def run_rc20(ctx, rc03):
    out = {}
    for v in (11.0, 14.0):
        g = rc03[v]["gap"]
        th = {"gap_m": float(g), "v_mps": v, "tau_s": TAU}
        spec = TL.rc03_spec(th, CFG)
        rest = lambda: TL.rc03_library(False, RC03_LC)
        a = ctx.point("rc20", f"restricted_{v:g}", lambda: ctx.tl("RC03", th, library=rest(), spec=spec), library_id=LIB_RESTRICTED)
        b = ctx.point("rc20", f"budget_{v:g}", lambda: ctx.tl("RC03", th, library=rest(), spec=spec, run_mask={(0, 0)}),
                      library_id=LIB_RESTRICTED + "+budget1")
        c = ctx.point("rc20", f"expanded_{v:g}", lambda: ctx.tl("RC03", th, library=TL.rc03_library(True, RC03_LC), spec=spec),
                      library_id=LIB_FULL)
        out[v] = {"theta": th, "restricted": a, "budget": b, "expanded": c,
                  "same_spec": a["geometry"]["spec"] == c["geometry"]["spec"] == b["geometry"]["spec"]}
    return out


def validate_all():
    import validate_records as VR
    bad, n = [], 0
    for fn in sorted(os.listdir(OUT)):
        if not fn.endswith(".jsonl") or "superseded" in fn or fn.startswith("probe"):
            continue
        for line in open(f"{OUT}/{fn}"):
            r = json.loads(line)
            n += 1
            e = VR.check_record(r)
            if e:
                bad.append((fn, r.get("_key"), e))
    return n, bad


def fm(x):
    if x is None:
        return "None"
    return "-inf" if x == -math.inf else f"{x:.3f}"


def mrow(r):
    return ", ".join(f"{p}={fm(v)}" for p, v in margins(r).items())


def term(r, pol):
    for k, ev in r["events"].items():
        if k.startswith(pol + "/"):
            return ev.get("termination"), sorted({e.get("other") for e in ev.get("events", [])} - {None})
    return None, []


def report(ctx, r03, r07, r12, r19, r20):
    L = []
    w = L.append
    w("# CARLA core cases (Task C2: RC03, RC07, RC12, RC19, RC20)\n")
    w(f"Physics profile `{ctx.pid}`, decel target {DECEL_TARGET}, eps = {CFG.eps}, reserve c = {C} m (margin.DEFAULT_V2, unchanged), act delay "
      f"{TAU} s. Records: `results/carla/core/*.jsonl`. Labels are `contract.classify` output only.\n")
    w("## RC03: braking fails, bypass passes (v in 8, 11, 14)\n")
    w(f"**Library revision (reported, not relabelled).** The mock library's 2.0 s lane change never beats CARLA braking (b_cal = {B_CAL}; "
      "superseded `rc03_v*_lc2_superseded.jsonl`: bypass only passes at gaps where brake_hold already passes).  The CARLA library "
      f"(id `{LIB_FULL}`) uses an emergency lane change lc_time = {RC03_LC} s (quintic profile): brake_hold, lane_follow, bypass_left, "
      "brake_steer_left (0.4 s brake first).  Bypass margins are small (0.02-0.05): left-lane clearance only just exceeds the reserve.\n")
    w("| v | gap scan | window (brake_hold<eps and a bypass>=eps) | chosen gap | brake_hold G | best bypass G | evidence | closed-lane evidence | closed margins |")
    w("|---|---|---|---|---|---|---|---|---|")
    for v, d in r03.items():
        rng = f"{min(d['scan']):g}..{max(d['scan']):g}"
        if d["gap"] is None:
            w(f"| {v:g} | {rng} | NONE | - | - | - | - | - | - |")
            continue
        r = d["scan"][d["gap"]]
        m = margins(r)
        byp = max(x for k, x in m.items() if k.startswith(("bypass", "brake_steer")))
        c = d["closed"]
        w(f"| {v:g} | {rng} | {d['window']} | {d['gap']:g} | {fm(m['brake_hold'])} | {fm(byp)} | {r['evidence_class']} | "
          f"{c['evidence_class']} | {mrow(c)} |")
    w("\nv = 8 m/s: **mechanism NOT shown on CARLA**, excluded (nothing relabelled).  Braking stops from 8 m/s in ~8 m; the bypass first becomes "
      "finite only at the gap where brake_hold already passes (scan below).  Closed pairs run only where a positive exists.  "
      "Closing the escape lane gives a finite-library negative: U_LIBRARY, never U_MODEL, never pre-labelled.\n")
    for v, d in r03.items():
        w(f"\nScan v = {v:g}:\n\n| gap | evidence | margins |\n|---|---|---|")
        for g in sorted(d["scan"]):
            w(f"| {g:g} | {d['scan'][g]['evidence_class']} | {mrow(d['scan'][g])} |")
    w("\n## RC07: premature hand-back vs continuation\n")
    w(f"Library `{RC07_LIBRARY_ID}` = bypass_then_stop, bypass_then_return, brake_hold, bypass_left (primary-only avoidance).  v = 12, rear actor in the left lane.\n")
    w("| gap | rear_gap | rear_v | evidence | margins |\n|---|---|---|---|---|")
    for k, r in r07["scan"].items():
        w(f"| {k[0]} | {k[1]} | {k[2]} | {r['evidence_class']} | {mrow(r)} |")
    t, r, rv = r07["sel_theta"], r07["sel"], r07["rear_v"]
    w(f"\nSelected: gap {t['gap_m']:g}, rear_gap {t['rear_gap_m']:g}, rear_v {t['rear_v_mps']:g} (first coarse-scan setting with the contrast, not tuned).  "
      f"Evidence **{r['evidence_class']}**; margins: {mrow(r)}.\n")
    for pol in ("bypass_then_stop", "bypass_then_return", "bypass_left", "brake_hold"):
        tm, oth = term(r, pol)
        w(f"- {pol}: termination {tm}, contacted {oth or 'none'}")
    w(f"\nMeasured rear speed (per-tick spy, {rv['n']} ticks): nominal {rv['nominal']:g}, at start {rv['start']:.2f}, max {rv['max']:.2f}, mean {rv['mean']:.2f} m/s.")
    nr = r07["no_rear"]
    w(f"\nPrimary-only avoidance passes the obstacle: with a far/slow rear actor (rear_gap 100, rear_v 18) bypass_left = {fm(margins(nr)['bypass_left'])}; "
      f"in the selected scene its failure is a contact with the rear actor {term(r, 'bypass_left')}, not the blocker.  brake_hold passes in place at gap 26 m, so the scene stays {r['evidence_class']}.\n")
    w("## RC12: baseline-keep prefix before braking (corridor, gap 40, v 12, tau 0.25, brake_hold@prefix)\n")
    s, tp = r12["summary"], t_p_star(40.0, 12.0)
    w("| prefix_s | brake_hold G | evidence |\n|---|---|---|")
    for r in r12["rows"]:
        w(f"| {r['theta']['prefix_s']:.2f} | {fm(gval(r['R_raw'][0][0]))} | {r['evidence_class']} |")
    w(f"\nFeasible-prefix set (R_REF): {s['set']}; upper end {s['upper']}; down-set (feasible iff prefix <= upper): {s['downset']}.")
    w(f"Analytic T_p* = (gap - D_stop - c)/v, D_stop = v^2/(2 b_cal) + v tau_eff, b_cal = {B_CAL}, tau_eff = {TAU_EFF}: **{tp:.3f} s**.  "
      "The prefix is quantised to the 0.05 s tick (step function), so agreement means within one tick.  b_cal/tau_eff come from the same "
      "brake_hold calibration trace (Task 3): this checks structure, not an independent prediction; the ego also runs at ~11.89 m/s, not 12.")
    if s["upper"] is not None:
        w(f"\nCARLA upper end - T_p* = {s['upper'] - tp:+.3f} s.\n")
    w("## RC19: benign anchor vs nuisance twin\n")
    w("| v | anchor G | nuisance G | |dV| (max abs profile diff) | evidence anchor / twin |\n|---|---|---|---|---|")
    for v, d in r19.items():
        w(f"| {v:g} | {[fm(x) for x in d['ga']]} | {[fm(x) for x in d['gb']]} | {d['dV']:.4f} | {d['anchor']['evidence_class']} / {d['nuisance']['evidence_class']} |")
    w("\n## RC20: library completeness (same RC03 scene, physics unchanged)\n")
    w(f"Restricted `{LIB_RESTRICTED}` (brake_hold, lane_follow); budget-limited = restricted with run_mask {{(0,0)}} (library id +budget1); expanded `{LIB_FULL}`; policy version `sim.policies.VERSION` in each descriptor.\n")
    w("| v | gap | restricted | budget-limited (R_raw) | expanded | identical physical spec |\n|---|---|---|---|---|---|")
    for v, d in r20.items():
        w(f"| {v:g} | {d['theta']['gap_m']:g} | {d['restricted']['evidence_class']} ({d['restricted']['library_id']}) | {d['budget']['evidence_class']}, {d['budget']['R_raw']} | "
          f"{d['expanded']['evidence_class']} ({d['expanded']['library_id']}) | {d['same_spec']} |")
    n, bad = validate_all()
    w(f"\n## Record validation\n\n{n} records in `results/carla/core/` (excluding superseded/probe files) checked with validate_records.check_record: {len(bad)} failures.")
    for b in bad[:10]:
        w(f"- {b}")
    return "\n".join(L)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--case", nargs="*", default=["RC03", "RC07", "RC12", "RC19", "RC20"])
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    ctx = Ctx(a.host, a.port)
    r03 = run_rc03(ctx, {8.0: [6.0, 6.5, 7.0, 7.5, 8.0, 8.5, 9.0, 10.0]})
    r07, r12, r19 = run_rc07(ctx), run_rc12(ctx), run_rc19(ctx)
    r20 = run_rc20(ctx, r03)
    text = report(ctx, r03, r07, r12, r19, r20)
    open("results/carla/core.md", "w", encoding="utf-8").write(text)
    print(text)


if __name__ == "__main__":
    main()
