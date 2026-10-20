"""
Task C2: core cases on CARLA (RC03, RC07, RC12, RC19, RC20).

    cd mining && PYTHONNOUSERSITE=1 python ../scripts/carla_core.py [--host localhost --port 2000]

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
SPEEDS = (8.0, 11.0, 14.0, 18.0, 22.0)   # 8/11/14 are the brief's speeds; 18/22 added because no window exists at 8-14 under the a_lat limit
A_LAT_MAX = 8.0                        # m/s^2, controller ruling: declared in the RC03/RC07/RC20 CARLA specs (harness dyn-limit margin)
LC_CANDIDATES = (1.0, 1.2, 1.4, 1.6, 2.0)   # lane-change durations searched; the largest one with a window under the limit is used
RC19_TOL = 0.05                        # pre-declared |dV| tolerance (margins are normalised, unit-free), fixed before any run
RC07_LIBRARY_ID = f"rc07_v2_alat{A_LAT_MAX:g}_fixplant"      # re-run on the plant with moving actors pinned to their declared speed in warm-up (Task 6 fix round 1)
TAG_HL = f"al{A_LAT_MAX:g}hl"          # RC03/RC20 sweeps re-run after _obstacle_passed began using the delivered ego_hl (2.3959 m on CARLA, not the literal 2.3)
TAG = f"al{A_LAT_MAX:g}"               # part of every limited sweep name / key, so a limit or library change cannot reuse stale records


def lib_id(kind, lc):
    return f"rc03_carla_lc{lc:g}_{kind}_alat{A_LAT_MAX:g}"


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
    """Full RC07 acceptance in ONE scene: primary-only avoidance (bypass_left) passes, the premature hand-back (bypass_then_stop)
    is refuted, the continuation (bypass_then_return) passes."""
    m = margins(r)
    if any(m.get(k) is None for k in ("bypass_left", "bypass_then_stop", "bypass_then_return")):
        return False
    return m["bypass_left"] >= eps and m["bypass_then_stop"] < eps and m["bypass_then_return"] >= eps


def peak_alat(r, policy, limit=A_LAT_MAX):
    """Peak |a_lat| of a policy's episode from the recorded dynamic-limit margin: dyn = (limit - peak)/limit."""
    for k, tr in r["traces"].items():
        if k.startswith(policy + "/"):
            dyn = tr["margin"]["parts"]["dyn"]
            return None if dyn is None or dyn in ("inf", "Infinity") or math.isinf(float(dyn)) else limit * (1.0 - float(dyn))
    return None


def with_limit(spec, limit=A_LAT_MAX):
    return dict(spec, a_lat_max=limit)


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
def rc03_th(v, g, closed=False):
    th = {"gap_m": float(g), "v_mps": v, "tau_s": TAU}
    return dict(th, closed=True) if closed else th


def rc03_rec(ctx, th, lc, with_bypass=True, **kw):
    return ctx.tl("RC03", th, library=TL.rc03_library(with_bypass, lc), spec=with_limit(TL.rc03_spec(th, CFG)), **kw)


def rc03_scan(ctx, v, lc, gaps):
    name = f"rc03_v{v:g}_lc{lc:g}_{TAG_HL}"
    return {g: ctx.point(name, f"{g:.3f}", lambda g=g: rc03_rec(ctx, rc03_th(v, g), lc), library_id=lib_id("full", lc), lc_time=lc,
                         a_lat_max=A_LAT_MAX) for g in gaps}


def rc03_pick(scan):
    """Window of gaps with the RC03 contrast; pick the middle one (robust to the window edges)."""
    gaps = sorted(scan)
    flags = [rc03_contrast(scan[g]) for g in gaps]
    return middle_of_window(gaps, flags), [g for g, f in zip(gaps, flags) if f]


GAP_RANGE = {8.0: [7.0 + 0.5 * i for i in range(9)], 11.0: [float(g) for g in range(10, 21)], 14.0: [float(g) for g in range(14, 27)],
             18.0: [float(g) for g in range(24, 51, 2)], 22.0: [float(g) for g in range(30, 57, 2)]}


def run_rc03(ctx):
    """Per speed: scan gaps for each lc_time; rule = use the LARGEST lc_time (least aggressive manoeuvre) whose scan has a window
    (brake_hold < eps and a bypass >= eps under the a_lat limit); gap = median of that window."""
    res = {}
    for v in SPEEDS:
        per_lc = {}
        for lc in sorted(LC_CANDIDATES, reverse=True):
            scan = rc03_scan(ctx, v, lc, GAP_RANGE[v])
            g, window = rc03_pick(scan)
            per_lc[lc] = {"scan": scan, "gap": g, "window": window}
            print(f"RC03 v={v} lc={lc}: window {window}", flush=True)
        ok = [lc for lc in per_lc if per_lc[lc]["gap"] is not None]
        lc = max(ok) if ok else None
        closed = None
        if lc is not None:
            g = per_lc[lc]["gap"]
            th = rc03_th(v, g, True)
            closed = ctx.point(f"rc03_closed_v{v:g}_{TAG_HL}", f"{lc:g}_{g:.3f}", lambda: rc03_rec(ctx, th, lc), library_id=lib_id("full", lc),
                               lc_time=lc, a_lat_max=A_LAT_MAX)
        res[v] = {"per_lc": per_lc, "lc": lc, "gap": per_lc[lc]["gap"] if lc else None, "window": per_lc[lc]["window"] if lc else [],
                  "scan": per_lc[lc]["scan"] if lc else per_lc[max(per_lc)]["scan"], "closed": closed}
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
    return TL.run("RC07", fac, th, CFG, "carla", "carla_finite_replay", library=rc07_library(), spec=with_limit(TL.rc07_spec(th, CFG)),
                  physics_profile_id=ctx.pid, runtime=ctx.runtime)


def rc07_theta(gap, rg, rv):
    return {"gap_m": float(gap), "v_mps": 12.0, "tau_s": TAU, "rear_gap_m": float(rg), "rear_v_mps": float(rv)}


def rc07_scan(ctx, name, grid):
    out = {}
    for gap, rg, rv in grid:
        th = rc07_theta(gap, rg, rv)
        out[(gap, rg, rv)] = ctx.point(name, f"{gap}_{rg}_{rv}", lambda th=th: rc07_rec(ctx, th), library_id=RC07_LIBRARY_ID,
                                       a_lat_max=A_LAT_MAX)
        m = margins(out[(gap, rg, rv)])
        print("RC07", gap, rg, rv, out[(gap, rg, rv)]["evidence_class"],
              {k: (None if x is None else round(x, 2)) for k, x in m.items()}, flush=True)
    return out


def rc07_select(scan):
    """Selection rule: among scenes with the FULL contrast (rc07_contrast), take the highest rear_v; within it the median of the
    remaining scenes ordered by (gap, rear_gap).  Returns (key, all full-contrast keys)."""
    full = sorted(k for k, r in scan.items() if rc07_contrast(r))
    if not full:
        return None, []
    vmax = max(k[2] for k in full)
    cand = [k for k in full if k[2] == vmax]
    return cand[len(cand) // 2], full


def run_rc07(ctx):
    grid = [(g, rg, rv) for g in (26, 34) for rg in (40, 60, 80, 100) for rv in (18, 21, 24)]
    scan = rc07_scan(ctx, f"rc07_scan_{TAG}_fixplant", grid)
    sel, full = rc07_select(scan)
    res = {"scan": scan, "sel": None, "full": full, "fast_rear_full": sorted(k for k in full if k[2] >= 24)}
    if sel is None:
        return res
    th = rc07_theta(*sel)
    spy = []
    r = ctx.point(f"rc07_sel_{TAG}_fixplant", "_".join(map(str, sel)), lambda: rc07_rec(ctx, th, spy), library_id=RC07_LIBRARY_ID, a_lat_max=A_LAT_MAX)
    if not spy:                              # cached: re-measure the rear speed with one episode of bypass_then_return
        from sim import policies as P
        TL.run("RC07", lambda: SpyPlant(ctx.factory(), spy), th, CFG, "carla", "carla_finite_replay",
               library=[lambda: P.BypassThenReturn(lc_time=2.0)], spec=with_limit(TL.rc07_spec(th, CFG)),
               physics_profile_id=ctx.pid, runtime=ctx.runtime)
    rear = [x["rear"][2] for x in spy if "rear" in x]
    res["sel"], res["sel_theta"] = r, th
    res["rear_v"] = {"nominal": th["rear_v_mps"], "start": rear[0], "max": max(rear), "mean": sum(rear) / len(rear), "n": len(rear)}
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
    for v in SPEEDS:
        lc, g = rc03[v]["lc"], rc03[v]["gap"]
        if g is None:
            continue
        th = rc03_th(v, g)
        spec = with_limit(TL.rc03_spec(th, CFG))
        rest = lambda: TL.rc03_library(False, lc)
        x = dict(lc_time=lc, a_lat_max=A_LAT_MAX)
        a = ctx.point(f"rc20_{TAG_HL}", f"restricted_{v:g}_{lc:g}_{g:g}", lambda: ctx.tl("RC03", th, library=rest(), spec=spec),
                      library_id=lib_id("restricted", lc), **x)
        b = ctx.point(f"rc20_{TAG_HL}", f"budget_{v:g}_{lc:g}_{g:g}", lambda: ctx.tl("RC03", th, library=rest(), spec=spec, run_mask={(0, 0)}),
                      library_id=lib_id("restricted", lc) + "+budget1", **x)
        c = ctx.point(f"rc20_{TAG_HL}", f"expanded_{v:g}_{lc:g}_{g:g}", lambda: ctx.tl("RC03", th, library=TL.rc03_library(True, lc), spec=spec),
                      library_id=lib_id("full", lc), **x)
        out[v] = {"theta": th, "lc": lc, "restricted": a, "budget": b, "expanded": c,
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


def best_bypass(r):
    m = {k: x for k, x in margins(r).items() if k.startswith(("bypass", "brake_steer")) and x is not None}
    k = max(m, key=lambda k: m[k])
    return k, m[k]


def fp(x):
    return "n/a" if x is None else f"{x:.2f}"


def hl_rerun_note():
    """Computed from the records: RC03/RC20 were re-run under tag TAG_HL after _obstacle_passed began using the delivered ego_hl (2.3959 m on CARLA,
    from the ego bounding box, instead of the literal 2.3).  Old records are kept as *_prehl_superseded.jsonl.  Returns md text or ''."""
    n = ev = 0
    dmax, nchg, nbh = 0.0, 0, 0
    for fn_ in sorted(os.listdir(OUT)):
        if not fn_.endswith(f"_{TAG}_prehl_superseded.jsonl"):
            continue
        new = f"{OUT}/{fn_[:-len(f'_{TAG}_prehl_superseded.jsonl')]}_{TAG_HL}.jsonl"
        if not os.path.exists(new):
            continue
        old_r = {r["_key"]: r for r in map(json.loads, open(f"{OUT}/{fn_}"))}
        for r in map(json.loads, open(new)):
            o = old_r.get(r["_key"])
            if o is None:
                continue
            n += 1
            ev += o["evidence_class"] != r["evidence_class"]
            a, b = margins(o), margins(r)
            for k in a:
                if a[k] != b.get(k):
                    nchg += 1
                    nbh += k == "brake_hold"
                if a[k] is not None and b.get(k) is not None and math.isfinite(a[k]) and math.isfinite(b[k]):
                    dmax = max(dmax, abs(a[k] - b[k]))
    if not n:
        return ""
    return (f"**RC03/RC20 re-run after the ego_hl change.**  `_obstacle_passed` (commit e81aa9d) now uses the delivered `ego_hl` instead of the literal 2.3 m; "
            f"the CARLA ego (vehicle.tesla.model3) delivers 2.3959 m (bounding-box extent, so the pass test shifts by 0.096 m).  RC03 (all scans, closed-lane records) and RC20 were re-run "
            f"under the tag `{TAG_HL}`; the {n} earlier records matching a re-run key are kept as `*_{TAG}_prehl_superseded.jsonl` and are not used.  Compared key by key: "
            f"{ev} evidence-class changes; {nchg} policy margins differ at all, {nbh} of them for brake_hold, which never reads ego_hl, so the differences are CARLA run-to-run variation, not an ego_hl effect; "
            f"largest change of a finite margin {dmax:.3f}.  The RC07/RC12/RC19 sweeps were not re-run here (RC07 was re-run on the same code earlier).\n")


def report(ctx, r03, r07, r12, r19, r20):
    L = []
    w = L.append
    w("# CARLA core cases (Task C2: RC03, RC07, RC12, RC19, RC20)\n")
    w(f"Physics profile `{ctx.pid}`, decel target {DECEL_TARGET}, eps = {CFG.eps}, reserve c = {C} m (margin.DEFAULT_V2, unchanged), act delay "
      f"{TAU} s. Records: `results/carla/core/*.jsonl`. Labels are `contract.classify` output only.\n")
    w(f"**Actuation limit.** RC03, RC07 and RC20 CARLA specs declare `a_lat_max = {A_LAT_MAX:g}` m/s^2 (recorded in every record's "
      "`geometry.spec`): the harness adds a dynamic-limit margin (limit - peak |v * yaw_rate|)/limit, so a manoeuvre exceeding the limit "
      "is below eps and cannot be a witness.  Reason: the CARLA tyre friction of the profile is non-physical, so an unlimited lane change "
      "could exploit unrealistic grip (measured on CARLA at v = 14: bypass peak |a_lat| 14.7 m/s^2 at lc_time 1.0, 8.5 at 1.6, 6.0 at 2.0; per-tick |v*yaw_rate| is smooth, a 3-tick moving average changes the peak by < 2%, so no smoothing is applied).  "
      "Peak |a_lat| values below are measured from CARLA traces (peak = limit * (1 - dyn margin)).  The earlier un-limited runs (lc_time 1.0) are kept "
      "as `*_nolimit_superseded.jsonl` and are not used.\n")
    w("**Server sessions.** RC12 and RC19 records were produced in the earlier server session (before the restart); every limited RC03/RC07/RC20 "
      "record and the a_lat check were produced in the restarted session.  Selftest values shifted slightly after the restart (keep-test v0 11.97 -> 11.87 m/s); "
      "RC12 and RC19 were not re-run and are mixed in by key (same physics_profile_id).\n")
    w(hl_rerun_note())
    w("## RC03: braking fails, bypass passes (brief speeds 8, 11, 14, plus 18 and 22 added after no window was found at 8-14)\n")
    w("Search: for each speed, gap scan for lc_time in " + ", ".join(f"{x:g}" for x in LC_CANDIDATES) +
      ".  Rule: use the largest lc_time with a window (brake_hold < eps and some bypass >= eps under the limit); chosen gap = median of that window.  "
      "Library per run: brake_hold, lane_follow, bypass_left(lc), brake_steer_left(lc, 0.4 s brake).\n")
    w("Windows found per lc_time:\n\n| v | " + " | ".join(f"lc {x:g}" for x in sorted(LC_CANDIDATES, reverse=True)) + " |\n|---|" + "---|" * len(LC_CANDIDATES))
    for v, d in r03.items():
        w(f"| {v:g} | " + " | ".join(str(d["per_lc"][lc]["window"] or "none") for lc in sorted(LC_CANDIDATES, reverse=True)) + " |")
    w("\nChosen witnesses:\n\n| v | lc_time | gap | brake_hold G | best bypass (policy) G | peak abs a_lat of witness (m/s^2) | evidence | closed-lane evidence | closed margins |")
    w("|---|---|---|---|---|---|---|---|---|")
    for v, d in r03.items():
        if d["gap"] is None:
            w(f"| {v:g} | - | NONE | - | - | - | - | - | - |")
            continue
        r = d["scan"][d["gap"]]
        m = margins(r)
        bk, bg = best_bypass(r)
        c = d["closed"]
        w(f"| {v:g} | {d['lc']:g} | {d['gap']:g} | {fm(m['brake_hold'])} | {bk} {fm(bg)} | {fp(peak_alat(r, bk))} | {r['evidence_class']} | "
          f"{c['evidence_class']} | {mrow(c)} |")
    nowin = [v for v, d in r03.items() if d["gap"] is None]
    if nowin:
        w(f"\nSpeeds with NO window under the limit (mechanism not shown on CARLA, template excluded for these, nothing relabelled): {nowin}.  "
          "Under the a_lat limit every admissible lane change (lc_time 1.0-1.6 all exceed the limit or fail clearance in the scans; only 2.0 is admissible) needs more gap than braking, so the bypass becomes a witness only where brake_hold already passes; a window appears only at higher speeds (18 and 22 m/s, outside the brief's set).  The v = 18 window is a single gap (30 m, bypass margin 0.008), so it is fragile; v = 22 has 3 gaps.")
    w("\nClosing the escape lane (blocker in the left lane at the same x) gives a finite-library negative: labelled by evidence only (U_LIBRARY expected, never U_MODEL, never pre-labelled).\n")
    for v, d in r03.items():
        for lc in sorted(LC_CANDIDATES, reverse=True):
            sc = d["per_lc"][lc]["scan"]
            w(f"\nScan v = {v:g}, lc_time = {lc:g}:\n\n| gap | evidence | margins |\n|---|---|---|")
            for g in sorted(sc):
                w(f"| {g:g} | {sc[g]['evidence_class']} | {mrow(sc[g])} |")
    w("\n## RC07: premature hand-back vs continuation\n")
    w("*RC07 was re-run on the fixed plant (moving actors pinned to their declared speed during warm-up; Task 6 fix round 1). The first run, whose rear actor started at 15.90 m/s for a nominal 18, is superseded (sweeps `rc07_scan_al8` / `rc07_sel_al8`, kept in the jsonl directory; previous report text in git history at commit 21719eb).*\n")
    w(f"Library `{RC07_LIBRARY_ID}` = bypass_then_stop, bypass_then_return, brake_hold, bypass_left (primary-only avoidance, no hand-back), lc_time 2.0 (peak a_lat ~5), "
      f"a_lat_max {A_LAT_MAX:g}.  v = 12, rear actor in the left lane.  Full contrast in one scene = bypass_left >= eps AND bypass_then_stop < eps AND bypass_then_return >= eps (`rc07_contrast`).\n")
    w("| gap | rear_gap | rear_v | evidence | full contrast | margins |\n|---|---|---|---|---|---|")
    for k, r in r07["scan"].items():
        w(f"| {k[0]} | {k[1]} | {k[2]} | {r['evidence_class']} | {rc07_contrast(r)} | {mrow(r)} |")
    fr = r07["fast_rear_full"]
    w(f"\nScenes with the full contrast: {r07['full']}.  Fast-rear (rear_v = 24) scenes with the full contrast: {fr if fr else 'NONE'}"
      + ("" if fr else " (at rear_v 24 the primary-only bypass_left keeps driving in the left lane and is always hit by the rear actor, so all three never hold together there)") + ".\n")
    if r07["sel"] is None:
        w("No scene shows the full contrast: RC07 template excluded on CARLA.")
    else:
        t, r, rv = r07["sel_theta"], r07["sel"], r07["rear_v"]
        w(f"Selection rule: among full-contrast scenes take the highest rear_v, then the median of those ordered by (gap, rear_gap).  "
          f"Selected: gap {t['gap_m']:g}, rear_gap {t['rear_gap_m']:g}, rear_v {t['rear_v_mps']:g}.  Evidence **{r['evidence_class']}**; margins: {mrow(r)}.\n")
        for pol in ("bypass_left", "bypass_then_stop", "bypass_then_return", "brake_hold"):
            tm, oth = term(r, pol)
            w(f"- {pol}: termination {tm}, contacted {oth or 'none'}, peak abs a_lat {fp(peak_alat(r, pol))} m/s^2")
        w(f"\nMeasured rear speed (per-tick spy over the policy episodes, {rv['n']} ticks): nominal {rv['nominal']:g}, at start {rv['start']:.2f}, max {rv['max']:.2f}, mean {rv['mean']:.2f} m/s.  "
          f"brake_hold also passes in place at gap {t['gap_m']:g}, so the scene stays {r['evidence_class']} (one refuted policy never makes a scene unrecoverable).\n")
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
    w(f"Tolerance pre-declared before any run: |dV| <= {RC19_TOL} (normalised margins, unit-free).\n")
    w("| v | anchor G | nuisance G | |dV| (max abs profile diff) | within tolerance | evidence anchor / twin |\n|---|---|---|---|---|---|")
    for v, d in r19.items():
        w(f"| {v:g} | {[fm(x) for x in d['ga']]} | {[fm(x) for x in d['gb']]} | {d['dV']:.4f} | {d['dV'] <= RC19_TOL} | {d['anchor']['evidence_class']} / {d['nuisance']['evidence_class']} |")
    w("\n## RC20: library completeness (same RC03 scene, physics unchanged)\n")
    w(f"Same scene as the RC03 witness at each speed (lc_time as chosen above, a_lat_max {A_LAT_MAX:g}).  Restricted = brake_hold, lane_follow; budget-limited = restricted with run_mask {{(0,0)}} (library id +budget1); expanded = full RC03 library.  Library versions: ids below, `sim.policies.VERSION` in each descriptor.\n")
    w("| v | lc | gap | restricted | budget-limited (R_raw) | expanded | identical physical spec |\n|---|---|---|---|---|---|---|")
    for v, d in r20.items():
        w(f"| {v:g} | {d['lc']:g} | {d['theta']['gap_m']:g} | {d['restricted']['evidence_class']} ({d['restricted']['library_id']}) | {d['budget']['evidence_class']}, {d['budget']['R_raw']} | "
          f"{d['expanded']['evidence_class']} ({d['expanded']['library_id']}) | {d['same_spec']} |")
    n, bad = validate_all()
    w(f"\n## Record validation\n\n{n} records in `results/carla/core/` (excluding superseded/probe files) checked with validate_records.check_record: {len(bad)} failures.")
    for b in bad[:10]:
        w(f"- {b}")
    return "\n".join(L) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    ctx = Ctx(a.host, a.port)
    r03 = run_rc03(ctx)
    r07, r12, r19 = run_rc07(ctx), run_rc12(ctx), run_rc19(ctx)
    r20 = run_rc20(ctx, r03)
    text = report(ctx, r03, r07, r12, r19, r20)
    open("results/carla/core.md", "w", encoding="utf-8", newline="\n").write(text)
    print(text)


if __name__ == "__main__":
    main()
