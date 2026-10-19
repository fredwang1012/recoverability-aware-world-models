"""
Task 6: compositional cases on CARLA (RC04, RC05, RC06, RC08, RC15, RC18).

    cd mining && PYTHONNOUSERSITE=1 python ../scripts/carla_compositional.py [--host localhost --port 2000]

Needs a live, bring-up-passed server (never started here).  Every point is a full ExecutionRecord checkpointed to
mining/results/carla/compositional/<sweep>.jsonl (a rerun skips finished points); summary mining/results/carla/compositional.md.
Labels come from evidence only (sim.harness.evaluate -> contract.classify); eps/reserve are margin.DEFAULT_V2, untouched.

Selection rules (declared before the run, applied mechanically to the scans; the scans themselves are all reported):
    RC04  staged_brake_* >= eps in every world, brake_hold < eps, immediate_lane_change < eps          -> median gap
    RC05  commit < eps in some world, abort_and_stabilize >= eps in all worlds                        -> median over scenes
    RC06  immediate_entry < eps, wait_observe_enter >= eps                                            -> median over scenes
    RC15  strong: always_merge >= eps in adj_yields and < eps in adj_nonyield, observe_yield_else_fallback >= eps in both
          weak  : always_merge < eps in some world, observe_yield_else_fallback >= eps in both
    RC18  a requested brake+steer command passes the a_lat-only spec (G >= eps) but is refuted by a_comb_max (G < eps)
    RC08  fixed scene (no search): open lane and closed lane
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "mining"))
sys.path.insert(0, HERE)

import margin as M
from cases import compositional as CP
import carla_core as CC

OUT = "results/carla/compositional"
CFG = M.DEFAULT_V2
TAU = 0.25
DECEL_TARGET = CC.DECEL_TARGET
TAG = f"al{CP.A_LAT_MAX:g}c{CP.A_COMB_MAX:g}"
SCOPE = "carla_finite_replay"
gval = CC.gval


# ----------------------------------------------------------------------------- pure helpers (mock-tested)
def mw(r):
    """{policy: [G per world]} (None = unrun, -inf = hard failure)."""
    return {p: [gval(x) for x in row] for p, row in zip(r["policies"], r["R_raw"])}


def ok(g, eps=CFG.eps):
    return g is not None and g >= eps


def bad(g, eps=CFG.eps):
    return g is not None and g < eps


def all_ok(gs, eps=CFG.eps):
    return all(ok(g, eps) for g in gs)


def any_bad(gs, eps=CFG.eps):
    return any(bad(g, eps) for g in gs)


def rc04_contrast(r, eps=CFG.eps):
    m = mw(r)
    staged = [v for k, v in m.items() if k.startswith("staged_brake")]
    return (any(all_ok(v, eps) for v in staged) and any_bad(m.get("brake_hold", [None]), eps)
            and any_bad(m.get("immediate_lane_change", [None]), eps))


def rc05_contrast(r, eps=CFG.eps):
    m = mw(r)
    return any_bad(m.get("commit_lane_change", [None]), eps) and all_ok(m.get("abort_and_stabilize", [None]), eps)


def rc06_contrast(r, eps=CFG.eps):
    m = mw(r)
    return any_bad(m.get("immediate_entry", [None]), eps) and all_ok(m.get("wait_observe_enter", [None]), eps)


def rc15_contrast(r, eps=CFG.eps):
    """'strong' / 'weak' / None (see module docstring)."""
    m = mw(r)
    am, oy = m.get("always_merge"), m.get("observe_yield_else_fallback")
    if am is None or oy is None or not all_ok(oy, eps):
        return None
    if ok(am[0], eps) and bad(am[1], eps):
        return "strong"
    return "weak" if any_bad(am, eps) else None


def rc18_refuted_by_comb(r_nocomb, r_comb, eps=CFG.eps):
    """Commands (brake_steer_cmd_*) with G >= eps under the a_lat-only spec and G < eps under a_lat + a_comb."""
    a, b = mw(r_nocomb), mw(r_comb)
    return sorted(k for k in a if k.startswith("brake_steer_cmd") and ok(a[k][0], eps) and bad(b.get(k, [None])[0], eps))


def median_key(keys):
    keys = sorted(keys)
    return keys[len(keys) // 2] if keys else None


def gpu_wait(limit=85, resume=78):
    try:
        while True:
            t = int(subprocess.check_output(["nvidia-smi", "--query-gpu=temperature.gpu", "--format=csv,noheader"], text=True).strip())
            if t < limit:
                return t
            print(f"GPU {t} C >= {limit}; waiting for < {resume}", flush=True)
            while t >= resume:
                time.sleep(60)
                t = int(subprocess.check_output(["nvidia-smi", "--query-gpu=temperature.gpu", "--format=csv,noheader"], text=True).strip())
    except Exception:
        return None


def pk(r, policy, world, key):
    """Recorded dyn peak (a_lat / a_long_smoothed / a_comb) of one episode from traces[...]['margin']['dyn_peaks']."""
    for k, tr in r["traces"].items():
        if k == f"{policy}/{world}":
            return tr["margin"].get("dyn_peaks", {}).get(key)
    return None


def fm(x):
    return "None" if x is None else ("-inf" if x == -math.inf else f"{x:.3f}")


def mrows(r):
    return "; ".join(f"{p}=" + "/".join(fm(g) for g in gs) for p, gs in mw(r).items())


# ----------------------------------------------------------------------------- context
class Ctx(CC.Ctx):
    def load(self, sweep):
        path = f"{OUT}/{sweep}.jsonl"
        d = {}
        if os.path.exists(path):
            for line in open(path):
                r = json.loads(line)
                if r.get("physics_profile_id") != self.pid:
                    raise RuntimeError(f"{path}: cached physics_profile_id {r.get('physics_profile_id')} != {self.pid}")
                d[r["_key"]] = r
        self.cache[sweep] = d
        return d

    def point(self, sweep, key, fn, **extra):
        d = self.cache.get(sweep) or self.load(sweep)
        if key in d:
            return d[key]
        gpu_wait()
        r = fn().to_json()
        r["_key"] = key
        r.update(extra)
        with open(f"{OUT}/{sweep}.jsonl", "a") as f:
            f.write(json.dumps(r) + "\n")
        d[key] = r
        return r

    def run(self, case, theta, **kw):
        return CP.run(case, self.factory, theta, CFG, "carla", SCOPE, physics_profile_id=self.pid, runtime=self.runtime, **kw)


def audit(ctx, case, theta, policy_factory, world_name, **kw):
    """Re-run ONE witness policy in ONE world with a per-tick spy (ego + other actors) and summarise it."""
    log = []
    fac = lambda: CC.SpyPlant(ctx.factory(), log)
    worlds = kw.pop("worlds", None)
    if worlds is not None:
        worlds = [w for w in worlds if w["name"] == world_name]
    rec = CP.run(case, fac, theta, CFG, "carla", SCOPE, library=[policy_factory], worlds=worlds, physics_profile_id=ctx.pid,
                 runtime=ctx.runtime, **kw)
    return summarize_trace(log, rec, 0.05)


def summarize_trace(log, rec, dt):
    ego = [s["ego"] for s in log]
    lat, v = [e[1] for e in ego], [e[2] for e in ego]
    step = max((abs(lat[i + 1] - lat[i]) for i in range(len(lat) - 1)), default=0.0)
    step_ratio = max((abs(lat[i + 1] - lat[i]) / max(v[i] * dt, 1e-6) for i in range(len(lat) - 1) if v[i] > 2.0), default=0.0)
    out = {"n_ticks": len(log), "ego_v_start": v[0], "ego_v_end": v[-1], "ego_lat_min": min(lat), "ego_lat_max": max(lat), "ego_lat_end": lat[-1],
           "max_lat_step_m": step, "max_lat_step_over_v_dt": step_ratio, "x_end": ego[-1][0]}
    names = [n for n in log[0] if n != "ego"]
    out["others"] = {}
    for n in names:
        o = [s[n] for s in log]
        out["others"][n] = {"v_start": o[0][2], "v_end": o[-1][2], "lat_end": o[-1][1], "x_end": o[-1][0],
                            "min_abs_dx": min(abs(a[0] - b[0]) for a, b in zip(o, ego))}
    key = next(iter(rec.traces))
    tr = rec.traces[key]
    out.update({"run": key, "termination": rec.events[key]["termination"], "contact_events": rec.events[key]["events"],
                "G": rec.R_raw[0][0], "min_clearance_m": {k: (None if (v2 is None or v2 == math.inf) else v2)
                                                           for k, v2 in tr["margin"]["min_clearance_m"].items()},
                "dyn_peaks": tr["margin"].get("dyn_peaks"), "route_goal_completed": tr.get("route_goal_completed"),
                "commands": rec.timestamps[key]["commands"], "evidence_class_single_policy": rec.evidence_class})
    return out


def at_time(log, t, dt=0.05):
    k = min(int(round(t / dt)), len(log) - 1)
    return log[k]


# ----------------------------------------------------------------------------- sweeps
def run_rc04(ctx):
    res = {}
    for lc in (1.6, 2.0):
        for v in (14.0, 18.0):
            name = f"rc04_v{v:g}_lc{lc:g}_{TAG}"
            for g in range(18, 40, 2):
                th = {"gap_m": float(g), "v_mps": v, "tau_s": TAU}
                res[(lc, v, g)] = ctx.point(name, f"{g}", lambda th=th, lc=lc: ctx.run("RC04", th, library=CP.rc04_library(lc)),
                                            library_id=f"rc04_lc{lc:g}_ps0.4-0.8_{TAG}")
            print("RC04", lc, v, [g for (l, vv, g), r in res.items() if l == lc and vv == v and rc04_contrast(r)], flush=True)
    return res


def run_rc05(ctx):
    res = {}
    for g in (60.0, 80.0):
        for rg in (20.0, 35.0, 50.0):
            for rv in (20.0, 24.0):
                th = {"gap_m": g, "v_mps": 14.0, "tau_s": TAU, "rear_gap_m": rg, "rear_v_mps": rv, "t_reveal_s": 1.25}
                res[(g, rg, rv)] = ctx.point(f"rc05_{TAG}", f"{g:g}_{rg:g}_{rv:g}", lambda th=th: ctx.run("RC05", th),
                                             library_id=f"rc05_prefix1.0_lc3.0_abort1.5_ttc4_{TAG}")
    print("RC05", [k for k, r in res.items() if rc05_contrast(r)], flush=True)
    return res


def run_rc06(ctx):
    res = {}
    for g in (45.0, 60.0, 75.0):
        for pg in (10.0, 20.0):
            for pv in (16.0, 20.0):
                th = {"gap_m": g, "v_mps": 10.0, "tau_s": TAU, "pass_gap_m": pg, "pass_v_mps": pv}
                res[(g, pg, pv)] = ctx.point(f"rc06_{TAG}", f"{g:g}_{pg:g}_{pv:g}", lambda th=th: ctx.run("RC06", th),
                                             library_id=f"rc06_lc2.0_{TAG}")
    print("RC06", [k for k, r in res.items() if rc06_contrast(r)], flush=True)
    return res


def run_rc15(ctx):
    res = {}
    for g in (45.0, 60.0, 75.0):
        for ag in (25.0, 35.0):
            for av in (16.0, 20.0):
                th = {"gap_m": g, "v_mps": 10.0, "tau_s": TAU, "adj_gap_m": ag, "adj_v_mps": av}
                res[(g, ag, av)] = ctx.point(f"rc15_{TAG}", f"{g:g}_{ag:g}_{av:g}", lambda th=th: ctx.run("RC15", th),
                                             library_id=f"rc15_lc2.0_wait2.0_drop1.5_{TAG}")
    print("RC15", {k: rc15_contrast(r) for k, r in res.items() if rc15_contrast(r)}, flush=True)
    return res


def run_rc08(ctx):
    res = {}
    for closed in (False, True):
        th = {"gap_m": 45.0, "v_mps": 14.0, "tau_s": TAU, "closed": closed}
        res[closed] = ctx.point(f"rc08_{TAG}", f"closed{int(closed)}", lambda th=th: ctx.run("RC08", th), library_id=f"rc08_lc2.0_{TAG}")
    return res


RC18_LCS = (1.6, 2.0, 2.5)


def run_rc18(ctx):
    res = {}
    for v, gaps in ((18.0, range(26, 48, 4)), (22.0, range(30, 52, 4))):
        for g in gaps:
            th = {"gap_m": float(g), "v_mps": v, "tau_s": TAU}
            for comb in (False, True):
                res[(v, g, comb)] = ctx.point(f"rc18_{TAG}", f"{v:g}_{g}_{int(comb)}",
                                              lambda th=th, comb=comb: ctx.run("RC18", th, library=CP.rc18_library(RC18_LCS, 2.0),
                                                                              spec=CP.rc18_spec(th, CFG, comb=comb)),
                                              library_id=f"rc18_lcs{'-'.join(map(str, RC18_LCS))}_{TAG}", comb=comb)
            print("RC18", v, g, rc18_refuted_by_comb(res[(v, g, False)], res[(v, g, True)]), flush=True)
    return res


# ----------------------------------------------------------------------------- selection + audit
def pick(scan, pred):
    keys = [k for k, r in scan.items() if pred(r)]
    return median_key(keys), keys


def select_and_audit(ctx, r04, r05, r06, r15, r18):
    out = {}
    # RC04: prefer lc 1.6, then 2.0; within the library pick the highest speed with a window, median gap
    sel = None
    for lc in (1.6, 2.0):
        for v in (18.0, 14.0):
            keys = [g for (l, vv, g), r in r04.items() if l == lc and vv == v and rc04_contrast(r)]
            if keys:
                sel = (lc, v, median_key(keys), keys)
                break
        if sel:
            break
    if sel:
        lc, v, g, keys = sel
        th = {"gap_m": float(g), "v_mps": v, "tau_s": TAU}
        r = r04[(lc, v, g)]
        stg = [k for k, gs in mw(r).items() if k.startswith("staged_brake") and all_ok(gs)][0]
        fac = [f for f in CP.rc04_library(lc) if f().name == stg][0]
        out["RC04"] = {"sel": (lc, v, g), "window": keys, "witness": stg, "record": r, "audit": audit(ctx, "RC04", th, fac, "static_blocker")}
    else:
        out["RC04"] = None
    k5, keys5 = pick(r05, rc05_contrast)
    if k5:
        th = {"gap_m": k5[0], "v_mps": 14.0, "tau_s": TAU, "rear_gap_m": k5[1], "rear_v_mps": k5[2], "t_reveal_s": 1.25}
        out["RC05"] = {"sel": k5, "window": keys5, "witness": "abort_and_stabilize", "record": r05[k5],
                       "audit": {w["name"]: audit(ctx, "RC05", th, CP.rc05_library()[1], w["name"], worlds=CP.rc05_worlds(1.25)) for w in CP.rc05_worlds(1.25)},
                       "audit_commit": audit(ctx, "RC05", th, CP.rc05_library()[0], "gap_closes", worlds=CP.rc05_worlds(1.25))}
    else:
        out["RC05"] = None
    k6, keys6 = pick(r06, rc06_contrast)
    if k6:
        th = {"gap_m": k6[0], "v_mps": 10.0, "tau_s": TAU, "pass_gap_m": k6[1], "pass_v_mps": k6[2]}
        out["RC06"] = {"sel": k6, "window": keys6, "witness": "wait_observe_enter", "record": r06[k6],
                       "audit": audit(ctx, "RC06", th, CP.rc06_library()[1], "passer_cruise"),
                       "audit_immediate": audit(ctx, "RC06", th, CP.rc06_library()[0], "passer_cruise")}
    else:
        out["RC06"] = None
    strong = [k for k, r in r15.items() if rc15_contrast(r) == "strong"]
    weak = [k for k, r in r15.items() if rc15_contrast(r) in ("strong", "weak")]
    k15 = median_key(strong) or median_key(weak)
    if k15:
        th = {"gap_m": k15[0], "v_mps": 10.0, "tau_s": TAU, "adj_gap_m": k15[1], "adj_v_mps": k15[2]}
        lib = CP.rc15_library()
        out["RC15"] = {"sel": k15, "kind": rc15_contrast(r15[k15]), "strong": strong, "weak": weak, "witness": "observe_yield_else_fallback",
                       "record": r15[k15],
                       "audit": {w["name"]: audit(ctx, "RC15", th, lib[1], w["name"], worlds=CP.rc15_worlds()) for w in CP.rc15_worlds()},
                       "audit_always": {w["name"]: audit(ctx, "RC15", th, lib[0], w["name"], worlds=CP.rc15_worlds()) for w in CP.rc15_worlds()}}
    else:
        out["RC15"] = None
    # RC18: scenes where >= 1 command is refuted only by the combined limit
    cand = [(v, g) for (v, g, c) in r18 if c and rc18_refuted_by_comb(r18[(v, g, False)], r18[(v, g, True)])]
    kc = median_key(cand)
    if kc:
        v, g = kc
        cmds = rc18_refuted_by_comb(r18[(v, g, False)], r18[(v, g, True)])
        out["RC18"] = {"sel": kc, "window": cand, "cmds": cmds, "rec_nocomb": r18[(v, g, False)], "rec_comb": r18[(v, g, True)]}
        feas = [p for p, gs in mw(r18[(v, g, True)]).items() if all_ok(gs)]
        out["RC18"]["feasible_alternatives"] = feas
        if feas:
            th = {"gap_m": float(g), "v_mps": v, "tau_s": TAU}
            f = [x for x in CP.rc18_library(RC18_LCS, 2.0) if x().name == feas[0]][0]
            out["RC18"]["audit"] = audit(ctx, "RC18", th, f, "static_blocker", spec=CP.rc18_spec(th, CFG, comb=True))
            out["RC18"]["witness"] = feas[0]
    else:
        out["RC18"] = None
    return out


# ----------------------------------------------------------------------------- report
def ev_row(r):
    return r["evidence_class"]


def trace_lines(a, indent="  "):
    L = [f"{indent}- run `{a['run']}`: termination {a['termination']}, contact events {a['contact_events'] or 'none'}, G {fm(gval(a['G']))}, "
         f"evidence (single-policy re-run) {a['evidence_class_single_policy']}",
         f"{indent}- ego v {a['ego_v_start']:.2f} -> {a['ego_v_end']:.2f} m/s; lateral range [{a['ego_lat_min']:.2f}, {a['ego_lat_max']:.2f}] m, end {a['ego_lat_end']:.2f}; "
         f"x_end {a['x_end']:.1f}; max per-tick lateral step {a['max_lat_step_m']:.3f} m = {a['max_lat_step_over_v_dt']:.3f} x (v*dt) (continuous, no teleport)",
         f"{indent}- min clearance to each actor (m): {({k: (None if v is None else round(v, 2)) for k, v in a['min_clearance_m'].items()})}",
         f"{indent}- dynamic peaks: {({k: (round(v, 2) if not isinstance(v, dict) else {n: round(x, 3) for n, x in v.items()}) for k, v in (a['dyn_peaks'] or {}).items()})}",
         f"{indent}- commands (decision_t -> effective_t): " + ", ".join(f"{c['long']}/{c['lat_target']} {c['decision_t']:.2f}->{c['effective_t']:.2f}" for c in a["commands"])]
    for n, o in a["others"].items():
        L.append(f"{indent}- actor `{n}`: v {o['v_start']:.2f} -> {o['v_end']:.2f}, lat_end {o['lat_end']:.2f}, closest |dx| to ego {o['min_abs_dx']:.1f} m")
    if a["route_goal_completed"] is not None:
        L.append(f"{indent}- route_goal_completed: {a['route_goal_completed']}")
    return L


def validate_all():
    import validate_records as VR
    bad_, n = [], 0
    for fn in sorted(os.listdir(OUT)):
        if fn.endswith(".jsonl"):
            for line in open(f"{OUT}/{fn}"):
                r = json.loads(line)
                n += 1
                e = VR.check_record(r)
                if e:
                    bad_.append((fn, r.get("_key"), e))
    return n, bad_


def report(ctx, r04, r05, r06, r08, r15, r18, sel):
    L, w = [], None
    w = L.append
    w("# CARLA compositional cases (Task 6: RC04, RC05, RC06, RC08, RC15, RC18)\n")
    w(f"Physics profile `{ctx.pid}`, decel target {DECEL_TARGET}, eps = {CFG.eps}, reserve c = {CFG.reserve_m} m (margin.DEFAULT_V2, unchanged), act delay {TAU} s. "
      "Records: `results/carla/compositional/*.jsonl`. Labels are `contract.classify` output only. G cells below read `world1/world2` (-inf = hard failure, None = unrun).\n")
    w(f"**Authority.** Every spec declares `a_lat_max = {CP.A_LAT_MAX:g}` m/s^2 (dynamic-limit margin from |v * yaw_rate|, as in the core cases); RC18 additionally declares "
      f"`a_comb_max = {CP.A_COMB_MAX:g}` m/s^2, a friction-circle margin sqrt(a_long^2 + a_lat^2) <= limit from MEASURED accelerations "
      f"(a_long = tick difference of ego speed, centred {__import__('sim.harness', fromlist=['x']).A_LONG_SMOOTH_TICKS}-tick moving average = 0.25 s, declared before any run; a_lat = |v * yaw_rate| per tick, unsmoothed). "
      "**Disclosure:** CARLA's tyre friction in this profile is non-physical (`friction_fixed` 5.0, braking set through brake torque), so CARLA's own tyre limit is not the authority; "
      "the declared 8.0 m/s^2 limits are the physical-plausibility authority and enter only as margins (they make a manoeuvre a non-witness, never a hand-written label).\n")
    w("Selection rules were declared in the script header before the run; every scan row is reported below.\n")
    # ---- per-case table
    w("## Per-case table\n")
    w("| case | scanned scenes | scenes with the declared contrast | selected scene | label (evidence class) | witness policy | refuted policies in selected scene |")
    w("|---|---|---|---|---|---|---|")

    def refuted(r):
        return ", ".join(f"{p} ({'/'.join(fm(g) for g in gs)})" for p, gs in mw(r).items() if any_bad(gs)) or "none"
    for case, scan, key in (("RC04", r04, "RC04"), ("RC05", r05, "RC05"), ("RC06", r06, "RC06"), ("RC15", r15, "RC15")):
        s = sel[key]
        n_con = {"RC04": sum(rc04_contrast(r) for r in scan.values()), "RC05": sum(rc05_contrast(r) for r in scan.values()),
                 "RC06": sum(rc06_contrast(r) for r in scan.values()), "RC15": sum(1 for r in scan.values() if rc15_contrast(r))}[key]
        if s is None:
            w(f"| {case} | {len(scan)} | {n_con} | NONE | - | no positive witness with the contrast | - |")
        else:
            w(f"| {case} | {len(scan)} | {n_con} | {s['sel']} | {ev_row(s['record'])} | {s['witness']} | {refuted(s['record'])} |")
    for closed, r in r08.items():
        w(f"| RC08 ({'closed lane' if closed else 'open lane'}) | 1 | - | gap 45, v 14 | {ev_row(r)} | " +
          ", ".join(p for p, gs in mw(r).items() if all_ok(gs)) + f" | {refuted(r)} |")
    s = sel["RC18"]
    ncomb = sum(1 for (v, g, c) in r18 if c and rc18_refuted_by_comb(r18[(v, g, False)], r18[(v, g, True)]))
    if s is None:
        w(f"| RC18 | {len(r18) // 2} | {ncomb} | NONE | - | no command passes a_lat-only and is refuted by a_comb | - |")
    else:
        w(f"| RC18 | {len(r18) // 2} | {ncomb} | {s['sel']} | {ev_row(s['rec_comb'])} | {s.get('witness', 'none (no feasible alternative)')} | {', '.join(s['cmds'])} refuted by a_comb |")
    # ---- RC04
    w("\n## RC04: brake then change lanes\n")
    w("Library (per lc_time L, L in 1.6 and 2.0): brake_hold, lane_follow, immediate_lane_change (Bypass, L), staged_brake_0.4_change and staged_brake_0.8_change (StagedBrakeChange, L). "
      "Immediate lane change is a different candidate at each L; scenes where the immediate change at lc 2.0 already passes are therefore not witnesses of the staged policy in the lc-2.0 library.\n")
    w("| lc | v | gap | evidence | contrast | margins |\n|---|---|---|---|---|---|")
    for (lc, v, g), r in sorted(r04.items()):
        w(f"| {lc:g} | {v:g} | {g} | {r['evidence_class']} | {rc04_contrast(r)} | {mrows(r)} |")
    s = sel["RC04"]
    if s:
        w(f"\nSelected: lc {s['sel'][0]:g}, v {s['sel'][1]:g}, gap {s['sel'][2]} (window {s['window']}); witness `{s['witness']}`.\n\nAudited witness trace summary:")
        L.extend(trace_lines(s["audit"]))
    else:
        w("\nNo scene shows the contrast (staged passes while brake_hold and the immediate change fail); no positive RC04 witness on CARLA under the 8.0 m/s^2 limit in the searched grid "
          "(lc 1.6 and 2.0, v 14 and 18, gap 18-38 step 2).")
    # ---- RC05
    w("\n## RC05: abort an unsafe lane change\n")
    w("Every policy first executes a physical left lane change (prefix 1.0 s, lc_time 3.0) inside the episode; the rear vehicle in the target lane is hidden until t = 1.25 s. "
      "Worlds (mode fixed per world): `gap_closes` (rear cruises) and `gap_open` (rear brakes at 5 m/s^2 from t = 0). Library: commit_lane_change, abort_and_stabilize (steer back, settle 1.8 s, brake), "
      "abort_if_closing (observation-conditioned, ttc 4 s). v = 14, tau 0.25.\n")
    w("| blocker gap | rear_gap | rear_v | evidence | contrast | margins (gap_closes/gap_open) |\n|---|---|---|---|---|---|")
    for k, r in sorted(r05.items()):
        w(f"| {k[0]:g} | {k[1]:g} | {k[2]:g} | {r['evidence_class']} | {rc05_contrast(r)} | {mrows(r)} |")
    s = sel["RC05"]
    if s:
        w(f"\nSelected: {s['sel']} (window size {len(s['window'])}); witness `abort_and_stabilize`.\n")
        for wn, a in s["audit"].items():
            w(f"Audited witness trace, world `{wn}`:")
            L.extend(trace_lines(a))
        w("\nRefuted commit trace (gap_closes):")
        L.extend(trace_lines(s["audit_commit"]))
    else:
        w("\nNo scene with commit refuted and abort_and_stabilize passing in both worlds.")
    # ---- RC06
    w("\n## RC06: yield, then use the time-varying gap\n")
    w("v = 10, static blocker in the ego lane, a faster vehicle passes in the target lane (cruise). Library: immediate_entry, wait_observe_enter (keep speed while the observed gap is unusable), "
      "wait_brake_observe_enter (brake 0.3 while waiting), brake_hold, lane_follow. With a static blocker, braking in place is also a witness whenever the gap is long enough, so the scene is R_REF either way; "
      "the case contrast is wait vs immediate.\n")
    w("| blocker gap | pass_gap | pass_v | evidence | contrast | margins |\n|---|---|---|---|---|---|")
    for k, r in sorted(r06.items()):
        w(f"| {k[0]:g} | {k[1]:g} | {k[2]:g} | {r['evidence_class']} | {rc06_contrast(r)} | {mrows(r)} |")
    s = sel["RC06"]
    if s:
        w(f"\nSelected: {s['sel']}; witness `wait_observe_enter`.\n\nAudited witness trace summary:")
        L.extend(trace_lines(s["audit"]))
        w("\nRefuted immediate_entry trace:")
        L.extend(trace_lines(s["audit_immediate"]))
    else:
        w("\nNo scene with immediate_entry refuted and wait_observe_enter passing.")
    # ---- RC08
    w("\n## RC08: protected waiting, route non-completion\n")
    w("Static blocker at gap 45, v 14, goal x = gap + 4.6 + 15 m. `route_goal_completed` is computed separately by the harness (front bumper reaches the goal, no contact) and stored in `traces[...]`; "
      "it is never part of the margin, the terminal test or the label.\n")
    w("| scene | evidence | margins | route_goal_completed per policy |\n|---|---|---|---|")
    for closed, r in r08.items():
        rg = {k.split("/")[0]: v.get("route_goal_completed") for k, v in r["traces"].items()}
        w(f"| {'closed lane' if closed else 'open lane'} | {r['evidence_class']} | {mrows(r)} | {rg} |")
    # ---- RC15
    w("\n## RC15: yielding must be observed\n")
    w("v = 10; adjacent vehicle in the left lane behind the ego. Worlds (mode fixed per world): `adj_yields` (starts reacting at t = 0.5 s, +0.5 s reaction, brakes 3 m/s^2 to 5 m/s) and `adj_nonyield`. "
      "Library: always_merge, observe_yield_else_fallback (merge once the neighbour's closing speed has dropped by 1.5 m/s, else brake after 2.0 s), brake_hold, lane_follow.\n")
    w("| blocker gap | adj_gap | adj_v | evidence | contrast | margins (adj_yields/adj_nonyield) |\n|---|---|---|---|---|---|")
    for k, r in sorted(r15.items()):
        w(f"| {k[0]:g} | {k[1]:g} | {k[2]:g} | {r['evidence_class']} | {rc15_contrast(r)} | {mrows(r)} |")
    s = sel["RC15"]
    if s:
        w(f"\nSelected ({s['kind']} contrast; strong scenes {s['strong']}, weak-or-strong scenes {len(s['weak'])}): {s['sel']}; witness `observe_yield_else_fallback`.\n")
        for wn, a in s["audit"].items():
            w(f"Audited witness trace, world `{wn}`:")
            L.extend(trace_lines(a))
        for wn, a in s["audit_always"].items():
            w(f"\nalways_merge trace, world `{wn}`:")
            L.extend(trace_lines(a))
    else:
        w("\nNo scene where observe_yield_else_fallback passes both worlds while always_merge fails.")
    # ---- RC18
    w("\n## RC18: requested control exceeds authority\n")
    w(f"Scene: static blocker, v 18 or 22. Library: brake_hold, lane_follow, bypass_left (keep speed, lc 2.0), brake_steer_cmd_lc{{L}}_b1 for L in {RC18_LCS} (full brake AND steer to the left lane in L seconds). "
      "Each scene is run twice: spec with a_lat_max only, and spec with a_lat_max + a_comb_max. A command is *refuted by the combined limit* when it passes under the first and fails under the second (same plant, same command). "
      "'Specific command refuted' means exactly this: the request violates the limit margin, so it is outside the admissible set; it does NOT claim the scene is unrecoverable.\n")
    w("| v | gap | a_lat-only: evidence / margins | a_lat + a_comb: evidence / margins | commands refuted only by a_comb |\n|---|---|---|---|---|")
    for (v, g, c), r in sorted(r18.items()):
        if c:
            a = r18[(v, g, False)]
            w(f"| {v:g} | {g} | {a['evidence_class']} / {mrows(a)} | {r['evidence_class']} / {mrows(r)} | {rc18_refuted_by_comb(a, r) or '-'} |")
    s = sel["RC18"]
    if s:
        v, g = s["sel"]
        w(f"\nSelected scene v {v:g}, gap {g}: commands refuted only by the combined limit: {s['cmds']}; feasible alternatives under both limits: {s['feasible_alternatives'] or 'NONE in this library'} "
          f"(evidence under both limits: {s['rec_comb']['evidence_class']}).")
        w("Recorded peaks (m/s^2) for the refuted command(s) and the alternative, from the harness traces:\n\n| policy | peak a_lat | peak smoothed a_long | peak a_comb | dyn margins |\n|---|---|---|---|---|")
        for p in list(s["cmds"]) + s["feasible_alternatives"][:1]:
            dp = s["rec_comb"]["traces"][f"{p}/static_blocker"]["margin"].get("dyn_peaks", {})
            w(f"| {p} | {fm(dp.get('a_lat'))} | {fm(dp.get('a_long_smoothed'))} | {fm(dp.get('a_comb'))} | {dp.get('dyn_margin')} |")
        if "audit" in s:
            w(f"\nAudited feasible-alternative trace (`{s['witness']}`):")
            L.extend(trace_lines(s["audit"]))
    else:
        w("\nNo command passes the a_lat-only spec and is refuted by a_comb in the searched grid; RC18 is physically implausible/uninformative at this plant (decel 6.2 m/s^2 leaves 5.05 m/s^2 of lateral budget) "
          "or the gap grid missed the window; no positive RC18 contrast is claimed.")
    n, bad_ = validate_all()
    w(f"\n## Record validation\n\n{n} records in `results/carla/compositional/` checked with validate_records.check_record: {len(bad_)} failures.")
    for b in bad_[:10]:
        w(f"- {b}")
    return "\n".join(L) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--only", default="")
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    ctx = Ctx(a.host, a.port)
    r04, r05, r06 = run_rc04(ctx), run_rc05(ctx), run_rc06(ctx)
    r08, r15, r18 = run_rc08(ctx), run_rc15(ctx), run_rc18(ctx)
    sel = select_and_audit(ctx, r04, r05, r06, r15, r18)
    text = report(ctx, r04, r05, r06, r08, r15, r18, sel)
    open("results/carla/compositional.md", "w", encoding="utf-8", newline="\n").write(text)
    print(text)


if __name__ == "__main__":
    main()
