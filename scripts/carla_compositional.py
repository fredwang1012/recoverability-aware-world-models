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
from sim import harness as H

OUT = "results/carla/compositional"
CFG = M.DEFAULT_V2
TAU = 0.25
DECEL_TARGET = CC.DECEL_TARGET
TAG = f"al{CP.A_LAT_MAX:g}c{CP.A_COMB_MAX:g}v{H.A_COMB_VMIN:g}m2"      # m2: movers pinned to declared speed in warm-up + mask after smoothing
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


def audit_full(ctx, case, theta, **kw):
    """Re-run the WHOLE library of the selected scene with a per-tick spy on every episode (same episode order as the scan, so the
    re-run is comparable); returns (record, logs) with logs[policy_idx * n_worlds + world_idx]."""
    logs = []

    def fac():
        lg = []
        logs.append(lg)
        return CC.SpyPlant(ctx.factory(), lg)
    rec = CP.run(case, fac, theta, CFG, "carla", SCOPE, physics_profile_id=ctx.pid, runtime=ctx.runtime, **kw)
    return rec, logs


def trace_of(rec, logs, policy, world, dt=0.05):
    pi, wi = rec.policies.index(policy), rec.worlds.index(world)
    return summarize_trace(logs[pi * len(rec.worlds) + wi], rec, f"{policy}/{world}", rec.R_raw[pi][wi], dt)


def rerun_diff(scan_rec, rec):
    """Max |G_rerun - G_scan| over entries finite in both; 'pattern differs' if the -inf/None pattern differs."""
    a = [gval(x) for row in scan_rec["R_raw"] for x in row]
    b = [gval(x) for row in rec.R_raw for x in row]
    d = 0.0
    for x, y in zip(a, b):
        if x is None or y is None or math.isinf(x) or math.isinf(y):
            if x != y:
                return "pattern differs"
        else:
            d = max(d, abs(x - y))
    return d


def summarize_trace(log, rec, key, g, dt):
    ego = [s["ego"] for s in log]
    lat, v = [e[1] for e in ego], [e[2] for e in ego]
    step = max((abs(lat[i + 1] - lat[i]) for i in range(len(lat) - 1)), default=0.0)
    step_ratio = max((abs(lat[i + 1] - lat[i]) / max(v[i] * dt, 1e-6) for i in range(len(lat) - 1) if v[i] > 2.0), default=0.0)
    out = {"n_ticks": len(log), "ego_v_start": v[0], "ego_v_end": v[-1], "ego_lat_min": min(lat), "ego_lat_max": max(lat), "ego_lat_end": lat[-1],
           "max_lat_step_m": step, "max_lat_step_over_v_dt": step_ratio, "x_end": ego[-1][0]}
    out["others"] = {}
    for n in [n for n in log[0] if n != "ego"]:
        o = [s[n] for s in log]
        out["others"][n] = {"v_start": o[0][2], "v_end": o[-1][2], "lat_end": o[-1][1], "x_end": o[-1][0],
                            "min_abs_dx": min(abs(a[0] - b[0]) for a, b in zip(o, ego))}
    tr = rec.traces[key]
    out.update({"run": key, "termination": rec.events[key]["termination"], "contact_events": rec.events[key]["events"], "G": g,
                "min_clearance_m": {k: (None if (x is None or x == math.inf) else float(x)) for k, x in tr["margin"]["min_clearance_m"].items()},
                "dyn_peaks": tr["margin"].get("dyn_peaks"), "route_goal_completed": tr.get("route_goal_completed"),
                "commands": rec.timestamps[key]["commands"]})
    return out


def at_time(log, t, dt=0.05):
    k = min(int(round(t / dt)), len(log) - 1)
    return log[k]


# ----------------------------------------------------------------------------- sweeps
RC04_PS_BASE, RC04_PS_EXT = (0.4, 0.8), (0.2, 0.4, 0.6, 0.8, 1.0)


def stop_dist(v):
    """Analytic braking envelope used ONLY to place the extended RC04 gap grid (b = 6.2 CARLA calibration, tau 0.3, reserve 1 m)."""
    return v * v / (2 * 6.2) + 0.3 * v + CFG.reserve_m


def run_rc04(ctx):
    """Keys (lib, lc, v, g): 'base' = first grid (v 14/18, gap 18-38 step 2, ps 0.4/0.8), 'ext' = extended search added after the base grid
    showed no contrast (v 16/18/20/22, gaps from stop_dist-10 to stop_dist step 2, ps 0.2-1.0)."""
    res = {}
    for lc in (1.6, 2.0):
        for v in (14.0, 18.0):
            for g in range(18, 40, 2):
                th = {"gap_m": float(g), "v_mps": v, "tau_s": TAU}
                res[("base", lc, v, g)] = ctx.point(f"rc04_v{v:g}_lc{lc:g}_{TAG}", f"{g}", lambda th=th, lc=lc: ctx.run("RC04", th, library=CP.rc04_library(lc)),
                                                    library_id=f"rc04_lc{lc:g}_ps0.4-0.8_{TAG}")
    for lc in (1.6, 2.0):
        for v in (16.0, 18.0, 20.0, 22.0):
            top = int(stop_dist(v))
            for g in range(top - 10, top + 1, 2):
                th = {"gap_m": float(g), "v_mps": v, "tau_s": TAU}
                res[("ext", lc, v, g)] = ctx.point(f"rc04x_v{v:g}_lc{lc:g}_{TAG}", f"{g}",
                                                   lambda th=th, lc=lc: ctx.run("RC04", th, library=CP.rc04_library(lc, RC04_PS_EXT)),
                                                   library_id=f"rc04_lc{lc:g}_ps{'-'.join(map(str, RC04_PS_EXT))}_{TAG}")
    for lib in ("base", "ext"):
        print("RC04", lib, sorted((lc, v, g) for (l, lc, v, g), r in res.items() if l == lib and rc04_contrast(r)), flush=True)
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


def tight_gaps(v, ks=(1, 4, 7, 10)):
    """Blocker gaps INSIDE the braking envelope (stop_dist(v) - k m): brake_hold cannot be a witness there."""
    return [float(int(stop_dist(v)) - k) for k in ks]


def run_tight(ctx):
    """RC15 / RC06 with the static blocker inside stop distance, so merging is the only possible witness (added in fix round 1)."""
    r15, r06 = {}, {}
    for v in (10.0, 14.0, 18.0):
        for g in tight_gaps(v):
            for ag in (25.0, 35.0):
                for var, kw in (("default", {}), ("fast", {"trigger_t": 0.0, "react_s": 0.2})):
                    th = {"gap_m": g, "v_mps": v, "tau_s": TAU, "adj_gap_m": ag, "adj_v_mps": v + 6.0, "yield_variant": var}
                    r15[(v, g, ag, var)] = ctx.point(f"rc15tight_{TAG}", f"{v:g}_{g:g}_{ag:g}_{var}",
                                                     lambda th=th, kw=kw: ctx.run("RC15", th, worlds=CP.rc15_worlds(**kw)),
                                                     library_id=f"rc15tight_lc2.0_wait2.0_drop1.5_{TAG}")
    for v in (10.0, 14.0):
        for g in tight_gaps(v, (1, 5)):
            th = {"gap_m": g, "v_mps": v, "tau_s": TAU, "pass_gap_m": 10.0, "pass_v_mps": v + 10.0}
            r06[(v, g)] = ctx.point(f"rc06tight_{TAG}", f"{v:g}_{g:g}", lambda th=th: ctx.run("RC06", th), library_id=f"rc06tight_lc2.0_{TAG}")
    print("TIGHT RC15", {k: (r["evidence_class"], rc15_contrast(r)) for k, r in r15.items() if rc15_contrast(r) or r["evidence_class"] == "R_REF"}, flush=True)
    return r15, r06


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
    # RC04: highest speed with a window (ties: base before ext, lc 1.6 before 2.0), median gap of that window
    wins = {}
    for (lib, lc, v, g), r in r04.items():
        if rc04_contrast(r):
            wins.setdefault((v, lib == "base", lc == 1.6), []).append((lib, lc, v, g))
    if wins:
        keys = sorted(wins[max(wins)])
        lib, lc, v, g = median_key(keys)
        th = {"gap_m": float(g), "v_mps": v, "tau_s": TAU}
        r = r04[(lib, lc, v, g)]
        stg = [k for k, gs in mw(r).items() if k.startswith("staged_brake") and all_ok(gs)][0]
        rec, logs = audit_full(ctx, "RC04", th, library=CP.rc04_library(lc, RC04_PS_BASE if lib == "base" else RC04_PS_EXT))
        out["RC04"] = {"sel": (lib, lc, v, g), "window": keys, "witness": stg, "record": r, "rerun": rerun_diff(r, rec),
                       "audit": trace_of(rec, logs, stg, "static_blocker"), "audit_bh": trace_of(rec, logs, "brake_hold", "static_blocker"),
                       "audit_imm": trace_of(rec, logs, "immediate_lane_change", "static_blocker")}
    else:
        out["RC04"] = None
    k5, keys5 = pick(r05, rc05_contrast)
    if k5:
        th = {"gap_m": k5[0], "v_mps": 14.0, "tau_s": TAU, "rear_gap_m": k5[1], "rear_v_mps": k5[2], "t_reveal_s": 1.25}
        rec, logs = audit_full(ctx, "RC05", th)
        out["RC05"] = {"sel": k5, "window": keys5, "witness": "abort_and_stabilize", "record": r05[k5], "rerun": rerun_diff(r05[k5], rec),
                       "audit": {wn: trace_of(rec, logs, "abort_and_stabilize", wn) for wn in rec.worlds},
                       "audit_commit": trace_of(rec, logs, "commit_lane_change", "gap_closes")}
    else:
        out["RC05"] = None
    k6, keys6 = pick(r06, rc06_contrast)
    if k6:
        th = {"gap_m": k6[0], "v_mps": 10.0, "tau_s": TAU, "pass_gap_m": k6[1], "pass_v_mps": k6[2]}
        rec, logs = audit_full(ctx, "RC06", th)
        out["RC06"] = {"sel": k6, "window": keys6, "witness": "wait_observe_enter", "record": r06[k6], "rerun": rerun_diff(r06[k6], rec),
                       "audit": trace_of(rec, logs, "wait_observe_enter", "passer_cruise"),
                       "audit_immediate": trace_of(rec, logs, "immediate_entry", "passer_cruise")}
    else:
        out["RC06"] = None
    strong = [k for k, r in r15.items() if rc15_contrast(r) == "strong"]
    weak = [k for k, r in r15.items() if rc15_contrast(r) in ("strong", "weak")]
    k15 = median_key(strong) or median_key(weak)
    if k15:
        th = {"gap_m": k15[0], "v_mps": 10.0, "tau_s": TAU, "adj_gap_m": k15[1], "adj_v_mps": k15[2]}
        rec, logs = audit_full(ctx, "RC15", th)
        out["RC15"] = {"sel": k15, "kind": rc15_contrast(r15[k15]), "strong": strong, "weak": weak, "witness": "observe_yield_else_fallback",
                       "record": r15[k15], "rerun": rerun_diff(r15[k15], rec),
                       "audit": {wn: trace_of(rec, logs, "observe_yield_else_fallback", wn) for wn in rec.worlds},
                       "audit_always": {wn: trace_of(rec, logs, "always_merge", wn) for wn in rec.worlds}}
    else:
        out["RC15"] = None
    cand = [(v, g) for (v, g, c) in r18 if c and rc18_refuted_by_comb(r18[(v, g, False)], r18[(v, g, True)])]
    kc = median_key(cand)
    if kc:
        v, g = kc
        cmds = rc18_refuted_by_comb(r18[(v, g, False)], r18[(v, g, True)])
        feas = [p for p, gs in mw(r18[(v, g, True)]).items() if all_ok(gs)]
        out["RC18"] = {"sel": kc, "window": cand, "cmds": cmds, "rec_nocomb": r18[(v, g, False)], "rec_comb": r18[(v, g, True)],
                       "feasible_alternatives": feas}
        th = {"gap_m": float(g), "v_mps": v, "tau_s": TAU}
        rec, logs = audit_full(ctx, "RC18", th, library=CP.rc18_library(RC18_LCS, 2.0), spec=CP.rc18_spec(th, CFG, comb=True))
        out["RC18"]["rerun"] = rerun_diff(r18[(v, g, True)], rec)
        out["RC18"]["audit_cmd"] = trace_of(rec, logs, cmds[0], "static_blocker")
        if feas:
            out["RC18"]["witness"] = feas[0]
            out["RC18"]["audit"] = trace_of(rec, logs, feas[0], "static_blocker")
    else:
        out["RC18"] = None
    return out


# ----------------------------------------------------------------------------- report
def ev_row(r):
    return r["evidence_class"]


def _rnd(o):
    return {k: _rnd(v) for k, v in o.items()} if isinstance(o, dict) else (round(o, 3) if isinstance(o, float) else o)


def trace_lines(a, indent="  "):
    L = [f"{indent}- run `{a['run']}`: termination {a['termination']}, contact events {a['contact_events'] or 'none'}, G (re-run) {fm(gval(a['G']))}",
         f"{indent}- ego v {a['ego_v_start']:.2f} -> {a['ego_v_end']:.2f} m/s; lateral range [{a['ego_lat_min']:.2f}, {a['ego_lat_max']:.2f}] m, end {a['ego_lat_end']:.2f}; "
         f"x_end {a['x_end']:.1f}; max per-tick lateral step {a['max_lat_step_m']:.3f} m = {a['max_lat_step_over_v_dt']:.3f} x (v*dt) (continuous, no teleport)",
         f"{indent}- min clearance to each actor (m): {({k: (None if v is None else round(v, 2)) for k, v in a['min_clearance_m'].items()})}",
         f"{indent}- dynamic peaks: {_rnd(a['dyn_peaks'] or {})}",
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


def report(ctx, r04, r05, r06, r08, r15, r18, sel, tight):
    L, w = [], None
    w = L.append
    w("# CARLA compositional cases (Task 6: RC04, RC05, RC06, RC08, RC15, RC18)\n")
    w(f"Physics profile `{ctx.pid}`, decel target {DECEL_TARGET}, eps = {CFG.eps}, reserve c = {CFG.reserve_m} m (margin.DEFAULT_V2, unchanged), act delay {TAU} s. "
      "Records: `results/carla/compositional/*.jsonl`. Labels are `contract.classify` output only. G cells below read `world1/world2` (-inf = hard failure, None = unrun).\n")
    w(f"**Authority.** Every spec declares `a_lat_max = {CP.A_LAT_MAX:g}` m/s^2 (dynamic-limit margin from |v * yaw_rate|, as in the core cases); RC18 additionally declares "
      f"`a_comb_max = {CP.A_COMB_MAX:g}` m/s^2, a friction-circle margin sqrt(a_long^2 + a_lat^2) <= limit from MEASURED accelerations "
      f"(a_long = tick difference of ego speed, centred {H.A_LONG_SMOOTH_TICKS}-tick moving average = 0.25 s, declared before any run; a_lat = |v * yaw_rate| per tick, unsmoothed; ticks with ego speed < {H.A_COMB_VMIN:g} m/s are excluded from a_comb (the mask is applied AFTER smoothing, so no smoothing window is truncated) because CARLA's brake model locks the wheels at the end of a stop, measured -28..-35 m/s^2 for ~4 ticks below ~4.3 m/s in a brake_hold diagnostic at v 18; the unmasked peak is still recorded as `a_comb_unmasked`). "
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
    w("Library (per lc_time L, L in 1.6 and 2.0): brake_hold, lane_follow, immediate_lane_change (Bypass, L), staged_brake_{p}_change (StagedBrakeChange, L) with p = 0.4, 0.8 in the base grid and p = 0.2, 0.4, 0.6, 0.8, 1.0 in the extended grid. "
      "Immediate lane change is a different candidate at each L; scenes where the immediate change at lc 2.0 already passes are therefore not witnesses of the staged policy in the lc-2.0 library.\n")
    w("| grid | lc | v | gap | evidence | contrast | margins |\n|---|---|---|---|---|---|---|")
    for (lib, lc, v, g), r in sorted(r04.items()):
        w(f"| {lib} | {lc:g} | {v:g} | {g} | {r['evidence_class']} | {rc04_contrast(r)} | {mrows(r)} |")
    s = sel["RC04"]
    if s:
        w(f"\nSelected: {s['sel']} (window {s['window']}); witness `{s['witness']}`; re-run vs scan max |dG| = {s['rerun']}.\n\nAudited witness trace summary:")
        L.extend(trace_lines(s["audit"]))
        w("\nbrake_hold trace (refuted):")
        L.extend(trace_lines(s["audit_bh"]))
        w("\nimmediate_lane_change trace (refuted):")
        L.extend(trace_lines(s["audit_imm"]))
    else:
        w("\nNo scene shows the contrast (a staged policy passes while brake_hold and the immediate change both fail): no positive RC04 witness on CARLA under the 8.0 m/s^2 limit "
          "in the searched grid.  Searched: base grid lc {1.6, 2.0} x v {14, 18} x gap 18-38 step 2 (ps 0.4, 0.8); extended grid lc {1.6, 2.0} x v {16, 18, 20, 22} x gap [stop_dist-10, stop_dist] step 2 (ps 0.2-1.0).  "
          "Nothing is forced.")
        near = [(k, mw(r)) for k, r in sorted(r04.items())
                if any(all_ok(gs) for p, gs in mw(r).items() if p.startswith("staged_brake")) and any_bad(mw(r)["brake_hold"])]
        imm_ok = [k for k, m in near if all_ok(m["immediate_lane_change"])]
        w("Scenes where a staged policy passes and brake_hold fails (the only candidates): " +
          (("; ".join(f"{k}: immediate_lane_change={'/'.join(fm(x) for x in m['immediate_lane_change'])}" for k, m in near)) if near else "none") +
          f".  Of these, the immediate lane change also passes in {len(imm_ok)} of {len(near)}" +
          (" (so the staged policy is not the unique supported continuation there)" if imm_ok else "") + ".")
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
        w(f"\nSelected: {s['sel']} (window size {len(s['window'])}); witness `abort_and_stabilize`; full-library re-run vs scan max |dG| = {s['rerun']}.\n")
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
        w(f"\nSelected: {s['sel']}; witness `wait_observe_enter`; full-library re-run vs scan max |dG| = {s['rerun']}.\n\nAudited witness trace summary:")
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
        w(f"\nSelected ({s['kind']} contrast; strong scenes {s['strong']}, weak-or-strong scenes {len(s['weak'])}): {s['sel']}; witness `observe_yield_else_fallback`; full-library re-run vs scan max |dG| = {s['rerun']}.\n")
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
    flips = [(v, g, r18[(v, g, False)]["evidence_class"], r["evidence_class"]) for (v, g, c), r in sorted(r18.items())
             if c and r18[(v, g, False)]["evidence_class"] != r["evidence_class"]]
    w(f"\nScenes whose evidence class changes when the combined limit is added (a_lat-only -> a_lat + a_comb): {flips or 'none'}.  "
      "A change R_REF -> U_LIBRARY means the only witness was a command outside the declared acceleration authority: the scene is then negative only relative to this library, never U_MODEL.")
    s = sel["RC18"]
    if s:
        v, g = s["sel"]
        w(f"\nSelected scene v {v:g}, gap {g}: commands that pass the a_lat-only spec and fail once a_comb_max is added: {s['cmds']}; feasible alternatives under both limits: {s['feasible_alternatives'] or 'NONE in this library'} "
          f"(evidence under both limits: {s['rec_comb']['evidence_class']}).")
        w("Recorded peaks (m/s^2) with the time and ego speed at each peak (a_long / a_comb only count while v >= " + f"{H.A_COMB_VMIN:g} m/s), from the harness traces of the a_lat + a_comb run:\n\n"
          "| policy | peak a_lat | peak a_long (masked) @ t, v | peak a_comb (masked) @ t, v | peak a_comb unmasked @ t, v | a_long alone >= limit | dyn margins |\n|---|---|---|---|---|---|---|")
        lim = CP.A_COMB_MAX
        for p in list(s["cmds"]) + [x for x in ("brake_hold",) if x not in s["cmds"]] + s["feasible_alternatives"][:1]:
            dp = s["rec_comb"]["traces"][f"{p}/static_blocker"]["margin"].get("dyn_peaks", {})
            pt = dp.get("peak_times", {})
            at = lambda k: (f"@ {pt[k]['t']:.2f} s, {pt[k]['v']:.1f} m/s" if k in pt else "")
            w(f"| {p} | {fm(dp.get('a_lat'))} | {fm(dp.get('a_long_smoothed'))} {at('a_long')} | {fm(dp.get('a_comb'))} {at('a_comb')} | "
              f"{fm(dp.get('a_comb_unmasked'))} {at('a_comb_unmasked')} | {dp.get('a_long_smoothed', 0) >= lim} | {dp.get('dyn_margin')} |")
        cmd_dp = s["rec_comb"]["traces"][f"{s['cmds'][0]}/static_blocker"]["margin"]["dyn_peaks"]
        if cmd_dp["a_long_smoothed"] >= lim:
            w(f"\n**Framing.** The refuted command's peak a_long ({cmd_dp['a_long_smoothed']:.2f}) already exceeds {lim:g} m/s^2 on its own, so its refutation holds for the longitudinal acceleration alone; "
              "it is NOT shown to be an effect of the combination.  The a_comb margin is the declared friction-circle authority, but these numbers do not isolate a combined-only violation.")
        else:
            w(f"\n**Framing.** The command's peak a_long ({cmd_dp['a_long_smoothed']:.2f}) stays below {lim:g} m/s^2 while its a_comb peak ({cmd_dp['a_comb']:.2f}) exceeds it: the violation is a combined-limit effect.")
        w(f"\nFull-library re-run vs scan max |dG| = {s['rerun']}.  Audited refuted-command trace (`{s['cmds'][0]}`):")
        L.extend(trace_lines(s["audit_cmd"]))
        if "audit" in s:
            w(f"\nAudited feasible-alternative trace (`{s['witness']}`):")
            L.extend(trace_lines(s["audit"]))
    else:
        w("\nNo command passes the a_lat-only spec and is refuted by a_comb in the searched grid; RC18 is physically implausible/uninformative at this plant (decel 6.2 m/s^2 leaves 5.05 m/s^2 of lateral budget) "
          "or the gap grid missed the window; no positive RC18 contrast is claimed.")
    r15t, r06t = tight
    w("\n## RC15 / RC06 with the blocker inside stop distance (merge is the only possible witness)\n")
    w("RC15 disclosure: in the main RC15 grid brake_hold is a witness in BOTH worlds, so the evidence label (R_REF) carries no information about whether the neighbour yields; "
      "the yield/no-yield contrast lives only in the per-policy margins (always_merge refuted in adj_nonyield).  The grid below puts the static blocker inside the braking envelope "
      "(gap = stop_dist(v) - k m, stop_dist = v^2/(2*6.2) + 0.3 v + 1), v 10/14/18, adj_v = v + 6, adj_gap 25/35, for the default yield timing (reacts from t = 0.5 s) and a fast one "
      "(trigger_t 0, react 0.2 s).\n")
    w("| v | gap | adj_gap | yield timing | evidence | contrast | margins (adj_yields/adj_nonyield) |\n|---|---|---|---|---|---|---|")
    for k, r in sorted(r15t.items()):
        w(f"| {k[0]:g} | {k[1]:g} | {k[2]:g} | {k[3]} | {r['evidence_class']} | {rc15_contrast(r)} | {mrows(r)} |")
    merge_only = [k for k, r in r15t.items() if all(not ok(g) for g in mw(r)["brake_hold"]) and all_ok(mw(r)["observe_yield_else_fallback"])]
    info = [k for k, r in r15t.items() if r["evidence_class"] == "U_INFORMATION"]
    w(f"\nTight scenes where brake_hold fails and observe_yield_else_fallback passes both worlds (a recoverability-relevant contrast: always_merge refuted in the bad world, R_REF only through observation): {merge_only or 'NONE'}.  "
      f"Scenes labelled U_INFORMATION: {info or 'none'}.  R_REF scenes: {sorted(k for k, r in r15t.items() if r['evidence_class'] == 'R_REF') or 'none'}.")
    w("\nRC06 tight scenes (pass_gap 10, pass_v = v + 10):\n\n| v | gap | evidence | margins |\n|---|---|---|---|")
    for k, r in sorted(r06t.items()):
        w(f"| {k[0]:g} | {k[1]:g} | {r['evidence_class']} | {mrows(r)} |")
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
    tight = run_tight(ctx)
    text = report(ctx, r04, r05, r06, r08, r15, r18, sel, tight)
    open("results/carla/compositional.md", "w", encoding="utf-8", newline="\n").write(text)
    print(text)


if __name__ == "__main__":
    main()
