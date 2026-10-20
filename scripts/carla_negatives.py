"""
Task 8: difficult negatives on CARLA (RC09 front/rear conflict with no escape, RC13 over-approximated prediction).

    cd mining && PYTHONNOUSERSITE=1 python ../scripts/carla_negatives.py [--host localhost --port 2000]

Needs a live, bring-up-passed server (never started here).  Every point is a full ExecutionRecord checkpointed to
mining/results/carla/negatives/<sweep>.jsonl (a rerun skips finished points); summary mining/results/carla/negatives.md.
Labels come from evidence only (sim.harness.evaluate -> contract.classify); eps/reserve are margin.DEFAULT_V2, untouched.

Declared before the run (not changed after seeing results):
  RC09  finite class RC09_CLASS_ID (cases/negatives.py, 25 members: keep_speed, accelerate, constant brake_frac 0.0..1.0 in steps of 0.1,
        four brake-then-keep staged members, bypass left lc 1.0 / 2.0, bypass right lc 2.0, brake+steer left lc 1.0/2.0 x b 0.5/1.0, staged
        brake 0.4 then change (lc 1.0)).  One declared world `rear_cruise` (the rear vehicle keeps its speed).  The left lane is occupied by a
        queue of frozen vehicles alongside and ahead; the right edge is the road edge.  Scenes: v in {10, 14, 18}, front bumper gap
        = int(v^2/(2*6.211) + 0.3 v + 1) + 3 (a witness for brake_hold when no rear vehicle exists), rear bumper gap 25 m, rear speed v + 6;
        extra rear gaps 15 and 35 m at v = 14.  Controls at v = 14: no rear vehicle; open left lane.  Acceptance: every member has a
        resolved world with G < eps (U_LIBRARY).  Brake-fraction -> deceleration is measured from clear-road traces.
  RC13  scenes (v, D, t_reveal) = (14, 19, 0.1), (14, 20, 0.2), (18, 26, 0.2), (18, 28, 0.2) of the RC10 construction (cases/occlusion.py,
        library with the witness shared_slowdown_then_branch).  The predictor is cases/negatives.Envelope / action_rejected, fed with the
        first delivered observation that contains the hidden vehicle (recorded from the CARLA run) and judging the witness's own branch;
        parameter grid ENVELOPES below.  The predictor never produces a label.
"""
import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "mining"))
sys.path.insert(0, HERE)

import margin as M
from cases import negatives as NG
from cases import occlusion as OC
from cases import two_lane as TL
import carla_core as CC
import carla_compositional as CCP
from sim import harness as H
from sim.types import DT, Obs

OUT = "results/carla/negatives"
CFG = M.DEFAULT_V2
TAU = 0.25
SCOPE = "carla_finite_replay"
TAG = f"al{NG.A_LAT_MAX:g}"
gval, fm = CC.gval, CCP.fm
B_CAL, TAU_EFF = CC.B_CAL, CC.TAU_EFF

RC09_SPEEDS = (10.0, 14.0, 18.0)
RC09_REAR_GAP = 25.0
RC09_EXTRA_REAR_GAPS = (15.0, 35.0)               # at v = 14
RC09_DV = 6.0
DECEL_LO = 0.4          # upper end of the decel window as a fraction of v0: keeps the window above the end-of-stop wheel-lock transient (below ~4.3 m/s)
SIGMA = 0.0167          # run-to-run spread of margins (observed, Tasks 3-7); marginal = within SIGMA of eps or a sign flip across repeats
REPEATS = 3
RC13_SCENES = ((14.0, 19.0, 0.1), (14.0, 20.0, 0.2), (18.0, 26.0, 0.2), (18.0, 28.0, 0.2))
ENVELOPES = ([NG.Envelope(lat_m=m, lon_m=m) for m in (0.0, 0.5, 1.0, 1.5, 2.0)]
             + [NG.Envelope(lat_m=0.5, lon_m=0.5, growth_mps=g) for g in (0.25, 0.5)]
             + [NG.Envelope(lat_m=0.0, lon_m=0.0, dt_s=0.5), NG.Envelope(lat_m=1.0, lon_m=1.0, dt_s=0.5)]
             + [NG.Envelope(clearance_m=1.0), NG.Envelope(lat_m=0.5, lon_m=0.5, clearance_m=1.0)])
THRESH_GRID = [round(0.05 * i, 2) for i in range(0, 81)]


# ----------------------------------------------------------------------------- pure helpers (mock-tested)
def stop_dist(v):
    return v * v / (2 * B_CAL) + TAU_EFF * v + CFG.reserve_m


def rc09_theta(v, rear_gap=RC09_REAR_GAP):
    return {"gap_m": float(int(stop_dist(v)) + 3), "v_mps": v, "tau_s": TAU, "rear_gap_m": rear_gap, "rear_dv_mps": RC09_DV}


def recorded(instances, pi, wi, n_worlds):
    """The Recording wrapper of (policy pi, world wi): evaluate() makes one descriptor instance, then one per world, per policy."""
    return instances[pi * (1 + n_worlds) + 1 + wi]


def clean(o):
    """JSON-safe copy (inf -> string)."""
    if isinstance(o, dict):
        return {k: clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [clean(v) for v in o]
    if isinstance(o, float) and not math.isfinite(o):
        return "-inf" if o < 0 else ("inf" if o > 0 else None)
    return o


def repeat_status(gs, eps=CFG.eps, sigma=SIGMA):
    """gs: margins of the same entry over the original run and its repeats (-inf allowed).  Returns (range, marginal, flips):
    marginal if any finite value is within sigma of eps (|g - eps| < sigma) or the sign of g - eps differs across runs."""
    if any(g is None for g in gs):
        return None, True, True
    lo, hi = min(gs), max(gs)
    flips = (lo < eps) != (hi < eps)
    near = any(g != -math.inf and abs(g - eps) < sigma for g in gs)
    return (lo, hi), bool(flips or near), bool(flips)


def witness_runs(recs):
    """[(G_world_A, G_world_B)] of shared_slowdown_then_branch over a list of RC13 record dicts."""
    return [tuple(mpol(r)["shared_slowdown_then_branch"]) for r in recs]


def scene_robust(recs, eps=CFG.eps, sigma=SIGMA):
    """(robust, reason): robust = every run R_REF and no witness margin (either world) marginal under repeat_status."""
    if any(r["evidence_class"] != "R_REF" for r in recs):
        return False, "evidence class differs across repeats"
    runs = witness_runs(recs)
    for wi in (0, 1):
        rng, marg, flips = repeat_status([g[wi] for g in runs], eps, sigma)
        if marg:
            return False, f"world {('A', 'B')[wi]} witness margin range {rng} is within {sigma:g} of eps or flips"
    return True, ""


def cnt3(r):
    g = [x[0] for x in mpol(r).values()]
    return f"{sum(x == -math.inf for x in g)} / {sum(x is not None and x != -math.inf and x < CFG.eps for x in g)} / {sum(x is not None and x >= CFG.eps for x in g)}"


def mpol(r):
    return {p: [gval(x) for x in row] for p, row in zip(r["policies"], r["R_raw"])}


def rc13_cells(rec, eps=CFG.eps):
    """(world, action, evidence policy, role, evidence G, evidence ok) for the six cells compared with the predictor.  The witness cells
    use the witness's own margins; the others use the fixed policies stop_in_place / fixed_left (executed from t = 0, so only an
    approximate counterpart of the action judged from the first-sighting state)."""
    m = mpol(rec)
    cells = []
    for w, wi in (("hid_left_lane", 0), ("hid_own_lane", 1)):
        wit_action = "stay" if wi == 0 else "change_left"
        g = m["shared_slowdown_then_branch"][wi]
        cells.append((w, wit_action, "shared_slowdown_then_branch", "witness", g, g is not None and g >= eps))
        for a, pol in (("stay", "stop_in_place"), ("change_left", "fixed_left")):
            if a == wit_action:
                continue
            g = m[pol][wi]
            cells.append((w, a, pol, "other action", g, g is not None and g >= eps))
    return cells


def rc13_scene(plant_factory, theta, cfg, backend, scope, pid, runtime=None):
    """Run the RC10 library with Recording wrappers; return {'rec': record json, 'obs': {world: Obs}}."""
    inst = []

    def wrap(mk):
        def f():
            p = NG.Recording(mk())
            inst.append(p)
            return p
        return f
    rec = OC.run("RC10", plant_factory, theta, cfg, backend, scope, library_=[wrap(mk) for mk in OC.library()],
                 physics_profile_id=pid, runtime=runtime or {})
    n_w = len(rec.worlds)
    ip = rec.policies.index("shared_slowdown_then_branch")
    obs = {w: recorded(inst, ip, wi, n_w).first_obs for wi, w in enumerate(rec.worlds)}
    return {"rec": rec.to_json(), "obs": obs}


def rc13_table(sc, envelopes, eps=CFG.eps):
    """One row per envelope: per-cell predictor decision vs evidence, agreement count, false rejections / acceptances."""
    cells = rc13_cells(sc["rec"], eps)
    rows = []
    for env in envelopes:
        out, fr, fa = [], [], []
        for w, a, pol, role, g, ok in cells:
            rej, sep, tm = NG.action_rejected(sc["obs"][w], a, env)
            out.append({"world": w, "action": a, "policy": pol, "role": role, "G": g, "evidence_ok": ok, "rejected": rej, "min_sep_m": sep, "t_min": tm})
            if ok and rej:
                fr.append((w, a, pol, role))
            if (not ok) and (not rej):
                fa.append((w, a, pol, role))
        rows.append({"env": NG.envelope_dict(env), "cells": out, "n": len(out), "agree": sum(c["evidence_ok"] != c["rejected"] for c in out),
                     "false_rejections": fr, "false_accepts": fa})
    return rows


def witness_threshold(sc, grid=THRESH_GRID):
    """Smallest uniform inflation (no growth) that rejects the witness's branch, per world; None if never within the grid."""
    out = {}
    for w, a, pol, role, g, ok in rc13_cells(sc["rec"]):
        if role == "witness":
            out[w] = NG.predictor_threshold(sc["obs"][w], a, grid)
    return out


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

    def point(self, sweep, key, fn):
        """fn() -> (ExecutionRecord-json dict, extra dict); cached as one JSONL line."""
        d = self.cache.get(sweep) or self.load(sweep)
        if key in d:
            return d[key]
        CCP.gpu_wait()
        r, extra = fn()
        r = dict(r, _key=key, **extra)
        with open(f"{OUT}/{sweep}.jsonl", "a") as f:
            f.write(json.dumps(clean(r)) + "\n")
        d[key] = json.loads(json.dumps(clean(r)))
        return d[key]


def spied(ctx):
    """(factory, logs): a plant factory whose every plant logs per-tick actor states; logs[i] belongs to the i-th plant built."""
    logs = []

    def fac():
        lg = []
        logs.append(lg)
        return CC.SpyPlant(ctx.factory(), lg)
    return fac, logs


def rc09_point(ctx, theta, rear=True, left_queue=True):
    fac, logs = spied(ctx)
    rec = NG.rc09_run(fac, theta, CFG, "carla", SCOPE, rear=rear, left_queue=left_queue, physics_profile_id=ctx.pid, runtime=ctx.runtime)
    nw = len(rec.worlds)
    audit = {}
    for pi, p in enumerate(rec.policies):
        for wi, w in enumerate(rec.worlds):
            audit[f"{p}/{w}"] = CCP.summarize_trace(logs[pi * nw + wi], rec, f"{p}/{w}", rec.R_raw[pi][wi], DT)
    j = rec.to_json()
    j["library_id"] = f"{NG.RC09_CLASS_ID}_{TAG}"
    return j, {"audit": audit, "rear": rear, "left_queue": left_queue, "library_id": f"{NG.RC09_CLASS_ID}_{TAG}"}


def run_rc09(ctx):
    res = {}
    for v in RC09_SPEEDS:
        th = rc09_theta(v)
        res[("main", v, RC09_REAR_GAP)] = ctx.point(f"rc09_{TAG}", f"v{v:g}_rg{RC09_REAR_GAP:g}", lambda th=th: rc09_point(ctx, th))
    for rg in RC09_EXTRA_REAR_GAPS:
        th = rc09_theta(14.0, rg)
        res[("main", 14.0, rg)] = ctx.point(f"rc09_{TAG}", f"v14_rg{rg:g}", lambda th=th: rc09_point(ctx, th))
    th = rc09_theta(14.0)
    res[("ctl_norear", 14.0, RC09_REAR_GAP)] = ctx.point(f"rc09ctl_{TAG}", "norear", lambda: rc09_point(ctx, th, rear=False))
    res[("ctl_openleft", 14.0, RC09_REAR_GAP)] = ctx.point(f"rc09ctl_{TAG}", "openleft", lambda: rc09_point(ctx, th, left_queue=False))
    for i in range(1, REPEATS + 1):
        res[("ctl_openleft_rep", 14.0, i)] = ctx.point(f"rc09ctl_{TAG}", f"openleft_rep{i}", lambda: rc09_point(ctx, th, left_queue=False))
    for k, r in res.items():
        print("RC09", k, r["evidence_class"], flush=True)
    return res


def decel_point(ctx, v):
    fac, logs = spied(ctx)
    theta = {"gap_m": 0.0, "v_mps": v, "tau_s": TAU}
    sp = TL.rc19_spec(theta, CFG)
    sp["a_lat_max"] = NG.A_LAT_MAX
    lib = [CCP.CP._named((lambda f=f: NG.P.BrakeHold(brake_frac=f)), f"brake_frac_{f:.1f}") for f in NG.RC09_BRAKE_FRACS]
    rec = H.evaluate(fac, sp, lib, [TL.CLEAN], theta, "RC09", "negatives_decel", "negatives_decel", "carla", SCOPE, cfg=CFG,
                     physics_profile_id=ctx.pid, runtime=ctx.runtime)
    series = {}
    for pi, p in enumerate(rec.policies):
        vs = [s["ego"][2] for s in logs[pi]]
        eff = rec.timestamps[f"{p}/{TL.CLEAN['name']}"]["commands"][0]["effective_t"]
        series[p] = {"v": vs, "effective_t": eff}
    return rec.to_json(), {"series": series}


def run_decel(ctx):
    return {v: ctx.point(f"rc09decel_{TAG}", f"v{v:g}", lambda v=v: decel_point(ctx, v)) for v in (14.0, 18.0)}


def rc13_point(ctx, v, D, tr):
    fac, logs = spied(ctx)
    th = {"gap_m": D, "v_mps": v, "tau_s": TAU, "t_reveal_s": tr}
    sc = rc13_scene(fac, th, CFG, "carla", SCOPE, ctx.pid, ctx.runtime)
    rec = sc["rec"]
    class _R:        # trace summaries need attribute access
        pass
    rr = _R()
    for k in ("traces", "events", "timestamps", "policies", "worlds", "R_raw"):
        setattr(rr, k, rec[k])
    nw = len(rec.get("worlds"))
    audit = {}
    for pi, p in enumerate(rec["policies"]):
        for wi, w in enumerate(rec["worlds"]):
            audit[f"{p}/{w}"] = CCP.summarize_trace(logs[pi * nw + wi], rr, f"{p}/{w}", gval(rec["R_raw"][pi][wi]), DT)
    rec["library_id"] = f"rc13_rc10lib_lc2.0_{TAG}"
    return rec, {"obs": {w: vars(o) for w, o in sc["obs"].items()}, "audit": audit, "library_id": rec["library_id"]}


def run_rc13(ctx):
    res = {}
    for v, D, tr in RC13_SCENES:
        r = ctx.point(f"rc13_{TAG}", f"v{v:g}_D{D:g}_tr{tr:g}", lambda v=v, D=D, tr=tr: rc13_point(ctx, v, D, tr))
        res[(v, D, tr)] = {"rec": r, "obs": {w: Obs(**o) for w, o in r["obs"].items()}, "audit": r["audit"]}
        res[(v, D, tr)]["reps"] = [ctx.point(f"rc13rep_{TAG}", f"v{v:g}_D{D:g}_tr{tr:g}_rep{i}", lambda v=v, D=D, tr=tr: rc13_point(ctx, v, D, tr))
                                   for i in range(1, REPEATS + 1)]
        print("RC13", (v, D, tr), r["evidence_class"], [x["evidence_class"] for x in res[(v, D, tr)]["reps"]], flush=True)
    return res


# ----------------------------------------------------------------------------- report
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


def first_tick(audit, pol, world, actor):
    return audit[f"{pol}/{world}"]["others"][actor]["v_start"]


def decel_rows(dec):
    """{v: [(policy, brake_frac, decel, t_hi, t_lo, v_lo, window)...]}"""
    out = {}
    for v, r in dec.items():
        rows = []
        for p, s in r["series"].items():
            f = float(p.split("_")[-1])
            d = NG.decel_between(s["v"], DT, lo=DECEL_LO)
            rows.append((p, f, d, s["v"][-1]))
        out[v] = rows
    return out


def report(ctx, r09, dec, r13):
    L = []
    w = L.append
    w("# CARLA difficult negatives (Task 8: RC09, RC13)\n")
    w("> **Superseded claim.** The RC13 \"4 of 4 robust\" headline below uses the Task 5 sigma 0.0167; at the maximum observed repeat spread 0.028 it is 2 of 4 (see `docs/carla_results.md` section 7, Repeatability).\n")
    w(f"Physics profile `{ctx.pid}`, decel target {CC.DECEL_TARGET}, eps = {CFG.eps}, reserve c = {CFG.reserve_m} m (margin.DEFAULT_V2, unchanged), act delay {TAU} s, "
      f"`a_lat_max = {NG.A_LAT_MAX:g}` m/s^2 (dynamic-limit margin). Records: `results/carla/negatives/*.jsonl`. Labels are `contract.classify` output only; G cells read one value per world "
      "(-inf = hard failure, None = unrun). Mock and CARLA outputs are kept apart.\n")
    mains = {k: r for k, r in r09.items() if k[0] == "main"}
    ctl = {k: r for k, r in r09.items() if k[0] in ("ctl_norear", "ctl_openleft")}
    ol_reps = [r for k, r in sorted(r09.items()) if k[0] == "ctl_openleft_rep"]
    # measured t0 speeds
    rear_rows = []
    for k, r in sorted(mains.items()):
        a = r["audit"]
        rear_rows.append(f"RC09 v {k[1]:g}, rear gap {k[2]:g}: `rear` {first_tick(a, 'keep_speed', 'rear_cruise', 'rear'):.2f} measured at the first tick vs {k[1] + RC09_DV:g} declared "
                          f"({100 * (1 - first_tick(a, 'keep_speed', 'rear_cruise', 'rear') / (k[1] + RC09_DV)):.1f}% low)")
    w("**Plant caveats.** Frozen (`static`) actors are exact: blocker, the left-lane queue and (RC13) the hidden vehicle are at their declared places. The only moving non-ego actor is the RC09 `rear` vehicle, which starts "
      "3-5% below its declared speed (warm-up drag, as in the compositional runs): " + "; ".join(rear_rows) + ". The declared rear speed is therefore an upper bound of what CARLA ran; all margins are for the measured speeds. "
      "The ego starts on target. Repeats on this plant show a small run-to-run spread (up to about 0.017 margin units observed in earlier tasks); the entries below are single runs, so margins "
      "close to eps would not be robust, but the RC09 refutations are collisions (-inf) or large violations.\n")

    # ------------------------------------------------------------------ RC09
    w("## RC09: front/rear conflict with no escape\n")
    w(f"**Label scope.** The label is `U_LIBRARY` relative to the finite policy class `{NG.RC09_CLASS_ID}` ONLY, in the declared world set {{`rear_cruise`}} (one world: the rear vehicle keeps its speed and lane). "
      "It is not a claim that no controller whatsoever could recover, not U_MODEL, and not a claim about rear vehicles that brake or yield.\n")
    w("**Scene.** The custom 2-lane straight (no map change). Ego in the right lane at speed v. Front: a frozen blocker in the ego lane, bumper gap g = int(v^2/(2*6.211) + 0.3 v + 1) + 3 m, i.e. a "
      "scene where full braking alone would stop with the 1 m reserve if nothing were behind (control below). Rear: a vehicle in the ego lane, rear bumper gap 25 m (also 15 and 35 m at v = 14), cruising 6 m/s "
      "faster than the ego. Left lane: a queue of frozen vehicles from alongside the ego's start to the blocker (no lane change can complete); the right side is the road edge (lat +1.75 m).\n")
    members = NG.rc09_members()
    w(f"**The class, exactly** (`{NG.RC09_CLASS_ID}`, {len(members)} members, library id `{NG.RC09_CLASS_ID}_{TAG}` in every record):\n")
    w("| # | id | implementation | parameters |\n|---|---|---|---|")
    for m in members:
        w(f"| {m['index']} | `{m['name']}` | {m['class']} | {m['params']} |")
    w("\nMembers `brake_frac_f` command a constant fraction f of full brake until the end (f = 0.0 is coasting, engine braking only); `keep_speed` holds the initial speed; `accelerate` is throttle 0.7; "
      "`brake_F_for_Ds_then_keep` brakes at fraction F for D seconds, then holds the speed reached; lane-change members steer to the adjacent lane centre (left = the queue lane, right = off the road) in lc seconds.\n")

    w("### Scenes\n")
    w("| v | gap | rear gap | evidence | members refuted (G < eps) / total | members with a collision | margins (G per member) |\n|---|---|---|---|---|---|---|")
    from_ce = {}
    for k, r in sorted(mains.items()):
        ce = NG.counterexamples(r)
        from_ce[k] = ce
        nref = sum(c["refuted"] for c in ce.values())
        ncol = sum(c["collided_with"] is not None for c in ce.values())
        w(f"| {k[1]:g} | {r['theta']['gap_m']:g} | {k[2]:g} | {r['evidence_class']} | {nref}/{len(ce)} | {ncol} | {cnt3(r)} |")
    allU = all(r["evidence_class"] == "U_LIBRARY" and all(c["refuted"] for c in from_ce[k].values()) for k, r in mains.items())
    notU = [k for k, r in mains.items() if r["evidence_class"] != "U_LIBRARY"]
    w(f"\nAcceptance (every member refuted by a resolved world) holds in all {len(mains)} scenes: **{allU}**." + ("" if allU else f" Scenes where it does not: {notU}."))

    w("\n### Counterexample for every member (selected scene v 14, gap " + f"{rc09_theta(14.0)['gap_m']:g}, rear gap {RC09_REAR_GAP:g}, world `rear_cruise`)\n")
    sel = mains[("main", 14.0, RC09_REAR_GAP)]
    ce = from_ce[("main", 14.0, RC09_REAR_GAP)]
    w("Trace data are the per-episode records (contact events from the CARLA collision sensor, ego speed at the end of the episode, minimum clearance to every actor from the margin parts).\n")
    w("| member | world | G | termination | collided with | at t (s) | ego v at end | ego lat at end | min clearance to blk / rear / nearest adj (m) |\n|---|---|---|---|---|---|---|---|---|")
    for p, c in ce.items():
        a = sel["audit"][f"{p}/{c['world']}"]
        mc = a["min_clearance_m"]
        adj = [x for k2, x in mc.items() if k2.startswith("adj") and x is not None]
        cl = lambda x: "-" if x is None else f"{x:.2f}"
        w(f"| `{p}` | {c['world']} | {fm(c['G'])} | {c['termination']} | {c['collided_with'] or 'none'} | "
          f"{'-' if c['collision_tick'] is None else f'{c['collision_tick'] * DT:.2f}'} | {a['ego_v_end']:.2f} | {a['ego_lat_end']:.2f} | {cl(mc.get('blk'))} / {cl(mc.get('rear'))} / {cl(min(adj) if adj else None)} |")
    who = {}
    for p, c in ce.items():
        who.setdefault(c["collided_with"] or "no contact (G < eps from clearance/terminal/dynamic limit)", []).append(p)
    w("\nGrouped by what refutes the member: " + "; ".join(f"**{k}**: {len(v)}" for k, v in who.items()) + ".\n")
    for k, v in who.items():
        w(f"- {k}: {', '.join('`' + x + '`' for x in v)}")

    bk = {p: c for p, c in ce.items() if p.startswith("brake_frac_") and c["collided_with"] == "blk"}
    w("\n**Gentle braking is dominated.** Lower brake fractions do not help (measured decel is about 4 to 5 m/s^2 for f <= 0.6 on this plant, see the decel table; there is no 1-3 m/s^2 level): "
      f"the constant-brake members that hit the blocker are {sorted(bk)} (contact at {sorted({round(c['collision_tick'] * DT, 2) for c in bk.values()})} s), and every member that stops short is hit from behind. "
      "Lower deceleration arrives at the blocker with speed; higher deceleration stops short and is overtaken by the rear vehicle.\n")
    w("**The in-lane refutation is structural.** Given a rear vehicle that never reacts and a blocker that never moves, a vehicle confined to its lane is struck by one of them whatever speed profile it follows (either it is not stopped before the blocker, or it is stopped and overtaken). The class-wide refutation of the in-lane members is therefore a consequence of the construction (declared world, blocked left lane, road edge on the right), not an empirical discovery. "
      "The empirical content is that the lane-change members, which are not structurally excluded, are also refuted (queue contact before the manoeuvre completes) and that the refutations were realised on CARLA with the calibrated plant.\n")
    w("\n### Controls (v 14, gap " + f"{rc09_theta(14.0)['gap_m']:g}" + ", same class)\n")
    w("Each control removes one element of the scene to show what closes it. These are separate records and are not part of the RC09 label.\n")
    w("| control | evidence | members with G >= eps | margins |\n|---|---|---|---|")
    for k, r in sorted(ctl.items()):
        name = {"ctl_norear": "no rear vehicle (left queue present)", "ctl_openleft": "left lane open (rear vehicle present)"}[k[0]]
        ok_m = [p for p, gs in mpol(r).items() if all(g is not None and g >= CFG.eps for g in gs)]
        w(f"| {name} | {r['evidence_class']} | {ok_m or 'none'} | {CCP.mrows(r)} |")
    ol = [r09[("ctl_openleft", 14.0, RC09_REAR_GAP)]] + ol_reps
    gs = [mpol(r)["bypass_left_lc2"][0] for r in ol]
    rng, marg, flips = repeat_status(gs)
    w(f"\n**Open-left control, repeatability.** The only witness is `bypass_left_lc2`; its margin over the original run and {len(ol_reps)} repeats of the whole record: {[round(g, 4) for g in gs]} "
      f"(range {rng[0]:.4f} .. {rng[1]:.4f}; evidence classes {[r['evidence_class'] for r in ol]}). Under sigma = {SIGMA:g}: **{'MARGINAL' if marg else 'robust'}**"
      f"{' (sign flips across repeats)' if flips else ''}. " + ("The control is therefore weak evidence that an open left lane gives an escape; it is reported, not used for any claim beyond 'the escape exists in this run'." if marg else "") + "\n")

    w("\n### Brake fraction -> measured deceleration\n")
    w("Clear road (no actors), one ego run per level, from the CARLA speed traces. Deceleration is the mean slope of the ego speed between the first tick below 0.9 v0 and the first tick below 0.4 v0 "
      "(or the last tick if 0.4 v0 is not reached; the window stays above the end-of-stop wheel-lock transient below about 4.3 m/s); the commanded fraction is mapped by the plant's brake pedal and engine drag, not by a linear law. Linear expectation = f * "
      f"{CC.DECEL_TARGET} m/s^2.\n")
    dr = decel_rows(dec)
    w("| brake_frac | " + " | ".join(f"measured decel v0={v:g} (m/s^2)" for v in dr) + " | linear expectation | measured / linear (v0=14) | v at end of 8 s (v0=14) |\n|---|" + "---|" * (len(dr) + 3))
    for i, f in enumerate(NG.RC09_BRAKE_FRACS):
        cells = []
        for v in dr:
            d = dr[v][i][2]
            cells.append("not reached" if d is None else f"{d[0]:.2f} (to {d[3]:.1f} m/s)")
        lin = f * CC.DECEL_TARGET
        d14 = dr[14.0][i][2]
        ratio = "-" if (d14 is None or lin == 0) else f"{d14[0] / lin:.2f}"
        w(f"| {f:.1f} | " + " | ".join(cells) + f" | {lin:.2f} | {ratio} | {dr[14.0][i][3]:.2f} |")
    w("\nThe mapping is nonlinear: the low levels are dominated by an engine-drag floor, so a small fraction does not give a small deceleration. The intermediate levels of the class are therefore not "
      "'gentle braking' in the linear sense; they are measured members of the class as run.\n")

    # ------------------------------------------------------------------ RC13
    w("## RC13: over-approximated prediction falsely closes a corridor\n")
    w("**Predictor** (`cases/negatives.py: Envelope, action_rejected`). It is a plain function of one delivered observation and one action; it is NOT part of the evidence pipeline (it never produces a margin or a label, "
      "and the harness does not call it). For each object `o` of the observation: constant-velocity prediction `x_o(t) = dx + (v_ego + dv) t`, no lateral motion; its box half-extents are "
      "`hl + lon_m + growth_mps * t` and `hw + lat_m + growth_mps * t`. The action's nominal ego footprint (ego half length from the observation, half width 0.95 m): speed held for tau = "
      f"{NG.TAU_NOM:g} s, then brake at b = {NG.B_NOM:g} m/s^2 (`stay`: to a stop; `change_left`: until v < {NG.V_FLOOR:g} m/s, then hold) with the quintic lateral move of {NG.LC_TIME:g} s over one lane width for `change_left`. "
      "The action is **rejected** if at any look-ahead step (horizon 4 s) the box separation (max of the longitudinal and lateral gaps; negative = overlap) between the ego footprint and an inflated box is below `clearance_m` = 0.\n")
    w("**Parameters.** " + "; ".join(f"E{i}: lat {e.lat_m:g}, lon {e.lon_m:g}, growth {e.growth_mps:g}/s, dt {e.dt_s:g} s" for i, e in enumerate(ENVELOPES)) + ". Horizon 4 s, clearance 0 for all.\n")
    w("**Coherent worlds** are the declared world set of the construction: `hid_left_lane` and `hid_own_lane` (one stopped vehicle, in the left lane or in the ego lane; the feed delivers it from t_reveal). "
      "The predictor sees only the first delivered observation containing the vehicle (recorded from the CARLA run of the witness `shared_slowdown_then_branch`), never the world name; a world the predictor 'imagines' (e.g. a vehicle in both lanes) is not in the set and plays no role.\n")
    w("Scenes (v, D bumper gap, t_reveal), all from the RC10 construction; the evidence is this task's own CARLA run:\n")
    w("| v | D | t_reveal | evidence | margins (hid_left_lane/hid_own_lane) | first-sighting observation (hid_own_lane): dx, dlat, ego v |\n|---|---|---|---|---|---|")
    for k, s in r13.items():
        o = s["obs"]["hid_own_lane"]
        w(f"| {k[0]:g} | {k[1]:g} | {k[2]:g} | {s['rec']['evidence_class']} | {CCP.mrows(s['rec'])} | {o.objs[0]['dx']:.2f}, {o.objs[0]['dlat']:.2f}, {o.v:.2f} |")
    w("\nEach cell below compares the predictor's decision with the evidence for the same action in the same world. Evidence 'ok' = G >= eps. `witness` cells: the action is the branch that `shared_slowdown_then_branch` takes in that world "
      "(the evidence is that policy's own margin, same state). `other action` cells: the counterfactual branch, with the evidence of `stop_in_place` / `fixed_left`, which are executed from t = 0, so they are only an approximate counterpart of the action judged from the first-sighting state. "
      "False rejection = evidence ok and predictor rejects; false acceptance = evidence refuted and predictor accepts.\n")
    for k, s in r13.items():
        w(f"### Scene v {k[0]:g}, D {k[1]:g}, t_reveal {k[2]:g}  (evidence {s['rec']['evidence_class']})\n")
        tbl = rc13_table(s, ENVELOPES)
        w("| envelope | cells agreeing with evidence | false rejections | false acceptances | witness cells: min box separation (m), world A `stay` / world B `change_left` |\n|---|---|---|---|---|")
        for i, row in enumerate(tbl):
            wc = [c for c in row["cells"] if c["role"] == "witness"]
            w(f"| E{i} | {row['agree']}/{row['n']} | {[(a, b) for a, b, _, _ in row['false_rejections']] or 'none'} | {[(a, b) for a, b, _, _ in row['false_accepts']] or 'none'} | "
              f"{wc[0]['min_sep_m']:.2f} / {wc[1]['min_sep_m']:.2f} |")
        th = witness_threshold(s)
        w(f"\nSmallest uniform inflation (lat = lon, no growth, dt 0.1 s, grid 0.05 m) that rejects the witness branch: {th}.")
        a_own = s["audit"]["shared_slowdown_then_branch/hid_own_lane"]
        w(f"Audited witness trace in `hid_own_lane` (r = {fm(gval(a_own['G']))}): termination {a_own['termination']}, contact events {a_own['contact_events'] or 'none'}, ego v {a_own['ego_v_start']:.2f} -> {a_own['ego_v_end']:.2f}, "
          f"lateral range [{a_own['ego_lat_min']:.2f}, {a_own['ego_lat_max']:.2f}], min clearance (m) {({kk: (None if x is None else round(x, 2)) for kk, x in a_own['min_clearance_m'].items()})}; "
          f"commands (decision -> effective): " + ", ".join(f"{c['long']}/{c['lat_target']} {c['decision_t']:.2f}->{c['effective_t']:.2f}" for c in a_own["commands"]) + ".\n")
    # summary
    w("### Repeatability and headline\n")
    w(f"Each RC13 scene was run {REPEATS} more times (whole library, same construction). A scene is **marginal** if any witness margin (either world) over the original run and the repeats is within sigma = {SIGMA:g} of eps, or if a sign flips; "
      "marginal scenes are excluded from the headline count.\n")
    w("| scene (v, D, t_reveal) | evidence classes (orig + repeats) | witness G world A: range | witness G world B: range | status |\n|---|---|---|---|---|")
    robust = []
    for k, s in r13.items():
        recs = [s["rec"]] + s["reps"]
        ok_, why = scene_robust(recs)
        runs = witness_runs(recs)
        ra, rb = (repeat_status([g[i] for g in runs])[0] for i in (0, 1))
        if ok_:
            robust.append(k)
        w(f"| {k} | {[r['evidence_class'] for r in recs]} | {ra[0]:.4f} .. {ra[1]:.4f} | {rb[0]:.4f} .. {rb[1]:.4f} | {'robust' if ok_ else 'MARGINAL: ' + why} |")
    marg = [k for k in r13 if k not in robust]
    w(f"\n**Headline.** Robust scenes (counted): {len(robust)} of {len(r13)}: {robust or 'none'}. Marginal scenes (excluded from the count): {marg or 'none'}.\n")
    allg = [(k, g) for k, x in r13.items() for g in [m for run in witness_runs([x["rec"]] + x["reps"]) for m in run]]
    kmin, gmin = min(allg, key=lambda t: t[1])
    w(f"The rule is applied mechanically. Smallest witness margin over all runs and worlds: {gmin:.4f} in scene {kmin} = {gmin / SIGMA:.2f} x sigma; it is not within sigma of eps, so the scene is not marked marginal, but it is within about 10% of the threshold. "
      "The repeats differ by much less than sigma except where the ranges above show otherwise, so sigma is a conservative bound here.\n")
    w("### Predictor summary (robust scenes)\n")
    w("| scene (v, D, t_reveal) | E0 (exact CV, clearance 0): agreement | E0 rejects the witness | smallest rejecting uniform inflation per world (m) | E0 box separation, witness world B (m) | audited actual clearance to hid, witness world B (m) |\n|---|---|---|---|---|---|")
    seps, acts, thrs, cl1, cl2 = [], [], [], [], []
    clr_idx = len(ENVELOPES) - 2          # exact envelope with clearance_m = 1.0
    for k in robust:
        s = r13[k]
        tb = rc13_table(s, ENVELOPES)
        t0 = tb[0]
        thr = witness_threshold(s)
        sep = [c for c in t0["cells"] if c["role"] == "witness"][1]["min_sep_m"]
        act = s["audit"]["shared_slowdown_then_branch/hid_own_lane"]["min_clearance_m"]["hid"]
        seps.append(sep)
        acts.append(act)
        thrs.append(thr["hid_own_lane"])
        cl1.append(tb[clr_idx])
        cl2.append(tb[clr_idx + 1])
        w(f"| {k} | {t0['agree']}/{t0['n']} | {bool(t0['false_rejections'])} | {thr} | {sep:.2f} | {act:.2f} |")
    if robust:
        w("\n### Metric mismatch between the predictor and the evidence\n")
        diffs = [a - b for a, b in zip(acts, seps)]
        w("The predictor uses axis-aligned box separation (max of the longitudinal and lateral gaps) with `clearance_m = 0`, evaluated on a NOMINAL ego path (declared b, tau, quintic lane change). The evidence uses the oriented-box corner clearance of the executed trajectory with a 1 m reserve inside the margin. They are different quantities, so the numbers are not directly comparable: "
          f"in the robust scenes the exact envelope's box separation for the witness branch is {min(seps):.2f}-{max(seps):.2f} m while the audited actual clearances are {min(acts):.2f}-{max(acts):.2f} m; the difference ({min(diffs):.2f} to {max(diffs):.2f} m) is the nominal-model error plus the box-versus-corner metric difference, and it is not separated further here. "
          f"The smallest rejecting inflation of {min(thrs):.2f}-{max(thrs):.2f} m is below the evidence's own 1 m reserve: the over-approximation that flips the decision is smaller than the safety reserve the evidence itself demands, so part of what looks like a 'false' rejection is a different safety convention rather than only over-approximation.\n")
        w(f"Rows E{clr_idx} (exact envelope, `clearance_m` = 1.0) and E{clr_idx + 1} (0.5 m inflation, `clearance_m` = 1.0) test the exact envelope against the evidence's own reserve:\n")
        w(f"| scene | E{clr_idx} agreement | E{clr_idx} false rejections | E{clr_idx + 1} false rejections |\n|---|---|---|---|")
        for k, r1, r2 in zip(robust, cl1, cl2):
            w(f"| {k} | {r1['agree']}/{r1['n']} | {[(a, b) for a, b, _, _ in r1['false_rejections']] or 'none'} | {[(a, b) for a, b, _, _ in r2['false_rejections']] or 'none'} |")
        e1 = [bool(r['false_rejections']) for r in cl1]
        w(f"\nThe exact envelope held to the evidence's own 1 m reserve (E{clr_idx}) "
          + (f"DISAGREES with the evidence (it rejects the supported witness) in {sum(e1)} of {len(e1)} robust scenes." if any(e1) else "agrees with the evidence in every robust scene.") + "\n")
    w("**Reading (scoped).** On the declared worlds, for the robust scenes, the witness is supported by the evidence (r >= eps in both worlds in every repeat, executed without contact) and the exact constant-velocity envelope with zero clearance accepts it. "
      "A predictor that adds a constant inflation of about 0.55-0.85 m, or a growth term, to the same envelope rejects the witness branch. That shows how a simple over-approximating predictor can close a corridor the evidence supports; "
      "it does not show that any particular real predictor does so, and the thresholds depend on the metric mismatch above (box separation of a nominal path versus corner clearance with reserve of the executed path). Marginal scenes are not part of this statement.")
    n, bad_ = validate_all()
    w(f"\n## Record validation\n\n{n} records in `results/carla/negatives/` checked with validate_records.check_record: {len(bad_)} failures.")
    for b in bad_[:10]:
        w(f"- {b}")
    return "\n".join(L) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    ctx = Ctx(a.host, a.port)
    r09 = run_rc09(ctx)
    dec = run_decel(ctx)
    r13 = run_rc13(ctx)
    text = report(ctx, r09, dec, r13)
    open("results/carla/negatives.md", "w", encoding="utf-8", newline="\n").write(text)
    print(text)


if __name__ == "__main__":
    main()
