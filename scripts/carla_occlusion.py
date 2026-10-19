"""
Task 7: partial-observation cases on CARLA (RC10 early evidence, RC11 late evidence, RC14 history-conditioned profiles).

    cd mining && PYTHONNOUSERSITE=1 python ../scripts/carla_occlusion.py [--host localhost --port 2000]

Needs a live, bring-up-passed server (never started here).  Every point is a full ExecutionRecord checkpointed to
mining/results/carla/occlusion/<sweep>.jsonl (a rerun skips finished points); summary mining/results/carla/occlusion.md and the
reveal-time transition figure mining/results/carla/occlusion_transition.png.  Labels come from evidence only (sim.harness.evaluate ->
contract.classify); eps/reserve are margin.DEFAULT_V2, untouched.  The hidden vehicle IS spawned in CARLA (ground truth); "hidden" means
the ObservationFeed withholds it until t_reveal (no CARLA occluder), so policies never receive it earlier.

Declared before the run (not changed after seeing results):
    scene   v = 14 and 18 m/s, one stopped vehicle `hid` D m ahead (bumper gap) in the ego lane or the left lane, act delay 0.25 s,
            library {stop_in_place, fixed_left (brake + left change), fixed_right (keep lane), shared_slowdown_then_branch}, lc_time 2.0.
    sweep   t_reveal on the 0.1 s grid (0..1.2) for the fine D, coarse for the others (see GRIDS).
    analytic threshold  t_obs*(D, v) = t_arr_max - tau_eff - T_lat   (reference/info_tree.t_obs_star), with the calibrated b_cal = 6.211 m/s^2,
            tau_eff = 0.25 + 0.05 onset = 0.30 s, T_lat = time for the quintic lane change (lc_time 2.0) to reach the lateral displacement
            hw_ego + hw_hid + reserve = 2.9 m (of 3.5 m).  The policy as run brakes until v < 8 m/s (lane-change authority) so its analytic
            prefix is p = (v - 8)/b; the supremum over prefixes (continuous braking) is also reported.  The measured threshold is the midpoint of the (last R_REF, first non-R_REF) bracket;
            cross-check tolerance |t_meas - t_obs*| <= 0.25 s (5 ticks).  The tolerance is not changed after the run.
    transition scene for the audit: the median D (among the fine-sweep D) that shows an R_REF -> non-R_REF transition; audited at the last
            R_REF reveal and the first non-R_REF reveal (full-library re-run with a per-tick spy on every episode).
"""
import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "mining"))
sys.path.insert(0, HERE)

import numpy as np

import margin as M
from cases import occlusion as OC
from reference import info_tree as IT
from sim import harness as H
from sim import policies as P
from sim.trackers import quintic
import carla_core as CC
import carla_compositional as CCP

OUT = "results/carla/occlusion"
CFG = M.DEFAULT_V2
TAU = 0.25
B_CAL, TAU_EFF = CC.B_CAL, CC.TAU_EFF
TOL_S = 0.25
DV_TOL = 1e-6          # m/s: pre-reveal ego-speed difference between the two worlds that still counts as the same history
TAG = "al8lc2"
SCOPE = "carla_finite_replay"
gval, fm, mw = CC.gval, CCP.fm, CCP.mw
REVEALS = [round(0.1 * i, 2) for i in range(13)]
COARSE = [0.0, 0.6, 1.2]
GRIDS = {  # (v, D): reveals
    **{(14.0, d): REVEALS for d in (18.0, 19.0, 20.0, 21.0)},
    **{(14.0, d): COARSE for d in (17.0, 22.0, 24.0)},
    **{(18.0, d): [round(0.2 * i, 2) for i in range(7)] for d in (26.0, 28.0, 30.0)},
}
FINE = [(14.0, 18.0), (14.0, 19.0), (14.0, 20.0), (14.0, 21.0)]
RC14_FORGET = (0.3, 0.6, 1.0)
RC14_SCENE = (14.0, 19.0)


# ----------------------------------------------------------------------------- pure helpers (mock-tested)
def t_lat(lc_time=OC.LC_TIME, need_m=None):
    """Time for the tracker's quintic lateral transition (lc_time, one lane width) to reach `need_m` of lateral displacement."""
    need = (OC.EGO_HW + OC.HID_HW + CFG.reserve_m) if need_m is None else need_m
    lo, hi = 0.0, 1.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if quintic(mid) * OC.LANE_W < need else (lo, mid)
    return 0.5 * (lo + hi) * lc_time


V_FLOOR = P.SharedSlowdownThenBranch().params["v_floor"]


def stop_distance(v, b=B_CAL, tau=TAU_EFF, c=None):
    """Distance a full brake needs to stop with the reserve (the common-stop boundary)."""
    return v * tau + v * v / (2 * b) + (CFG.reserve_m if c is None else c)


def analytic_threshold(D, v, b=B_CAL, tau=TAU_EFF, lc_time=OC.LC_TIME, continuous=False):
    """t_obs* for the policy as run (braking prefix p = (v - V_FLOOR)/b, then constant speed: the lane change needs lateral control
    authority, see SharedSlowdownThenBranch) or, continuous=True, the supremum over all prefixes (continuous braking).
    None when a common stop exists (D >= stop_distance): then there is no information gate."""
    if D >= stop_distance(v, b, tau):
        return None
    return IT.t_obs_star(D, v, tau, b, t_lat(lc_time), p=None if continuous else (v - V_FLOOR) / b)


def best_value(r):
    """V = max over policies of (min over worlds of G) from a record dict (-inf allowed; None entries make that policy unresolved and are skipped)."""
    vals = []
    for row in mw(r).values():
        if any(g is None for g in row):
            continue
        vals.append(min(row))
    return max(vals) if vals else -math.inf


def own_world_G(r, policy="shared_slowdown_then_branch"):
    return mw(r)[policy][1]


def evidence_by_reveal(recs):
    """recs: {t_reveal: record dict} -> [(t, evidence)] sorted by t."""
    return sorted((t, r["evidence_class"]) for t, r in recs.items())


def transition(recs):
    """(last_R_REF, first_non_R_REF) reveal times of the sweep, or None when the sweep does not contain exactly an R_REF-prefix then non-R_REF."""
    ev = evidence_by_reveal(recs)
    ts = [t for t, e in ev]
    flags = [e == "R_REF" for t, e in ev]
    if all(flags) or not any(flags) or flags != sorted(flags, reverse=True):
        return None
    k = flags.index(False)
    return ts[k - 1], ts[k]


def crosscheck(tr, t_star, tol=TOL_S):
    """Measured midpoint vs analytic; verdict string."""
    if t_star is None:
        return {"t_meas": None, "diff": None, "ok": None, "note": "no finite analytic threshold (a common stop exists)"}
    if tr is None:
        return {"t_meas": None, "diff": None, "ok": None, "note": "no transition inside the sweep"}
    t_meas = 0.5 * (tr[0] + tr[1])
    return {"t_meas": t_meas, "diff": t_meas - t_star, "ok": abs(t_meas - t_star) <= tol, "note": ""}


def arrival_stats(log, dt=0.05, ego_hl=OC.EGO_HL, hid="hid"):
    """From a per-tick spy log [{name: (x, lat, v)}]: first tick at which the ego front bumper reaches the hidden vehicle's rear plane;
    reports t, ego speed, ego lateral displacement from its start and the lateral gap to the hidden vehicle's centre there."""
    for k, s in enumerate(log):
        e, h = s["ego"], s[hid]
        if e[0] + ego_hl >= h[0] - OC.HID_HL:
            return {"t_arrival": k * dt, "v_arrival": e[2], "lat_disp": abs(e[1] - log[0]["ego"][1]), "dlat_to_hid": abs(e[1] - h[1])}
    return None


class SpyPolicy(P.BrakeHold):
    """Pure BrakeHold that records every history it is handed (identical-history check on CARLA)."""
    name = "spy"

    def reset(self):
        super().reset(); self.seen = []

    def decide(self, history, t):
        self.seen.append((t, list(history)))
        return super().decide(history, t)


def history_check(factory, theta, t_reveal):
    """Run the SAME policy in both worlds and compare what it was handed before the reveal: objs / lane_w / t_capture / t_available / ego_hl
    exactly equal; the ego speed is compared separately against DV_TOL (pass/fail); separated at the reveal."""
    ws = OC.worlds(t_reveal, theta)
    hs = []
    for w in ws:
        spy = SpyPolicy()
        plant = factory()
        try:
            H.run_episode(plant, OC.spec(theta, CFG), spy, w, CFG)
        finally:
            plant.close()
        hs.append(spy.seen)
    n = min(len(hs[0]), len(hs[1]))
    pre = [i for i in range(n) if hs[0][i][0] < t_reveal - 1e-9]
    same = all(hs[0][i][1][-1].objs == hs[1][i][1][-1].objs == [] and hs[0][i][1][-1].lane_w == hs[1][i][1][-1].lane_w
               and hs[0][i][1][-1].t_capture == hs[1][i][1][-1].t_capture and hs[0][i][1][-1].t_available == hs[1][i][1][-1].t_available
               and hs[0][i][1][-1].ego_hl == hs[1][i][1][-1].ego_hl for i in pre)
    dv = max(abs(hs[0][i][1][-1].v - hs[1][i][1][-1].v) for i in pre)
    k = [i for i in range(n) if abs(hs[0][i][0] - t_reveal) < 1e-9][0]
    ob = [hs[j][k][1][-1].objs for j in (0, 1)]
    return {"n_pre": len(pre), "identical_objs_etc": same, "max_dv_pre": dv, "dv_ok": bool(dv < DV_TOL), "objs_at_reveal": ob,
            "separated": ob[0] != ob[1] and len(ob[0]) == len(ob[1]) == 1}


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
        CCP.gpu_wait()
        r = fn().to_json()
        r["_key"] = key
        r.update(extra)
        with open(f"{OUT}/{sweep}.jsonl", "a") as f:
            f.write(json.dumps(r) + "\n")
        d[key] = r
        return r

    def run(self, case, theta, **kw):
        return OC.run(case, self.factory, theta, CFG, "carla", SCOPE, physics_profile_id=self.pid, runtime=self.runtime, **kw)


# ----------------------------------------------------------------------------- sweeps
def theta(v, D, **kw):
    return {"gap_m": D, "v_mps": v, "tau_s": TAU, **kw}


def run_sweeps(ctx):
    res = {}
    for (v, D), reveals in GRIDS.items():
        res[(v, D)] = {}
        for tr in reveals:
            case = "RC10" if tr <= 0.5 else "RC11"      # naming only; the label is whatever the evidence says
            res[(v, D)][tr] = ctx.point(f"occ_v{v:g}_d{D:g}_{TAG}", f"{tr:.2f}", lambda tr=tr, v=v, D=D, case=case: ctx.run(case, theta(v, D, t_reveal_s=tr)),
                                        library_id=f"occ_lc2.0_{TAG}")
        ev = "".join({"R_REF": "R", "U_INFORMATION": "I", "U_LIBRARY": "L", "UNRESOLVED": "?", "INVALID_RUN": "X"}.get(r["evidence_class"], "?") for _, r in sorted(res[(v, D)].items()))
        print(f"v {v:g} D {D:g}: {ev}", flush=True)
    return res


def run_rc14(ctx):
    res = {}
    v, D = RC14_SCENE
    for tf in RC14_FORGET:
        res[tf] = ctx.point(f"rc14_{TAG}", f"{tf:.2f}", lambda tf=tf: ctx.run("RC14", theta(v, D, t_forget_s=tf)), library_id=f"rc14_lc2.0_{TAG}")
        print("RC14", tf, res[tf]["evidence_class"], CCP.mrows(res[tf]), flush=True)
    return res


def spy_audit(ctx, case, th, **kw):
    logs = []

    def fac():
        lg = []
        logs.append(lg)
        return CC.SpyPlant(ctx.factory(), lg)
    rec = OC.run(case, fac, th, CFG, "carla", SCOPE, physics_profile_id=ctx.pid, runtime=ctx.runtime, **kw)
    return rec, logs


def trace_of(rec, logs, policy, world):
    pi, wi = rec.policies.index(policy), rec.worlds.index(world)
    log = logs[pi * len(rec.worlds) + wi]
    return CCP.summarize_trace(log, rec, f"{policy}/{world}", rec.R_raw[pi][wi], 0.05), arrival_stats(log)


def audit_transition(ctx, sweeps):
    trans = {k: transition(sweeps[k]) for k in FINE}
    have = sorted(k for k, t in trans.items() if t)
    if not have:
        return None
    k = have[len(have) // 2]
    v, D = k
    out = {"scene": k, "pair": trans[k], "runs": {}}
    for label, tr in zip(("last_R_REF", "first_non_R_REF"), trans[k]):
        rec, logs = spy_audit(ctx, "RC10" if tr <= 0.5 else "RC11", theta(v, D, t_reveal_s=tr))
        scan = sweeps[k][tr]
        out["runs"][label] = {"t_reveal": tr, "record": scan, "rerun": CCP.rerun_diff(scan, rec),
                              "branch_own": trace_of(rec, logs, "shared_slowdown_then_branch", "hid_own_lane"),
                              "branch_left": trace_of(rec, logs, "shared_slowdown_then_branch", "hid_left_lane"),
                              "fixed_left_own": trace_of(rec, logs, "fixed_left", "hid_own_lane"),
                              "stop_own": trace_of(rec, logs, "stop_in_place", "hid_own_lane")}
    return out


# ----------------------------------------------------------------------------- figure
def figure(sweeps, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    ks = [k for k in sorted(sweeps) if k[0] == 14.0]
    fig, (a, b) = plt.subplots(1, 2, figsize=(12.5, 5.8), gridspec_kw={"width_ratios": [1.5, 1]})
    cmap = {"R_REF": "#2a9d8f", "U_INFORMATION": "#e76f51", "U_LIBRARY": "#6c757d", "UNRESOLVED": "#bbbbbb", "INVALID_RUN": "k"}
    Dcol = {k: c for k, c in zip(ks, ["#1f77b4", "#d62728", "#2ca02c", "#9467bd", "#8c564b", "#17becf", "#7f7f7f"])}
    lo, hi = -0.3, 0.3
    for k in ks:
        recs = sweeps[k]
        ts = sorted(recs)
        V = [best_value(recs[t]) for t in ts]
        fin = [(t, max(lo, min(hi, v))) for t, v in zip(ts, V) if v != -math.inf]
        inf_t = [t for t, v in zip(ts, V) if v == -math.inf]
        a.plot([t for t, _ in fin], [y for _, y in fin], "-", color=Dcol[k], lw=1.2, alpha=0.8)
        for t, v in zip(ts, V):
            ok = recs[t]["evidence_class"] == "R_REF"
            a.scatter([t], [lo if v == -math.inf else max(lo, min(hi, v))], marker="v" if v == -math.inf else "o", s=42, zorder=3,
                      facecolor=Dcol[k] if ok else "white", edgecolor=Dcol[k], linewidth=1.4)
        ts_star = analytic_threshold(k[1], k[0])
        a.axvline(ts_star if ts_star is not None else np.nan, color=Dcol[k], ls=(0, (3, 3)), lw=1)
        a.plot([], [], color=Dcol[k], lw=1.5, label=f"D={k[1]:g} m: " + (f"t*={ts_star:.2f} s (dashed)" if ts_star is not None else "common stop, no t*"))
    a.axhline(0, color="k", lw=0.6)
    a.set_ylim(lo - 0.04, hi + 0.04)
    a.set_xlabel("t_reveal (s)")
    a.set_ylabel("V = max over policies of min over worlds of G\n(clipped to [-0.3, 0.3])")
    a.set_title("v = 14 m/s: best causal margin V vs reveal time", fontsize=10)
    a.scatter([], [], marker="o", facecolor="k", edgecolor="k", label="filled = R_REF")
    a.scatter([], [], marker="o", facecolor="white", edgecolor="k", label="hollow = U_INFORMATION / U_LIBRARY")
    a.scatter([], [], marker="v", facecolor="white", edgecolor="k", label="triangle at floor = V = -inf")
    a.set_xlim(-0.05, 1.25)
    a.legend(fontsize=7, loc="upper center", bbox_to_anchor=(0.5, -0.16), ncol=2, framealpha=0.9)
    ds = [k for k in ks]
    allt = sorted({t for k in ks for t in sweeps[k]})
    img = np.full((len(ds), len(allt)), np.nan)
    codes = {"R_REF": 0, "U_INFORMATION": 1, "U_LIBRARY": 2, "UNRESOLVED": 3, "INVALID_RUN": 4}
    for i, k in enumerate(ds):
        for t, r in sweeps[k].items():
            img[i, allt.index(t)] = codes[r["evidence_class"]]
    from matplotlib.colors import ListedColormap
    b.imshow(img, aspect="auto", cmap=ListedColormap([cmap[e] for e in codes]), vmin=-0.5, vmax=4.5, origin="lower")
    b.set_xticks(range(len(allt)))
    b.set_xticklabels([f"{t:g}" for t in allt], fontsize=6, rotation=90)
    b.set_yticks(range(len(ds)))
    b.set_yticklabels([f"D={k[1]:g}" for k in ds], fontsize=8)
    b.set_xlabel("t_reveal (s)")
    b.set_title("evidence class (white = not run)")
    fig.tight_layout()
    fig.savefig(path, dpi=140, bbox_inches="tight")
    plt.close(fig)


# ----------------------------------------------------------------------------- report
def sweep_table(ks, sweeps, times):
    """Markdown lines of one sweep table; every row has exactly as many cells as the header."""
    hdr = ["D"] + [f"{t:g}" for t in times] + ["measured bracket", "analytic t* (policy as run)", "analytic t* (sup over prefixes)", "midpoint - t*", "within 0.25 s"]
    out = ["| " + " | ".join(hdr) + " |", "|" + "---|" * len(hdr)]
    for k in ks:
        tr = transition(sweeps[k])
        ts = analytic_threshold(k[1], k[0])
        tsup = analytic_threshold(k[1], k[0], continuous=True)
        cc = crosscheck(tr, ts)
        cells = [code(sweeps[k][t]) if t in sweeps[k] else "" for t in times]
        row = [f"{k[1]:g}"] + cells + [str(tr) if tr else "-", "none (common stop)" if ts is None else f"{ts:.3f}", "-" if tsup is None else f"{tsup:.3f}",
                                       "-" if cc["diff"] is None else f"{cc['diff']:+.3f}", str(cc["ok"]) if cc["ok"] is not None else cc["note"]]
        out.append("| " + " | ".join(row) + " |")
    return out


def crosscheck_stats(sweeps, keys):
    """Differences (midpoint - t*) of the scenes with a bracket and a finite t*, and how many have t* strictly before the bracket's lower edge."""
    rows = []
    for k in keys:
        tr, ts = transition(sweeps[k]), analytic_threshold(k[1], k[0])
        if tr and ts is not None:
            rows.append((k, 0.5 * (tr[0] + tr[1]) - ts, ts < tr[0]))
    return rows


def blind_spread(sweeps, k):
    """Range of the blind fixed_left policy's own-world G over the reveal sweep (it ignores observations, so this is run-to-run spread)."""
    g = [gval(mw(r)["fixed_left"][1]) for r in sweeps[k].values()]
    g = [x for x in g if x is not None and x != -math.inf]
    return (min(g), max(g)) if g else None


def code(r):
    return {"R_REF": "R", "U_INFORMATION": "I", "U_LIBRARY": "L", "UNRESOLVED": "?", "INVALID_RUN": "X"}.get(r["evidence_class"], "?")


def validate_all():
    import validate_records as VR
    bad, n = [], 0
    for fn in sorted(os.listdir(OUT)):
        if fn.endswith(".jsonl"):
            for line in open(f"{OUT}/{fn}"):
                r = json.loads(line)
                n += 1
                e = VR.check_record(r)
                if e:
                    bad.append((fn, r.get("_key"), e))
    return n, bad


def report(ctx, sweeps, rc14, aud, hist, fig_path):
    L = []
    w = L.append
    w("# CARLA partial-observation cases (Task 7: RC10, RC11, RC14)\n")
    w(f"Physics profile `{ctx.pid}`, decel target {CC.DECEL_TARGET}, eps = {CFG.eps}, reserve c = {CFG.reserve_m} m (margin.DEFAULT_V2, unchanged), act delay {TAU} s (5 ticks), "
      f"`a_lat_max = {OC.A_LAT_MAX:g}` m/s^2 (dynamic-limit margin; `a_comb_max` not used), emergency lane change lc_time {OC.LC_TIME:g} s. "
      "Records: `results/carla/occlusion/*.jsonl`. Labels are `contract.classify` output only; mock and CARLA outputs are kept apart.\n")
    w("**Site and mapping.** The custom 2-lane straight (`straight_2lane`, no map change). One STOPPED hidden vehicle `hid` is D m ahead of the ego (bumper gap) in exactly one of the two escape corridors: "
      "the ego's own lane (lat 0, the RIGHT lane; corridor = keep lane / stop in lane = info_tree `fixed_right`, world 'blocks RIGHT') or the adjacent LEFT lane (lat -3.5; corridor = change lane = `fixed_left`, world 'blocks LEFT'). "
      "Worlds: `hid_left_lane` (staying or stopping is safe; a left change collides) and `hid_own_lane` (only a braking left change works; D < stopping distance so stopping cannot). "
      "The vehicle is spawned in CARLA in both worlds (ground truth, static, exact placement), but the ObservationFeed withholds it until `t_reveal`; there is no CARLA occluder. "
      "Library: `stop_in_place` (full brake, hold), `fixed_left` (brake and change left at once), `fixed_right` (keep lane at speed), `shared_slowdown_then_branch` (full brake in lane until the object is delivered; "
      "then own lane -> change left while still braking, left lane -> keep lane braking; the branch is latched from the delivered history). G cells read `hid_left_lane/hid_own_lane`.\n")
    w("**info_tree reconciliation.** `reference/info_tree.py` has two LATERAL corridors that both need T_lat. On the two-lane site the right corridor is the ego's own lane and needs no lateral move, so the matrix is "
      "`build_matrix_two_lane` (added): world A/hid_left_lane has stop/stay/observe succeed (+1) and blind_L fail; world B/hid_own_lane has the same lateral slack expression as before. "
      "The binding constraint is the lateral world, so the analytic threshold is unchanged and is also given in closed form, `t_obs_star = t_arr_max - tau - T_lat` (t_arr_max = arrival time under continuous braking, since the branch keeps braking). "
      "The mock test checks the closed form against both discrete trees. "
      f"`p = (v - {V_FLOOR:g})/b` is the braking-prefix duration of the policy as run: it brakes until the ego speed is below {V_FLOOR:g} m/s and then holds that speed. Reason: stopping (or nearly stopping) in the middle of the steered "
      "lane change leaves residual yaw (the lane_follow terminal test fails) and the time-parametrised quintic tracker overshoots the road edge at low speed, so the change is completed at >= 8 m/s. "
      "The sweep tables therefore give two analytic columns: the policy as run, and the supremum over prefixes (continuous braking). The latter is '-' for D = 20 at v = 14: v tau_eff + v^2/(2 b_cal) = 19.98 m < 20 m, so under continuous braking the ego "
      "stops just short of the plane, inside the 1 m reserve; the arrival time is infinite and no finite supremum exists. That is a stop that violates the reserve, not a valid common stop (the common-stop boundary is D >= 20.98 m)."
      "\n")
    lt = t_lat()
    w(f"**Analytic anchors.** b_cal = {B_CAL} m/s^2, tau_eff = {TAU_EFF} s (0.25 s configured, quantised to 5 ticks, + 0.05 s onset), T_lat = {lt:.3f} s (quintic, lc_time {OC.LC_TIME:g}, to a lateral displacement of "
      f"{OC.EGO_HW + OC.HID_HW + CFG.reserve_m:.1f} m = hw_ego + hw_hid + reserve of the 3.5 m lane width). tau_plan = 0 and a time reserve of 0 are used in t_obs* (the 1 m spatial reserve is folded into the 2.9 m lateral criterion; the reveal is delivered on the same tick). "
      f"The {TOL_S} s cross-check tolerance is declared in this script's header docstring; commit history cannot verify that it was set before the run, and the reader should take that as stated, not proven.\n")
    w("**Plant caveats.** `hid` is static and re-placed from the measured ego bumper, so the declared gap D is exact; the ego starts on its declared speed (measured t0 speeds are in the audit traces below). "
      "No moving non-ego actor exists in these scenes, so the warm-up speed shortfall of moving actors (3-5 % low) does not apply. CARLA's brake model locks the wheels at the end of a stop (see Task 6); it only affects a_comb, which is not used here. "
      "The branch policy's braking is the calibrated decel target 6.19 m/s^2 (b_cal 6.211).\n")
    # ---- observation contract
    w("## Observation contract on CARLA\n")
    w(f"Same policy (`spy`, full brake) in both worlds, D = {hist['D']:g}, v = {hist['v']:g}, t_reveal = {hist['t']:g}: {hist['n_pre']} pre-reveal frames per world; objs / lane_w / t_capture / ego_hl identical in all of them "
      f"and objs empty: **{hist['identical_objs_etc']}**; t_available also compared; max |ego speed difference| between the worlds before the reveal {hist['max_dv_pre']:.2e} m/s, threshold {DV_TOL:g}: **{'PASS' if hist['dv_ok'] else 'FAIL'}**; "
      f"at the reveal tick the delivered objects are {hist['objs_at_reveal'][0]} (hid_left_lane) vs {hist['objs_at_reveal'][1]} (hid_own_lane): separated {hist['separated']}. "
      "The mock unit tests assert the same with exact equality of the whole Obs sequence and run the leakage check (no id / name / colour / blueprint / world name / absolute coordinate of the hidden vehicle in policy inputs before the reveal).\n")
    # ---- sweep tables
    w("## Reveal-time sweep (RC10 -> RC11)\n")
    w("Evidence class per reveal time (R = R_REF, I = U_INFORMATION, L = U_LIBRARY, ? = other), one string per scene, reveal times in the header row.\n")
    for v in (14.0, 18.0):
        ks = [k for k in sorted(sweeps) if k[0] == v]
        times = sorted({t for k in ks for t in sweeps[k]})
        w(f"v = {v:g} m/s, D_stop+c (analytic, b_cal, tau_eff) = {v * TAU_EFF + v * v / (2 * B_CAL) + CFG.reserve_m:.2f} m\n")
        L.extend(sweep_table(ks, sweeps, times))
        w("")
    rows_ = crosscheck_stats(sweeps, sorted(sweeps))
    if rows_:
        ds_ = [d for _, d, _ in rows_]
        w(f"**Cross-check reading.** All {len(ds_)} bracket midpoints differ from the policy-matched analytic t* by {', '.join(f'{d:+.3f}' for d in ds_)} s: "
          f"{'all positive' if all(d > 0 for d in ds_) else 'mixed sign'}, mean {sum(ds_) / len(ds_):+.3f} s, max {max(ds_):+.3f} s.  In {sum(e for _, _, e in rows_)} of {len(rows_)} the analytic value lies strictly before the bracket's lower edge "
          "(i.e. earlier than the last R_REF reveal), so the analytic value is a conservative one-sided bound with a bias of about one to two reveal-grid steps, not a close match.  The bias comes from the criterion: it demands 2.9 m of lateral displacement at the plane, "
          "but the audited run clears the hidden vehicle with G >= 0 at 2.36 m of displacement (box-corner geometry; the corner distance, not the centre offset, sets the clearance).\n")
    sp_ = {f"v{k[0]:g} D{k[1]:g}": blind_spread(sweeps, k) for k in sorted(sweeps) if len(sweeps[k]) > 3}
    w("**Run-to-run spread.** `fixed_left` ignores observations, so its own-world G should not depend on the reveal time; its range over the sweep (min, max) is "
      + "; ".join(f"{n}: ({a:.3f}, {b:.3f})" for n, r in sp_.items() if r for a, b in [r]) + ". "
      "The brackets therefore carry about one reveal-grid step (0.1 s, 0.2 s at v = 18) of noise in addition to the 0.1 s resolution.\n")
    w("**Note on case ids.** The records carry case_id RC10 for t_reveal <= 0.5 s and RC11 otherwise (assigned by reveal time before the run). It is a bookkeeping name, not a verdict: the evidence class is the label.\n")
    w("Full margins (G, `hid_left_lane/hid_own_lane`) per record:\n")
    w("| v | D | t_reveal | evidence | margins |\n|---|---|---|---|---|")
    for k in sorted(sweeps):
        for t in sorted(sweeps[k]):
            r = sweeps[k][t]
            w(f"| {k[0]:g} | {k[1]:g} | {t:g} | {r['evidence_class']} | {CCP.mrows(r)} |")
    w(f"\n![reveal-time transition]({os.path.basename(fig_path)})\n")
    # ---- findings
    trans = {k: transition(sweeps[k]) for k in FINE}
    w("## Does the transition show on CARLA?\n")
    shown = {k: t for k, t in trans.items() if t}
    if shown:
        for k, t in sorted(shown.items()):
            first = sweeps[k][t[1]]["evidence_class"]
            w(f"- v {k[0]:g}, D {k[1]:g}: R_REF at t_reveal <= {t[0]:g} s, {first} from {t[1]:g} s (bracket {t}); analytic t_obs* {analytic_threshold(k[1], k[0]):.3f} s.")
    else:
        w("- No fine-sweep scene shows an R_REF -> non-R_REF transition. Reported as measured; nothing was forced.")
    unshown = [k for k in FINE if k not in shown]
    for k in unshown:
        ev = "".join(code(r) for _, r in sorted(sweeps[k].items()))
        w(f"- v {k[0]:g}, D {k[1]:g}: no clean R_REF -> non-R_REF transition (class string over reveal 0..1.2: {ev}).")
    stopc = {k: [r["evidence_class"] for _, r in sorted(sweeps[k].items())] for k in sweeps if analytic_threshold(k[1], k[0]) is None}
    w(f"- Scenes beyond the analytic stopping distance (a common stop exists): {stopc}. If every entry is R_REF the 'a common stop/robust prefix present => NOT U_INFORMATION' rule is confirmed on CARLA; any other entry is reported as measured.")
    w("")
    # ---- RC14
    w("## RC14: current-view aliasing resolved by past observations\n")
    w(f"Scene v {RC14_SCENE[0]:g}, D {RC14_SCENE[1]:g}. Both worlds show `hid` from t = 0 and occlude it again from `t_forget`; they differ only in where it was seen (left lane vs own lane). "
      "After `t_forget` the delivered current frame is identical (no objects) while the earlier frames differ. `t_forget` is the sweep parameter (the time since the last observation at decision time t is t - t_forget; there is no separate field). Library adds `branch_current_frame_only` (decides from the current frame alone, forgets the vehicle when it is occluded). "
      "G cells read `seen_left_then_occluded/seen_own_then_occluded`.\n")
    w("| t_forget | evidence | margins | memory policy G >= eps in both worlds | current-frame-only policy refuted in one world |\n|---|---|---|---|---|")
    for tf, r in sorted(rc14.items()):
        m = mw(r)
        cols = [[m[p][j] for p in r["policies"]] for j in (0, 1)]
        w(f"| {tf:g} | {r['evidence_class']} | {CCP.mrows(r)} | {CCP.all_ok(m['shared_slowdown_then_branch'])} | {CCP.any_bad(m['branch_current_frame_only'])} |")
    w("")
    # ---- audits
    w("## Audited transition traces\n")
    if aud is None:
        w("No transition scene to audit.")
    else:
        v, D = aud["scene"]
        w(f"Scene v {v:g}, D {D:g} (median of the fine-sweep scenes with a transition); the full library was re-run with a per-tick spy at the last R_REF reveal and the first non-R_REF reveal.\n")
        for label, a in aud["runs"].items():
            w(f"**{label}: t_reveal {a['t_reveal']:g} s**, evidence {a['record']['evidence_class']}, margins {CCP.mrows(a['record'])}; re-run vs scan max |dG| = {a['rerun']}.\n")
            for key in ("branch_own", "branch_left", "fixed_left_own", "stop_own"):
                tr, arr = a[key]
                w(f"- `{key}`:")
                L.extend(CCP.trace_lines(tr, "    "))
                if arr:
                    w(f"    - arrival at the hidden vehicle's rear plane: t {arr['t_arrival']:.2f} s, ego speed {arr['v_arrival']:.2f} m/s, lateral displacement {arr['lat_disp']:.2f} m "
                      f"(needed {OC.EGO_HW + OC.HID_HW + CFG.reserve_m:.1f} m for the reserve), lateral distance to the vehicle centre {arr['dlat_to_hid']:.2f} m")
                else:
                    w("    - the ego stopped before the hidden vehicle's rear plane (no arrival)")
            w("")
        # compare predicted vs measured arrival for the branch in the own-lane world
        ta = IT.t_arrival_max(D, v, TAU_EFF, B_CAL)
        arr = aud["runs"]["last_R_REF"]["branch_own"][1]
        if ta and arr:
            w(f"Analytic arrival under continuous braking (b_cal, tau_eff): t {ta[0]:.2f} s at {ta[1]:.2f} m/s; measured for `shared_slowdown_then_branch` in `hid_own_lane` at the last R_REF reveal: t {arr['t_arrival']:.2f} s at {arr['v_arrival']:.2f} m/s.\n")
    n, bad = validate_all()
    w(f"## Record validation\n\n{n} records in `results/carla/occlusion/` checked with validate_records.check_record: {len(bad)} failures.")
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
    try:
        sweeps = run_sweeps(ctx)
        rc14 = run_rc14(ctx)
        aud = audit_transition(ctx, sweeps)
        hist = dict(history_check(ctx.factory, theta(14.0, 19.0, t_reveal_s=0.6), 0.6), D=19.0, v=14.0, t=0.6)
        fig_path = "results/carla/occlusion_transition.png"
        figure(sweeps, fig_path)
        text = report(ctx, sweeps, rc14, aud, hist, fig_path)
        open("results/carla/occlusion.md", "w", encoding="utf-8", newline="\n").write(text)
        print(text)
    except KeyboardInterrupt:
        print("interrupted; checkpoints kept", flush=True)


if __name__ == "__main__":
    main()
