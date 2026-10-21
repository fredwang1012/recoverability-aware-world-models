"""
Task V: CARLA camera videos of the 20 recoverability cases (RC01-RC20).  Visualisation only: no label, record or report is changed.

    cd mining && PYTHONNOUSERSITE=1 python ../scripts/carla_videos.py [--only RC03,RC05] [--force] [--host localhost --port 2000]

Needs a live, bring-up-passed server (never started here).  Each case's headline scene is RE-RUN through the group scripts' own
record-building functions with the plant factory replaced by sim.video.RecordingPlant (jsonl caches are only READ afterwards, for
the stored-G sign check; nothing is appended to them).  Clips: results/carla/videos/<stem>.mp4 (local, near-lossless; scripts/publish_videos.py makes the small committed copies in docs/videos/), thumbnails
results/carla/videos/thumbs/<stem>.png, per-clip sidecars results/carla/videos/meta/<stem>.json, summary results/carla/videos/index.md.
A clip whose mp4 already exists is skipped unless --force.
"""
import argparse
import json
import math
import os
import re
import subprocess
import sys
import textwrap
import time
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "mining"))
sys.path.insert(0, HERE)

import margin as M
from cases import compositional as CP
from cases import corridor as CO
from cases import two_lane as TL
import carla_core as CC
import carla_compositional as CCP
import carla_foundation as CF
import carla_negatives as CN
import carla_occlusion as COC

VID = "results/carla/videos"
CFG = M.DEFAULT_V2
TAU = CC.TAU
SCOPE = "carla_finite_replay"
CAT = os.path.join(HERE, "..", "mining", "cases", "catalogue.yaml")


# ----------------------------------------------------------------------------- pure helpers (no CARLA; unit-tested)
def titles(path=CAT):
    """{RCxx: title} from the catalogue (line scan: no yaml dependency)."""
    out, cur = {}, None
    for line in open(path, encoding="utf-8"):
        m = re.match(r"- id: (RC\d+)", line)
        if m:
            cur = m.group(1)
        m = re.match(r"  title: (.*)", line)
        if m and cur:
            out[cur] = m.group(1).strip()
    return out


def stem(case, policy, world=None, tag=None):
    """RCxx_<policy>[_<world>][_<tag>]; characters unsafe in a file name ('@') become '-'."""
    s = f"{case}_{policy}" + (f"_{world}" if world else "") + (f"_{tag}" if tag else "")
    return re.sub(r"[^A-Za-z0-9_.+-]", "-", s)


def gnum(x):
    """Decode a stored/raw G: '-inf' -> -inf, None -> None."""
    return -math.inf if x == "-inf" else (None if x is None else float(x))


def stored_G(rec, policy, world):
    """G of (policy, world) in a stored jsonl record dict (None when not found / unrun)."""
    if policy not in rec["policies"]:
        return None
    pi = rec["policies"].index(policy)
    wi = rec["worlds"].index(world) if world in rec["worlds"] else 0
    return gnum(rec["R_raw"][pi][wi])


def safe(g, eps=CFG.eps):
    return g is not None and math.isfinite(g) and g >= eps


def sign_check(g_new, g_old, eps=CFG.eps):
    """'agree' when both are on the same side of eps (-inf and a collision count as below), else 'DIFFERS'; 'n/a' without a stored G."""
    if g_old is None:
        return "n/a"
    return "agree" if safe(g_new, eps) == safe(g_old, eps) else "DIFFERS"


def fmt_g(g):
    return "n/a" if g is None else ("-inf" if g == -math.inf else f"{g:+.3f}")


def note_text(sentence, rec, extra=""):
    body = textwrap.fill(sentence + (" " + extra if extra else ""), width=88)
    return body + f"\nStored evidence class: {rec['evidence_class']}"


class Run:
    """One re-run of one scene.  clips = [(policy_name, world_name or None, note sentence)]; world None = the only world."""
    def __init__(self, case, scene, stored, clips, go, tag=None, extra_note=""):
        self.case, self.scene, self.stored, self.clips, self.go, self.tag, self.extra = case, scene, stored, clips, go, tag, extra_note

    def stem(self, policy, world):
        return stem(self.case, policy, world if self._multi(world) else None, self.tag)

    def _multi(self, world):
        return world is not None


def load_stored(path, key):
    for line in open(os.path.join("results/carla", path), encoding="utf-8"):
        r = json.loads(line)
        if r["_key"] == key:
            return r
    raise KeyError((path, key))


# ----------------------------------------------------------------------------- environment (one client, three Ctx views)
class Env:
    def __init__(self, host, port):
        self.core = CC.Ctx(host, port)
        self.comp = object.__new__(CCP.Ctx)
        self.occ = object.__new__(COC.Ctx)

    def share(self):
        self.comp.__dict__.update(self.core.__dict__)
        self.occ.__dict__.update(self.core.__dict__)


def cleanup(world):
    """Destroy leftover vehicles / sensors (a killed run blocks spawns); the actor list only refreshes after a tick."""
    world.tick()
    for a in list(world.get_actors().filter("sensor.*")):
        try:
            a.stop()
        except Exception:
            pass
        a.destroy()
    for a in list(world.get_actors().filter("vehicle.*")):
        a.destroy()
    world.tick()


def gpu_temp():
    try:
        return int(subprocess.check_output(["nvidia-smi", "--query-gpu=temperature.gpu", "--format=csv,noheader"], text=True).strip())
    except Exception:
        return None


# ----------------------------------------------------------------------------- the 20 cases
def th(gap, v, **kw):
    return {"gap_m": float(gap), "v_mps": float(v), "tau_s": TAU, **kw}


def build_runs():
    R = []
    # ---- RC01 / RC02 (foundation corridor)
    R.append(Run("RC01", "gap 16   v 12   tau 0.25", ("foundation/rc01_gap_brake_only.jsonl", "16.0000"),
                 [("brake_hold", None, "Watch the ego brake and stop short of the blocker: the 16 m gap is just enough with the 1 m reserve.")],
                 lambda e: CF.corr_rec(e.core, "RC01", th(16, 12), False)))
    R.append(Run("RC02", "gap 15.75   v 12   full library", ("foundation/rc02_gap_full_library.jsonl", "15.7500"),
                 [("brake_hold", None, "Watch the ego brake: it stops, but inside the 1 m reserve, so braking is too late."),
                  ("bypass_left", None, "Watch the swerve: the corridor is blocked in both lanes, so the escape fails.")],
                 lambda e: CF.corr_rec(e.core, "RC02", th(15.75, 12), True)))
    # ---- RC03 / RC20 (two-lane core)
    s03 = "v 18   gap 30   lc 2.0"
    R.append(Run("RC03", s03, ("core/rc03_v18_lc2_al8hl.jsonl", "30.000"),
                 [("bypass_left", None, "Watch the lane change around the blocker: braking cannot stop in time, the bypass just clears it."),
                  ("brake_hold", None, "Watch braking alone: the ego cannot stop before the blocker.")],
                 lambda e: CC.rc03_rec(e.core, CC.rc03_th(18.0, 30.0), 2.0)))
    # ---- RC04 (no positive witness)
    R.append(Run("RC04", "v 22   gap 42   lc 2.0", ("compositional/rc04x_v22_lc2_al8c8v5m2.jsonl", "42"),
                 [("staged_brake_0.2_change", None, "Watch a short brake then a lane change; no positive witness found on CARLA (the immediate change also passes)."),
                  ("immediate_lane_change", None, "Watch the immediate lane change; no positive witness found on CARLA for the staged contrast.")],
                 lambda e: e.comp.run("RC04", th(42, 22), library=CP.rc04_library(2.0, CCP.RC04_PS_EXT))))
    # ---- RC05
    th5 = th(80, 14, rear_gap_m=35.0, rear_v_mps=20.0, t_reveal_s=1.25)
    R.append(Run("RC05", "gap 80   rear gap 35   rear v 20", ("compositional/rc05_al8c8v5m2.jsonl", "80_35_20"),
                 [("abort_and_stabilize", "gap_closes", "Watch the ego start a lane change, see the rear car close in, steer back and settle."),
                  ("commit_lane_change", "gap_closes", "Watch the ego commit to the lane change into the closing rear car: it is hit.")],
                 lambda e: e.comp.run("RC05", th5)))
    # ---- RC06
    R.append(Run("RC06", "gap 60   pass gap 20   pass v 16", ("compositional/rc06_al8c8v5m2.jsonl", "60_20_16"),
                 [("wait_observe_enter", None, "Watch the ego keep speed, wait for the passing car, then enter the gap."),
                  ("immediate_entry", None, "Watch the ego enter the left lane at once, in front of the passing car.")],
                 lambda e: e.comp.run("RC06", th(60, 10, pass_gap_m=20.0, pass_v_mps=16.0))))
    # ---- RC07
    R.append(Run("RC07", "gap 34   rear gap 100   rear v 18", ("core/rc07_sel_al8_fixplant.jsonl", "34_100_18"),
                 [("bypass_then_return", None, "Watch the bypass, then the return to the right lane once the rear car has passed."),
                  ("bypass_then_stop", None, "Watch the bypass, then a stop in the left lane where the rear car is coming: it is hit.")],
                 lambda e: CC.rc07_rec(e.core, CC.rc07_theta(34, 100, 18))))
    # ---- RC08
    def route_note():
        r8 = load_stored("compositional/rc08_al8c8v5m2.jsonl", "closed1")
        return f"route_goal_completed = {r8['traces']['brake_hold/static_blocker'].get('route_goal_completed')} (stored)."
    R.append(Run("RC08", "closed lane   gap 45   v 14", ("compositional/rc08_al8c8v5m2.jsonl", "closed1"),
                 [("brake_hold", None, "Watch the ego stop safely in front of the blocker; the route goal is not reached.")],
                 lambda e: e.comp.run("RC08", th(45, 14, closed=True)), extra_note=route_note))
    # ---- RC09
    keep = {"brake_frac_1.0", "bypass_left_lc2", "keep_speed"}
    lib9 = [mk for mk in CN.NG.rc09_library() if mk().name in keep]
    R.append(Run("RC09", "v 14   gap 23   rear gap 25", ("negatives/rc09_al8.jsonl", "v14_rg25"),
                 [("brake_frac_1.0", None, "Watch full braking: the fast rear car hits the ego from behind."),
                  ("bypass_left_lc2", None, "Watch the lane change into the left-lane queue: no escape."),
                  ("keep_speed", None, "Watch the ego keep speed into the blocker: it is hit.")],
                 lambda e: CN.NG.rc09_run(e.core.factory, CN.rc09_theta(14.0), CFG, "carla", SCOPE, library=lib9,
                                          physics_profile_id=e.core.pid, runtime=e.core.runtime),
                 extra_note=""))
    # ---- RC10 / RC11
    R.append(Run("RC10", "v 14   D 19   t_reveal 0.1", ("occlusion/occ_v14_d19_al8lc2.jsonl", "0.10"),
                 [("shared_slowdown_then_branch", "hid_own_lane", "Watch the shared slowdown, then the lane change once the early sighting shows the hidden car in the ego lane.")],
                 lambda e: e.occ.run("RC10", COC.theta(14.0, 19.0, t_reveal_s=0.1))))
    R.append(Run("RC11", "v 14   D 19   t_reveal 0.6", ("occlusion/occ_v14_d19_al8lc2.jsonl", "0.60"),
                 [("shared_slowdown_then_branch", "hid_own_lane", "Watch the same policy with a late sighting: the lane change comes too late to clear the hidden car.")],
                 lambda e: e.occ.run("RC11", COC.theta(14.0, 19.0, t_reveal_s=0.6))))
    # ---- RC12
    for p, tag, sent in ((0.0, "p0", "Watch brake_hold start at once: it stops with a wide margin."),
                         (2.25, "p2.25", "Watch the same braking start after a 2.25 s delay: it is too late and the ego hits the blocker.")):
        pol = "brake_hold" if p == 0 else "brake_hold@prefix"
        R.append(Run("RC12", f"gap 40   v 12   prefix {p:g} s", ("core/rc12_prefix.jsonl", f"{p:.3f}"), [(pol, None, sent)],
                     lambda e, p=p: CC.rc12_rec(e.core, p), tag=tag))
    # ---- RC13
    R.append(Run("RC13", "v 14   D 20   t_reveal 0.2", ("negatives/rc13_al8.jsonl", "v14_D20_tr0.2"),
                 [("shared_slowdown_then_branch", "hid_own_lane", "Watch the witness clear the hidden car safely; the over-approximating predictor wrongly rejects this lane change.")],
                 lambda e: e.occ.run("RC10", COC.theta(14.0, 20.0, t_reveal_s=0.2))))
    # ---- RC14
    th14 = COC.theta(14.0, 19.0, t_forget_s=0.6)
    R.append(Run("RC14", "v 14   D 19   t_forget 0.6", ("occlusion/rc14_al8lc2.jsonl", "0.60"),
                 [("shared_slowdown_then_branch", "seen_own_then_occluded", "Watch the memory policy remember the hidden car after it is occluded and change lanes."),
                  ("branch_current_frame_only", "seen_own_then_occluded", "Watch the current-frame-only policy forget the occluded car and stay in lane: it is hit.")],
                 lambda e: e.occ.run("RC14", th14)))
    # ---- RC15
    R.append(Run("RC15", "gap 60   adj gap 35   adj v 16", ("compositional/rc15_al8c8v5m2.jsonl", "60_35_16"),
                 [("observe_yield_else_fallback", "adj_nonyield", "Watch the ego notice the neighbour does not yield and fall back to braking."),
                  ("always_merge", "adj_nonyield", "Watch the ego merge into a neighbour that does not yield: it is hit.")],
                 lambda e: e.comp.run("RC15", th(60, 10, adj_gap_m=35.0, adj_v_mps=16.0))))
    # ---- RC16
    def rc16(e):
        t = {"v_mps": 12.0, "tau_s": TAU, "offset_m": 1.6}
        sp = TL.rc16_spec(t, CFG)
        sp["actors"]["post"]["frozen"] = True
        sp["actors"]["post"]["bp"] = CF.POST_BP
        return TL.run("RC16", e.core.factory, t, CFG, "carla", SCOPE, spec=sp, physics_profile_id=e.core.pid, runtime=e.core.runtime)
    R.append(Run("RC16", "v 12   post offset 1.6 m", ("foundation/rc16_offset.jsonl", "1.6000"),
                 [("nudge_left", None, "Watch the small nudge clear the thin post: the body passes the post."),
                  ("lane_follow", None, "Watch the ego follow the centre line: the centre clears the post but the body does not.")],
                 rc16))
    # ---- RC17
    for tau, tag, key, sent in ((0.40, "tau0.40", "0.4125", "Watch braking with a 0.40 s delay: the ego stops short of the blocker."),
                                (0.45, "tau0.45", "0.4500", "Watch braking with a 0.45 s delay: one tick later and the ego cannot stop in time.")):
        R.append(Run("RC17", f"gap 18   v 12   delay {tau:.2f} s", ("foundation/rc17_delay.jsonl", key), [("brake_hold", None, sent)],
                     lambda e, tau=tau: CF.corr_rec(e.core, "RC17", {"gap_m": 18.0, "v_mps": 12.0, "tau_s": tau}, True), tag=tag,
                     extra_note="Stored G is from the 0.4125 s run (the same 8-tick delay)." if tau == 0.40 else ""))
    # ---- RC18
    th18 = th(42, 22)
    R.append(Run("RC18", "v 22   gap 42   a_lat + a_comb limits", ("compositional/rc18_al8c8v5m2fw.jsonl", "22_42_1"),
                 [("bypass_left", None, "Watch the keep-speed lane change, which stays inside both acceleration limits."),
                  ("brake_steer_cmd_lc2_b1", None, "Watch the full-brake-and-steer command: it passes the lateral limit but not the combined one.")],
                 lambda e: e.comp.run("RC18", th18, library=CP.rc18_library(CCP.RC18_LCS, 2.0), spec=CP.rc18_spec(th18, CFG, comb=True))))
    # ---- RC19
    for nuis, tag in ((False, "anchor"), (True, "nuisance")):
        t = {"v_mps": 8.0}
        R.append(Run("RC19", f"v 8   {'nuisance twin' if nuis else 'benign anchor'}", ("core/rc19.jsonl", f"{tag}_8"),
                     [("lane_follow", None, "Watch the ego keep its lane past a blocker in the other lane " +
                       ("(nuisance twin): same outcome as the anchor." if nuis else "(benign anchor): it passes safely."))],
                     (lambda e, t=t: e.core.tl("RC19", t, spec=TL.rc19_spec(t, CFG, nuisance=True))) if nuis else (lambda e, t=t: e.core.tl("RC19", t)),
                     tag=tag))
    # ---- RC20 (RC03 scene, v 18)
    t20 = CC.rc03_th(18.0, 30.0)
    sp20 = CC.with_limit(TL.rc03_spec(t20, CFG))
    R.append(Run("RC20", "v 18   gap 30   restricted library", ("core/rc20_al8hl.jsonl", "restricted_18_2_30"),
                 [("brake_hold", None, "Watch braking: with only brake_hold and lane_follow in the library, no recovery is supported.")],
                 lambda e: e.core.tl("RC03", t20, library=TL.rc03_library(False, 2.0), spec=sp20), tag="restricted"))
    R.append(Run("RC20", "v 18   gap 30   full library", ("core/rc20_al8hl.jsonl", "expanded_18_2_30"),
                 [("bypass_left", None, "Watch the bypass added to the library: the same scene is recoverable.")],
                 lambda e: e.core.tl("RC03", t20, library=TL.rc03_library(True, 2.0), spec=sp20), tag="full"))
    return R


# ----------------------------------------------------------------------------- thumbnails and index
def thumb(mp4, png):
    import imageio_ffmpeg
    from PIL import Image
    exe = imageio_ffmpeg.get_ffmpeg_exe()
    _, dur = imageio_ffmpeg.count_frames_and_secs(mp4)
    ts = [1.05, 1.0 + (dur - 2.0) / 3, 1.0 + 2 * (dur - 2.0) / 3, dur - 1.05]
    tmp, tiles = png + ".part%d.png", []
    for i, t in enumerate(ts):
        subprocess.run([exe, "-y", "-loglevel", "error", "-ss", f"{t:.3f}", "-i", mp4, "-frames:v", "1", tmp % i], check=True)
        tiles.append(Image.open(tmp % i).convert("RGB").resize((640, 360)))
    sheet = Image.new("RGB", (1280, 720))
    for i, im in enumerate(tiles):
        sheet.paste(im, ((i % 2) * 640, (i // 2) * 360))
    os.makedirs(os.path.dirname(png), exist_ok=True)
    sheet.save(png)
    for i in range(4):
        os.remove(tmp % i)


def write_index(runs, ttl):
    rows = ["| file | case | policy | world | verdict | G (this run) | stored G | sign check |", "|---|---|---|---|---|---|---|---|"]
    n_diff = 0
    for r in runs:
        rec = load_stored(*r.stored)
        for pol, world, _ in r.clips:
            st = r.stem(pol, world)
            mp = f"{VID}/meta/{st}.json"
            if not os.path.exists(mp):
                rows.append(f"| {st}.mp4 | {r.case} | {pol} | {world or rec['worlds'][0]} | NOT RECORDED | n/a | n/a | n/a |")
                continue
            m = json.load(open(mp))
            g_new, g_old = gnum(m["G"]), stored_G(rec, pol, world)
            chk = sign_check(g_new, g_old)
            n_diff += chk == "DIFFERS"
            rows.append(f"| {st}.mp4 | {r.case} | {pol} | {m['world']} | {m['verdict']} | {fmt_g(g_new)} | {fmt_g(g_old)} | {chk} |")
    head = ["# CARLA case videos (Task V)\n",
            "One clip per (case, policy[, world]) of each case's headline scene, re-run live on CARLA with a chase camera "
            "(`scripts/carla_videos.py`; clips are git-ignored, `results/carla/videos/*.mp4`).  Visualisation only: labels and records are unchanged.  "
            "`stored G` is the margin of the same scene/policy/world in the jsonl cache; `sign check` compares the side of eps = 0 (CARLA repeats are not "
            "bit-identical, a DIFFERS row is reported as-is and never re-run).\n",
            f"DIFFERS rows: {n_diff}\n"]
    os.makedirs(VID, exist_ok=True)
    open(f"{VID}/index.md", "w", encoding="utf-8", newline="\n").write("\n".join(head + rows) + "\n")
    return n_diff


# ----------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--only", default="")
    ap.add_argument("--force", action="store_true")
    a = ap.parse_args()
    only = {c.strip().upper() for c in a.only.split(",") if c.strip()}
    ttl = titles()
    runs = build_runs()
    todo = [r for r in runs if (not only or r.case in only)]
    os.makedirs(f"{VID}/meta", exist_ok=True)
    os.makedirs(f"{VID}/thumbs", exist_ok=True)
    pending = [r for r in todo if a.force or any(not os.path.exists(f"{VID}/{r.stem(p, w)}.mp4") for p, w, _ in r.clips)]
    print(f"{len(todo)} runs selected, {len(pending)} need recording", flush=True)
    failures, peak, t_start = {}, 0, time.time()
    if pending:
        from sim import video as V
        env = Env(a.host, a.port)
        world = env.core.world
        try:
            cleanup(world)
            V.enable_rendering(world, env.core.site)
            env.core.factory = lambda: V.RecordingPlant(env.core.client, env.core.world, env.core.site, decel=CC.DECEL_TARGET)
            env.share()
            state = {"run": None, "rec": None}

            def describe(spec, policy, w):
                r = state["run"]
                for pol, wn, sent in r.clips:
                    if policy.name == pol and (wn is None or w.get("name") == wn):
                        st = r.stem(pol, wn)
                        if os.path.exists(f"{VID}/{st}.mp4") and not a.force:
                            return None
                        sub = f"policy: {pol}   world: {w.get('name')}   {r.scene}"
                        return st, f"{r.case}  {ttl[r.case]}", sub, note_text(sent, state["rec"], r.extra() if callable(r.extra) else r.extra)
                return None

            written = V.install(VID, describe, eps=CFG.eps)
            for r in pending:
                state["run"], state["rec"] = r, load_stored(*r.stored)
                n0 = len(written)
                t0 = time.time()
                t = CCP.gpu_wait()
                peak = max(peak, t or 0)
                print(f"{r.case} {r.tag or ''} [{r.scene}] gpu {t} C ...", flush=True)
                try:
                    r.go(env)
                except Exception as ex:
                    traceback.print_exc()
                    if "time-out" in str(ex).lower() or "timeout" in str(ex).lower():
                        raise
                    failures[f"{r.case} {r.tag or ''}"] = f"{type(ex).__name__}: {ex}"
                    try:
                        cleanup(world)
                    except Exception:
                        raise
                for e in written[n0:]:
                    st = os.path.splitext(os.path.basename(e["file"]))[0]
                    json.dump({k: (str(v) if isinstance(v, float) and not math.isfinite(v) else v) for k, v in e.items()},
                              open(f"{VID}/meta/{st}.json", "w"))
                    thumb(e["file"], f"{VID}/thumbs/{st}.png")
                    print(f"  {st}.mp4  {e['verdict']}", flush=True)
                got = {os.path.splitext(os.path.basename(e["file"]))[0] for e in written[n0:]}
                for pol, wn, _ in r.clips:
                    if r.stem(pol, wn) not in got and not os.path.exists(f"{VID}/{r.stem(pol, wn)}.mp4"):
                        failures[f"{r.case} {r.tag or ''} {pol}"] = "no clip produced"
                print(f"  run wall {time.time() - t0:.0f} s", flush=True)
                peak = max(peak, gpu_temp() or 0)
        finally:
            try:
                s = world.get_settings()
                s.no_rendering_mode = True
                world.apply_settings(s)
            except Exception as ex:
                print("WARNING: could not restore no_rendering_mode:", ex, flush=True)
    n_diff = write_index(runs, ttl)
    print(f"index written; DIFFERS rows {n_diff}; failures {failures or 'none'}; peak GPU {peak} C; wall {time.time() - t_start:.0f} s", flush=True)


if __name__ == "__main__":
    main()
