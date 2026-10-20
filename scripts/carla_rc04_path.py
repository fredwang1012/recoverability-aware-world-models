"""
RC04 with path-length lane changes on CARLA (follow-up to Task 6, where 92 scenes with TIME-defined lane changes gave no witness).

    cd mining && PYTHONNOUSERSITE=1 python ../scripts/carla_rc04_path.py [--host localhost --port 2000]

Why the earlier search found nothing: an ideally tracked lane change defined in time has the same lateral acceleration at every
speed, so braking first shortens the distance it needs only when v < b (T + p/2) (about 10-12 m/s here), while a swerve only beats
braking when v > 2 b T (about 17 m/s): the windows do not overlap, and on CARLA the tracker's speed dependence was not enough to open
one (the kinematic mock plant does show one at v 18, lc 1.6).  cases/compositional.rc04_path_library defines the change by path length
lc_dist, so its duration is lc_dist / v and its lateral acceleration grows with v^2; the immediate and staged changes share lc_dist.

Declared before the CARLA run (taken from the mock-plant scan, which found the contrast at lc_dist 30 m / v 22 and lc_dist 40 m /
v 25-30): scenes below, each gap grid covering the mock window plus its edges.  Contrast rule = carla_compositional.rc04_contrast
(some staged member passes, brake_hold and immediate_lane_change both fail).  Spec, limits and tau are the Task 6 RC04 ones
(a_lat_max 8, tau 0.25, decel target 6.19).  Records: mining/results/carla/compositional/rc04p_*.jsonl; summary
mining/results/carla/rc04_path.md.  Labels come from evidence only.
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "mining"))
sys.path.insert(0, HERE)

from cases import compositional as CP  # noqa: E402
import carla_compositional as CCP  # noqa: E402

PS = (0.4, 0.8, 1.2)
GRID = [(30.0, 22.0, range(36, 48, 2)), (40.0, 25.0, range(42, 60, 2)), (40.0, 28.0, range(60, 74, 2))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    a = ap.parse_args()
    os.makedirs(CCP.OUT, exist_ok=True)
    ctx = CCP.Ctx(a.host, a.port)
    rows = []
    for L, v, gaps in GRID:
        lib_id = f"rc04_path_d{L:g}_ps{'-'.join(map(str, PS))}_{CCP.TAG}"
        for g in gaps:
            th = {"gap_m": float(g), "v_mps": v, "tau_s": CCP.TAU}
            r = ctx.point(f"rc04p_d{L:g}_v{v:g}_{CCP.TAG}", f"{g}", lambda th=th, L=L: ctx.run("RC04", th, library=CP.rc04_path_library(L, PS)),
                          library_id=lib_id)
            m = CCP.mw(r)
            staged = {k: x[0] for k, x in m.items() if k.startswith("staged")}
            best = max(staged, key=lambda k: -1e9 if staged[k] is None else staged[k])
            rows.append((L, v, g, r["evidence_class"], CCP.rc04_contrast(r), m["brake_hold"][0], m["immediate_lane_change"][0], best, staged[best],
                         r["traces"].get(f"immediate_lane_change/static_blocker", {}).get("margin", {}).get("dyn_peaks", {}).get("a_lat")))
            print("RC04p", rows[-1], flush=True)
    f = lambda x: "-inf" if x == float("-inf") else ("None" if x is None else f"{x:+.3f}")
    L_ = ["# RC04 with path-length lane changes (CARLA)\n",
          "Follow-up to Task 6 (`compositional.md`, RC04: 92 scenes with time-defined lane changes, no witness).  Method and the reason the "
          "earlier search found nothing: `scripts/carla_rc04_path.py` docstring.  Library `cases/compositional.rc04_path_library` "
          f"(brake_hold, lane_follow, immediate_lane_change and staged_brake_p_change for p in {PS}, all changes over the same path length), "
          "a_lat_max 8 m/s^2, tau 0.25 s, decel target 6.19 m/s^2.  Contrast = some staged member >= eps while brake_hold and the immediate change both < eps.\n",
          "| lc_dist (m) | v (m/s) | gap (m) | evidence | contrast | brake_hold G | immediate G | best staged (G) | immediate peak a_lat (m/s^2) |",
          "|---|---|---|---|---|---|---|---|---|"]
    for L, v, g, ev, c, bh, imm, best, sg, alat in rows:
        L_.append(f"| {L:g} | {v:g} | {g} | {ev} | {'**yes**' if c else 'no'} | {f(bh)} | {f(imm)} | {best} ({f(sg)}) | {'-' if alat is None else f'{alat:.2f}'} |")
    hits = [(L, v, g) for L, v, g, _, c, *_ in rows if c]
    L_.append(f"\nScenes with the RC04 contrast on CARLA: {len(hits)} of {len(rows)}: {hits or 'none'}.\n")
    text = "\n".join(L_) + "\n"
    open("results/carla/rc04_path.md", "w", encoding="utf-8", newline="\n").write(text)
    print(text)


if __name__ == "__main__":
    main()
