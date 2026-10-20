"""
Time-step sensitivity on CARLA: the headline scenes re-run at fixed_delta_seconds 0.05 (the setting of every result) and 0.025, in the
SAME server session, and their margins compared policy by policy.

    cd mining && PYTHONNOUSERSITE=1 python ../scripts/carla_dt_check.py [--host localhost --port 2000]

Scenes: the headline scenes of scripts/carla_videos.build_runs (the same record-building calls), restricted to the cases whose margin
code is step-independent.  Excluded: RC18 (its a_comb margin smooths over a fixed number of TICKS, so a different step changes the
smoothing window itself, not just the physics).  Substepping stays at 0.01 s x 10, valid for both steps.  The actuation delay is
quantised to ticks (0.25 s = 5 ticks at 0.05, 10 at 0.025), so RC17's delay edge is expected to move toward the analytic 0.401 s.
Nothing is written to the jsonl caches; the world is restored to 0.05 s at the end.  Output: mining/results/carla/dt_check.md.
"""
import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "mining"))
sys.path.insert(0, HERE)

import carla_core as CC  # noqa: E402
import carla_videos as CV  # noqa: E402
from sim.plant_carla import CarlaPlant  # noqa: E402

DTS = (0.05, 0.025)
EXCLUDE = {"RC18"}


def gvals(rec):
    d = rec.to_json() if hasattr(rec, "to_json") else rec
    return d["evidence_class"], {f"{p}/{w}": CV.gnum(d["R_raw"][i][j]) for i, p in enumerate(d["policies"]) for j, w in enumerate(d["worlds"])}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    a = ap.parse_args()
    env = CV.Env(a.host, a.port)
    world = env.core.world
    runs = [r for r in CV.build_runs() if r.case not in EXCLUDE]
    res = {}
    try:
        for dt in DTS:
            CarlaPlant.configure_world(world, dt)
            world.tick()
            env.core.factory = lambda dt=dt: CarlaPlant(env.core.client, world, env.core.site, decel=CC.DECEL_TARGET, dt=dt)
            env.share()
            for r in runs:
                CV.cleanup(world)
                key = (r.case, r.tag or "", r.scene)
                res.setdefault(key, {})[dt] = gvals(r.go(env))
                print(dt, key, res[key][dt][0], flush=True)
    finally:
        CarlaPlant.configure_world(world, 0.05)
        world.tick()

    f = lambda g: "n/a" if g is None else ("-inf" if g == -math.inf else f"{g:+.3f}")
    L = ["# Time-step sensitivity on CARLA (0.05 s vs 0.025 s)\n",
         "Method: `scripts/carla_dt_check.py` docstring.  Same server session for both steps; substepping 0.01 s x 10.  G per policy and world; "
         "`shift` = G(0.025) - G(0.05) for finite pairs; a sign change across 0 is marked.\n",
         "| case | scene | evidence at 0.05 | evidence at 0.025 | policy / world | G at 0.05 | G at 0.025 | shift |", "|---|---|---|---|---|---|---|---|"]
    shifts, flips, ev_changes = [], 0, 0
    for (case, tag, scene), d in res.items():
        e05, g05 = d[0.05]
        e25, g25 = d[0.025]
        ev_changes += e05 != e25
        for k in g05:
            a0, a1 = g05[k], g25.get(k)
            sh = (a1 - a0) if (a0 is not None and a1 is not None and math.isfinite(a0) and math.isfinite(a1)) else None
            if sh is not None:
                shifts.append(abs(sh))
            flip = a0 is not None and a1 is not None and ((a0 >= 0) != (a1 >= 0))
            flips += flip
            L.append(f"| {case}{(' ' + tag) if tag else ''} | {scene} | {e05} | {e25}{' **changed**' if e05 != e25 else ''} | {k} | {f(a0)} | {f(a1)} | "
                     f"{'-' if sh is None else f'{sh:+.3f}'}{' **sign change**' if flip else ''} |")
    L.append(f"\n{len(res)} scenes, {sum(len(d[0.05][1]) for d in res.values())} policy-world margins.  Evidence-class changes: {ev_changes}.  "
             f"Margins changing sign: {flips}.  Largest finite shift: {max(shifts) if shifts else 0:.3f}; median {sorted(shifts)[len(shifts) // 2] if shifts else 0:.3f}.  "
             "For scale: repeats within one server session differ by up to 0.028, and across sessions by up to 0.196.\n")
    text = "\n".join(L) + "\n"
    open("results/carla/dt_check.md", "w", encoding="utf-8", newline="\n").write(text)
    print(text)


if __name__ == "__main__":
    main()
