# Task V report: CARLA case videos

## Built
- `scripts/carla_videos.py`: one CARLA client, one `Ctx`, three views (core/compositional/occlusion) sharing it; plant factory = `sim.video.RecordingPlant`. Each case's headline scene is RE-RUN through the group scripts' own functions (`CF.corr_rec`, `CC.rc03_rec/rc07_rec/rc12_rec/ctx.tl`, `CCP.Ctx.run`, `COC.Ctx.run`, `cases.negatives.rc09_run`); jsonl caches are only read for the stored-G sign check, never written.
- Per run: `gpu_wait()` before it; at start tick, destroy `vehicle.*`/`sensor.*`, tick; `finally` restores `no_rendering_mode=True`. `--only RC03,RC05`, `--force`, resume by existing mp4. Time-out errors re-raise (BLOCKED); none occurred.
- Output (git-ignored): `mining/results/carla/videos/<stem>.mp4`, `thumbs/<stem>.png` (2x2 tile at start / 1/3 / 2/3 / end via imageio_ffmpeg), `meta/<stem>.json`. Committed: `mining/results/carla/videos/index.md` (force-added, `results/` is ignored).
- `tests/test_carla_videos_pure.py`: stem/sanitising, sign check, stored-G lookup, catalogue titles, all 20 cases covered with unique stems.
- `mining/sim/video.py` change (drain-after-tick untouched): `_fit()` shrinks the font so long headers/sub lines (RC07 title is 66 chars) fit the frame, in the title/result cards and the overlay boxes.

## Result
36 clips, 20 cases, every case recorded, no failures. Sign check: 36 agree, 0 DIFFERS (G values reproduced to the printed 3 digits). Full set was recorded twice: pass 1 before the font-fit change, pass 2 (`--force`) after it so all clips share the same layout; both passes gave identical verdicts and G. Wall time: 439 s (pass 1), 451 s (pass 2). GPU temperature peak 63 C (sampled before/after each run), GPU memory about 2.8 GB.
Tests: `pytest -q tests`: 166 passed (5:25).

Clips (verdict): see `mining/results/carla/videos/index.md` for file, policy, world, verdict, G, stored G.

## Deviations / decisions
- RC04: the md has no positive witness. Used the only scene where a staged policy passes and brake_hold fails: v 22, gap 42, lc 2.0, extended library. Clips staged_brake_0.2_change (G +0.023) and immediate_lane_change (+0.068); note says "no positive witness found on CARLA" (the immediate change also passes there). Other staged_* variants not filmed.
- RC05: both clips in world gap_closes (the world where commit fails); abort_and_stabilize is safe in both worlds.
- RC09: only the 3 named policies were run (reduced library via `rc09_run(library=...)`, 25 -> 3 episodes); G of a policy depends only on its own episode.
- RC10/RC11/RC13: clips in world hid_own_lane (the world where the branch is exercised). RC13 uses scene (14,20,0.2); note records that the predictor wrongly rejects the lane change (envelope E2, per negatives.md). RC14: t_forget 0.6 (all three give identical results), world seen_own_then_occluded.
- RC12: prefix 0 (`brake_hold`) and prefix 2.25 (`brake_hold@prefix`, collision; first prefix above the stored feasible set). Stems carry `_p0` / `_p2.25`.
- RC16: offset 1.6 m (lane_follow G -0.155, nudge_left +0.045); the frozen-bicycle post spec is copied from `carla_foundation.rc16_sweep`.
- RC17: tau 0.40 has no exact stored key; stored G taken from key 0.4125 (same 8-tick delay). Noted in the clip text.
- RC19: v 8, anchor vs nuisance twin, stems `_anchor` / `_nuisance`. RC20: `_restricted` (brake_hold, 2-policy library) and `_full` (bypass_left, full library) re-runs of the RC03 v 18 gap 30 scene (so the RC20 clips repeat the RC03 outcomes).
- RC08 note carries the stored route_goal_completed flag. Notes carry the stored evidence class of the scene record.
- Collision clips: episode ends at contact; the clip continues with the last controls held (video.py design), so post-contact motion (e.g. RC11 ego flung aside) is physics aftermath, not part of the label.
- The untracked `scripts/g_sensitivity_carla.py` and `.idea/` in the tree are not mine and were not committed.
