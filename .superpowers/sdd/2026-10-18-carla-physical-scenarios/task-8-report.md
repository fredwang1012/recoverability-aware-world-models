# Task 8 report: difficult negatives RC09, RC13

Status: DONE_WITH_CONCERNS

## Implemented
- `mining/cases/negatives.py`
  - RC09: the finite class `rc09_noescape_v1` (25 members: keep_speed, accelerate, constant brake_frac 0.0..1.0 step 0.1, four brake-then-keep staged members, bypass left lc 1.0/2.0, bypass right lc 2.0, brake+steer left lc 1.0/2.0 x b 0.5/1.0, staged brake 0.4 then change lc 1.0), scene builder (front frozen blocker, rear cruiser in the ego lane, queue of frozen vehicles filling the left lane from alongside the ego start to the blocker), `counterexamples()` (reads only record numbers/events), `decel_between()`.
  - RC13: `Envelope` / `action_rejected` (constant-velocity prediction of each observed object, box inflated by lat_m/lon_m + growth_mps*t, tested against the nominal swept footprint of one action; rejects if box separation < clearance_m), `predictor_threshold`, and `Recording` (transparent wrapper capturing the first delivered observation with an object). Not part of the evidence pipeline.
- `scripts/carla_negatives.py`: checkpointed CARLA run (JSONL per sweep, physics_profile_id check, audited per-member traces stored in the records), md generated from records. Output `mining/results/carla/negatives.md` (force-added); records in `mining/results/carla/negatives/*.jsonl` (ignored).
- `tests/test_sim_negatives.py`: 13 mock tests.

## Tests
`pytest -q tests`: 128 passed (115 before + 13 new), 347.9 s.
New tests: class is exactly the declared 25 members with parameters; every member refuted in the mock (U_LIBRARY, mix of blocker / rear / queue / off-road); controls (no rear -> brake_frac_1.0 witness; open left lane -> bypass witness); scene geometry (no queue overlaps, bumper gap); counterexamples follow numbers; decel helper; nominal ego motion; exact predictor accepts/rejects correctly and inflation closes the corridor with a threshold; growth/time resolution are parameters; predictor input is anonymous; Recording wrapper transparent and captures first sighting; mock RC13 end-to-end (exact predictor 4/4 agreement, inflated predictor false-rejects the witness).

## CARLA results (single server, about 8 min of CARLA time, 13 records, validate_records 0 failures)
### RC09 (label U_LIBRARY relative to `rc09_noescape_v1` only, one world `rear_cruise`)
Scenes (front gap = int(v^2/(2*6.211)+0.3v+1)+3; rear bumper gap, rear speed v+6; queue in left lane):
| v | gap | rear gap | evidence | refuted / 25 | contact episodes |
|---|---|---|---|---|---|
| 10 | 15 | 25 | U_LIBRARY | 25/25 | 24 |
| 14 | 23 | 15 | U_LIBRARY | 25/25 | 24 |
| 14 | 23 | 25 | U_LIBRARY | 25/25 | 24 |
| 14 | 23 | 35 | U_LIBRARY | 25/25 | 24 |
| 18 | 35 | 25 | U_LIBRARY | 25/25 | 24 |
All members G = -inf. The 24 contacts; the 25th (`bypass_right_lc2`) leaves the road (road-edge margin). Selected scene (v 14, gap 23, rear gap 25) per-member counterexample (actor hit, time): keep_speed/accelerate/brake_frac 0.0-0.3/brake_0.3_*/brake_0.6_for_1s hit the blocker (1.55-2.35 s); brake_frac 0.4-1.0 and brake_0.6_for_2s stop and are hit by the rear vehicle (2.25-2.40 s); every lane-change member hits a queue vehicle adj2/adj3/adj4 (0.85-1.30 s). Full table in negatives.md.
Controls (v 14): no rear vehicle -> R_REF (witnesses brake_frac 0.6..1.0, brake_0.6_for_2s); left lane open -> R_REF (witness bypass_left_lc2, G 0.022). So the rear vehicle and the queue each close the scene.
Brake-fraction -> measured decel (clear road, window 0.9 v0 .. 0.4 v0): v0 14: f=0 3.98, 0.1 4.11, 0.2 4.24, 0.3 4.40, 0.4 4.55, 0.5 4.72, 0.6 4.86, 0.7 5.32, 0.8 5.71, 0.9 6.10, 1.0 6.27 m/s^2 (v0 18: 4.71 .. 6.25). Nonlinear, engine-drag floor about 4 m/s^2 at f=0.
### RC13 (4 RC10-construction scenes, all R_REF with this task's own runs)
Witness `shared_slowdown_then_branch`: G in world A / world B = 0.105/0.049, 0.121/0.050, 0.105/0.018, 0.105/0.105. Predictor first-sighting observations recorded from CARLA.
- Exact CV envelope (E0, lat=lon=0, growth 0, horizon 4 s, dt 0.1, clearance 0): agrees with evidence on 4/4 cells in all 4 scenes (min box separation world B change_left 0.54-0.83 m).
- Over-approximation: uniform inflation 1.0 m (E2) already rejects the witness branch in world B (`change_left`) in all 4 scenes although r >= eps in both worlds; smallest rejecting uniform inflation world B: 0.85, 0.55, 0.55, 0.85 m; world A (`stay`): 1.7 m in all. Growth 0.25 m/s on 0.5 m (E5) also rejects world B in all scenes. 0 false acceptances anywhere. Time resolution 0.5 s (E7) did not change decisions with zero inflation except for none (agrees 4/4).
- Predictor parameters per row E0..E8 are listed in negatives.md.

## Files changed
- mining/cases/negatives.py (new), scripts/carla_negatives.py (new), tests/test_sim_negatives.py (new), mining/results/carla/negatives.md (force-added), this report.

## Self-review notes
- Labels only from evidence; the predictor never feeds the pipeline; the "other action" cells use fixed policies executed from t=0, only an approximate counterpart of the first-sighting state (stated in md).
- RC09 class and counterexamples are generated from the policy descriptors and records. Ego v at end in the counterexample table is post-impact (negative recoil); noted in md.
- No map change, no load_world, no server control.

## Concerns
- RC09 is a single-world, strongly constructed negative: with a non-braking rear vehicle and a fully occupied left lane every in-lane policy fails by construction; the label is U_LIBRARY for that class and world only, and the md says so. The left-lane block is a queue of up to 5 frozen vehicles (not a single actor).
- Rear vehicle starts 3.2-5.3% below declared speed (warm-up drag), reported in the Plant caveats paragraph.
- RC13 world-B witness margins are small (0.018 to 0.105; 0.018 is within the observed run-to-run spread 0.017 of eps); single runs.
- RC13 uses box (axis-aligned) separation with clearance 0 for the predictor, a different metric from the evidence's OBB corner clearance with 1 m reserve; the exact predictor happens to agree with evidence on all cells.
- The decel diagnostic is from clear-road runs (the RC09 episodes end at first contact).
