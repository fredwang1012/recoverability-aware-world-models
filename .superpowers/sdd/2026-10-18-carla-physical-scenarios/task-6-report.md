# Task 6 report: compositional CARLA cases (RC04, RC05, RC06, RC08, RC15, RC18)

Status: DONE_WITH_CONCERNS. Commits: 3b6840f (code, mock tests), 8e72cb1 (CARLA run, summary force-added).
Full CARLA table, scan rows and audited witness traces: `mining/results/carla/compositional.md` (force-added). Records: `mining/results/carla/compositional/*.jsonl` (154, 0 validation failures). Total CARLA time about 10 min; GPU 55 C throughout; no server problems.

## Implemented
- `mining/cases/compositional.py`: spec builders, libraries, worlds and `run` for the six cases. All specs declare `a_lat_max` 8.0; RC18 also `a_comb_max` 8.0 (and `a_comb_vmin`).
- `mining/sim/policies.py`, observation-only, additive: `BrakeSteer` (simultaneous brake and steer), `ImmediateEntry`, `AlwaysMerge`, `WaitObserveEnter`, `ObserveYieldElseFallback`, `LaneChangePrefix` (commit), `AbortAndStabilize`, `AbortIfClosing`.
- `mining/sim/actors.py`: `yield` mode accepts `trigger_t` (time-keyed). Actor modes are per world (`rc05_worlds`: cruise or brake; `rc15_worlds`: yield or nonyield).
- `mining/sim/harness.py` (backward compatible):
  - `a_comb_max` friction-circle dynamic-limit margin, sqrt(a_long^2 + a_lat^2), from measured accelerations. a_long is a 5-tick (0.25 s) centred average of the speed difference. a_lat is |v*yaw_rate| as before.
  - `dyn_peaks` and per-series `dyn_margin` stored in `traces[...]["margin"]`.
  - `route_goal` gives `route_goal_completed` per run in `traces[...]`. It is never part of the margin, the terminal test or the label.
- RC05: the ego physically executes the left lane-change prefix inside the episode. A test and the CARLA audit show max per-tick lateral step <= 0.18 x (v*dt). No lateral teleport.
- `scripts/carla_compositional.py`: checkpointed sweeps, pre-declared selection rules (in the header), full-library audit re-runs with a per-tick spy, report.
- `tests/test_sim_compositional.py`: 11 mock tests, one or more per builder, plus harness smoothing, the `trigger_t` actor and the script's pure helpers.

## Tests
`pytest -q tests`: 96 passed (85 before + 11 new).

## CARLA results (physics profile pp_381b4c6e55, eps 0.0, reserve 1.0 m)
| case | scenes scanned | scenes with contrast | selected scene | label | witness | refuted |
|---|---|---|---|---|---|---|
| RC04 | 92 | 0 | NONE | - | no positive witness | - |
| RC05 | 12 | 12 | blocker 80, rear_gap 20, rear_v 20, v 14 | R_REF | abort_and_stabilize (0.105/0.416) | commit_lane_change -inf in gap_closes |
| RC06 | 12 | 5 | blocker 60, pass_gap 10, pass_v 20, v 10 | R_REF | wait_observe_enter (0.084) | immediate_entry -inf |
| RC08 open lane | 1 | - | gap 45, v 14 | R_REF | brake_hold and bypass_left | route_goal_completed: brake_hold False, bypass True |
| RC08 closed lane | 1 | - | gap 45, v 14 | R_REF | brake_hold only | route_goal_completed False for all |
| RC15 | 12 | 9 (all strong) | blocker 60, adj_gap 35, adj_v 16, v 10 | R_REF | observe_yield_else_fallback (0.084/0.105) | always_merge 0.083/-inf |
| RC18 | 12 scenes x 2 specs | 5 | v 22, gap 42 | R_REF | bypass_left (keep speed, lc 2.0) | brake_steer_cmd_lc2_b1 refuted by a_comb |

Audited traces are in the md, one per family, each with a full-library re-run. Re-run vs scan max |dG| was at most 4.4e-6.

- **RC05:** abort_and_stabilize passes both worlds. The commit trace collides with the rear vehicle in gap_closes. The abort trace's lateral range is only [-0.74, 0.03] m, so it aborted early in the prefix.
- **RC18:**
  - The refuted command is full brake plus steer to the left lane in 2.0 s.
  - Under the a_lat-only spec it passes (G 0.194). Its measured peaks are a_lat 6.13, smoothed a_long 8.92, a_comb 10.7. Under the combined spec G = -0.339.
  - The feasible alternative, bypass_left, has a_comb 7.14 and G 0.068.
  - At v 22, gap 34 the evidence flips R_REF to U_LIBRARY when the combined limit is added: the only witness was the refuted command. That is U_LIBRARY, never U_MODEL.
  - Refuted means the request violates the margin. It is not a claim that the scene is unrecoverable.

## Honest negatives and disclosures
- **RC04 has no positive witness on CARLA** under the 8.0 m/s^2 limit.
  - Grid: lc_time {1.6, 2.0} x v {14, 16, 18, 20, 22} x gap about 16-46 step 2 (base grid plus extended grid), staged ps 0.2-1.0.
  - Where brake_hold fails, every staged policy fails too, because braking first uses up the distance.
  - The one scene where a staged policy passes and brake_hold fails is v 22, lc 2.0, gap 42, and there the immediate lane change also passes (0.068). So the staged policy is not the unique continuation. Nothing was forced.
- **RC06 and RC15:** with a static blocker, brake_hold is also a witness in every scene. The case contrast is wait vs immediate (RC06) and observe-yield vs always-merge (RC15). The record is R_REF either way.
- **RC18 authority:** CARLA's tyre friction (`friction_fixed` 5.0) is non-physical, so the declared 8.0 m/s^2 limits are the authority. This is stated in the md.
- **a_comb speed mask (my addition):**
  - Diagnostic: a brake_hold run at v 18 showed CARLA locking the wheels at the end of a stop, -28 to -35 m/s^2 for about 4 ticks below about 4.3 m/s.
  - The first run counted those ticks and made every full brake look like it exceeded the friction circle (brake_hold a_comb margin -1.96).
  - I added `a_comb_vmin` = 5 m/s: a_comb is evaluated only while the ego speed is at least 5 m/s. The unmasked peak is still recorded as `a_comb_unmasked`.
  - The constant was set from a measurement-validity diagnostic, not from the eps or label results. I discarded the first run, which used no mask, and reran everything. It is kept under `superseded_run1/` (git-ignored).
  - Only the a_comb margin is affected. RC04, RC05, RC06 and RC15 use a_lat only, so their labels did not change between runs.
- **Reproducibility:** CARLA is run-to-run repeatable on these scenes, with re-run dG at most 4.4e-6. The earlier single-policy audit gave a different G for bypass_left (-0.024 vs 0.068), which depends on episode order. I replaced it with full-library re-runs in the scan's episode order.
- **Margins are thin.** Several R_REF witnesses sit at 0.05-0.12, as in the core cases: RC05 abort 0.05 at blocker gap 60, RC06 wait 0.084. They are not tolerance-tuned.
- **abort_if_closing** (observation-conditioned, ttc 4 s) is a witness in only some RC05 scenes. I did not tune it after seeing results.
- Mock and CARLA outputs stay in separate files.

## Files changed
- mining/sim/harness.py
- mining/sim/policies.py
- mining/sim/actors.py
- mining/cases/compositional.py (new)
- scripts/carla_compositional.py (new)
- tests/test_sim_compositional.py (new)
- mining/results/carla/compositional.md (force-added)

## Self-review
- I read my diff.
- Policies use only delivered `Obs`. Worlds fix the actor mode once. Labels come only from `evaluate`/`classify`.
- No eps or tolerance was moved. The selection rules were declared in the script header.
- `AbortAndStabilize` initially stopped with a residual yaw and failed the lane_follow terminal test on the mock. I fixed it by steering back first and braking after the return settles. This is part of the policy definition and was done before any CARLA run.
- The RC04 explanatory sentence in the report ("the immediate lane change passes as well") is hard-coded text next to a computed list. It is true for the current data. It would need a recheck if the grid changes.

---
# Fix report, round 1 (commit after 8e72cb1)

Tests: `pytest -q tests` -> 98 passed (96 + 2 new: mask-after-smoothing synthetic trace incl. vmin override and unmasked peak; harness records spec vmin and peak times). CARLA re-run: 206 records, 0 validation failures, GPU at most 63 C. Old runs kept git-ignored under `mining/results/carla/superseded_run1/` and `superseded_run2/`.

1. **Actor start speeds (plant bug, fixed).**
   - Cause: warm-up. The single-gear clutch engagement drags every car down; the ego is re-imposed after its physics control, but moving non-ego actors were not. A Mini nominal 20 m/s dipped to about 11 m/s and the PI hold (ki 0.3, i from 0) only recovered to 16.8-17.04 by t0.
   - Fix in `plant_carla.build`: every warm-up tick, after reading the measured speed, moving non-ego actors get `set_target_velocity` to the declared speed (and once more before the final t0 tick). The PI therefore integrates the per-tick deficit and learns hold_i.
   - Result at t0 (audit traces, first tick): rear 18.94 vs 20 (was 17.04), pass 15.50 vs 16 and adj 15.50 vs 16 (adj was 14.79 vs 16). Still 2.5-5% low: one un-pinned drag tick (engine braking is about 0.5-0.8 m/s per tick at throttle below 0.9) remains. Not fully closed; disclosed.
   - No mock-free unit test is possible (needs the CARLA server). Checked by a live diagnostic and the audited traces.
   - Affected: all moving-actor cases (RC05, RC06, RC15), re-run. Their scans changed (RC05 window 12 -> 8 scenes, RC06 5 -> 11, RC15 9 -> 6, all still R_REF witnesses). The selected scenes moved: RC05 (80,35,20), RC06 (60,20,16).
   - Task 4 impact (not re-run): RC07 used a mover `rear` (rear_v 18-24), so it started below nominal by the same mechanism (about 10-15% low at 20 m/s), and its rear-actor gap/arrival timing is shifted. RC20 and RC03 use static blockers only, so they are unaffected. The controller should rule on whether to re-run RC07.
2. **RC18 framing.**
   - The refutation holds for a_long alone. With the mask applied after smoothing, peak smoothed a_long: brake_steer_cmd_lc2_b1 13.76 at t 2.80 s, v 6.0 m/s (a_comb 13.77, same time); brake_hold 14.93 at t 2.85 s, v 5.9 m/s; bypass_left 1.72 (a_comb 7.14 at v 22.3). Unmasked peaks 24.4 at v 3.3 and 2.9 m/s.
   - These peaks are the end-of-stop wheel-lock transient leaking through the 5-tick smoothing window into the first ticks above 5 m/s. brake_hold is flagged too (a_comb margin -0.87), so the numbers isolate neither a combined-limit effect nor a command-specific violation. Before the mask change (superseded_run2): command a_long 8.92 (a_comb 10.7), brake_hold a_comb margin +0.056. The refutation of the command is therefore sensitive to the mask-edge treatment.
   - RC18 keeps a clean contrast only for the feasible alternative (bypass_left, a_comb 7.14, R_REF). The md no longer calls the refutation a combined-limit effect. Scene (22,34) still flips R_REF -> U_LIBRARY when a_comb_max is added; that label is relative to this library.
3. **Mask edge.** `harness.accel_series`: smooth first, then mask. Tested on a synthetic trace (ticks below vmin excluded, unmasked still sees the spike, spec vmin overrides default). It also records the time and speed of each peak.
4. **RC06/RC15 contrast.**
   - RC15 disclosure added: with brake_hold a witness in both worlds the evidence label carries no yielding information.
   - Tight grid (blocker inside stop distance), RC15: v 10/14/18, 4 gaps each, adj_gap 25/35, default and fast yield timing (48 records); RC06: 4 scenes. Result: every scene is U_LIBRARY, none R_REF, none U_INFORMATION. observe_yield_else_fallback never passes both worlds there (it waits to observe the yield, which costs the distance). No recoverability-relevant contrast appears. The only passes: always_merge at v 18, gaps 31 (0.043 in the yield world), and none for the other policies.
5. **Minors.** Correct dG values in the superseded run: RC05 1.1e-6, RC06 4.4e-6, RC15 6.08e-4, RC18 0.0. New run: RC05 3.6e-10, RC06 7.8e-7, RC15 2.9e-7, RC18 9.2e-4.
   - The RC04 library text now says ps 0.4/0.8 base and 0.2-1.0 extended. The "immediate also passes" sentence is computed (1 of 1).
   - The RC04 test is renamed `test_rc04_mock_staged_passes_where_brake_hold_and_immediate_change_fail`.
   - Ego half-length now comes from `Obs.ego_hl`, filled by the observation feed (the old Task 4 `_obstacle_passed` is unchanged).
   - `__import__` replaced by a normal import.
   - `snap.ax` exists in both plants (the kinematic returns the ego acceleration; CARLA returns the tick difference of ego speed; first CARLA step is None, treated as 0).
