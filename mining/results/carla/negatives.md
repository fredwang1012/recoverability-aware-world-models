# CARLA difficult negatives (Task 8: RC09, RC13)

> **Superseded claim.** The RC13 "4 of 4 robust" headline below uses the Task 5 sigma 0.0167; at the maximum observed repeat spread 0.028 it is 2 of 4 (see `docs/carla_results.md` section 7, Repeatability).

Physics profile `pp_381b4c6e55`, decel target 6.19, eps = 0.0, reserve c = 1.0 m (margin.DEFAULT_V2, unchanged), act delay 0.25 s, `a_lat_max = 8` m/s^2 (dynamic-limit margin). Records: `results/carla/negatives/*.jsonl`. Labels are `contract.classify` output only; G cells read one value per world (-inf = hard failure, None = unrun). Mock and CARLA outputs are kept apart.

**Plant caveats.** Frozen (`static`) actors are exact: blocker, the left-lane queue and (RC13) the hidden vehicle are at their declared places. The only moving non-ego actor is the RC09 `rear` vehicle, which starts 3-5% below its declared speed (warm-up drag, as in the compositional runs): RC09 v 10, rear gap 25: `rear` 15.50 measured at the first tick vs 16 declared (3.2% low); RC09 v 14, rear gap 15: `rear` 18.94 measured at the first tick vs 20 declared (5.3% low); RC09 v 14, rear gap 25: `rear` 18.94 measured at the first tick vs 20 declared (5.3% low); RC09 v 14, rear gap 35: `rear` 18.94 measured at the first tick vs 20 declared (5.3% low); RC09 v 18, rear gap 25: `rear` 22.82 measured at the first tick vs 24 declared (4.9% low). The declared rear speed is therefore an upper bound of what CARLA ran; all margins are for the measured speeds. The ego starts on target. Repeats on this plant show a small run-to-run spread (up to about 0.017 margin units observed in earlier tasks); the entries below are single runs, so margins close to eps would not be robust, but the RC09 refutations are collisions (-inf) or large violations.

## RC09: front/rear conflict with no escape

**Label scope.** The label is `U_LIBRARY` relative to the finite policy class `rc09_noescape_v1` ONLY, in the declared world set {`rear_cruise`} (one world: the rear vehicle keeps its speed and lane). It is not a claim that no controller whatsoever could recover, not U_MODEL, and not a claim about rear vehicles that brake or yield.

**Scene.** The custom 2-lane straight (no map change). Ego in the right lane at speed v. Front: a frozen blocker in the ego lane, bumper gap g = int(v^2/(2*6.211) + 0.3 v + 1) + 3 m, i.e. a scene where full braking alone would stop with the 1 m reserve if nothing were behind (control below). Rear: a vehicle in the ego lane, rear bumper gap 25 m (also 15 and 35 m at v = 14), cruising 6 m/s faster than the ego. Left lane: a queue of frozen vehicles from alongside the ego's start to the blocker (no lane change can complete); the right side is the road edge (lat +1.75 m).

**The class, exactly** (`rc09_noescape_v1`, 25 members, library id `rc09_noescape_v1_al8` in every record):

| # | id | implementation | parameters |
|---|---|---|---|
| 0 | `keep_speed` | LaneFollow | {} |
| 1 | `accelerate` | Accelerate | {} |
| 2 | `brake_frac_0.0` | BrakeHold | {'brake_frac': 0.0} |
| 3 | `brake_frac_0.1` | BrakeHold | {'brake_frac': 0.1} |
| 4 | `brake_frac_0.2` | BrakeHold | {'brake_frac': 0.2} |
| 5 | `brake_frac_0.3` | BrakeHold | {'brake_frac': 0.3} |
| 6 | `brake_frac_0.4` | BrakeHold | {'brake_frac': 0.4} |
| 7 | `brake_frac_0.5` | BrakeHold | {'brake_frac': 0.5} |
| 8 | `brake_frac_0.6` | BrakeHold | {'brake_frac': 0.6} |
| 9 | `brake_frac_0.7` | BrakeHold | {'brake_frac': 0.7} |
| 10 | `brake_frac_0.8` | BrakeHold | {'brake_frac': 0.8} |
| 11 | `brake_frac_0.9` | BrakeHold | {'brake_frac': 0.9} |
| 12 | `brake_frac_1.0` | BrakeHold | {'brake_frac': 1.0} |
| 13 | `brake_0.3_for_1s_then_keep` | BrakeThenKeep | {'frac': 0.3, 'dur_s': 1.0} |
| 14 | `brake_0.3_for_2s_then_keep` | BrakeThenKeep | {'frac': 0.3, 'dur_s': 2.0} |
| 15 | `brake_0.6_for_1s_then_keep` | BrakeThenKeep | {'frac': 0.6, 'dur_s': 1.0} |
| 16 | `brake_0.6_for_2s_then_keep` | BrakeThenKeep | {'frac': 0.6, 'dur_s': 2.0} |
| 17 | `bypass_left_lc1` | Bypass | {'side': -1, 'lc_time': 1.0, 'brake_s': 0.0} |
| 18 | `bypass_left_lc2` | Bypass | {'side': -1, 'lc_time': 2.0, 'brake_s': 0.0} |
| 19 | `bypass_right_lc2` | Bypass | {'side': 1, 'lc_time': 2.0, 'brake_s': 0.0} |
| 20 | `brake_steer_left_lc1_b0.5` | BrakeSteer | {'side': -1, 'lc_time': 1.0, 'brake_frac': 0.5} |
| 21 | `brake_steer_left_lc1_b1` | BrakeSteer | {'side': -1, 'lc_time': 1.0, 'brake_frac': 1.0} |
| 22 | `brake_steer_left_lc2_b0.5` | BrakeSteer | {'side': -1, 'lc_time': 2.0, 'brake_frac': 0.5} |
| 23 | `brake_steer_left_lc2_b1` | BrakeSteer | {'side': -1, 'lc_time': 2.0, 'brake_frac': 1.0} |
| 24 | `staged_brake_0.4_change_lc1` | StagedBrakeChange | {'p': 0.4, 'lc_time': 1.0, 'side': -1} |

Members `brake_frac_f` command a constant fraction f of full brake until the end (f = 0.0 is coasting, engine braking only); `keep_speed` holds the initial speed; `accelerate` is throttle 0.7; `brake_F_for_Ds_then_keep` brakes at fraction F for D seconds, then holds the speed reached; lane-change members steer to the adjacent lane centre (left = the queue lane, right = off the road) in lc seconds.

### Scenes

| v | gap | rear gap | evidence | members refuted (G < eps) / total | members with a collision | margins (G per member) |
|---|---|---|---|---|---|---|
| 10 | 15 | 25 | U_LIBRARY | 25/25 | 24 | 25 / 0 / 0 |
| 14 | 23 | 15 | U_LIBRARY | 25/25 | 24 | 25 / 0 / 0 |
| 14 | 23 | 25 | U_LIBRARY | 25/25 | 24 | 25 / 0 / 0 |
| 14 | 23 | 35 | U_LIBRARY | 25/25 | 24 | 25 / 0 / 0 |
| 18 | 35 | 25 | U_LIBRARY | 25/25 | 24 | 25 / 0 / 0 |

Acceptance (every member refuted by a resolved world) holds in all 5 scenes: **True**.

### Counterexample for every member (selected scene v 14, gap 23, rear gap 25, world `rear_cruise`)

Trace data are the per-episode records (contact events from the CARLA collision sensor, ego speed at the end of the episode, minimum clearance to every actor from the margin parts).

| member | world | G | termination | collided with | at t (s) | ego v at end | ego lat at end | min clearance to blk / rear / nearest adj (m) |
|---|---|---|---|---|---|---|---|---|
| `keep_speed` | rear_cruise | -inf | collision | blk | 1.65 | 14.00 | -0.00 | -0.10 / 15.58 / 1.52 |
| `accelerate` | rear_cruise | -inf | collision | blk | 1.55 | -2.57 | -0.01 | -0.07 / 17.66 / 1.50 |
| `brake_frac_0.0` | rear_cruise | -inf | collision | blk | 2.20 | -0.97 | -0.00 | -0.03 / 4.37 / 1.51 |
| `brake_frac_0.1` | rear_cruise | -inf | collision | blk | 2.25 | -0.63 | -0.00 | -0.03 / 3.37 / 1.51 |
| `brake_frac_0.2` | rear_cruise | -inf | collision | blk | 2.30 | -0.39 | -0.01 | -0.01 / 2.35 / 1.51 |
| `brake_frac_0.3` | rear_cruise | -inf | collision | blk | 2.35 | -0.02 | 0.00 | -0.03 / 1.36 / 1.52 |
| `brake_frac_0.4` | rear_cruise | -inf | collision | rear | 2.40 | -0.00 | -0.00 | 0.34 / -0.01 / 1.52 |
| `brake_frac_0.5` | rear_cruise | -inf | collision | rear | 2.40 | 0.02 | -0.00 | 0.95 / 0.07 / 1.52 |
| `brake_frac_0.6` | rear_cruise | -inf | collision | rear | 2.35 | 0.24 | -0.00 | 1.57 / -0.02 / 1.52 |
| `brake_frac_0.7` | rear_cruise | -inf | collision | rear | 2.35 | 0.02 | 0.00 | 2.09 / -0.04 / 1.52 |
| `brake_frac_0.8` | rear_cruise | -inf | collision | rear | 2.30 | 0.27 | 0.00 | 2.61 / -0.05 / 1.52 |
| `brake_frac_0.9` | rear_cruise | -inf | collision | rear | 2.30 | 0.02 | 0.00 | 3.06 / -0.03 / 1.52 |
| `brake_frac_1.0` | rear_cruise | -inf | collision | rear | 2.25 | -0.00 | -0.00 | 3.50 / -0.17 / 1.52 |
| `brake_0.3_for_1s_then_keep` | rear_cruise | -inf | collision | blk | 2.10 | -1.21 | 0.01 | 0.02 / 6.33 / 1.51 |
| `brake_0.3_for_2s_then_keep` | rear_cruise | -inf | collision | blk | 2.35 | -0.58 | 0.00 | -0.02 / 1.36 / 1.52 |
| `brake_0.6_for_1s_then_keep` | rear_cruise | -inf | collision | blk | 2.15 | -1.44 | 0.00 | -0.08 / 5.42 / 1.52 |
| `brake_0.6_for_2s_then_keep` | rear_cruise | -inf | collision | rear | 2.35 | 8.87 | -0.00 | 1.49 / 0.00 / 1.52 |
| `bypass_left_lc1` | rear_cruise | -inf | collision | adj3 | 0.90 | 13.92 | -1.25 | 10.41 / 20.15 / -0.09 |
| `bypass_left_lc2` | rear_cruise | -inf | collision | adj4 | 1.30 | 14.00 | -1.30 | 4.73 / 17.74 / -0.31 |
| `bypass_right_lc2` | rear_cruise | -inf | horizon | none | - | 12.77 | 40.91 | 1.27 / 9.49 / 1.51 |
| `brake_steer_left_lc1_b0.5` | rear_cruise | -inf | collision | adj2 | 0.85 | 10.72 | -0.91 | 11.83 / 19.71 / -0.05 |
| `brake_steer_left_lc1_b1` | rear_cruise | -inf | collision | adj2 | 0.85 | 10.38 | -0.90 | 11.91 / 19.63 / -0.07 |
| `brake_steer_left_lc2_b0.5` | rear_cruise | -inf | collision | adj3 | 1.25 | 9.11 | -1.02 | 7.69 / 15.78 / -0.10 |
| `brake_steer_left_lc2_b1` | rear_cruise | -inf | collision | adj3 | 1.25 | 8.53 | -1.00 | 7.95 / 15.54 / -0.09 |
| `staged_brake_0.4_change_lc1` | rear_cruise | -inf | collision | adj3 | 1.25 | 12.19 | -1.02 | 7.06 / 16.36 / -0.31 |

Grouped by what refutes the member: **blk**: 9; **rear**: 8; **adj3**: 4; **adj4**: 1; **no contact (G < eps from clearance/terminal/dynamic limit)**: 1; **adj2**: 2.

- blk: `keep_speed`, `accelerate`, `brake_frac_0.0`, `brake_frac_0.1`, `brake_frac_0.2`, `brake_frac_0.3`, `brake_0.3_for_1s_then_keep`, `brake_0.3_for_2s_then_keep`, `brake_0.6_for_1s_then_keep`
- rear: `brake_frac_0.4`, `brake_frac_0.5`, `brake_frac_0.6`, `brake_frac_0.7`, `brake_frac_0.8`, `brake_frac_0.9`, `brake_frac_1.0`, `brake_0.6_for_2s_then_keep`
- adj3: `bypass_left_lc1`, `brake_steer_left_lc2_b0.5`, `brake_steer_left_lc2_b1`, `staged_brake_0.4_change_lc1`
- adj4: `bypass_left_lc2`
- no contact (G < eps from clearance/terminal/dynamic limit): `bypass_right_lc2`
- adj2: `brake_steer_left_lc1_b0.5`, `brake_steer_left_lc1_b1`

**Gentle braking is dominated.** Lower brake fractions do not help (measured decel is about 4 to 5 m/s^2 for f <= 0.6 on this plant, see the decel table; there is no 1-3 m/s^2 level): the constant-brake members that hit the blocker are ['brake_frac_0.0', 'brake_frac_0.1', 'brake_frac_0.2', 'brake_frac_0.3'] (contact at [2.2, 2.25, 2.3, 2.35] s), and every member that stops short is hit from behind. Lower deceleration arrives at the blocker with speed; higher deceleration stops short and is overtaken by the rear vehicle.

**The in-lane refutation is structural.** Given a rear vehicle that never reacts and a blocker that never moves, a vehicle confined to its lane is struck by one of them whatever speed profile it follows (either it is not stopped before the blocker, or it is stopped and overtaken). The class-wide refutation of the in-lane members is therefore a consequence of the construction (declared world, blocked left lane, road edge on the right), not an empirical discovery. The empirical content is that the lane-change members, which are not structurally excluded, are also refuted (queue contact before the manoeuvre completes) and that the refutations were realised on CARLA with the calibrated plant.


### Controls (v 14, gap 23, same class)

Each control removes one element of the scene to show what closes it. These are separate records and are not part of the RC09 label.

| control | evidence | members with G >= eps | margins |
|---|---|---|---|
| no rear vehicle (left queue present) | R_REF | ['brake_frac_0.6', 'brake_frac_0.7', 'brake_frac_0.8', 'brake_frac_0.9', 'brake_frac_1.0', 'brake_0.6_for_2s_then_keep'] | keep_speed=-inf; accelerate=-inf; brake_frac_0.0=-inf; brake_frac_0.1=-inf; brake_frac_0.2=-inf; brake_frac_0.3=-inf; brake_frac_0.4=-0.132; brake_frac_0.5=-0.006; brake_frac_0.6=0.104; brake_frac_0.7=0.104; brake_frac_0.8=0.104; brake_frac_0.9=0.104; brake_frac_1.0=0.104; brake_0.3_for_1s_then_keep=-inf; brake_0.3_for_2s_then_keep=-inf; brake_0.6_for_1s_then_keep=-inf; brake_0.6_for_2s_then_keep=0.104; bypass_left_lc1=-inf; bypass_left_lc2=-inf; bypass_right_lc2=-inf; brake_steer_left_lc1_b0.5=-inf; brake_steer_left_lc1_b1=-inf; brake_steer_left_lc2_b0.5=-inf; brake_steer_left_lc2_b1=-inf; staged_brake_0.4_change_lc1=-inf |
| left lane open (rear vehicle present) | R_REF | ['bypass_left_lc2'] | keep_speed=-inf; accelerate=-inf; brake_frac_0.0=-inf; brake_frac_0.1=-inf; brake_frac_0.2=-inf; brake_frac_0.3=-inf; brake_frac_0.4=-inf; brake_frac_0.5=-inf; brake_frac_0.6=-inf; brake_frac_0.7=-inf; brake_frac_0.8=-inf; brake_frac_0.9=-inf; brake_frac_1.0=-inf; brake_0.3_for_1s_then_keep=-inf; brake_0.3_for_2s_then_keep=-inf; brake_0.6_for_1s_then_keep=-inf; brake_0.6_for_2s_then_keep=-inf; bypass_left_lc1=-0.840; bypass_left_lc2=0.022; bypass_right_lc2=-inf; brake_steer_left_lc1_b0.5=-inf; brake_steer_left_lc1_b1=-inf; brake_steer_left_lc2_b0.5=-inf; brake_steer_left_lc2_b1=-inf; staged_brake_0.4_change_lc1=-0.691 |

**Open-left control, repeatability.** The only witness is `bypass_left_lc2`; its margin over the original run and 3 repeats of the whole record: [0.0222, 0.0222, 0.0222, 0.0222] (range 0.0222 .. 0.0222; evidence classes ['R_REF', 'R_REF', 'R_REF', 'R_REF']). Under sigma = 0.0167: **robust**. 


### Brake fraction -> measured deceleration

Clear road (no actors), one ego run per level, from the CARLA speed traces. Deceleration is the mean slope of the ego speed between the first tick below 0.9 v0 and the first tick below 0.4 v0 (or the last tick if 0.4 v0 is not reached; the window stays above the end-of-stop wheel-lock transient below about 4.3 m/s); the commanded fraction is mapped by the plant's brake pedal and engine drag, not by a linear law. Linear expectation = f * 6.19 m/s^2.

| brake_frac | measured decel v0=14 (m/s^2) | measured decel v0=18 (m/s^2) | linear expectation | measured / linear (v0=14) | v at end of 8 s (v0=14) |
|---|---|---|---|---|---|
| 0.0 | 3.98 (to 5.6 m/s) | 4.71 (to 7.1 m/s) | 0.00 | - | 0.56 |
| 0.1 | 4.11 (to 5.5 m/s) | 4.87 (to 7.2 m/s) | 0.62 | 6.64 | 0.00 |
| 0.2 | 4.24 (to 5.4 m/s) | 5.02 (to 7.2 m/s) | 1.24 | 3.43 | 0.00 |
| 0.3 | 4.40 (to 5.6 m/s) | 5.15 (to 7.1 m/s) | 1.86 | 2.37 | -0.00 |
| 0.4 | 4.55 (to 5.5 m/s) | 5.29 (to 7.1 m/s) | 2.48 | 1.84 | 0.00 |
| 0.5 | 4.72 (to 5.5 m/s) | 5.43 (to 7.1 m/s) | 3.10 | 1.52 | -0.00 |
| 0.6 | 4.86 (to 5.5 m/s) | 5.63 (to 7.0 m/s) | 3.71 | 1.31 | 0.00 |
| 0.7 | 5.32 (to 5.1 m/s) | 5.78 (to 7.1 m/s) | 4.33 | 1.23 | -0.00 |
| 0.8 | 5.71 (to 4.8 m/s) | 5.93 (to 7.1 m/s) | 4.95 | 1.15 | -0.00 |
| 0.9 | 6.10 (to 4.5 m/s) | 6.08 (to 7.1 m/s) | 5.57 | 1.09 | -0.00 |
| 1.0 | 6.27 (to 4.6 m/s) | 6.25 (to 7.1 m/s) | 6.19 | 1.01 | 0.00 |

The mapping is nonlinear: the low levels are dominated by an engine-drag floor, so a small fraction does not give a small deceleration. The intermediate levels of the class are therefore not 'gentle braking' in the linear sense; they are measured members of the class as run.

## RC13: over-approximated prediction falsely closes a corridor

**Predictor** (`cases/negatives.py: Envelope, action_rejected`). It is a plain function of one delivered observation and one action; it is NOT part of the evidence pipeline (it never produces a margin or a label, and the harness does not call it). For each object `o` of the observation: constant-velocity prediction `x_o(t) = dx + (v_ego + dv) t`, no lateral motion; its box half-extents are `hl + lon_m + growth_mps * t` and `hw + lat_m + growth_mps * t`. The action's nominal ego footprint (ego half length from the observation, half width 0.95 m): speed held for tau = 0.3 s, then brake at b = 6.211 m/s^2 (`stay`: to a stop; `change_left`: until v < 8 m/s, then hold) with the quintic lateral move of 2 s over one lane width for `change_left`. The action is **rejected** if at any look-ahead step (horizon 4 s) the box separation (max of the longitudinal and lateral gaps; negative = overlap) between the ego footprint and an inflated box is below `clearance_m` = 0.

**Parameters.** E0: lat 0, lon 0, growth 0/s, dt 0.1 s; E1: lat 0.5, lon 0.5, growth 0/s, dt 0.1 s; E2: lat 1, lon 1, growth 0/s, dt 0.1 s; E3: lat 1.5, lon 1.5, growth 0/s, dt 0.1 s; E4: lat 2, lon 2, growth 0/s, dt 0.1 s; E5: lat 0.5, lon 0.5, growth 0.25/s, dt 0.1 s; E6: lat 0.5, lon 0.5, growth 0.5/s, dt 0.1 s; E7: lat 0, lon 0, growth 0/s, dt 0.5 s; E8: lat 1, lon 1, growth 0/s, dt 0.5 s; E9: lat 0, lon 0, growth 0/s, dt 0.1 s; E10: lat 0.5, lon 0.5, growth 0/s, dt 0.1 s. Horizon 4 s, clearance 0 for all.

**Coherent worlds** are the declared world set of the construction: `hid_left_lane` and `hid_own_lane` (one stopped vehicle, in the left lane or in the ego lane; the feed delivers it from t_reveal). The predictor sees only the first delivered observation containing the vehicle (recorded from the CARLA run of the witness `shared_slowdown_then_branch`), never the world name; a world the predictor 'imagines' (e.g. a vehicle in both lanes) is not in the set and plays no role.

Scenes (v, D bumper gap, t_reveal), all from the RC10 construction; the evidence is this task's own CARLA run:

| v | D | t_reveal | evidence | margins (hid_left_lane/hid_own_lane) | first-sighting observation (hid_own_lane): dx, dlat, ego v |
|---|---|---|---|---|---|
| 14 | 19 | 0.1 | R_REF | stop_in_place=0.105/-inf; fixed_left=-inf/0.081; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/0.049 | 21.85, 0.00, 14.01 |
| 14 | 20 | 0.2 | R_REF | stop_in_place=0.121/-0.101; fixed_left=-inf/0.093; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.121/0.050 | 21.45, 0.00, 14.00 |
| 18 | 26 | 0.2 | R_REF | stop_in_place=0.105/-inf; fixed_left=-inf/0.092; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/0.018 | 26.60, 0.00, 18.29 |
| 18 | 28 | 0.2 | R_REF | stop_in_place=0.105/-inf; fixed_left=-inf/0.135; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/0.105 | 28.60, 0.00, 18.29 |

Each cell below compares the predictor's decision with the evidence for the same action in the same world. Evidence 'ok' = G >= eps. `witness` cells: the action is the branch that `shared_slowdown_then_branch` takes in that world (the evidence is that policy's own margin, same state). `other action` cells: the counterfactual branch, with the evidence of `stop_in_place` / `fixed_left`, which are executed from t = 0, so they are only an approximate counterpart of the action judged from the first-sighting state. False rejection = evidence ok and predictor rejects; false acceptance = evidence refuted and predictor accepts.

### Scene v 14, D 19, t_reveal 0.1  (evidence R_REF)

| envelope | cells agreeing with evidence | false rejections | false acceptances | witness cells: min box separation (m), world A `stay` / world B `change_left` |
|---|---|---|---|---|
| E0 | 4/4 | none | none | 1.66 / 0.83 |
| E1 | 4/4 | none | none | 1.16 / 0.33 |
| E2 | 3/4 | [('hid_own_lane', 'change_left')] | none | 0.66 / -0.17 |
| E3 | 3/4 | [('hid_own_lane', 'change_left')] | none | 0.16 / -0.67 |
| E4 | 2/4 | [('hid_left_lane', 'stay'), ('hid_own_lane', 'change_left')] | none | -0.34 / -1.17 |
| E5 | 3/4 | [('hid_own_lane', 'change_left')] | none | 0.16 / -0.07 |
| E6 | 2/4 | [('hid_left_lane', 'stay'), ('hid_own_lane', 'change_left')] | none | -0.84 / -0.47 |
| E7 | 4/4 | none | none | 1.66 / 0.89 |
| E8 | 3/4 | [('hid_own_lane', 'change_left')] | none | 0.66 / -0.11 |
| E9 | 3/4 | [('hid_own_lane', 'change_left')] | none | 1.66 / 0.83 |
| E10 | 3/4 | [('hid_own_lane', 'change_left')] | none | 1.16 / 0.33 |

Smallest uniform inflation (lat = lon, no growth, dt 0.1 s, grid 0.05 m) that rejects the witness branch: {'hid_left_lane': 1.7, 'hid_own_lane': 0.85}.
Audited witness trace in `hid_own_lane` (r = 0.049): termination horizon, contact events none, ego v 14.01 -> 6.72, lateral range [-3.84, -0.00], min clearance (m) {'hid': 1.25}; commands (decision -> effective): brake/None 0.00->0.25, brake/-3.5 0.10->0.35, keep/-3.5 1.35->1.60.

### Scene v 14, D 20, t_reveal 0.2  (evidence R_REF)

| envelope | cells agreeing with evidence | false rejections | false acceptances | witness cells: min box separation (m), world A `stay` / world B `change_left` |
|---|---|---|---|---|
| E0 | 4/4 | none | none | 1.66 / 0.54 |
| E1 | 4/4 | none | none | 1.16 / 0.04 |
| E2 | 3/4 | [('hid_own_lane', 'change_left')] | none | 0.66 / -0.46 |
| E3 | 3/4 | [('hid_own_lane', 'change_left')] | none | 0.16 / -0.96 |
| E4 | 2/4 | [('hid_left_lane', 'stay'), ('hid_own_lane', 'change_left')] | none | -0.34 / -1.46 |
| E5 | 3/4 | [('hid_own_lane', 'change_left')] | none | 0.16 / -0.33 |
| E6 | 2/4 | [('hid_left_lane', 'stay'), ('hid_own_lane', 'change_left')] | none | -0.84 / -0.71 |
| E7 | 4/4 | none | none | 1.66 / 0.54 |
| E8 | 3/4 | [('hid_own_lane', 'change_left')] | none | 0.66 / -0.46 |
| E9 | 3/4 | [('hid_own_lane', 'change_left')] | none | 1.66 / 0.54 |
| E10 | 3/4 | [('hid_own_lane', 'change_left')] | none | 1.16 / 0.04 |

Smallest uniform inflation (lat = lon, no growth, dt 0.1 s, grid 0.05 m) that rejects the witness branch: {'hid_left_lane': 1.7, 'hid_own_lane': 0.55}.
Audited witness trace in `hid_own_lane` (r = 0.050): termination horizon, contact events none, ego v 14.01 -> 6.71, lateral range [-3.85, -0.00], min clearance (m) {'hid': 1.36}; commands (decision -> effective): brake/None 0.00->0.25, brake/-3.5 0.20->0.45, keep/-3.5 1.35->1.60.

### Scene v 18, D 26, t_reveal 0.2  (evidence R_REF)

| envelope | cells agreeing with evidence | false rejections | false acceptances | witness cells: min box separation (m), world A `stay` / world B `change_left` |
|---|---|---|---|---|
| E0 | 4/4 | none | none | 1.66 / 0.51 |
| E1 | 4/4 | none | none | 1.16 / 0.01 |
| E2 | 3/4 | [('hid_own_lane', 'change_left')] | none | 0.66 / -0.49 |
| E3 | 3/4 | [('hid_own_lane', 'change_left')] | none | 0.16 / -0.99 |
| E4 | 2/4 | [('hid_left_lane', 'stay'), ('hid_own_lane', 'change_left')] | none | -0.34 / -1.49 |
| E5 | 3/4 | [('hid_own_lane', 'change_left')] | none | 0.16 / -0.34 |
| E6 | 2/4 | [('hid_left_lane', 'stay'), ('hid_own_lane', 'change_left')] | none | -0.84 / -0.71 |
| E7 | 4/4 | none | none | 1.66 / 0.54 |
| E8 | 3/4 | [('hid_own_lane', 'change_left')] | none | 0.66 / -0.46 |
| E9 | 3/4 | [('hid_own_lane', 'change_left')] | none | 1.66 / 0.51 |
| E10 | 3/4 | [('hid_own_lane', 'change_left')] | none | 1.16 / 0.01 |

Smallest uniform inflation (lat = lon, no growth, dt 0.1 s, grid 0.05 m) that rejects the witness branch: {'hid_left_lane': 1.7, 'hid_own_lane': 0.55}.
Audited witness trace in `hid_own_lane` (r = 0.018): termination horizon, contact events none, ego v 18.18 -> 6.52, lateral range [-3.76, -0.00], min clearance (m) {'hid': 1.09}; commands (decision -> effective): brake/None 0.00->0.25, brake/-3.5 0.20->0.45, keep/-3.5 1.95->2.20.

### Scene v 18, D 28, t_reveal 0.2  (evidence R_REF)

| envelope | cells agreeing with evidence | false rejections | false acceptances | witness cells: min box separation (m), world A `stay` / world B `change_left` |
|---|---|---|---|---|
| E0 | 4/4 | none | none | 1.66 / 0.83 |
| E1 | 4/4 | none | none | 1.16 / 0.33 |
| E2 | 3/4 | [('hid_own_lane', 'change_left')] | none | 0.66 / -0.17 |
| E3 | 3/4 | [('hid_own_lane', 'change_left')] | none | 0.16 / -0.67 |
| E4 | 2/4 | [('hid_left_lane', 'stay'), ('hid_own_lane', 'change_left')] | none | -0.34 / -1.17 |
| E5 | 3/4 | [('hid_own_lane', 'change_left')] | none | 0.16 / -0.07 |
| E6 | 2/4 | [('hid_left_lane', 'stay'), ('hid_own_lane', 'change_left')] | none | -0.84 / -0.47 |
| E7 | 4/4 | none | none | 1.66 / 1.39 |
| E8 | 4/4 | none | none | 0.66 / 0.39 |
| E9 | 3/4 | [('hid_own_lane', 'change_left')] | none | 1.66 / 0.83 |
| E10 | 3/4 | [('hid_own_lane', 'change_left')] | none | 1.16 / 0.33 |

Smallest uniform inflation (lat = lon, no growth, dt 0.1 s, grid 0.05 m) that rejects the witness branch: {'hid_left_lane': 1.7, 'hid_own_lane': 0.85}.
Audited witness trace in `hid_own_lane` (r = 0.105): termination horizon, contact events none, ego v 18.18 -> 6.52, lateral range [-3.76, -0.00], min clearance (m) {'hid': 1.53}; commands (decision -> effective): brake/None 0.00->0.25, brake/-3.5 0.20->0.45, keep/-3.5 1.95->2.20.

### Repeatability and headline

Each RC13 scene was run 3 more times (whole library, same construction). A scene is **marginal** if any witness margin (either world) over the original run and the repeats is within sigma = 0.0167 of eps, or if a sign flips; marginal scenes are excluded from the headline count.

| scene (v, D, t_reveal) | evidence classes (orig + repeats) | witness G world A: range | witness G world B: range | status |
|---|---|---|---|---|
| (14.0, 19.0, 0.1) | ['R_REF', 'R_REF', 'R_REF', 'R_REF'] | 0.1047 .. 0.1047 | 0.0490 .. 0.0491 | robust |
| (14.0, 20.0, 0.2) | ['R_REF', 'R_REF', 'R_REF', 'R_REF'] | 0.1206 .. 0.1206 | 0.0224 .. 0.0504 | robust |
| (18.0, 26.0, 0.2) | ['R_REF', 'R_REF', 'R_REF', 'R_REF'] | 0.1046 .. 0.1047 | 0.0182 .. 0.0182 | robust |
| (18.0, 28.0, 0.2) | ['R_REF', 'R_REF', 'R_REF', 'R_REF'] | 0.1046 .. 0.1046 | 0.1051 .. 0.1062 | robust |

**Headline.** Robust scenes (counted): 4 of 4: [(14.0, 19.0, 0.1), (14.0, 20.0, 0.2), (18.0, 26.0, 0.2), (18.0, 28.0, 0.2)]. Marginal scenes (excluded from the count): none.

The rule is applied mechanically. Smallest witness margin over all runs and worlds: 0.0182 in scene (18.0, 26.0, 0.2) = 1.09 x sigma; it is not within sigma of eps, so the scene is not marked marginal, but it is within about 10% of the threshold. The repeats differ by much less than sigma except where the ranges above show otherwise, so sigma is a conservative bound here.

### Predictor summary (robust scenes)

| scene (v, D, t_reveal) | E0 (exact CV, clearance 0): agreement | E0 rejects the witness | smallest rejecting uniform inflation per world (m) | E0 box separation, witness world B (m) | audited actual clearance to hid, witness world B (m) |
|---|---|---|---|---|---|
| (14.0, 19.0, 0.1) | 4/4 | False | {'hid_left_lane': 1.7, 'hid_own_lane': 0.85} | 0.83 | 1.25 |
| (14.0, 20.0, 0.2) | 4/4 | False | {'hid_left_lane': 1.7, 'hid_own_lane': 0.55} | 0.54 | 1.36 |
| (18.0, 26.0, 0.2) | 4/4 | False | {'hid_left_lane': 1.7, 'hid_own_lane': 0.55} | 0.51 | 1.09 |
| (18.0, 28.0, 0.2) | 4/4 | False | {'hid_left_lane': 1.7, 'hid_own_lane': 0.85} | 0.83 | 1.53 |

### Metric mismatch between the predictor and the evidence

The predictor uses axis-aligned box separation (max of the longitudinal and lateral gaps) with `clearance_m = 0`, evaluated on a NOMINAL ego path (declared b, tau, quintic lane change). The evidence uses the oriented-box corner clearance of the executed trajectory with a 1 m reserve inside the margin. They are different quantities, so the numbers are not directly comparable: in the robust scenes the exact envelope's box separation for the witness branch is 0.51-0.83 m while the audited actual clearances are 1.09-1.53 m; the difference (0.41 to 0.82 m) is the nominal-model error plus the box-versus-corner metric difference, and it is not separated further here. The smallest rejecting inflation of 0.55-0.85 m is below the evidence's own 1 m reserve: the over-approximation that flips the decision is smaller than the safety reserve the evidence itself demands, so part of what looks like a 'false' rejection is a different safety convention rather than only over-approximation.

Rows E9 (exact envelope, `clearance_m` = 1.0) and E10 (0.5 m inflation, `clearance_m` = 1.0) test the exact envelope against the evidence's own reserve:

| scene | E9 agreement | E9 false rejections | E10 false rejections |
|---|---|---|---|
| (14.0, 19.0, 0.1) | 3/4 | [('hid_own_lane', 'change_left')] | [('hid_own_lane', 'change_left')] |
| (14.0, 20.0, 0.2) | 3/4 | [('hid_own_lane', 'change_left')] | [('hid_own_lane', 'change_left')] |
| (18.0, 26.0, 0.2) | 3/4 | [('hid_own_lane', 'change_left')] | [('hid_own_lane', 'change_left')] |
| (18.0, 28.0, 0.2) | 3/4 | [('hid_own_lane', 'change_left')] | [('hid_own_lane', 'change_left')] |

The exact envelope held to the evidence's own 1 m reserve (E9) DISAGREES with the evidence (it rejects the supported witness) in 4 of 4 robust scenes.

**Reading (scoped).** On the declared worlds, for the robust scenes, the witness is supported by the evidence (r >= eps in both worlds in every repeat, executed without contact) and the exact constant-velocity envelope with zero clearance accepts it. A predictor that adds a constant inflation of about 0.55-0.85 m, or a growth term, to the same envelope rejects the witness branch. That shows how a simple over-approximating predictor can close a corridor the evidence supports; it does not show that any particular real predictor does so, and the thresholds depend on the metric mismatch above (box separation of a nominal path versus corner clearance with reserve of the executed path). Marginal scenes are not part of this statement.

## Record validation

28 records in `results/carla/negatives/` checked with validate_records.check_record: 0 failures.
