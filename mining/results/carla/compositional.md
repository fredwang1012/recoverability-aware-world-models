# CARLA compositional cases (Task 6: RC04, RC05, RC06, RC08, RC15, RC18)

Physics profile `pp_381b4c6e55`, decel target 6.19, eps = 0.0, reserve c = 1.0 m (margin.DEFAULT_V2, unchanged), act delay 0.25 s. Records: `results/carla/compositional/*.jsonl`. Labels are `contract.classify` output only. G cells below read `world1/world2` (-inf = hard failure, None = unrun).

**Authority.** Every spec declares `a_lat_max = 8` m/s^2 (dynamic-limit margin from |v * yaw_rate|, as in the core cases); RC18 additionally declares `a_comb_max = 8` m/s^2, a friction-circle margin sqrt(a_long^2 + a_lat^2) <= limit from MEASURED accelerations (a_long = tick difference of ego speed, centred 5-tick moving average = 0.25 s, declared before any run; a_lat = |v * yaw_rate| per tick, unsmoothed; ticks with ego speed < 5 m/s are excluded from a_comb because CARLA's brake model locks the wheels at the end of a stop, measured -28..-35 m/s^2 for ~4 ticks below ~4.3 m/s in a brake_hold diagnostic at v 18; the unmasked peak is still recorded as `a_comb_unmasked`). **Disclosure:** CARLA's tyre friction in this profile is non-physical (`friction_fixed` 5.0, braking set through brake torque), so CARLA's own tyre limit is not the authority; the declared 8.0 m/s^2 limits are the physical-plausibility authority and enter only as margins (they make a manoeuvre a non-witness, never a hand-written label).

Selection rules were declared in the script header before the run; every scan row is reported below.

## Per-case table

| case | scanned scenes | scenes with the declared contrast | selected scene | label (evidence class) | witness policy | refuted policies in selected scene |
|---|---|---|---|---|---|---|
| RC04 | 92 | 0 | NONE | - | no positive witness with the contrast | - |
| RC05 | 12 | 12 | (80.0, 20.0, 20.0) | R_REF | abort_and_stabilize | commit_lane_change (-inf/0.105) |
| RC06 | 12 | 5 | (60.0, 10.0, 20.0) | R_REF | wait_observe_enter | immediate_entry (-inf), lane_follow (-inf) |
| RC15 | 12 | 9 | (60.0, 35.0, 16.0) | R_REF | observe_yield_else_fallback | always_merge (0.083/-inf), lane_follow (-inf/-inf) |
| RC08 (open lane) | 1 | - | gap 45, v 14 | R_REF | brake_hold, bypass_left | lane_follow (-inf) |
| RC08 (closed lane) | 1 | - | gap 45, v 14 | R_REF | brake_hold | bypass_left (-inf), lane_follow (-inf) |
| RC18 | 12 | 5 | (22.0, 42) | R_REF | bypass_left | brake_steer_cmd_lc2_b1 refuted by a_comb |

## RC04: brake then change lanes

Library (per lc_time L, L in 1.6 and 2.0): brake_hold, lane_follow, immediate_lane_change (Bypass, L), staged_brake_0.4_change and staged_brake_0.8_change (StagedBrakeChange, L). Immediate lane change is a different candidate at each L; scenes where the immediate change at lc 2.0 already passes are therefore not witnesses of the staged policy in the lc-2.0 library.

| grid | lc | v | gap | evidence | contrast | margins |
|---|---|---|---|---|---|---|
| base | 1.6 | 14 | 18 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-0.065; staged_brake_0.4_change=-inf; staged_brake_0.8_change=-inf |
| base | 1.6 | 14 | 20 | U_LIBRARY | False | brake_hold=-0.101; lane_follow=-inf; immediate_lane_change=-0.058; staged_brake_0.4_change=-0.143; staged_brake_0.8_change=-inf |
| base | 1.6 | 14 | 22 | R_REF | False | brake_hold=0.299; lane_follow=-inf; immediate_lane_change=-0.058; staged_brake_0.4_change=-0.005; staged_brake_0.8_change=-0.143 |
| base | 1.6 | 14 | 24 | R_REF | False | brake_hold=0.667; lane_follow=-inf; immediate_lane_change=-0.058; staged_brake_0.4_change=0.038; staged_brake_0.8_change=0.025 |
| base | 1.6 | 14 | 26 | R_REF | False | brake_hold=0.667; lane_follow=-inf; immediate_lane_change=-0.058; staged_brake_0.4_change=0.038; staged_brake_0.8_change=0.039 |
| base | 1.6 | 14 | 28 | R_REF | False | brake_hold=0.667; lane_follow=-inf; immediate_lane_change=-0.058; staged_brake_0.4_change=0.038; staged_brake_0.8_change=0.039 |
| base | 1.6 | 14 | 30 | R_REF | False | brake_hold=0.667; lane_follow=-inf; immediate_lane_change=-0.058; staged_brake_0.4_change=0.038; staged_brake_0.8_change=0.039 |
| base | 1.6 | 14 | 32 | R_REF | False | brake_hold=0.667; lane_follow=-inf; immediate_lane_change=-0.058; staged_brake_0.4_change=0.038; staged_brake_0.8_change=0.039 |
| base | 1.6 | 14 | 34 | R_REF | False | brake_hold=0.667; lane_follow=-inf; immediate_lane_change=-0.058; staged_brake_0.4_change=0.038; staged_brake_0.8_change=0.039 |
| base | 1.6 | 14 | 36 | R_REF | False | brake_hold=0.667; lane_follow=-inf; immediate_lane_change=-0.058; staged_brake_0.4_change=0.038; staged_brake_0.8_change=0.039 |
| base | 1.6 | 14 | 38 | R_REF | False | brake_hold=0.667; lane_follow=-inf; immediate_lane_change=-0.058; staged_brake_0.4_change=0.038; staged_brake_0.8_change=0.039 |
| base | 1.6 | 18 | 18 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-inf; staged_brake_0.4_change=-inf; staged_brake_0.8_change=-inf |
| base | 1.6 | 18 | 20 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-inf; staged_brake_0.4_change=-inf; staged_brake_0.8_change=-inf |
| base | 1.6 | 18 | 22 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-0.248; staged_brake_0.4_change=-inf; staged_brake_0.8_change=-inf |
| base | 1.6 | 18 | 24 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-0.248; staged_brake_0.4_change=-inf; staged_brake_0.8_change=-inf |
| base | 1.6 | 18 | 26 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-0.248; staged_brake_0.4_change=-inf; staged_brake_0.8_change=-inf |
| base | 1.6 | 18 | 28 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-0.248; staged_brake_0.4_change=-0.167; staged_brake_0.8_change=-inf |
| base | 1.6 | 18 | 30 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-0.248; staged_brake_0.4_change=-0.167; staged_brake_0.8_change=-0.128 |
| base | 1.6 | 18 | 32 | R_REF | False | brake_hold=0.134; lane_follow=-inf; immediate_lane_change=-0.248; staged_brake_0.4_change=-0.167; staged_brake_0.8_change=-0.021 |
| base | 1.6 | 18 | 34 | R_REF | False | brake_hold=0.534; lane_follow=-inf; immediate_lane_change=-0.248; staged_brake_0.4_change=-0.167; staged_brake_0.8_change=-0.021 |
| base | 1.6 | 18 | 36 | R_REF | False | brake_hold=0.667; lane_follow=-inf; immediate_lane_change=-0.248; staged_brake_0.4_change=-0.167; staged_brake_0.8_change=-0.021 |
| base | 1.6 | 18 | 38 | R_REF | False | brake_hold=0.668; lane_follow=-inf; immediate_lane_change=-0.248; staged_brake_0.4_change=-0.167; staged_brake_0.8_change=-0.021 |
| base | 2 | 14 | 18 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-inf; staged_brake_0.4_change=-inf; staged_brake_0.8_change=-inf |
| base | 2 | 14 | 20 | U_LIBRARY | False | brake_hold=-0.101; lane_follow=-inf; immediate_lane_change=-0.121; staged_brake_0.4_change=-inf; staged_brake_0.8_change=-inf |
| base | 2 | 14 | 22 | R_REF | False | brake_hold=0.299; lane_follow=-inf; immediate_lane_change=-0.023; staged_brake_0.4_change=-0.172; staged_brake_0.8_change=-inf |
| base | 2 | 14 | 24 | R_REF | False | brake_hold=0.667; lane_follow=-inf; immediate_lane_change=0.063; staged_brake_0.4_change=-0.053; staged_brake_0.8_change=-0.139 |
| base | 2 | 14 | 26 | R_REF | False | brake_hold=0.667; lane_follow=-inf; immediate_lane_change=0.130; staged_brake_0.4_change=0.049; staged_brake_0.8_change=0.004 |
| base | 2 | 14 | 28 | R_REF | False | brake_hold=0.667; lane_follow=-inf; immediate_lane_change=0.158; staged_brake_0.4_change=0.126; staged_brake_0.8_change=0.078 |
| base | 2 | 14 | 30 | R_REF | False | brake_hold=0.667; lane_follow=-inf; immediate_lane_change=0.158; staged_brake_0.4_change=0.130; staged_brake_0.8_change=0.078 |
| base | 2 | 14 | 32 | R_REF | False | brake_hold=0.667; lane_follow=-inf; immediate_lane_change=0.158; staged_brake_0.4_change=0.130; staged_brake_0.8_change=0.078 |
| base | 2 | 14 | 34 | R_REF | False | brake_hold=0.667; lane_follow=-inf; immediate_lane_change=0.158; staged_brake_0.4_change=0.130; staged_brake_0.8_change=0.078 |
| base | 2 | 14 | 36 | R_REF | False | brake_hold=0.667; lane_follow=-inf; immediate_lane_change=0.151; staged_brake_0.4_change=0.130; staged_brake_0.8_change=0.078 |
| base | 2 | 14 | 38 | R_REF | False | brake_hold=0.667; lane_follow=-inf; immediate_lane_change=0.141; staged_brake_0.4_change=0.130; staged_brake_0.8_change=0.078 |
| base | 2 | 18 | 18 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-inf; staged_brake_0.4_change=-inf; staged_brake_0.8_change=-inf |
| base | 2 | 18 | 20 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-inf; staged_brake_0.4_change=-inf; staged_brake_0.8_change=-inf |
| base | 2 | 18 | 22 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-inf; staged_brake_0.4_change=-inf; staged_brake_0.8_change=-inf |
| base | 2 | 18 | 24 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-inf; staged_brake_0.4_change=-inf; staged_brake_0.8_change=-inf |
| base | 2 | 18 | 26 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-0.149; staged_brake_0.4_change=-inf; staged_brake_0.8_change=-inf |
| base | 2 | 18 | 28 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-0.068; staged_brake_0.4_change=-inf; staged_brake_0.8_change=-inf |
| base | 2 | 18 | 30 | R_REF | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=0.008; staged_brake_0.4_change=-0.163; staged_brake_0.8_change=-inf |
| base | 2 | 18 | 32 | R_REF | False | brake_hold=0.134; lane_follow=-inf; immediate_lane_change=0.074; staged_brake_0.4_change=-0.073; staged_brake_0.8_change=-0.171 |
| base | 2 | 18 | 34 | R_REF | False | brake_hold=0.534; lane_follow=-inf; immediate_lane_change=0.119; staged_brake_0.4_change=0.010; staged_brake_0.8_change=-0.062 |
| base | 2 | 18 | 36 | R_REF | False | brake_hold=0.667; lane_follow=-inf; immediate_lane_change=0.118; staged_brake_0.4_change=0.082; staged_brake_0.8_change=0.034 |
| base | 2 | 18 | 38 | R_REF | False | brake_hold=0.667; lane_follow=-inf; immediate_lane_change=0.119; staged_brake_0.4_change=0.137; staged_brake_0.8_change=0.110 |
| ext | 1.6 | 16 | 16 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-inf; staged_brake_0.2_change=-inf; staged_brake_0.4_change=-inf; staged_brake_0.6_change=-inf; staged_brake_0.8_change=-inf; staged_brake_1_change=-inf |
| ext | 1.6 | 16 | 18 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-inf; staged_brake_0.2_change=-inf; staged_brake_0.4_change=-inf; staged_brake_0.6_change=-inf; staged_brake_0.8_change=-inf; staged_brake_1_change=-inf |
| ext | 1.6 | 16 | 20 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-0.158; staged_brake_0.2_change=-inf; staged_brake_0.4_change=-inf; staged_brake_0.6_change=-inf; staged_brake_0.8_change=-inf; staged_brake_1_change=-inf |
| ext | 1.6 | 16 | 22 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-0.158; staged_brake_0.2_change=-0.142; staged_brake_0.4_change=-inf; staged_brake_0.6_change=-inf; staged_brake_0.8_change=-inf; staged_brake_1_change=-inf |
| ext | 1.6 | 16 | 24 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-0.158; staged_brake_0.2_change=-0.136; staged_brake_0.4_change=-0.108; staged_brake_0.6_change=-0.194; staged_brake_0.8_change=-inf; staged_brake_1_change=-inf |
| ext | 1.6 | 16 | 26 | R_REF | False | brake_hold=0.064; lane_follow=-inf; immediate_lane_change=-0.158; staged_brake_0.2_change=-0.134; staged_brake_0.4_change=-0.068; staged_brake_0.6_change=-0.053; staged_brake_0.8_change=-0.129; staged_brake_1_change=-inf |
| ext | 1.6 | 18 | 22 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-0.248; staged_brake_0.2_change=-inf; staged_brake_0.4_change=-inf; staged_brake_0.6_change=-inf; staged_brake_0.8_change=-inf; staged_brake_1_change=-inf |
| ext | 1.6 | 18 | 24 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-0.248; staged_brake_0.2_change=-inf; staged_brake_0.4_change=-inf; staged_brake_0.6_change=-inf; staged_brake_0.8_change=-inf; staged_brake_1_change=-inf |
| ext | 1.6 | 18 | 26 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-0.248; staged_brake_0.2_change=-0.231; staged_brake_0.4_change=-inf; staged_brake_0.6_change=-inf; staged_brake_0.8_change=-inf; staged_brake_1_change=-inf |
| ext | 1.6 | 18 | 28 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-0.248; staged_brake_0.2_change=-0.231; staged_brake_0.4_change=-0.167; staged_brake_0.6_change=-0.175; staged_brake_0.8_change=-inf; staged_brake_1_change=-inf |
| ext | 1.6 | 18 | 30 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-0.248; staged_brake_0.2_change=-0.231; staged_brake_0.4_change=-0.167; staged_brake_0.6_change=-0.096; staged_brake_0.8_change=-0.128; staged_brake_1_change=-inf |
| ext | 1.6 | 18 | 32 | R_REF | False | brake_hold=0.134; lane_follow=-inf; immediate_lane_change=-0.248; staged_brake_0.2_change=-0.231; staged_brake_0.4_change=-0.167; staged_brake_0.6_change=-0.096; staged_brake_0.8_change=-0.021; staged_brake_1_change=-0.062 |
| ext | 1.6 | 20 | 29 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-0.327; staged_brake_0.2_change=-0.315; staged_brake_0.4_change=-inf; staged_brake_0.6_change=-inf; staged_brake_0.8_change=-inf; staged_brake_1_change=-inf |
| ext | 1.6 | 20 | 31 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-0.327; staged_brake_0.2_change=-0.315; staged_brake_0.4_change=-0.253; staged_brake_0.6_change=-inf; staged_brake_0.8_change=-inf; staged_brake_1_change=-inf |
| ext | 1.6 | 20 | 33 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-0.327; staged_brake_0.2_change=-0.315; staged_brake_0.4_change=-0.253; staged_brake_0.6_change=-0.185; staged_brake_0.8_change=-0.197; staged_brake_1_change=-inf |
| ext | 1.6 | 20 | 35 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-0.327; staged_brake_0.2_change=-0.315; staged_brake_0.4_change=-0.253; staged_brake_0.6_change=-0.185; staged_brake_0.8_change=-0.116; staged_brake_1_change=-0.156 |
| ext | 1.6 | 20 | 37 | R_REF | False | brake_hold=0.030; lane_follow=-inf; immediate_lane_change=-0.327; staged_brake_0.2_change=-0.315; staged_brake_0.4_change=-0.253; staged_brake_0.6_change=-0.185; staged_brake_0.8_change=-0.116; staged_brake_1_change=-0.039 |
| ext | 1.6 | 20 | 39 | R_REF | False | brake_hold=0.406; lane_follow=-inf; immediate_lane_change=-0.327; staged_brake_0.2_change=-0.315; staged_brake_0.4_change=-0.253; staged_brake_0.6_change=-0.185; staged_brake_0.8_change=-0.116; staged_brake_1_change=-0.039 |
| ext | 1.6 | 22 | 36 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-0.394; staged_brake_0.2_change=-0.379; staged_brake_0.4_change=-0.322; staged_brake_0.6_change=-0.265; staged_brake_0.8_change=-inf; staged_brake_1_change=-inf |
| ext | 1.6 | 22 | 38 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-0.394; staged_brake_0.2_change=-0.379; staged_brake_0.4_change=-0.322; staged_brake_0.6_change=-0.265; staged_brake_0.8_change=-0.198; staged_brake_1_change=-inf |
| ext | 1.6 | 22 | 40 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-0.394; staged_brake_0.2_change=-0.379; staged_brake_0.4_change=-0.322; staged_brake_0.6_change=-0.265; staged_brake_0.8_change=-0.198; staged_brake_1_change=-0.130 |
| ext | 1.6 | 22 | 42 | U_LIBRARY | False | brake_hold=-0.174; lane_follow=-inf; immediate_lane_change=-0.394; staged_brake_0.2_change=-0.379; staged_brake_0.4_change=-0.322; staged_brake_0.6_change=-0.265; staged_brake_0.8_change=-0.198; staged_brake_1_change=-0.130 |
| ext | 1.6 | 22 | 44 | R_REF | False | brake_hold=0.226; lane_follow=-inf; immediate_lane_change=-0.394; staged_brake_0.2_change=-0.379; staged_brake_0.4_change=-0.322; staged_brake_0.6_change=-0.265; staged_brake_0.8_change=-0.198; staged_brake_1_change=-0.130 |
| ext | 1.6 | 22 | 46 | R_REF | False | brake_hold=0.626; lane_follow=-inf; immediate_lane_change=-0.394; staged_brake_0.2_change=-0.379; staged_brake_0.4_change=-0.322; staged_brake_0.6_change=-0.265; staged_brake_0.8_change=-0.198; staged_brake_1_change=-0.130 |
| ext | 2 | 16 | 16 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-inf; staged_brake_0.2_change=-inf; staged_brake_0.4_change=-inf; staged_brake_0.6_change=-inf; staged_brake_0.8_change=-inf; staged_brake_1_change=-inf |
| ext | 2 | 16 | 18 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-inf; staged_brake_0.2_change=-inf; staged_brake_0.4_change=-inf; staged_brake_0.6_change=-inf; staged_brake_0.8_change=-inf; staged_brake_1_change=-inf |
| ext | 2 | 16 | 20 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-inf; staged_brake_0.2_change=-inf; staged_brake_0.4_change=-inf; staged_brake_0.6_change=-inf; staged_brake_0.8_change=-inf; staged_brake_1_change=-inf |
| ext | 2 | 16 | 22 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-0.182; staged_brake_0.2_change=-inf; staged_brake_0.4_change=-inf; staged_brake_0.6_change=-inf; staged_brake_0.8_change=-inf; staged_brake_1_change=-inf |
| ext | 2 | 16 | 24 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-0.090; staged_brake_0.2_change=-0.198; staged_brake_0.4_change=-inf; staged_brake_0.6_change=-inf; staged_brake_0.8_change=-inf; staged_brake_1_change=-inf |
| ext | 2 | 16 | 26 | R_REF | False | brake_hold=0.064; lane_follow=-inf; immediate_lane_change=-0.004; staged_brake_0.2_change=-0.103; staged_brake_0.4_change=-0.161; staged_brake_0.6_change=-inf; staged_brake_0.8_change=-inf; staged_brake_1_change=-inf |
| ext | 2 | 18 | 22 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-inf; staged_brake_0.2_change=-inf; staged_brake_0.4_change=-inf; staged_brake_0.6_change=-inf; staged_brake_0.8_change=-inf; staged_brake_1_change=-inf |
| ext | 2 | 18 | 24 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-inf; staged_brake_0.2_change=-inf; staged_brake_0.4_change=-inf; staged_brake_0.6_change=-inf; staged_brake_0.8_change=-inf; staged_brake_1_change=-inf |
| ext | 2 | 18 | 26 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-0.149; staged_brake_0.2_change=-inf; staged_brake_0.4_change=-inf; staged_brake_0.6_change=-inf; staged_brake_0.8_change=-inf; staged_brake_1_change=-inf |
| ext | 2 | 18 | 28 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-0.068; staged_brake_0.2_change=-0.186; staged_brake_0.4_change=-inf; staged_brake_0.6_change=-inf; staged_brake_0.8_change=-inf; staged_brake_1_change=-inf |
| ext | 2 | 18 | 30 | R_REF | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=0.008; staged_brake_0.2_change=-0.101; staged_brake_0.4_change=-0.163; staged_brake_0.6_change=-inf; staged_brake_0.8_change=-inf; staged_brake_1_change=-inf |
| ext | 2 | 18 | 32 | R_REF | False | brake_hold=0.134; lane_follow=-inf; immediate_lane_change=0.074; staged_brake_0.2_change=-0.020; staged_brake_0.4_change=-0.073; staged_brake_0.6_change=-0.121; staged_brake_0.8_change=-0.168; staged_brake_1_change=-inf |
| ext | 2 | 20 | 29 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-0.161; staged_brake_0.2_change=-inf; staged_brake_0.4_change=-inf; staged_brake_0.6_change=-inf; staged_brake_0.8_change=-inf; staged_brake_1_change=-inf |
| ext | 2 | 20 | 31 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-0.086; staged_brake_0.2_change=-inf; staged_brake_0.4_change=-inf; staged_brake_0.6_change=-inf; staged_brake_0.8_change=-inf; staged_brake_1_change=-inf |
| ext | 2 | 20 | 33 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-0.015; staged_brake_0.2_change=-0.138; staged_brake_0.4_change=-inf; staged_brake_0.6_change=-inf; staged_brake_0.8_change=-inf; staged_brake_1_change=-inf |
| ext | 2 | 20 | 35 | R_REF | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=0.049; staged_brake_0.2_change=-0.063; staged_brake_0.4_change=-0.126; staged_brake_0.6_change=-0.184; staged_brake_0.8_change=-inf; staged_brake_1_change=-inf |
| ext | 2 | 20 | 37 | R_REF | False | brake_hold=0.030; lane_follow=-inf; immediate_lane_change=0.091; staged_brake_0.2_change=0.006; staged_brake_0.4_change=-0.046; staged_brake_0.6_change=-0.095; staged_brake_0.8_change=-0.144; staged_brake_1_change=-0.196 |
| ext | 2 | 20 | 39 | R_REF | False | brake_hold=0.430; lane_follow=-inf; immediate_lane_change=0.091; staged_brake_0.2_change=0.068; staged_brake_0.4_change=0.026; staged_brake_0.6_change=-0.011; staged_brake_0.8_change=-0.049; staged_brake_1_change=-0.089 |
| ext | 2 | 22 | 36 | U_LIBRARY | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=-0.037; staged_brake_0.2_change=-0.172; staged_brake_0.4_change=-inf; staged_brake_0.6_change=-inf; staged_brake_0.8_change=-inf; staged_brake_1_change=-inf |
| ext | 2 | 22 | 38 | R_REF | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=0.024; staged_brake_0.2_change=-0.103; staged_brake_0.4_change=-0.173; staged_brake_0.6_change=-inf; staged_brake_0.8_change=-inf; staged_brake_1_change=-inf |
| ext | 2 | 22 | 40 | R_REF | False | brake_hold=-inf; lane_follow=-inf; immediate_lane_change=0.068; staged_brake_0.2_change=-0.037; staged_brake_0.4_change=-0.100; staged_brake_0.6_change=-0.159; staged_brake_0.8_change=-inf; staged_brake_1_change=-inf |
| ext | 2 | 22 | 42 | R_REF | False | brake_hold=-0.159; lane_follow=-inf; immediate_lane_change=0.068; staged_brake_0.2_change=0.023; staged_brake_0.4_change=-0.030; staged_brake_0.6_change=-0.079; staged_brake_0.8_change=-0.131; staged_brake_1_change=-0.184 |
| ext | 2 | 22 | 44 | R_REF | False | brake_hold=0.226; lane_follow=-inf; immediate_lane_change=0.068; staged_brake_0.2_change=0.068; staged_brake_0.4_change=0.034; staged_brake_0.6_change=-0.006; staged_brake_0.8_change=-0.046; staged_brake_1_change=-0.089 |
| ext | 2 | 22 | 46 | R_REF | False | brake_hold=0.626; lane_follow=-inf; immediate_lane_change=0.068; staged_brake_0.2_change=0.068; staged_brake_0.4_change=0.086; staged_brake_0.6_change=0.061; staged_brake_0.8_change=0.031; staged_brake_1_change=-0.001 |

No scene shows the contrast (a staged policy passes while brake_hold and the immediate change both fail): no positive RC04 witness on CARLA under the 8.0 m/s^2 limit in the searched grid.  Searched: base grid lc {1.6, 2.0} x v {14, 18} x gap 18-38 step 2 (ps 0.4, 0.8); extended grid lc {1.6, 2.0} x v {16, 18, 20, 22} x gap [stop_dist-10, stop_dist] step 2 (ps 0.2-1.0).  Nothing is forced.
Scenes where a staged policy passes and brake_hold fails (the only candidates): ('ext', 2.0, 22.0, 42): immediate_lane_change=0.068.  Where such a scene exists the immediate lane change passes as well, so the staged policy is not the unique supported continuation; elsewhere brake_hold fails and every staged policy fails too (braking first spends the distance the lane change needs).

## RC05: abort an unsafe lane change

Every policy first executes a physical left lane change (prefix 1.0 s, lc_time 3.0) inside the episode; the rear vehicle in the target lane is hidden until t = 1.25 s. Worlds (mode fixed per world): `gap_closes` (rear cruises) and `gap_open` (rear brakes at 5 m/s^2 from t = 0). Library: commit_lane_change, abort_and_stabilize (steer back, settle 1.8 s, brake), abort_if_closing (observation-conditioned, ttc 4 s). v = 14, tau 0.25.

| blocker gap | rear_gap | rear_v | evidence | contrast | margins (gap_closes/gap_open) |
|---|---|---|---|---|---|
| 60 | 20 | 20 | R_REF | True | commit_lane_change=-inf/0.111; abort_and_stabilize=0.050/0.050; abort_if_closing=-inf/0.111 |
| 60 | 20 | 24 | R_REF | True | commit_lane_change=-inf/0.111; abort_and_stabilize=0.050/0.050; abort_if_closing=-inf/-inf |
| 60 | 35 | 20 | R_REF | True | commit_lane_change=-inf/0.111; abort_and_stabilize=0.050/0.050; abort_if_closing=-inf/0.111 |
| 60 | 35 | 24 | R_REF | True | commit_lane_change=-inf/0.111; abort_and_stabilize=0.050/0.050; abort_if_closing=-inf/0.111 |
| 60 | 50 | 20 | R_REF | True | commit_lane_change=-inf/0.111; abort_and_stabilize=0.050/0.050; abort_if_closing=-0.160/0.111 |
| 60 | 50 | 24 | R_REF | True | commit_lane_change=-inf/0.111; abort_and_stabilize=0.050/0.050; abort_if_closing=-inf/0.111 |
| 80 | 20 | 20 | R_REF | True | commit_lane_change=-inf/0.105; abort_and_stabilize=0.105/0.416; abort_if_closing=0.107/0.105 |
| 80 | 20 | 24 | R_REF | True | commit_lane_change=-inf/0.105; abort_and_stabilize=0.084/0.416; abort_if_closing=0.004/0.181 |
| 80 | 35 | 20 | R_REF | True | commit_lane_change=-inf/0.105; abort_and_stabilize=0.105/0.416; abort_if_closing=-0.099/0.105 |
| 80 | 35 | 24 | R_REF | True | commit_lane_change=-inf/0.105; abort_and_stabilize=0.105/0.416; abort_if_closing=0.107/0.105 |
| 80 | 50 | 20 | R_REF | True | commit_lane_change=-inf/0.105; abort_and_stabilize=0.105/0.416; abort_if_closing=-inf/0.105 |
| 80 | 50 | 24 | R_REF | True | commit_lane_change=-inf/0.105; abort_and_stabilize=0.105/0.416; abort_if_closing=-0.021/0.105 |

Selected: (80.0, 20.0, 20.0) (window size 12); witness `abort_and_stabilize`; full-library re-run vs scan max |dG| = 1.126971203246896e-06.

Audited witness trace, world `gap_closes`:
  - run `abort_and_stabilize/gap_closes`: termination horizon, contact events none, G (re-run) 0.105
  - ego v 14.01 -> -0.00 m/s; lateral range [-0.74, 0.03] m, end 0.01; x_end 56.2; max per-tick lateral step 0.068 m = 0.096 x (v*dt) (continuous, no teleport)
  - min clearance to each actor (m): {'blk': 21.0, 'rear': 1.53}
  - dynamic peaks: {'a_lat': 4.67, 'a_long_smoothed': 6.17, 'a_comb': 6.17, 'a_comb_unmasked': 24.51, 'dyn_margin': {'a_lat': 0.416}}
  - commands (decision_t -> effective_t): keep/-3.5 0.00->0.25, keep/0.0 1.00->1.25, brake/0.0 2.80->3.05
  - actor `blk`: v -0.00 -> -0.00, lat_end 0.00, closest |dx| to ego 25.5 m
  - actor `rear`: v 17.04 -> 20.00, lat_end -3.50, closest |dx| to ego 0.2 m
Audited witness trace, world `gap_open`:
  - run `abort_and_stabilize/gap_open`: termination horizon, contact events none, G (re-run) 0.416
  - ego v 14.01 -> -0.00 m/s; lateral range [-0.74, 0.03] m, end 0.01; x_end 56.2; max per-tick lateral step 0.068 m = 0.096 x (v*dt) (continuous, no teleport)
  - min clearance to each actor (m): {'blk': 21.0, 'rear': 18.52}
  - dynamic peaks: {'a_lat': 4.67, 'a_long_smoothed': 6.17, 'a_comb': 6.17, 'a_comb_unmasked': 24.51, 'dyn_margin': {'a_lat': 0.416}}
  - commands (decision_t -> effective_t): keep/-3.5 0.00->0.25, keep/0.0 1.00->1.25, brake/0.0 2.80->3.05
  - actor `blk`: v -0.00 -> -0.00, lat_end 0.00, closest |dx| to ego 25.5 m
  - actor `rear`: v 17.04 -> 0.01, lat_end -3.50, closest |dx| to ego 22.8 m

Refuted commit trace (gap_closes):
  - run `commit_lane_change/gap_closes`: termination collision, contact events [{'type': 'collision', 'other': 'rear', 'k': 77, 'source': 'obb'}], G (re-run) -inf
  - ego v 14.01 -> 14.01 m/s; lateral range [-3.81, -0.00] m, end -3.67; x_end 51.1; max per-tick lateral step 0.123 m = 0.175 x (v*dt) (continuous, no teleport)
  - min clearance to each actor (m): {'blk': 21.0, 'rear': -0.04}
  - dynamic peaks: {'a_lat': 2.78, 'a_long_smoothed': 0.06, 'a_comb': 2.78, 'a_comb_unmasked': 2.78, 'dyn_margin': {'a_lat': 0.653}}
  - commands (decision_t -> effective_t): keep/-3.5 0.00->0.25
  - actor `blk`: v -0.00 -> 0.00, lat_end 0.00, closest |dx| to ego 30.6 m
  - actor `rear`: v 17.04 -> 20.13, lat_end -3.50, closest |dx| to ego 4.2 m

## RC06: yield, then use the time-varying gap

v = 10, static blocker in the ego lane, a faster vehicle passes in the target lane (cruise). Library: immediate_entry, wait_observe_enter (keep speed while the observed gap is unusable), wait_brake_observe_enter (brake 0.3 while waiting), brake_hold, lane_follow. With a static blocker, braking in place is also a witness whenever the gap is long enough, so the scene is R_REF either way; the case contrast is wait vs immediate.

| blocker gap | pass_gap | pass_v | evidence | contrast | margins |
|---|---|---|---|---|---|
| 45 | 10 | 16 | R_REF | False | immediate_entry=-inf; wait_observe_enter=-inf; wait_brake_observe_enter=0.105; brake_hold=0.105; lane_follow=-inf |
| 45 | 10 | 20 | R_REF | True | immediate_entry=-inf; wait_observe_enter=0.084; wait_brake_observe_enter=0.105; brake_hold=0.105; lane_follow=-inf |
| 45 | 20 | 16 | R_REF | False | immediate_entry=-inf; wait_observe_enter=-inf; wait_brake_observe_enter=-inf; brake_hold=0.105; lane_follow=-inf |
| 45 | 20 | 20 | R_REF | False | immediate_entry=-inf; wait_observe_enter=-inf; wait_brake_observe_enter=-inf; brake_hold=0.105; lane_follow=-inf |
| 60 | 10 | 16 | R_REF | True | immediate_entry=-inf; wait_observe_enter=0.084; wait_brake_observe_enter=0.105; brake_hold=0.105; lane_follow=-inf |
| 60 | 10 | 20 | R_REF | True | immediate_entry=-inf; wait_observe_enter=0.084; wait_brake_observe_enter=0.105; brake_hold=0.105; lane_follow=-inf |
| 60 | 20 | 16 | R_REF | False | immediate_entry=-inf; wait_observe_enter=-inf; wait_brake_observe_enter=-inf; brake_hold=0.105; lane_follow=-inf |
| 60 | 20 | 20 | R_REF | False | immediate_entry=-inf; wait_observe_enter=-inf; wait_brake_observe_enter=-inf; brake_hold=0.105; lane_follow=-inf |
| 75 | 10 | 16 | R_REF | True | immediate_entry=-inf; wait_observe_enter=0.084; wait_brake_observe_enter=0.105; brake_hold=0.105; lane_follow=-inf |
| 75 | 10 | 20 | R_REF | True | immediate_entry=-inf; wait_observe_enter=0.084; wait_brake_observe_enter=0.105; brake_hold=0.105; lane_follow=-inf |
| 75 | 20 | 16 | R_REF | False | immediate_entry=-inf; wait_observe_enter=-inf; wait_brake_observe_enter=-inf; brake_hold=0.105; lane_follow=-inf |
| 75 | 20 | 20 | R_REF | False | immediate_entry=-inf; wait_observe_enter=-inf; wait_brake_observe_enter=-inf; brake_hold=0.105; lane_follow=-inf |

Selected: (60.0, 10.0, 20.0); witness `wait_observe_enter`; full-library re-run vs scan max |dG| = 4.435552766857143e-06.

Audited witness trace summary:
  - run `wait_observe_enter/passer_cruise`: termination horizon, contact events none, G (re-run) 0.084
  - ego v 9.94 -> 9.93 m/s; lateral range [-3.92, -0.00] m, end -3.50; x_end 118.4; max per-tick lateral step 0.170 m = 0.343 x (v*dt) (continuous, no teleport)
  - min clearance to each actor (m): {'blk': 1.58, 'pass': 1.52}
  - dynamic peaks: {'a_lat': 5.02, 'a_long_smoothed': 0.39, 'a_comb': 5.02, 'a_comb_unmasked': 5.02, 'dyn_margin': {'a_lat': 0.373}}
  - commands (decision_t -> effective_t): keep/None 0.00->0.25, keep/-3.5 2.70->2.95
  - actor `blk`: v -0.00 -> 0.00, lat_end 0.00, closest |dx| to ego 0.1 m
  - actor `pass`: v 17.04 -> 20.00, lat_end -3.50, closest |dx| to ego 0.2 m

Refuted immediate_entry trace:
  - run `immediate_entry/passer_cruise`: termination collision, contact events [{'type': 'collision', 'other': 'pass', 'k': 30, 'source': 'obb'}], G (re-run) -inf
  - ego v 9.94 -> 9.89 m/s; lateral range [-1.89, -0.00] m, end -1.89; x_end 12.0; max per-tick lateral step 0.169 m = 0.341 x (v*dt) (continuous, no teleport)
  - min clearance to each actor (m): {'blk': 21.0, 'pass': -0.12}
  - dynamic peaks: {'a_lat': 4.59, 'a_long_smoothed': 0.39, 'a_comb': 4.59, 'a_comb_unmasked': 4.59, 'dyn_margin': {'a_lat': 0.426}}
  - commands (decision_t -> effective_t): keep/-3.5 0.00->0.25
  - actor `blk`: v -0.00 -> 0.00, lat_end 0.00, closest |dx| to ego 49.7 m
  - actor `pass`: v 17.04 -> 20.55, lat_end -3.50, closest |dx| to ego 2.7 m

## RC08: protected waiting, route non-completion

Static blocker at gap 45, v 14, goal x = gap + 4.6 + 15 m. `route_goal_completed` is computed separately by the harness (front bumper reaches the goal, no contact) and stored in `traces[...]`; it is never part of the margin, the terminal test or the label.

| scene | evidence | margins | route_goal_completed per policy |
|---|---|---|---|
| open lane | R_REF | brake_hold=0.667; bypass_left=0.119; lane_follow=-inf | {'brake_hold': False, 'bypass_left': True, 'lane_follow': False} |
| closed lane | R_REF | brake_hold=0.667; bypass_left=-inf; lane_follow=-inf | {'brake_hold': False, 'bypass_left': False, 'lane_follow': False} |

## RC15: yielding must be observed

v = 10; adjacent vehicle in the left lane behind the ego. Worlds (mode fixed per world): `adj_yields` (starts reacting at t = 0.5 s, +0.5 s reaction, brakes 3 m/s^2 to 5 m/s) and `adj_nonyield`. Library: always_merge, observe_yield_else_fallback (merge once the neighbour's closing speed has dropped by 1.5 m/s, else brake after 2.0 s), brake_hold, lane_follow.

| blocker gap | adj_gap | adj_v | evidence | contrast | margins (adj_yields/adj_nonyield) |
|---|---|---|---|---|---|
| 45 | 25 | 16 | R_REF | strong | always_merge=0.083/-inf; observe_yield_else_fallback=0.084/0.105; brake_hold=0.105/0.105; lane_follow=-inf/-inf |
| 45 | 25 | 20 | R_REF | None | always_merge=-0.198/-inf; observe_yield_else_fallback=-inf/0.105; brake_hold=0.105/0.105; lane_follow=-inf/-inf |
| 45 | 35 | 16 | R_REF | strong | always_merge=0.083/-inf; observe_yield_else_fallback=0.084/0.105; brake_hold=0.104/0.105; lane_follow=-inf/-inf |
| 45 | 35 | 20 | R_REF | strong | always_merge=0.083/-inf; observe_yield_else_fallback=0.084/0.105; brake_hold=0.105/0.105; lane_follow=-inf/-inf |
| 60 | 25 | 16 | R_REF | strong | always_merge=0.083/-inf; observe_yield_else_fallback=0.084/0.105; brake_hold=0.105/0.105; lane_follow=-inf/-inf |
| 60 | 25 | 20 | R_REF | None | always_merge=-0.198/-inf; observe_yield_else_fallback=-inf/0.105; brake_hold=0.105/0.105; lane_follow=-inf/-inf |
| 60 | 35 | 16 | R_REF | strong | always_merge=0.083/-inf; observe_yield_else_fallback=0.084/0.105; brake_hold=0.104/0.105; lane_follow=-inf/-inf |
| 60 | 35 | 20 | R_REF | strong | always_merge=0.083/-inf; observe_yield_else_fallback=0.084/0.105; brake_hold=0.105/0.105; lane_follow=-inf/-inf |
| 75 | 25 | 16 | R_REF | strong | always_merge=0.083/-inf; observe_yield_else_fallback=0.084/0.105; brake_hold=0.105/0.105; lane_follow=-inf/-inf |
| 75 | 25 | 20 | R_REF | None | always_merge=-0.198/-inf; observe_yield_else_fallback=-inf/0.105; brake_hold=0.105/0.105; lane_follow=-inf/-inf |
| 75 | 35 | 16 | R_REF | strong | always_merge=0.083/-inf; observe_yield_else_fallback=0.084/0.105; brake_hold=0.104/0.105; lane_follow=-inf/-inf |
| 75 | 35 | 20 | R_REF | strong | always_merge=0.083/-inf; observe_yield_else_fallback=0.084/0.105; brake_hold=0.105/0.105; lane_follow=-inf/-inf |

Selected (strong contrast; strong scenes [(45.0, 25.0, 16.0), (45.0, 35.0, 16.0), (45.0, 35.0, 20.0), (60.0, 25.0, 16.0), (60.0, 35.0, 16.0), (60.0, 35.0, 20.0), (75.0, 25.0, 16.0), (75.0, 35.0, 16.0), (75.0, 35.0, 20.0)], weak-or-strong scenes 9): (60.0, 35.0, 16.0); witness `observe_yield_else_fallback`; full-library re-run vs scan max |dG| = 0.0006077273085329793.

Audited witness trace, world `adj_yields`:
  - run `observe_yield_else_fallback/adj_yields`: termination horizon, contact events none, G (re-run) 0.084
  - ego v 9.94 -> 9.93 m/s; lateral range [-3.92, -0.00] m, end -3.50; x_end 108.4; max per-tick lateral step 0.170 m = 0.343 x (v*dt) (continuous, no teleport)
  - min clearance to each actor (m): {'blk': 1.53, 'adj': 21.0}
  - dynamic peaks: {'a_lat': 5.01, 'a_long_smoothed': 0.39, 'a_comb': 5.01, 'a_comb_unmasked': 5.01, 'dyn_margin': {'a_lat': 0.374}}
  - commands (decision_t -> effective_t): keep/None 0.00->0.25, keep/-3.5 1.55->1.80
  - actor `blk`: v -0.00 -> -0.00, lat_end 0.00, closest |dx| to ego 0.1 m
  - actor `adj`: v 14.79 -> 4.99, lat_end -3.50, closest |dx| to ego 26.3 m
Audited witness trace, world `adj_nonyield`:
  - run `observe_yield_else_fallback/adj_nonyield`: termination horizon, contact events none, G (re-run) 0.105
  - ego v 9.94 -> -0.00 m/s; lateral range [-0.00, -0.00] m, end -0.00; x_end 27.8; max per-tick lateral step 0.000 m = 0.000 x (v*dt) (continuous, no teleport)
  - min clearance to each actor (m): {'blk': 21.0, 'adj': 1.52}
  - dynamic peaks: {'a_lat': 0.0, 'a_long_smoothed': 5.19, 'a_comb': 5.19, 'a_comb_unmasked': 23.79, 'dyn_margin': {'a_lat': 1.0}}
  - commands (decision_t -> effective_t): keep/None 0.00->0.25, brake/None 2.00->2.25
  - actor `blk`: v -0.00 -> -0.00, lat_end 0.00, closest |dx| to ego 34.0 m
  - actor `adj`: v 14.79 -> 16.00, lat_end -3.50, closest |dx| to ego 0.1 m

always_merge trace, world `adj_yields`:
  - run `always_merge/adj_yields`: termination horizon, contact events none, G (re-run) 0.083
  - ego v 9.94 -> 9.93 m/s; lateral range [-3.92, -0.00] m, end -3.50; x_end 108.4; max per-tick lateral step 0.170 m = 0.344 x (v*dt) (continuous, no teleport)
  - min clearance to each actor (m): {'blk': 1.52, 'adj': 21.0}
  - dynamic peaks: {'a_lat': 5.01, 'a_long_smoothed': 0.39, 'a_comb': 5.01, 'a_comb_unmasked': 5.01, 'dyn_margin': {'a_lat': 0.373}}
  - commands (decision_t -> effective_t): keep/-3.5 0.00->0.25
  - actor `blk`: v -0.00 -> -0.00, lat_end 0.00, closest |dx| to ego 0.1 m
  - actor `adj`: v 14.79 -> 4.99, lat_end -3.50, closest |dx| to ego 26.0 m

always_merge trace, world `adj_nonyield`:
  - run `always_merge/adj_nonyield`: termination collision, contact events [{'type': 'collision', 'other': 'adj', 'k': 113, 'source': 'sensor', 'impulse': 4753.047797213689}], G (re-run) -inf
  - ego v 9.94 -> 12.62 m/s; lateral range [-3.92, -0.00] m, end -3.50; x_end 52.9; max per-tick lateral step 0.170 m = 0.344 x (v*dt) (continuous, no teleport)
  - min clearance to each actor (m): {'blk': 4.82, 'adj': 0.01}
  - dynamic peaks: {'a_lat': 5.01, 'a_long_smoothed': 32.49, 'a_comb': 32.49, 'a_comb_unmasked': 32.49, 'dyn_margin': {'a_lat': 0.373}}
  - commands (decision_t -> effective_t): keep/-3.5 0.00->0.25
  - actor `blk`: v -0.00 -> 0.00, lat_end 0.00, closest |dx| to ego 8.8 m
  - actor `adj`: v 14.79 -> 11.23, lat_end -3.50, closest |dx| to ego 4.3 m

## RC18: requested control exceeds authority

Scene: static blocker, v 18 or 22. Library: brake_hold, lane_follow, bypass_left (keep speed, lc 2.0), brake_steer_cmd_lc{L}_b1 for L in (1.6, 2.0, 2.5) (full brake AND steer to the left lane in L seconds). Each scene is run twice: spec with a_lat_max only, and spec with a_lat_max + a_comb_max. A command is *refuted by the combined limit* when it passes under the first and fails under the second (same plant, same command). 'Specific command refuted' means exactly this: the request violates the limit margin, so it is outside the admissible set; it does NOT claim the scene is unrecoverable.

| v | gap | a_lat-only: evidence / margins | a_lat + a_comb: evidence / margins | commands refuted only by a_comb |
|---|---|---|---|---|
| 18 | 26 | U_LIBRARY / brake_hold=-inf; lane_follow=-inf; bypass_left=-0.149; brake_steer_cmd_lc1.6_b1=-inf; brake_steer_cmd_lc2_b1=-inf; brake_steer_cmd_lc2.5_b1=-inf | U_LIBRARY / brake_hold=-inf; lane_follow=-inf; bypass_left=-0.149; brake_steer_cmd_lc1.6_b1=-inf; brake_steer_cmd_lc2_b1=-inf; brake_steer_cmd_lc2.5_b1=-inf | - |
| 18 | 30 | R_REF / brake_hold=-inf; lane_follow=-inf; bypass_left=0.008; brake_steer_cmd_lc1.6_b1=-inf; brake_steer_cmd_lc2_b1=-inf; brake_steer_cmd_lc2.5_b1=-inf | R_REF / brake_hold=-inf; lane_follow=-inf; bypass_left=0.008; brake_steer_cmd_lc1.6_b1=-inf; brake_steer_cmd_lc2_b1=-inf; brake_steer_cmd_lc2.5_b1=-inf | - |
| 18 | 34 | R_REF / brake_hold=0.534; lane_follow=-inf; bypass_left=0.119; brake_steer_cmd_lc1.6_b1=-inf; brake_steer_cmd_lc2_b1=-inf; brake_steer_cmd_lc2.5_b1=-inf | R_REF / brake_hold=0.056; lane_follow=-inf; bypass_left=0.119; brake_steer_cmd_lc1.6_b1=-inf; brake_steer_cmd_lc2_b1=-inf; brake_steer_cmd_lc2.5_b1=-inf | - |
| 18 | 38 | R_REF / brake_hold=0.667; lane_follow=-inf; bypass_left=0.119; brake_steer_cmd_lc1.6_b1=-inf; brake_steer_cmd_lc2_b1=-inf; brake_steer_cmd_lc2.5_b1=-inf | R_REF / brake_hold=0.056; lane_follow=-inf; bypass_left=0.119; brake_steer_cmd_lc1.6_b1=-inf; brake_steer_cmd_lc2_b1=-inf; brake_steer_cmd_lc2.5_b1=-inf | - |
| 18 | 42 | R_REF / brake_hold=0.667; lane_follow=-inf; bypass_left=0.119; brake_steer_cmd_lc1.6_b1=-inf; brake_steer_cmd_lc2_b1=-inf; brake_steer_cmd_lc2.5_b1=-inf | R_REF / brake_hold=0.056; lane_follow=-inf; bypass_left=0.119; brake_steer_cmd_lc1.6_b1=-inf; brake_steer_cmd_lc2_b1=-inf; brake_steer_cmd_lc2.5_b1=-inf | - |
| 18 | 46 | R_REF / brake_hold=0.667; lane_follow=-inf; bypass_left=0.119; brake_steer_cmd_lc1.6_b1=-inf; brake_steer_cmd_lc2_b1=-inf; brake_steer_cmd_lc2.5_b1=-inf | R_REF / brake_hold=0.056; lane_follow=-inf; bypass_left=0.112; brake_steer_cmd_lc1.6_b1=-inf; brake_steer_cmd_lc2_b1=-inf; brake_steer_cmd_lc2.5_b1=-inf | - |
| 22 | 30 | U_LIBRARY / brake_hold=-inf; lane_follow=-inf; bypass_left=-inf; brake_steer_cmd_lc1.6_b1=-0.123; brake_steer_cmd_lc2_b1=-0.039; brake_steer_cmd_lc2.5_b1=-inf | U_LIBRARY / brake_hold=-inf; lane_follow=-inf; bypass_left=-inf; brake_steer_cmd_lc1.6_b1=-0.596; brake_steer_cmd_lc2_b1=-0.339; brake_steer_cmd_lc2.5_b1=-inf | - |
| 22 | 34 | R_REF / brake_hold=-inf; lane_follow=-inf; bypass_left=-0.103; brake_steer_cmd_lc1.6_b1=-0.123; brake_steer_cmd_lc2_b1=0.112; brake_steer_cmd_lc2.5_b1=-inf | U_LIBRARY / brake_hold=-inf; lane_follow=-inf; bypass_left=-0.104; brake_steer_cmd_lc1.6_b1=-0.596; brake_steer_cmd_lc2_b1=-0.339; brake_steer_cmd_lc2.5_b1=-inf | ['brake_steer_cmd_lc2_b1'] |
| 22 | 38 | R_REF / brake_hold=-inf; lane_follow=-inf; bypass_left=0.024; brake_steer_cmd_lc1.6_b1=-0.123; brake_steer_cmd_lc2_b1=0.147; brake_steer_cmd_lc2.5_b1=-inf | R_REF / brake_hold=-inf; lane_follow=-inf; bypass_left=0.024; brake_steer_cmd_lc1.6_b1=-0.596; brake_steer_cmd_lc2_b1=-0.339; brake_steer_cmd_lc2.5_b1=-inf | ['brake_steer_cmd_lc2_b1'] |
| 22 | 42 | R_REF / brake_hold=-0.174; lane_follow=-inf; bypass_left=0.068; brake_steer_cmd_lc1.6_b1=-0.123; brake_steer_cmd_lc2_b1=0.194; brake_steer_cmd_lc2.5_b1=-inf | R_REF / brake_hold=-0.174; lane_follow=-inf; bypass_left=0.068; brake_steer_cmd_lc1.6_b1=-0.596; brake_steer_cmd_lc2_b1=-0.339; brake_steer_cmd_lc2.5_b1=-inf | ['brake_steer_cmd_lc2_b1'] |
| 22 | 46 | R_REF / brake_hold=0.626; lane_follow=-inf; bypass_left=0.068; brake_steer_cmd_lc1.6_b1=-0.123; brake_steer_cmd_lc2_b1=0.233; brake_steer_cmd_lc2.5_b1=-inf | R_REF / brake_hold=-0.099; lane_follow=-inf; bypass_left=0.068; brake_steer_cmd_lc1.6_b1=-0.596; brake_steer_cmd_lc2_b1=-0.339; brake_steer_cmd_lc2.5_b1=-inf | ['brake_steer_cmd_lc2_b1'] |
| 22 | 50 | R_REF / brake_hold=0.667; lane_follow=-inf; bypass_left=0.068; brake_steer_cmd_lc1.6_b1=-0.123; brake_steer_cmd_lc2_b1=0.233; brake_steer_cmd_lc2.5_b1=-inf | R_REF / brake_hold=-0.099; lane_follow=-inf; bypass_left=0.068; brake_steer_cmd_lc1.6_b1=-0.596; brake_steer_cmd_lc2_b1=-0.339; brake_steer_cmd_lc2.5_b1=-inf | ['brake_steer_cmd_lc2_b1'] |

Scenes whose evidence class changes when the combined limit is added (a_lat-only -> a_lat + a_comb): [(22.0, 34, 'R_REF', 'U_LIBRARY')].  A change R_REF -> U_LIBRARY means the only witness was a command outside the combined authority: the scene is then negative only relative to this library, never U_MODEL.

Selected scene v 22, gap 42: commands refuted only by the combined limit: ['brake_steer_cmd_lc2_b1']; feasible alternatives under both limits: ['bypass_left'] (evidence under both limits: R_REF).
Recorded peaks (m/s^2) for the refuted command(s) and the alternative, from the harness traces:

| policy | peak a_lat | peak smoothed a_long | peak a_comb | dyn margins |
|---|---|---|---|---|
| brake_steer_cmd_lc2_b1 | 6.133 | 8.921 | 10.715 | {'a_lat': 0.23335564230261674, 'a_comb': -0.3393515577808779} |
| bypass_left | 7.135 | 1.716 | 7.135 | {'a_lat': 0.10812606520143642, 'a_comb': 0.10808845252554689} |

Full-library re-run vs scan max |dG| = 0.0.  Audited refuted-command trace (`brake_steer_cmd_lc2_b1`):
  - run `brake_steer_cmd_lc2_b1/static_blocker`: termination horizon, contact events none, G (re-run) -0.339
  - ego v 22.23 -> 0.00 m/s; lateral range [-3.72, -0.00] m, end -3.71; x_end 38.1; max per-tick lateral step 0.168 m = 0.257 x (v*dt) (continuous, no teleport)
  - min clearance to each actor (m): {'blk': 1.97}
  - dynamic peaks: {'a_lat': 6.13, 'a_long_smoothed': 8.92, 'a_comb': 10.71, 'a_comb_unmasked': 24.36, 'dyn_margin': {'a_lat': 0.233, 'a_comb': -0.339}}
  - commands (decision_t -> effective_t): brake/-3.5 0.00->0.25
  - actor `blk`: v -0.00 -> -0.00, lat_end 0.00, closest |dx| to ego 5.2 m

Audited feasible-alternative trace (`bypass_left`):
  - run `bypass_left/static_blocker`: termination horizon, contact events none, G (re-run) 0.068
  - ego v 22.23 -> 22.21 m/s; lateral range [-4.03, -0.00] m, end -3.50; x_end 277.4; max per-tick lateral step 0.193 m = 0.172 x (v*dt) (continuous, no teleport)
  - min clearance to each actor (m): {'blk': 1.61}
  - dynamic peaks: {'a_lat': 7.13, 'a_long_smoothed': 1.72, 'a_comb': 7.14, 'a_comb_unmasked': 7.14, 'dyn_margin': {'a_lat': 0.108, 'a_comb': 0.108}}
  - commands (decision_t -> effective_t): keep/-3.5 0.00->0.25
  - actor `blk`: v -0.00 -> -0.00, lat_end 0.00, closest |dx| to ego 0.5 m

## Record validation

154 records in `results/carla/compositional/` checked with validate_records.check_record: 0 failures.
