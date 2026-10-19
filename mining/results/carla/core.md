# CARLA core cases (Task C2: RC03, RC07, RC12, RC19, RC20)

Physics profile `pp_381b4c6e55`, decel target 6.19, eps = 0.0, reserve c = 1.0 m (margin.DEFAULT_V2, unchanged), act delay 0.25 s. Records: `results/carla/core/*.jsonl`. Labels are `contract.classify` output only.

## RC03: braking fails, bypass passes (v in 8, 11, 14)

**Library revision (reported, not relabelled).** The mock library's 2.0 s lane change never beats CARLA braking (b_cal = 6.211; superseded `rc03_v*_lc2_superseded.jsonl`: bypass only passes at gaps where brake_hold already passes).  The CARLA library (id `rc03_carla_lc1.0_full`) uses an emergency lane change lc_time = 1.0 s (quintic profile): brake_hold, lane_follow, bypass_left, brake_steer_left (0.4 s brake first).  Bypass margins are small (0.02-0.05): left-lane clearance only just exceeds the reserve.

| v | gap scan | window (brake_hold<eps and a bypass>=eps) | chosen gap | brake_hold G | best bypass G | evidence | closed-lane evidence | closed margins |
|---|---|---|---|---|---|---|---|---|
| 8 | 6..10 | NONE | - | - | - | - | - | - |
| 11 | 6..22 | [12.0, 13.0] | 13 | -0.133 | 0.051 | R_REF | U_LIBRARY | brake_hold=-0.133, lane_follow=-inf, bypass_left=-inf, brake_steer_left=-inf |
| 14 | 6..22 | [15.0, 16.0, 17.0, 18.0, 19.0, 20.0] | 18 | -inf | 0.020 | R_REF | U_LIBRARY | brake_hold=-inf, lane_follow=-inf, bypass_left=-inf, brake_steer_left=-inf |

v = 8 m/s: **mechanism NOT shown on CARLA**, excluded (nothing relabelled).  Braking stops from 8 m/s in ~8 m; the bypass first becomes finite only at the gap where brake_hold already passes (scan below).  Closed pairs run only where a positive exists.  Closing the escape lane gives a finite-library negative: U_LIBRARY, never U_MODEL, never pre-labelled.


Scan v = 8:

| gap | evidence | margins |
|---|---|---|
| 6 | U_LIBRARY | brake_hold=-inf, lane_follow=-inf, bypass_left=-inf, brake_steer_left=-inf |
| 6.5 | U_LIBRARY | brake_hold=-0.180, lane_follow=-inf, bypass_left=-inf, brake_steer_left=-inf |
| 7 | U_LIBRARY | brake_hold=-0.080, lane_follow=-inf, bypass_left=-0.145, brake_steer_left=-inf |
| 7.5 | R_REF | brake_hold=0.020, lane_follow=-inf, bypass_left=-0.071, brake_steer_left=-inf |
| 8 | R_REF | brake_hold=0.120, lane_follow=-inf, bypass_left=0.001, brake_steer_left=-inf |
| 8.5 | R_REF | brake_hold=0.220, lane_follow=-inf, bypass_left=0.038, brake_steer_left=-inf |
| 9 | R_REF | brake_hold=0.320, lane_follow=-inf, bypass_left=0.068, brake_steer_left=-0.122 |
| 10 | R_REF | brake_hold=0.520, lane_follow=-inf, bypass_left=0.088, brake_steer_left=0.020 |

Scan v = 11:

| gap | evidence | margins |
|---|---|---|
| 6 | U_LIBRARY | brake_hold=-inf, lane_follow=-inf, bypass_left=-inf, brake_steer_left=-inf |
| 7 | U_LIBRARY | brake_hold=-inf, lane_follow=-inf, bypass_left=-inf, brake_steer_left=-inf |
| 8 | U_LIBRARY | brake_hold=-inf, lane_follow=-inf, bypass_left=-inf, brake_steer_left=-inf |
| 9 | U_LIBRARY | brake_hold=-inf, lane_follow=-inf, bypass_left=-inf, brake_steer_left=-inf |
| 10 | U_LIBRARY | brake_hold=-inf, lane_follow=-inf, bypass_left=-0.115, brake_steer_left=-inf |
| 11 | U_LIBRARY | brake_hold=-inf, lane_follow=-inf, bypass_left=-0.004, brake_steer_left=-inf |
| 12 | R_REF | brake_hold=-inf, lane_follow=-inf, bypass_left=0.051, brake_steer_left=-inf |
| 13 | R_REF | brake_hold=-0.133, lane_follow=-inf, bypass_left=0.051, brake_steer_left=-0.092 |
| 14 | R_REF | brake_hold=0.067, lane_follow=-inf, bypass_left=0.051, brake_steer_left=0.024 |
| 15 | R_REF | brake_hold=0.267, lane_follow=-inf, bypass_left=0.051, brake_steer_left=0.052 |
| 16 | R_REF | brake_hold=0.467, lane_follow=-inf, bypass_left=0.051, brake_steer_left=0.052 |
| 17 | R_REF | brake_hold=0.667, lane_follow=-inf, bypass_left=0.051, brake_steer_left=0.052 |
| 18 | R_REF | brake_hold=0.668, lane_follow=-inf, bypass_left=0.051, brake_steer_left=0.052 |
| 19 | R_REF | brake_hold=0.668, lane_follow=-inf, bypass_left=0.051, brake_steer_left=0.052 |
| 20 | R_REF | brake_hold=0.668, lane_follow=-inf, bypass_left=0.051, brake_steer_left=0.052 |
| 21 | R_REF | brake_hold=0.668, lane_follow=-inf, bypass_left=0.051, brake_steer_left=0.052 |
| 22 | R_REF | brake_hold=0.668, lane_follow=-inf, bypass_left=0.051, brake_steer_left=0.052 |

Scan v = 14:

| gap | evidence | margins |
|---|---|---|
| 6 | U_LIBRARY | brake_hold=-inf, lane_follow=-inf, bypass_left=-inf, brake_steer_left=-inf |
| 7 | U_LIBRARY | brake_hold=-inf, lane_follow=-inf, bypass_left=-inf, brake_steer_left=-inf |
| 8 | U_LIBRARY | brake_hold=-inf, lane_follow=-inf, bypass_left=-inf, brake_steer_left=-inf |
| 9 | U_LIBRARY | brake_hold=-inf, lane_follow=-inf, bypass_left=-inf, brake_steer_left=-inf |
| 10 | U_LIBRARY | brake_hold=-inf, lane_follow=-inf, bypass_left=-inf, brake_steer_left=-inf |
| 11 | U_LIBRARY | brake_hold=-inf, lane_follow=-inf, bypass_left=-inf, brake_steer_left=-inf |
| 12 | U_LIBRARY | brake_hold=-inf, lane_follow=-inf, bypass_left=-0.203, brake_steer_left=-inf |
| 13 | U_LIBRARY | brake_hold=-inf, lane_follow=-inf, bypass_left=-0.110, brake_steer_left=-inf |
| 14 | U_LIBRARY | brake_hold=-inf, lane_follow=-inf, bypass_left=-0.029, brake_steer_left=-inf |
| 15 | R_REF | brake_hold=-inf, lane_follow=-inf, bypass_left=0.020, brake_steer_left=-inf |
| 16 | R_REF | brake_hold=-inf, lane_follow=-inf, bypass_left=0.020, brake_steer_left=-0.188 |
| 17 | R_REF | brake_hold=-inf, lane_follow=-inf, bypass_left=0.020, brake_steer_left=-0.081 |
| 18 | R_REF | brake_hold=-inf, lane_follow=-inf, bypass_left=0.020, brake_steer_left=0.008 |
| 19 | R_REF | brake_hold=-inf, lane_follow=-inf, bypass_left=0.020, brake_steer_left=0.033 |
| 20 | R_REF | brake_hold=-0.101, lane_follow=-inf, bypass_left=0.020, brake_steer_left=0.033 |
| 21 | R_REF | brake_hold=0.099, lane_follow=-inf, bypass_left=0.020, brake_steer_left=0.033 |
| 22 | R_REF | brake_hold=0.299, lane_follow=-inf, bypass_left=0.020, brake_steer_left=0.033 |

## RC07: premature hand-back vs continuation

Library `rc07_v1` = bypass_then_stop, bypass_then_return, brake_hold, bypass_left (primary-only avoidance).  v = 12, rear actor in the left lane.

| gap | rear_gap | rear_v | evidence | margins |
|---|---|---|---|---|
| 26 | 40 | 18 | R_REF | bypass_then_stop=-inf, bypass_then_return=0.099, brake_hold=0.105, bypass_left=-inf |
| 26 | 40 | 24 | R_REF | bypass_then_stop=-inf, bypass_then_return=-inf, brake_hold=0.105, bypass_left=-inf |
| 26 | 60 | 18 | R_REF | bypass_then_stop=-inf, bypass_then_return=0.099, brake_hold=0.105, bypass_left=-inf |
| 26 | 60 | 24 | R_REF | bypass_then_stop=-inf, bypass_then_return=0.099, brake_hold=0.105, bypass_left=-inf |
| 26 | 80 | 18 | R_REF | bypass_then_stop=-inf, bypass_then_return=0.099, brake_hold=0.105, bypass_left=0.127 |
| 26 | 80 | 24 | R_REF | bypass_then_stop=-inf, bypass_then_return=0.099, brake_hold=0.105, bypass_left=-inf |
| 26 | 100 | 18 | R_REF | bypass_then_stop=-inf, bypass_then_return=0.099, brake_hold=0.105, bypass_left=0.127 |
| 26 | 100 | 24 | R_REF | bypass_then_stop=-inf, bypass_then_return=0.099, brake_hold=0.105, bypass_left=-inf |
| 34 | 40 | 18 | R_REF | bypass_then_stop=-inf, bypass_then_return=0.107, brake_hold=0.105, bypass_left=-inf |
| 34 | 40 | 24 | R_REF | bypass_then_stop=-inf, bypass_then_return=-inf, brake_hold=0.105, bypass_left=-inf |
| 34 | 60 | 18 | R_REF | bypass_then_stop=-inf, bypass_then_return=0.105, brake_hold=0.105, bypass_left=-inf |
| 34 | 60 | 24 | R_REF | bypass_then_stop=-inf, bypass_then_return=-0.109, brake_hold=0.105, bypass_left=-inf |
| 34 | 80 | 18 | R_REF | bypass_then_stop=-inf, bypass_then_return=0.119, brake_hold=0.105, bypass_left=0.127 |
| 34 | 80 | 24 | R_REF | bypass_then_stop=-inf, bypass_then_return=0.109, brake_hold=0.105, bypass_left=-inf |
| 34 | 100 | 18 | R_REF | bypass_then_stop=-inf, bypass_then_return=0.119, brake_hold=0.105, bypass_left=0.127 |
| 34 | 100 | 24 | R_REF | bypass_then_stop=-inf, bypass_then_return=0.105, brake_hold=0.105, bypass_left=-inf |

Selected: gap 26, rear_gap 60, rear_v 24 (first coarse-scan setting with the contrast, not tuned).  Evidence **R_REF**; margins: bypass_then_stop=-inf, bypass_then_return=0.099, brake_hold=0.105, bypass_left=-inf.

- bypass_then_stop: termination collision, contacted ['rear']
- bypass_then_return: termination horizon, contacted none
- bypass_left: termination collision, contacted ['rear']
- brake_hold: termination horizon, contacted none

Measured rear speed (per-tick spy, 699 ticks): nominal 24, at start 19.32, max 24.69, mean 23.77 m/s.

Primary-only avoidance passes the obstacle: with a far/slow rear actor (rear_gap 100, rear_v 18) bypass_left = 0.127; in the selected scene its failure is a contact with the rear actor ('collision', ['rear']), not the blocker.  brake_hold passes in place at gap 26 m, so the scene stays R_REF.

## RC12: baseline-keep prefix before braking (corridor, gap 40, v 12, tau 0.25, brake_hold@prefix)

| prefix_s | brake_hold G | evidence |
|---|---|---|
| 0.00 | 0.668 | R_REF |
| 0.25 | 0.668 | R_REF |
| 0.50 | 0.668 | R_REF |
| 0.75 | 0.668 | R_REF |
| 1.00 | 0.668 | R_REF |
| 1.25 | 0.668 | R_REF |
| 1.50 | 0.668 | R_REF |
| 1.75 | 0.620 | R_REF |
| 2.00 | 0.032 | R_REF |
| 2.05 | -0.098 | U_LIBRARY |
| 2.10 | -inf | U_LIBRARY |
| 2.15 | -inf | U_LIBRARY |
| 2.20 | -inf | U_LIBRARY |
| 2.25 | -inf | U_LIBRARY |
| 2.50 | -inf | U_LIBRARY |
| 2.75 | -inf | U_LIBRARY |
| 3.00 | -inf | U_LIBRARY |

Feasible-prefix set (R_REF): [0.0, 0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 1.75, 2.0]; upper end 2.0; down-set (feasible iff prefix <= upper): True.
Analytic T_p* = (gap - D_stop - c)/v, D_stop = v^2/(2 b_cal) + v tau_eff, b_cal = 6.211, tau_eff = 0.3: **1.984 s**.  The prefix is quantised to the 0.05 s tick (step function), so agreement means within one tick.  b_cal/tau_eff come from the same brake_hold calibration trace (Task 3): this checks structure, not an independent prediction; the ego also runs at ~11.89 m/s, not 12.

CARLA upper end - T_p* = +0.016 s.

## RC19: benign anchor vs nuisance twin

| v | anchor G | nuisance G | |dV| (max abs profile diff) | evidence anchor / twin |
|---|---|---|---|---|
| 8 | ['0.668'] | ['0.668'] | 0.0000 | R_REF / R_REF |
| 12 | ['0.668'] | ['0.668'] | 0.0000 | R_REF / R_REF |

## RC20: library completeness (same RC03 scene, physics unchanged)

Restricted `rc03_carla_lc1.0_restricted` (brake_hold, lane_follow); budget-limited = restricted with run_mask {(0,0)} (library id +budget1); expanded `rc03_carla_lc1.0_full`; policy version `sim.policies.VERSION` in each descriptor.

| v | gap | restricted | budget-limited (R_raw) | expanded | identical physical spec |
|---|---|---|---|---|---|
| 11 | 13 | U_LIBRARY (rc03_carla_lc1.0_restricted) | UNRESOLVED, [[-0.13283333907419026], [None]] | R_REF (rc03_carla_lc1.0_full) | True |
| 14 | 18 | U_LIBRARY (rc03_carla_lc1.0_restricted) | UNRESOLVED, [['-inf'], [None]] | R_REF (rc03_carla_lc1.0_full) | True |

## Record validation

89 records in `results/carla/core/` (excluding superseded/probe files) checked with validate_records.check_record: 0 failures.