# CARLA foundation cases vs analytic anchors (Task C1: RC01, RC02, RC16, RC17)

Physics profile `pp_381b4c6e55`, decel target 6.19 m/s^2 (nearest to 6.0), map Carla/Maps/OpenDriveMap, v = 12.0 m/s, configured act delay tau = 0.25 s, reserve c = 1.0 m, eps = 0.0 (margin.DEFAULT_V2, unchanged). All numbers below come from CARLA runs logged in `results/carla/foundation/*.jsonl`.

## Calibration measured from the brake_hold trace (CARLA, free road, 60 m gap)

| configured tau (s) | plant onset (s) | tau_eff (s) | v at onset | brake time (s) | brake dist (m) | b_time_mean | b_dist_equiv |
|---|---|---|---|---|---|---|---|
| 0.25 | 0.050 | 0.300 | 11.888 | 1.45 | 11.33 | 8.198 | 6.238 |
| 0.0 | 0.050 | 0.050 | 11.930 | 1.45 | 11.45 | 8.227 | 6.213 |
| 0.5 | 0.050 | 0.550 | 11.875 | 1.45 | 11.40 | 8.190 | 6.182 |

b_cal (primary) = distance-equivalent constant decel v_onset^2/(2 s_brake) = **6.211 m/s^2** (mean of the 3 runs; the decel that reproduces the measured braking distance, which is what the stopping-distance formula uses).  Diagnostic only: the time-mean v_onset/t_brake = 8.205 m/s^2 is inflated by the final snap to rest (see bands: ~5 m/s^2 at speed, tens of m/s^2 below ~6 m/s, where the wheel brake locks and static friction takes over), so it is NOT a constant-decel b.  Plant onset (first tick with ax < -1 after the effective brake command) = 0.050 s, so tau_eff = 0.25 + 0.050 = **0.300 s**.  Profile value for target 6.19: 6.19; selftest measured 6.23.

Decel by speed band, first run (m/s^2; drag + brake):

| v from | v to | decel |
|---|---|---|
| 11.89 | 8.92 | 4.99 |
| 8.92 | 5.94 | 5.19 |
| 5.94 | 2.97 | 22.74 |
| 2.97 | 0.00 | 33.11 |

## Anchor vs CARLA (residual)

**Independence caveat.** b_cal and tau_eff above are calibrated from the same plant, speed (12 m/s), decel target and brake_hold trace that the sweeps use.  The RC01/RC02/RC17 residuals therefore test the STRUCTURE of the constant-b + constant-delay formula (and the harness/labelling chain), not an independent prediction of CARLA from first principles.

Residual = CARLA - analytic.  CARLA bracket (lo infeasible, hi feasible; label = R_REF, i.e. brake_hold witness) is bisected to 0.25 m (gap) / 0.01 s (tau); `G=0` is the linear zero of brake_hold's continuous margin G between the bracketing samples.

| quantity | analytic (b_cal) | analytic (b_time, diagnostic) | CARLA label bracket | CARLA bracket mid | CARLA G=0 | residual (mid - analytic) | residual (G=0 - analytic) | residual (mid - b_time analytic, diagnostic) |
|---|---|---|---|---|---|---|---|---|
| RC01 gap d_required, brake_hold only (m) | 16.193 | 13.375 | (15.750, 16.000] | 15.875 | 15.909 | -0.318 | -0.283 | +2.500 |
| RC02 gap d_required, full library (m) | 16.193 | 13.375 | (15.750, 16.000] | 15.875 | 15.909 | -0.318 | -0.283 | +2.500 |
| RC17 tau* configured delay at gap 18.0 (s) | 0.401 | 0.635 | [0.422, 0.431) = ticks [0.40, 0.45] | 0.427 | n/a (tick step, see note) | within one tick (+/-0.05 s): +0.026 | n/a | -0.209 |

RC01/RC02 residual decomposition (m): d_required at nominal v = 12.0 m/s: 16.193; with the measured onset speed v_onset = 11.888 m/s (same tau_eff, same b_cal): 15.943.  The speed shortfall explains -0.250 m of the -0.318 m residual (bracket mid 15.875); what remains against the v_onset anchor is -0.068 m (bracket width 0.25 m, 0.05 s onset tick, non-constant decel).


RC17 note: the act delay is quantised to the 0.05 s tick grid (round to nearest tick), so the label is a step function of the configured delay with its edge at a tick midpoint (here 0.425 s: 0.40 s = 8 ticks feasible, 0.45 s = 9 ticks infeasible).  The 0.01 s bisection only locates that edge; the RC17 residual is therefore reported as within one tick (+/-0.05 s), and the linear G=0 interpolation is dropped for RC17 (G is constant between tick edges, so the crossing is not a real boundary estimate).  The analytic tau* lies inside the feasible-to-infeasible tick interval [0.40, 0.45].


Analytic details: d_required = v*tau_eff + v^2/(2 b_cal) + c = 12.0*0.300 + 144/(2*b_cal) + 1.0; tau* (total) = (gap - v^2/(2b) - c)/v = 0.685 s (b_time) / 0.451 s (b_cal), configured-delay equivalent subtracts the plant onset 0.050 s.

### RC01 sweep (brake_hold only, with_bypass=False)

| gap (m) | brake_hold G | evidence | V | policies refuted | termination (brake_hold) / contact source |
|---|---|---|---|---|---|
| 8.000 | -inf | U_LIBRARY | -inf | 1/1 | collision / obb+sensor |
| 10.000 | -inf | U_LIBRARY | -inf | 1/1 | collision / sensor |
| 12.000 | -inf | U_LIBRARY | -inf | 1/1 | collision / obb+sensor |
| 14.000 | -inf | U_LIBRARY | -inf | 1/1 | collision / obb+sensor |
| 15.000 | -0.182 | U_LIBRARY | -0.182 | 1/1 | horizon / none |
| 15.500 | -0.082 | U_LIBRARY | -0.082 | 1/1 | horizon / none |
| 15.750 | -0.032 | U_LIBRARY | -0.032 | 1/1 | horizon / none |
| 16.000 | 0.018 | R_REF | 0.018 | 0/1 | horizon / none |
| 18.000 | 0.418 | R_REF | 0.418 | 0/1 | horizon / none |
| 20.000 | 0.668 | R_REF | 0.668 | 0/1 | horizon / none |
| 22.000 | 0.668 | R_REF | 0.668 | 0/1 | horizon / none |
| 24.000 | 0.668 | R_REF | 0.668 | 0/1 | horizon / none |

### RC02 sweep (full corridor library: brake_hold, bypass_left, lane_follow)

| gap (m) | brake_hold G | evidence | V | policies refuted | termination (brake_hold) / contact source |
|---|---|---|---|---|---|
| 8.000 | -inf | U_LIBRARY | -inf | 3/3 | collision / obb+sensor |
| 10.000 | -inf | U_LIBRARY | -inf | 3/3 | collision / sensor |
| 12.000 | -inf | U_LIBRARY | -inf | 3/3 | collision / obb+sensor |
| 14.000 | -inf | U_LIBRARY | -inf | 3/3 | collision / obb+sensor |
| 15.000 | -0.182 | U_LIBRARY | -0.182 | 3/3 | horizon / none |
| 15.500 | -0.082 | U_LIBRARY | -0.082 | 3/3 | horizon / none |
| 15.750 | -0.032 | U_LIBRARY | -0.032 | 3/3 | horizon / none |
| 16.000 | 0.018 | R_REF | 0.018 | 2/3 | horizon / none |
| 18.000 | 0.418 | R_REF | 0.418 | 2/3 | horizon / none |
| 20.000 | 0.668 | R_REF | 0.668 | 2/3 | horizon / none |
| 22.000 | 0.668 | R_REF | 0.668 | 2/3 | horizon / none |
| 24.000 | 0.668 | R_REF | 0.668 | 2/3 | horizon / none |

RC02 evidence classes observed below the boundary (CARLA): R_REF, U_LIBRARY. CARLA never reports U_MODEL (reference-model-only label).

### RC17 delay sweep, gap 18 m (full library)

| configured tau (s) | tau_eff (s) | brake_hold G | evidence | feasible (R_REF) | measured onset (s) |
|---|---|---|---|---|---|
| 0.000 | 0.050 | 0.668 | R_REF | True | 0.050 |
| 0.150 | 0.200 | 0.656 | R_REF | True | 0.050 |
| 0.300 | 0.350 | 0.308 | R_REF | True | 0.050 |
| 0.375 | 0.425 | 0.068 | R_REF | True | 0.050 |
| 0.412 | 0.463 | 0.068 | R_REF | True | 0.050 |
| 0.422 | 0.472 | 0.068 | R_REF | True | 0.050 |
| 0.431 | 0.481 | -0.061 | U_LIBRARY | False | 0.050 |
| 0.450 | 0.500 | -0.061 | U_LIBRARY | False | 0.050 |
| 0.600 | 0.650 | -inf | U_LIBRARY | False | 0.050 |
| 0.750 | 0.800 | -inf | U_LIBRARY | False | 0.050 |
| 0.900 | 0.950 | -inf | U_LIBRARY | False | 0.050 |
| 1.050 | 1.100 | -inf | U_LIBRARY | False | 0.050 |

Feasible-delay set (tested configured taus with label R_REF): {0.000, 0.150, 0.300, 0.375, 0.412, 0.422} s (every tested tau <= 0.422 s; bracket above).

## RC16: centre line clears, body does not (two_lane site, v = 12, tau = 0.25)

RC16 is a policy-level contrast, not a label boundary: the case label stays R_REF throughout because brake_hold is a witness at every offset (G = 0.668).  The contrast is in the per-policy margins: lane_follow (centre-line path) vs nudge_left (wider path).  The analytic threshold 2.375 m below uses the MEASURED ego half-width (fitted from the lane_follow G=0 crossing, not predicted from the declared 0.95 m).  The post is frozen (`set_simulate_physics(False)`), i.e. rigid by design: it cannot be pushed or tip over.

**Collision-sensor corroboration was NOT obtained for RC16.**  Cause (one probe, 3 rollouts of an uncontrolled ego coasting at 12 m/s into the post at offset 0.4 m): a frozen actor DOES generate collision-sensor events, but the sensor event arrives 1-2 ticks AFTER the first oriented-box overlap (bike frozen: obb at tick 127, sensor at tick 129; bike free: obb 127, sensor 128; frozen A2: obb 127, sensor 128), and the harness ends the episode at the first OBB overlap (termination rule unchanged), so no sensor event is ever logged in the RC16 records.  Sensor corroboration exists only for the RC01/RC02 gap-sweep collisions, where the sensor event lands on or before the tick the harness stops.

Post = vehicle.diamondback.century (measured half-extents 0.83 x 0.29 m, physics frozen so it cannot tip into the lane) at lateral offset_m (+ = right of ego centre line).  Declared ego half-width 0.95 m, but the measured Tesla box half-width is ~1.085 m (back-solved from the G=0 crossing), reserve 1 m: lane_follow body+reserve conflict when offset < 1.085 + 0.29 + 1.0 = 2.375 m; contact when offset < ~1.375 m.  G values are the label-grade margins from the OBB evaluator; `contact` columns list the collision-sensor corroboration (source `sensor` = CARLA collision sensor, `obb` = evaluator box overlap).

| offset (m) | point model safe? (abs(offset) > post half-width 0.3; true throughout this sweep) | lane_follow G | lane_follow term / contact src | nudge_left G | brake_hold G | evidence | best policy |
|---|---|---|---|---|---|---|---|
| 0.40 | True | -inf | collision / obb | -0.195 | 0.668 | R_REF | brake_hold |
| 0.60 | True | -inf | collision / obb | -0.155 | 0.668 | R_REF | brake_hold |
| 0.80 | True | -inf | collision / obb | -0.115 | 0.668 | R_REF | brake_hold |
| 1.00 | True | -inf | collision / obb | -0.075 | 0.668 | R_REF | brake_hold |
| 1.20 | True | -inf | collision / obb | -0.035 | 0.668 | R_REF | brake_hold |
| 1.40 | True | -0.195 | horizon / none | 0.005 | 0.668 | R_REF | brake_hold |
| 1.60 | True | -0.155 | horizon / none | 0.045 | 0.668 | R_REF | brake_hold |
| 1.80 | True | -0.115 | horizon / none | 0.085 | 0.668 | R_REF | brake_hold |
| 2.00 | True | -0.075 | horizon / none | 0.125 | 0.668 | R_REF | brake_hold |
| 2.20 | True | -0.035 | horizon / none | 0.165 | 0.668 | R_REF | brake_hold |
| 2.40 | True | 0.005 | horizon / none | 0.205 | 0.668 | R_REF | brake_hold |
| 2.60 | True | 0.045 | horizon / none | 0.245 | 0.668 | R_REF | brake_hold |
| 2.80 | True | 0.085 | horizon / none | 0.285 | 0.668 | R_REF | brake_hold |
| 3.00 | True | 0.125 | horizon / none | 0.325 | 0.668 | R_REF | brake_hold |
