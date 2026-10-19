# CARLA partial-observation cases (Task 7: RC10, RC11, RC14)

Physics profile `pp_381b4c6e55`, decel target 6.19, eps = 0.0, reserve c = 1.0 m (margin.DEFAULT_V2, unchanged), act delay 0.25 s (5 ticks), `a_lat_max = 8` m/s^2 (dynamic-limit margin; `a_comb_max` not used), emergency lane change lc_time 2 s. Records: `results/carla/occlusion/*.jsonl`. Labels are `contract.classify` output only; mock and CARLA outputs are kept apart.

**Site and mapping.** The custom 2-lane straight (`straight_2lane`, no map change). One STOPPED hidden vehicle `hid` is D m ahead of the ego (bumper gap) in exactly one of the two escape corridors: the ego's own lane (lat 0, the RIGHT lane; corridor = keep lane / stop in lane = info_tree `fixed_right`, world 'blocks RIGHT') or the adjacent LEFT lane (lat -3.5; corridor = change lane = `fixed_left`, world 'blocks LEFT'). Worlds: `hid_left_lane` (staying or stopping is safe; a left change collides) and `hid_own_lane` (only a braking left change works; D < stopping distance so stopping cannot). The vehicle is spawned in CARLA in both worlds (ground truth, static, exact placement), but the ObservationFeed withholds it until `t_reveal`; there is no CARLA occluder. Library: `stop_in_place` (full brake, hold), `fixed_left` (brake and change left at once), `fixed_right` (keep lane at speed), `shared_slowdown_then_branch` (full brake in lane until the object is delivered; then own lane -> change left while still braking, left lane -> keep lane braking; the branch is latched from the delivered history). G cells read `hid_left_lane/hid_own_lane`.

**info_tree reconciliation.** `reference/info_tree.py` has two LATERAL corridors that both need T_lat. On the two-lane site the right corridor is the ego's own lane and needs no lateral move, so the matrix is `build_matrix_two_lane` (added): world A/hid_left_lane has stop/stay/observe succeed (+1) and blind_L fail; world B/hid_own_lane has the same lateral slack expression as before. The binding constraint is the lateral world, so the analytic threshold is unchanged and is also given in closed form, `t_obs_star = t_arr_max - tau - T_lat` (t_arr_max = arrival time under continuous braking, since the branch keeps braking). The mock test checks the closed form against both discrete trees.

**Analytic anchors.** b_cal = 6.211 m/s^2, tau_eff = 0.3 s (0.25 s configured, quantised to 5 ticks, + 0.05 s onset), T_lat = 1.388 s (quintic, lc_time 2, to a lateral displacement of 2.9 m = hw_ego + hw_hid + reserve of the 3.5 m lane width). Cross-check tolerance 0.25 s was declared in the script header.

**Plant caveats.** `hid` is static and re-placed from the measured ego bumper, so the declared gap D is exact; the ego starts on its declared speed (measured t0 speeds are in the audit traces below). No moving non-ego actor exists in these scenes, so the warm-up speed shortfall of moving actors (3-5 % low) does not apply. CARLA's brake model locks the wheels at the end of a stop (see Task 6); it only affects a_comb, which is not used here. The branch policy's braking is the calibrated decel target 6.19 m/s^2 (b_cal 6.211).

## Observation contract on CARLA

Same policy (`spy`, full brake) in both worlds, D = 19, v = 14, t_reveal = 0.6: 12 pre-reveal frames per world; objs / lane_w / t_capture / ego_hl identical in all of them and objs empty: **True**; max |ego speed difference| between the worlds before the reveal 0.0000 m/s (CARLA is not bit-reproducible, and the ego trajectory is identical only up to that); at the reveal tick the delivered objects are [{'dx': 15.06390380859375, 'dlat': -3.49965763092041, 'dv': -12.439548928543635, 'hl': 1.852684736251831, 'hw': 0.8943392634391785}] (hid_left_lane) vs [{'dx': 15.06390380859375, 'dlat': 0.00034236907958984375, 'dv': -12.439548928543635, 'hl': 1.852684736251831, 'hw': 0.8943392634391785}] (hid_own_lane): separated True. The mock unit tests assert the same with exact equality of the whole Obs sequence and run the leakage check (no id / name / colour / blueprint / world name / absolute coordinate of the hidden vehicle in policy inputs before the reveal).

## Reveal-time sweep (RC10 -> RC11)

Evidence class per reveal time (R = R_REF, I = U_INFORMATION, L = U_LIBRARY, ? = other), one string per scene, reveal times in the header row.

v = 14 m/s, D_stop+c (analytic, b_cal, tau_eff) = 20.98 m

| D | 0 | 0.1 | 0.2 | 0.3 | 0.4 | 0.5 | 0.6 | 0.7 | 0.8 | 0.9 | 1 | 1.1 | 1.2 | measured bracket | analytic t_obs* | midpoint - t* | within 0.25 s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 17 | L |  |  |  |  |  | L |  |  |  |  |  | L | - | -0.150 | -0.113 | - | no transition inside the sweep |
| 18 | R | I | I | I | I | I | I | I | I | I | I | I | I | (0.0, 0.1) | -0.025 | 0.068 | +0.075 | True |
| 19 | R | R | I | I | I | I | I | I | I | I | I | I | I | (0.1, 0.2) | 0.100 | 0.305 | +0.050 | True |
| 20 | R | R | R | R | I | I | I | I | I | I | I | I | I | (0.3, 0.4) | 0.225 | - | +0.125 | True |
| 21 | R | R | R | R | R | R | R | R | R | R | R | R | R | - | none (common stop) | - | - | no finite analytic threshold (a common stop exists) |
| 22 | R |  |  |  |  |  | R |  |  |  |  |  | R | - | none (common stop) | - | - | no finite analytic threshold (a common stop exists) |
| 24 | R |  |  |  |  |  | R |  |  |  |  |  | R | - | none (common stop) | - | - | no finite analytic threshold (a common stop exists) |

v = 18 m/s, D_stop+c (analytic, b_cal, tau_eff) = 32.48 m

| D | 0 | 0.2 | 0.4 | 0.6 | 0.8 | 1 | 1.2 | measured bracket | analytic t_obs* | midpoint - t* | within 0.25 s |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 26 | R | R | I | I | I | I | I | (0.2, 0.4) | 0.182 | 0.182 | +0.118 | True |
| 28 | R | R | R | I | I | I | I | (0.4, 0.6) | 0.431 | 0.452 | +0.069 | True |
| 30 | R | R | R | R | R | I | I | (0.8, 1.0) | 0.681 | 0.820 | +0.219 | True |

Full margins (G, `hid_left_lane/hid_own_lane`) per record:

| v | D | t_reveal | evidence | margins |
|---|---|---|---|---|
| 14 | 17 | 0 | U_LIBRARY | stop_in_place=0.105/-inf; fixed_left=-inf/-0.072; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/-0.072 |
| 14 | 17 | 0.6 | U_LIBRARY | stop_in_place=0.105/-inf; fixed_left=-inf/-0.072; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/-inf |
| 14 | 17 | 1.2 | U_LIBRARY | stop_in_place=0.105/-inf; fixed_left=-inf/-0.072; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/-inf |
| 14 | 18 | 0 | R_REF | stop_in_place=0.105/-inf; fixed_left=-inf/0.025; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/0.025 |
| 14 | 18 | 0.1 | U_INFORMATION | stop_in_place=0.105/-inf; fixed_left=-inf/0.025; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/-0.042 |
| 14 | 18 | 0.2 | U_INFORMATION | stop_in_place=0.105/-inf; fixed_left=-inf/0.025; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/-0.111 |
| 14 | 18 | 0.3 | U_INFORMATION | stop_in_place=0.105/-inf; fixed_left=-inf/0.025; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/-0.178 |
| 14 | 18 | 0.4 | U_INFORMATION | stop_in_place=0.105/-inf; fixed_left=-inf/0.025; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/-inf |
| 14 | 18 | 0.5 | U_INFORMATION | stop_in_place=0.105/-inf; fixed_left=-inf/0.025; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/-inf |
| 14 | 18 | 0.6 | U_INFORMATION | stop_in_place=0.105/-inf; fixed_left=-inf/0.025; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/-inf |
| 14 | 18 | 0.7 | U_INFORMATION | stop_in_place=0.105/-inf; fixed_left=-inf/0.025; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/-inf |
| 14 | 18 | 0.8 | U_INFORMATION | stop_in_place=0.105/-inf; fixed_left=-inf/0.025; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/-inf |
| 14 | 18 | 0.9 | U_INFORMATION | stop_in_place=0.105/-inf; fixed_left=-inf/0.025; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/-inf |
| 14 | 18 | 1 | U_INFORMATION | stop_in_place=0.105/-inf; fixed_left=-inf/0.025; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/-inf |
| 14 | 18 | 1.1 | U_INFORMATION | stop_in_place=0.105/-inf; fixed_left=-inf/0.025; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/-inf |
| 14 | 18 | 1.2 | U_INFORMATION | stop_in_place=0.105/-inf; fixed_left=-inf/0.025; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/-inf |
| 14 | 19 | 0 | R_REF | stop_in_place=0.105/-inf; fixed_left=-inf/0.081; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/0.081 |
| 14 | 19 | 0.1 | R_REF | stop_in_place=0.105/-inf; fixed_left=-inf/0.081; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/0.049 |
| 14 | 19 | 0.2 | U_INFORMATION | stop_in_place=0.105/-inf; fixed_left=-inf/0.081; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/-0.002 |
| 14 | 19 | 0.3 | U_INFORMATION | stop_in_place=0.105/-inf; fixed_left=-inf/0.081; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/-0.070 |
| 14 | 19 | 0.4 | U_INFORMATION | stop_in_place=0.105/-inf; fixed_left=-inf/0.081; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/-0.133 |
| 14 | 19 | 0.5 | U_INFORMATION | stop_in_place=0.105/-inf; fixed_left=-inf/0.081; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/-0.180 |
| 14 | 19 | 0.6 | U_INFORMATION | stop_in_place=0.105/-inf; fixed_left=-inf/0.081; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/-inf |
| 14 | 19 | 0.7 | U_INFORMATION | stop_in_place=0.105/-inf; fixed_left=-inf/0.081; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/-inf |
| 14 | 19 | 0.8 | U_INFORMATION | stop_in_place=0.105/-inf; fixed_left=-inf/0.081; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/-inf |
| 14 | 19 | 0.9 | U_INFORMATION | stop_in_place=0.105/-inf; fixed_left=-inf/0.081; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/-inf |
| 14 | 19 | 1 | U_INFORMATION | stop_in_place=0.105/-inf; fixed_left=-inf/0.081; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/-inf |
| 14 | 19 | 1.1 | U_INFORMATION | stop_in_place=0.105/-inf; fixed_left=-inf/0.081; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/-inf |
| 14 | 19 | 1.2 | U_INFORMATION | stop_in_place=0.105/-inf; fixed_left=-inf/0.081; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/-inf |
| 14 | 20 | 0 | R_REF | stop_in_place=0.121/-0.101; fixed_left=-inf/0.093; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.121/0.093 |
| 14 | 20 | 0.1 | R_REF | stop_in_place=0.121/-0.101; fixed_left=-inf/0.065; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.121/0.070 |
| 14 | 20 | 0.2 | R_REF | stop_in_place=0.121/-0.101; fixed_left=-inf/0.092; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.121/0.050 |
| 14 | 20 | 0.3 | R_REF | stop_in_place=0.121/-0.101; fixed_left=-inf/0.093; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.121/0.036 |
| 14 | 20 | 0.4 | U_INFORMATION | stop_in_place=0.121/-0.101; fixed_left=-inf/0.093; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.121/-0.000 |
| 14 | 20 | 0.5 | U_INFORMATION | stop_in_place=0.121/-0.101; fixed_left=-inf/0.093; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.121/-0.030 |
| 14 | 20 | 0.6 | U_INFORMATION | stop_in_place=0.121/-0.101; fixed_left=-inf/0.093; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.121/-0.112 |
| 14 | 20 | 0.7 | U_INFORMATION | stop_in_place=0.121/-0.101; fixed_left=-inf/0.093; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.121/-inf |
| 14 | 20 | 0.8 | U_INFORMATION | stop_in_place=0.121/-0.101; fixed_left=-inf/0.093; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.121/-inf |
| 14 | 20 | 0.9 | U_INFORMATION | stop_in_place=0.121/-0.101; fixed_left=-inf/0.093; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.121/-inf |
| 14 | 20 | 1 | U_INFORMATION | stop_in_place=0.121/-0.101; fixed_left=-inf/0.093; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.121/-inf |
| 14 | 20 | 1.1 | U_INFORMATION | stop_in_place=0.121/-0.101; fixed_left=-inf/0.093; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.121/-inf |
| 14 | 20 | 1.2 | U_INFORMATION | stop_in_place=0.121/-0.101; fixed_left=-inf/0.093; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.121/-inf |
| 14 | 21 | 0 | R_REF | stop_in_place=0.227/0.099; fixed_left=-inf/0.093; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.227/0.093 |
| 14 | 21 | 0.1 | R_REF | stop_in_place=0.227/0.099; fixed_left=-inf/0.093; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.227/0.070 |
| 14 | 21 | 0.2 | R_REF | stop_in_place=0.227/0.099; fixed_left=-inf/0.093; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.227/0.051 |
| 14 | 21 | 0.3 | R_REF | stop_in_place=0.227/0.099; fixed_left=-inf/0.093; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.227/0.036 |
| 14 | 21 | 0.4 | R_REF | stop_in_place=0.227/0.099; fixed_left=-inf/0.093; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.227/-0.000 |
| 14 | 21 | 0.5 | R_REF | stop_in_place=0.227/0.099; fixed_left=-inf/0.093; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.227/-0.003 |
| 14 | 21 | 0.6 | R_REF | stop_in_place=0.227/0.099; fixed_left=-inf/0.092; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.227/-0.007 |
| 14 | 21 | 0.7 | R_REF | stop_in_place=0.227/0.099; fixed_left=-inf/0.093; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.227/-0.103 |
| 14 | 21 | 0.8 | R_REF | stop_in_place=0.227/0.099; fixed_left=-inf/0.093; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.227/-0.172 |
| 14 | 21 | 0.9 | R_REF | stop_in_place=0.227/0.099; fixed_left=-inf/0.086; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.227/-inf |
| 14 | 21 | 1 | R_REF | stop_in_place=0.227/0.099; fixed_left=-inf/0.093; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.227/-inf |
| 14 | 21 | 1.1 | R_REF | stop_in_place=0.227/0.099; fixed_left=-inf/0.093; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.227/-inf |
| 14 | 21 | 1.2 | R_REF | stop_in_place=0.227/0.099; fixed_left=-inf/0.093; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.227/-inf |
| 14 | 22 | 0 | R_REF | stop_in_place=0.385/0.299; fixed_left=-inf/0.093; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.385/0.093 |
| 14 | 22 | 0.6 | R_REF | stop_in_place=0.385/0.299; fixed_left=-inf/0.093; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.385/-0.007 |
| 14 | 22 | 1.2 | R_REF | stop_in_place=0.385/0.299; fixed_left=-inf/0.093; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.385/-inf |
| 14 | 24 | 0 | R_REF | stop_in_place=0.667/0.667; fixed_left=-inf/0.093; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.667/0.093 |
| 14 | 24 | 0.6 | R_REF | stop_in_place=0.667/0.667; fixed_left=-inf/0.093; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.667/-0.003 |
| 14 | 24 | 1.2 | R_REF | stop_in_place=0.667/0.667; fixed_left=-inf/0.093; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.667/-0.120 |
| 18 | 26 | 0 | R_REF | stop_in_place=0.105/-inf; fixed_left=-inf/0.092; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/0.092 |
| 18 | 26 | 0.2 | R_REF | stop_in_place=0.105/-inf; fixed_left=-inf/0.092; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/0.018 |
| 18 | 26 | 0.4 | U_INFORMATION | stop_in_place=0.105/-inf; fixed_left=-inf/0.092; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/-0.111 |
| 18 | 26 | 0.6 | U_INFORMATION | stop_in_place=0.105/-inf; fixed_left=-inf/0.092; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/-inf |
| 18 | 26 | 0.8 | U_INFORMATION | stop_in_place=0.105/-inf; fixed_left=-inf/0.092; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/-inf |
| 18 | 26 | 1 | U_INFORMATION | stop_in_place=0.105/-inf; fixed_left=-inf/0.092; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.104/-inf |
| 18 | 26 | 1.2 | U_INFORMATION | stop_in_place=0.105/-inf; fixed_left=-inf/0.092; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/-inf |
| 18 | 28 | 0 | R_REF | stop_in_place=0.105/-inf; fixed_left=-inf/0.135; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/0.135 |
| 18 | 28 | 0.2 | R_REF | stop_in_place=0.105/-inf; fixed_left=-inf/0.136; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/0.105 |
| 18 | 28 | 0.4 | R_REF | stop_in_place=0.105/-inf; fixed_left=-inf/0.135; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/0.048 |
| 18 | 28 | 0.6 | U_INFORMATION | stop_in_place=0.105/-inf; fixed_left=-inf/0.135; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/-0.068 |
| 18 | 28 | 0.8 | U_INFORMATION | stop_in_place=0.105/-inf; fixed_left=-inf/0.135; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/-0.200 |
| 18 | 28 | 1 | U_INFORMATION | stop_in_place=0.105/-inf; fixed_left=-inf/0.135; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/-inf |
| 18 | 28 | 1.2 | U_INFORMATION | stop_in_place=0.105/-inf; fixed_left=-inf/0.135; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/-inf |
| 18 | 30 | 0 | R_REF | stop_in_place=0.105/-inf; fixed_left=-inf/0.143; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/0.143 |
| 18 | 30 | 0.2 | R_REF | stop_in_place=0.105/-inf; fixed_left=-inf/0.143; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/0.144 |
| 18 | 30 | 0.4 | R_REF | stop_in_place=0.105/-inf; fixed_left=-inf/0.143; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/0.125 |
| 18 | 30 | 0.6 | R_REF | stop_in_place=0.105/-inf; fixed_left=-inf/0.143; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/0.085 |
| 18 | 30 | 0.8 | R_REF | stop_in_place=0.105/-inf; fixed_left=-inf/0.143; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/0.009 |
| 18 | 30 | 1 | U_INFORMATION | stop_in_place=0.105/-inf; fixed_left=-inf/0.143; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/-0.126 |
| 18 | 30 | 1.2 | U_INFORMATION | stop_in_place=0.105/-inf; fixed_left=-inf/0.143; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/-inf |

![reveal-time transition](occlusion_transition.png)

## Does the transition show on CARLA?

- v 14, D 18: R_REF at t_reveal <= 0 s, U_INFORMATION from 0.1 s (bracket (0.0, 0.1)); analytic t_obs* -0.025 s.
- v 14, D 19: R_REF at t_reveal <= 0.1 s, U_INFORMATION from 0.2 s (bracket (0.1, 0.2)); analytic t_obs* 0.100 s.
- v 14, D 20: R_REF at t_reveal <= 0.3 s, U_INFORMATION from 0.4 s (bracket (0.3, 0.4)); analytic t_obs* 0.225 s.
- v 14, D 21: no clean R_REF -> non-R_REF transition (class string over reveal 0..1.2: RRRRRRRRRRRRR).
- Scenes beyond the analytic stopping distance (a common stop exists): {(14.0, 21.0): ['R_REF', 'R_REF', 'R_REF', 'R_REF', 'R_REF', 'R_REF', 'R_REF', 'R_REF', 'R_REF', 'R_REF', 'R_REF', 'R_REF', 'R_REF'], (14.0, 22.0): ['R_REF', 'R_REF', 'R_REF'], (14.0, 24.0): ['R_REF', 'R_REF', 'R_REF']}. If every entry is R_REF the 'a common stop/robust prefix present => NOT U_INFORMATION' rule is confirmed on CARLA; any other entry is reported as measured.

## RC14: current-view aliasing resolved by past observations

Scene v 14, D 19. Both worlds show `hid` from t = 0 and occlude it again from `t_forget`; they differ only in where it was seen (left lane vs own lane). After `t_forget` the delivered current frame is identical (no objects); the earlier frames differ, and so do the margin profiles. Library adds `branch_current_frame_only` (decides from the current frame alone, forgets the vehicle when it is occluded). G cells read `seen_left_then_occluded/seen_own_then_occluded`.

| t_forget | evidence | margins | profiles differ | memory policy G >= eps in both | current-frame-only policy refuted |
|---|---|---|---|---|---|
| 0.3 | R_REF | stop_in_place=0.105/-inf; fixed_left=-inf/0.081; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/0.081; branch_current_frame_only=0.105/-inf | True | True | True |
| 0.6 | R_REF | stop_in_place=0.105/-inf; fixed_left=-inf/0.081; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/0.081; branch_current_frame_only=0.105/-inf | True | True | True |
| 1 | R_REF | stop_in_place=0.105/-inf; fixed_left=-inf/0.081; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/0.081; branch_current_frame_only=0.105/-inf | True | True | True |

## Audited transition traces

Scene v 14, D 19 (median of the fine-sweep scenes with a transition); the full library was re-run with a per-tick spy at the last R_REF reveal and the first non-R_REF reveal.

**last_R_REF: t_reveal 0.1 s**, evidence R_REF, margins stop_in_place=0.105/-inf; fixed_left=-inf/0.081; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/0.049; re-run vs scan max |dG| = 0.0.

- `branch_own`:
    - run `shared_slowdown_then_branch/hid_own_lane`: termination horizon, contact events none, G (re-run) 0.049
    - ego v 14.01 -> 6.72 m/s; lateral range [-3.84, -0.00] m, end -3.50; x_end 85.6; max per-tick lateral step 0.140 m = 0.443 x (v*dt) (continuous, no teleport)
    - min clearance to each actor (m): {'hid': 1.25}
    - dynamic peaks: {'a_lat': 4.521, 'a_long_smoothed': 6.303, 'a_comb': 7.399, 'a_comb_unmasked': 7.399, 'peak_times': {'a_long': {'t': 0.7, 'v': 11.802}, 'a_comb': {'t': 1.05, 'v': 9.676}, 'a_comb_unmasked': {'t': 1.05, 'v': 9.676}}, 'dyn_margin': {'a_lat': 0.435}}
    - commands (decision_t -> effective_t): brake/None 0.00->0.25, brake/-3.5 0.10->0.35, keep/-3.5 1.35->1.60
    - actor `hid`: v -0.00 -> 0.00, lat_end 0.00, closest |dx| to ego 0.1 m
    - arrival at the hidden vehicle's rear plane: t 1.80 s, ego speed 6.11 m/s, lateral displacement 2.36 m (needed 2.9 m for the reserve), lateral distance to the vehicle centre 2.36 m
- `branch_left`:
    - run `shared_slowdown_then_branch/hid_left_lane`: termination horizon, contact events none, G (re-run) 0.105
    - ego v 14.01 -> -0.00 m/s; lateral range [-0.00, 0.00] m, end -0.00; x_end 17.0; max per-tick lateral step 0.000 m = 0.001 x (v*dt) (continuous, no teleport)
    - min clearance to each actor (m): {'hid': 1.52}
    - dynamic peaks: {'a_lat': 0.029, 'a_long_smoothed': 6.229, 'a_comb': 6.229, 'a_comb_unmasked': 24.433, 'peak_times': {'a_long': {'t': 0.65, 'v': 12.125}, 'a_comb': {'t': 0.65, 'v': 12.125}, 'a_comb_unmasked': {'t': 1.9, 'v': 2.866}}, 'dyn_margin': {'a_lat': 0.996}}
    - commands (decision_t -> effective_t): brake/None 0.00->0.25, brake/0.0 0.10->0.35
    - actor `hid`: v -0.00 -> 0.00, lat_end -3.50, closest |dx| to ego 3.7 m
    - arrival at the hidden vehicle's rear plane: t 1.70 s, ego speed 6.11 m/s, lateral displacement 0.00 m (needed 2.9 m for the reserve), lateral distance to the vehicle centre 3.50 m
- `fixed_left_own`:
    - run `fixed_left/hid_own_lane`: termination horizon, contact events none, G (re-run) 0.081
    - ego v 14.01 -> 6.74 m/s; lateral range [-3.83, -0.00] m, end -3.50; x_end 85.8; max per-tick lateral step 0.144 m = 0.418 x (v*dt) (continuous, no teleport)
    - min clearance to each actor (m): {'hid': 1.41}
    - dynamic peaks: {'a_lat': 4.671, 'a_long_smoothed': 6.359, 'a_comb': 7.569, 'a_comb_unmasked': 7.569, 'peak_times': {'a_long': {'t': 0.7, 'v': 11.791}, 'a_comb': {'t': 0.95, 'v': 10.247}, 'a_comb_unmasked': {'t': 0.95, 'v': 10.247}}, 'dyn_margin': {'a_lat': 0.416}}
    - commands (decision_t -> effective_t): brake/-3.5 0.00->0.25, keep/-3.5 1.35->1.60
    - actor `hid`: v -0.00 -> 0.00, lat_end 0.00, closest |dx| to ego 0.1 m
    - arrival at the hidden vehicle's rear plane: t 1.80 s, ego speed 6.11 m/s, lateral displacement 2.64 m (needed 2.9 m for the reserve), lateral distance to the vehicle centre 2.65 m
- `stop_own`:
    - run `stop_in_place/hid_own_lane`: termination collision, contact events [{'type': 'collision', 'other': 'hid', 'k': 37, 'source': 'obb'}, {'type': 'collision', 'other': 'hid', 'k': 37, 'source': 'sensor', 'impulse': 11268.155578886559}], G (re-run) -inf
    - ego v 14.01 -> -0.46 m/s; lateral range [-0.00, 0.00] m, end -0.00; x_end 16.5; max per-tick lateral step 0.003 m = 0.010 x (v*dt) (continuous, no teleport)
    - min clearance to each actor (m): {'hid': -0.02}
    - dynamic peaks: {'a_lat': 0.133, 'a_long_smoothed': 6.229, 'a_comb': 6.229, 'a_comb_unmasked': 77.904, 'peak_times': {'a_long': {'t': 0.65, 'v': 12.125}, 'a_comb': {'t': 0.65, 'v': 12.125}, 'a_comb_unmasked': {'t': 1.85, 'v': -0.463}}, 'dyn_margin': {'a_lat': 0.983}}
    - commands (decision_t -> effective_t): brake/None 0.00->0.25
    - actor `hid`: v -0.00 -> 0.26, lat_end 0.00, closest |dx| to ego 4.2 m
    - arrival at the hidden vehicle's rear plane: t 1.70 s, ego speed 6.11 m/s, lateral displacement 0.00 m (needed 2.9 m for the reserve), lateral distance to the vehicle centre 0.00 m

**first_non_R_REF: t_reveal 0.2 s**, evidence U_INFORMATION, margins stop_in_place=0.105/-inf; fixed_left=-inf/0.081; fixed_right=0.105/-inf; shared_slowdown_then_branch=0.105/-0.002; re-run vs scan max |dG| = 0.0.

- `branch_own`:
    - run `shared_slowdown_then_branch/hid_own_lane`: termination horizon, contact events none, G (re-run) -0.002
    - ego v 14.01 -> 6.71 m/s; lateral range [-3.85, -0.00] m, end -3.50; x_end 85.4; max per-tick lateral step 0.141 m = 0.463 x (v*dt) (continuous, no teleport)
    - min clearance to each actor (m): {'hid': 0.99}
    - dynamic peaks: {'a_lat': 4.363, 'a_long_smoothed': 6.256, 'a_comb': 7.225, 'a_comb_unmasked': 7.225, 'peak_times': {'a_long': {'t': 0.7, 'v': 11.808}, 'a_comb': {'t': 1.15, 'v': 9.117}, 'a_comb_unmasked': {'t': 1.15, 'v': 9.117}}, 'dyn_margin': {'a_lat': 0.455}}
    - commands (decision_t -> effective_t): brake/None 0.00->0.25, brake/-3.5 0.20->0.45, keep/-3.5 1.35->1.60
    - actor `hid`: v -0.00 -> 0.00, lat_end 0.00, closest |dx| to ego 0.2 m
    - arrival at the hidden vehicle's rear plane: t 1.80 s, ego speed 6.10 m/s, lateral displacement 2.06 m (needed 2.9 m for the reserve), lateral distance to the vehicle centre 2.06 m
- `branch_left`:
    - run `shared_slowdown_then_branch/hid_left_lane`: termination horizon, contact events none, G (re-run) 0.105
    - ego v 14.01 -> 0.00 m/s; lateral range [-0.00, 0.00] m, end -0.00; x_end 17.0; max per-tick lateral step 0.000 m = 0.001 x (v*dt) (continuous, no teleport)
    - min clearance to each actor (m): {'hid': 1.52}
    - dynamic peaks: {'a_lat': 0.029, 'a_long_smoothed': 6.229, 'a_comb': 6.229, 'a_comb_unmasked': 24.433, 'peak_times': {'a_long': {'t': 0.65, 'v': 12.125}, 'a_comb': {'t': 0.65, 'v': 12.125}, 'a_comb_unmasked': {'t': 1.9, 'v': 2.866}}, 'dyn_margin': {'a_lat': 0.996}}
    - commands (decision_t -> effective_t): brake/None 0.00->0.25, brake/0.0 0.20->0.45
    - actor `hid`: v -0.00 -> 0.00, lat_end -3.50, closest |dx| to ego 3.7 m
    - arrival at the hidden vehicle's rear plane: t 1.70 s, ego speed 6.11 m/s, lateral displacement 0.00 m (needed 2.9 m for the reserve), lateral distance to the vehicle centre 3.50 m
- `fixed_left_own`:
    - run `fixed_left/hid_own_lane`: termination horizon, contact events none, G (re-run) 0.081
    - ego v 14.01 -> 6.74 m/s; lateral range [-3.83, -0.00] m, end -3.50; x_end 85.8; max per-tick lateral step 0.144 m = 0.418 x (v*dt) (continuous, no teleport)
    - min clearance to each actor (m): {'hid': 1.41}
    - dynamic peaks: {'a_lat': 4.671, 'a_long_smoothed': 6.359, 'a_comb': 7.569, 'a_comb_unmasked': 7.569, 'peak_times': {'a_long': {'t': 0.7, 'v': 11.791}, 'a_comb': {'t': 0.95, 'v': 10.247}, 'a_comb_unmasked': {'t': 0.95, 'v': 10.247}}, 'dyn_margin': {'a_lat': 0.416}}
    - commands (decision_t -> effective_t): brake/-3.5 0.00->0.25, keep/-3.5 1.35->1.60
    - actor `hid`: v -0.00 -> 0.00, lat_end 0.00, closest |dx| to ego 0.1 m
    - arrival at the hidden vehicle's rear plane: t 1.80 s, ego speed 6.11 m/s, lateral displacement 2.64 m (needed 2.9 m for the reserve), lateral distance to the vehicle centre 2.65 m
- `stop_own`:
    - run `stop_in_place/hid_own_lane`: termination collision, contact events [{'type': 'collision', 'other': 'hid', 'k': 37, 'source': 'obb'}, {'type': 'collision', 'other': 'hid', 'k': 37, 'source': 'sensor', 'impulse': 11268.155578886559}], G (re-run) -inf
    - ego v 14.01 -> -0.46 m/s; lateral range [-0.00, 0.00] m, end -0.00; x_end 16.5; max per-tick lateral step 0.003 m = 0.010 x (v*dt) (continuous, no teleport)
    - min clearance to each actor (m): {'hid': -0.02}
    - dynamic peaks: {'a_lat': 0.133, 'a_long_smoothed': 6.229, 'a_comb': 6.229, 'a_comb_unmasked': 77.904, 'peak_times': {'a_long': {'t': 0.65, 'v': 12.125}, 'a_comb': {'t': 0.65, 'v': 12.125}, 'a_comb_unmasked': {'t': 1.85, 'v': -0.463}}, 'dyn_margin': {'a_lat': 0.983}}
    - commands (decision_t -> effective_t): brake/None 0.00->0.25
    - actor `hid`: v -0.00 -> 0.26, lat_end 0.00, closest |dx| to ego 4.2 m
    - arrival at the hidden vehicle's rear plane: t 1.70 s, ego speed 6.11 m/s, lateral displacement 0.00 m (needed 2.9 m for the reserve), lateral distance to the vehicle centre 0.00 m

Analytic arrival under continuous braking (b_cal, tau_eff): t 1.99 s at 3.49 m/s; measured for `shared_slowdown_then_branch` in `hid_own_lane` at the last R_REF reveal: t 1.80 s at 6.11 m/s.

## Record validation

85 records in `results/carla/occlusion/` checked with validate_records.check_record: 0 failures.
