# Time-step sensitivity on CARLA (0.05 s vs 0.025 s)

Method: `scripts/carla_dt_check.py` docstring.  Same server session for both steps; substepping 0.01 s x 10.  G per policy and world; `shift` = G(0.025) - G(0.05) for finite pairs; a sign change across 0 is marked.

| case | scene | evidence at 0.05 | evidence at 0.025 | policy / world | G at 0.05 | G at 0.025 | shift |
|---|---|---|---|---|---|---|---|
| RC01 | gap 16   v 12   tau 0.25 | R_REF | U_LIBRARY **changed** | brake_hold/stationary_blocker_no_rear | +0.018 | -inf | - **sign change** |
| RC02 | gap 15.75   v 12   full library | U_LIBRARY | U_LIBRARY | brake_hold/stationary_blocker_no_rear | -0.032 | -inf | - |
| RC02 | gap 15.75   v 12   full library | U_LIBRARY | U_LIBRARY | bypass_left/stationary_blocker_no_rear | -inf | -inf | - |
| RC02 | gap 15.75   v 12   full library | U_LIBRARY | U_LIBRARY | lane_follow/stationary_blocker_no_rear | -inf | -inf | - |
| RC03 | v 18   gap 30   lc 2.0 | R_REF | R_REF | brake_hold/static_blocker | -inf | -inf | - |
| RC03 | v 18   gap 30   lc 2.0 | R_REF | R_REF | lane_follow/static_blocker | -inf | -inf | - |
| RC03 | v 18   gap 30   lc 2.0 | R_REF | R_REF | bypass_left/static_blocker | +0.008 | +0.014 | +0.006 |
| RC03 | v 18   gap 30   lc 2.0 | R_REF | R_REF | brake_steer_left/static_blocker | -0.163 | -0.147 | +0.017 |
| RC04 | v 22   gap 42   lc 2.0 | R_REF | R_REF | brake_hold/static_blocker | -0.174 | -inf | - |
| RC04 | v 22   gap 42   lc 2.0 | R_REF | R_REF | lane_follow/static_blocker | -inf | -inf | - |
| RC04 | v 22   gap 42   lc 2.0 | R_REF | R_REF | immediate_lane_change/static_blocker | +0.068 | +0.079 | +0.011 |
| RC04 | v 22   gap 42   lc 2.0 | R_REF | R_REF | staged_brake_0.2_change/static_blocker | +0.023 | +0.027 | +0.003 |
| RC04 | v 22   gap 42   lc 2.0 | R_REF | R_REF | staged_brake_0.4_change/static_blocker | -0.030 | -0.025 | +0.005 |
| RC04 | v 22   gap 42   lc 2.0 | R_REF | R_REF | staged_brake_0.6_change/static_blocker | -0.080 | -0.074 | +0.006 |
| RC04 | v 22   gap 42   lc 2.0 | R_REF | R_REF | staged_brake_0.8_change/static_blocker | -0.131 | -0.123 | +0.008 |
| RC04 | v 22   gap 42   lc 2.0 | R_REF | R_REF | staged_brake_1_change/static_blocker | -0.184 | -0.175 | +0.009 |
| RC05 | gap 80   rear gap 35   rear v 20 | R_REF | R_REF | commit_lane_change/gap_closes | -inf | -inf | - |
| RC05 | gap 80   rear gap 35   rear v 20 | R_REF | R_REF | commit_lane_change/gap_open | +0.105 | +0.105 | -0.000 |
| RC05 | gap 80   rear gap 35   rear v 20 | R_REF | R_REF | abort_and_stabilize/gap_closes | +0.105 | +0.105 | -0.000 |
| RC05 | gap 80   rear gap 35   rear v 20 | R_REF | R_REF | abort_and_stabilize/gap_open | +0.416 | +0.419 | +0.003 |
| RC05 | gap 80   rear gap 35   rear v 20 | R_REF | R_REF | abort_if_closing/gap_closes | +0.106 | -inf | - **sign change** |
| RC05 | gap 80   rear gap 35   rear v 20 | R_REF | R_REF | abort_if_closing/gap_open | +0.105 | +0.105 | -0.000 |
| RC06 | gap 60   pass gap 20   pass v 16 | R_REF | R_REF | immediate_entry/passer_cruise | -inf | -inf | - |
| RC06 | gap 60   pass gap 20   pass v 16 | R_REF | R_REF | wait_observe_enter/passer_cruise | +0.017 | -inf | - **sign change** |
| RC06 | gap 60   pass gap 20   pass v 16 | R_REF | R_REF | wait_brake_observe_enter/passer_cruise | +0.105 | -inf | - **sign change** |
| RC06 | gap 60   pass gap 20   pass v 16 | R_REF | R_REF | brake_hold/passer_cruise | +0.105 | +0.105 | -0.000 |
| RC06 | gap 60   pass gap 20   pass v 16 | R_REF | R_REF | lane_follow/passer_cruise | -inf | -inf | - |
| RC07 | gap 34   rear gap 100   rear v 18 | R_REF | R_REF | bypass_then_stop/rear_cruise | -inf | -inf | - |
| RC07 | gap 34   rear gap 100   rear v 18 | R_REF | R_REF | bypass_then_return/rear_cruise | +0.119 | +0.134 | +0.015 |
| RC07 | gap 34   rear gap 100   rear v 18 | R_REF | R_REF | brake_hold/rear_cruise | +0.105 | +0.104 | -0.001 |
| RC07 | gap 34   rear gap 100   rear v 18 | R_REF | R_REF | bypass_left/rear_cruise | +0.127 | +0.134 | +0.007 |
| RC08 | closed lane   gap 45   v 14 | R_REF | R_REF | brake_hold/static_blocker | +0.667 | +0.667 | +0.000 |
| RC08 | closed lane   gap 45   v 14 | R_REF | R_REF | bypass_left/static_blocker | -inf | -inf | - |
| RC08 | closed lane   gap 45   v 14 | R_REF | R_REF | lane_follow/static_blocker | -inf | -inf | - |
| RC09 | v 14   gap 23   rear gap 25 | U_LIBRARY | U_LIBRARY | keep_speed/rear_cruise | -inf | -inf | - |
| RC09 | v 14   gap 23   rear gap 25 | U_LIBRARY | U_LIBRARY | brake_frac_1.0/rear_cruise | -inf | -inf | - |
| RC09 | v 14   gap 23   rear gap 25 | U_LIBRARY | U_LIBRARY | bypass_left_lc2/rear_cruise | -inf | -inf | - |
| RC10 | v 14   D 19   t_reveal 0.1 | R_REF | R_REF | stop_in_place/hid_left_lane | +0.105 | +0.105 | -0.000 |
| RC10 | v 14   D 19   t_reveal 0.1 | R_REF | R_REF | stop_in_place/hid_own_lane | -inf | -inf | - |
| RC10 | v 14   D 19   t_reveal 0.1 | R_REF | R_REF | fixed_left/hid_left_lane | -inf | -inf | - |
| RC10 | v 14   D 19   t_reveal 0.1 | R_REF | R_REF | fixed_left/hid_own_lane | +0.081 | +0.086 | +0.005 |
| RC10 | v 14   D 19   t_reveal 0.1 | R_REF | R_REF | fixed_right/hid_left_lane | +0.105 | +0.105 | +0.000 |
| RC10 | v 14   D 19   t_reveal 0.1 | R_REF | R_REF | fixed_right/hid_own_lane | -inf | -inf | - |
| RC10 | v 14   D 19   t_reveal 0.1 | R_REF | R_REF | shared_slowdown_then_branch/hid_left_lane | +0.105 | +0.104 | -0.001 |
| RC10 | v 14   D 19   t_reveal 0.1 | R_REF | R_REF | shared_slowdown_then_branch/hid_own_lane | +0.049 | +0.058 | +0.009 |
| RC11 | v 14   D 19   t_reveal 0.6 | U_INFORMATION | U_INFORMATION | stop_in_place/hid_left_lane | +0.105 | +0.105 | -0.000 |
| RC11 | v 14   D 19   t_reveal 0.6 | U_INFORMATION | U_INFORMATION | stop_in_place/hid_own_lane | -inf | -inf | - |
| RC11 | v 14   D 19   t_reveal 0.6 | U_INFORMATION | U_INFORMATION | fixed_left/hid_left_lane | -inf | -inf | - |
| RC11 | v 14   D 19   t_reveal 0.6 | U_INFORMATION | U_INFORMATION | fixed_left/hid_own_lane | +0.081 | +0.086 | +0.005 |
| RC11 | v 14   D 19   t_reveal 0.6 | U_INFORMATION | U_INFORMATION | fixed_right/hid_left_lane | +0.105 | +0.105 | +0.000 |
| RC11 | v 14   D 19   t_reveal 0.6 | U_INFORMATION | U_INFORMATION | fixed_right/hid_own_lane | -inf | -inf | - |
| RC11 | v 14   D 19   t_reveal 0.6 | U_INFORMATION | U_INFORMATION | shared_slowdown_then_branch/hid_left_lane | +0.105 | +0.104 | -0.000 |
| RC11 | v 14   D 19   t_reveal 0.6 | U_INFORMATION | U_INFORMATION | shared_slowdown_then_branch/hid_own_lane | -inf | -inf | - |
| RC12 p0 | gap 40   v 12   prefix 0 s | R_REF | R_REF | brake_hold/stationary_blocker_no_rear | +0.668 | +0.665 | -0.003 |
| RC12 p2.25 | gap 40   v 12   prefix 2.25 s | U_LIBRARY | U_LIBRARY | brake_hold@prefix/stationary_blocker_no_rear | -inf | -inf | - |
| RC13 | v 14   D 20   t_reveal 0.2 | R_REF | R_REF | stop_in_place/hid_left_lane | +0.121 | +0.104 | -0.016 |
| RC13 | v 14   D 20   t_reveal 0.2 | R_REF | R_REF | stop_in_place/hid_own_lane | -0.101 | -inf | - |
| RC13 | v 14   D 20   t_reveal 0.2 | R_REF | R_REF | fixed_left/hid_left_lane | -inf | -inf | - |
| RC13 | v 14   D 20   t_reveal 0.2 | R_REF | R_REF | fixed_left/hid_own_lane | +0.093 | +0.121 | +0.028 |
| RC13 | v 14   D 20   t_reveal 0.2 | R_REF | R_REF | fixed_right/hid_left_lane | +0.105 | +0.105 | +0.000 |
| RC13 | v 14   D 20   t_reveal 0.2 | R_REF | R_REF | fixed_right/hid_own_lane | -inf | -inf | - |
| RC13 | v 14   D 20   t_reveal 0.2 | R_REF | R_REF | shared_slowdown_then_branch/hid_left_lane | +0.121 | +0.104 | -0.016 |
| RC13 | v 14   D 20   t_reveal 0.2 | R_REF | R_REF | shared_slowdown_then_branch/hid_own_lane | +0.050 | +0.078 | +0.027 |
| RC14 | v 14   D 19   t_forget 0.6 | R_REF | R_REF | stop_in_place/seen_left_then_occluded | +0.105 | +0.105 | -0.000 |
| RC14 | v 14   D 19   t_forget 0.6 | R_REF | R_REF | stop_in_place/seen_own_then_occluded | -inf | -inf | - |
| RC14 | v 14   D 19   t_forget 0.6 | R_REF | R_REF | fixed_left/seen_left_then_occluded | -inf | -inf | - |
| RC14 | v 14   D 19   t_forget 0.6 | R_REF | R_REF | fixed_left/seen_own_then_occluded | +0.081 | +0.086 | +0.005 |
| RC14 | v 14   D 19   t_forget 0.6 | R_REF | R_REF | fixed_right/seen_left_then_occluded | +0.105 | +0.105 | +0.000 |
| RC14 | v 14   D 19   t_forget 0.6 | R_REF | R_REF | fixed_right/seen_own_then_occluded | -inf | -inf | - |
| RC14 | v 14   D 19   t_forget 0.6 | R_REF | R_REF | shared_slowdown_then_branch/seen_left_then_occluded | +0.105 | +0.104 | -0.000 |
| RC14 | v 14   D 19   t_forget 0.6 | R_REF | R_REF | shared_slowdown_then_branch/seen_own_then_occluded | +0.081 | +0.086 | +0.005 |
| RC14 | v 14   D 19   t_forget 0.6 | R_REF | R_REF | branch_current_frame_only/seen_left_then_occluded | +0.105 | +0.104 | -0.000 |
| RC14 | v 14   D 19   t_forget 0.6 | R_REF | R_REF | branch_current_frame_only/seen_own_then_occluded | -inf | -inf | - |
| RC15 | gap 60   adj gap 35   adj v 16 | R_REF | R_REF | always_merge/adj_yields | +0.083 | +0.105 | +0.022 |
| RC15 | gap 60   adj gap 35   adj v 16 | R_REF | R_REF | always_merge/adj_nonyield | -inf | -inf | - |
| RC15 | gap 60   adj gap 35   adj v 16 | R_REF | R_REF | observe_yield_else_fallback/adj_yields | +0.084 | +0.105 | +0.021 |
| RC15 | gap 60   adj gap 35   adj v 16 | R_REF | R_REF | observe_yield_else_fallback/adj_nonyield | +0.105 | -inf | - **sign change** |
| RC15 | gap 60   adj gap 35   adj v 16 | R_REF | R_REF | brake_hold/adj_yields | +0.105 | +0.104 | -0.001 |
| RC15 | gap 60   adj gap 35   adj v 16 | R_REF | R_REF | brake_hold/adj_nonyield | +0.105 | +0.105 | -0.000 |
| RC15 | gap 60   adj gap 35   adj v 16 | R_REF | R_REF | lane_follow/adj_yields | -inf | -inf | - |
| RC15 | gap 60   adj gap 35   adj v 16 | R_REF | R_REF | lane_follow/adj_nonyield | -inf | -inf | - |
| RC16 | v 12   post offset 1.6 m | R_REF | R_REF | lane_follow/static_blocker | -0.155 | -0.155 | -0.000 |
| RC16 | v 12   post offset 1.6 m | R_REF | R_REF | nudge_left/static_blocker | +0.045 | +0.045 | -0.000 |
| RC16 | v 12   post offset 1.6 m | R_REF | R_REF | brake_hold/static_blocker | +0.668 | +0.665 | -0.003 |
| RC17 tau0.40 | gap 18   v 12   delay 0.40 s | R_REF | U_LIBRARY **changed** | brake_hold/stationary_blocker_no_rear | +0.068 | -inf | - **sign change** |
| RC17 tau0.40 | gap 18   v 12   delay 0.40 s | R_REF | U_LIBRARY **changed** | bypass_left/stationary_blocker_no_rear | -inf | -inf | - |
| RC17 tau0.40 | gap 18   v 12   delay 0.40 s | R_REF | U_LIBRARY **changed** | lane_follow/stationary_blocker_no_rear | -inf | -inf | - |
| RC17 tau0.45 | gap 18   v 12   delay 0.45 s | U_LIBRARY | U_LIBRARY | brake_hold/stationary_blocker_no_rear | -0.061 | -inf | - |
| RC17 tau0.45 | gap 18   v 12   delay 0.45 s | U_LIBRARY | U_LIBRARY | bypass_left/stationary_blocker_no_rear | -inf | -inf | - |
| RC17 tau0.45 | gap 18   v 12   delay 0.45 s | U_LIBRARY | U_LIBRARY | lane_follow/stationary_blocker_no_rear | -inf | -inf | - |
| RC19 anchor | v 8   benign anchor | R_REF | R_REF | lane_follow/static_blocker | +0.668 | +0.668 | +0.000 |
| RC19 nuisance | v 8   nuisance twin | R_REF | R_REF | lane_follow/static_blocker | +0.668 | +0.668 | +0.000 |
| RC20 restricted | v 18   gap 30   restricted library | U_LIBRARY | U_LIBRARY | brake_hold/static_blocker | -inf | -inf | - |
| RC20 restricted | v 18   gap 30   restricted library | U_LIBRARY | U_LIBRARY | lane_follow/static_blocker | -inf | -inf | - |
| RC20 full | v 18   gap 30   full library | R_REF | R_REF | brake_hold/static_blocker | -inf | -inf | - |
| RC20 full | v 18   gap 30   full library | R_REF | R_REF | lane_follow/static_blocker | -inf | -inf | - |
| RC20 full | v 18   gap 30   full library | R_REF | R_REF | bypass_left/static_blocker | +0.008 | +0.014 | +0.006 |
| RC20 full | v 18   gap 30   full library | R_REF | R_REF | brake_steer_left/static_blocker | -0.163 | -0.147 | +0.017 |

23 scenes, 98 policy-world margins.  Evidence-class changes: 2.  Margins changing sign: 6.  Largest finite shift: 0.028; median 0.003.  For scale: repeats within one server session differ by up to 0.028, and across sessions by up to 0.196.

