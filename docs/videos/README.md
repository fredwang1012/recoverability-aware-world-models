# Case videos

36 clips of 10 to 15 s, one per move in each case's headline scene, re-run on CARLA with a chase camera. They are a visual aid only: labels come from the evidence in the records, never from these runs. Blue = ego, red = parked car, orange = moving car; the road and grass are painted in afterwards because the generated map has no scenery. A car the ego has not been told about yet (RC10, RC11, RC13, RC14) is drawn as a faint ghost marked "not visible to ego" until its reveal time. A run ends at first contact and the clip freezes on a CONTACT banner. Each clip starts with a 1 s title card and ends with a 1 s result card. The RC04 clips are the closest attempt, not a witness: no RC04 scene on CARLA shows the staged move working where both simple moves fail.

Click a file name to open the clip on GitHub. Margin G above 0 means the move keeps the required clearance; the headline margins are only stable to about 0.2 across server sessions (see the [report](../REPORT.md#results-by-case)).

| Case | Clip | Move | World | Outcome | G |
|---|---|---|---|---|---|
| RC01 Braking with a protected stop | [RC01_brake_hold.mp4](RC01_brake_hold.mp4) | `brake_hold` | stationary_blocker_no_rear | safe | +0.018 |
| RC02 Too late to stop in a closed corridor | [RC02_brake_hold.mp4](RC02_brake_hold.mp4) | `brake_hold` | stationary_blocker_no_rear | fails | -0.032 |
| RC02 Too late to stop in a closed corridor | [RC02_bypass_left.mp4](RC02_bypass_left.mp4) | `bypass_left` | stationary_blocker_no_rear | fails | collision / -inf |
| RC03 Braking fails but lateral escape remains | [RC03_brake_hold.mp4](RC03_brake_hold.mp4) | `brake_hold` | static_blocker | fails | collision / -inf |
| RC03 Braking fails but lateral escape remains | [RC03_bypass_left.mp4](RC03_bypass_left.mp4) | `bypass_left` | static_blocker | safe | +0.008 |
| RC04 Staged slowing before a feasible lane change | [RC04_immediate_lane_change.mp4](RC04_immediate_lane_change.mp4) | `immediate_lane_change` | static_blocker | safe | +0.069 |
| RC04 Staged slowing before a feasible lane change | [RC04_staged_brake_0.2_change.mp4](RC04_staged_brake_0.2_change.mp4) | `staged_brake_0.2_change` | static_blocker | safe | +0.023 |
| RC05 Aborting an unsafe lane change | [RC05_abort_and_stabilize_gap_closes.mp4](RC05_abort_and_stabilize_gap_closes.mp4) | `abort_and_stabilize` | gap_closes | safe | +0.105 |
| RC05 Aborting an unsafe lane change | [RC05_commit_lane_change_gap_closes.mp4](RC05_commit_lane_change_gap_closes.mp4) | `commit_lane_change` | gap_closes | fails | collision / -inf |
| RC06 Yield, then use a time-varying gap | [RC06_immediate_entry.mp4](RC06_immediate_entry.mp4) | `immediate_entry` | passer_cruise | fails | collision / -inf |
| RC06 Yield, then use a time-varying gap | [RC06_wait_observe_enter.mp4](RC06_wait_observe_enter.mp4) | `wait_observe_enter` | passer_cruise | safe | +0.017 |
| RC07 Primary avoidance with continued secondary-threat management | [RC07_bypass_then_return.mp4](RC07_bypass_then_return.mp4) | `bypass_then_return` | rear_cruise | safe | +0.119 |
| RC07 Primary avoidance with continued secondary-threat management | [RC07_bypass_then_stop.mp4](RC07_bypass_then_stop.mp4) | `bypass_then_stop` | rear_cruise | fails | collision / -inf |
| RC08 Protected waiting despite route noncompletion | [RC08_brake_hold.mp4](RC08_brake_hold.mp4) | `brake_hold` | static_blocker | safe | +0.667 |
| RC09 Front/rear conflict with no supported escape | [RC09_brake_frac_1.0.mp4](RC09_brake_frac_1.0.mp4) | `brake_frac_1.0` | rear_cruise | fails | collision / -inf |
| RC09 Front/rear conflict with no supported escape | [RC09_bypass_left_lc2.mp4](RC09_bypass_left_lc2.mp4) | `bypass_left_lc2` | rear_cruise | fails | collision / -inf |
| RC09 Front/rear conflict with no supported escape | [RC09_keep_speed.mp4](RC09_keep_speed.mp4) | `keep_speed` | rear_cruise | fails | collision / -inf |
| RC10 Early evidence enables a contingent escape | [RC10_shared_slowdown_then_branch_hid_own_lane.mp4](RC10_shared_slowdown_then_branch_hid_own_lane.mp4) | `shared_slowdown_then_branch` | hid_own_lane | safe | +0.049 |
| RC11 Late evidence prevents robust causal recovery | [RC11_shared_slowdown_then_branch_hid_own_lane.mp4](RC11_shared_slowdown_then_branch_hid_own_lane.mp4) | `shared_slowdown_then_branch` | hid_own_lane | fails | collision / -inf |
| RC12 Recoverability lost by delaying intervention | [RC12_brake_hold-prefix_p2.25.mp4](RC12_brake_hold-prefix_p2.25.mp4) | `brake_hold@prefix` | stationary_blocker_no_rear | fails | collision / -inf |
| RC12 Recoverability lost by delaying intervention | [RC12_brake_hold_p0.mp4](RC12_brake_hold_p0.mp4) | `brake_hold` | stationary_blocker_no_rear | safe | +0.668 |
| RC13 Over-approximated predictions falsely close a corridor | [RC13_shared_slowdown_then_branch_hid_own_lane.mp4](RC13_shared_slowdown_then_branch_hid_own_lane.mp4) | `shared_slowdown_then_branch` | hid_own_lane | safe | +0.050 |
| RC14 Current-view aliasing resolved by past observations | [RC14_branch_current_frame_only_seen_own_then_occluded.mp4](RC14_branch_current_frame_only_seen_own_then_occluded.mp4) | `branch_current_frame_only` | seen_own_then_occluded | fails | collision / -inf |
| RC14 Current-view aliasing resolved by past observations | [RC14_shared_slowdown_then_branch_seen_own_then_occluded.mp4](RC14_shared_slowdown_then_branch_seen_own_then_occluded.mp4) | `shared_slowdown_then_branch` | seen_own_then_occluded | safe | +0.081 |
| RC15 Yielding must be observed, not assumed | [RC15_always_merge_adj_nonyield.mp4](RC15_always_merge_adj_nonyield.mp4) | `always_merge` | adj_nonyield | fails | collision / -inf |
| RC15 Yielding must be observed, not assumed | [RC15_observe_yield_else_fallback_adj_nonyield.mp4](RC15_observe_yield_else_fallback_adj_nonyield.mp4) | `observe_yield_else_fallback` | adj_nonyield | safe | +0.105 |
| RC16 Center-point clearance but body-footprint conflict | [RC16_lane_follow.mp4](RC16_lane_follow.mp4) | `lane_follow` | static_blocker | fails | -0.155 |
| RC16 Center-point clearance but body-footprint conflict | [RC16_nudge_left.mp4](RC16_nudge_left.mp4) | `nudge_left` | static_blocker | safe | +0.045 |
| RC17 Response delay eliminates an earlier recovery option | [RC17_brake_hold_tau0.40.mp4](RC17_brake_hold_tau0.40.mp4) | `brake_hold` | stationary_blocker_no_rear | safe | +0.068 |
| RC17 Response delay eliminates an earlier recovery option | [RC17_brake_hold_tau0.45.mp4](RC17_brake_hold_tau0.45.mp4) | `brake_hold` | stationary_blocker_no_rear | fails | -0.061 |
| RC18 Requested control exceeds available authority | [RC18_brake_steer_cmd_lc2_b1.mp4](RC18_brake_steer_cmd_lc2_b1.mp4) | `brake_steer_cmd_lc2_b1` | static_blocker | fails | -0.343 |
| RC18 Requested control exceeds available authority | [RC18_bypass_left.mp4](RC18_bypass_left.mp4) | `bypass_left` | static_blocker | safe | +0.066 |
| RC19 Benign and label-preserving nuisance controls | [RC19_lane_follow_anchor.mp4](RC19_lane_follow_anchor.mp4) | `lane_follow` | static_blocker | safe | +0.668 |
| RC19 Benign and label-preserving nuisance controls | [RC19_lane_follow_nuisance.mp4](RC19_lane_follow_nuisance.mp4) | `lane_follow` | static_blocker | safe | +0.668 |
| RC20 Incomplete library must remain unresolved | [RC20_brake_hold_restricted.mp4](RC20_brake_hold_restricted.mp4) | `brake_hold` | static_blocker | fails | collision / -inf |
| RC20 Incomplete library must remain unresolved | [RC20_bypass_left_full.mp4](RC20_bypass_left_full.mp4) | `bypass_left` | static_blocker | safe | +0.008 |

Rebuild: `cd mining && python ../scripts/carla_videos.py --force` (needs the CARLA server, see [CARLA_SETUP](../CARLA_SETUP.md)), then `python scripts/publish_videos.py` from the repo root. The uncompressed originals stay in `mining/results/carla/videos/` (local only).
