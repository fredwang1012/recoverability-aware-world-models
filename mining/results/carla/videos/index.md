# CARLA case videos (Task V)

One clip per (case, policy[, world]) of each case's headline scene, re-run live on CARLA with a chase camera (`scripts/carla_videos.py`; clips are git-ignored, `results/carla/videos/*.mp4`).  Visualisation only: labels and records are unchanged.  `stored G` is the margin of the same scene/policy/world in the jsonl cache; `sign check` compares the side of eps = 0 (CARLA repeats are not bit-identical, a DIFFERS row is reported as-is and never re-run).

DIFFERS rows: 0

| file | case | policy | world | verdict | G (this run) | stored G | sign check |
|---|---|---|---|---|---|---|---|
| RC01_brake_hold.mp4 | RC01 | brake_hold | stationary_blocker_no_rear | SAFE: margin G = +0.018 | +0.018 | +0.018 | agree |
| RC02_brake_hold.mp4 | RC02 | brake_hold | stationary_blocker_no_rear | FAILS: margin G = -0.032 (below 0) | -0.032 | -0.032 | agree |
| RC02_bypass_left.mp4 | RC02 | bypass_left | stationary_blocker_no_rear | FAILS: collision | -inf | -inf | agree |
| RC03_bypass_left.mp4 | RC03 | bypass_left | static_blocker | SAFE: margin G = +0.008 | +0.008 | +0.008 | agree |
| RC03_brake_hold.mp4 | RC03 | brake_hold | static_blocker | FAILS: collision | -inf | -inf | agree |
| RC04_staged_brake_0.2_change.mp4 | RC04 | staged_brake_0.2_change | static_blocker | SAFE: margin G = +0.023 | +0.023 | +0.023 | agree |
| RC04_immediate_lane_change.mp4 | RC04 | immediate_lane_change | static_blocker | SAFE: margin G = +0.068 | +0.068 | +0.068 | agree |
| RC05_abort_and_stabilize_gap_closes.mp4 | RC05 | abort_and_stabilize | gap_closes | SAFE: margin G = +0.105 | +0.105 | +0.105 | agree |
| RC05_commit_lane_change_gap_closes.mp4 | RC05 | commit_lane_change | gap_closes | FAILS: collision | -inf | -inf | agree |
| RC06_wait_observe_enter.mp4 | RC06 | wait_observe_enter | passer_cruise | SAFE: margin G = +0.017 | +0.017 | +0.017 | agree |
| RC06_immediate_entry.mp4 | RC06 | immediate_entry | passer_cruise | FAILS: collision | -inf | -inf | agree |
| RC07_bypass_then_return.mp4 | RC07 | bypass_then_return | rear_cruise | SAFE: margin G = +0.119 | +0.119 | +0.119 | agree |
| RC07_bypass_then_stop.mp4 | RC07 | bypass_then_stop | rear_cruise | FAILS: collision | -inf | -inf | agree |
| RC08_brake_hold.mp4 | RC08 | brake_hold | static_blocker | SAFE: margin G = +0.667 | +0.667 | +0.667 | agree |
| RC09_brake_frac_1.0.mp4 | RC09 | brake_frac_1.0 | rear_cruise | FAILS: collision | -inf | -inf | agree |
| RC09_bypass_left_lc2.mp4 | RC09 | bypass_left_lc2 | rear_cruise | FAILS: collision | -inf | -inf | agree |
| RC09_keep_speed.mp4 | RC09 | keep_speed | rear_cruise | FAILS: collision | -inf | -inf | agree |
| RC10_shared_slowdown_then_branch_hid_own_lane.mp4 | RC10 | shared_slowdown_then_branch | hid_own_lane | SAFE: margin G = +0.049 | +0.049 | +0.049 | agree |
| RC11_shared_slowdown_then_branch_hid_own_lane.mp4 | RC11 | shared_slowdown_then_branch | hid_own_lane | FAILS: collision | -inf | -inf | agree |
| RC12_brake_hold_p0.mp4 | RC12 | brake_hold | stationary_blocker_no_rear | SAFE: margin G = +0.668 | +0.668 | +0.668 | agree |
| RC12_brake_hold-prefix_p2.25.mp4 | RC12 | brake_hold@prefix | stationary_blocker_no_rear | FAILS: collision | -inf | -inf | agree |
| RC13_shared_slowdown_then_branch_hid_own_lane.mp4 | RC13 | shared_slowdown_then_branch | hid_own_lane | SAFE: margin G = +0.050 | +0.050 | +0.050 | agree |
| RC14_shared_slowdown_then_branch_seen_own_then_occluded.mp4 | RC14 | shared_slowdown_then_branch | seen_own_then_occluded | SAFE: margin G = +0.081 | +0.081 | +0.081 | agree |
| RC14_branch_current_frame_only_seen_own_then_occluded.mp4 | RC14 | branch_current_frame_only | seen_own_then_occluded | FAILS: collision | -inf | -inf | agree |
| RC15_observe_yield_else_fallback_adj_nonyield.mp4 | RC15 | observe_yield_else_fallback | adj_nonyield | SAFE: margin G = +0.105 | +0.105 | +0.105 | agree |
| RC15_always_merge_adj_nonyield.mp4 | RC15 | always_merge | adj_nonyield | FAILS: collision | -inf | -inf | agree |
| RC16_nudge_left.mp4 | RC16 | nudge_left | static_blocker | SAFE: margin G = +0.045 | +0.045 | +0.045 | agree |
| RC16_lane_follow.mp4 | RC16 | lane_follow | static_blocker | FAILS: margin G = -0.155 (below 0) | -0.155 | -0.155 | agree |
| RC17_brake_hold_tau0.40.mp4 | RC17 | brake_hold | stationary_blocker_no_rear | SAFE: margin G = +0.068 | +0.068 | +0.068 | agree |
| RC17_brake_hold_tau0.45.mp4 | RC17 | brake_hold | stationary_blocker_no_rear | FAILS: margin G = -0.061 (below 0) | -0.061 | -0.061 | agree |
| RC18_bypass_left.mp4 | RC18 | bypass_left | static_blocker | SAFE: margin G = +0.068 | +0.068 | +0.068 | agree |
| RC18_brake_steer_cmd_lc2_b1.mp4 | RC18 | brake_steer_cmd_lc2_b1 | static_blocker | FAILS: margin G = -0.339 (below 0) | -0.339 | -0.339 | agree |
| RC19_lane_follow_anchor.mp4 | RC19 | lane_follow | static_blocker | SAFE: margin G = +0.668 | +0.668 | +0.668 | agree |
| RC19_lane_follow_nuisance.mp4 | RC19 | lane_follow | static_blocker | SAFE: margin G = +0.668 | +0.668 | +0.668 | agree |
| RC20_brake_hold_restricted.mp4 | RC20 | brake_hold | static_blocker | FAILS: collision | -inf | -inf | agree |
| RC20_bypass_left_full.mp4 | RC20 | bypass_left | static_blocker | SAFE: margin G = +0.008 | +0.008 | +0.008 | agree |
