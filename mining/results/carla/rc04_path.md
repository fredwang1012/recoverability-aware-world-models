# RC04 with path-length lane changes (CARLA)

Follow-up to Task 6 (`compositional.md`, RC04: 92 scenes with time-defined lane changes, no witness).  Method and the reason the earlier search found nothing: `scripts/carla_rc04_path.py` docstring.  Library `cases/compositional.rc04_path_library` (brake_hold, lane_follow, immediate_lane_change and staged_brake_p_change for p in (0.4, 0.8, 1.2), all changes over the same path length), a_lat_max 8 m/s^2, tau 0.25 s, decel target 6.19 m/s^2.  Contrast = some staged member >= eps while brake_hold and the immediate change both < eps.

| lc_dist (m) | v (m/s) | gap (m) | evidence | contrast | brake_hold G | immediate G | best staged (G) | immediate peak a_lat (m/s^2) |
|---|---|---|---|---|---|---|---|---|
| 30 | 22 | 36 | U_LIBRARY | no | -inf | -0.904 | staged_brake_0.4_change (-0.767) | 15.23 |
| 30 | 22 | 38 | U_LIBRARY | no | -inf | -0.904 | staged_brake_0.8_change (-0.241) | 15.23 |
| 30 | 22 | 40 | U_LIBRARY | no | -inf | -0.904 | staged_brake_0.8_change (-0.241) | 15.23 |
| 30 | 22 | 42 | U_LIBRARY | no | -0.174 | -0.904 | staged_brake_0.8_change (-0.241) | 15.23 |
| 30 | 22 | 44 | R_REF | no | +0.226 | -0.904 | staged_brake_1.2_change (-0.097) | 15.23 |
| 30 | 22 | 46 | R_REF | no | +0.626 | -0.904 | staged_brake_1.2_change (+0.004) | 15.23 |
| 40 | 25 | 42 | U_LIBRARY | no | -inf | -0.505 | staged_brake_0.4_change (-0.419) | 12.04 |
| 40 | 25 | 44 | U_LIBRARY | no | -inf | -0.505 | staged_brake_0.4_change (-0.419) | 12.04 |
| 40 | 25 | 46 | U_LIBRARY | no | -inf | -0.505 | staged_brake_0.8_change (-0.152) | 12.04 |
| 40 | 25 | 48 | U_LIBRARY | no | -inf | -0.505 | staged_brake_0.8_change (-0.069) | 12.04 |
| 40 | 25 | 50 | U_LIBRARY | no | -inf | -0.505 | staged_brake_0.8_change (-0.027) | 12.04 |
| 40 | 25 | 52 | R_REF | no | +0.163 | -0.505 | staged_brake_0.8_change (-0.027) | 12.04 |
| 40 | 25 | 54 | R_REF | no | +0.563 | -0.505 | staged_brake_0.8_change (-0.027) | 12.04 |
| 40 | 25 | 56 | R_REF | no | +0.667 | -0.505 | staged_brake_1.2_change (+0.005) | 12.04 |
| 40 | 25 | 58 | R_REF | no | +0.667 | -0.505 | staged_brake_1.2_change (+0.071) | 12.04 |
| 40 | 28 | 60 | U_LIBRARY | no | -0.013 | -1.078 | staged_brake_1.2_change (-0.013) | 16.63 |
| 40 | 28 | 62 | R_REF | no | +0.387 | -1.078 | staged_brake_1.2_change (+0.051) | 16.63 |
| 40 | 28 | 64 | R_REF | no | +0.667 | -1.078 | staged_brake_1.2_change (+0.092) | 16.63 |
| 40 | 28 | 66 | R_REF | no | +0.667 | -1.078 | staged_brake_1.2_change (+0.092) | 16.63 |
| 40 | 28 | 68 | R_REF | no | +0.667 | -1.078 | staged_brake_1.2_change (+0.092) | 16.63 |
| 40 | 28 | 70 | R_REF | no | +0.667 | -1.078 | staged_brake_1.2_change (+0.092) | 16.63 |
| 40 | 28 | 72 | R_REF | no | +0.667 | -1.078 | staged_brake_1.2_change (+0.092) | 16.63 |

Scenes with the RC04 contrast on CARLA: 0 of 22: none.

