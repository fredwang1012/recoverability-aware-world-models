## CARLA physics profile pp_381b4c6e55 (Tesla Model 3, dt=0.05)

knob: max_brake_torque at fixed tire_friction 5.0; default torque 700
decel range 4.77..17.51 m/s^2; monotone in torque: True (strict: True); covers 5-9 m/s^2: True
Note: torque factors 0.1 and 0.25 were added to reach 5-9 m/s^2 once gear 1 is engaged; the low end (~4.8-6 m/s^2) is engine-drag dominated, not a constant brake decel.
mean resid RMS 0.275 m/s (max 0.375); max repeat difference 0.000 m/s^2; response delay 0.05 s

### Torque sweep (friction fixed)

| torque factor | max_brake_torque | decel (m/s^2) | repeat diff | resid RMS (m/s) | onset delay (s) | max abs yaw (rad) |
|---|---|---|---|---|---|---|
| 0.1 | 70 | 4.77 | 0.000 | 0.375 | 0.05 | 0.0000 |
| 0.25 | 175 | 5.35 | 0.000 | 0.344 | 0.05 | 0.0005 |
| 0.5 | 350 | 6.19 | 0.000 | 0.285 | 0.05 | 0.0024 |
| 0.75 | 525 | 7.13 | 0.000 | 0.284 | 0.05 | 0.0004 |
| 1.0 | 700 | 7.91 | 0.000 | 0.218 | 0.05 | 0.0039 |
| 1.5 | 1050 | 10.06 | 0.000 | 0.361 | 0.05 | 0.0006 |
| 2.0 | 1400 | 11.88 | 0.000 | 0.296 | 0.05 | 0.0035 |
| 2.5 | 1750 | 13.42 | 0.000 | 0.261 | 0.05 | 0.0034 |
| 3.0 | 2100 | 14.87 | 0.000 | 0.231 | 0.05 | 0.0022 |
| 4.0 | 2800 | 17.51 | 0.000 | 0.095 | 0.05 | 0.0018 |

### Friction sweep (torque fixed at default 700)

| friction | decel (m/s^2) | repeat diff | resid RMS (m/s) | onset delay (s) |
|---|---|---|---|---|
| 0.6 | 4.78 | 0.000 | 0.030 | 0.05 |
| 1.0 | 6.24 | 0.000 | 0.079 | 0.05 |
| 1.5 | 7.20 | 0.000 | 0.374 | 0.05 |
| 2.5 | 7.42 | 0.000 | 0.340 | 0.05 |
| 3.5 | 7.73 | 0.000 | 0.288 | 0.05 |
| 5.0 | 7.91 | 0.000 | 0.218 | 0.05 |

### Steer response (12 m/s, constant steer, yaw unwrapped; positive steer expected -> positive road-frame yaw)

- {'steer_cmd': 0.02, 'yaw_rate_ss': 0.06582141201731306, 't90_s': 0.05, 'yaw_rate_per_steer': 3.291070600865653, 'sign_ok': True, 'v_start': 11.953885099985825, 'v_end': 7.172457050123336, 'total_yaw_rad': 0.12718778426344865}
- {'steer_cmd': 0.05, 'yaw_rate_ss': 0.1470126673771516, 't90_s': 0.0, 'yaw_rate_per_steer': 2.940253347543032, 'sign_ok': True, 'v_start': 11.949265184333143, 'v_end': 7.750064365109807, 'total_yaw_rad': 0.16762440471157053}
- {'steer_cmd': 0.1, 'yaw_rate_ss': 0.22370118728512783, 't90_s': 0.0, 'yaw_rate_per_steer': 2.2370118728512782, 'sign_ok': True, 'v_start': 11.937321559847138, 'v_end': 8.402648882499605, 'total_yaw_rad': 0.14201815842406015}

lateral tracking (3 s lane change @12 m/s): {'max_abs_err_m': 0.3737402034505215, 'rms_err_m': 0.19535846741612264, 'final_lat_m': -3.501168370246887, 'target_lat_m': -3.5}
combined brake+steer: {'decel': 9.813905491215914, 'resid_rms': 0.22008010318318175, 'onset_delay_s': 0.05, 'a_lat_max': 2.0510082311185047, 'max_abs_yaw': 0.1280740550158654, 'max_abs_lat': 1.184180736541748}
