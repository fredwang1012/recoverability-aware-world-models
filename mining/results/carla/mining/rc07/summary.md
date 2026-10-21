## rc07 mining - backend: CARLA

bisect: 6 bisection iterations per chain; 2 seed(s) x 100 scenes per algorithm (40 for falsifiers: bo_lcb, cem, qd_boundary, qd_fail); mean +/- 95% CI (n = 2 seeds, CIs are rough). near_boundary = |V_search| < 0.3; boundary_dist_m = mean |analytic G| of the 20 scenes closest to the boundary (corridor only; reflects sampling density near the boundary, not a localisation error); label_vs_anchor_agree = fraction of R/U-labelled scenes whose label matches the sign of the analytic margin; pair_mid_anchor_*_m = analytic margin at the midpoint of verified bisect pairs (localisation error of the CARLA flip vs the anchor).

| algo | scenes | unrecoverable(U_*) | R_REF | UNRESOLVED | INVALID_RUN | near_boundary | boundary_dist_m | label_vs_anchor_agree | pairs | distinct_pairs | pairs_verified | pair_mid_anchor_abs_m | pair_mid_anchor_signed_m |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| bisect | 100.00 +/- 0.00 | 42.50 +/- 10.78 | 57.50 +/- 10.78 | 0.00 +/- 0.00 | 0.00 +/- 0.00 | 100.00 +/- 0.00 | n/a | n/a | 8.00 +/- 1.96 | 7.00 +/- 0.00 | 8.00 +/- 1.96 | n/a | n/a |
| bo_lcb | 40.00 +/- 0.00 | 30.50 +/- 2.94 | 9.50 +/- 2.94 | 0.00 +/- 0.00 | 0.00 +/- 0.00 | 40.00 +/- 0.00 | n/a | n/a | 0.00 +/- 0.00 | 0.00 +/- 0.00 | 0.00 +/- 0.00 | n/a | n/a |
| cem | 40.00 +/- 0.00 | 11.50 +/- 22.54 | 28.50 +/- 22.54 | 0.00 +/- 0.00 | 0.00 +/- 0.00 | 40.00 +/- 0.00 | n/a | n/a | 0.00 +/- 0.00 | 0.00 +/- 0.00 | 0.00 +/- 0.00 | n/a | n/a |
| qd_boundary | 40.00 +/- 0.00 | 4.00 +/- 7.84 | 36.00 +/- 7.84 | 0.00 +/- 0.00 | 0.00 +/- 0.00 | 40.00 +/- 0.00 | n/a | n/a | 0.00 +/- 0.00 | 0.00 +/- 0.00 | 0.00 +/- 0.00 | n/a | n/a |
| qd_fail | 40.00 +/- 0.00 | 3.50 +/- 6.86 | 36.50 +/- 6.86 | 0.00 +/- 0.00 | 0.00 +/- 0.00 | 40.00 +/- 0.00 | n/a | n/a | 0.00 +/- 0.00 | 0.00 +/- 0.00 | 0.00 +/- 0.00 | n/a | n/a |
| sobol | 100.00 +/- 0.00 | 6.50 +/- 0.98 | 93.50 +/- 0.98 | 0.00 +/- 0.00 | 0.00 +/- 0.00 | 100.00 +/- 0.00 | n/a | n/a | 0.00 +/- 0.00 | 0.00 +/- 0.00 | 0.00 +/- 0.00 | n/a | n/a |
| straddle | 100.00 +/- 0.00 | 29.50 +/- 0.98 | 70.50 +/- 0.98 | 0.00 +/- 0.00 | 0.00 +/- 0.00 | 100.00 +/- 0.00 | n/a | n/a | 0.00 +/- 0.00 | 0.00 +/- 0.00 | 0.00 +/- 0.00 | n/a | n/a |

### Evidence mix (total scenes over seeds)

| algo | R_REF | U_LIBRARY |
|---|---|---|
| sobol | 187 | 13 |
| straddle | 141 | 59 |
| bisect | 115 | 85 |
| cem | 57 | 23 |
| bo_lcb | 19 | 61 |
| qd_fail | 73 | 7 |
| qd_boundary | 72 | 8 |
