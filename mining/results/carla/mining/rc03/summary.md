## rc03 mining - backend: CARLA

2 seed(s) x 100 scenes per algorithm (40 for falsifiers: bo_lcb, cem); mean +/- 95% CI (n = 2 seeds, CIs are rough). near_boundary = |V_search| < 0.3; boundary_dist_m = mean |analytic G| of the 20 scenes closest to the boundary (corridor only; reflects sampling density near the boundary, not a localisation error); label_vs_anchor_agree = fraction of R/U-labelled scenes whose label matches the sign of the analytic margin; pair_mid_anchor_*_m = analytic margin at the midpoint of verified bisect pairs (localisation error of the CARLA flip vs the anchor).

| algo | scenes | unrecoverable(U_*) | R_REF | UNRESOLVED | INVALID_RUN | near_boundary | boundary_dist_m | label_vs_anchor_agree | pairs | pairs_verified | pair_mid_anchor_abs_m | pair_mid_anchor_signed_m |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| bisect | 100.00 +/- 0.00 | 39.00 +/- 5.88 | 61.00 +/- 5.88 | 0.00 +/- 0.00 | 0.00 +/- 0.00 | 72.00 +/- 7.84 | n/a | n/a | 11.00 +/- 0.00 | 11.00 +/- 0.00 | n/a | n/a |
| bo_lcb | 40.00 +/- 0.00 | 33.50 +/- 0.98 | 6.50 +/- 0.98 | 0.00 +/- 0.00 | 0.00 +/- 0.00 | 34.00 +/- 0.00 | n/a | n/a | 0.00 +/- 0.00 | 0.00 +/- 0.00 | n/a | n/a |
| cem | 40.00 +/- 0.00 | 30.50 +/- 8.82 | 9.50 +/- 8.82 | 0.00 +/- 0.00 | 0.00 +/- 0.00 | 32.50 +/- 6.86 | n/a | n/a | 0.00 +/- 0.00 | 0.00 +/- 0.00 | n/a | n/a |
| sobol | 100.00 +/- 0.00 | 29.50 +/- 0.98 | 70.50 +/- 0.98 | 0.00 +/- 0.00 | 0.00 +/- 0.00 | 36.50 +/- 2.94 | n/a | n/a | 0.00 +/- 0.00 | 0.00 +/- 0.00 | n/a | n/a |
| straddle | 100.00 +/- 0.00 | 67.50 +/- 4.90 | 32.50 +/- 4.90 | 0.00 +/- 0.00 | 0.00 +/- 0.00 | 89.50 +/- 0.98 | n/a | n/a | 0.00 +/- 0.00 | 0.00 +/- 0.00 | n/a | n/a |

### Evidence mix (total scenes over seeds)

| algo | R_REF | U_LIBRARY |
|---|---|---|
| sobol | 141 | 59 |
| straddle | 65 | 135 |
| bisect | 122 | 78 |
| cem | 19 | 61 |
| bo_lcb | 13 | 67 |
