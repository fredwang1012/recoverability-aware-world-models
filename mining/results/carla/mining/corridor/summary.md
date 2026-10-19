## corridor mining - backend: CARLA

2 seed(s) x 100 scenes per algorithm (40 for falsifiers: bo_lcb, cem); mean +/- 95% CI (n = 2 seeds, CIs are rough). near_boundary = |V_search| < 0.3; boundary_dist_m = mean |analytic G| of the 20 scenes closest to the boundary (corridor only; reflects sampling density near the boundary, not a localisation error); label_vs_anchor_agree = fraction of R/U-labelled scenes whose label matches the sign of the analytic margin; pair_mid_anchor_*_m = analytic margin at the midpoint of verified bisect pairs (localisation error of the CARLA flip vs the anchor).

| algo | scenes | unrecoverable(U_*) | R_REF | UNRESOLVED | INVALID_RUN | near_boundary | boundary_dist_m | label_vs_anchor_agree | pairs | pairs_verified | pair_mid_anchor_abs_m | pair_mid_anchor_signed_m |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| bisect | 100.00 +/- 0.00 | 55.50 +/- 12.74 | 44.50 +/- 12.74 | 0.00 +/- 0.00 | 0.00 +/- 0.00 | 75.00 +/- 1.96 | 0.45 +/- 0.11 | 0.86 +/- 0.06 | 7.50 +/- 0.98 | 7.50 +/- 0.98 | 0.97 +/- 0.31 | -0.97 +/- 0.31 |
| bo_lcb | 40.00 +/- 0.00 | 32.50 +/- 0.98 | 7.50 +/- 0.98 | 0.00 +/- 0.00 | 0.00 +/- 0.00 | 33.50 +/- 2.94 | 7.64 +/- 3.15 | 0.97 +/- 0.00 | 0.00 +/- 0.00 | 0.00 +/- 0.00 | n/a | n/a |
| cem | 40.00 +/- 0.00 | 32.50 +/- 2.94 | 7.50 +/- 2.94 | 0.00 +/- 0.00 | 0.00 +/- 0.00 | 33.00 +/- 1.96 | 6.56 +/- 5.27 | 0.99 +/- 0.02 | 0.00 +/- 0.00 | 0.00 +/- 0.00 | n/a | n/a |
| sobol | 100.00 +/- 0.00 | 48.50 +/- 0.98 | 51.50 +/- 0.98 | 0.00 +/- 0.00 | 0.00 +/- 0.00 | 51.50 +/- 0.98 | 2.17 +/- 0.28 | 0.97 +/- 0.01 | 0.00 +/- 0.00 | 0.00 +/- 0.00 | n/a | n/a |
| straddle | 100.00 +/- 0.00 | 71.50 +/- 0.98 | 28.50 +/- 0.98 | 0.00 +/- 0.00 | 0.00 +/- 0.00 | 85.50 +/- 6.86 | 0.64 +/- 0.30 | 0.92 +/- 0.00 | 0.00 +/- 0.00 | 0.00 +/- 0.00 | n/a | n/a |

### Evidence mix (total scenes over seeds)

| algo | R_REF | U_LIBRARY |
|---|---|---|
| sobol | 103 | 97 |
| straddle | 57 | 143 |
| bisect | 89 | 111 |
| cem | 15 | 65 |
| bo_lcb | 15 | 65 |
