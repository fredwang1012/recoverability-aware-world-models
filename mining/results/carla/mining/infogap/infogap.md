## D5 info-gap mining - backend: CARLA

Family: RC10/RC11 (`cases/occlusion`), theta = (D, v, t_reveal) in D [14, 34] m, v [12, 18] m/s, t_reveal [0, 1.2] s on the 0.05 s tick grid (D to 0.1 m, v to 0.1 m/s).  Objective: maximise V_hind - V subject to V_hind >= eps (eps = 0); an info-gap scene is V_hind >= eps and V < eps.  Search algorithms see only the finite search signal (cem: -(V_hind - V) from R_norm for feasible scenes, penalty otherwise; straddle/sobol: V_search from R_norm); labels, V, V_hind and the objective come from the full record (R_raw, contract.classify).  When V = -inf the label-grade gap is +inf; those scenes are listed separately and never mixed with finite gaps.  2 seed(s) x 9 scenes per algorithm, batch 3; wall 253 s (recorded by the original run; md regenerated from the checkpoints. Original CARLA run (log mining/results/carla/mining_infogap_run.log, md of commit ed3087e).).

### Per algorithm

| algo | scenes | distinct (D,v) | feasible (V_hind >= eps) | info-gap (V < eps) | of which V finite | of which V = -inf | R_REF | max finite gap | evidence mix |
|---|---|---|---|---|---|---|---|---|---|
| cem | 18 | 18 | 13 | 3 | 1 | 2 | 10 | 0.188 | R_REF=10, U_INFORMATION=3, U_LIBRARY=5 |
| straddle | 18 | 18 | 11 | 3 | 1 | 2 | 8 | 0.176 | R_REF=8, U_INFORMATION=3, U_LIBRARY=7 |

### Found info-gap scenes (best first: finite gaps by size, then V = -inf scenes by V_hind)

t* = Task 7 policy-matched analytic reveal threshold for this (D, v) (the reveal time above which the shared-slowdown policy loses its witness); `t_reveal - t*` is the distance after it.  D_stop = common-stop distance at this v (D >= D_stop: a common stop exists, no info gap expected).

| algo | seed | D (m) | v (m/s) | t_reveal (s) | V | V_hind | V_hind - V | t* (s) | t_reveal - t* | D_stop (m) | evidence |
|---|---|---|---|---|---|---|---|---|---|---|---|
| cem | 1 | 26.8 | 17.1 | 0.7 | -0.083 | 0.105 | 0.188 | 0.488 | 0.21 | 29.67 | U_INFORMATION |
| straddle | 1 | 30.6 | 17.9 | 1.1 | -0.040 | 0.136 | 0.176 | 0.780 | 0.32 | 32.16 | U_INFORMATION |
| cem | 1 | 22.6 | 16.1 | 0.55 | -inf | 0.085 | inf | 0.174 | 0.38 | 26.70 | U_INFORMATION |
| straddle | 1 | 23.9 | 17.2 | 1.05 | -inf | 0.063 | inf | 0.103 | 0.95 | 29.98 | U_INFORMATION |
| straddle | 1 | 24.4 | 17.6 | 0.5 | -inf | 0.055 | inf | 0.078 | 0.42 | 31.22 | U_INFORMATION |
| cem | 1 | 20.8 | 16 | 0.75 | -inf | 0.002 | inf | -0.032 | 0.78 | 26.41 | U_INFORMATION |

Comparison with the Task 7 transition: 6 info-gap scene(s); 6 lie below the analytic common-stop distance (a necessary condition); 0 have t_reveal earlier than the analytic t* (Task 7 measured the threshold 0.05 to 0.22 s AFTER t*, so t_reveal slightly below t* is not by itself a contradiction).
Both are necessary conditions only: they do not predict a hit. The converse is checked below: R_REF scenes AFTER t* and below D_stop would be counter-examples to 'late reveal below the stopping distance gives an info gap'; those within 0.2 m of D_stop are the expected edge cases (the analytic D_stop uses b_cal and tau_eff, which carry the plant noise).

### R_REF scenes after t* and below D_stop

| algo | seed | D (m) | v (m/s) | t_reveal (s) | t* (s) | D_stop - D (m) | V | within 0.2 m of D_stop |
|---|---|---|---|---|---|---|---|---|
| straddle | 0 | 20.4 | 13.8 | 1.15 | 0.306 | 0.07 | 0.051 | yes |
| straddle | 1 | 26.8 | 16.2 | 0.9 | 0.678 | 0.19 | 0.114 | yes |
| cem | 1 | 24.5 | 15.7 | 0.5 | 0.490 | 1.05 | 0.044 | no |
Scenes labelled U_INFORMATION that are not info-gap by the objective: 0 (must be 0; U_INFORMATION implies V_hind >= eps and V < eps).
