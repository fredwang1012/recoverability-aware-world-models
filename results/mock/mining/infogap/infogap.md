## D5 info-gap mining - backend: MOCK (MOCK plant: kinematic stand-in, NOT CARLA)

Family: RC10/RC11 (`cases/occlusion`), theta = (D, v, t_reveal) in D [14, 34] m, v [12, 18] m/s, t_reveal [0, 1.2] s on the 0.05 s tick grid (D to 0.1 m, v to 0.1 m/s).  Objective: maximise V_hind - V subject to V_hind >= eps (eps = 0); an info-gap scene is V_hind >= eps and V < eps.  Search algorithms see only the finite search signal (cem: -(V_hind - V) from R_norm for feasible scenes, penalty otherwise; straddle/sobol: V_search from R_norm); labels, V, V_hind and the objective come from the full record (R_raw, contract.classify).  When V = -inf the label-grade gap is +inf; those scenes are listed separately and never mixed with finite gaps.  2 seed(s) x 24 scenes per algorithm, batch 3; wall 265 s.

### Per algorithm

| algo | scenes | distinct (D,v) | feasible (V_hind >= eps) | info-gap (V < eps) | of which V finite | of which V = -inf | R_REF | max finite gap | evidence mix |
|---|---|---|---|---|---|---|---|---|---|
| sobol | 48 | 48 | 39 | 13 | 9 | 4 | 26 | 0.283 | R_REF=26, U_INFORMATION=13, U_LIBRARY=9 |
| cem | 48 | 48 | 43 | 18 | 7 | 11 | 25 | 0.323 | R_REF=25, U_INFORMATION=18, U_LIBRARY=5 |
| straddle | 48 | 47 | 37 | 13 | 5 | 8 | 24 | 0.283 | R_REF=24, U_INFORMATION=13, U_LIBRARY=11 |

### Found info-gap scenes (best first: finite gaps by size, then V = -inf scenes by V_hind)

t* = Task 7 policy-matched analytic reveal threshold for this (D, v) (the reveal time above which the shared-slowdown policy loses its witness); `t_reveal - t*` is the distance after it.  D_stop = common-stop distance at this v (D >= D_stop: a common stop exists, no info gap expected).

| algo | seed | D (m) | v (m/s) | t_reveal (s) | V | V_hind | V_hind - V | t* (s) | t_reveal - t* | D_stop (m) | evidence |
|---|---|---|---|---|---|---|---|---|---|---|---|
| cem | 1 | 21.4 | 15.8 | 0.6 | -0.203 | 0.120 | 0.323 | 0.083 | 0.52 | 25.84 | U_INFORMATION |
| cem | 1 | 21.6 | 16.2 | 0.5 | -0.192 | 0.112 | 0.304 | 0.028 | 0.47 | 26.99 | U_INFORMATION |
| cem | 1 | 22.3 | 16.3 | 0.55 | -0.173 | 0.120 | 0.293 | 0.095 | 0.45 | 27.28 | U_INFORMATION |
| sobol | 1 | 24.4 | 17.6 | 0.5 | -0.163 | 0.120 | 0.283 | 0.078 | 0.42 | 31.22 | U_INFORMATION |
| straddle | 1 | 24.4 | 17.6 | 0.5 | -0.163 | 0.120 | 0.283 | 0.078 | 0.42 | 31.22 | U_INFORMATION |
| sobol | 0 | 17.9 | 14.1 | 0.45 | -0.180 | 0.102 | 0.282 | -0.053 | 0.50 | 21.23 | U_INFORMATION |
| cem | 1 | 22.2 | 16.1 | 0.55 | -0.147 | 0.120 | 0.267 | 0.124 | 0.43 | 26.70 | U_INFORMATION |
| cem | 1 | 22.3 | 15.8 | 0.65 | -0.143 | 0.120 | 0.263 | 0.195 | 0.45 | 25.84 | U_INFORMATION |
| sobol | 0 | 22.9 | 15.2 | 1 | -0.113 | 0.132 | 0.244 | 0.383 | 0.62 | 24.16 | U_INFORMATION |
| sobol | 0 | 26.6 | 17.1 | 0.9 | -0.113 | 0.120 | 0.233 | 0.463 | 0.44 | 29.67 | U_INFORMATION |
| straddle | 1 | 22.2 | 14.9 | 1.2 | -0.092 | 0.138 | 0.230 | 0.350 | 0.85 | 23.34 | U_INFORMATION |
| cem | 1 | 22.6 | 16.1 | 0.55 | -0.103 | 0.120 | 0.223 | 0.174 | 0.38 | 26.70 | U_INFORMATION |
| straddle | 1 | 15.1 | 12 | 0.8 | -0.107 | 0.099 | 0.205 | -0.111 | 0.91 | 16.19 | U_INFORMATION |
| cem | 1 | 21.6 | 15.6 | 0.5 | -0.080 | 0.120 | 0.200 | 0.146 | 0.35 | 25.27 | U_INFORMATION |
| sobol | 1 | 30.1 | 17.8 | 1.15 | -0.061 | 0.120 | 0.181 | 0.741 | 0.41 | 31.85 | U_INFORMATION |
| sobol | 1 | 21.4 | 15.2 | 0.55 | -0.058 | 0.120 | 0.178 | 0.196 | 0.35 | 24.16 | U_INFORMATION |
| straddle | 1 | 29.7 | 18 | 1 | -0.046 | 0.120 | 0.166 | 0.644 | 0.36 | 32.48 | U_INFORMATION |
| straddle | 1 | 16.9 | 13.5 | 0.25 | -0.059 | 0.092 | 0.151 | -0.086 | 0.34 | 19.72 | U_INFORMATION |
| sobol | 1 | 18.7 | 13.4 | 1.2 | -0.031 | 0.099 | 0.130 | 0.154 | 1.05 | 19.47 | U_INFORMATION |
| sobol | 0 | 25.5 | 17.3 | 0.5 | -0.006 | 0.120 | 0.126 | 0.281 | 0.22 | 30.28 | U_INFORMATION |
| sobol | 1 | 14.2 | 12 | 0.05 | -0.006 | 0.027 | 0.033 | -0.224 | 0.27 | 16.19 | U_INFORMATION |
| sobol | 1 | 23.9 | 17.2 | 1.05 | -inf | 0.120 | inf | 0.103 | 0.95 | 29.98 | U_INFORMATION |
| sobol | 1 | 20.7 | 14.8 | 0.95 | -inf | 0.120 | inf | 0.180 | 0.77 | 23.07 | U_INFORMATION |
| cem | 1 | 21.3 | 15.9 | 0.6 | -inf | 0.120 | inf | 0.051 | 0.55 | 26.12 | U_INFORMATION |
| straddle | 0 | 24.1 | 16.2 | 1.1 | -inf | 0.120 | inf | 0.341 | 0.76 | 26.99 | U_INFORMATION |
| straddle | 0 | 26.5 | 17.5 | 1.1 | -inf | 0.120 | inf | 0.361 | 0.74 | 30.90 | U_INFORMATION |
| straddle | 1 | 23.9 | 17.2 | 1.05 | -inf | 0.120 | inf | 0.103 | 0.95 | 29.98 | U_INFORMATION |
| cem | 1 | 21.1 | 15.9 | 0.6 | -inf | 0.113 | inf | 0.026 | 0.57 | 26.12 | U_INFORMATION |
| straddle | 1 | 19.3 | 14.9 | 0.9 | -inf | 0.111 | inf | -0.013 | 0.91 | 23.34 | U_INFORMATION |
| straddle | 0 | 24.6 | 17.9 | 1.15 | -inf | 0.110 | inf | 0.041 | 1.11 | 32.16 | U_INFORMATION |
| cem | 1 | 21.9 | 16.4 | 0.65 | -inf | 0.109 | inf | 0.025 | 0.63 | 27.57 | U_INFORMATION |
| cem | 1 | 21.4 | 16.2 | 0.55 | -inf | 0.101 | inf | 0.003 | 0.55 | 26.99 | U_INFORMATION |
| cem | 1 | 21.2 | 16.1 | 0.5 | -inf | 0.100 | inf | -0.001 | 0.50 | 26.70 | U_INFORMATION |
| cem | 1 | 21.3 | 16.2 | 0.55 | -inf | 0.095 | inf | -0.009 | 0.56 | 26.99 | U_INFORMATION |
| cem | 1 | 21.6 | 16.4 | 0.55 | -inf | 0.093 | inf | -0.013 | 0.56 | 27.57 | U_INFORMATION |
| straddle | 0 | 19.1 | 14 | 1.15 | -inf | 0.092 | inf | 0.113 | 1.04 | 20.98 | U_INFORMATION |
| straddle | 1 | 18.9 | 14 | 1.2 | -inf | 0.092 | inf | 0.088 | 1.11 | 20.98 | U_INFORMATION |
| sobol | 1 | 21.5 | 16.4 | 0.65 | -inf | 0.087 | inf | -0.025 | 0.68 | 27.57 | U_INFORMATION |
| cem | 1 | 20.8 | 16 | 0.75 | -inf | 0.086 | inf | -0.032 | 0.78 | 26.41 | U_INFORMATION |
| cem | 1 | 21.1 | 16.4 | 0.45 | -inf | 0.063 | inf | -0.074 | 0.52 | 27.57 | U_INFORMATION |
| straddle | 0 | 14.8 | 12.3 | 1 | -inf | 0.049 | inf | -0.185 | 1.18 | 16.87 | U_INFORMATION |
| cem | 1 | 20.5 | 16.2 | 0.6 | -inf | 0.045 | inf | -0.108 | 0.71 | 26.99 | U_INFORMATION |
| sobol | 0 | 21.6 | 17 | 0.75 | -inf | 0.033 | inf | -0.126 | 0.88 | 29.37 | U_INFORMATION |
| cem | 1 | 20.4 | 16.3 | 0.6 | -inf | 0.026 | inf | -0.139 | 0.74 | 27.28 | U_INFORMATION |

Comparison with the Task 7 transition: 44 info-gap scene(s); 44 lie below the analytic common-stop distance (a necessary condition); 0 have t_reveal earlier than the analytic t* (Task 7 measured the threshold 0.05 to 0.22 s AFTER t*, so t_reveal slightly below t* is not by itself a contradiction).
Scenes labelled U_INFORMATION that are not info-gap by the objective: 0 (must be 0; U_INFORMATION implies V_hind >= eps and V < eps).
