## Repeat spread of the same CARLA scene (input to D4)

10 RC12 corridor scenes (gap 40, v 12, tau 0.25, brake_hold, prefix varied) re-run twice in the current server session; the old records in `results/carla/core/rc12_prefix.jsonl` were produced before the server restart (same physics_profile_id). Margin G is normalised (clearance metres / q_clear = 5.0); metres = |dG| * 5.0.

| prefix_s | G (pre-restart) | G (rep 1) | G (rep 2) | same-session |dG| | cross-session |dG| | label old / new |
|---|---|---|---|---|---|---|
| 0.00 | 0.66826 | 0.66826 | 0.66826 | 0.00000 | 0.00000 | R_REF / R_REF |
| 0.25 | 0.66826 | 0.66826 | 0.66826 | 0.00000 | 0.00000 | R_REF / R_REF |
| 0.50 | 0.66826 | 0.66826 | 0.66826 | 0.00000 | 0.00000 | R_REF / R_REF |
| 0.75 | 0.66826 | 0.66826 | 0.66826 | 0.00000 | 0.00000 | R_REF / R_REF |
| 1.00 | 0.66826 | 0.66826 | 0.66826 | 0.00000 | 0.00000 | R_REF / R_REF |
| 1.25 | 0.66826 | 0.66826 | 0.66826 | 0.00000 | 0.00000 | R_REF / R_REF |
| 1.50 | 0.66826 | 0.66826 | 0.66826 | 0.00000 | 0.00000 | R_REF / R_REF |
| 1.75 | 0.61991 | 0.61991 | 0.61991 | 0.00000 | 0.00000 | R_REF / R_REF |
| 2.00 | 0.03246 | 0.03246 | 0.03246 | 0.00000 | 0.00000 | R_REF / R_REF |
| 2.05 | -0.09755 | -0.09755 | -0.09755 | 0.00000 | 0.00000 | U_LIBRARY / U_LIBRARY |

Max same-session spread: 0.0000 m-equivalent.  Max cross-session spread (restart in between): 0.0000 m-equivalent (small sample of 10 scenes, one family).  RC12-check sigma = max(0.05 m, observed here) = 0.0500 m = 0.01000 in margin units.

Caveat: this is 10 scenes of ONE family and 7 of them are saturated (G = 0.66826, the stopped-vehicle clearance), so it does NOT establish zero cross-session or same-session spread. The spread observed over the D4 repeats of mined scenes near the boundary is larger (see `corridor/d4.md`, `rc03/d4.md`: up to ~0.017 margin units) and sets sigma.
