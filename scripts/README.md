# scripts

Entry points. CARLA scripts need a running server (see [`../docs/CARLA_SETUP.md`](../docs/CARLA_SETUP.md)) and are run **from `mining/`**, so their outputs land in `mining/results/carla/`. Mock and reference scripts run from the repo root. Every CARLA script checkpoints each scene and skips finished ones on a re-run.

## CARLA cases (one script per case group)

| Script | Cases | Output |
| --- | --- | --- |
| `carla_foundation.py` | RC01, RC02, RC16, RC17 (against the analytic stopping and delay anchors) | `results/carla/foundation.md` |
| `carla_core.py` | RC03, RC07, RC12, RC19, RC20 | `results/carla/core.md` |
| `carla_compositional.py` | RC04, RC05, RC06, RC08, RC15, RC18 | `results/carla/compositional.md` |
| `carla_occlusion.py` | RC10, RC11, RC14 (reveal-time sweep and transition figure) | `results/carla/occlusion.md` |
| `carla_negatives.py` | RC09, RC13 | `results/carla/negatives.md` |
| `carla_rc04_path.py` | RC04 with path-length lane changes (22 scenes) | `results/carla/rc04_path.md` |
| `carla_dt_check.py` | The headline scenes at 0.05 s and 0.025 s in one session (time-step sensitivity) | `results/carla/dt_check.md` |
| `carla_videos.py` | Re-runs each headline scene with a chase camera; `--only RC03,RC05`, `--force` | `results/carla/videos/` (local, uncompressed) |
| `publish_videos.py` | No server: compresses the clips into `docs/videos/` and writes its index | `docs/videos/` |

## Mining

| Script | What it does |
| --- | --- |
| `mine_cases.py` | Boundary mining inside a family: `--family corridor|rc03|rc07 --backend mock|carla --algos sobol,straddle,bisect,cem,bo_lcb,qd_fail,qd_boundary --budget N --seeds K` |
| `mining_post.py` | After a CARLA mining run: repeat checks of found scenes (D4) and the recoverability maps |
| `carla_repeat_check.py` | Measures the repeat spread of selected scenes on the live server |
| `mine_infogap.py` | Info-gap mining on the RC10/RC11 family (`--from-records` rebuilds the summary without the simulator) |
| `reference_mining.py` | Mining on closed-form reference families, where the true boundary is known (no simulator) |

## Analysis and reports

| Script | What it does |
| --- | --- |
| `g_sensitivity_carla.py` | Re-scores every stored CARLA record with each margin constant ×0.5 / ×1.5 (no new runs) → `mining/results/carla/g_sensitivity.md` |
| `g_sensitivity_v2.py` | The same question on the analytic corridor model (reference, not CARLA) |
| `build_carla_report.py` | Builds `docs/carla_results.md` from `docs/carla_results.template.md` and the result files |
| `spread_reflag.py` | Re-flags the Task 5 mining claims at the measured repeat spread |

## Earlier mock-only tools

`g_sensitivity.py`, `pnr.py` (point-of-no-return scan) and `proposal_assets.py` (proposal figures) belong to the first mock pipeline (`mining/core.py`); their outputs are mock results, not CARLA.
