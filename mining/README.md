# mining

The Python package behind every result: the recoverability contract, the margin G, one harness for every simulator, the case builders for RC01–RC20, the mining algorithms and the dataset export. Run scripts from this folder so outputs land in `results/` here (see [`../scripts/README.md`](../scripts/README.md)).

## Core definitions

| Module | Role |
| --- | --- |
| `contract.py` | The recoverability contract from the PDF: `classify(R)` turns the matrix of G values (moves × worlds) into V, V_hind, info_gap and the evidence class (R_REF, U_LIBRARY, U_INFORMATION, UNRESOLVED) |
| `margin.py` | Label-grade margin G: oriented-box clearance with swept checks, road edges, dynamic limits, terminal test. `DEFAULT_V2` holds the constants |
| `records.py` | ExecutionRecord (one evaluated scene), with checksum and `finalize()` that calls `contract.classify` |
| `validate_records.py` | Schema and consistency checks for record files |

## Simulation (`sim/`)

| Module | Role |
| --- | --- |
| `harness.py` | `run_episode` (one move in one world) and `evaluate` (every move × every world → one record) |
| `plant_carla.py` | CARLA 0.9.16 plant: spawn, warm-up, calibrated braking, step, collision events |
| `plant_kinematic.py` | Deterministic kinematic-bicycle plant with the same interface (mock; used by the tests) |
| `bringup.py` | Connect, build the custom site, calibrate, selftest, determinism, timing |
| `sites.py` | Finds the straight two-lane stretch and converts road ↔ world coordinates |
| `policies.py` | The move library (brake_hold, bypass, staged changes, wait/observe, memory policies, …); policies see only delivered observations |
| `actors.py` | Finite-state controllers for the other vehicles (cruise, yield, pass, …) |
| `queues.py` | Observation delivery (rate, delay, reveal / forget for hidden actors) and the command pipeline (planning and actuation delay) |
| `trackers.py`, `types.py` | Low-level speed and lane controllers; shared types |
| `video.py` | Camera recording for the case videos (visual only) |
| `assets/straight_2lane.xodr` | The custom OpenDRIVE road (1.5 km, two same-direction lanes) |

## Cases (`cases/`)

`catalogue.yaml` is the PDF's 20 cases as data (intentions only, never labels). Each builder makes the scene spec, the move library and the worlds:

| Module | Cases |
| --- | --- |
| `corridor.py` | RC01, RC02, RC12, RC17 |
| `two_lane.py` | RC03, RC07, RC16, RC19, RC20 |
| `compositional.py` | RC04, RC05, RC06, RC08, RC15, RC18 |
| `occlusion.py` | RC10, RC11, RC14 |
| `negatives.py` | RC09, RC13 (and the RC13 predictor, outside the labelling) |

## Mining and export

| Module | Role |
| --- | --- |
| `algos.py` | Ask/tell search algorithms: sobol, cem, bo_lcb, straddle, qd_fail, qd_boundary, bisect |
| `families.py` | Parameter spaces in [0,1]^d for the mined families (corridor, rc03, rc07; CARLA variants) |
| `infogap.py` | Info-gap (V_hind − V) objective helpers for the RC10/RC11 family |
| `export.py`, `splits.py`, `validate_dataset.py` | E1 dataset export, split groups that keep counterfactual neighbours together, dataset validation |
| `reference/` | Closed-form reference models (corridor stopping distance, information tree) used as analytic anchors |

## Earlier version (still used by tests)

`core.py`, `carla_backend.py`, `run.py`, `replay_scene.py` are the first mock-only mining pipeline. Its results are in `../results/mock/`; the CARLA work above replaced it.

## Algorithms and the papers behind them

Checked against the papers (October 2026):
- Corso, Moss, Koren, Lee, Kochenderfer. A Survey of Algorithms for Black-Box Safety Validation of CPS. JAIR 2021. arXiv:2005.02979  -> sobol baseline, cem, bo_lcb
- Bryan, Schneider, Nichol, Moore, Schneider. Active Learning for Identifying Function Threshold Boundaries. NIPS 2005  -> straddle
- Inatsu et al. Active Learning for Level Set Estimation Using Randomized Straddle Algorithms. TMLR 2024. arXiv:2408.03144  -> randomized beta ~ chi2(2)
- Fontaine & Nikolaidis. A Quality Diversity Approach to Automatically Generating HRI Scenarios in Shared Autonomy. arXiv:2012.04283  -> qd_fail / qd_boundary
- Nikolaidis lab. Algorithmic Scenario Generation as Quality Diversity Optimization. arXiv:2409.04711
- CaDRE: Controllable and Diverse Generation of Safety-Critical Driving Scenarios using Real-World Trajectories. arXiv:2403.13208  (QD for driving; CMA-ME)
- Lee et al. Adaptive Stress Testing: Finding Likely Failure Events with RL. JAIR 2020. arXiv:1811.02188 ; Koren et al. AST for Autonomous Vehicles. arXiv:1902.01909  (NOT implemented, see below)
- Fremont et al. Scenic: A Language for Scenario Specification and Data Generation. arXiv:2010.06580  (CARLA scenario language)
- Hanselmann et al. KING (ECCV 2022, arXiv:2204.13683); Rempe et al. STRIVE (CVPR 2022, arXiv:2112.05077)  (gradient / learned-prior attacks + solvability filter)
Cited from memory, re-check before putting in the proposal:
- Mouret & Clune. Illuminating search spaces by mapping elites. arXiv:1504.04909 (MAP-Elites)
- Dreossi et al. VerifAI: A Toolkit for the Formal Design and Analysis of AI-Based Systems. CAV 2019
- Fontaine et al. Covariance Matrix Adaptation for the Rapid Illumination of Behavior Space (CMA-ME). GECCO 2020

Project-specific (no paper): bisect = one-factor boundary bisection that yields matched pairs (proposal Sec. 5.1).
Not implemented on purpose: AST/MCTS. It searches over a *sequence* of stochastic disturbances; our theta is a static
parameter vector. It becomes the right tool once the hidden car's behaviour is a per-step disturbance (occluded-actor family).

