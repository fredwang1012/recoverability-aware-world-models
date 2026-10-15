# Approach 1 - simulation mining of unrecoverable scenes (CARLA, headless)

Files
- core.py          scenario space theta, causal recovery policies, margin G, oracle V, categories, mock backend
- algos.py         the search algorithms (ask/tell, simulator-agnostic)
- carla_backend.py CARLA 0.9.16 backend (UNTESTED against a live server - see "First run")
- run.py           driver: batching, CSV checkpoints, server pool, metrics, plots, --bench

## Algorithms and the papers behind them
Verified by web search this session:
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

## First run on CARLA
1. ./CarlaUE4.sh -RenderOffScreen -nosound -carla-rpc-port=2000      (one per port for parallelism)
2. python carla_backend.py --calibrate      (measures decel vs tire friction -> friction_calibration.json)
3. python carla_backend.py --selftest       (check: keep holds speed, brake decel ~ brake_limit, left_now ends at lat ~ -lane_w)
4. python run.py --backend carla --ports 2000,2002 --algos sobol,straddle,bisect --budget 60 --out carla_try
5. then the full list, --profiles to attach full R(h) to every mined scene.
If step 3 looks wrong, fix the backend before trusting any CARLA numbers; the mock results say nothing about CARLA dynamics.
