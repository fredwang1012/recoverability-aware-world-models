# Approach 1 — Simulation Mining of Unrecoverable Scenes
## Research findings, algorithm specifications, and design decisions

Project: CSC2626, *Recoverability-Preserving World Models for Safety-Critical Imitation Learning*
Owner of this part: Fred (Approach 1). Written 15 Oct 2026.
Companion files: `CLAUDE_CODE_PROMPT.md` (hand this to Claude Code), `mining/` (working code).

**How to read the status tags.** `[verified]` = I read the source this session by web search. `[memory]` = cited from my own knowledge, re-check before it goes in the proposal. `[mine]` = my own construction, no paper. `[untested]` = code/idea not yet run against a live CARLA server.

---

## 0. Executive summary (10 bullets)

1. **What "unrecoverable" should mean.** Use the proposal's definition, not "a crash happened": a scene θ is unrecoverable *relative to* a finite library of causal recovery policies Π, a finite set of hidden worlds Ω, a horizon, and a margin threshold ε, when `V(θ) = max_j min_ω G(π_j, ω; θ) < ε`. It is a *label relative to Π and Ω*, never a physical-impossibility claim.
2. **It is a known concept under other names.** Inevitable Collision States (Fraichard), Time-To-React (max over a maneuver set of time-to-maneuver), Brake/Steer Threat Number ≥ 1 ("point of no return"), and Hamilton–Jacobi "collision-avoidance reachable sets". Our simulation-based V is a finite-library, finite-Ω approximation of these. Citing them makes the definition defensible.
3. **Errors go both ways.** A too-small Π over-reports unrecoverable (a better policy may exist). A too-small Ω over-reports recoverable (a nastier world may exist). Both must be stated in the proposal.
4. **Search is a black-box falsification + level-set problem.** The right toolbox is the safety-validation literature (Corso et al., JAIR 2021): random/QMC baseline, cross-entropy, Bayesian optimization, adaptive stress testing, importance sampling. For *edge cases on the boundary* the better fit is **active level-set estimation (straddle)**, and for *coverage* it is **quality-diversity (MAP-Elites)**.
5. **Seven algorithms are implemented** (`mining/algos.py`): `sobol`, `cem`, `bo_lcb`, `straddle`, `qd_fail`, `qd_boundary`, `bisect`. Adaptive stress testing is deliberately not implemented (static θ, not a disturbance sequence).
6. **Mock-backend benchmark (3 seeds × 100 scenes).** `straddle` finds ~5× more near-boundary scenes than random and the widest spread of unrecoverable situations; `bo_lcb`/`cem` are crash-finders (85–90% unrecoverable, ~0 near boundary); `bisect` is the only source of one-factor matched pairs; MAP-Elites is under-budgeted at 100 scenes. **These are not CARLA results.**
7. **CARLA backend is written but untested against a live server.** API names were checked against the 0.9.16 client wheel. First run must be `--calibrate` then `--selftest`.
8. **Use CARLA 0.9.16, not 0.10.0.** 0.9.16 has the Cosmos Transfer1 integration and is what Autoware's Open AD Kit CARLA sample targets. 0.10.0 (UE5.5) is still missing migrated functionality and wants ≥16 GB VRAM.
9. **Scenic/VerifAI compatibility risk.** Scenic's documented CARLA support is 0.9.9–0.9.14. Use our own backend; optionally use VerifAI's samplers as a *baseline*, not as the pipeline.
10. **Cosmos Transfer changes appearance, not dynamics.** The meeting-notes step "use Cosmos to generate variations of unrecoverable cases" yields *perceptual* variations (weather, lighting, textures) conditioned on CARLA output. Dynamic variations (different gaps, speeds, delays) must come from the search itself. This matches the proposal's "appearance-only variations" in §5.1 and should be stated explicitly to the team.

---

## 1. Problem formalization

### 1.1 Objects

| Symbol | Meaning | In code |
|---|---|---|
| θ ∈ Θ ⊂ ℝ^d | scenario parameters (d = 6 today) | `core.SPACE`, `core.decode` |
| Ω | finite set of hidden actor mechanisms consistent with the same observation history | `core.WORLDS` = constant, IDM-reactive, accelerating |
| Π = {π_1…π_N} | library of **causal** recovery policies (observation → command; never sees the world id) | `core.LIBRARY` (6 policies) |
| G(π, ω; θ) | normalized margin of one rollout, computed from privileged state, labels only | `core.margin` |
| r_j(θ) = min_{ω∈Ω} G(π_j, ω; θ) | worst-case margin of policy j | `core.oracle` |
| V(θ) = max_j r_j(θ) | best worst-case margin the library achieves | `core.oracle` |
| R(θ) = [r_1…r_N] | **recovery profile** (the world model's training target) | `oracle(full=True)` |

Classes (EPS = 0, EDGE_BAND = 0.3):

- `unrecoverable`: V < ε
- `planner_failure`: V ≥ ε (a recovery exists) but the planner under test fails (STRIVE-style solvability filter)
- `edge`: ε ≤ V < ε + band (recoverable, barely)
- `safe`: otherwise

### 1.2 Evidence status of a label (from proposal §4)

- *verified witness*: some π_j has r_j ≥ ε in all ω ∈ Ω → **recoverable** is a positive claim, backed by an execution.
- *verified counterexample for π_j*: r_j < ε for a specific (π_j, ω).
- *unresolved*: not evaluated (pruned or not run). Mask these in training; never invent a target.
- "Unrecoverable" = every policy has a counterexample. It is **a statement about Π, not about physics.**

### 1.3 Direction of errors (state this in the proposal)

| Approximation | Failure mode | Consequence |
|---|---|---|
| Π too small | a better recovery exists but is not in the library | scene wrongly labelled **unrecoverable** |
| Ω too small | the true uncertainty set contains a worse world | scene wrongly labelled **recoverable** |
| Sim fidelity | CARLA tire/brake model differs from reality | both directions |
| Horizon too short | failure occurs after the horizon | wrongly **recoverable** |

Mitigations: (a) the proposal's transfer test (hold out policies — if V changes a lot, the library is not saturated); (b) grow Π until the set of unrecoverable scenes stops shrinking (a convergence curve is itself a result); (c) add worlds until the recoverable set stops shrinking.

### 1.4 Related concepts (why our definition is standard)

- **Inevitable Collision States (Fraichard).** A state from which, whatever trajectory is followed, a collision eventually occurs, accounting for dynamics of the system and the obstacles `[verified, via search result]`. Our V<ε is a finite-library under-approximation of the ICS set. `[verified]`
- **Time To React (TTR).** Latest time until a reaction is required, aggregated as the maximum Time-To-Maneuver over a set of maneuvers (e.g. brake, steer, kickdown) `[verified]` — this is exactly "max over a policy library". Our **point-of-no-return scan** (clone the state at each step of a nominal rollout and re-label) is a simulation-based TTR over Π. Use the name in the paper.
- **Brake Threat Number (BTN).** Required longitudinal acceleration divided by the maximum available; BTN ≥ 1 means a braking maneuver cannot avoid the accident ("point of no return") `[verified]`. Use as an **analytic oracle check** for the brake-only policy: `BTN = (v²/(2·gap_eff)) / a_max`.
- **Steer Threat Number (STN)** and TTS (Time To Steer) cover the lateral escape; same logic with lateral acceleration. `[verified as existing metrics; formulas not re-derived]`
- **Hamilton–Jacobi reachability.** The backward reachable set from the collision set; "collision avoidance reachable sets (CARS)" are the states from which collision is unavoidable `[verified]`. Exact for low-dimensional models; use as a **ground-truth check of the oracle on a reduced longitudinal two-car model** (stretch goal).
- **Criticality-metric review (Westhofen et al.).** Catalogue of ~50 metrics (TTC, THW, PET, TTB, TTM, TTR, BTN, STN, TTK…) with suitability analysis `[verified: arXiv:2108.02403; online catalogue criticality-metrics.readthedocs.io]`. Use it to justify the margin components of G and to pick which metric to log alongside V.
- **STRIVE.** Generates accident-prone scenarios, then runs a solution-optimization stage to verify a collision-free planner trajectory exists, so scenarios are "challenging but not impossible" `[verified]`. We want *both halves* and especially the boundary between them.

### 1.5 Margin G — design as implemented, and its known weaknesses

```
actor_margin(ego, other):
    long_gap = |dx| − (half_len_ego + half_len_other)
    lat_gap  = |dlat| − (half_wid_ego + half_wid_other)
    if lat_gap < 0 and long_gap < 0:      return COLLISION            # box overlap
    if lat_gap >= LAT_CLEAR (0.5 m):      return 1.0                  # clearly different lane
    if long_gap < 0:                      return (lat_gap − 0.5)/5    # squeezing alongside
    closing = (v_ego − v_other) if other ahead else (v_other − v_ego)
    return (long_gap − max(closing,0)·THW(1.0 s)) / 5 m
G = clip( min over time, actors of actor_margin , −1, 1 )
collision  → −clip(0.1 + impact_speed/30, 0.1, 1)     # continuous severity
off-road   → −clip(0.1 + ego_speed/30,   0.1, 1)
```

- The `min over time` includes the final step, so it doubles as the proposal's **terminal-continuation margin** (a stopped ego with a fast follower closing is penalized).
- Why a continuous failure score: with a flat −1 for any crash, V has a plateau and gradient-based/GP-based search sees nothing. Severity (impact speed) restores slope.
- **Known weaknesses.** (1) Positive side saturates at +1, so the edge band is narrow in parameter space (≈ 1.5 m of slack). (2) Severity scale 30 is arbitrary. (3) Lateral clearance 0.5 m, headway 1.0 s, scale 5 m are guesses. (4) Components are not yet in declared units with justified normalization scales (proposal §4 requires this). **Andy and Victor should own the final G.** The code isolates it in one function.

---

## 2. Search problem and the algorithm landscape

### 2.1 What we are actually searching for

Four *different* goals, which need different algorithms. Do not benchmark them against a single number.

| Goal | Question | Best-fit algorithm family |
|---|---|---|
| **Falsification** | find *any* θ with V < ε quickly | optimization (CEM, BO-LCB), importance sampling |
| **Coverage** | find *diverse* unrecoverable scenes | quality diversity (MAP-Elites, CMA-ME) |
| **Boundary / edge** | find θ with V ≈ ε; characterize the surface | active level-set estimation (straddle), bisection |
| **Most-likely failure** | among failures, which are most probable under a model of the world | adaptive stress testing (MCTS / DRL) |

The Corso et al. survey frames safety validation as exactly these tasks — falsification, most-likely failure, probability estimation — and surveys optimization, path-planning, RL and importance-sampling algorithms for them `[verified: JAIR 2021, arXiv:2005.02979]`.

### 2.2 Interface (all algorithms)

Ask/tell over the unit cube `[0,1]^d`, simulator-agnostic, batch-friendly (batch = number of CARLA servers):

```python
U = algo.ask(n)            # n points in [0,1]^d
rows = oracle_pool(U)      # each row: theta fields, V, category, planner_margin, m1, m2, u
algo.tell(U, rows)
```

Every batch is appended to a CSV (crash-safe). Algorithms see only scalar `V` (and the descriptors `m1, m2` for QD).

### 2.3 `sobol` — scrambled Sobol quasi-random baseline

- **Paper.** Corso et al. 2021 treat random/QMC sampling as the baseline any falsifier must beat `[verified]`.
- **Why QMC not iid.** Better space-filling at equal budget; scrambling gives randomness for seeds.
- **Use.** Always run it; it is also the right way to estimate base rates (the reference set had 37% unrecoverable).

### 2.4 `cem` — cross-entropy falsification

- **Paper.** Corso et al. 2021 (CE as importance sampling / optimization) `[verified]`; VerifAI ships a cross-entropy active sampler `[verified: Scenic/VerifAI parallel-falsification paper]`.
- **Algorithm.**
  ```
  init μ = 0.5·1, σ = 0.3·1
  each round:
      sample n points  u ~ N(μ, diag σ²), clipped to [0,1]; with prob 0.1 replace by uniform
      evaluate V
      history ← history ∪ new;  elite ← lowest-V ⌈0.25·|history|⌉ (min 3)
      μ ← 0.7·mean(elite) + 0.3·μ
      σ ← max(0.7·std(elite) + 0.3·σ, 0.05)
  ```
- **Strength.** Very efficient crash-finder (85–90% of samples unrecoverable on the mock).
- **Weakness.** Mode collapse to one failure basin; returns almost nothing near the boundary; elite threshold on V ignores the −1 plateau structure.
- **Knobs.** elite fraction (0.25), exploration (0.1), smoothing (0.7/0.3), σ floor (0.05).

### 2.5 `bo_lcb` — Bayesian-optimization falsification

- **Paper.** Corso et al. 2021 (BO for falsification) `[verified]`; VerifAI (Dreossi et al., CAV 2019) `[memory]`.
- **Algorithm.** Fit GP to (u, V). Next point: `argmin μ(u) − κ σ(u)`, κ = 2, over 3000 fresh Sobol candidates. Batch via **kriging believer**: after choosing u*, add (u*, μ(u*)) to the training set, refit with frozen kernel hyper-parameters, choose again.
- **GP.** `ConstantKernel × Matern(ν=2.5, ARD, length-scale ∈ [0.05, 5]) + WhiteKernel`, `normalize_y=True`, 1 optimizer restart.
- **Strength.** Sample-efficient when each simulation is expensive (CARLA).
- **Weakness.** V is bimodal with sharp transitions; a stationary GP smooths over the boundary. Like CEM it finds failures, not the surface.

### 2.6 `straddle` — active level-set estimation (**recommended for edge cases**)

- **Papers.** Bryan et al., *Active Learning for Identifying Function Threshold Boundaries*, NIPS 2005 (straddle heuristic: `1.96σ − |μ − h|`) `[verified]`. Randomized variant: Inatsu et al., TMLR 2024, replacing the confidence parameter with a draw `β ~ χ²₂` and taking `max(0, ·)` `[verified: arXiv:2408.03144]`.
- **Algorithm.** Same GP as above, threshold `h = ε = 0`. Acquisition:
  ```
  a(u) = max(0, sqrt(β)·σ(u) − |μ(u) − h|),    β ~ χ²₂ (fresh draw each pick)
  next = argmax a over 3000 Sobol candidates (fallback: argmax σ if all a = 0)
  ```
  Batch by kriging believer.
- **Why it fits.** It spends samples where the model is *uncertain and near the threshold*, i.e. on the recoverability boundary — what the proposal needs for matched pairs and edge cases.
- **Result (mock).** 28 near-boundary scenes per 100 vs 5.7 for Sobol; 20 distinct unrecoverable descriptor cells vs 12.7.
- **Caveat.** GP smoothness assumption violated by the plateau at +1 and the failure region; consider modelling a *transformed* V (e.g. `tanh` or clip to [−0.5, 0.5]) so the boundary dominates the fit.

### 2.7 `qd_fail`, `qd_boundary` — MAP-Elites quality diversity

- **Papers.** MAP-Elites: Mouret & Clune 2015 `[memory]`. Scenario generation as QD: Fontaine & Nikolaidis, *A Quality Diversity Approach to Automatically Generating HRI Scenarios in Shared Autonomy*, arXiv:2012.04283 `[verified]`, and *Algorithmic Scenario Generation as Quality Diversity Optimization*, arXiv:2409.04711 `[verified]`; for driving, **CaDRE** (arXiv:2403.13208), which uses CMA-ME with an occupancy-aware restart `[verified]`.
- **Descriptors (ours, `core.measures`).** `m1` = obstacle_gap / ego_speed (time budget of the *brake* route); `m2` = time until the left-lane car catches the ego (time budget of the *lane-change* route; 0 if alongside/ahead, 10 if never). Archive: 10 × 10 over m1 ∈ [0,8], m2 ∈ [0,10]. These are intrinsic features of the scene, not of the policy — the point of QD is that "unrecoverable" cells then tile the *types* of trap.
- **Algorithm.** Standard MAP-Elites: seed with uniform samples (n_init = 20); then repeatedly pick a random elite, perturb with Gaussian σ = 0.12, evaluate, keep if better in its cell.
  - `qd_fail`: fitness = −V (fill each cell with its worst scene).
  - `qd_boundary`: fitness = −|V − ε| (fill each cell with the scene closest to the boundary).
- **Result (mock).** Underperformed at 100 scenes. MAP-Elites needs thousands of evaluations; do not conclude it is bad.
- **Upgrade path.** CMA-ME (Fontaine et al., GECCO 2020 `[memory]`) when the budget allows; CaDRE's restart rule to escape saturated regions.

### 2.8 `bisect` — one-factor boundary bisection `[mine]`

- **Purpose.** Produce **matched pairs**: two scenes identical except for one factor, one unrecoverable and one recoverable — the "paired histories that vary one factor" of proposal §5.1, which is the cleanest training signal for a recoverability-sensitive representation.
- **Algorithm (per chain, state machine so it parallelizes).**
  ```
  pick a known unrecoverable u (V<ε), a random factor f, random end order of {0,1}
  anchor:  u' = u with u'[f] = end;  if V(u') ≥ ε → hi = u', lo = u;   else try other end;  both fail → abandon
  repeat 4×:  mid = (lo+hi)/2;  if V(mid) < ε: lo = mid  else hi = mid
  emit pair (lo, hi) with varied factor f, factor gap
  ```
- **Cost.** 1–2 anchor probes + 4 bisections ≈ 5–6 sims per pair, but needs ≥ 6 rounds of batches (chain steps are sequential), and needs a first unrecoverable seed.
- **Result (mock).** 6.7 pairs per 100 scenes; 15.7 near-boundary scenes.
- **Caveats.** (1) Only finds boundaries reachable along a coordinate axis; combine with straddle for oblique boundaries. (2) Discontinuous boundaries can produce pairs whose "gap" is not small — report `gap_unit`. (3) Both members are only as trustworthy as the oracle at that point; re-label the pair with the full profile R(h).
- **Interesting finding.** One mock pair was counter-intuitive: a *faster* adjacent car was the recoverable side, because it clears the escape lane sooner. Such non-monotone structure is exactly what the world model should learn.

### 2.9 Not implemented, with reasons

| Method | Why not now | When it becomes right |
|---|---|---|
| **Adaptive Stress Testing** (Lee et al., JAIR 2020, arXiv:1811.02188; Koren et al., arXiv:1902.01909) `[verified]` — MDP formulation; MCTS with double progressive widening or DRL finds the *most likely* failure path `[verified]` | Our θ is a static parameter vector; AST searches over a *sequence* of stochastic disturbances | When the hidden actor's behaviour is a per-step disturbance (occluded-actor family with reveal time) |
| **NSGA-II / many-objective + surrogate** (Ben Abdessalem et al.: multi-objective search with neural-network surrogates; FITEST many-objective, ASE 2018) `[verified]` | Single scalar objective today | When optimizing several criteria jointly (e.g. minimize V while maximizing scenario likelihood and diversity) |
| **Gradient/kinematic attacks** (KING, ECCV 2022, arXiv:2204.13683) `[verified]`: kinematic bicycle proxy gives gradients, reported 20% higher success than black-box | Needs a differentiable proxy of CARLA's dynamics | If CARLA runs get too expensive: use a differentiable proxy to propose, CARLA to verify |
| **Learned traffic prior** (STRIVE, CVPR 2022, arXiv:2112.05077) `[verified]` | Needs a trained prior and real data | Approach 2 bridge: realism-constrained generation |
| **LLM-written scenarios** (ChatScene, CVPR 2024: LLM → Scenic → CARLA, +15% collision rate vs baselines `[verified]`; ScenGE 2025) | Produces *semantic* scenarios, not parameter sweeps; Scenic–CARLA 0.9.16 compatibility unclear | Proposal diversity of *types* of trap; seeds for the parametric search |
| **Importance sampling for probability estimation** (Zhao et al. accelerated evaluation `[memory]`) | We want a set of scenes, not a failure probability | If the team wants to claim a rate on a distribution |

### 2.10 Recommended multi-stage pipeline

```
Stage 0  Calibrate + selftest the CARLA backend; unit-test the oracle against analytic braking (BTN)
Stage 1  EXPLORE   sobol (≈10% budget)            → base rate, rough map
Stage 2  FALSIFY   cem or bo_lcb (≈20%)            → unrecoverable seeds in several basins
Stage 3  BOUNDARY  straddle (≈40%) + bisect (≈15%) → edge cases and matched pairs
Stage 4  COVER     qd_boundary / qd_fail (≈15%+ with larger budget) → diversity by descriptor cell
Stage 5  FILTER    full R(h) re-label; dedupe (min unit distance); solvability + planner_failure split
Stage 6  EXPORT    JSONL schema for the world model; CARLA replay config; sensor-conditioning dump for Cosmos
```

Warm-start tip: later stages should be seeded with earlier results (`algo.tell(rows)` on old rows) so GP-based stages do not waste an init design.

---

## 3. Evaluation protocol for the search algorithms

| Metric | What it measures | Notes |
|---|---|---|
| `unrecoverable` count / rate | falsification power | the survey's *falsification* task |
| `sims_to_first_unrec` | time-to-first-failure | cheap and standard |
| `near_boundary` = #{|V−ε| < band} | boundary targeting | straddle's home turf |
| `edge` count | recoverable-but-barely scenes | needs EDGE_BAND justified |
| `qd_cells_unrec` | coverage of descriptor space | 10 × 10 grid |
| `diversity` | mean nearest-neighbour distance among unrecoverable, in [0,1]^d | cheap |
| `matched_pairs`, `gap_unit` | quality of boundary localization | only bisect |
| `levelset_f1` | 1-NN reconstruction of the true region from the algorithm's samples | **rewards space-filling, not boundary focus — do not rank boundary methods with it**; replace with surrogate-based F1 or a boundary-distance metric |

Report **intervals clustered by seed**, never per-frame. 3 seeds is too few for ranking close algorithms; use ≥ 10 on the mock before drawing conclusions, and treat CARLA runs as case studies unless budget allows repeats.

### 3.1 Mock-backend results (what we have)

Reference set: 600 Sobol scenes, 37% unrecoverable. 3 seeds × 100 scenes each.

| algorithm | unrec. | edge | near-boundary | sims to 1st | QD cells | diversity | matched pairs |
|---|---|---|---|---|---|---|---|
| sobol | 34 | 2.0 | 5.7 | 1.3 | 12.7 | 0.45 | 0 |
| cem | 85 | 1.0 | 3.0 | 2.7 | 7.7 | 0.16 | 0 |
| bo_lcb | 90 | 0.0 | 0.0 | 1.3 | 10.3 | 0.30 | 0 |
| **straddle** | 62 | **4.0** | **28.0** | 1.3 | **20.3** | 0.44 | 0 |
| **bisect** | 65 | **5.7** | 15.7 | 1.3 | 11.7 | 0.14 | **6.7** |
| qd_fail | 29 | 0.3 | 3.0 | 1.7 | 10.7 | 0.27 | 0 |
| qd_boundary | 24 | 0.3 | 3.7 | 1.7 | 8.0 | 0.28 | 0 |

Honest reading: straddle is the strongest all-round for this project; BO/CEM are failure-finders; bisect is the pair factory; QD needs a bigger budget; `levelset_f1` favoured Sobol (0.79) over straddle (0.66) but that metric is biased (above). The mock is a point-mass model — **none of this is evidence about CARLA.**

---

## 4. Tooling landscape (what to use, what to avoid)

### 4.1 CARLA

- **Version.** Use **0.9.16** (UE4.26). Its release added the **Cosmos Transfer1 integration**, NuRec integration, ROS2, and Left-Handed Traffic `[verified]`. **0.10.0** (UE5.5, Dec 2024) fully supports Scenic 3 but is "still a work in progress", some functionality not yet migrated, and recommends ≥16 GB VRAM `[verified]`. Autoware's Open AD Kit has a sample running closed-loop CARLA 0.9.16 end to end with modular containers and `autoware_carla_interface` `[verified]`.
- **Determinism.** Synchronous mode + fixed time step, no Traffic Manager/autopilot (we drive every actor ourselves). Synchronous mode makes the server wait for the client tick `[verified]`. The CARLA docs also warn the Traffic Manager must be in sync mode if used — avoiding it removes a source of non-determinism.
- **No state snapshot.** CARLA cannot restore an arbitrary physics state. Our rollouts therefore **re-spawn from θ** every time (cost: ~6 policies × 3 worlds + planner ≈ up to 21 episodes per scene, reduced by exact pruning). Parallelize with **one server per port**.
- **Recorder as approximate restore `[untested idea]`.** The recorder logs vehicle pose, velocities and physics control and can start playback at any time, re-enacting "a few seconds before" an event `[verified]`. Playback teleports actors to recorded states; one could stop playback at t and hand control to a policy to branch for the point-of-no-return scan. Internal controller/suspension state is not restored — measure how much that perturbs outcomes before trusting it.
- **Headless.** `./CarlaUE4.sh -RenderOffScreen -nosound -carla-rpc-port=PORT`; `settings.no_rendering_mode = True` since we use no cameras. Re-enable rendering only for Cosmos conditioning dumps.
- **Friction/brake calibration.** CARLA's `tire_friction` does not equal a decel in m/s². Measure it (`--calibrate`) and map `brake_limit → friction` by interpolation. Without that, the brake-limit axis is meaningless.
- **Road frame.** x forward, **lateral positive to the right** (CARLA +y); the left lane is negative lateral. Pick a straight multi-lane stretch (Town04 highway; try Town06).

### 4.2 Scenic / VerifAI / ScenarioRunner

- Scenic's CARLA interface documents testing on **0.9.9–0.9.14** `[verified]` — not 0.9.16. Treat Scenic-on-0.9.16 as a risk; our own backend avoids it.
- VerifAI provides **active samplers** (cross-entropy; also Halton, multi-armed-bandit, ε-greedy, multi-objective bandit) and a **parallel falsifier** that runs several simulator instances with decoupled sampling and updating `[verified]`. Our ask/tell + server pool mirrors that pattern. Suggested use: VerifAI's cross-entropy and Halton samplers as *external baselines* on the same oracle. Whether VerifAI can be driven without Scenic through a plain feature-space spec should be checked `[memory]`.
- Scenic includes a 2D Newtonian simulator for debugging without an external simulator `[verified]` — a possible optional 2D backend.

### 4.3 Cosmos

- Cosmos Transfer1 in CARLA generates style variations of CARLA output from text prompts — architecture, vehicles, weather, lighting `[verified]`. NVIDIA's pipelines condition on simulation outputs (e.g. an HD-map-conditioned Transfer model with CARLA physics) `[verified]`.
- **Implication.** It gives *photorealistic appearance variants of a fixed, already-labelled dynamic scene*. This is perfect for the proposal's appearance-only variations (same θ, same R, different pixels) and for testing whether the representation ignores nuisance appearance. It does **not** vary gaps, speeds, or delays. Say this in the proposal.
- Cosmos-Drive-Dreams (arXiv:2506.09042) is NVIDIA's corner-case synthetic-data pipeline with world foundation models `[verified]`; relevant related work for step 4.
- Label inheritance: all appearance variants of a mined scene share its θ and its R(θ). Keep the provenance field.

### 4.4 Autoware

Plan to replace `TTCBrakePlanner` with Autoware as the planner under test via Open AD Kit's CARLA sample (`autoware_carla_interface`) `[verified]`. The `planner_failure` class (recovery exists, planner fails) is the bridge: those are the cases where Autoware should be scored. Budget a week; do not block the search work on it.

### 4.5 Real-data mining (Approach 2 — for Kent; context only)

- **RefAV / Argoverse 2 Scenario Mining** (Davidson, Ramanan, Peri; CVPR 2026): 10,000 natural-language queries over 1,000 AV2 sensor logs; LLM-written programs run over ground-truth tracks; the authors found naive VLMs perform poorly `[verified]`. AV2 annotates tracks at **10 Hz**, whereas **nuScenes annotates at 2 Hz** `[verified]` — 2 Hz is coarse for TTC/margin-style criticality, which matters for deciding which dataset to mine.
- nuPlan defines 80 scenario types relevant to planning `[verified via RefAV text]`.
- **Key conceptual point for the team.** A log shows one factual trajectory; "unrecoverable" is counterfactual. So real-data mining can only *propose candidates* (low TTR, high BTN, aggressive cut-ins, occlusions); the oracle in simulation must *verify*. Our θ schema should therefore be extractable from logs (gap, relative speeds, offsets, timing) — agree this with Kent early.

---

## 5. Design decisions and pitfalls (read before coding)

1. **Never feed privileged state to a policy.** Policies see only an observation dict (relative positions/velocities in the road frame). Margin uses privileged state for *labels only*.
2. **Do not use perceptually identical hidden states as contradictory labels** (proposal §5.1). Hidden worlds share an observation; the correct input to the world model is the shared history with a belief over worlds. For the occluded family, store the **reveal time**.
3. **Exact pruning is valid.** While evaluating policy j, stop its world loop as soon as its running min cannot beat the current best V. Disable pruning (`full=True`) for the final profile R(h).
4. **Keep KEEP semantics identical across backends.** `KEEP` latches the speed at the moment the previous command was not KEEP. (Mock and CARLA both implement this.) Otherwise the ego re-accelerates after braking.
5. **Reaction delay is the strongest single axis.** On the mock, one second of dead time at highway speed dominates geometry. Include it.
6. **Budget realism.** CARLA oracle cost ≈ minutes per scene before parallelism; plan for ~100–200 scenes per algorithm in the proposal window, not thousands.
7. **Determinism test first.** Same θ twice must give traces within tolerance. If not, every comparison is noise.
8. **Calibration before claims.** Never report a CARLA number until `--selftest` passes and brake decel matches `brake_limit` within tolerance.
9. **Keep mock and CARLA results in separate tables and files.**
10. **Name things honestly.** "Unrecoverable relative to Π, Ω, horizon, ε."

---

## 6. Open decisions for the team

| Decision | Owner | Why it matters |
|---|---|---|
| Final definition and units of G (collision, drivable-area, dynamic limits, latency-consistent margins, terminal-continuation margin) | Andy, Victor | everything downstream is labelled by it |
| Policy library Π: contents and growth rule; hold-out for transfer tests | Andy, Victor | determines the false-"unrecoverable" rate |
| Hidden-world set Ω and whether "never brakes" is plausible | team | determines false-"recoverable" rate |
| Scenario families for the proposal: ambiguous cut-in with secondary actor; occluded crossing with reveal time | team | proposal §6 |
| Schema shared between Approach 1 and Approach 2 (θ fields + provenance + R vector + evidence status + reveal time) | Fred + Kent | lets real-data scenes be reconstructed and labelled by the same oracle |
| Which planner is "under test" (TTC baseline → Autoware) | Fred | defines `planner_failure` |

---

## 7. Risks

- **CARLA backend untested** (largest near-term risk). Mitigation: selftest, calibration, determinism check before any run.
- **Sim cost.** Mitigation: server pool, exact pruning, shrink Π/Ω first, then budget.
- **GP model misspecification** on a discontinuous V. Mitigation: transform V; compare with CEM/Sobol; fall back to bisect.
- **Label noise from physics jitter** near the boundary. Mitigation: determinism test; for scenes within 1σ of ε, re-run with 3 settle seeds.
- **Definition drift** between teammates. Mitigation: one `core.py`, versioned, with tests.
- **Overclaiming.** The proposal itself forbids it (no infinite-horizon claim, no physical-impossibility claim).

---

## 8. Reference list with verification status

**Verified by search this session**
- Corso, Moss, Koren, Lee, Kochenderfer. *A Survey of Algorithms for Black-Box Safety Validation of Cyber-Physical Systems.* JAIR 2021. arXiv:2005.02979
- Lee et al. *Adaptive Stress Testing: Finding Likely Failure Events with Reinforcement Learning.* JAIR 2020. arXiv:1811.02188
- Koren, Alsaif, Lee, Kochenderfer. *Adaptive Stress Testing for Autonomous Vehicles.* arXiv:1902.01909
- Bryan, Schneider, Nichol, Moore, Schneider. *Active Learning for Identifying Function Threshold Boundaries.* NIPS 2005
- Inatsu et al. *Active Learning for Level Set Estimation Using Randomized Straddle Algorithms.* TMLR 2024. arXiv:2408.03144
- Fontaine & Nikolaidis. *A Quality Diversity Approach to Automatically Generating Human-Robot Interaction Scenarios in Shared Autonomy.* arXiv:2012.04283
- *Algorithmic Scenario Generation as Quality Diversity Optimization.* arXiv:2409.04711
- *CaDRE: Controllable and Diverse Generation of Safety-Critical Driving Scenarios using Real-World Trajectories.* arXiv:2403.13208
- Hanselmann et al. *KING.* ECCV 2022. arXiv:2204.13683
- Rempe et al. *STRIVE: Generating Useful Accident-Prone Driving Scenarios via a Learned Traffic Prior.* CVPR 2022. arXiv:2112.05077
- Fremont et al. *Scenic: A Language for Scenario Specification and Data Generation.* arXiv:2010.06580
- *Parallel and Multi-Objective Falsification with Scenic and VerifAI.* arXiv:2107.04164
- Zhang, Xu, Li. *ChatScene.* CVPR 2024. arXiv:2405.14062
- Westhofen et al. *Criticality Metrics for Automated Driving: A Review and Suitability Analysis.* arXiv:2108.02403; catalogue: criticality-metrics.readthedocs.io
- Davidson, Ramanan, Peri. *RefAV: Towards Planning-Centric Scenario Mining.* CVPR 2026. arXiv:2505.20981
- Ben Abdessalem et al. *Testing Autonomous Cars for Feature Interaction Failures using Many-Objective Search* (ASE 2018); *Testing Advanced Driver Assistance Systems using Multi-objective Search and Neural Networks* (title verified; ASE 2016 venue/year from memory)
- Cosmos-Drive-Dreams. arXiv:2506.09042
- CARLA 0.9.16 release notes (carla.org, 16 Sep 2025); CARLA 0.10.0 release notes (carla.org, 19 Dec 2024); CARLA recorder docs; Autoware Open AD Kit CARLA sample
- Zhao et al. *Reachability-aware RL for collision avoidance* (HJ CARS), arXiv:2502.10610

**Cited from memory — re-check before the proposal**
- Mouret & Clune. *Illuminating search spaces by mapping elites.* arXiv:1504.04909
- Fontaine et al. *Covariance Matrix Adaptation for the Rapid Illumination of Behavior Space (CMA-ME).* GECCO 2020
- Dreossi et al. *VerifAI: A Toolkit for the Formal Design and Analysis of AI-Based Systems.* CAV 2019
- Fraichard & Asama, *Inevitable collision states — a step towards safer robots?* (the original ICS reference; the definition text I used came from a search snippet)
- Zhao et al. *Accelerated Evaluation of Automated Vehicles Safety in Lane-Change Scenarios based on Importance Sampling Techniques.* 2016
- Jansson 2005 (BTN origin), Hillenbrand 2006 (TTR origin), Eidehall 2011 (multi-actor BTN) — as attributed on the criticality-metrics catalogue page

**Mine (no paper)**: `bisect` one-factor boundary bisection; QD descriptors (m1, m2); the margin G as implemented; point-of-no-return scan as simulation-based TTR over Π.
