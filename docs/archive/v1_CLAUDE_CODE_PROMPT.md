# PROMPT FOR CLAUDE CODE — Approach 1: simulation mining of unrecoverable driving scenes (CARLA, headless)

You are working in the repo/folder that contains `mining/` (core.py, algos.py, carla_backend.py, run.py, README.md) and `approach1_research_findings.md`. **Read `approach1_research_findings.md` first, then `mining/core.py`, `mining/algos.py`, `mining/carla_backend.py`, `mining/run.py`.** Do not start coding until you have read them.

## 1. Project context (read carefully)

University of Toronto CSC2626 (Imitation Learning for Robotics) final project, 4 people. Title: *Recoverability-Preserving World Models for Safety-Critical Imitation Learning.* Central hypothesis: a latent dynamics model trained to preserve observation-conditioned **recovery options** produces fewer false feasibility judgments and better imitation targets than predictive learning or a scalar safety head alone, including when the continuation-policy library changes.

Team split: **Fred = Approach 1** (headless simulation mining of edge and unrecoverable cases — *this task*); Kent = Approach 2 (public data: nuScenes / Argoverse 2 / Waymo); Andy + Victor = formalizing recoverability and building the world model.

Pipeline (team plan): (1) find unrecoverable scenes — Approach 1 simulation mining, Approach 2 public data; (2) formalize unrecoverable-case semantics; (3) reconstruct them in CARLA; (4) use Cosmos to generate **appearance** variations; (5) formalize cases for world-model training; (6) train the world model; (7) integrate with Autoware; (8) test. Tech stack: **CARLA 0.9.16**, Cosmos Transfer1, Autoware (Open AD Kit CARLA sample), nuScenes/Waymo/Argoverse.

Timeline (today is 15 Oct 2026): week of 12 Oct — discover data + unrecoverable scenes; **by 19 Oct — proposal done** (needs real Approach 1 results and figures); week of 26 Oct — reconstruct unrecoverable cases in CARLA + Cosmos; ~week of 9 Nov — integrate with world model and Autoware.

Research highlights to serve: (a) how to meaningfully define unrecoverable scenarios; (b) how to formalize unrecoverability and build a world model to solve it.

## 2. Formal definition you must preserve

Scenario parameters θ ∈ [0,1]^d (decoded to physical units). Hidden worlds Ω (finite). Library Π of **causal** recovery policies (observation → command; never sees the world id). For each rollout, margin G is computed from privileged state, **for labels only**.

```
r_j(θ) = min_{ω ∈ Ω} G(π_j, ω; θ)
V(θ)   = max_j r_j(θ)
recovery profile R(θ) = [r_1 … r_N]
unrecoverable  ⇔  V(θ) < ε            (EPS = 0)
```

Categories: `unrecoverable` (V<ε), `planner_failure` (V≥ε but the planner under test fails), `edge` (ε ≤ V < ε+0.3), `safe`.

**Non-negotiables**
- "Unrecoverable" always means *relative to Π, Ω, horizon and ε* — never "physically impossible". Say so in docstrings, README text and plot titles.
- Error directions: too-small Π → false "unrecoverable"; too-small Ω → false "recoverable". Do not hide this.
- Three evidence statuses are kept: verified witness / verified counterexample for a policy / unresolved. Unresolved entries are masked, never invented.
- Policies must not read privileged state or the world id.
- Related concepts to cite: Inevitable Collision States, Time-To-React (max over maneuvers), Brake Threat Number ≥ 1, Hamilton–Jacobi collision-avoidance reachable sets.

## 3. What already exists (do not rewrite from scratch)

- `core.py`: space `SPACE` (obstacle_gap, adjacent_offset, adjacent_speed, ego_speed, reaction_delay, brake_limit), `decode`, `measures` (QD descriptors m1, m2), `WORLDS` (constant, idm_reactive, accelerating), 6 causal policies + `TTCBrakePlanner` (planner under test), `actor_margin`/`margin`, `oracle(backend, theta, full=False)` with exact pruning, and a fast numpy `MockBackend`.
- `algos.py`: ask/tell optimizers: `sobol`, `cem`, `bo_lcb`, `straddle` (randomized straddle, β~χ²₂, kriging-believer batching), `qd_fail`, `qd_boundary` (MAP-Elites, 10×10 over m1,m2), `bisect` (one-factor boundary bisection producing matched pairs). `REGISTRY` maps names → constructors.
- `carla_backend.py`: CARLA 0.9.16 backend. Synchronous mode, fixed 0.05 s, no_rendering_mode, **no Traffic Manager**, road frame (x forward, **lateral positive to the right**, left lane negative), scripted hidden-world actors, Stanley lane tracker, calibration of friction→decel, `--selftest`. **It has never been run against a live server. API names were checked against the 0.9.16 client wheel only.**
- `run.py`: batched search loop with CSV checkpoint per batch, server pool (`--ports`), metrics, plots, `--bench`, `--report`, `--profiles`.
- Mock benchmark (3 seeds × 100 scenes; NOT CARLA): straddle ≈ 28 near-boundary scenes/100 vs 5.7 for Sobol and the most distinct unrecoverable QD cells; cem/bo_lcb 85–90% unrecoverable but ~0 near boundary; bisect 6.7 matched pairs/100; MAP-Elites under-budgeted at 100.

## 4. Environment

- Python 3.12. Dependencies: numpy, scipy, scikit-learn, pandas, matplotlib, `carla==0.9.16` (client), pytest.
- Server: `./CarlaUE4.sh -RenderOffScreen -nosound -carla-rpc-port=PORT` (one process per port). If no GPU/server is available in your environment, **say so, run everything that can run on the mock, and leave CARLA tasks clearly marked as not executed.** Never fabricate CARLA outputs.
- Use **CARLA 0.9.16**, not 0.10.0 (Cosmos Transfer1 integration and the Autoware Open AD Kit sample target 0.9.16).
- Scenic's documented CARLA support is 0.9.9–0.9.14. Do not make the pipeline depend on Scenic.

## 5. Tasks, in priority order (each has acceptance criteria)

### T0 — Tests and hygiene (first, ~1 hour)
Create `tests/` with pytest:
1. `actor_margin` unit tests: collision → None; car in another lane → 1.0; stopped behind obstacle with gap g → (g)/5 when closing = 0; squeeze alongside → negative.
2. **Analytic cross-check (BTN).** For the mock with only `emergency_brake` and no other actors: scene is recoverable iff stopping distance `v²/(2·a)` + reaction-delay travel `v·delay` < gap − vehicle length (± one timestep tolerance). Sweep ≥ 200 random θ; ≥ 99% agreement; list disagreements.
3. Determinism: same θ → identical trace on the mock (exact) and later on CARLA (tolerance).
4. Algorithm smoke tests: each algorithm in `REGISTRY` runs 24 scenes on the mock without error; `bisect` pairs satisfy V(lo)<ε≤V(hi) and differ in exactly one coordinate.
5. Ask/tell contract: `ask(n)` returns an (n, d) array in [0,1].
**Accept:** `pytest -q` green; BTN agreement reported in the test output.

### T1 — Fix known weaknesses in the label definition (mock first)
1. Put every constant of G (LAT_CLEAR, HEADWAY_S, MARGIN_SCALE, failure severity scales) into one frozen dataclass `MarginConfig` with units in comments; thread it through `margin`, `oracle`, backends. This lets Andy/Victor change G in one place.
2. Add a sensitivity sweep script: vary each G constant ±50% and report how many scenes change category. Report as a table. **Accept:** table in `results/g_sensitivity.md`.
3. Transform V for GP-based algorithms (e.g. `clip(V, −0.5, 0.5)` or `tanh(2V)`) behind a flag `--vtransform`; benchmark straddle with and without on the mock (≥ 10 seeds). **Accept:** table + one-paragraph conclusion.

### T2 — CARLA bring-up (blocks everything real)
1. `python carla_backend.py --port 2000 --calibrate`; save `friction_calibration.json`; plot decel vs friction. **Accept:** monotone mapping covering 5–9 m/s²; if not, adjust the friction range and document.
2. `--selftest`; verify: `keep` holds speed (|Δv| < 0.5 m/s over 8 s), `emergency_brake` decel within 10% of `brake_limit`, `left_now` ends at lateral ≈ −lane_w ± 0.3 m, no off-road for a clean lane change. Fix the controller if not (Stanley gains, throttle map, lane tracking), keeping the road-frame sign conventions.
3. **Determinism check:** same θ, same policy, same world, run twice (and across two servers) → traces agree to < 0.05 m / 0.05 m/s. If they do not, find the nondeterminism (settle ticks, physics substepping, spawn z, Traffic Manager) before anything else.
4. Timing profile: ms per tick, seconds per rollout, seconds per oracle call (pruned and full); estimate scenes/hour per server. Write to `results/carla_timing.md`.
5. Parallel servers: run `--ports 2000,2002,2004,2006`; verify linear-ish speed-up.
**Accept:** selftest output pasted into `results/carla_selftest.txt`, determinism and timing tables written. Do not proceed to T4 if determinism fails.

### T3 — Validate the oracle against an independent method
1. BTN/analytic check from T0 on CARLA's brake-only policy (stopping distance with calibrated decel).
2. (Stretch) Hamilton–Jacobi reachability on a reduced longitudinal two-car model (ego brakes/accelerates, lead vehicle worst-case braking) using a JAX/Python HJ toolbox (e.g. `hj_reachability` or OptimizedDP — verify availability); compare the avoidable set with the oracle restricted to the longitudinal policies. Report disagreement.
**Accept:** agreement numbers + a short note on where and why the oracle differs.

### T4 — Run the algorithms on CARLA (the proposal deliverable)
Budget plan (adjust after T2 timing): `sobol` 40 → `cem` 30 → `straddle` 80 → `bisect` 50 → `qd_boundary` 100. Warm-start later stages with earlier rows (`algo.tell`). Use ≥ 2 servers. Checkpoint every batch.
Outputs per algorithm: `samples_<algo>.csv`, `summary.csv`, `descriptor_space.png`, `matched_pairs.json`, `mined_scenes.json` (with full R(h) via `--profiles`).
**Accept:** (a) table of metrics for CARLA runs (separate from mock), (b) recoverability scatter in (m1,m2) coloured by category, (c) 5–10 matched pairs with a short human-readable description each ("same scene except reaction delay 1.0 s vs 1.2 s"), (d) all numbers reproducible from saved CSVs with `run.py --report`.

### T5 — Point-of-no-return (Time-To-React) scan
For a nominal keep-speed rollout, find the time t* at which V(h_t) crosses ε. Implement two ways and compare: (i) re-spawn and replay the nominal prefix to time t then branch into the library; (ii) CARLA recorder: record the nominal run, `replay_file(..., start=t)`, stop playback, take control, branch. Measure how much (ii) perturbs outcomes vs (i). **Accept:** plot V(h_t) vs t for ≥ 5 scenes; table of t* from both methods; recommendation of which to use.

### T6 — Occluded-actor scenario family (proposal's second family)
Add a hidden vehicle revealed at a configurable time `t_reveal` (and optionally an occluder object). Policies see the actor only after reveal. Worlds Ω now vary the hidden actor's behaviour. Keep the label contract unchanged (shared history, same observation before reveal). Add `t_reveal` to SPACE. **Accept:** mock + CARLA both run; show that V decreases with later `t_reveal` for fixed geometry.

### T7 — Dataset export for the world model (schema agreed with Kent, Andy, Victor)
JSONL, one record per scene:
```
{ "id", "source": "sim_mining" | "real_log", "backend": "carla-0.9.16" | "mock",
  "theta": {...physical units...}, "u": [...], "seed", "world_set": [...], "policy_library": [...],
  "R": {policy: margin}, "V", "category", "evidence": {policy: "verified_witness"|"counterexample"|"unresolved"},
  "t_reveal": null|float, "margin_config": {...}, "descriptors": {"m1","m2"},
  "pair_id": null|str, "pair_role": null|"unrecoverable"|"recoverable", "varied_factor": null|str,
  "trace_path": "traces/<id>.npz" }
```
Also export per-scene state/observation traces (npz), a `replay_scene.py <id>` script that re-creates the scene in CARLA (and optionally records sensor streams: segmentation, depth, HD-map render) for Cosmos conditioning. **Cosmos Transfer only varies appearance:** all appearance variants inherit the scene's θ and R; store `parent_id` and `variant_kind: "appearance"`.
**Accept:** `validate_dataset.py` checks schema, no NaNs, `V == max_j R_j`, category consistent with V and ε.

### T8 — Planner under test: Autoware
Replace `TTCBrakePlanner` with Autoware (Open AD Kit CARLA sample, `autoware_carla_interface`, CARLA 0.9.16) behind a `PlannerBackend` interface so the oracle can still run with the TTC stand-in. Report `planner_failure` rate for Autoware vs TTC stand-in on the mined scenes. **Accept:** same scenes scored by both planners; if Autoware cannot be brought up in the time available, document the blocker and leave the interface in place.

### T9 — Proposal assets (needed by 19 Oct)
Produce, from saved results only: one figure of the recoverability map in (m1,m2); one figure of the point-of-no-return curve (T5 if ready, else mock); one table comparing algorithms (mark mock vs CARLA); one paragraph defining unrecoverability with the error-direction statement; list of matched pairs. Put in `results/proposal_assets/`.

## 6. Algorithm specifications (reference; implemented in `algos.py`)

- **sobol** — scrambled Sobol baseline. Corso et al., JAIR 2021.
- **cem** — Gaussian over [0,1]^d; elites = lowest-V 25% of history (min 3); `μ ← 0.7·mean(elite)+0.3·μ`; `σ ← max(0.7·std(elite)+0.3·σ, 0.05)`; 10% uniform exploration. Falsifier.
- **bo_lcb** — GP (Constant × Matern ν=2.5 ARD + White), acquisition `argmin μ − 2σ` over 3000 Sobol candidates; batch by kriging believer (frozen hyper-parameters). Falsifier.
- **straddle** — same GP, threshold h=ε; acquisition `max(0, √β·σ − |μ−h|)`, β~χ²₂ per pick (Inatsu et al. 2024; Bryan et al. 2005). Boundary finder. Fall back to `argmax σ` if all zero.
- **qd_fail / qd_boundary** — MAP-Elites (Mouret & Clune 2015) with descriptors m1 = gap/ego_speed, m2 = time until the left-lane car catches the ego; archive 10×10; mutation σ=0.12; fitness −V or −|V−ε|. Upgrade to CMA-ME for larger budgets (CaDRE, arXiv:2403.13208).
- **bisect** — pick unrecoverable u, random factor, anchor on a recoverable end of that factor, bisect 4×, emit matched pair. Chains are state machines so they batch.
- **Not implemented on purpose:** adaptive stress testing (static θ), NSGA-II/surrogates, KING-style gradients, LLM/Scenic generation. Add only if a task needs them.

## 7. Engineering rules

- Keep the ask/tell interface and `core.oracle` signature stable; backends implement only `rollout(theta, world, policy) -> trace`.
- No Traffic Manager/autopilot in CARLA; every actor is driven by our controllers. Fixed seeds everywhere; log them.
- Every long run checkpoints per batch and can resume by skipping finished algorithms.
- Mock results and CARLA results go in different files and different table columns.
- Commands that may exceed your tool time limit must run in chunks writing intermediate CSVs.
- Prefer small, reviewable commits: tests → G config → CARLA bring-up → runs → export.
- When something fails or looks wrong (e.g. selftest off, nondeterminism, collisions at spawn), **stop and diagnose before running search**; report what you saw.
- Do not claim a result you did not produce. If an acceptance criterion is not met, say which and why.

## 8. Final report format (what to hand back)

1. What was run (commands, versions, seeds, number of servers).
2. Acceptance checklist per task: ✅ / ⚠️ with evidence (paths to files).
3. Tables: algorithm comparison (CARLA), timing, G sensitivity, BTN agreement.
4. Figures: recoverability map, point-of-no-return curves, descriptor-space scatter.
5. Matched pairs (human-readable).
6. Known issues, surprises, and what you would change in the definition of G, Π or Ω.
7. Open questions for Andy/Victor (definition of G, library growth rule, Ω) and for Kent (θ schema from logs).
