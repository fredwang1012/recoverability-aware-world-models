# PROMPT FOR CLAUDE CODE (v2) — Recoverability benchmark: physical CARLA scenarios + evidence-aware labels + boundary mining

You are working in a folder containing `mining/` (code), `pdf_verification_and_additions.md`, `approach1_research_findings.md`. **Read, in this order:** (1) this prompt, (2) `pdf_verification_and_additions.md`, (3) `mining/contract.py`, `mining/core.py`, `mining/reference/*.py`, `mining/tests/*.py`, `mining/cases/catalogue.yaml`, (4) `mining/algos.py`, `mining/carla_backend.py`, `mining/run.py`, (5) `approach1_research_findings.md` for algorithm background. Do not write code before you have run `pytest -q` and seen it pass.

## 1. Project and your part

CSC2626 final project (4 people): *Recoverability-Preserving World Models for Safety-Critical Imitation Learning.* Pipeline:
1. Mathematical definition of unrecoverability — **done** (the contract below; `mining/contract.py`).
2. Concrete physical scenarios from that definition — **the 20-case catalogue RC01–RC20** (`mining/cases/catalogue.yaml`, from the PDF "Recoverability: definitions, formulations and a constructed CARLA case catalogue").
3. **Generate simulations of those scenarios in CARLA, with evidence-backed labels — THIS IS YOUR JOB (Fred's).**
4. World model that recognizes unrecoverability and improves recoverability without false judgment, trained on CARLA data (Andy, Victor).
5. Evaluation (false admission / false rejection, etc.). 6. Autoware baseline.
Fred also keeps the **boundary-mining algorithms** going: search within each case family to locate boundary bands and matched pairs, and to mine information-limited scenes. Kent does Approach 2 (public data, nuScenes/Argoverse/Waymo). Timeline (today 15 Oct 2026): proposal ~19 Oct; scenario reconstruction + Cosmos (appearance variants only) late Oct–early Nov; world-model/Autoware integration ~mid Nov.

## 2. The contract you must implement faithfully

A **history** h contains only what the policy was allowed to know (observation capture/delivery times, past commands and *applied* controls). A **world** ω is a mechanism (actor response rules + hidden state consistent with h), not a recorded path. A **policy** π is an executable, observation-conditioned, deterministic-given-history continuation (emergency phase + stabilization + terminal continuation); it never sees world id, future trajectories, scenario names or evaluator outcomes.

```
G(π,ω;h) = min{ min_{s∈[t,t+H)} min_i m_i(x_s)/q_i ,  m_term/q_term }      m_i signed, + = accepted; q_i > 0 declared units
r_π(h)  = min_{ω∈Ω(h)} G(π,ω;h)       R(h) = [r_π1 … r_πN]
Rec(h;Π,Ω)   ⇔ ∃π∈Π_causal ∀ω∈Ω: G ≥ ε            Unrec ⇔ ∀π ∃ω: G < ε
V = max_π r_π      V_hind = min_ω max_π G(π,ω)     V ≤ V_hind always; info_gap = V_hind − V
```
Hard constraints = full-body (swept OBB) clearance minus a declared reserve c, permitted road area, physical actuation limits. Route progress, comfort, task completion are stored separately. Terminal continuation is **not** "speed = 0": e.g. protected hold with a specified future actor response, verified over the whole post-maneuver interval. If the terminal test is Boolean, G = path margin on a pass and −∞ on a fail, and the Boolean is stored separately.

**Evidence classes** (`contract.classify`): `R_REF` (a witness policy passes every declared world), `U_LIBRARY` (every policy in the named finite library has a valid refutation; says nothing about other policies), `U_INFORMATION` (U_LIBRARY plus every world individually has a witness: hindsight-feasible, causally infeasible), `UNRESOLVED` (missing entries / incomplete search; never a negative label), `U_MODEL` (needs an external sound proof under reference dynamics with a genuine b_max; **cannot be earned from CARLA runs**), `INVALID_RUN` (spawn/timing/controller/logging contract failed; excluded from denominators).

**Non-negotiables.** Intentions (`intended_outcome`) never become labels. A failed maneuver refutes one policy, not the scene. Failure to find a solution is not impossibility. Pruned or unrun policy×world entries are NaN, never zero or failure. "Unrecoverable" always means *relative to Π, Ω, horizon, ε, vehicle/physics profile*. A positive claim in one world supports only that execution. Learned predictions carry no evidential authority.

## 3. State of the repo (what exists, what is verified)

- `contract.py` — evidence classifier; tested (random-matrix duality, hindsight inequality, masking).
- `reference/corridor.py` — the PDF's exact 1-D model: D_stop = vτ + v²/(2b); contact vs reserve vs witness labels; boundaries `T_p* = (gap−D_stop−c)/v` (RC12) and `τ* = (gap − v²/(2b) − c)/v` (RC17); `model_certificate` (needs genuine τ_min, b_max). **These are the analytic anchors CARLA must reproduce after calibration.**
- `reference/info_tree.py` — finite information tree for RC10/RC11; label flips R_REF → U_INFORMATION at the analytic reveal threshold; physical impossibility gives U_LIBRARY; a common stop gives R_REF for any reveal time.
- `reference/mine_corridor.py` — runs all mining algorithms against the analytic corridor boundary. Result (5 seeds, 80 scenes): straddle localizes the boundary ≈12× better than Sobol (0.19 m vs 2.45 m mean |G| of 20 closest) with GP-surrogate F1 0.987; bisect yields ≈9.6 matched pairs with 0.44 m midpoint error; CEM and BO-LCB falsify but do not localize.
- `core.py` — scenario space θ (obstacle_gap, adjacent_offset, adjacent_speed, ego_speed, reaction_delay, brake_limit), 6 causal policies + `TTCBrakePlanner` (stand-in planner under test), heuristic margin G (**not yet the PDF's**), `oracle(full=True)` returns the R matrix + evidence + V_hind; numpy `MockBackend`.
- `algos.py` — ask/tell optimizers, dimension-generic (`REGISTRY[name](rng, d=...)`): sobol, cem, bo_lcb, straddle, qd_fail, qd_boundary, bisect.
- `carla_backend.py` — CARLA backend (road frame x forward, **lat positive to the right**, synchronous, fixed 0.05 s, no Traffic Manager, calibrated friction→decel, selftest). **Never run against a live server.** CARLA API names were checked against the 0.9.16 client wheel.
- `cases/catalogue.yaml` — the 20 cases as data (site, mechanism, sweeps, witness/refutation policies, profile contrast, analytic references, build order). Intentions only.
- `run.py` — batched ask/tell driver, CSV checkpoints, server pool, metrics.
- Finding to respect: on the current two-lane mock scene, 0/200 random scenes are U_INFORMATION → that scene cannot test the information hypothesis; RC10/RC11/RC14 are required for it.

**Verified CARLA facts you may rely on:** `fixed_delta_seconds ≤ max_substep_delta_time × max_substeps` (use 0.05 / 0.01 / **10**; valid max_substeps range 1–16); synchronous mode + fixed step for reproducibility (still run a repeatability test); axes x forward / y right / z up, meters, degrees, left-handed; `CollisionEvent` has `other_actor`, `normal_impulse`, `frame`, `timestamp`; `BoundingBox.get_world_vertices/get_local_vertices/extent`; `WheelPhysicsControl.tire_friction/max_brake_torque/max_steer_angle`; recorder methods exist. **Not verified (treat as unknown):** ScenarioRunner compatibility with 0.9.16 (repo lists up to 0.9.15) → keep our own harness; "GPU sensor outputs arrive later than capture frames" (irrelevant until cameras); TM reseeding after reload (we do not use the TM).

## 4. Environment and rules

Python 3.12; numpy, scipy, scikit-learn, pandas, matplotlib, pyyaml, pytest, `carla` client matching the server (pin and record the server build; 0.9.16 preferred because Cosmos Transfer1 and the Autoware Open AD Kit CARLA sample target it; do not upgrade the simulator just for this). Server: `./CarlaUE4.sh -RenderOffScreen -nosound -carla-rpc-port=PORT`, one process per port. **If no CARLA server/GPU is available, run all mock/reference work, mark CARLA tasks "NOT EXECUTED", and never fabricate CARLA outputs.** Scenic's documented CARLA support is 0.9.9–0.9.14: do not depend on Scenic. No Traffic Manager/autopilot for decisive actors; use project-owned finite-state actor controllers keyed to simulation time or observed events. Keep mock and CARLA results in separate files/tables. Long runs: chunk, checkpoint, resume.

## 5. Tasks, in order, each with acceptance criteria

### Phase A — contract and label machinery (no CARLA needed; do first)
- **A0** `pytest -q` green (currently 9 tests). Add tests as you go.
- **A1 Margin per the PDF.** Replace the heuristic `margin` with a `MarginConfig` dataclass (reserve_m, q_clear, q_road, q_dyn, q_latency, terminal spec, ε, horizon, post-maneuver seconds; units in comments) and compute: signed clearance between **oriented-box polygons** minus reserve (use `BoundingBox.get_world_vertices` in CARLA; shapely or manual SAT in the mock), per-tick **swept** check (refine between ticks), permitted-road margin, dynamic-limit margins, terminal test stored as a Boolean beside G. Keep a separate continuous **search-grade** G̃ for the mining algorithms (clipped severity, never used for labels). Defaults to start from: c = 1 m, ε = 0, hold 4 s, q_clear = 5 m. **Accept:** corridor-family unit tests reproduce the PDF's numbers (15 m, 16 m, three labels) *through the new margin code path*; sensitivity table of category changes under ±50% of each constant → `results/g_sensitivity.md`.
- **A2 Execution record + manifest.** Implement the PDF §12 schema: `case_spec` (intent) / `execution_record` (measured) / `learning_example` (features only) kept separate. Mandatory execution fields: resolved transforms and blueprint IDs, actual geometry/physics values, every policy descriptor + version, capture/delivery/decision/command/response timestamps, per-world raw and normalized margins, collision/road events, witness/refutation traces, policy/world coverage, termination cause, seed, code version, invalidity reasons, checksums, `split_group`, `parent_family_id`, `varied_factor`. **Accept:** `validate_records.py` checks schema, `V == max_j min_w R`, evidence consistent with `contract.classify`, no NaN written as numbers, hidden metadata absent from features.
- **A3 Mining on analytic families (keep trying).** Extend `reference/mine_corridor.py` to (i) RC12 (θ = gap, v, τ, b, prefix) and RC17 (delay) and (ii) the information tree (θ = D, t_obs, T_lat, τ_plan). Add **information-gap mining**: objective V_info = V_hind − V subject to V_hind ≥ ε (maximize), via CEM and via straddle on V with V_hind as a constraint. **Accept:** table of boundary distance / surrogate F1 / pairs per algorithm over ≥ 10 seeds; for the info tree, the found U_INFORMATION region matches the analytic threshold within one grid step.

### Phase B — CARLA bring-up (gates everything real)
- **B1** Pin runtime: server build, client wheel version, map name/hash, vehicle blueprints, physics profile id, settings (sync, 0.05 s, substepping 0.01×10). Write `results/runtime.json`.
- **B2 Calibrate** (`--calibrate`): sweep **both** `tire_friction` and `max_brake_torque`; measure achievable straight-line decel vs both; measure delivered-command→response delay, steering-rate response, lateral tracking, combined brake–steer limits for the actual blueprint. Output a versioned physics profile. **Accept:** decel mapping covering ~5–9 m/s² with residuals; profile hash stored in every record.
- **B3 Selftest + determinism.** keep holds speed (|Δv|<0.5 m/s/8 s); emergency brake decel within 10% of the commanded limit; left lane change ends at lateral ≈ −lane_w ± 0.3 m; same case twice (and across two servers) agree to < 0.05 m / 0.05 m·s⁻¹. If not: diagnose (settle ticks, spawn z, substepping, controller state) before anything else.
- **B4** Timing profile (ms/tick, s/rollout, s/oracle call pruned vs full), parallel scaling with `--ports`.
- **B5** Geometry: OBB polygons per tick, swept check, collision-sensor corroboration, separate permitted-road test (`map.get_waypoint(..., project_to_road=False)`), lane invasion not used as a safety verdict.
- **B6** Observation and command queues: explicit capture/delivery/decision/command/response timestamps; quantize delays to the tick grid and log realized delay; during a delay apply the actual previous control; no backdating. Observation gate over simulator-derived structured observations first (cameras later).
- **B7** Actor controllers: bounded cruise, delayed braking, gap-triggered yield, explicit non-yield, hidden-until-reveal actor; exogenous modes chosen per world, not re-drawn per tick.
**Accept for Phase B:** `results/carla_selftest.txt`, `runtime.json`, determinism table, timing table.

### Phase C — build the physical cases (the PDF's order; reuse three sites)
Site registry first: `corridor`, `two_lane`, `occlusion` (find once with the waypoint API; straight length ≥ ~350 m, rightmost lane with a left neighbour; try Town04, then Town06). Every case = `case_spec` YAML + a builder + a policy library + a world set + an evidence run. Build a witness first, then place the hazard relative to it; the negative counterpart changes exactly one declared variable. Build order and acceptance:
1. **Foundation: RC01, RC02, RC16, RC17.** **Accept:** CARLA gap/delay boundaries from sweeps match the analytic anchors `d_required` and `τ*` after substituting the *calibrated* b and delay (report the residual in metres/seconds); RC16 point-safe maneuver refuted by body-level trace while a wider path is a witness; RC02 reported as `U_LIBRARY` in CARLA (and `U_MODEL` only for the reference model).
2. **Core: RC03, RC07, RC12, RC19, RC20.** **Accept:** RC03 shows r(brake_hold)<ε and r(bypass)≥ε; RC07 shows primary-only passes while handback fails and continuation passes; RC12 feasible-delay *set* recorded (not a single threshold); RC19 benign anchor + nuisance pair agree within tolerance; RC20 shows UNRESOLVED → R_REF when a held-out witness is admitted with physics unchanged.
3. **Compositional: RC04, RC05, RC06, RC08, RC15, RC18.** At least one audited positive witness per family.
4. **Partial observation: RC10, RC11, RC14.** Two worlds with identical permitted histories before reveal (unit-test it). Witness for RC10 = shared slowdown trunk + branch on the delivered observation (construct it with branch-MPC / contingency-MPC ideas: Chen et al. arXiv:2109.05128; Alsterda & Gerdes arXiv:2102.12045). Cross-check the reveal-time boundary against `reference/info_tree.py` and, if feasible, the Zhang & Fisac (RSS 2021) closed-form safe set. **Accept:** reveal sweep shows the R_REF → U_INFORMATION transition; common stop/robust prefix present ⇒ NOT classified information-infeasible; leakage test (no hidden id/colour/blueprint/coordinate in policy inputs).
5. **Difficult negatives: RC09, RC13.** Only after simple negatives work. RC09 needs a counterexample for *every* policy in the stated class including intermediate braking/forward progress.
First audit batch (PDF §11.1): 6 selected contrasts × 3 parameter settings × 3 matched seeds = 54 physical instances; with 8 policies × 2 worlds = 864 policy/world executions. Inspect jointly one positive, one refuted policy and one unresolved instance before expanding. A template that repeatedly violates its intended mechanism is revised or excluded, never relabeled.

### Phase D — boundary mining inside the case families (continue the algorithm work)
- **D1** For each built family, define θ from its `sweep` block, run **sobol (baseline) → straddle → bisect** (and cem/bo_lcb as falsifiers, qd_boundary at larger budget), warm-starting later stages with earlier rows. Use the label-grade evidence for the final scenes, search-grade G̃ for the search.
- **D2** Score boundary localization against the analytic anchors where they exist (RC01/02/12/17, info tree); elsewhere against a dense local scan.
- **D3** Emit matched pairs (single declared changed variable, same `split_group`), each with a human-readable description ("same scene except reaction delay 1.0 s vs 1.2 s") and both members re-labeled with the *full* R matrix.
- **D4** Near the threshold (within 1σ of repeat variation) run ≥ 3 matched repeats; if the sign changes mark the band UNRESOLVED. Never move ε after seeing student results.
- **D5** Information-gap mining on RC10/RC11 and the hidden-actor variant of the two-lane scene: find scenes with V_hind ≥ ε and V < ε.
- Budget realism: plan ≈ 100–200 scenes per algorithm per family; stay inside the 864-execution audit batch first.
**Accept:** per-family table (algorithm × {unrecoverable count, near-boundary count, boundary distance, pairs, evidence mix}); recoverability map for at least RC01 and the two-lane family; list of matched pairs.

### Phase E — interfaces for teammates (do not build the world model)
- **E1 Dataset export**, JSONL + npz traces: `case_id, family_id, split_group, theta, history (delivered observations with timestamps), policy_library_id/version, world_set_id, R (masked), V, V_hind, info_gap, evidence, margin_config, physics_profile, terminal {mode, boolean}, pair_id/role/varied_factor, parent_id, variant_kind ("appearance" for Cosmos variants)`. **Appearance variants (Cosmos Transfer1) inherit θ and R; they never create new dynamics.** Split complete counterfactual families together (all distance neighbours, observation variants, renderings, policy-withholding variants).
- **E2 Evaluation harness** (metrics only + a trivial TTC-threshold baseline to test it): false admission, false rejection, retained feasible actions, profile fidelity near ε, multi-step consistency, information correctness (pre-reveal action consistency, premature branching), evidence integrity (resolved fraction, invalid-run rate, cost), paired independent episodes, intervals at episode/family level; mock vs CARLA separate.
- **E3 Planner backend interface**: `PlannerBackend` with `TTCBrakePlanner` as default; leave a documented stub for Autoware (Open AD Kit CARLA sample, `autoware_carla_interface`, CARLA 0.9.16). Do not integrate Autoware before the finite reference tests are stable.

## 6. Engineering rules
- One source of truth for constants (`MarginConfig`, physics profile, library and world-set versions); write them into every record.
- Backends implement only `rollout(theta, world, policy) -> trace`; keep `core.oracle` signature stable; every algorithm stays ask/tell.
- Every long run checkpoints per batch; resume by skipping completed work.
- When something looks wrong (selftest off, nondeterminism, spawn overlap, anchors not reproduced) **stop and diagnose before running search**, then report what you saw.
- Do not claim what you did not produce. If an acceptance criterion is not met, say which and why.
- Small reviewable commits: tests → margin/config → records → CARLA bring-up → cases → mining → export.

## 7. Final report to hand back
1. Runtime/versions/seeds/servers used. 2. Acceptance checklist per task (✅/⚠️/NOT EXECUTED) with paths. 3. Tables: anchors vs CARLA (residuals), G sensitivity, determinism, timing, per-family mining, evidence mix. 4. Figures: recoverability maps, feasible-delay sets, reveal-time transition. 5. Matched pairs (human-readable). 6. Known issues/surprises and any case template that failed its intended mechanism. 7. Open questions for Andy/Victor (reserve c, normalization, ε, library growth rule, Ω contents) and for Kent (θ schema extractable from logs).
