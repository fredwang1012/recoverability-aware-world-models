# Verification of "Recoverability: definitions, formulations and a constructed CARLA case catalogue"
Checked 15 Oct 2026. Scope: every checkable claim in the PDF, plus what I added. Tags: **VERIFIED** (I confirmed it by running code, reading the primary/official source, or inspecting the installed CARLA 0.9.16 client), **PARTIAL**, **NOT VERIFIED** (could not confirm in this pass), **MINE** (my addition).

## 1. What the PDF is, and what it is not
- It is a **specification**: a contract, a mathematical formulation, 20 case cards (RC01–RC20), a CARLA construction guide, a labeling protocol, a build order, and a manifest schema. It says itself that nothing was executed in CARLA.
- It does **not** contain: executed results, measured CARLA vehicle capabilities, runnable OpenSCENARIO, or the "machine-readable catalogue / YAML" it mentions (not in the upload). I generated `cases/catalogue.yaml` from the text.
- All numbers are starting points. The only numbers that are *derivations* are the §4.1 example, which I checked.

## 2. Mathematics — verified by code (`mining/tests/`, 9 tests pass)

| Claim | Status | How |
|---|---|---|
| D_stop = vτ + v²/(2b) = 15 m, d_required = 16 m for v=12, τ=0.25, b=6, c=1 | VERIFIED | closed form + numerical integration (`reference/corridor.py`) |
| gap 18 m → ideal witness with 3 m final clearance; 15.5 m → no contact but reserve violated; 12 m → contact | VERIFIED | simulated final clearance 3.006 m (Euler error); labels match |
| Rec ⇔ ∃π∀ω G≥ε and Unrec ⇔ ∀π∃ω G<ε are exact duals | VERIFIED | 300 random policy×world matrices |
| Per-world feasibility does not imply causal recovery (§2.4) | VERIFIED | constructed example; also **V ≤ V_hind always** (max-min ≤ min-max) |
| Missing entries must stay unresolved, a single violating world refutes a policy, a witness needs all worlds | VERIFIED | `contract.classify` tests |
| Comfortable braking must not certify impossibility (needs a genuine b_max) | VERIFIED | `model_certificate` test: with b_max=10 no impossibility proof at gap 12 |
| "Do not assume delay sweeps are monotonic" (§4.3) | PARTIAL | the corridor family *is* monotone in delay (tested); the warning matters for moving-traffic families (RC06, RC12 with traffic), not tested |
| 54 instances = 6 contrasts × 3 settings × 3 seeds; 864 executions = 54 × 8 policies × 2 worlds | VERIFIED | arithmetic |

**Derived boundaries I added (checked against simulation within 5 ms):** largest admissible baseline prefix (RC12) `T_p* = (gap − D_stop − c)/v` (= 0.167 s at gap 18); largest admissible delay (RC17) `τ* = (gap − v²/(2b) − c)/v` (= 0.417 s). These are the **analytic anchors CARLA must reproduce after calibration**.

**Information-tree check (RC10/RC11), `reference/info_tree.py`.** Toy but exact: an ego that cannot stop must clear one of two corridors; a hidden mode blocks one; the mode is delivered at t_obs. Result: the label flips from R_REF to U_INFORMATION at exactly the analytic reveal threshold `t_obs* = t_arr_max − τ_plan − τ − T_lat − reserve` (0.237 s for the pilot numbers), robust V decreases linearly with t_obs while hindsight V_hind stays constant, a too-close obstacle gives U_LIBRARY (no world has a witness), and a far obstacle gives R_REF via a common stop regardless of reveal time. This matches the PDF's logic point for point, including "a common stop invalidates the information-infeasibility claim".

## 3. CARLA claims

| Claim in PDF | Status | Evidence |
|---|---|---|
| `fixed_delta_seconds <= max_substep_delta_time × max_substeps`; pilot 0.05 / 0.01 / 5 | VERIFIED | CARLA synchrony doc and server source; pilot satisfies it **with equality**; the doc's own example uses max_substeps=10 and the source accepts 1–16, so I recommend **10** (margin; "time deltas are not guaranteed" otherwise) |
| Fixed time step + synchronous mode for reproducibility; "a seed alone is not proof" | VERIFIED | docs; determinism is something you must *test* (task B3) |
| Traffic Manager deterministic mode needs seed + sync mode | VERIFIED | 0.9.11 release notes: `set_random_device_seed`, world and TM both synchronous |
| "Reseeding needed after reload" | NOT VERIFIED | not in the sources I retrieved; irrelevant to us (we do not use the Traffic Manager) |
| Axes x-forward, y-right, z-up, meters, degrees | VERIFIED | CARLA coordinates page: left-handed UE4. Yaw is clockwise-positive viewed from above, consistent with `lat = +y` being **right** in our road frame |
| Collision events contain the other actor and impulse | VERIFIED | API reference + installed 0.9.16 client: `other_actor`, `normal_impulse`, `frame`, `timestamp` |
| Bounding boxes and world-space vertices | VERIFIED | installed client: `BoundingBox.get_world_vertices`, `get_local_vertices`, `extent` |
| Runtime vehicle/wheel physics control | VERIFIED | `WheelPhysicsControl`: `tire_friction`, `max_brake_torque`, `max_steer_angle`, `damping_rate`, `radius`, stiffness terms. **Note:** `tire_friction` is a scalar, not an SI μ; `max_brake_torque` is a *second* deceleration knob. Calibrate both |
| ScenarioRunner: Python scenarios, trigger/behavior/criterion structure | VERIFIED (structure) / NOT VERIFIED (0.9.16) | scenarios subclass `BasicScenario` with a py_trees behavior tree and criteria; **the repo's version table lists releases up to 0.9.15** and says versions must match CARLA. Compatibility with 0.9.16 is unconfirmed → keep our own harness |
| "GPU sensor outputs can arrive later than their capture frames" | NOT VERIFIED | not found in this pass. Irrelevant for the first phase (structured observations); verify before any camera-based work |
| Waypoint/OpenDRIVE site construction, Recorder, `World.get_snapshot` | VERIFIED | present in the installed client |

## 4. Citations

| Ref | Status |
|---|---|
| [2] Fraichard & Asama, *Inevitable collision states — a step towards safer robots?*, Advanced Robotics 18(10):1001–1024, 2004, DOI 10.1163/1568553042674662 | **VERIFIED** (scispace record, dblp, citing papers). The abstract states an ICS is a state from which *no matter what the future trajectory followed, a collision eventually occurs*, taking into account the dynamics of system and obstacles |
| [3] Zhang & Fisac, *Safe Occlusion-aware Autonomous Driving via Game-Theoretic Active Perception*, RSS 2021, arXiv:2105.08169, DOI 10.15607/RSS.2021.XVII.066 | **VERIFIED**. Stronger than the PDF uses it: it derives *the set of initial conditions from which the vehicle is guaranteed to avoid collisions* under an initially hidden actor, with public code (tested on CARLA 0.9.11; safe sets in closed form with simplified dynamics) |
| [1] the proposal (pp. 2–6) | **VERIFIED** against the uploaded draft: r_π, V_Π, A_rec, witness/counterexample/unresolved, terminal continuation, §10 reuse boundary all match |
| [4]–[12] CARLA documentation | URLs plausible; claims above checked individually |

## 5. Issues I found (not errors in the math, gaps between spec and our code)
1. **Our current margin does not follow the PDF.** We used heuristic headway/lateral terms and a continuous failure severity. The PDF wants: signed **swept-footprint clearance minus a declared reserve**, permitted-road margin, dynamic-limit margins, latency-consistent margins, a **Boolean terminal test stored separately**, all in declared units with declared normalization. → task A1.
2. **Two margins are needed.** The PDF's G puts `−∞` on a failed terminal Boolean, which is right for *labels* and useless for *search* (no gradient). Keep a label-grade G and a continuous, clipped search-grade G̃; never label with G̃.
3. **Pruning creates unresolved entries.** Our exact pruning preserves V but leaves policy×world entries unrun; they must be masked (NaN), never written as negatives. Fixed in `core.oracle` (non-full mode only decides R_REF vs not-R_REF).
4. **No U_INFORMATION in the old mining scene.** 0 of 200 random scenes in the two-lane mock were information-limited, so that scene cannot test the information hypothesis. RC10/RC11 (and RC14) are required for it.
5. **U_MODEL cannot be earned in CARLA.** It needs a sound proof under reference dynamics with a genuine b_max; only the corridor family has one (`model_certificate`). In CARLA the strongest negative is U_LIBRARY (or U_INFORMATION within the library).
6. **Simulator version.** The PDF says not to upgrade just for this. Our backend targets 0.9.16; it should work against any 0.9.x server whose client wheel matches. Pin whatever is installed and record it.
7. **Parameters that must be declared before running:** reserve c, normalization q_i, ε, hold duration, horizon, post-maneuver interval, library/world versions. The PDF leaves these open by design; defaults used in the sandbox: c = 1 m, ε = 0, hold 4 s.

## 6. Additions from research (each with a source I read)
- **Branch MPC** (Chen, Rosolia, Ubellacker, Csomay-Shanklin, Ames; arXiv:2109.05128) plans over feedback policies with a shared trunk and branches; **Contingency MPC** (Alsterda & Gerdes; arXiv:2102.12045) plans a nominal and a contingency branch with a shared initial segment. Use either as the **construction method for the RC10 witness** (shared slowdown trunk, branch on the delivered observation).
- **Zhang & Fisac safe sets** as an independent analytic reference for the RC10/RC11 reveal-time boundary (public code).
- **CommonRoad-Reach** (Irani Liu, Würsching, Klischat, Althoff; ITSC 2022; `pip install commonroad-reach`): reachable sets and drivable area, and its authors list time-to-react among the metrics it supports. Candidate verifier for a *reduced-model* U_MODEL-style check. **Invariably safe sets** (Althoff group) are the established tool for the "indefinite safety needs an invariant set" remark in the PDF's terminal section.
- **Braking-ICS / passive safety** (Fraichard): "whatever braking trajectory, a collision occurs before standstill". Report **brake-only recoverability as an explicit sub-library result**.
- **Minimum safe distance with response time** (KIT, RSS-style): cross-check for RC01/RC02 anchors.
- **Criticality metrics** (Westhofen et al., arXiv:2108.02403; online catalogue): log TTC, TTR, BTN, STN beside V; BTN ≥ 1 is the analytic brake-only point of no return.
- **Hindsight gap V_hind − V** (mine): the numerical price of not knowing the hidden mode; the quantity the information-structure hypothesis predicts. Implemented in `contract.py` and `core.oracle(full=True)`.
- **Information-gap mining** (mine): search for scenes with V_hind ≥ ε and V < ε by maximizing V_hind − V; run on RC10/RC11 families.
- **Evidence-aware classifier** (`contract.py`), **reference sandbox** (`reference/`), **tests**, **catalogue.yaml**.

## 7. Do the mining algorithms work? (analytic-ground-truth test)
Family RC01/RC02/RC12/RC17 in the PDF's 1-D model, θ = (gap, v, τ, b), 80 scenes per run, 5 seeds, boundary known in closed form. Search sees only a clipped margin.

| algorithm | unrecoverable | within ±1.5 m of boundary | mean \|G\| of 20 closest (m) | GP-surrogate F1 | pairs | pair midpoint error (m) |
|---|---|---|---|---|---|---|
| sobol | 29.4 | 5.6 | 2.45 | 0.947 | – | – |
| cem | 68.2 | 3.6 | 4.23 | 0.880 | – | – |
| bo_lcb | 72.4 | 1.4 | 7.75 | 0.886 | – | – |
| **straddle** | 38.4 | **38.6** | **0.19** | **0.987** | – | – |
| qd_boundary | 29.4 | 6.2 | 3.02 | 0.919 | – | – |
| **bisect** | 40.4 | 24.2 | 0.50 | 0.888 | **9.6** | **0.44** |

Reading: **straddle localizes the boundary about 12× better than random sampling** (0.19 m vs 2.45 m) and has the best surrogate F1; **bisect** is the matched-pair generator (0.44 m midpoint error); CEM/BO-LCB are falsifiers, bad at the boundary; MAP-Elites needs a larger budget. This is a 4-D analytic model, not CARLA; it shows the *algorithms* are sound, not that the CARLA labels are. Note: the earlier 1-NN "level-set F1" metric is retired in favour of surrogate F1 and boundary distance.

## 8. Still unverified / to do
- Any CARLA behaviour (backend never run against a live server).
- ScenarioRunner with 0.9.16; GPU-sensor delay statement; TM reseeding after reload.
- Whether CARLA's measured braking and delay reproduce the analytic anchors (this is the acceptance test of the whole contract).
