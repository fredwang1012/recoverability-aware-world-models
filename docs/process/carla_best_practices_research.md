# CARLA 0.9.16 setup vs official docs and literature

Date: 2026-10-20. Read-only research; no repo code was inspected or run, and the CARLA server was not touched. The "our setup" column comes from the task brief only.

Source key. "Opened" means the page text was actually fetched and read. GitHub issues were read through the GitHub API (api.github.com/repos/carla-simulator/carla/...), which carries the same content as the issue pages.

- [D1] https://carla.readthedocs.io/en/0.9.16/adv_synchrony_timestep/ (opened)
- [D2] https://carla.readthedocs.io/en/0.9.16/adv_rendering_options/ (opened)
- [D3] https://carla.readthedocs.io/en/0.9.16/tuto_G_control_vehicle_physics/ (opened)
- [D4] https://carla.readthedocs.io/en/0.9.16/adv_opendrive/ (opened)
- [D5] https://carla.readthedocs.io/en/0.9.16/python_api/ (opened; about 255k chars, read in 3 chunks through a summarising fetcher, so it may omit things)
- [D6] https://carla.readthedocs.io/en/0.9.16/foundations/ (opened)
- [D7] https://carla.readthedocs.io/en/0.9.16/ref_sensors/ (opened)
- [D8] https://carla.readthedocs.io/en/0.9.16/tuto_G_openstreetmap/ (opened; only for the generate_opendrive_world example)
- [P1] Chance, Ghobrial et al., "On Determinism of Game Engines used for Simulation-based Autonomous Vehicle Verification", https://arxiv.org/pdf/2104.06262v2 (opened; full text extracted with pdftotext; abstract at https://www.arxiv.org/abs/2104.06262)
- [G1] https://github.com/carla-simulator/carla/pull/9947 (set_target_velocity doc clarification)
- [G2] https://github.com/carla-simulator/carla/issues/7202 (behavior of set_target_velocity)
- [G3] https://github.com/carla-simulator/carla/issues/3710 (slowing after high set_target_velocity)
- [G4] https://github.com/carla-simulator/carla/issues/8400 (instantaneous speed)
- [G5] https://github.com/carla-simulator/carla/issues/6477 (delayed acceleration, substepping)
- [G6] https://github.com/carla-simulator/carla/issues/7108 (low-rate sensors in sync mode)
- [G7] https://github.com/carla-simulator/carla/issues/6963 (generate_opendrive_world does not block)
- [G8] https://github.com/carla-simulator/carla/issues/8112 (brake spike below 5 m/s; question text only, no answer visible)

Not opened, so not used as evidence (search-result snippets only, or nothing):

- Dosovitskiy 2017 (the PDF returned binary only).
- Scenic (arXiv 2010.06580), Scenic+VerifAI (arXiv 2107.04164) and Ding et al. 2022 (arXiv 2202.02215): snippets only.
- ScenarioRunner / OpenSCENARIO docs: not searched or opened.
- GitHub issues for the "RPC before draining the image" deadlock and for repeated `load_world` crashes: searched, none found.

## 1. Item-by-item table

| # | Our setup | What docs / literature say | Verdict | Source |
|---|---|---|---|---|
| 1 | Synchronous mode, one client ticking | The server waits for the client tick. Only one client should tick. With a Traffic Manager (TM) running it must also be in sync mode (n/a here, no TM). Fixed step plus sync gives "client in total control". | matches | D1, D6 |
| 2 | fixed_delta_seconds = 0.05 | "In synchronous mode, always use a fixed time-step." 0.05 is the value in the docs' determinism examples and in Chance et al. Physics cannot run properly above 0.1 s. | matches | D1, P1 |
| 3 | Substepping on, max_substep_delta_time 0.01, max_substeps 10 | These are the defaults. Constraint: fixed_delta <= max_substep_delta_time * max_substeps (0.05 <= 0.1, satisfied). Docs: the physics substep should be "at least below 0.01666 and ideally below 0.01"; their own test showed velocity deviations once the physical delta exceeds 0.01. From memory (not in the pages read): 0.05 s splits into 5 substeps of 0.01, exactly at the "ideal" bound. | matches (at the edge of the "ideal" bound) | D1; G5 (one user got more precise dynamics with 0.001 x 100, another saw no change) |
| 4 | no_rendering_mode on, off for video | Docs: no-rendering "skips everything regarding graphics"; cameras and GPU sensors return empty data. Docs do not say whether physics is affected. I found no source claiming a physics difference, and none ruling it out. | matches for sensors; physics-neutrality unverified in docs | D2, D5 |
| 5 | No Traffic Manager; own FSM controllers via apply_control | Removes the TM as a nondeterminism source (TM determinism issue #3839 and 0.9.11 notes seen only as titles/snippets). Chance et al. recommend hard-coded actor trajectories and deterministic controllers. | matches | P1 Sec. 7.1.2, 7.3.5; D1 |
| 6 | Custom OpenDRIVE via generate_opendrive_world(vertex_distance 2.0, max_road_length 50, additional_width 0.6, mesh visibility True) | Your values equal the API defaults (2.0, 50.0, 0.6, True). The docs' OSM example uses max_road_length 500. Docs: the .xodr is used as-is, errors propagate, validate it. Default wall_height is 1.0 m (walls stop vehicles falling off); the OSM tutorial sets 0.0 to avoid unwanted collisions with overlapping walls. reset_settings defaults to True. | matches (check wall_height and reset_settings, see 2.6) | D4, D5, D8 |
| 7 | Map generated once per server session because repeated load_world crashed | Docs say "reload the world for each repetition" for determinism. Nothing on repeated-load crashes; no GitHub issue found. Issue #6963: generate_opendrive_world can return before the world is ready on large maps (0.9.13, 10 km map). | deviates-acceptable (undocumented crash; restart server per session) | D1, G7 |
| 8 | Ego = vehicle.tesla.model3 | Standard blueprint. A #6477 commenter with a Tesla Model 3 reported coarse throttle response (user report, not maintainer-confirmed). | matches | G5 |
| 9 | apply_physics_control right after spawn (tire_friction 5.0, max_brake_torque calibrated) | Docs: get_physics_control(), modify, apply. API: "Applies a physics control object in the next tick". Changes are runtime-only. Defaults: tire_friction 2.0, max_brake_torque 1500 N*m. No semantics or valid range for tire_friction in the docs. | matches, provided a tick passes before behaviour is relied on (your 2 s warm-up does this) | D3, D5 |
| 10 | 2 s warm-up drive at declared speed; set_target_velocity pinning for other moving actors | API (via PR #9947): set_target_velocity sets a world-space linear velocity, applied before the physics step so friction and other forces still act; it does not change rotation. Not documented as the way to spawn at speed. #7202, #8400, #3710: users get stalled or decaying speed (gear left at 0, PID overshoot); #8400 found no exact-speed API. A #6477 user moved to the Ackermann control API (0.9.14+) to set speed directly (not opened). | deviates-acceptable (no documented initial-speed API; verify realised speed at t0) | D5, G1-G5 |
| 11 | Static obstacles held with hand brake; one frozen with set_simulate_physics(False) | set_simulate_physics has a one-line doc. hand_brake defaults to False; controls apply on the next tick. Nothing on collision-sensor behaviour with a physics-disabled actor. | deviates-acceptable (OBB check does not depend on the sensor) | D5, D7 |
| 12 | Collision by OBB overlap each tick, corroborated by collision sensor | Sensor emits one event per collision per frame and uses "fake" actors for static scenery; silent on missed collisions. Chance et al.: after a collision variance becomes non-permissible, and it can reach uninvolved actors about 5 s later; they recommend terminating tests at first collision. | matches (independent geometric check is stronger than the sensor alone) | D7, P1 Sec. 6 |
| 13 | Same-session repeats differ by up to 0.028; cross-session shift 0.196 | Chance et al. (CARLA 0.9.6, UE4.22, sync, fixed dt 0.05, RTX 2080, Linux): deviation about 1e-13 cm in 997 of 1000 runs, but three runs exceeded 10 cm; vehicle-involving scenarios reached 59 cm; variance rises with CPU/GPU load. Docs claim determinism with sync + fixed step + world reload + seeds but give no tolerance. | matches (you measured it and flag it) | D1, P1 |
| 14 | Cross-server determinism untested | Chance et al.: results depend on CARLA/UE version and hardware; keep versions fixed. | deviates-acceptable; state as limitation | P1 Sec. 7.2 |
| 15 | Video: RGB + instance-seg cameras in sync; drain images right after each tick | Docs pattern: camera -> queue; tick; read from queue. GPU sensors deliver "with a delay of a couple of frames". The foundations page's code ticks first while its prose says tick after retrieval. #7108 documents sync-mode hangs when sensor rate and tick mismatch. World.tick(timeout=10 s): without a tick, sync sim freezes. The specific RPC-before-drain deadlock is not in docs or issues I found. | matches the documented queue pattern; your deadlock finding is undocumented | D1, D5, D6, G6 |
| 16 | Declared dynamic limits (lat 8, friction-circle 8 m/s^2) in margin because tuned friction is non-physical | No source prescribes this; it is a modelling choice. Docs give tire_friction no physical meaning, so a declared limit is defensible. #8112 (question only; no answer visible) reports a brake-deceleration spike near 5 m/s, so braking calibrated at one speed may not transfer. | deviates-acceptable (project-specific; document it) | D3, G8 |

## 2. Concerns ranked by impact on result validity

1. Post-collision and near-boundary determinism (highest). Chance et al. say post-collision CARLA variance is non-permissible and spreads to uninvolved actors, and recommend ending tests at first collision. Your 0.196 cross-session shift and 0.028 same-session spread are comparable to margins near the boundary. Any margin or label that depends on motion after first contact is least trustworthy. Check that margin and label use only pre-contact states (videos already freeze at contact), and mark margins within about +/-0.2 of the boundary uncertain (report already flags this).
2. Resource load changes variance (P1: variance rose with CPU/GPU load, some cases breached tolerance above 75% load). Recording videos on the same 6 GB GPU while labelling could shift results. Keep labelling runs and recording runs separate and do not take labels from video re-runs. Cheap mitigations from P1: higher process priority (TaskSettings.Priority on Windows), close other tasks.
3. Initial-speed realism. set_target_velocity is a world-frame kick before the physics step (G1); wheels, drivetrain and gear are not consistent with it, so transients can appear (G3, G5). The 2 s warm-up is the right idea but should be verified: log realised speed and yaw at the end of warm-up per scene. Pinning other moving actors every tick overrides tyre and brake physics for them, so they are kinematic, not dynamic; say so in the report.
4. apply_physics_control timing. API says it applies on the next tick. Anything reading speed or brakes in the same tick sees old physics. After the warm-up this is harmless; confirm by reading get_physics_control() back after the first tick.
5. fixed_delta 0.05 vs substep 0.01. Within the documented constraint and defaults, at the "ideal" bound. No violation. A sensitivity run at 0.025 s would show whether margins move by more than the 0.2 noise.
6. generate_opendrive_world details. (a) reset_settings defaults to True, so synchronous mode and fixed delta must be set after the call (or pass reset_settings=False); D1 also says to enable sync before loading. Confirm in code. (b) wall_height default 1.0 m: if a scenario can leave the road the ego may hit a wall; the docs' OSM tutorial uses 0.0. (c) #6963: on large maps the call may return before the world is ready; on 1.5 km this is unlikely, but a few warm-up ticks cost nothing.
7. no_rendering_mode and physics. Docs are silent and I found no report of an effect, but silent is not verified. A render-on vs render-off numeric margin comparison on the same scenes gives direct evidence.
8. One map per session. Deviates from "reload per repetition". Compensate by destroying all actors and ticking between runs and by logging run order per session so order effects can be checked.
9. Camera deadlock. Drain-after-tick matches the documented queue pattern; the RPC-before-drain deadlock is undocumented. Worth a note in CARLA_SETUP.md.
10. Collision sensor vs frozen actors: unknown behaviour; irrelevant while the OBB check is primary.

## 3. Recommended changes

| Change | Effort | Re-run cost |
|---|---|---|
| A. Assert that synchronous_mode, fixed_delta_seconds and substepping are set after generate_opendrive_world (or pass reset_settings=False); log settings with each result | 10 min | None if already true; if wrong, all runs affected |
| B. After first tick post-apply_physics_control, read get_physics_control() and assert tire_friction/max_brake_torque; log realised speed/yaw at end of warm-up per scene | 1 h | None for stored data; new runs only |
| C. Report an explicit noise band: treat |margin| < about 0.2 as undetermined; repeat boundary scenes n times and report flip rate; monitor maximum deviation, not mean (P1 Sec. 7.4.1) | 2-4 h analysis | Re-runs only of near-boundary scenes (n x those) |
| D. Keep labelling runs free of recording load; raise server priority; close other GPU/CPU tasks (P1 Sec. 7.3.3) | 15 min | None; applies to future runs |
| E. Confirm margins and labels use only pre-contact states (or flag post-contact margins) | 1-2 h code reading | None if already true |
| F. Render-on vs render-off margin comparison on 5-10 scenes, plus fixed_delta 0.025 sensitivity | 2-3 h | About 20 scene runs per variant |
| G. wall_height = 0.0 if scenarios can leave the road, or confirm walls are never reached | 15 min | Full re-run only if walls were ever hit |
| H. Document: other actors kinematically pinned, cross-server/hardware determinism untested, CARLA version / UE4 build / driver | 1 h writing | None |
| I. (Optional) Replace set_target_velocity-only start with warm-up plus throttle/PID or Ackermann speed hold (Ackermann exists since 0.9.14 per G5; docs not opened) | 1-2 days | Full re-run (about 2230 stored records) |

Suggested order: A, B, E, D, C, then F. Nothing I found forces a full re-run, conditional on A and E being true in code (not inspected).

## 4. What I could not verify

- Whether no_rendering_mode changes physics: not in docs; no issue found.
- The "RPC before draining the camera image" deadlock: not found.
- Repeated load_world / generate_opendrive_world crashes: not found.
- Exact substep count for 0.05 s: from memory (5 x 0.01), not read.
- Dosovitskiy 2017, Scenic, VerifAI, Ding et al., ScenarioRunner/OpenSCENARIO: not opened. A search found nothing on determinism or reproducibility in Scenic/VerifAI. Any claim that they use sync mode without bit-level determinism is from memory only.
- 0.9.11 release notes (deterministic TM, deterministic_ragdolls, sensor noise_seed) and issue #3839: titles/snippets only.
- Chance et al. used CARLA 0.9.6 on Linux; applicability to 0.9.16 on Windows is by analogy.
