# Research Notes: Mining Methods and Simulation Validity

As of 2026-10-20. Background reading for the CSC2626 report (Approach 1, CARLA). This note places our mining and simulation choices against the literature. It does not change any result.

How sources are marked. "Opened" means I fetched the paper or doc page and read the abstract or the relevant text (not always the full paper). "Search only" means I saw it only in a search-result summary and did not open the page. "From memory" means neither. Section 4 lists every source with its status. Where an arXiv id is given it was shown on the page I opened.

## 0. Short answer

- The closest formal notion to our "unrecoverable" is the **inevitable collision state** (Fraichard and Asama 2004) and, in control theory, the **backward reachable set / viability kernel** computed by Hamilton-Jacobi reachability with a worst-case disturbance. Our U_LIBRARY label is a finite-library, finite-world-set, simulation-evaluated version of "no control avoids the failure set whatever the disturbance does".
- Our labels are weaker in one way and different in another. Weaker: we search a finite move library in a finite set of worlds, so "unrecoverable" means "unrecoverable for this library", and the report already says so. Different: our U_INFORMATION class (every world solvable in hindsight, no single committed move works) has no direct counterpart in ICS or reachability; the nearest analogue is the gap between a feedback/hindsight value and a non-anticipative (committed) value, which I only know from stochastic-programming and robust-control folklore (from memory, unverified).
- Our mining algorithms are standard black-box falsification and active-learning tools (Corso et al. survey; Bryan et al. and Inatsu et al. for straddle; Fontaine and Nikolaidis for QD). What is unusual is the target: not "does a failure exist" but "is the scene unrecoverable, and where does it flip", with matched pairs for counterfactual analysis. I did not find prior work that mines matched recoverable/unrecoverable pairs in this sense, but I did not search exhaustively.
- The weakest point of the simulation side is the plant: a 0.05 s step that produces a numerical snap to rest, a non-physical tyre friction of 5.0 with declared limits, and a 0.196 cross-session shift that is larger than most margins. These are the first caveats to state.

## 1. Mining unrecoverable, safety-critical and boundary scenes

### 1.1 Method families

"Relation" says how the family relates to what we run: same, adapted, or not used.

| Family | One-sentence description | Relation to ours | What it would add | Source |
| --- | --- | --- | --- | --- |
| Black-box safety validation / falsification | Treat the system and simulator as a black box and search environment parameters or disturbances for failures, for the most likely failure, or for the failure probability. | Same. sobol, cem and bo_lcb are the survey's sampling, optimisation and Bayesian-optimisation falsifiers. | Nothing new in algorithms. The survey gives a vocabulary (falsification, most-likely failure, failure probability) and says which of the three we are not doing (probability). | Corso et al. 2021 (opened) |
| Adaptive stress testing (AST) | Formulate failure search as an MDP over a sequence of stochastic disturbances and solve it with RL or MCTS, rewarding likely failures. | Not used, on purpose (our theta is a static vector). | Right tool once hidden-actor behaviour is a per-step disturbance (RC10/11/13/14 family). Also gives a likelihood term, so the most plausible unrecoverable scene, not just any. | Lee et al. 2020 (opened); Koren et al. (from memory, in `mining/README.md`, not opened) |
| Bayesian optimisation and level-set / active-learning boundary search | Fit a surrogate (usually a Gaussian process) and choose the next query to find a failure (BO) or to resolve where a function crosses a threshold (level set). | Same for straddle (Bryan et al.; randomised variant of Inatsu et al.) and bo_lcb. Adapted: we use a clipped margin as the function. | GP classification of the performance boundary for AV scenarios is established (Batsch et al.). Their work would give a citation that the boundary framing is standard in AV testing. Trust-region and multi-fidelity variants exist but were only seen in search results. | Inatsu et al. 2024 (opened); Batsch et al. 2019 (opened); Bryan et al. 2005 (from memory, not opened); TuRS-BSE and multi-fidelity BO falsification (search only) |
| Quality-diversity scenario generation | Fill an archive over user-chosen behaviour descriptors with the best scenario in each cell (MAP-Elites and descendants). | Same in spirit, adapted: qd_fail and qd_boundary use our margin as the quality score. Not a published implementation. | Coverage claims over descriptor space. The Nikolaidis review argues the point and lists lessons. CaDRE applies QD (CMA-ME per `mining/README.md`, not confirmed by me) to driving. | Fontaine and Nikolaidis 2021 (opened); Nikolaidis 2024 (opened); Mouret and Clune 2015 (search only); CaDRE (opened) |
| Importance sampling and rare-event simulation | Sample from a tilted distribution so rare failures occur often, then reweight to get an unbiased failure probability. | Not used. We sample uniformly in [0,1]^d and report counts. | A failure-probability estimate. Needs a distribution over scene parameters, which we do not have: our parameter ranges are chosen, not measured. | Zhao et al. 2016/2017, cut-in with cross-entropy skewing (opened) |
| Reachability, viability kernels, Hamilton-Jacobi, control barrier functions | Compute the set of states from which a safety constraint can (not) be maintained under best control and worst-case disturbance; barrier functions enforce forward invariance of a safe set. | Not used. Closest formal analogue to recoverability (see 1.2). | An exact (in low dimension) definition of the recoverable set, to compare against our labels in a reduced model such as the corridor family. | Bansal et al. 2017 (opened); Ames et al. 2019 (opened) |
| Scenario-based testing and scenario languages | Describe scenarios as functional (words), logical (parameter ranges) and concrete (values) levels, and sample or search them with a language or toolkit. | Adapted. Our families are logical scenarios and each mined scene is a concrete one. We use our own builders, not Scenic, VerifAI or OpenSCENARIO. | A shared, citable format. Scenic and VerifAI both drive CARLA and ship samplers and falsifiers. Exporting our scenes to OpenSCENARIO would make them reusable (not checked in detail). | Scenic, Fremont et al. 2020 (opened); VerifAI, Dreossi et al. 2019 (search result page only); PEGASUS levels (search only) |
| Adversarial and learned scenario generation | Perturb or generate actor trajectories by gradient attack, learned priors, diffusion or LLM guidance to make a planner fail while staying realistic. | Not used. We have one controlled ego and scripted actors, no learned planner. | A realism argument: our scenes are constructed, not drawn from traffic. STRIVE's solvability filter is the same idea as ours, see 1.3. | KING (opened); STRIVE (opened); CaDRE (opened); Ding et al. survey 2022 (opened); Song et al. 2025 survey (opened); LD-Scene 2025 (search only) |
| Counterfactual and near-miss mining from real data | Find subtly avoided risks in logged driving and perturb or replay them counterfactually. | Not used. | Grounding: a check that our constructed families resemble situations that occur in data. | SafeShift (opened); CounterScene and the 2025 position paper on near-miss synthesis (search only) |
| Inevitable collision states and RSS | ICS: states from which every trajectory collides. RSS: a rule-based model of what a responsible driver must do (safe distance, proper response). | ICS: closest definition (see 1.2). RSS: not used. | ICS gives the right name and a computability argument. RSS gives a rule-based comparison for the corridor family. | Fraichard and Asama 2004 (via Petti and Fraichard 2005, opened; the 2004 paper itself not opened); Shalev-Shwartz et al. 2017 (opened) |

### 1.2 Which is the best match to our definition, and where we differ

Our definition (from `docs/REPORT.md`): a scene is recoverable when some move in a stated library keeps a positive margin G in every world the scene allows. V = max over moves of min over worlds of G; V_hind = min over worlds of max over moves; info_gap = V_hind - V.

**Best match: inevitable collision states, with Hamilton-Jacobi reachability as the control-theoretic version.**

- ICS (as summarised in the sources I opened and in search results): a state is an ICS if, whatever trajectory the system follows, a collision eventually occurs. Fraichard's line of work adds that, with limited sensing, absolute safety cannot be guaranteed in general, and that practical variants use "braking ICS" (collision before coming to rest under braking). The latter two points I saw only in search-result summaries. Our U_LIBRARY label is the ICS idea restricted to a finite move set.
- HJ reachability: the backward reachable set is the set of states from which the system can be forced into the failure set despite the best control, under bounded worst-case disturbance. The complement is the safe set. This is the same min-max structure as our V, with a continuous control and disturbance in place of a finite library and finite world set. Its known obstacle is exponential cost in state dimension (Bansal et al.), which is why it is not run on a 4-D scene with other vehicles' behaviours. It can be run on the corridor family (a 1-D or 2-D longitudinal model), where we already have a closed form.
- Control barrier functions enforce forward invariance of a safe set. They are a way to stay recoverable, not a way to label it. Relevant if the report discusses what a recoverability-aware controller would do.

**Where our definition differs.**

1. **Finite library.** ICS and reachability quantify over all controls. We quantify over a stated library, so a U_LIBRARY label is a statement about the library. This is why RC20 (library completeness) exists. It is a real difference, not a footnote.
2. **A margin, not a binary.** G is a normalised clearance minimum with a 1 m reserve and a terminal test, so "recoverable" means "recoverable with margin", and our G-sensitivity table shows 5-10% of labels move with reserve and eps. ICS and HJ use a failure set with no reserve (or a reserve folded into the set). Our labels are closer to "recoverable with a robustness margin" than to "can avoid collision".
3. **Worlds are a declared finite set.** Other vehicles' behaviours and observation timing are enumerated as worlds. HJ treats these as a continuous disturbance set; ICS for dynamic obstacles needs their future motion (Petti and Fraichard note ICS accounts for moving obstacles). Our min over worlds is only as good as the world list.
4. **Information structure.** The ego commits to one move and sees only delivered observations. U_INFORMATION (V_hind >= 0 > V) is about commitment under delayed evidence. ICS and reachability analyses typically assume state feedback, so this class is not an ICS notion. I believe the closest standard ideas are open-loop versus closed-loop robust control and non-anticipativity in stochastic programming (from memory, unverified). The report should present info_gap as our own measure and say where the analogy is loose.
5. **Executed labels, not computed ones.** Labels come from a stochastic simulator run with noise (0.028 within a session, 0.196 across sessions), not from a certificate. ICS and HJ results are certificates for their model; ours are samples.

**RSS** (Shalev-Shwartz et al.) is a white-box rule model of a safe distance and a proper response. It defines when an ego is not at fault and what a proper response is; it does not define unavoidable crashes. It is useful for the corridor family as a comparison (RSS minimum safe longitudinal distance versus our boundary) but is not the closest match. I did not read the paper's formulas, only its abstract.

### 1.3 Related notes on specific methods

- **STRIVE's solvability step.** STRIVE generates collisions for a planner and then runs a second optimisation to find a solution to each scenario, so that scenarios are "useful rather than simply impossible to survive" (opened abstract). This is the same concern as ours in the opposite direction: they want solvable failures, we want to label whether a solution exists. Our info-gap mining also keeps only scenes solvable in hindsight. A one-line citation of this is cheap.
- **KING** uses a differentiable kinematic-bicycle proxy for gradients and reports 20% higher success than black-box optimisation (opened). We do not have a differentiable proxy of the CARLA plant. Our `plant_kinematic.py` mock is a kinematic bicycle, so the idea is available, but the mock-versus-CARLA discrepancy at the braking end (CARLA brakes harder near standstill) would hurt a proxy.
- **Survey scope.** Ding et al. (opened) split scenario generation into data-driven, adversarial and knowledge-based and list fidelity, efficiency, diversity, transferability and controllability as open challenges. Song et al. 2025 (opened) reviews 91 generative-AI studies for ADS testing and lists 27 limitations. Both are good background citations for "what we do not do and why". LD-Scene (LLM-guided latent diffusion, 2025) was only seen in search results.
- **Relation of our benchmark to ours.** The corridor family has a closed-form stopping distance, so it works as an oracle for mining: agreement of 86-99% with the analytic sign (REPORT) is a useful sanity check that none of the cited tools offers by default.

## 2. Simulation methodology and validity

### 2.1 Determinism and reproducibility in CARLA

What the sources say (CARLA docs and Chance et al.; opened):

- CARLA docs state that physics and collision determinism are supported under specific conditions: synchronous mode with a fixed `fixed_delta_seconds`; synchronous mode enabled before loading the world; reload the world for each repetition; use `apply_batch_sync`; synchronous Traffic Manager with a fixed seed.
- The docs require `fixed_delta_seconds <= max_substep_delta_time * max_substeps` (0.1 s with defaults). For good physics the substep should be below 0.01666 s and ideally below 0.01 s. In their experiments velocity stayed consistent across fixed time steps only while the physical substep was 0.01 s.
- Chance et al. (2021 preprint, Feb 2022 revision) find that game engines are "seldom deterministic", that CARLA shows shortcomings in precision, and that one can find a configuration domain where run-to-run variance is negligible. They recommend characterising variance for your own setup. The search snippet also said CARLA and Unreal versions affect results; I did not read that in the abstract.

How this maps onto us. We use synchronous mode, fixed 0.05 s step, 0.01 s x 10 substeps, no Traffic Manager. We do **not** reload the world for each repetition (repeated `load_world` crashed the server; REPORT), which is one of the documented conditions for determinism. That likely contributes to the 0.028 within-session spread and may contribute to the 0.196 cross-session shift. I cannot say how much without testing it. A repeat with a world reload per episode, and a comparison, would tell. The two-server determinism check was not run (GPU memory).

### 2.2 Time step and vehicle dynamics for braking and evasive manoeuvres

- The CARLA docs tie physics accuracy to the substep size, not to the client tick, and our substep is 0.01 s, which meets the "ideally below 0.01" guidance only at the boundary. The 0.025 s check in REPORT shows the large effect is not substepping in general but a 23-33 m/s^2 "snap to rest" at the end of braking at 0.05 s. I did not find a source that documents this artefact, so treat the explanation as our finding, not a literature result.
- Vehicle fidelity. I searched for a validation of CARLA's emergency braking or lane-change dynamics against measured vehicle data and found none (search only). What I did find: a CARLA behavioural-fidelity study reports good temporal fidelity but significant absolute discrepancies in reactive scenarios (search result summary of "Mind the Gap", not opened), and a racing-focused paper reports a 42% reduction in dynamics sim-to-real gap after adding a digital-twin framework (search result summary, not opened). So: braking and handling in CARLA are known to need calibration, and we already did calibrate braking, but with non-physical tyre friction, which means evasive-manoeuvre limits are declared (8 m/s^2) and not emergent.
- Conclusion for the report: our manoeuvre-limit results are *model* results under declared limits. They do not claim that a real car could do the swerve.

### 2.3 Sim-to-real gap for safety evaluation

I did not open a paper that quantifies the sim-to-real gap for safety evaluation specifically. From the search results only: the gap is typically reported at the level of behaviour correlation (Pearson, cross-correlation) and absolute physical differences, and for decision timing versus force magnitudes the gap differs. We make no real-world claim; the report should say so once and cite one gap paper after reading it. Candidates seen in search: "Mind the Gap" (CARLA behaviour with drone data), Chalmers sim-to-real MPC study.

### 2.4 Repeats and statistical treatment for boundary scenes

I found no source I could open that gives a standard repeat count for boundary scenes. What the sources support:

- Chance et al.: measure the variance of your own setup and operate where it is small.
- Kalra and Paddock (RAND 2016, seen in search results only): proving AV safety statistically from driving miles needs hundreds of millions of miles, which is the motivation for scenario-based and simulation testing. It supports the point that a scene catalogue is the alternative, not that it replaces statistical evidence.
- Our own treatment (REPORT): a result is borderline if any repeat is within 0.0167 of the boundary (superseded by the 0.196 cross-session shift). The statistics below are standard methods, not from a source I opened (from memory, unverified): a binomial confidence interval (Wilson or Clopper-Pearson) on the pass fraction per scene over k repeats, labelling a scene "boundary" if the interval contains 0.5, and for GP-based search, treating label noise explicitly (noisy level-set estimation).

### 2.5 Scenario coverage and arguing completeness of a catalogue

- PEGASUS-style abstraction (functional, logical, concrete scenarios) is the common frame; the sources I saw (search only) say sampling a logical scenario gives many similar or redundant concrete scenarios and propose clustering, and that testing only critical scenarios is the premise for reducing effort though the remaining effort still exceeds practical capacity.
- No source I saw claims that a catalogue can be proven complete. The usual argument is relative: complete with respect to a stated operational design domain and a stated hazard analysis, plus coverage measures over parameter space. Our QD archives give a coverage measure over descriptors; our 20 cases are a *constructed* catalogue (one per mechanism in the PDF), so completeness here means "each mechanism has an example", which is a weaker claim and should be written that way.

### 2.6 Standards (only what I could confirm)

- **ISO 21448 (SOTIF), 2022.** Concerns hazards arising from functional insufficiencies and foreseeable misuse of an intended function, without a fault. It distinguishes four areas by whether scenarios are known or unknown and hazardous or not: the goal for known hazardous scenarios (Area 2) is to reduce their probability by design and evaluate specific scenarios; for unknown hazardous ones (Area 3) it is to reduce probability to an acceptable level through verification and validation. Clause 7 covers identifying triggering conditions and functional insufficiencies. Confirmed from search-result summaries (including NHTSA papers), not from the standard. One source mislabelled areas 3 and 4; I used the version agreed by the others. Relation to us: our mining is Area 3 style search for unknown hazardous scenes, in a toy ODD.
- **UL 4600.** A safety-case standard for evaluating autonomous products. It is claim-based: it lists topics a safety case must address rather than prescribing methods. Its drafters motivated it by ISO 26262 and ISO/PAS 21448 assuming a human driver. Edition status was not confirmed (sources differ on date). Relation to us: it supports stating a *claim and its evidence limits* explicitly, which is what the "label holds for the session that produced it" caveat does.
- **ISO 34502:2022.** A scenario-based safety evaluation framework for automated driving systems: set safety targets and a test range, identify critical scenarios from factors that can lead to dangerous events, derive and allocate test scenarios, execute and evaluate. Scope per the sources I saw: ADS per ISO/SAE PAS 22736, limited-access highways, excluding misuse, HMI and cybersecurity. Companion parts 34503 (ODD), 34504 (categorisation), 34505 (scenario evaluation and test case generation; was in DIS ballot at the time of the source). I did not open the standard. Relation to us: our straight two-lane, one-direction road is close to a limited-access highway segment.
- **PEGASUS.** German project that popularised the functional / logical / concrete scenario levels and the idea of testing critical scenarios. Confirmed only through papers that describe it (search only), not from project documents.

## 3. Implications for our work

Ranked by how much each affects the credibility of the report. Items 1-4 are caveats that should be stated; 5-9 are additions, cheapest first.

1. **State the finite-library and finite-world qualifier next to every "unrecoverable" claim, and name ICS / HJ reachability as the formal reference.** Our U_LIBRARY is not ICS. Say: "ICS quantifies over all controls; we quantify over a stated library and a stated world set, so we show unrecoverable-for-the-library." The report already says this for RC02 and RC09; put it in the abstract and in the mining section too. Cite Fraichard and Asama (via the opened Petti and Fraichard paper if you cannot read the original), Bansal et al. for the reachability definition.
2. **Put the noise next to the margins, and drop claims on margins under 0.2.** Only three headline margins (RC08, RC12, RC19) exceed the 0.196 cross-session shift. Mining counts near the boundary (|V| < 0.3) are therefore the same order as the noise. Honest statement: boundaries are located to within about the noise, matched pairs are "flipped within one session", and a fresh-session re-run of the 51 pairs would tell how many survive. This re-run is cheap in CARLA time compared with new mining and is the single best addition for credibility.
3. **Say plainly that braking boundaries are specific to the 0.05 s step and that the plant is not validated against a real vehicle.** Cite the CARLA docs on substepping, and Chance et al. on non-determinism. I found no published validation of CARLA's emergency braking or evasive dynamics; say that too, and say the 8 m/s^2 limits are declared. The 3 m optimism in RC01 at 0.025 s should appear wherever the corridor boundary is quoted.
4. **Do not call the scenes a representative sample, and do not report failure rates.** Parameter ranges are chosen, not measured, so there is no exposure distribution. The importance-sampling literature (Zhao et al.) gets probabilities only by modelling real driver behaviour. Our counts ("48.5 of 100 unrecoverable") are properties of the search ranges. State that algorithm comparisons are about efficiency at finding boundary scenes, not about how often such scenes occur.
5. **Cheap test: world reload per episode.** CARLA docs list "reload the world for each repetition" as a determinism condition, and we skip it because `load_world` crashed the server. Try re-spawning all actors at fixed transforms plus a `tick` flush, or reload on a small batch, and compare the repeat spread (0.028). If the spread shrinks, that changes the borderline rule; if not, you have evidence the noise is intrinsic to the engine.
6. **Cheap test: an analytic oracle comparison for HJ-style claims.** The corridor family has a closed form. Compute the exact viability kernel for a point-mass braking model with the same delay and reserve and plot it against mined boundaries. REPORT already gives 86-99% agreement and a 0.97 m safe-side offset; framing it as "agreement with the reachability set of a reduced model" ties the project to the strongest formal reference.
7. **Add a statistical treatment for repeated scenes.** For each matched pair, run k (say 5) repeats per side across at least two sessions, and report a binomial interval on the pass fraction. State the repeat count and the rule. This is a method from memory, not a cited one.
8. **Cite the algorithm provenance accurately.** straddle: Bryan et al. 2005 (not opened) with the randomised beta of Inatsu et al. (opened). qd_fail / qd_boundary: *adapted from* Fontaine and Nikolaidis (their MAP-Elites with failure objectives) and not a reproduction of any published driving system. bisect: ours, no paper. Say "adapted" in the table. `mining/README.md` lists the Mouret and Clune, VerifAI and CMA-ME references as "from memory"; the first two are now checked (Mouret and Clune in search results; VerifAI page in search results), CMA-ME still is not.
9. **Optional, if time: one comparison with an existing toolkit.** Run Scenic or VerifAI's falsifier on the corridor family as an external baseline, or at least export a few matched pairs as OpenSCENARIO. This would turn "adapted" into a measured comparison. I did not verify what these toolkits support with CARLA 0.9.16.

Honest weaknesses to acknowledge (not fixable in the project):

- Scenes are constructed; no real-data grounding. SafeShift-style mining of real logs would be the way to check relevance, and it is out of scope.
- The ego is a scripted move library on an engine whose braking has a known numerical artefact. A learned or planner-based ego would not be tested.
- "Unrecoverable" is at most as strong as the library and world list. A better library can move any U_LIBRARY label to R_REF (that is RC20 by design).
- The information-gap class has no direct counterpart in the formal literature I checked; its value as a concept is argued by construction, not by precedent.
- RC04 has no positive example on CARLA. Say so as a negative result, with the analysis of why.

## 4. References

### 4.1 Opened (page fetched and read)

Venue and year are as shown on the page I opened. The arXiv listing is the URL.

| Short name | Reference | URL |
| --- | --- | --- |
| Corso 2021 | Corso, Moss, Koren, Lee, Kochenderfer. A Survey of Algorithms for Black-Box Safety Validation of Cyber-Physical Systems. JAIR 72:377-428, 2021 (arXiv 2005.02979). | https://arxiv.org/abs/2005.02979 |
| Lee 2020 | Lee, Mengshoel, Saksena, Gardner, Genin, Silbermann, Owen, Kochenderfer. Adaptive Stress Testing: Finding Likely Failure Events with Reinforcement Learning. JAIR 69:1165-1201, 2020 (arXiv 1811.02188). | https://arxiv.org/abs/1811.02188 |
| Inatsu 2024 | Inatsu, Takeno, Kutsukake, Takeuchi. Active Learning for Level Set Estimation Using Randomized Straddle Algorithms. arXiv preprint 2408.03144, 2024. | https://arxiv.org/abs/2408.03144 |
| Batsch 2019 | Batsch, Daneshkhah, Cheah, Kanarachos, Baxendale. Performance Boundary Identification for the Evaluation of Automated Vehicles using Gaussian Process Classification. IEEE ITSC 2019 (arXiv 1907.05364). | https://arxiv.org/abs/1907.05364 |
| Fontaine 2021 | Fontaine, Nikolaidis. A Quality Diversity Approach to Automatically Generating Human-Robot Interaction Scenarios in Shared Autonomy. RSS 2021 (arXiv 2012.04283). | https://arxiv.org/abs/2012.04283 |
| Nikolaidis 2024 | Nikolaidis. Algorithmic Scenario Generation as Quality Diversity Optimization. arXiv preprint 2409.04711, 2024. | https://arxiv.org/abs/2409.04711 |
| CaDRE 2024 | Huang, Ding, Stoler, Francis, Chen, Zhao. CaDRE: Controllable and Diverse Generation of Safety-Critical Driving Scenarios using Real-World Trajectories. arXiv preprint 2403.13208, 2024 (v2 Dec 2024). | https://arxiv.org/abs/2403.13208 |
| KING 2022 | Hanselmann, Renz, Chitta, Bhattacharyya, Geiger. KING: Generating Safety-Critical Driving Scenarios for Robust Imitation via Kinematics Gradients. arXiv preprint 2204.13683, 2022 (ECCV 2022 per `mining/README.md`; the arXiv page lists no venue). | https://arxiv.org/abs/2204.13683 |
| STRIVE 2022 | Rempe, Philion, Guibas, Fidler, Litany. Generating Useful Accident-Prone Driving Scenarios via a Learned Traffic Prior. CVPR 2022 (arXiv 2112.05077). | https://arxiv.org/abs/2112.05077 |
| Ding 2022 | Ding, Xu, Arief, Lin, Li, Zhao. A Survey on Safety-Critical Driving Scenario Generation: A Methodological Perspective. IEEE T-ITS (arXiv 2202.02215, v6 Jun 2023). | https://arxiv.org/abs/2202.02215 |
| Song 2025 | Song, Ye, Harman, Sarro. Generative AI for Testing of Autonomous Driving Systems: A Survey. arXiv preprint 2508.19882, 2025. | https://arxiv.org/abs/2508.19882 |
| SafeShift 2023 | Stoler, Navarro, Jana, Hwang, Francis, Oh. SafeShift: Safety-Informed Distribution Shifts for Robust Trajectory Prediction in Autonomous Driving. arXiv preprint 2309.08889, 2023. | https://arxiv.org/abs/2309.08889 |
| Scenic 2020 | Fremont, Kim, Dreossi, Ghosh, Yue, Sangiovanni-Vincentelli, Seshia. Scenic: A Language for Scenario Specification and Data Generation. arXiv preprint 2010.06580, 2020. | https://arxiv.org/abs/2010.06580 |
| Zhao 2016 | Zhao, Lam, Peng, Bao, LeBlanc, Nobukawa, Pan. Accelerated Evaluation of Automated Vehicles Safety in Lane Change Scenarios Based on Importance Sampling Techniques. arXiv 1605.04965, 2016; IEEE T-ITS 18(3), 2017 (venue from the search result, DOI shown on the arXiv page). | https://arxiv.org/abs/1605.04965 |
| Bansal 2017 | Bansal, Chen, Herbert, Tomlin. Hamilton-Jacobi Reachability: A Brief Overview and Recent Advances. CDC 2017 (arXiv 1709.07523). | https://arxiv.org/abs/1709.07523 |
| Ames 2019 | Ames, Coogan, Egerstedt, Notomista, Sreenath, Tabuada. Control Barrier Functions: Theory and Applications. ECC 2019 (arXiv 1903.11199; the venue is from a search result). | https://arxiv.org/abs/1903.11199 |
| Petti 2005 | Petti, Fraichard. Partial Motion Planning Framework for Reactive Planning within Dynamic Environments. ICINCO 2005, vol. 4, pp. 199-204. (Used as the source for the ICS idea; the ICS definition is cited there to Fraichard and Asama.) | https://www.scitepress.org/papers/2005/11854 |
| RSS 2017 | Shalev-Shwartz, Shammah, Shashua. On a Formal Model of Safe and Scalable Self-driving Cars. arXiv preprint 1708.06374, 2017. | https://arxiv.org/abs/1708.06374 |
| Chance 2022 | Chance, Ghobrial, McAreavey, Lemaignan, Pipe, Eder. On Determinism of Game Engines used for Simulation-based Autonomous Vehicle Verification. arXiv preprint 2104.06262 (2021, rev. 2022; related DOI 10.1109/TITS.2022.3177887 suggests IEEE T-ITS). | https://arxiv.org/abs/2104.06262 |
| CARLA docs | CARLA documentation, Synchrony and time-step (latest). | https://carla.readthedocs.io/en/latest/adv_synchrony_timestep/ |

### 4.2 Seen only in search results (not opened; check before citing)

| What | URL |
| --- | --- |
| Fraichard, Asama. Inevitable collision states: a step towards safer robots? Advanced Robotics, 2004. Seen via Scispace listing and the Petti and Fraichard citation; the paper was not opened. | https://stagecf.scispace.com/topics/collision-avoidance-3ja2m5ts/2004 |
| Dreossi et al. VerifAI: A Toolkit for the Formal Design and Analysis of Artificial Intelligence-Based Systems. CAV 2019 (author list and venue read from the Berkeley publication page snippet). | https://people.eecs.berkeley.edu/~sseshia/pubs/b2hd-verifai-cav19.html |
| Mouret, Clune. Illuminating search spaces by mapping elites. arXiv preprint 1504.04909, 2015. | https://arxiv.org/abs/1504.04909 |
| Kalra, Paddock. Driving to Safety: How Many Miles of Driving Would It Take to Demonstrate Autonomous Vehicle Reliability? RAND RR-1478-RC, 2016. | https://www.rand.org/pubs/research_reports/RR1478.html (from memory, unverified; the pages I saw were library catalogue records) |
| LD-Scene: LLM-guided diffusion for controllable generation of adversarial safety-critical driving scenarios (arXiv 2505.11247, 2025). | https://arxiv.org/html/2505.11247v2 |
| CounterScene (alphaxiv overview 2603.21104). | https://www.alphaxiv.org/overview/2603.21104 |
| Simulating the Unseen: Crash Prediction Must Learn from What Did Not Happen (arXiv 2505.21743, 2025). | https://arxiv.org/abs/2505.21743 |
| TuRS-BSE: Trust Region Search-Based Bayesian Safety Evaluation Method for Accelerated Testing of Autonomous Vehicles. | https://trid.trb.org/View/2717752 |
| Mind the Gap: Quantifying Behavioral Fidelity in CARLA Using Naturalistic Drone Data. | https://trid.trb.org/View/2724751 |
| Sim-to-real transfer and reality gap modeling in model predictive control for autonomous driving (Chalmers). | https://research.chalmers.se/publication/532512 |
| ISO 21448:2022 (catalogue page and secondary sources). | https://iteh.es/catalog/standards/iso/51ac264d-8f3f-4beb-afd1-eb526ff1e98c/iso-21448-2022 |
| ISO 34502:2022 (ISO catalogue page) and METI press release. | https://www.iso.org/standard/78951.html |
| UL 4600 intent and key principles (UL page). | https://ul.org/UL4600 |
| Clustering-based Criticality Analysis for Testing of Automated Driving Systems (PEGASUS levels, redundancy of concrete scenarios; arXiv 2306.12738). | https://arxiv.org/pdf/2306.12738 |

### 4.3 From memory, unverified

- Bryan, Schneider, Nichol, Moore, Schneider. Active Learning for Identifying Function Threshold Boundaries. NIPS 2005 (cited in `mining/README.md`; not opened).
- Koren et al. Adaptive Stress Testing for Autonomous Vehicles (cited in `mining/README.md`; not opened).
- Fontaine et al. CMA-ME, GECCO 2020 (cited in `mining/README.md`; not opened).
- Binomial interval methods (Wilson, Clopper-Pearson) in 2.4 and the non-anticipativity analogy in 1.2 are general knowledge, not tied to a source.
- OpenSCENARIO and ASAM standards: named in the brief; I did not open anything on them.
