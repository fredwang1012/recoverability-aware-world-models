# docs

| File | What it is |
| --- | --- |
| [`REPORT.md`](REPORT.md) | The write-up: PDF to code, CARLA setup, results by case, deviations from the PDF, mining, G sensitivity, decisions, problems, videos, open issues. Snapshot of the [live Claude Doc](https://claude.ai/artifact/FqWMCYVENDUFkjkbAVpykJ) |
| [`CARLA_SETUP.md`](CARLA_SETUP.md) | Install, start the server, run each case group, fix common server problems |
| [`carla_results.md`](carla_results.md) | Generated acceptance report (spec section 7): PASS / WARN / NOT EXECUTED per item, every number computed from the result files. Do not edit by hand; rebuild with `python scripts/build_carla_report.py` |
| [`carla_results.template.md`](carla_results.template.md) | Template the report is built from |
| [`carla_results_d5_reading.md`](carla_results_d5_reading.md) | Hand-written reading of the info-gap (D5) results, pasted into the report |
| [`RESEARCH_METHODS.md`](RESEARCH_METHODS.md) | Related work: methods for mining unrecoverable and safety-critical scenes, simulation validity, how our definition and algorithms compare, ranked recommendations |
| [`videos/`](videos/README.md) | The 36 case videos (compressed) with an index of each clip's outcome |
| `img/` | Figures used by the docs |

## spec/ (inputs)

| File | What it is |
| --- | --- |
| [`spec/Recoverability_Definitions_and_CARLA_Cases.pdf`](spec/Recoverability_Definitions_and_CARLA_Cases.pdf) | The source PDF: definitions and the 20 constructed cases |
| [`spec/CLAUDE_CODE_PROMPT.md`](spec/CLAUDE_CODE_PROMPT.md) | The implementation spec (v2, phases A–E), the binding authority for the code |
| [`spec/pdf_verification_and_additions.md`](spec/pdf_verification_and_additions.md) | Check of the PDF's maths and claims, and additions from research |
| [`spec/approach1_research_findings.md`](spec/approach1_research_findings.md) | Literature notes behind the mining algorithms |

## process/ (how the work was run)

- [`process/plan_carla_physical_scenarios.md`](process/plan_carla_physical_scenarios.md): the 9-task implementation plan.
- [`process/carla_best_practices_research.md`](process/carla_best_practices_research.md): our setup checked against the CARLA docs and the determinism literature.
