# Recoverability-aware world models

CSC2626 project. A driving scene is **recoverable** when some move in a stated library keeps a positive safety margin in every world the scene allows. This repo builds the 20 constructed recoverability cases on CARLA, labels them from executed evidence, and mines the recoverable/unrecoverable boundary inside each case family. It is Approach 1 of the project (Frederick Wang); the world model, its evaluation and the Autoware baseline belong to teammates.

**Start here:** [`docs/REPORT.md`](docs/REPORT.md), the full write-up: PDF to setup, results per case, decisions, problems, open issues. Live, editable version: [Claude Doc](https://claude.ai/artifact/FqWMCYVENDUFkjkbAVpykJ).

![Labels come only from executed CARLA evidence](docs/img/pipeline.png)

## Status (2026-10-20)

- All 20 cases (RC01–RC20) are built and labelled on CARLA 0.9.16; 19 show their intended mechanism, and RC04 has no positive example.
- Boundary mining (sobol, straddle, bisect, cem, bo_lcb, qd_fail, qd_boundary) ran on the corridor, RC03 and RC07 families: 51 matched pairs. Information-gap mining ran on RC10/RC11: 6 info-gap scenes.
- Dataset export (E1) validates on both the mock and CARLA data, and G sensitivity is re-scored on 2,230 CARLA records. 36 case videos are recorded.
- Braking-to-stop boundaries depend on the 0.05 s step (checked at 0.025 s); other margins do not.
- Open: braking at a finer step, the two-server determinism check, Cosmos appearance variants. See [the report](docs/REPORT.md#not-done-and-open-issues).

## Layout

| Folder | What is in it |
| --- | --- |
| [`docs/`](docs/README.md) | Report, CARLA setup guide, generated results report, the spec and the PDF |
| [`mining/`](mining/README.md) | The Python package: recoverability contract, margin, harness, CARLA plant, case builders, mining algorithms, export |
| [`mining/results/carla/`](mining/results/carla/README.md) | CARLA result summaries (committed) and raw records (local only) |
| [`scripts/`](scripts/README.md) | Entry points: one script per case group, mining, sensitivity, videos, report builder |
| `tests/` | `pytest` suite (mock plant and pure helpers; no CARLA server needed) |
| `results/mock/` | Mock-plant outputs kept for comparison |
| [`archive/`](archive/README.md) | Early starter code, superseded by `mining/` |

## Quickstart

```bash
conda create -n carla312 python=3.12
conda activate carla312
pip install -r requirements.txt
export PYTHONNOUSERSITE=1

python -m pytest -q tests          # about 6 min, no server needed
```

To run on CARLA (server install, start command, one script per case group, troubleshooting): [`docs/CARLA_SETUP.md`](docs/CARLA_SETUP.md).

Raw CARLA records (`*.jsonl`), the exported datasets and the videos are git-ignored because of their size; the committed `.md` summaries were generated from them.
