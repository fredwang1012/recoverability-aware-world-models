# Recoverability-aware world models

CSC2626 project. A driving scene is **recoverable** when some move in a stated library keeps a positive safety margin in every world the scene allows. This repo builds the 20 constructed recoverability cases on CARLA, labels them from executed evidence, and mines the recoverable/unrecoverable boundary inside each case family. It is Approach 1 of the project (Frederick Wang); the world model, its evaluation and the Autoware baseline belong to teammates.

**Start here:** [`docs/REPORT.md`](docs/REPORT.md), the full write-up: PDF to setup, results per case, decisions, problems, open issues. Live, editable version: [Claude Doc](https://claude.ai/artifact/FqWMCYVENDUFkjkbAVpykJ).

![Labels come only from executed CARLA evidence](docs/img/pipeline.png)

## Status (2026-10-20)

- All 20 cases (RC01–RC20) are built and labelled on CARLA 0.9.16; 19 show their intended mechanism, and RC04 has no positive example.
- Boundary mining (sobol, straddle, bisect, cem, bo_lcb, qd_fail, qd_boundary) ran on the corridor, RC03 and RC07 families: 51 matched pairs. Information-gap mining ran on RC10/RC11: 6 info-gap scenes.
- Dataset export (E1) validates on both the mock and CARLA data: the CARLA dataset has 2,318 labelled scenes (1,185 R_REF, 1,127 U_LIBRARY, 6 U_INFORMATION) from corridor, RC03, RC07 and info-gap mining, 0 invalid. G sensitivity is re-scored on 2,230 CARLA records.
- Braking-to-stop boundaries depend on the 0.05 s step (checked at 0.025 s); other margins do not.
- Open: braking at a finer step, the two-server determinism check, Cosmos appearance variants. See [the report](docs/REPORT.md#not-done-and-open-issues).

## Limitations (read before using the numbers)

- **Run-to-run noise.** CARLA is not deterministic: repeats in one server session move a margin by up to 0.028, and one re-run on a fresh server moved it by 0.196. Only RC08, RC12 and RC19 clear 0.196; every other passing headline margin (RC03 and RC20 at +0.008 the thinnest) could flip in a new session.
- **Braking depends on the 0.05 s step (documented limit, not fixed).** At 0.05 s the last part of braking ends in an artificial snap to rest. At 0.025 s it does not, and the RC01 scene overshoots by about 3 m. RC01, RC02, RC17 and the corridor boundary hold for the 0.05 s step; lane-change, library, observation and nuisance results do not depend on it. Re-calibrating at a finer step would take about a day of CARLA time. Details: [report](docs/REPORT.md#time-step-sensitivity).
- **"Unrecoverable" is relative** to the stated move library, worlds, horizon and threshold. RC02 and RC09 are not claims that no controller could recover.
- **RC04 has no positive example on CARLA**, and the hidden-car cases (RC10, RC11, RC13, RC14) hide the car from the ego's feed rather than physically.
- One server on one laptop: cross-server determinism was not tested, and the Cosmos appearance variants are not done.

## Videos

36 clips (about 10 s each) show every case's headline scene, with the working move and the failing move side by side where a case has both. They are re-runs on CARLA with a chase camera and are a visual aid only; labels never come from them. The generated road has no scenery, so the look is composited: blue ego, red parked cars, orange moving cars, grass and asphalt painted in afterwards. In the hidden-car cases (RC10, RC11, RC13, RC14) nothing physical hides the car; it is drawn as a faint ghost marked "not visible to ego" until its reveal time. Each run stops at first contact and the clip freezes on a CONTACT banner. The clips are git-ignored (23 MB); the index with the outcome check is [`mining/results/carla/videos/index.md`](mining/results/carla/videos/index.md), and `scripts/carla_videos.py` rebuilds them (see [the setup guide](docs/CARLA_SETUP.md#run)).

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
