# CARLA setup

How the CARLA runs in this repo are set up on the development laptop (Windows 11, RTX 3060 Laptop 6 GB). Everything here was used for the results in [`REPORT.md`](REPORT.md).

## Install

| Piece | Version / location |
| --- | --- |
| CARLA server | 0.9.16 (Unreal Engine 4 build), unpacked at `C:\CARLA_0.9.16\server` |
| Python | conda env `carla312` (Python 3.12). The `carla` wheel for 0.9.16 is cp312 only |
| Client packages | `pip install -r requirements.txt` inside that env (`carla==0.9.16`, numpy, scipy, matplotlib, pyyaml, pytest, imageio, imageio-ffmpeg) |
| Maps | Base install only: Town01–05 and Town10HD. Town06 (long highways) is in the separate AdditionalMaps package and is not used |

Always run the client with `PYTHONNOUSERSITE=1` so packages from the user site do not shadow the env:

```bash
PY="C:/Users/fredw/anaconda3/envs/carla312/python.exe"
export PYTHONNOUSERSITE=1
```

## Start the server

One server, off-screen, low quality (rendering is only needed for videos):

```powershell
Start-Process C:\CARLA_0.9.16\server\CarlaUE4.exe -ArgumentList "-RenderOffScreen -nosound -quality-level=Low -ResX=320 -ResY=240 -carla-rpc-port=2000"
```

Wait about 40 s, then check it answers:

```bash
cd mining
$PY -m sim.bringup selftest
```

Notes:

- **One server only.** Two do not fit in 6 GB of GPU memory, so the cross-server determinism check was not run.
- **The site is generated, not loaded.** The first client call in a server session builds the custom road `mining/sim/assets/straight_2lane.xodr` (`sim.bringup.connect(..., "custom_straight")`). Do not call `load_world` repeatedly: it crashed the server.
- **Windowed mode** (a live 3D view you can watch): replace `-RenderOffScreen` with `-windowed -ResX=960 -ResY=540`. It uses about 5.7 GB of GPU memory, close to the limit.
- **WSL is not recommended** for the server: GPU rendering there is unreliable. Native Windows works.

## Run

All CARLA scripts run from `mining/` so that outputs land in `mining/results/carla/`:

```bash
cd mining
$PY ../scripts/carla_foundation.py      # RC01 RC02 RC16 RC17
$PY ../scripts/carla_core.py            # RC03 RC07 RC12 RC19 RC20
$PY ../scripts/carla_compositional.py   # RC04 RC05 RC06 RC08 RC15 RC18
$PY ../scripts/carla_occlusion.py       # RC10 RC11 RC14
$PY ../scripts/carla_negatives.py       # RC09 RC13
$PY ../scripts/carla_videos.py          # 36 clips; --only RC03,RC05 for a subset, --force to redo
```

Every scene is checkpointed as one JSONL record in `mining/results/carla/<group>/`. A re-run skips finished scenes; move the `.jsonl` files aside to run fresh. The checkpoints are git-ignored (local only); the summary `.md` files are committed.

Mining and the dataset: see [`../scripts/README.md`](../scripts/README.md).

## When the server misbehaves

| Symptom | Cause | Fix |
| --- | --- | --- |
| `time-out of 120000ms while waiting for the simulator` | Server hung (often after a client was killed in synchronous mode) | Kill `CarlaUE4*` and start it again |
| `spawn failed ... collision at spawn position` | Actors left behind by a killed client | Tick once, destroy every `vehicle.*` and `sensor.*` actor, tick again (the actor list only refreshes after a tick) |
| Server freezes as soon as a camera is attached | In synchronous mode a camera image must be taken before any other request after a tick | Use `sim.video.RecordingPlant`, which drains the image right after every tick |
| GPU above 85 °C | Long runs on a laptop | The scripts pause automatically (`gpu_wait`) until it drops below 78 °C |
