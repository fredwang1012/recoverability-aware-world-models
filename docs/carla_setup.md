# CARLA bring-up on this machine (status: NOT executed)

Checked on 2026-10-15:

| item | finding |
|---|---|
| GPU | RTX 3060 Laptop, 6 GB VRAM (enough for `-RenderOffScreen` with `no_rendering_mode`; no cameras are used by the search) |
| Docker | `docker.exe` is installed, but the Docker Desktop Linux engine is **not running** (`dockerDesktopLinuxEngine` pipe missing) |
| Image | `carlasim/carla:0.9.16` exists on Docker Hub (`docker manifest inspect` succeeded); not pulled |
| Local install | no CARLA folder found at `C:\CARLA*` or `D:\CARLA*` |
| Python client | `pip download carla==0.9.16` fails on Python 3.14 (only 0.9.5 is offered); use `py -3.12 -m venv .venv312` and `pip install carla==0.9.16` |

Untested options (pick one; none has been run):

1. Docker Desktop (WSL2 + NVIDIA container toolkit):
   `docker run --rm --gpus all -p 2000-2002:2000-2002 carlasim/carla:0.9.16 ./CarlaUE4.sh -RenderOffScreen -nosound -carla-rpc-port=2000`
2. Native Windows package from the CARLA 0.9.16 release page: `CarlaUE4.exe -RenderOffScreen -nosound -carla-rpc-port=2000`.

Parallel servers = one process per port (2000, 2002, ...). 6 GB VRAM may limit this to 1-2 servers; measure first.

First thing to run once a server answers (spec T2, in order; stop if one fails):

1. `python carla_backend.py --port 2000 --calibrate` -> `friction_calibration.json`; decel must be monotone in friction and cover 5-9 m/s^2.
2. `python carla_backend.py --selftest` -> keep holds speed (|dv| < 0.5 m/s over 8 s), emergency_brake decel within 10% of brake_limit, left_now ends at lateral about -lane_w +/- 0.3 m.
3. Determinism: same theta twice (and across two servers) must agree to < 0.05 m / 0.05 m/s. **Do not run any search if this fails.**
4. Timing profile, then the T4 runs.

The occluded-family `t_reveal` masking in `carla_backend.py` was added without a server and is untested.
