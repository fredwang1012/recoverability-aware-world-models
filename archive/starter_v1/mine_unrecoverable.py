"""
Approach 1 starter: simulation mining for unrecoverable scenes (headless, 2D).

Builds on the team's highwayenv_two_lane_starter.ipynb scene (ego in right lane,
stationary obstacle ahead, faster car approaching in the left lane) and turns it
into the proposal's labeling contract (Proposal 01, Sec. 4-5.1):

    r_j(h)      = min_{omega in Omega} G(pi_j, omega; h)     # per-policy reference margin
    R(h)        = [r_1(h), ..., r_N(h)]                      # recovery profile
    V(h)        = max_j r_j(h)
    unrecoverable w.r.t. (Pi, Omega, horizon, eps)  <=>  V(h) < eps

"Unrecoverable" here only ever means "no policy in THIS finite library succeeds
in ALL worlds in THIS finite Omega". It is not a physical-impossibility claim.

Run:
    pip install highway-env==1.12.1 pandas matplotlib
    python mine_unrecoverable.py --out results            # grid sweep + heatmap
    python mine_unrecoverable.py --out results --pnr      # also point-of-no-return scan
"""
import argparse
import copy
import itertools
import json
import os
from dataclasses import dataclass, asdict
from multiprocessing import Pool

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

import numpy as np
import pandas as pd
from highway_env.envs.highway_env import HighwayEnv
from highway_env.vehicle.kinematics import Vehicle
from highway_env.vehicle.behavior import IDMVehicle
from highway_env.vehicle.objects import Obstacle

LANE_W = 4.0          # HighwayEnv default lane width (m)
CAR_L, CAR_W = 5.0, 2.0
EPS = 0.0             # recoverability threshold on the normalized margin
HORIZON_S = 8
POLICY_HZ = 5


# ----------------------------------------------------------------------------
# 1. Scenario parameters theta  (this is the search space for mining)
# ----------------------------------------------------------------------------
@dataclass(frozen=True)
class Scene:
    ego_x: float = 100.0
    ego_speed: float = 20.0
    obstacle_gap: float = 75.0       # obstacle ahead in ego lane (centre-to-centre, m)
    adjacent_offset: float = -25.0   # left-lane car position relative to ego (m)
    adjacent_speed: float = 25.0


# ----------------------------------------------------------------------------
# 2. Omega: hidden actor mechanisms consistent with the same initial observation
# ----------------------------------------------------------------------------
WORLDS = ("constant", "idm_reactive", "accelerating")


class TwoLaneEnv(HighwayEnv):
    @classmethod
    def default_config(cls):
        cfg = super().default_config()
        cfg.update({
            "lanes_count": 2, "vehicles_count": 0, "controlled_vehicles": 1,
            "duration": HORIZON_S, "simulation_frequency": 15,
            "policy_frequency": POLICY_HZ, "offroad_terminal": True,
            "action": {"type": "DiscreteMetaAction", "target_speeds": [0, 10, 20, 30]},
            "observation": {
                "type": "Kinematics", "vehicles_count": 3,
                "features": ["presence", "x", "y", "vx", "vy"],
                "absolute": True, "normalize": False, "clip": False,
                "see_behind": True, "include_obstacles": True, "order": "sorted",
            },
            "scene": asdict(Scene()), "world": "constant",
        })
        return cfg

    def _create_vehicles(self):
        s = self.config["scene"]
        left_id, right_id = self.road.network.all_side_lanes(("0", "1", 0))[:2]
        left, right = self.road.network.get_lane(left_id), self.road.network.get_lane(right_id)

        ego = self.action_type.vehicle_class(
            self.road, right.position(s["ego_x"], 0),
            heading=right.heading_at(s["ego_x"]), speed=s["ego_speed"])
        self.controlled_vehicles = [ego]

        ax = s["ego_x"] + s["adjacent_offset"]
        world = self.config["world"]
        if world == "idm_reactive":
            adj = IDMVehicle(self.road, left.position(ax, 0), heading=left.heading_at(ax),
                             speed=s["adjacent_speed"], target_speed=s["adjacent_speed"],
                             enable_lane_change=False)
        else:
            adj = Vehicle(self.road, left.position(ax, 0), heading=left.heading_at(ax),
                          speed=s["adjacent_speed"])
            if world == "accelerating":
                adj.action = {"steering": 0.0, "acceleration": 1.5}
        self.road.vehicles = [ego, adj]
        self.road.objects = [Obstacle(self.road, right.position(s["ego_x"] + s["obstacle_gap"], 0))]

    def step(self, action):
        # keep the "accelerating" world's adjacent car accelerating every step
        if self.config["world"] == "accelerating":
            for v in self.road.vehicles[1:]:
                if type(v) is Vehicle:
                    v.action = {"steering": 0.0, "acceleration": 1.5}
        return super().step(action)


def make_env(scene: Scene, world: str):
    env = TwoLaneEnv(config={"scene": asdict(scene), "world": world})
    obs, _ = env.reset(seed=0)
    return env, obs


# ----------------------------------------------------------------------------
# 3. Pi: causal recovery policies. They see ONLY the observation (+ own phase).
#    Hidden world identity is never an input.
# ----------------------------------------------------------------------------
A = {"LANE_LEFT": 0, "IDLE": 1, "LANE_RIGHT": 2, "FASTER": 3, "SLOWER": 4}


def left_gap_ok(obs, min_front=12.0, min_rear=10.0, ttc_rear=2.5):
    """Observed-gap predicate for a left lane change (uses obs rows only)."""
    ego = obs[0]
    for row in obs[1:]:
        if row[0] < 0.5 or abs(row[2] - (ego[2] - LANE_W)) > LANE_W / 2:
            continue                      # empty row or not in the left lane
        dx, closing = row[1] - ego[1], row[3] - ego[3]
        if 0 <= dx < min_front:
            return False
        if dx < 0 and (-dx < min_rear or (closing > 0 and -dx / closing < ttc_rear)):
            return False
    return True


class Policy:
    name = "base"
    def reset(self): self.t = 0
    def act(self, obs):
        a = self._act(obs); self.t += 1; return a


class KeepSpeed(Policy):
    name = "keep"
    def _act(self, obs): return A["IDLE"]


class BrakeStay(Policy):
    name = "brake_stay"
    def _act(self, obs): return A["SLOWER"]


class LeftNow(Policy):
    name = "left_now"
    def _act(self, obs): return A["LANE_LEFT"] if self.t == 0 else A["IDLE"]


class LeftIfGapElseBrake(Policy):
    name = "left_if_gap_else_brake"
    def reset(self): super().reset(); self.changed = False
    def _act(self, obs):
        if not self.changed and left_gap_ok(obs):
            self.changed = True; return A["LANE_LEFT"]
        return A["IDLE"] if self.changed else A["SLOWER"]


class BrakeThenLeft(Policy):
    name = "brake_then_left_if_gap"
    def reset(self): super().reset(); self.changed = False
    def _act(self, obs):
        if self.t < 3: return A["SLOWER"]
        if not self.changed and left_gap_ok(obs):
            self.changed = True; return A["LANE_LEFT"]
        return A["IDLE"] if self.changed else A["SLOWER"]


class AccelThenLeft(Policy):
    name = "accel_then_left_if_gap"
    def reset(self): super().reset(); self.changed = False
    def _act(self, obs):
        if self.changed: return A["IDLE"]
        if left_gap_ok(obs): self.changed = True; return A["LANE_LEFT"]
        return A["FASTER"] if self.t < 4 else A["SLOWER"]


LIBRARY = [KeepSpeed, BrakeStay, LeftNow, LeftIfGapElseBrake, BrakeThenLeft, AccelThenLeft]


# ----------------------------------------------------------------------------
# 4. G: normalized margin from PRIVILEGED sim state (labels only, never inputs)
#    G = min{ collision clearance, road margin, terminal-continuation margin }
# ----------------------------------------------------------------------------
def box_clearance(ego, other_pos):
    dx = abs(other_pos[0] - ego.position[0]) - CAR_L
    dy = abs(other_pos[1] - ego.position[1]) - CAR_W
    return max(dx, dy)                     # >0 means boxes are separated


def terminal_margin(env, headway_s=1.5):
    """Post-maneuver continuation: in a lane and not closing on anything in it."""
    ego = env.unwrapped.vehicle
    m = 10.0
    others = [v for v in env.unwrapped.road.vehicles if v is not ego] + list(env.unwrapped.road.objects)
    for o in others:
        if abs(o.position[1] - ego.position[1]) > LANE_W / 2:
            continue
        dx = o.position[0] - ego.position[0]
        ov = getattr(o, "speed", 0.0)
        if dx > 0:   gap, closing = dx - CAR_L, ego.speed - ov    # leader
        else:        gap, closing = -dx - CAR_L, ov - ego.speed   # follower
        m = min(m, gap - max(closing, 0.0) * headway_s)
    return m


def rollout_margin(env, obs, policy, scale=5.0):
    """Run one causal policy to the horizon; return normalized margin and trace."""
    policy.reset()
    u = env.unwrapped
    min_clear, crashed, offroad = 1e9, False, False
    n_steps = int(HORIZON_S * POLICY_HZ)
    for _ in range(n_steps):
        obs, _, term, trunc, _ = env.step(policy.act(obs))
        ego = u.vehicle
        for o in [v for v in u.road.vehicles if v is not ego] + list(u.road.objects):
            min_clear = min(min_clear, box_clearance(ego, o.position))
        crashed |= bool(ego.crashed); offroad |= not ego.on_road
        if term or trunc:
            break
    if crashed or offroad:
        return -1.0, {"crashed": crashed, "offroad": offroad}
    g = min(min_clear / scale, terminal_margin(env) / scale)
    return float(np.clip(g, -1.0, 1.0)), {"crashed": False, "offroad": False}


# ----------------------------------------------------------------------------
# 5. Oracle labeling: recovery profile R(h) for one scene (h = initial obs here)
# ----------------------------------------------------------------------------
def recovery_profile(scene: Scene):
    row = asdict(scene)
    per_policy = {}
    for P in LIBRARY:
        worst = 1.0
        for w in WORLDS:
            env, obs = make_env(scene, w)
            g, _ = rollout_margin(env, obs, P())
            env.close()
            worst = min(worst, g)                       # min over Omega
        per_policy[P.name] = worst
    row.update({f"r_{k}": v for k, v in per_policy.items()})
    row["V"] = max(per_policy.values())                 # best policy in library
    row["best_policy"] = max(per_policy, key=per_policy.get)
    row["unrecoverable"] = row["V"] < EPS
    row["evidence"] = "verified_success" if row["V"] >= EPS else "library_exhausted"
    return row


# ----------------------------------------------------------------------------
# 6. Point of no return along a nominal (keep-speed) rollout:
#    clone the state at each step and re-label from there.
# ----------------------------------------------------------------------------
def point_of_no_return(scene: Scene):
    rows = []
    n_steps = int(HORIZON_S * POLICY_HZ)
    for k in range(n_steps):
        best = -1.0
        for P in LIBRARY:
            worst = 1.0
            for w in WORLDS:
                env, obs = make_env(scene, w)
                crashed = False
                for _ in range(k):                      # nominal prefix: keep speed
                    obs, _, term, trunc, _ = env.step(A["IDLE"])
                    if term or trunc: crashed = True; break
                if crashed:
                    worst = -1.0; env.close(); break
                branch = copy.deepcopy(env.unwrapped)   # snapshot, then branch
                g, _ = rollout_margin(branch, obs, P())
                env.close()
                worst = min(worst, g)
            best = max(best, worst)
        rows.append({"t_s": k / POLICY_HZ, "V": best, "recoverable": best >= EPS})
        if best < EPS:
            break
    return pd.DataFrame(rows)


# ----------------------------------------------------------------------------
# 7. Matched pairs: neighbouring grid cells that differ in ONE factor and flip
#    status (proposal Sec. 5.1 "paired histories that vary one factor").
# ----------------------------------------------------------------------------
def matched_pairs(df, axes):
    pairs = []
    for ax in axes:
        others = [a for a in axes if a != ax]
        for _, grp in df.groupby(others):
            grp = grp.sort_values(ax)
            for (_, a), (_, b) in zip(grp.iterrows(), grp.iloc[1:].iterrows()):
                if a["unrecoverable"] != b["unrecoverable"]:
                    pairs.append({"varied": ax, "a": a[axes].to_dict(), "b": b[axes].to_dict(),
                                  "V_a": a["V"], "V_b": b["V"]})
    return pairs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="results")
    ap.add_argument("--workers", type=int, default=os.cpu_count())
    ap.add_argument("--pnr", action="store_true")
    ap.add_argument("--coarse", action="store_true", help="tiny grid for a smoke test")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    gaps = np.arange(20, 95, 15 if args.coarse else 5)
    offsets = np.arange(-40, 15, 15 if args.coarse else 5)
    scenes = [Scene(obstacle_gap=float(g), adjacent_offset=float(o))
              for g, o in itertools.product(gaps, offsets)]

    with Pool(args.workers) as pool:
        df = pd.DataFrame(pool.map(recovery_profile, scenes))
    df.to_csv(f"{args.out}/recovery_profiles.csv", index=False)

    axes = ["obstacle_gap", "adjacent_offset"]
    pairs = matched_pairs(df, axes)
    with open(f"{args.out}/matched_pairs.json", "w") as f:
        json.dump(pairs, f, indent=2, default=float)

    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    piv = df.pivot(index="adjacent_offset", columns="obstacle_gap", values="V")
    fig, ax = plt.subplots(figsize=(9, 5))
    im = ax.imshow(piv.values, origin="lower", cmap="RdYlGn", vmin=-1, vmax=1, aspect="auto",
                   extent=[gaps.min(), gaps.max(), offsets.min(), offsets.max()])
    ax.contour(piv.columns, piv.index, piv.values, levels=[EPS], colors="k")
    ax.set(xlabel="obstacle gap (m)", ylabel="adjacent car offset vs ego (m)",
           title="V(h) = max_j min_ω r_j   (black line: recoverability boundary)")
    fig.colorbar(im, label="best normalized margin")
    fig.tight_layout(); fig.savefig(f"{args.out}/recoverability_map.png", dpi=150)

    print(df[axes + ["V", "best_policy", "unrecoverable"]].to_string(index=False))
    print(f"\n{df['unrecoverable'].sum()} / {len(df)} scenes unrecoverable w.r.t. library; "
          f"{len(pairs)} matched pairs on the boundary")

    if args.pnr:
        pnr = point_of_no_return(Scene())
        pnr.to_csv(f"{args.out}/point_of_no_return.csv", index=False)
        print("\nPoint-of-no-return scan (default scene, nominal = keep speed):")
        print(pnr.to_string(index=False))


if __name__ == "__main__":
    main()
