"""
Approach 1: headless simulation mining of edge and unrecoverable driving scenes.

Three search algorithms share ONE oracle and ONE simulation budget:
  random    - uniform sampling over theta (the baseline every paper reports)
  cem       - cross-entropy falsification: shifts a Gaussian toward low V(theta)
              (finds unrecoverable scenes fast, tends to collapse to one mode)
  boundary  - one-factor bisection: from an unrecoverable scene, move ONE factor
              until it becomes recoverable, then bisect. Yields edge cases on the
              recoverability boundary AND matched pairs differing in one factor
              (proposal Sec. 5.1).

Oracle (proposal Sec. 4):
  r_j(theta) = min over worlds omega of normalized margin G(pi_j, omega)
  V(theta)   = max_j r_j(theta)
  category   = "unrecoverable"            if V < EPS
               "planner_failure"          if V >= EPS but the planner-under-test fails
               "edge"                     if EPS <= V < EPS + EDGE_BAND
               "safe"                     otherwise
"Unrecoverable" is always relative to THIS policy library, worlds and horizon.

Run (CPU only):
  pip install highway-env==1.12.1 pandas matplotlib
  python mine_search.py --budget 80 --out search_results            # all three
  python mine_search.py --algos cem --budget 80                       # or one at a time
  python mine_search.py --report --out search_results                 # compare + relabel + plot
"""
import argparse
import copy
import json
import os
from dataclasses import dataclass
from multiprocessing import Pool

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

import numpy as np
import pandas as pd
from highway_env.envs.highway_env import HighwayEnv
from highway_env.vehicle.behavior import IDMVehicle
from highway_env.vehicle.controller import MDPVehicle
from highway_env.vehicle.kinematics import Vehicle
from highway_env.vehicle.objects import Obstacle

LANE_W, CAR_L, CAR_W = 4.0, 5.0, 2.0
HORIZON_S, POLICY_HZ = 8, 5
EPS, EDGE_BAND, MARGIN_SCALE = 0.0, 0.2, 5.0
A_MAX_ACC = 3.0  # m/s^2, ego throttle limit

# ----------------------------------------------------------------------------
# Search space theta. Search happens in [0,1]^d; decode() maps to physical units.
# ----------------------------------------------------------------------------
SPACE = {
    "obstacle_gap":    (15.0, 90.0),   # m, obstacle ahead in ego lane
    "adjacent_offset": (-45.0, 15.0),  # m, left-lane car relative to ego
    "adjacent_speed":  (15.0, 32.0),   # m/s
    "ego_speed":       (12.0, 30.0),   # m/s
    "reaction_delay":  (0.0, 2.4),     # s, ego keeps speed before recovery starts
    "brake_limit":     (4.0, 9.0),     # m/s^2, ~wet road .. dry road max
}
NAMES = list(SPACE)
LO = np.array([SPACE[k][0] for k in NAMES])
HI = np.array([SPACE[k][1] for k in NAMES])


def decode(u):
    th = dict(zip(NAMES, LO + np.clip(u, 0, 1) * (HI - LO)))
    th["reaction_delay"] = round(th["reaction_delay"] * POLICY_HZ) / POLICY_HZ
    return th


# ----------------------------------------------------------------------------
# Environment: team's two-lane scene + physical brake/throttle limits
# ----------------------------------------------------------------------------
class LimitedMDPVehicle(MDPVehicle):
    """HighwayEnv's controller has no accel limit (it brakes at ~1.5 g). Clip it."""
    brake_limit = 8.0

    def clip_actions(self):
        super().clip_actions()
        a = self.action["acceleration"]
        self.action["acceleration"] = float(np.clip(a, -self.brake_limit, A_MAX_ACC))


WORLDS = ("constant", "idm_reactive", "accelerating")
EMERGENCY = 5  # extra action id handled by the env: target speed 0 now


class TwoLaneEnv(HighwayEnv):
    @classmethod
    def default_config(cls):
        cfg = super().default_config()
        cfg.update({
            "lanes_count": 2, "vehicles_count": 0, "controlled_vehicles": 1,
            "duration": HORIZON_S, "simulation_frequency": 15,
            "policy_frequency": POLICY_HZ, "offroad_terminal": True,
            "action": {"type": "DiscreteMetaAction", "target_speeds": [0, 10, 20, 30]},
            "observation": {"type": "Kinematics", "vehicles_count": 3,
                            "features": ["presence", "x", "y", "vx", "vy"],
                            "absolute": True, "normalize": False, "clip": False,
                            "see_behind": True, "include_obstacles": True,
                            "order": "sorted"},
            "theta": decode(np.full(len(NAMES), 0.5)), "world": "constant",
        })
        return cfg

    def _create_vehicles(self):
        th, ex = self.config["theta"], 100.0
        left_id, right_id = self.road.network.all_side_lanes(("0", "1", 0))[:2]
        left, right = (self.road.network.get_lane(i) for i in (left_id, right_id))

        ego = self.action_type.vehicle_class(self.road, right.position(ex, 0),
                                             heading=right.heading_at(ex), speed=th["ego_speed"])
        ego.__class__ = LimitedMDPVehicle
        ego.brake_limit = th["brake_limit"]
        self.controlled_vehicles = [ego]

        ax, w = ex + th["adjacent_offset"], self.config["world"]
        kw = dict(heading=left.heading_at(ax), speed=th["adjacent_speed"])
        if w == "idm_reactive":
            adj = IDMVehicle(self.road, left.position(ax, 0), target_speed=th["adjacent_speed"],
                             enable_lane_change=False, **kw)
        else:
            adj = Vehicle(self.road, left.position(ax, 0), **kw)
        self.road.vehicles = [ego, adj]
        self.road.objects = [Obstacle(self.road, right.position(ex + th["obstacle_gap"], 0))]

    def step(self, action):
        if self.config["world"] == "accelerating":
            for v in self.road.vehicles[1:]:
                if type(v) is Vehicle:
                    v.action = {"steering": 0.0, "acceleration": 1.5}
        if action == EMERGENCY:
            self.vehicle.speed_index, self.vehicle.target_speed = 0, 0.0
            action = A["IDLE"]
        return super().step(action)


def make_env(theta, world):
    env = TwoLaneEnv(config={"theta": theta, "world": world})
    obs, _ = env.reset(seed=0)
    return env, obs


# ----------------------------------------------------------------------------
# Policy library Pi (causal: observation + own phase only) + planner under test
# ----------------------------------------------------------------------------
A = {"LANE_LEFT": 0, "IDLE": 1, "LANE_RIGHT": 2, "FASTER": 3, "SLOWER": 4}


def left_gap_ok(obs, min_front=12.0, min_rear=10.0, ttc_rear=2.5):
    ego = obs[0]
    for row in obs[1:]:
        if row[0] < 0.5 or abs(row[2] - (ego[2] - LANE_W)) > LANE_W / 2:
            continue
        dx, closing = row[1] - ego[1], row[3] - ego[3]
        if 0 <= dx < min_front:
            return False
        if dx < 0 and (-dx < min_rear or (closing > 0 and -dx / closing < ttc_rear)):
            return False
    return True


def ttc_ahead(obs):
    ego, best = obs[0], np.inf
    for row in obs[1:]:
        if row[0] < 0.5 or abs(row[2] - ego[2]) > LANE_W / 2 or row[1] <= ego[1]:
            continue
        closing = ego[3] - row[3]
        if closing > 0:
            best = min(best, (row[1] - ego[1] - CAR_L) / closing)
    return best


class Policy:
    name = "base"
    def reset(self): self.t, self.changed = 0, False
    def act(self, obs):
        a = self._act(obs); self.t += 1; return a


class Keep(Policy):
    name = "keep"
    def _act(self, obs): return A["IDLE"]


class EmergencyBrake(Policy):
    name = "emergency_brake"
    def _act(self, obs): return EMERGENCY


class LeftNow(Policy):
    name = "left_now"
    def _act(self, obs): return A["LANE_LEFT"] if self.t == 0 else A["IDLE"]


class LeftIfGapElseBrake(Policy):
    name = "left_if_gap_else_brake"
    def _act(self, obs):
        if self.changed: return A["IDLE"]
        if left_gap_ok(obs): self.changed = True; return A["LANE_LEFT"]
        return EMERGENCY


class BrakeThenLeft(Policy):
    name = "brake_then_left_if_gap"
    def _act(self, obs):
        if self.changed: return A["IDLE"]
        if self.t >= 3 and left_gap_ok(obs): self.changed = True; return A["LANE_LEFT"]
        return EMERGENCY


class AccelThenLeft(Policy):
    name = "accel_then_left_if_gap"
    def _act(self, obs):
        if self.changed: return A["IDLE"]
        if left_gap_ok(obs): self.changed = True; return A["LANE_LEFT"]
        return A["FASTER"] if self.t < 4 else EMERGENCY


class TTCBrakePlanner(Policy):
    """Stand-in for the planner under test (later: Autoware). Brakes only, late."""
    name = "planner_under_test"
    def _act(self, obs): return EMERGENCY if ttc_ahead(obs) < 2.0 else A["IDLE"]


LIBRARY = [Keep, EmergencyBrake, LeftNow, LeftIfGapElseBrake, BrakeThenLeft, AccelThenLeft]


# ----------------------------------------------------------------------------
# Margin G from privileged state (labels only)
# ----------------------------------------------------------------------------
def others(u):
    return [v for v in u.road.vehicles if v is not u.vehicle] + list(u.road.objects)


def terminal_margin(u, headway_s=1.5):
    ego, m = u.vehicle, 10.0
    for o in others(u):
        if abs(o.position[1] - ego.position[1]) > LANE_W / 2:
            continue
        dx, ov = o.position[0] - ego.position[0], getattr(o, "speed", 0.0)
        gap, closing = (dx - CAR_L, ego.speed - ov) if dx > 0 else (-dx - CAR_L, ov - ego.speed)
        m = min(m, gap - max(closing, 0.0) * headway_s)
    return m


def rollout(theta, world, policy):
    env, obs = make_env(theta, world)
    u, delay = env.unwrapped, int(round(theta["reaction_delay"] * POLICY_HZ))
    policy.reset()
    min_clear, failed = np.inf, False
    for k in range(HORIZON_S * POLICY_HZ):
        obs, _, term, trunc, _ = env.step(A["IDLE"] if k < delay else policy.act(obs))
        ego = u.vehicle
        for o in others(u):
            min_clear = min(min_clear, max(abs(o.position[0] - ego.position[0]) - CAR_L,
                                           abs(o.position[1] - ego.position[1]) - CAR_W))
        if ego.crashed or not ego.on_road:
            # continuous failure score: harder impacts are "more unrecoverable",
            # which gives CEM a slope to descend instead of a flat -1 plateau
            near = min(others(u), key=lambda o: np.linalg.norm(o.position - ego.position))
            rel = abs(ego.speed - getattr(near, "speed", 0.0)) if ego.crashed else ego.speed
            fail_score = -float(np.clip(0.1 + rel / 15.0, 0.1, 1.0))
            failed = True
            break
        if term or trunc:
            break
    g = fail_score if failed else float(np.clip(min(min_clear, terminal_margin(u)) / MARGIN_SCALE, -1, 1))
    env.close()
    return g


def oracle(theta, full=False):
    """V = max_j min_w r_j. With full=False, prune policies that can't beat the best."""
    best, profile = -np.inf, {}
    for P in LIBRARY:
        worst = np.inf
        for w in WORLDS:
            worst = min(worst, rollout(theta, w, P()))
            if not full and worst <= best:
                break                      # can't improve V; exact pruning
        profile[P.name] = worst
        best = max(best, worst)
    planner = min(rollout(theta, w, TTCBrakePlanner()) for w in WORLDS)
    if best < EPS:
        cat = "unrecoverable"
    elif planner < EPS:
        cat = "planner_failure"
    elif best < EPS + EDGE_BAND:
        cat = "edge"
    else:
        cat = "safe"
    row = {**theta, "V": best, "planner_margin": planner, "category": cat,
           "closest_policy": max(profile, key=profile.get)}
    if full:
        row.update({f"r_{k}": v for k, v in profile.items()})
    return row


def eval_batch(us, pool):
    rows = pool.map(oracle, [decode(u) for u in us])
    for r, u in zip(rows, us):
        r["u"] = np.asarray(u).tolist()
    return rows


# ----------------------------------------------------------------------------
# Algorithm 1: uniform random baseline
# ----------------------------------------------------------------------------
def search_random(budget, rng, pool):
    return eval_batch(rng.random((budget, len(NAMES))), pool)


# ----------------------------------------------------------------------------
# Algorithm 2: cross-entropy method on V (falsification / importance sampling)
# ----------------------------------------------------------------------------
def search_cem(budget, rng, pool, batch=16, elite_frac=0.25):
    mu, sd, rows = np.full(len(NAMES), 0.5), np.full(len(NAMES), 0.3), []
    while len(rows) < budget:
        n = min(batch, budget - len(rows))
        us = np.clip(mu + sd * rng.standard_normal((n, len(NAMES))), 0, 1)
        new = eval_batch(us, pool)
        rows += new
        hist = sorted(rows, key=lambda r: r["V"])[: max(2, int(elite_frac * len(rows)))]
        elite = np.array([r["u"] for r in hist])
        mu = 0.7 * elite.mean(0) + 0.3 * mu
        sd = np.maximum(0.7 * elite.std(0) + 0.3 * sd, 0.05)   # keep some exploration
    return rows


# ----------------------------------------------------------------------------
# Algorithm 3: one-factor boundary bisection (edge cases + matched pairs)
# ----------------------------------------------------------------------------
def bisect_chain(args):
    u_bad, factor, iters = args
    u_bad = np.asarray(u_bad, float)
    rows = []
    # try both ends of this factor to find a recoverable anchor
    for end in (0.0, 1.0):
        u_good = u_bad.copy(); u_good[factor] = end
        r = oracle(decode(u_good)); r["u"] = u_good.tolist(); rows.append(r)
        if r["V"] >= EPS:
            break
    else:
        return rows, None                   # this factor alone can't rescue it
    lo, hi = u_bad.copy(), u_good.copy()    # lo: unrecoverable, hi: recoverable
    for _ in range(iters):
        mid = (lo + hi) / 2
        r = oracle(decode(mid)); r["u"] = mid.tolist(); rows.append(r)
        if r["V"] < EPS: lo = mid
        else: hi = mid
    pair = {"varied": NAMES[factor],
            "unrecoverable": decode(lo), "recoverable": decode(hi),
            "factor_gap": float(abs(decode(hi)[NAMES[factor]] - decode(lo)[NAMES[factor]]))}
    return rows, pair


def search_boundary(budget, rng, pool, seed_frac=0.35, iters=4):
    rows = eval_batch(rng.random((int(budget * seed_frac), len(NAMES))), pool)
    pairs = []
    while len(rows) < budget:
        bad = [r for r in rows if r["V"] < EPS]
        if not bad:                          # no failure yet: keep sampling
            rows += eval_batch(rng.random((4, len(NAMES))), pool); continue
        jobs = [(bad[rng.integers(len(bad))]["u"], int(rng.integers(len(NAMES))), iters)
                for _ in range(max(1, (budget - len(rows)) // (iters + 2)))]
        for chain_rows, pair in pool.map(bisect_chain, jobs):
            rows += chain_rows
            if pair: pairs.append(pair)
            if len(rows) >= budget: break
    return rows[:budget], pairs


# ----------------------------------------------------------------------------
# Metrics + driver
# ----------------------------------------------------------------------------
def diversity(rows):
    """Mean nearest-neighbour distance among unrecoverable finds (normalized space)."""
    pts = np.array([r["u"] for r in rows if r["category"] == "unrecoverable"])
    if len(pts) < 2: return 0.0
    d = np.linalg.norm(pts[:, None] - pts[None], axis=-1); np.fill_diagonal(d, np.inf)
    return float(d.min(1).mean())


def summarize(name, rows):
    df = pd.DataFrame(rows)
    c = df["category"].value_counts()
    return {"algorithm": name, "sims": len(df),
            "unrecoverable": int(c.get("unrecoverable", 0)),
            "planner_failure": int(c.get("planner_failure", 0)),
            "edge": int(c.get("edge", 0)),
            "near_boundary(|V|<0.2)": int((df["V"].abs() < EDGE_BAND).sum()),
            "diversity": round(diversity(rows), 3)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--algos", default="random,cem,boundary",
                    help="comma list; each saves <out>/<algo>_samples.csv")
    ap.add_argument("--budget", type=int, default=80, help="oracle calls per algorithm")
    ap.add_argument("--out", default="search_results")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--workers", type=int, default=os.cpu_count())
    ap.add_argument("--report", action="store_true",
                    help="skip search; summarize saved samples, relabel finds with full R(h), plot")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    algos = {"random": search_random, "cem": search_cem, "boundary": search_boundary}

    with Pool(args.workers) as pool:
        if not args.report:
            for i, name in enumerate(args.algos.split(",")):
                out = algos[name](args.budget, np.random.default_rng(args.seed + i), pool)
                rows, pairs = out if name == "boundary" else (out, None)
                pd.DataFrame(rows).to_csv(f"{args.out}/{name}_samples.csv", index=False)
                if pairs is not None:
                    with open(f"{args.out}/matched_pairs.json", "w") as f:
                        json.dump(pairs, f, indent=2, default=float)
                print(name, summarize(name, rows))
            return

        results = {}
        for name in algos:
            p = f"{args.out}/{name}_samples.csv"
            if os.path.exists(p):
                df = pd.read_csv(p)
                df["u"] = df["u"].apply(json.loads)
                results[name] = df.to_dict("records")
        keep = {json.dumps(r["u"]): r for rs in results.values() for r in rs
                if r["category"] in ("unrecoverable", "edge", "planner_failure")}
        dataset = pool.map(lambda_full, [decode(np.array(json.loads(k))) for k in keep])

    summary = pd.DataFrame([summarize(k, v) for k, v in results.items()])
    print(summary.to_string(index=False))
    summary.to_csv(f"{args.out}/algorithm_comparison.csv", index=False)
    pd.DataFrame(dataset).to_csv(f"{args.out}/mined_scenes_with_profiles.csv", index=False)
    pairs = json.load(open(f"{args.out}/matched_pairs.json")) if os.path.exists(f"{args.out}/matched_pairs.json") else []
    print(f"\n{len(dataset)} mined scenes re-labelled with full R(h); {len(pairs)} one-factor matched pairs")
    for p in pairs[:6]:
        print(f"  vary {p['varied']:>16}: {p['unrecoverable'][p['varied']]:.2f} (unrecoverable) -> "
              f"{p['recoverable'][p['varied']]:.2f} (recoverable)")
    plot(results, args.out)


def lambda_full(theta):
    return oracle(theta, full=True)


def plot(results, out):
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    colors = {"unrecoverable": "#c0392b", "planner_failure": "#e67e22", "edge": "#f1c40f", "safe": "#27ae60"}
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5), sharex=True, sharey=True)
    for ax, (name, rows) in zip(axes, results.items()):
        df = pd.DataFrame(rows)
        ttc = df["obstacle_gap"] / df["ego_speed"]
        for cat, col in colors.items():
            m = df["category"] == cat
            ax.scatter(ttc[m], df["reaction_delay"][m], c=col, s=22, label=cat, alpha=0.8)
        ax.set(title=name, xlabel="obstacle gap / ego speed  (s)")
    axes[0].set_ylabel("reaction delay (s)"); axes[0].legend(fontsize=8)
    fig.suptitle("Where each algorithm spent its simulation budget (projection of 6-D theta)")
    fig.tight_layout(); fig.savefig(f"{out}/algorithm_comparison.png", dpi=140)


if __name__ == "__main__":
    main()
