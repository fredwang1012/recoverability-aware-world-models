"""Shared types for the physical-scenario harness (backend independent).

Road frame (same as margin.py / carla_backend.py): x forward [m], lat positive to the RIGHT [m], yaw [rad] relative to the
road, positive = turning toward +lat (right).  Pose x is the box CENTRE.
"""
from dataclasses import dataclass, field
from typing import Optional

DT = 0.05                       # s, fixed simulation step (PDF 8.1)


@dataclass
class ActorState:
    x: float
    lat: float
    yaw: float
    v: float                    # speed along the heading [m/s]
    hl: float                   # half length [m]
    hw: float                   # half width  [m]


@dataclass
class Snapshot:
    k: int                      # tick index
    t: float                    # simulation time [s]
    actors: dict                # name -> ActorState  (ground truth; evaluator + trackers only, NEVER policies)
    ax: Optional[float] = None  # ego longitudinal acceleration measured this tick [m/s^2]
    events: list = field(default_factory=list)   # [{"type": "collision", "other": name, "k": k}]


@dataclass
class EgoCommand:
    """High-level command a policy may emit.  Execution (steer/throttle/brake) is done by trackers from ground truth,
    like an actuator-level controller; the DECISION only ever uses delivered observations."""
    long: str = "keep"                  # keep | brake | accel | hold
    brake_frac: float = 1.0             # fraction of full brake when long == brake
    lat_target: Optional[float] = None  # target lateral position in the road frame; None = keep current reference
    lc_time: float = 3.0                # s, duration of a lateral transition


@dataclass
class Obs:
    """What a policy is allowed to know at decision time (delivered observations only)."""
    t_capture: float
    t_available: float
    v: float
    lane_w: float
    objs: list                  # [{"dx","dlat","dv","hl","hw"}] anonymous, sorted by dx; no names/ids/modes
    ego_hl: float = 2.3         # the ego's own half length (own geometry, known to the ego)
