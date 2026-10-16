"""
sites.py - site registry (PDF 7.1): find a qualified straight multi-lane stretch ONCE with the waypoint API, then reuse it.

STATUS: not run against a live server yet.

Site types used by the case catalogue:
    corridor / two_lane : straight section, >= 2 same-direction lanes, ego in the right-most lane with a left neighbour
                          (corridor = blocked across both lanes; two_lane = adjacent lane usable)
    occlusion           : junction / occluder site (later; RC10/11/14 use an observation gate first)
The road frame matches margin.py: x forward, lat positive to the RIGHT, anchored at the first waypoint of the stretch.
"""
import math
from dataclasses import dataclass

import numpy as np


def _wrap(deg):
    return (deg + 180.0) % 360.0 - 180.0


@dataclass
class Site:
    origin: object            # carla.Transform of the anchor waypoint
    road_yaw: float           # degrees
    lane_w: float
    n_lanes: int
    f: tuple
    r: tuple                  # right-hand unit vector (CARLA +y direction at yaw 0)
    length: float
    town: str

    def to_road(self, loc):
        dx, dy = loc.x - self.origin.location.x, loc.y - self.origin.location.y
        return dx * self.f[0] + dy * self.f[1], dx * self.r[0] + dy * self.r[1]

    def to_world(self, x, lat, cmap, z_off=0.5):
        import carla
        o = self.origin.location
        px, py = o.x + x * self.f[0] + lat * self.r[0], o.y + x * self.f[1] + lat * self.r[1]
        wp = cmap.get_waypoint(carla.Location(px, py, o.z), project_to_road=True)
        return carla.Location(px, py, wp.transform.location.z + z_off)


def find_straight_site(world, length=350.0, behind=0.0, step=20.0, max_yaw_dev=1.0, max_dz=0.8):
    """First straight stretch of `behind + length` m whose right-most driving lane has a same-direction left neighbour.
    The road frame is anchored `behind` m into the stretch, so there is room behind x = 0 for rear actors and warm-up."""
    import carla
    cmap = world.get_map()
    for wp in cmap.generate_waypoints(step):
        if wp.lane_type != carla.LaneType.Driving or wp.is_junction:
            continue
        left = wp.get_left_lane()
        if left is None or left.lane_type != carla.LaneType.Driving or left.lane_id * wp.lane_id <= 0:
            continue
        right = wp.get_right_lane()
        if right is not None and right.lane_type == carla.LaneType.Driving and right.lane_id * wp.lane_id > 0:
            continue                                                  # want ego in the right-most lane
        ok = True
        for s in np.arange(10.0, behind + length + 1, 15.0):
            nx = wp.next(float(s))
            if (not nx or nx[0].is_junction or nx[0].get_left_lane() is None
                    or abs(_wrap(nx[0].transform.rotation.yaw - wp.transform.rotation.yaw)) > max_yaw_dev
                    or abs(nx[0].transform.location.z - wp.transform.location.z) > max_dz):
                ok = False
                break
        if ok:
            if behind > 0:
                wp = wp.next(float(behind))[0]
            yaw = math.radians(wp.transform.rotation.yaw)
            n, w = 1, wp
            while True:                                               # count same-direction driving lanes to the left
                l = w.get_left_lane()
                if l is None or l.lane_type != carla.LaneType.Driving or l.lane_id * wp.lane_id <= 0:
                    break
                n, w = n + 1, l
            return Site(origin=wp.transform, road_yaw=wp.transform.rotation.yaw, lane_w=float(wp.lane_width), n_lanes=n,
                        f=(math.cos(yaw), math.sin(yaw)), r=(-math.sin(yaw), math.cos(yaw)), length=length,
                        town=cmap.name)
    raise RuntimeError("no straight multi-lane stretch found; try Town06")
