"""
splits.py - E1 split groups (controller ruling, Task 9 fix round 1).  Counterfactual neighbours must never straddle a train/test split.

    corridor / rc03 (and any family with a bumper-gap axis): group = family + every NON-distance theta factor.  All gap (distance) neighbours of a
        scene share one group.
    infogap: group = family + the speed bin of width INFOGAP_V_BIN (all D and all t_reveal at that speed together).
A matched pair varies ONE factor; both members take the group of the UNRECOVERABLE member (so a pair that varies speed is not split).
"""
import math

DISTANCE_KEYS = {"corridor": {"gap_m"}, "rc03": {"gap_m"}}
INFOGAP_V_BIN = 1.0           # m/s, declared bin width for the continuous speed axis of the info-gap family


def split_group(family, theta):
    if family == "infogap":
        lo = math.floor(float(theta["v_mps"]) / INFOGAP_V_BIN) * INFOGAP_V_BIN
        return f"infogap:v[{lo:g},{lo + INFOGAP_V_BIN:g})"
    drop = DISTANCE_KEYS.get(family, {"gap_m"})
    items = sorted((k, round(float(v), 6)) for k, v in theta.items() if k not in drop)
    return family + ":" + ",".join(f"{k}={v:g}" for k, v in items)


def merge_groups(naturals, links):
    """Union-find over natural groups.  `links` = pairs (natural_a, natural_b), one per matched pair (its two members' natural groups).
    Returns {natural group: assigned group}; every natural group in a connected component gets the lexicographically smallest natural id of the
    component, so the assignment is deterministic and a pair (and every row of either member's natural group) always lands in one split group."""
    parent = {g: g for g in naturals}

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for a, b in links:
        parent.setdefault(a, a)
        parent.setdefault(b, b)
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[max(ra, rb)] = min(ra, rb)
    return {g: find(g) for g in parent}
