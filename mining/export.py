"""T7: export mined scenes to the shared JSONL schema (Approach 1 <-> Kent / Andy / Victor).

Every exported scene is re-labelled with the FULL recovery profile R(h) (no pruning), so no entry is 'unresolved'
unless a backend failure leaves it so.  'Unrecoverable' is relative to the library, worlds, horizon and eps recorded in
each record - never a physical-impossibility claim.  Cosmos appearance variants must set parent_id and
variant_kind='appearance' and inherit theta and R unchanged.

Usage:  python export.py --run ../results/mock_run --algos sobol,straddle,bisect --out ../results/dataset

E1 export of physical-case ExecutionRecords (schema "E1.1": corridor / rc03 / infogap mining records, mock or CARLA, never mixed):
    python mining/export.py records --backend mock                      (repo root; writes results/mock/dataset)
    cd mining && python export.py records --backend carla              (writes results/carla/dataset)
"""
import argparse
import hashlib
import json
import os
from dataclasses import asdict

import numpy as np
import pandas as pd

import core
import records as RC
import splits

SCHEMA_VERSION = 1
KEEP_CATS = ("unrecoverable", "edge", "planner_failure")


def scene_id(family, theta):
    key = json.dumps({k: round(float(v), 6) for k, v in sorted(theta.items())}) + family
    return "s_" + hashlib.sha1(key.encode()).hexdigest()[:12]


def evidence(profile):
    """verified_witness: r_j >= eps (a policy that succeeds in ALL worlds).  counterexample: r_j < eps (some world
    defeats the policy).  unresolved: r_j missing/NaN (never invented)."""
    out = {}
    for name, r in profile.items():
        if r is None or not np.isfinite(r):
            out[name] = "unresolved"
        else:
            out[name] = "verified_witness" if r >= core.EPS else "counterexample"
    return out


def trace_npz(backend, theta, row, path):
    """Ego/other traces of the best policy in every world (observation-side reconstruction aid)."""
    best = next(P for P in core.LIBRARY if P.name == row["best_policy"])
    arrays = {}
    for w in core.WORLDS:
        tr = backend.rollout(theta, w, best())
        arrays[f"{w}__ego"] = np.array(tr["ego"], dtype=float)
        for n, st in tr["others"].items():
            arrays[f"{w}__{n}"] = np.array(st, dtype=float)
    np.savez_compressed(path, **arrays)


def make_record(backend, u, cfg, family, seed, pair=None, source="sim_mining"):
    theta = core.decode(np.asarray(u))
    row = core.oracle(backend, theta, full=True, cfg=cfg)
    profile = {P.name: row[f"r_{P.name}"] for P in core.LIBRARY}
    sid = scene_id(family, theta)
    rec = {
        "schema_version": SCHEMA_VERSION, "id": sid, "source": source, "backend": backend.name,
        "family": family, "theta": theta, "u": [float(x) for x in u], "seed": seed,
        "world_set": list(core.WORLDS), "policy_library": [P.name for P in core.LIBRARY],
        "horizon_s": core.HORIZON_S, "eps": core.EPS, "edge_band": core.EDGE_BAND,
        "R": profile, "V": row["V"], "best_policy": row["best_policy"], "planner_margin": row["planner_margin"],
        "category": row["category"], "evidence": evidence(profile),
        "evidence_class": row["evidence"], "V_hind": row["V_hind"], "info_gap": row["info_gap"],
        "R_matrix": row["R"], "worlds": row["worlds"],
        "t_reveal": theta.get("t_reveal"), "margin_config": asdict(cfg),
        "descriptors": {"m1": row["m1"], "m2": row["m2"]},
        "pair_id": pair["pair_id"] if pair else None, "pair_role": pair["role"] if pair else None,
        "varied_factor": pair["varied"] if pair else None,
        "parent_id": None, "variant_kind": None,
        "trace_path": f"traces/{sid}.npz",
        "note": "unrecoverable = relative to policy_library, world_set, horizon_s, eps; not physical impossibility",
    }
    return rec, theta


def _key(u):
    return json.dumps([round(float(x), 9) for x in u])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True, help="run.py output dir with samples_<algo>.csv (+ matched_pairs.json)")
    ap.add_argument("--algos", default="sobol,cem,straddle,bisect,qd_boundary")
    ap.add_argument("--out", default="dataset")
    ap.add_argument("--family", choices=list(core.FAMILIES), default="two_lane")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--all", action="store_true", help="also export safe scenes")
    a = ap.parse_args()
    core.use_family(a.family)
    os.makedirs(f"{a.out}/traces", exist_ok=True)
    be, cfg = core.MockBackend(), core.DEFAULT_CFG
    us = {}
    for al in a.algos.split(","):
        p = f"{a.run}/samples_{al}.csv"
        if os.path.exists(p):
            for r in pd.read_csv(p).to_dict("records"):
                if a.all or r["category"] in KEEP_CATS:
                    u = json.loads(r["u"])
                    us[_key(u)] = u
    pair_of = {}
    pp = f"{a.run}/matched_pairs.json"
    if os.path.exists(pp):
        for i, p in enumerate(json.load(open(pp))):
            pid = f"pair_{i:03d}"
            for role, key in (("unrecoverable", "u_unrecoverable"), ("recoverable", "u_recoverable")):
                if key in p:
                    us[_key(p[key])] = p[key]
                    pair_of[_key(p[key])] = {"pair_id": pid, "role": role, "varied": core.NAMES[p["varied_index"]]}
    # distinct u can decode to the same theta (reaction_delay / t_reveal are quantised): one record per scene id,
    # keeping pair metadata if any member of the collision carries it
    recs = {}
    for k, u in us.items():
        rec, theta = make_record(be, u, cfg, a.family, a.seed, pair_of.get(k))
        if rec["id"] not in recs or (rec["pair_id"] and not recs[rec["id"]][0]["pair_id"]):
            recs[rec["id"]] = (rec, theta)
    n = 0
    with open(f"{a.out}/scenes.jsonl", "w") as f:
        for rec, theta in recs.values():
            trace_npz(be, theta, rec, f"{a.out}/{rec['trace_path']}")
            f.write(json.dumps(rec) + "\n")
            n += 1
    print(f"exported {n} scenes ({len(pair_of)} pair members) to {a.out}/scenes.jsonl")


# ============================================================================= E1: physical-case ExecutionRecords
E1_VERSION = "E1.1"
E1_FIELDS = ["id", "case_id", "family_id", "split_group", "theta", "history", "policy_library_id", "policy_library_version", "world_set_id",
             "R", "V", "V_hind", "info_gap", "evidence", "margin_config", "physics_profile", "terminal", "pair_id", "pair_role",
             "varied_factor", "parent_id", "variant_kind"]
E1_EXTRA = ["schema_version", "backend", "evidence_scope", "seed", "policy_library", "world_set", "R_search", "R_mask", "best_policy",
            "margin_config_id", "sources", "trace_path", "trace_key", "code_version", "record_checksum", "invalid_reasons"]
HIST_KEYS = {"t_capture", "t_available", "v", "lane_w", "ego_hl", "objs"}
OBJ_KEYS = {"dx", "dlat", "dv", "hl", "hw"}
SOURCE_FAMILIES = ("corridor", "rc03", "infogap")


def _num(x):
    """record JSON number -> float (None = unrun -> NaN, '-inf' -> -inf)."""
    return np.nan if x is None else (-np.inf if x == "-inf" else float(x))


def r_array(R):
    return np.array([[_num(x) for x in row] for row in R], float)


def _round_frame(fr):
    return {"t_capture": round(fr["t_capture"], 4), "t_available": round(fr["t_available"], 4), "v": round(fr["v"], 4),
            "lane_w": round(fr["lane_w"], 4), "ego_hl": round(fr["ego_hl"], 4),
            "objs": [{k: round(o[k], 4) for k in sorted(OBJ_KEYS)} for o in fr["objs"]]}


def history_of(rec):
    """Delivered-observation history of the best policy in each world (world order = rec['worlds']), or None when the record was produced
    without keep_history (every record before Task 9 D5).  Frames hold only what the policy was handed: timestamps, ego speed, lane width,
    anonymous objects.  common_prefix_frames = leading frames identical in all worlds (the pre-reveal, world-indistinguishable history)."""
    bp = rec.get("best_policy")
    if bp is None:
        return None
    per = []
    for w in rec["worlds"]:
        h = (rec.get("traces") or {}).get(f"{bp}/{w}", {}).get("history")
        if h is None:
            return None
        per.append([_round_frame(f) for f in h])
    prefix = 0
    for fs in zip(*per):
        if all(f == fs[0] for f in fs):
            prefix += 1
        else:
            break
    return {"policy": bp, "per_world": per, "common_prefix_frames": prefix}


def world_set_id(rec):
    ws = (rec.get("geometry") or {}).get("world_spec") or []
    return f"{rec['family_id']}_ws_{RC.checksum({'worlds': rec['worlds'], 'spec': ws})[:8]}"


def library_definition(desc, theta):
    """The library DEFINITION: policy ids, versions, terminal modes and non-theta parameters.  A `@prefix` wrapper is a scene value (the prefix
    length comes from theta, and at prefix 0 the wrapper is absent), so the wrapped (inner) policy is the library member; parameters whose key is a
    theta factor are dropped too."""
    def core(d):
        d = d.get("inner") or d
        return {"name": d["name"], "version": d.get("version"), "terminal": d.get("terminal"),
                "params": {k: v for k, v in (d.get("params") or {}).items() if k not in theta}}
    return [core(d) for d in desc]


def row_id(sid, pair):
    """Plain scenes: the scene id.  Pair members: scene id + pair id + role, so a theta shared by two pairs (or by both roles of one pair)
    never collapses a pair into a half-pair."""
    return sid if pair is None else f"{sid}.{pair['pair_id']}.{pair['role']}"


def case_row(rec, pair=None, sources=(), library_id=None, group=None):
    """One E1 row from an ExecutionRecord JSON.  Non-applicable fields are null (pair_*, parent_id, variant_kind, history without
    keep_history, physics_profile on the mock plant by design: the kinematic stand-in has no CARLA physics profile), never an empty string / 0.
    Labels are copied from the record, which derived them with contract.classify; the validator recomputes them.
    library_id: the family's library id (pair members carry no record-level id, so export_records resolves it from the family's plain records);
    group: split group (splits.split_group of the scene, or of the unrecoverable member for pair rows)."""
    sid = scene_id(f"{rec['family_id']}/{rec['case_id']}", rec["theta"])
    rid = row_id(sid, pair)
    desc = rec["policy_descriptors"]
    row = {
        "schema_version": E1_VERSION, "id": rid, "case_id": rec["case_id"], "family_id": rec["family_id"],
        "split_group": group or splits.split_group(rec["family_id"], rec["theta"]),
        "backend": rec["backend"], "evidence_scope": rec["evidence_scope"], "seed": rec["seed"], "theta": rec["theta"],
        "history": history_of(rec),
        "policy_library_id": library_id or rec.get("library_id") or f"{rec['family_id']}_library",
        "policy_library_version": RC.checksum(library_definition(desc, rec["theta"]))[:8],
        "policy_library": list(rec["policies"]),
        "world_set_id": world_set_id(rec), "world_set": list(rec["worlds"]),
        "R": rec["R_raw"], "R_search": rec["R_norm"], "R_mask": [[0 if x is None else 1 for x in r_] for r_ in rec["R_raw"]],
        "V": rec["V"], "V_hind": rec["V_hind"], "info_gap": rec["info_gap"], "evidence": rec["evidence_class"], "best_policy": rec["best_policy"],
        "margin_config": rec["margin_config"], "margin_config_id": RC.checksum(rec["margin_config"])[:8],
        "physics_profile": rec.get("physics_profile_id"),
        "terminal": {"mode": {d["name"]: d.get("terminal") for d in desc}, "boolean": rec["terminal_ok"]},
        "pair_id": pair["pair_id"] if pair else None, "pair_role": pair["role"] if pair else None,
        "varied_factor": pair["varied"] if pair else None,
        "parent_id": None, "variant_kind": None,        # Cosmos appearance variants would set both; none exist yet
        "sources": list(sources), "code_version": rec["code_version"], "record_checksum": rec["checksum"],
        "invalid_reasons": rec.get("invalid_reasons", []),
        "trace_path": "traces.npz", "trace_key": rid,
    }
    return row


def history_arrays(h):
    """per-world arrays for the npz: t_capture, t_available, v, n_obj and padded objs [frames, max_obj, 5] (dx, dlat, dv, hl, hw)."""
    out = {}
    for wi, fs in enumerate(h["per_world"]):
        mo = max([len(f["objs"]) for f in fs] + [1])
        objs = np.full((len(fs), mo, 5), np.nan)
        for i, f in enumerate(fs):
            for j, o in enumerate(f["objs"]):
                objs[i, j] = [o["dx"], o["dlat"], o["dv"], o["hl"], o["hw"]]
        out[f"w{wi}__t_capture"] = np.array([f["t_capture"] for f in fs])
        out[f"w{wi}__t_available"] = np.array([f["t_available"] for f in fs])
        out[f"w{wi}__v"] = np.array([f["v"] for f in fs])
        out[f"w{wi}__n_obj"] = np.array([len(f["objs"]) for f in fs])
        out[f"w{wi}__objs"] = objs
    return out


def load_jsonl(path):
    return [json.loads(line) for line in open(path) if line.strip()]


def collect_records(root, families=SOURCE_FAMILIES):
    """(rec, pair-or-None, source-file) for every pair member and record under root/<family>/; pair members first so they win a theta collision."""
    out = []
    for fam in families:
        d = os.path.join(root, fam)
        if not os.path.isdir(d):
            continue
        for fn in sorted(os.listdir(d)):
            if fn.startswith("pairs_") and fn.endswith(".jsonl"):
                for p in load_jsonl(os.path.join(d, fn)):
                    for role, key in (("unrecoverable", "unrecoverable_side"), ("recoverable", "recoverable_side")):
                        out.append((p[key], {"pair_id": p["pair_id"], "role": role, "varied": p["varied"]}, fn))
        for fn in sorted(os.listdir(d)):
            if fn.startswith("records_") and fn.endswith(".jsonl"):
                out += [(r, None, fn) for r in load_jsonl(os.path.join(d, fn))]
    return out


def export_records(root, out_dir, backend, families=SOURCE_FAMILIES):
    """Write out_dir/scenes.jsonl (E1.1 rows) and out_dir/traces.npz.
    Plain scenes: duplicate scene ids (same family, case and theta from several algorithms) keep ONE row (the first source) and list every source
    file in `sources`; other measurements of the same theta are not merged (CARLA re-runs of one theta differ by the repeat spread).
    Pair members are NEVER dropped: they are keyed by (scene id, pair id, role), and a plain duplicate of a pair member's theta is the one that is
    dropped.  Both members take the split group of the unrecoverable member.  The policy_library_id of a family is read from its plain records
    and applied to its pair rows."""
    os.makedirs(out_dir, exist_ok=True)
    items = collect_records(root, families)
    lib_of = {}
    for rec, pair, _ in items:
        if pair is None and rec.get("library_id"):
            lib_of.setdefault(rec["family_id"], rec["library_id"])
    pair_group = {}
    for rec, pair, _ in items:
        if pair is not None and pair["role"] == "unrecoverable":
            pair_group[pair["pair_id"]] = splits.split_group(rec["family_id"], rec["theta"])
    rows, arrays, srcs, pair_sids = {}, {}, {}, {}
    for rec, pair, src in items:
        if rec["backend"] != backend:
            raise RuntimeError(f"record backend {rec['backend']!r} != requested {backend!r}; mock and CARLA are exported separately")
        sid = scene_id(f"{rec['family_id']}/{rec['case_id']}", rec["theta"])
        rid = row_id(sid, pair)
        srcs.setdefault(rid, []).append(src)
        if rid in rows:
            continue
        if pair is None and sid in pair_sids:
            for r_ in pair_sids[sid]:
                srcs[r_].append(src)                   # keep provenance: the plain duplicate's source file is listed on the pair rows
            continue                                   # plain duplicate of a pair member's theta: the pair row carries it
        if pair is not None:
            pair_sids.setdefault(sid, []).append(rid)
        row = case_row(rec, pair, library_id=lib_of.get(rec["family_id"]), group=pair_group.get(pair["pair_id"]) if pair else None)
        rows[rid] = row
        arrays[f"{rid}/R"] = r_array(rec["R_raw"])
        arrays[f"{rid}/R_search"] = r_array(rec["R_norm"])
        if row["history"] is not None:
            for k, v in history_arrays(row["history"]).items():
                arrays[f"{rid}/{k}"] = v
    with open(f"{out_dir}/scenes.jsonl", "w") as f:
        for rid, row in rows.items():
            row["sources"] = sorted(set(srcs[rid]))
            f.write(json.dumps(row) + "\n")
    np.savez_compressed(f"{out_dir}/traces.npz", **arrays)
    return len(rows), sum(r["history"] is not None for r in rows.values())


def main_records():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd")
    ap.add_argument("--backend", choices=["mock", "carla"], required=True)
    ap.add_argument("--root", default=None, help="mining results dir holding corridor/ rc03/ infogap/ (default results/<backend>/mining)")
    ap.add_argument("--out", default=None, help="default results/<backend>/dataset")
    a = ap.parse_args()
    root = a.root or f"results/{a.backend}/mining"
    out = a.out or f"results/{a.backend}/dataset"
    n, nh = export_records(root, out, a.backend)
    print(f"exported {n} E1.1 scenes ({nh} with delivered-observation history) from {root} to {out}/scenes.jsonl + traces.npz")


if __name__ == "__main__":
    import sys
    main_records() if len(sys.argv) > 1 and sys.argv[1] == "records" else main()
