"""T7: validate an exported scenes.jsonl: schema, no NaNs, V == max_j R_j, category consistent with V and eps.
Rows with schema_version "E1.1" (export.py records) go through check_e1: shapes, null/mask consistency, labels recomputed with
contract.classify, terminal block, pair/variant field consistency, history = delivered-observation features only, npz agrees."""
import argparse
import json
import math
import os
import sys

import numpy as np

import contract
import splits

REQUIRED = ["id", "source", "backend", "theta", "u", "seed", "world_set", "policy_library", "R", "V", "category",
            "evidence", "t_reveal", "margin_config", "descriptors", "pair_id", "pair_role", "varied_factor",
            "trace_path", "eps", "edge_band", "planner_margin", "evidence_class", "V_hind", "info_gap", "R_matrix", "worlds"]
CATS = {"unrecoverable", "planner_failure", "edge", "safe"}
EVID = {"verified_witness", "counterexample", "unresolved"}


def finite(x):
    if isinstance(x, dict):
        return all(finite(v) for v in x.values())
    if isinstance(x, (list, tuple)):
        return all(finite(v) for v in x)
    if isinstance(x, float):
        return math.isfinite(x)
    return True


def check(rec, root):
    if rec.get("schema_version") == "E1.1" or "trace_key" in rec or "R_mask" in rec:
        return check_e1(rec, root)
    errs = [f"missing {k}" for k in REQUIRED if k not in rec]
    if errs:
        return errs
    if not finite(rec):
        errs.append("NaN/inf present")
    if rec["category"] not in CATS:
        errs.append(f"bad category {rec['category']}")
    if set(rec["R"]) != set(rec["policy_library"]) or set(rec["evidence"]) != set(rec["policy_library"]):
        errs.append("R/evidence keys differ from policy_library")
    if any(v not in EVID for v in rec["evidence"].values()):
        errs.append("bad evidence status")
    known = [v for k, v in rec["R"].items() if rec["evidence"].get(k) != "unresolved"]
    if known and abs(max(known) - rec["V"]) > 1e-9:
        errs.append(f"V={rec['V']} != max_j R_j={max(known)}")
    eps, band, V, pm = rec["eps"], rec["edge_band"], rec["V"], rec["planner_margin"]
    want = ("unrecoverable" if V < eps else "planner_failure" if pm < eps else "edge" if V < eps + band else "safe")
    if rec["category"] != want:
        errs.append(f"category {rec['category']} inconsistent (expected {want})")
    for k, st in rec["evidence"].items():
        if (st == "verified_witness" and not rec["R"][k] >= eps) or (st == "counterexample" and not rec["R"][k] < eps):
            errs.append(f"evidence {k}={st} contradicts R")
    # scene-level evidence class must equal contract.classify on the stored policy x world matrix (None = unresolved)
    Rm = np.array([[np.nan if x is None else x for x in rec["R_matrix"][p]] for p in rec["policy_library"]], float)
    c = contract.classify(Rm, eps)
    if c["evidence"] != rec["evidence_class"]:
        errs.append(f"evidence_class {rec['evidence_class']} != classify {c['evidence']}")
    if np.isfinite(c["V"]) and abs(c["V"] - V) > 1e-9:
        errs.append(f"V={V} != max_j min_w R = {c['V']}")
    if np.isfinite(c["V_hind"]) and (abs(c["V_hind"] - rec["V_hind"]) > 1e-9 or rec["V_hind"] < V - 1e-9):
        errs.append("V_hind inconsistent or below V")
    if rec["pair_id"] and (rec["pair_role"] not in ("unrecoverable", "recoverable") or not rec["varied_factor"]):
        errs.append("incomplete pair fields")
    if not os.path.exists(os.path.join(root, rec["trace_path"])):
        errs.append(f"missing trace {rec['trace_path']}")
    return errs


# ============================================================================= E1.1 rows (physical-case ExecutionRecords, export.py records)
E1_REQUIRED = ["schema_version", "id", "case_id", "family_id", "split_group", "backend", "evidence_scope", "seed", "theta", "history",
               "policy_library_id", "policy_library_version", "policy_library", "world_set_id", "world_set", "R", "R_search", "R_mask",
               "V", "V_hind", "info_gap", "evidence", "best_policy", "margin_config", "margin_config_id", "physics_profile", "terminal",
               "pair_id", "pair_role", "varied_factor", "parent_id", "variant_kind", "sources", "trace_path", "trace_key", "invalid_reasons"]
HIST_KEYS = {"t_capture", "t_available", "v", "lane_w", "ego_hl", "objs"}
OBJ_KEYS = {"dx", "dlat", "dv", "hl", "hw"}
TERMINAL_MODES = {"protected_hold", "lane_follow"}
_NPZ = {}


def _f(x):
    """record number -> float (None = unrun/unresolved -> NaN, '-inf' -> -inf)."""
    if x is None:
        return math.nan
    if x == "-inf":
        return -math.inf
    return float(x)


def _same(a, b, tol=1e-9):
    """equality that treats NaN == NaN and -inf == -inf."""
    if math.isnan(a) or math.isnan(b):
        return math.isnan(a) and math.isnan(b)
    if math.isinf(a) or math.isinf(b):
        return a == b
    return abs(a - b) <= tol


def _npz(path):
    if path not in _NPZ:
        _NPZ[path] = np.load(path) if os.path.exists(path) else None
    return _NPZ[path]


def check_e1(rec, root):
    errs = [f"missing {k}" for k in E1_REQUIRED if k not in rec]
    if errs:
        return errs
    P, W = len(rec["policy_library"]), len(rec["world_set"])
    for k in ("R", "R_search", "R_mask", "terminal"):
        m = rec["terminal"]["boolean"] if k == "terminal" else rec[k]
        if len(m) != P or any(len(r) != W for r in m):
            return [f"{k} shape != policies x worlds"]
    # numbers: no NaN/inf serialised as a number; unresolved = null, terminal failure = '-inf'
    for k in ("R", "R_search"):
        for row in rec[k]:
            for x in row:
                if isinstance(x, float) and not math.isfinite(x):
                    errs.append(f"{k}: non-finite float serialised as a number (use null / '-inf')")
                if k == "R_search" and x == "-inf":
                    errs.append("R_search must be finite or null")
    if not finite({k: rec[k] for k in ("theta", "margin_config", "seed")}):
        errs.append("NaN/inf in theta / margin_config")
    if rec["backend"] not in ("mock", "carla"):
        errs.append(f"bad backend {rec['backend']}")
    if rec["backend"] == "carla" and not (isinstance(rec["physics_profile"], str) and rec["physics_profile"].startswith("pp_")):
        errs.append(f"carla record without a physics profile id ({rec['physics_profile']!r})")
    if rec["backend"] == "mock" and rec["physics_profile"] is not None:
        errs.append("mock record must have physics_profile null")
    # R_mask <=> R is not null (an unrun entry is never 0 / failure)
    for j in range(P):
        for w in range(W):
            if (rec["R"][j][w] is None) != (rec["R_mask"][j][w] == 0):
                errs.append(f"R_mask[{j}][{w}] disagrees with R (unrun entries must be null)")
    # labels recomputed from R with contract.classify (labels only from evidence); INVALID_RUN is the only override
    R = np.array([[_f(x) for x in row] for row in rec["R"]], float)
    c = contract.classify(R, rec["margin_config"].get("eps", 0.0))
    if rec["evidence"] != "INVALID_RUN" and rec["evidence"] != c["evidence"]:
        errs.append(f"evidence {rec['evidence']} != classify {c['evidence']}")
    if rec["evidence"] == "INVALID_RUN" and not rec["invalid_reasons"]:
        errs.append("INVALID_RUN without invalid_reasons")
    for name, got, want in (("V", rec["V"], c["V"]), ("V_hind", rec["V_hind"], c["V_hind"]), ("info_gap", rec["info_gap"], c["info_gap"])):
        if not _same(_f(got), want):
            errs.append(f"{name}={got} != classify {want}")
    if not (np.isnan(c["V"]) or np.isnan(c["V_hind"])) and c["V"] > c["V_hind"] + 1e-9:
        errs.append("V > V_hind")
    if c["best_policy"] is not None and rec["best_policy"] != rec["policy_library"][c["best_policy"]]:
        errs.append("best_policy != argmax of r_j")
    # terminal block
    tm = rec["terminal"]
    if set(tm["mode"]) != set(rec["policy_library"]) or any(v not in TERMINAL_MODES for v in tm["mode"].values()):
        errs.append("terminal.mode keys differ from policy_library or bad mode")
    for j in range(P):
        for w in range(W):
            t = tm["boolean"][j][w]
            if (t is None) != (rec["R_mask"][j][w] == 0) or (t is not None and not isinstance(t, bool)):
                errs.append(f"terminal.boolean[{j}][{w}] inconsistent with R_mask")
            if t is False and rec["R"][j][w] not in ("-inf", None):
                errs.append(f"terminal failed but R[{j}][{w}] is not -inf/null")
    # pair fields: all set or all null; Cosmos variant fields consistent
    pf = [rec["pair_id"], rec["pair_role"], rec["varied_factor"]]
    if any(x is not None for x in pf) and (any(x is None for x in pf) or rec["pair_role"] not in ("unrecoverable", "recoverable")):
        errs.append("incomplete pair fields")
    if (rec["parent_id"] is None) != (rec["variant_kind"] is None):
        errs.append("parent_id and variant_kind must be both set or both null")
    # history: only delivered features, never world identity, per-world list in world_set order
    h = rec["history"]
    if h is not None:
        if set(h) != {"policy", "per_world", "common_prefix_frames"} or len(h["per_world"]) != W:
            errs.append("history must be {policy, per_world[len(world_set)], common_prefix_frames}")
        else:
            bad = False
            for fs in h["per_world"]:
                for fr in fs:
                    if set(fr) != HIST_KEYS or any(set(o) != OBJ_KEYS for o in fr["objs"]):
                        errs.append("history frame has keys other than delivered-observation features")
                        bad = True
                        break
                    if fr["t_available"] < fr["t_capture"] - 1e-9:
                        errs.append("history frame available before captured")
                        bad = True
                        break
                if bad:
                    break
            pre = h["common_prefix_frames"]
            if not bad and pre > 0 and any(fs[:pre] != h["per_world"][0][:pre] for fs in h["per_world"]):
                errs.append("common_prefix_frames claims identical frames that differ")
    # npz
    z = _npz(os.path.join(root, rec["trace_path"]))
    if z is None:
        errs.append(f"missing {rec['trace_path']}")
    else:
        for k in (f"{rec['trace_key']}/R", f"{rec['trace_key']}/R_search"):
            if k not in z.files:
                errs.append(f"npz missing {k}")
        if h is not None and f"{rec['trace_key']}/w0__t_capture" not in z.files:
            errs.append("npz missing history arrays")
        if f"{rec['trace_key']}/R" in z.files:
            a = z[f"{rec['trace_key']}/R"]
            if not np.array_equal(np.nan_to_num(a, nan=7e9), np.nan_to_num(R, nan=7e9)):
                errs.append("npz R differs from jsonl R")
    return errs


def check_dataset(rows):
    """Cross-row checks on E1.1 rows (returns a list of messages):
    1. all rows sharing a natural group (family + non-distance factors, splits.split_group), pair rows included, share ONE assigned split_group;
    2. every pair_id has exactly one recoverable and one unrecoverable member, both in the same split_group;
    3. the assigned groups are exactly the union-find merge of the natural groups linked by pairs (splits.merge_groups), so no counterfactual
       neighbour of either pair member can sit in another split;
    4. within (backend, family): one policy_library_id and one policy_library_version."""
    errs, by_nat, pairs, libs = [], {}, {}, {}
    rows = [r for r in rows if r.get("schema_version") == "E1.1"]
    for r in rows:
        nat = splits.split_group(r["family_id"], r["theta"])
        r_nat = (r["backend"], nat)
        by_nat.setdefault(r_nat, set()).add(r["split_group"])
        if r["pair_id"] is not None:
            pairs.setdefault(r["pair_id"], []).append((r["pair_role"], r["split_group"], nat))
        libs.setdefault((r["backend"], r["family_id"]), set()).add((r["policy_library_id"], r["policy_library_version"]))
    for k, g in by_nat.items():
        if len(g) > 1:
            errs.append(f"rows with the same non-distance factors {k} are in several split groups {sorted(g)}")
    links = []
    for pid, ms in pairs.items():
        roles = sorted(m[0] for m in ms)
        if roles != ["recoverable", "unrecoverable"]:
            errs.append(f"pair {pid}: members {roles}, need exactly one recoverable and one unrecoverable")
            continue
        if len({m[1] for m in ms}) != 1:
            errs.append(f"pair {pid}: members are in different split groups")
        links.append((ms[0][2], ms[1][2]))
    want = splits.merge_groups({n for _, n in by_nat}, links)
    for r in rows:
        w = want[splits.split_group(r["family_id"], r["theta"])]
        if r["split_group"] != w:
            errs.append(f"{r['id']}: split_group {r['split_group']!r} != merged group {w!r} (non-distance-factor group, merged across matched pairs)")
    for k, v in libs.items():
        if len({x[0] for x in v}) > 1 or len({x[1] for x in v}) > 1:
            errs.append(f"{k}: several library ids/versions {sorted(v)}")
    return errs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dataset", help="directory containing scenes.jsonl")
    a = ap.parse_args()
    ids, bad, n, rows = set(), 0, 0, []
    for line in open(f"{a.dataset}/scenes.jsonl"):
        rec = json.loads(line)
        rows.append(rec)
        n += 1
        errs = check(rec, a.dataset)
        if rec.get("id") in ids:
            errs.append("duplicate id")
        ids.add(rec.get("id"))
        if errs:
            bad += 1
            print(rec.get("id"), errs)
    cross = check_dataset(rows)
    for e in cross:
        print("dataset:", e)
    bad += len(cross)
    print(f"{n} records, {bad} invalid")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
