"""T7: validate an exported scenes.jsonl: schema, no NaNs, V == max_j R_j, category consistent with V and eps."""
import argparse
import json
import math
import os
import sys

REQUIRED = ["id", "source", "backend", "theta", "u", "seed", "world_set", "policy_library", "R", "V", "category",
            "evidence", "t_reveal", "margin_config", "descriptors", "pair_id", "pair_role", "varied_factor",
            "trace_path", "eps", "edge_band", "planner_margin"]
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
    if rec["pair_id"] and (rec["pair_role"] not in ("unrecoverable", "recoverable") or not rec["varied_factor"]):
        errs.append("incomplete pair fields")
    if not os.path.exists(os.path.join(root, rec["trace_path"])):
        errs.append(f"missing trace {rec['trace_path']}")
    return errs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dataset", help="directory containing scenes.jsonl")
    a = ap.parse_args()
    ids, bad, n = set(), 0, 0
    for line in open(f"{a.dataset}/scenes.jsonl"):
        rec = json.loads(line)
        n += 1
        errs = check(rec, a.dataset)
        if rec.get("id") in ids:
            errs.append("duplicate id")
        ids.add(rec.get("id"))
        if errs:
            bad += 1
            print(rec.get("id"), errs)
    print(f"{n} records, {bad} invalid")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
