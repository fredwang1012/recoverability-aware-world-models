"""A2: validate execution records (JSONL) and learning examples.  Exit 1 on any violation.

  python validate_records.py results/carla/records.jsonl [--examples results/carla/examples.jsonl]

Checks: required fields; R_raw shape == policies x worlds; no NaN/inf written as numbers (unresolved = null, -inf = "-inf");
V == max_j min_w R over resolved rows; evidence_class == contract.classify (INVALID_RUN excepted); V <= V_hind;
evidence_scope valid; terminal failure <=> -inf; no hidden metadata keys in learning-example features."""
import argparse
import json
import math
import sys

import numpy as np

import contract
from records import EVIDENCE_SCOPES, HIDDEN_KEYS

REQUIRED = ["case_id", "family_id", "split_group", "backend", "theta", "policies", "worlds", "R_raw", "R_norm",
            "terminal_ok", "margin_config", "evidence_scope", "seed", "V", "V_hind", "evidence_class", "coverage",
            "checksum", "code_version", "schema_version"]


def _num(x):
    if x is None:
        return np.nan
    if x == "-inf":
        return -math.inf
    return float(x)


def check_record(r):
    errs = [f"missing {k}" for k in REQUIRED if k not in r]
    if errs:
        return errs
    P, W = len(r["policies"]), len(r["worlds"])
    if len(r["R_raw"]) != P or any(len(row) != W for row in r["R_raw"]):
        return ["R_raw shape != policies x worlds"]
    for row in list(r["R_raw"]) + list(r["R_norm"]):
        for x in row:
            if isinstance(x, float) and not math.isfinite(x):
                errs.append("non-finite float serialized as a number (use null / '-inf')")
    if r["evidence_scope"] not in EVIDENCE_SCOPES:
        errs.append(f"bad evidence_scope {r['evidence_scope']}")
    R = np.array([[_num(x) for x in row] for row in r["R_raw"]], float)
    c = contract.classify(R, r["margin_config"].get("eps", 0.0))
    if r["evidence_class"] != "INVALID_RUN" and r["evidence_class"] != c["evidence"]:
        errs.append(f"evidence_class {r['evidence_class']} != classify {c['evidence']}")
    if r["evidence_class"] == "INVALID_RUN" and not r.get("invalid_reasons"):
        errs.append("INVALID_RUN without invalid_reasons")
    if r["V"] is not None and np.isfinite(c["V"]) and abs(float(r["V"]) - c["V"]) > 1e-9:
        errs.append(f"V={r['V']} != max_j min_w R = {c['V']}")
    if r["V"] is not None and r["V_hind"] is not None and float(r["V"]) > float(r["V_hind"]) + 1e-9:
        errs.append("V > V_hind")
    for j, row in enumerate(r["terminal_ok"]):
        for k, t in enumerate(row):
            if t is False and r["R_raw"][j][k] not in ("-inf", None):
                errs.append(f"terminal failed but R_raw[{j}][{k}] is not -inf/null")
    if r["coverage"]["run"] != int((~np.isnan(R)).sum()):
        errs.append("coverage.run != resolved entries")
    return errs


def hidden_leaks(example):
    """Paths of hidden-metadata keys found anywhere in the FEATURE part of a learning example (label/contract_ids exempt)."""
    leaks = []
    feats = {k: v for k, v in example.items() if k not in ("label", "contract_ids")}

    def walk(o, p):
        if isinstance(o, dict):
            for k, v in o.items():
                if k.lower() in HIDDEN_KEYS:
                    leaks.append(f"{p}/{k}")
                walk(v, f"{p}/{k}")
        elif isinstance(o, list):
            for i, v in enumerate(o):
                walk(v, f"{p}[{i}]")

    walk(feats, "")
    return leaks


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("records")
    ap.add_argument("--examples")
    a = ap.parse_args()
    bad = n = 0
    for line in open(a.records):
        r = json.loads(line)
        n += 1
        e = check_record(r)
        if e:
            bad += 1
            print(r.get("case_id"), e)
    if a.examples:
        for line in open(a.examples):
            lk = hidden_leaks(json.loads(line))
            if lk:
                bad += 1
                print("LEAK", lk)
    print(f"{n} records, {bad} problems")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
