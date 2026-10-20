"""
Task 9: build docs/carla_results.md (spec section 7 report) from the committed result files.

    python scripts/build_carla_report.py          (repo root)

Tables are copied or computed from files under mining/results/carla (and results/mock for the mock D5 table); prose is the template below.
Nothing here runs a simulator.  Every number in a table comes from a file; the repeat-spread flags are computed from the margins in occlusion.md,
negatives.md-listed values and core.md-listed values quoted in FLAGS (each with its source file).
"""
import json
import os
import re

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
CARLA = os.path.join(ROOT, "mining", "results", "carla")
MOCK = os.path.join(ROOT, "results", "mock")
MAX_SPREAD = 0.028          # RC13 scene (14, 20, 0.2), witness world B 0.0224 .. 0.0504 (negatives.md); exceeds the Task 5 sigma 0.0167


def rd(*p):
    return open(os.path.join(*p), encoding="utf-8").read()


def section(text, start, end=None):
    i = text.index(start)
    j = text.index(end, i + len(start)) if end else len(text)
    return text[i:j].strip()


def occ_rows():
    """[(v, D, t, evidence, {policy: (a, b)})] from the full-margins table of occlusion.md."""
    s = rd(CARLA, "occlusion.md")
    out = []
    for m in re.finditer(r"^\| (\d+) \| (\d+) \| ([\d.]+) \| (\w+) \| (.*) \|$", s, re.M):
        v, D, t, ev, mg = m.groups()
        pol = {}
        for part in mg.split("; "):
            name, val = part.split("=")
            a, b = val.split("/")
            pol[name] = (float(a), float(b))     # float('-inf') parses
        out.append((int(v), int(D), float(t), ev, pol))
    return out


def occ_v(pol):
    return max(min(x) for x in pol.values())


def evidence_mix_occ():
    mix = {}
    for _, _, _, ev, _ in occ_rows():
        mix[ev] = mix.get(ev, 0) + 1
    return mix


def flags_table():
    rows = [("RC13 (v 18, D 26, t_reveal 0.2) witness margin", 0.0182, "negatives.md, Repeatability"),
            ("RC13 (v 14, D 20, t_reveal 0.2) witness world B, lowest repeat", 0.0224, "negatives.md, Repeatability (this scene IS the 0.028 spread: range 0.0224 .. 0.0504)"),
            ("RC09 open-left control", 0.022, "negatives.md, Controls"),
            ("RC03 witness at v 18, gap 30 (bypass_left)", 0.008, "core.md, RC03 (single gap window)"),
            ("D5 CARLA info-gap scene D 20.8, v 16, t_reveal 0.75: V_hind", 0.002, "mining/infogap/infogap.md (hindsight witness margin; the label V_hind >= eps is not robust)")]
    for v, D, t, ev, pol in occ_rows():
        V = occ_v(pol)
        if ev == "R_REF" and 0 <= V < MAX_SPREAD:
            rows.append((f"Task 7 sweep v {v}, D {D}, t_reveal {t:g}: R_REF with V = {V:.3f}", V, "occlusion.md, full margins"))
    flips = []
    for v, D, t, ev, pol in occ_rows():
        V = occ_v(pol)
        if ev != "R_REF" and -MAX_SPREAD < V < 0 and V != float("-inf"):
            flips.append((f"Task 7 sweep v {v}, D {D}, t_reveal {t:g}: {ev} with V = {V:.3f} (bracket edge of the transition)", V, "occlusion.md, full margins"))
    rows += flips
    L = ["| headline claim | margin | below 0.028? | source |", "|---|---|---|---|"]
    for name, m, src in rows:
        L.append(f"| {name} | {m:+.3f} | {'yes' if abs(m) < MAX_SPREAD else 'no'} | {src} |")
    return "\n".join(L)


def transitions_table():
    rows = occ_rows()
    by = {}
    for v, D, t, ev, pol in rows:
        by.setdefault((v, D), []).append((t, ev))
    L = ["| v (m/s) | D (m) | classes over t_reveal (R = R_REF, I = U_INFORMATION, L = U_LIBRARY) |", "|---|---|---|"]
    for k in sorted(by):
        s = "".join({"R_REF": "R", "U_INFORMATION": "I", "U_LIBRARY": "L"}.get(e, "?") for _, e in sorted(by[k]))
        L.append(f"| {k[0]} | {k[1]} | {s} |")
    return "\n".join(L)


def main():
    rt = json.load(open(os.path.join(CARLA, "runtime.json")))
    pp = json.load(open(os.path.join(CARLA, "physics_profile.json")))
    foundation = rd(CARLA, "foundation.md")
    anchors = section(foundation, "## Anchor vs CARLA (residual)", "### RC01 sweep")
    anchors = anchors.replace("## Anchor vs CARLA (residual)", "").strip()
    calib = section(rd(CARLA, "calibration.md"), "## CARLA physics profile", "### Friction sweep")
    det = rd(CARLA, "determinism.md").strip()
    tim = rd(CARLA, "timing.md").strip()
    selft = rd(CARLA, "selftest.txt").strip()
    cs = rd(CARLA, "mining", "corridor", "summary.md")
    rs = rd(CARLA, "mining", "rc03", "summary.md")
    cmix = section(cs, "### Evidence mix")
    rmix = section(rs, "### Evidence mix")
    ctab = section(cs, "| algo | scenes", "### Evidence mix")
    rtab = section(rs, "| algo | scenes", "### Evidence mix")
    ig_c = rd(CARLA, "mining", "infogap", "infogap.md")
    ig_m = rd(MOCK, "mining", "infogap", "infogap.md")
    ds = {}
    for b, p in (("carla", os.path.join(CARLA, "dataset", "scenes.jsonl")), ("mock", os.path.join(MOCK, "dataset", "scenes.jsonl"))):
        rows = [json.loads(l) for l in open(p)]
        ds[b] = {"n": len(rows), "hist": sum(r["history"] is not None for r in rows), "pairs": sum(r["pair_id"] is not None for r in rows),
                 "fam": {f: sum(r["family_id"] == f for r in rows) for f in sorted({r["family_id"] for r in rows})},
                 "ev": {e: sum(r["evidence"] == e for r in rows) for e in sorted({r["evidence"] for r in rows})},
                 "pp": sorted({str(r["physics_profile"]) for r in rows}), "size": os.path.getsize(p) + os.path.getsize(os.path.join(os.path.dirname(p), "traces.npz"))}
    tpl = rd(ROOT, "docs", "carla_results.template.md")
    sub = {
        "RUNTIME_JSON": json.dumps({k: rt[k] for k in ("server_version", "client_version", "map", "map_hash", "python", "numpy", "gpu", "dt")}, indent=1),
        "PROFILE_ID": pp["id"], "BLUEPRINTS": json.dumps(rt["blueprints"]), "SETTINGS": json.dumps(rt["settings"]),
        "SELFTEST": selft, "CALIB": calib, "DETERMINISM": det, "TIMING": tim, "ANCHORS": anchors,
        "CORR_TAB": ctab, "RC03_TAB": rtab, "CORR_MIX": cmix, "RC03_MIX": rmix,
        "TRANSITIONS": transitions_table(), "OCC_MIX": ", ".join(f"{k} = {v}" for k, v in sorted(evidence_mix_occ().items())),
        "FLAGS": flags_table(), "IG_CARLA": ig_c.replace("## D5", "### D5", 1), "IG_MOCK": ig_m.replace("## D5", "### D5", 1),
        "DS": json.dumps(ds, indent=1), "D5_READING": rd(ROOT, "docs", "carla_results_d5_reading.md").strip(), "MAX_SPREAD": f"{MAX_SPREAD}",
    }
    for k, v in sub.items():
        tpl = tpl.replace("{{" + k + "}}", v)
    left = re.findall(r"\{\{[A-Z_0-9]+\}\}", tpl)
    assert not left, left
    open(os.path.join(ROOT, "docs", "carla_results.md"), "w", encoding="utf-8", newline="\n").write(tpl)
    print("wrote docs/carla_results.md", len(tpl), "chars")


if __name__ == "__main__":
    main()
