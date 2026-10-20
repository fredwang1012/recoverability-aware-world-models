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


def rc13_rows():
    """[(scene, min_margin_over_runs_and_worlds)] parsed from the Repeatability table of negatives.md."""
    t = rd(CARLA, "negatives.md")
    t = t[t.index("### Repeatability and headline"):]
    out = []
    for m in re.finditer(r"^\| (\([\d., ]+\)) \| \[.*?\] \| ([\d.]+) \.\. ([\d.]+) \| ([\d.]+) \.\. ([\d.]+) \| \w+ \|$", t, re.M):
        out.append((m.group(1), min(float(m.group(2)), float(m.group(4)))))
    return out


def rc13_summary():
    rows = rc13_rows()
    robust = [n for n, m in rows if m >= MAX_SPREAD]
    return (f"RC13: {len(robust)} of {len(rows)} scenes keep every witness margin at or above the max observed spread {MAX_SPREAD} "
            f"({', '.join(robust) or 'none'}); the scenes inside it are "
            f"{', '.join(f'{n} (smallest margin {m:.4f})' for n, m in rows if m < MAX_SPREAD) or 'none'}. negatives.md calls all {len(rows)} 'robust' only "
            f"because it applied sigma = 0.0167; at {MAX_SPREAD} the robust count is {len(robust)} of {len(rows)}.")


def flag_rows():
    rows = []
    for n, m in rc13_rows():
        rows.append((f"RC13 scene {n}: smallest witness margin over repeats and worlds", m, "negatives.md, Repeatability"))
    t = rd(CARLA, "negatives.md")
    m = re.search(r"Open-left control, repeatability.*?range ([\d.]+) \.\. ([\d.]+)", t, re.S)
    rows.append(("RC09 open-left control witness (bypass_left_lc2)", float(m.group(1)), "negatives.md, Controls"))
    c = rd(CARLA, "core.md")
    m = re.search(r"^\| 18 \| 2 \| 30 \| -inf \| bypass_left ([\d.]+) ", c, re.M)
    rows.append(("RC03 witness at v 18, gap 30 (bypass_left)", float(m.group(1)), "core.md, RC03 chosen witnesses"))
    k = rd(CARLA, "compositional.md")
    m = re.search(r"^\| 60 \| 20 \| 16 \| R_REF \| True \| immediate_entry=-inf; wait_observe_enter=([\d.]+);", k, re.M)
    rows.append(("RC06 named witness wait_observe_enter (scene 60/20/16; the scene also has wait_brake_observe_enter at 0.105, so the label stands)", float(m.group(1)), "compositional.md, RC06"))
    ig = rd(CARLA, "mining", "infogap", "infogap.md")
    ig = ig[ig.index("### Found info-gap scenes"):]
    for m in re.finditer(r"^\| (cem|straddle) \| (\d+) \| ([\d.]+) \| ([\d.]+) \| ([\d.]+) \| (-inf|-?[\d.]+) \| (-?[\d.]+) \|", ig, re.M):
        algo, seed, D, v, tr, V, Vh = m.groups()
        rows.append((f"D5 CARLA info-gap scene D {D}, v {v}, t_reveal {tr} ({algo} s{seed}): V_hind (hindsight witness)", float(Vh), "mining/infogap/infogap.md"))
        if V != "-inf":
            rows.append((f"D5 CARLA info-gap scene D {D}, v {v}, t_reveal {tr} ({algo} s{seed}): V (negative side)", float(V), "mining/infogap/infogap.md"))
    for v, D, t_, ev, pol in occ_rows():
        V = occ_v(pol)
        if ev == "R_REF" and 0 <= V < 2 * MAX_SPREAD:
            rows.append((f"Task 7 sweep v {v}, D {D}, t_reveal {t_:g}: R_REF, V", V, "occlusion.md, full margins"))
        elif ev != "R_REF" and -2 * MAX_SPREAD < V < 0:
            rows.append((f"Task 7 sweep v {v}, D {D}, t_reveal {t_:g}: {ev}, V (transition bracket edge)", V, "occlusion.md, full margins"))
    return rows


def flags_table():
    rows = flag_rows()
    L = ["| headline claim | margin | inside max observed spread (abs margin < 0.028)? | source |", "|---|---|---|---|"]
    for name, m, src in rows:
        L.append(f"| {name} | {m:+.4f} | {'**yes**' if abs(m) < MAX_SPREAD else 'no'} | {src} |")
    n = sum(abs(m) < MAX_SPREAD for _, m, _ in rows)
    return "\n".join(L) + f"\n\n{n} of the {len(rows)} listed margins are inside the spread. (Rows with margins up to twice the spread are listed so the reader can see how close the others are.)"


def pairs_table():
    """Representative matched pairs per family from pairs.md: first, middle and last distinct rows (one per varied factor where possible)."""
    out = []
    for fam in ("corridor", "rc03"):
        t = rd(CARLA, "mining", fam, "pairs.md")
        rows = [ln for ln in t.splitlines() if ln.startswith("| " + fam + "_bisect_s")]
        seen, pick = set(), []
        for ln in rows:
            cells = [c.strip() for c in ln.strip("|").split("|")]
            m = re.match(r"same scene except (\w+) ", cells[1])
            key = m.group(1) if m else cells[1]
            if key not in seen and cells[4] == "True":
                seen.add(key)
                pick.append(cells)
        out.append(f"**{fam}**: {len(rows)} pair rows in `mining/{fam}/pairs.md`; representative pairs (first verified pair for each varied factor):\n")
        out.append("| pair | description (same scene except ...) | unrecoverable side | recoverable side | verified |")
        out.append("|---|---|---|---|---|")
        for c in pick:
            out.append(f"| {c[0]} | {c[1]} | {c[2]} | {c[3]} | {c[4]} |")
        out.append("")
    return "\n".join(out)


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


def _roles(rows):
    out = {}
    for r in rows:
        if r["pair_id"]:
            out.setdefault(r["pair_id"], []).append(r["pair_role"])
    return out


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
        ds[b] = {"n": len(rows), "hist": sum(r["history"] is not None for r in rows), "pair_rows": sum(r["pair_id"] is not None for r in rows), "pair_ids": len({r["pair_id"] for r in rows if r["pair_id"]}),
                 "pair_ids_complete": sum(sorted(x) == ["recoverable", "unrecoverable"] for x in _roles(rows).values()),
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
        "FLAGS": flags_table(), "RC13_SUMMARY": rc13_summary(), "PAIRS": pairs_table(),
        "SIGMA028": rd(CARLA, "mining", "sigma028.md").split("\n", 2)[2].replace("### ", "#### "), "IG_CARLA": ig_c.replace("## D5", "### D5", 1), "IG_MOCK": ig_m.replace("## D5", "### D5", 1),
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
