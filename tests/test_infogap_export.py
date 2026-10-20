"""Task 9: D5 info-gap helpers + family (MOCK plant, no simulator) and the E1.1 export / validator.  Not a CARLA result."""
import copy
import json
import math
import os
import sys

import numpy as np
import pytest

import export as EX
import infogap as IG
import margin as M
import splits as SP
import validate_dataset as VD
from families import INFOGAP, mock_ctx

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import mine_cases as MC

CFG = M.DEFAULT_V2
NINF = "-inf"


# ----------------------------------------------------------------------------- pure D5 helpers
def test_full_objective_info_gap_and_labels_from_evidence():
    # two policies x two worlds: every world has a witness (V_hind 0.1) but no policy wins both -> V = -0.2 < eps
    rec = {"R_raw": [[0.1, -0.2], [-0.3, 0.1]], "evidence_class": "U_INFORMATION"}
    o = IG.full_objective(rec, 0.0)
    assert o["feasible"] and o["is_info_gap"] and not o["V_is_neg_inf"]
    assert o["V"] == pytest.approx(-0.2) and o["V_hind"] == pytest.approx(0.1) and o["objective"] == pytest.approx(0.3)
    assert o["recomputed_evidence"] == "U_INFORMATION"


def test_full_objective_neg_inf_is_infinite_gap_and_unrun_is_not_zero():
    o = IG.full_objective({"R_raw": [[0.1, NINF], [NINF, 0.1]]}, 0.0)
    assert o["is_info_gap"] and o["V_is_neg_inf"] and o["objective"] == math.inf
    # an unrun entry (None) is unresolved: it can never create an info gap or count as a witness
    o2 = IG.full_objective({"R_raw": [[0.1, None], [None, 0.1]]}, 0.0)
    assert not o2["is_info_gap"] and o2["objective"] is None


def test_full_objective_infeasible_and_r_ref():
    inf = IG.full_objective({"R_raw": [[0.1, NINF], [0.1, NINF]]}, 0.0)           # world 2 unsolvable by anyone: V_hind < eps
    assert not inf["feasible"] and inf["objective"] is None and not inf["is_info_gap"]
    ok = IG.full_objective({"R_raw": [[0.1, 0.2]]}, 0.0)                              # R_REF: V >= eps, no gap to mine
    assert ok["feasible"] and not ok["is_info_gap"] and ok["objective"] == pytest.approx(0.0)


def test_cem_surrogate_ranks_feasible_gap_before_infeasible():
    big = [[0.2, -0.3], [-0.3, 0.2]]       # V_hind 0.2, V -0.3 -> gap 0.5
    small = [[0.2, -0.1], [-0.1, 0.2]]     # gap 0.3
    infeasible = [[0.2, -0.5], [0.2, -0.5]]
    s = [IG.cem_surrogate(R) for R in (big, small, infeasible)]
    assert s[0] < s[1] < 0 < s[2]          # CEM minimises
    assert IG.cem_surrogate([[None, None]]) > 0          # nothing resolved: penalty, never a good value


def test_straddle_surrogate_is_search_v_and_missing_is_floor():
    assert IG.straddle_surrogate([[0.2, -0.1], [-0.3, 0.2]]) == pytest.approx(-0.1)
    assert IG.straddle_surrogate([[None, None]]) == -1.0


def test_rank_key_orders_finite_then_neg_inf_then_rest():
    f = lambda r: IG.full_objective({"R_raw": r}, 0.0)
    a, b = f([[0.1, -0.4], [-0.4, 0.1]]), f([[0.1, -0.1], [-0.1, 0.1]])
    c = f([[0.1, NINF], [NINF, 0.1]])
    d = f([[0.1, 0.2]])
    keys = sorted([c, d, b, a], key=IG.rank_key)
    assert keys == [a, b, c, d]


def test_transition_helper():
    cl = [(0.0, "R_REF"), (0.1, "R_REF"), (0.2, "U_INFORMATION"), (0.3, "U_INFORMATION")]
    assert IG.sweep_transition_from_class_string(cl) == (0.1, 0.2)
    assert IG.sweep_transition_from_class_string([(0.0, "R_REF"), (0.1, "R_REF")]) is None


# ----------------------------------------------------------------------------- family on the mock plant
@pytest.fixture(scope="module")
def mock_scenes():
    ctx = mock_ctx(CFG)
    out = []
    for D, v, tr in ((19.0, 14.0, 0.6), (25.0, 14.0, 0.2)):
        th = INFOGAP.decode((np.array([D, v, tr]) - INFOGAP.lo) / (INFOGAP.hi - INFOGAP.lo))
        out.append((th, INFOGAP.run(th, ctx)))
    return out


def test_family_theta_is_quantised_and_fixed_plant_parameters():
    th = INFOGAP.decode([0.33, 0.51, 0.4999])
    assert th["t_reveal_s"] == pytest.approx(round(th["t_reveal_s"] / 0.05) * 0.05)
    assert th["gap_m"] == round(th["gap_m"], 1) and th["v_mps"] == round(th["v_mps"], 1)
    assert th["tau_s"] == 0.25 and th["b_mps2"] == 6.19


def test_family_scene_labels_and_history(mock_scenes):
    (th1, r1), (th2, r2) = mock_scenes
    assert r1.evidence_class == "U_INFORMATION"                  # D 19 < stop distance, late reveal: info gap
    assert r2.evidence_class == "R_REF"                          # D 25 > stop distance at 14 m/s: common stop exists
    j = r1.to_json()
    obj = IG.full_objective(j, 0.0)
    assert obj["is_info_gap"] and obj["recomputed_evidence"] == r1.evidence_class
    # keep_history: every episode carries the delivered frames; before the reveal the two worlds are indistinguishable
    h = EX.history_of(j)
    assert h is not None and len(h["per_world"]) == 2
    n_pre = int(round(th1["t_reveal_s"] / 0.05))
    assert h["common_prefix_frames"] >= n_pre
    assert all(f["objs"] == [] for f in h["per_world"][0][:n_pre])
    assert h["per_world"][0][h["common_prefix_frames"]] != h["per_world"][1][h["common_prefix_frames"]]


def test_search_value_differs_per_algorithm_and_uses_search_grade(mock_scenes):
    _, r = mock_scenes[0]
    sc, ss = INFOGAP.search_value(r, "cem"), INFOGAP.search_value(r, "straddle")
    assert sc < 0 and ss != sc
    assert all(x is None or math.isfinite(x) for row in r.R_norm for x in row)        # the search signal is finite


def test_history_not_stored_without_keep_history():
    from cases import occlusion as OC
    from sim.plant_kinematic import KinematicPlant
    th = {"gap_m": 19.0, "v_mps": 14.0, "tau_s": 0.25, "t_reveal_s": 0.6}
    rec = OC.run("RC11", lambda: KinematicPlant(b_max=6.19), th, CFG, "mock", "mock_replay")
    assert all("history" not in v for v in rec.to_json()["traces"].values())


# ----------------------------------------------------------------------------- E1.1 export + validator
def _write_records(root, mock_scenes, pairs=False):
    d = root / "infogap"
    d.mkdir(parents=True)
    rows = []
    for k, (th, rec) in enumerate(mock_scenes):
        j = rec.to_json()
        j["_algo"], j["_seed"] = "cem", 0
        rows.append(j)
    with open(d / "records_cem_s0.jsonl", "w") as f:
        for j in rows:
            f.write(json.dumps(j) + "\n")
    if pairs:
        a, b = copy.deepcopy(rows[0]), copy.deepcopy(rows[1])
        a["parent_family_id"] = b["parent_family_id"] = "infogap_cem_s0_pair000"
        a["varied_factor"] = b["varied_factor"] = "gap_m"
        with open(d / "pairs_bisect_s0.jsonl", "w") as f:
            f.write(json.dumps({"pair_id": "infogap_cem_s0_pair000", "varied": "gap_m", "unrecoverable_side": a, "recoverable_side": b}) + "\n")
    return rows


def _export(tmp_path, mock_scenes, pairs=False):
    root = tmp_path / "mining"
    _write_records(root, mock_scenes, pairs)
    out = tmp_path / "dataset"
    n, nh = EX.export_records(str(root), str(out), "mock", families=("infogap",))
    rows = [json.loads(l) for l in open(out / "scenes.jsonl")]
    return n, nh, rows, out


def test_export_passes_validator_and_null_fields_consistent(tmp_path, mock_scenes):
    n, nh, rows, out = _export(tmp_path, mock_scenes)
    assert n == 2 and nh == 2 and len(rows) == 2
    for r in rows:
        assert VD.check(r, str(out)) == []
        assert r["backend"] == "mock" and r["physics_profile"] is None
        assert r["pair_id"] is None and r["pair_role"] is None and r["varied_factor"] is None
        assert r["parent_id"] is None and r["variant_kind"] is None
        assert set(EX.E1_FIELDS) <= set(r)
        assert r["split_group"] == SP.split_group("infogap", r["theta"])
    # labels in the export are exactly the evidence class of the record
    assert {r["evidence"] for r in rows} == {"U_INFORMATION", "R_REF"}


def test_export_pairs_share_split_group_and_dedupe(tmp_path, mock_scenes):
    n, nh, rows, out = _export(tmp_path, mock_scenes, pairs=True)
    assert n == 2                                        # pair members replace the same-theta plain records
    assert {r["pair_role"] for r in rows} == {"unrecoverable", "recoverable"}
    assert all(r["pair_id"] == "infogap_cem_s0_pair000" and r["varied_factor"] == "gap_m" for r in rows) and len({r["split_group"] for r in rows}) == 1
    assert all(VD.check(r, str(out)) == [] for r in rows)
    assert all(set(r["sources"]) == {"pairs_bisect_s0.jsonl", "records_cem_s0.jsonl"} for r in rows)


def test_carla_labelled_rows_need_a_profile(tmp_path, mock_scenes):
    root = tmp_path / "mining"
    rows = _write_records(root, mock_scenes)
    for j in rows:                                       # relabel a MOCK record as carla ONLY to test the checker (never exported to results)
        j["backend"], j["physics_profile_id"], j["evidence_scope"] = "carla", "pp_381b4c6e55", "carla_finite_replay"
    with open(root / "infogap" / "records_cem_s0.jsonl", "w") as f:
        for j in rows:
            f.write(json.dumps(j) + "\n")
    out = tmp_path / "ds"
    EX.export_records(str(root), str(out), "carla", families=("infogap",))
    recs = [json.loads(l) for l in open(out / "scenes.jsonl")]
    assert all(r["backend"] == "carla" and r["physics_profile"] == "pp_381b4c6e55" and VD.check(r, str(out)) == [] for r in recs)
    bad = copy.deepcopy(recs[0]); bad["physics_profile"] = None
    assert any("physics profile" in e for e in VD.check(bad, str(out)))
    with pytest.raises(RuntimeError):                    # backends are never mixed
        EX.export_records(str(root), str(tmp_path / "x"), "mock", families=("infogap",))


def test_validator_catches_e1_corruption(tmp_path, mock_scenes):
    _, _, rows, out = _export(tmp_path, mock_scenes)
    base = rows[0]
    c = lambda **kw: {**copy.deepcopy(base), **kw}
    assert any("evidence" in e for e in VD.check(c(evidence="R_REF" if base["evidence"] != "R_REF" else "U_LIBRARY"), str(out)))
    assert any("V=" in e or "V " in e for e in VD.check(c(V=0.5), str(out)))
    z = copy.deepcopy(base); z["R"][0][0] = None            # unrun entry but mask says run
    assert any("R_mask" in e for e in VD.check(z, str(out)))
    z = copy.deepcopy(base); z["history"]["per_world"][0][0]["world"] = "hid_own_lane"      # hidden metadata in the features
    assert any("delivered-observation" in e for e in VD.check(z, str(out)))
    z = copy.deepcopy(base); z["R"][0][0] = float("nan")
    assert VD.check(z, str(out))
    assert any("both set or both null" in e for e in VD.check(c(parent_id="x"), str(out)))
    assert any("pair" in e for e in VD.check(c(pair_id="p"), str(out)))
    z = copy.deepcopy(base); z["terminal"]["boolean"][0][0] = None
    assert any("terminal.boolean" in e for e in VD.check(z, str(out)))
    del z["history"]
    assert any("missing history" in e for e in VD.check(z, str(out)))


def test_records_without_history_export_null(tmp_path):
    """Records produced without keep_history (all pre-Task-9 CARLA records) export history = null, not an empty list."""
    from cases import occlusion as OC
    from sim.plant_kinematic import KinematicPlant
    th = {"gap_m": 19.0, "v_mps": 14.0, "tau_s": 0.25, "t_reveal_s": 0.6, "b_mps2": 6.19}
    rec = OC.run("RC11", lambda: KinematicPlant(b_max=6.19), th, CFG, "mock", "mock_replay", family="infogap").to_json()
    rec["_algo"], rec["_seed"] = "cem", 0
    d = tmp_path / "mining" / "infogap"
    d.mkdir(parents=True)
    open(d / "records_cem_s0.jsonl", "w").write(json.dumps(rec) + "\n")
    n, nh = EX.export_records(str(tmp_path / "mining"), str(tmp_path / "ds"), "mock", families=("infogap",))
    row = json.loads(open(tmp_path / "ds" / "scenes.jsonl").readline())
    assert (n, nh) == (1, 0) and row["history"] is None and VD.check(row, str(tmp_path / "ds")) == []


def test_run_one_uses_family_search_value(tmp_path):
    """mine_cases.run_one routes the algorithm name to the family's search surrogate (cem != straddle) and logs it per record."""
    ctx = mock_ctx(CFG)
    rows, _ = MC.run_one(INFOGAP, "cem", 3, 0, ctx, str(tmp_path), batch=3)
    assert len(rows) == 3 and all(math.isfinite(r["V"]) for r in rows)
    recs = [json.loads(l) for l in open(tmp_path / "records_cem_s0.jsonl")]
    for r in recs:
        assert r["_Vsearch"] == pytest.approx(IG.cem_surrogate(r["R_norm"]))
        assert r["split_group"].startswith("infogap_D")


# ----------------------------------------------------------------------------- fix round 1: split groups, half-pairs, library identity, required fields
def _plain_and_pairs(tmp_path, mock_scenes, pair_specs, with_lib=True):
    """infogap dir with the two mock scenes as plain records (library_id set like fam.meta() does) and the given pairs (list of (pair_id, unrec_idx, rec_idx));
    pair members carry NO library_id (as in the real pairs_*.jsonl)."""
    d = tmp_path / "mining" / "infogap"
    d.mkdir(parents=True)
    rows = []
    for th, rec in mock_scenes:
        j = rec.to_json()
        j["_algo"], j["_seed"] = "cem", 0
        if with_lib:
            j["library_id"] = "occ_test_lib"
        rows.append(j)
    with open(d / "records_cem_s0.jsonl", "w") as f:
        for j in rows:
            f.write(json.dumps(j) + "\n")
    with open(d / "pairs_bisect_s0.jsonl", "w") as f:
        for pid, iu, ir in pair_specs:
            a, b = copy.deepcopy(rows[iu]), copy.deepcopy(rows[ir])
            for m in (a, b):
                m.pop("library_id", None)
                m["parent_family_id"] = pid
            f.write(json.dumps({"pair_id": pid, "varied": "gap_m", "unrecoverable_side": a, "recoverable_side": b}) + "\n")
    out = tmp_path / "ds"
    EX.export_records(str(tmp_path / "mining"), str(out), "mock", families=("infogap",))
    return [json.loads(line) for line in open(out / "scenes.jsonl")], out


def test_two_pairs_sharing_a_theta_are_never_half_pairs(tmp_path, mock_scenes):
    rows, out = _plain_and_pairs(tmp_path, mock_scenes, [("p0", 0, 1), ("p1", 0, 1)])
    pair_rows = [r for r in rows if r["pair_id"]]
    assert len(pair_rows) == 4 and len({r["id"] for r in rows}) == len(rows)
    for pid in ("p0", "p1"):
        assert sorted(r["pair_role"] for r in pair_rows if r["pair_id"] == pid) == ["recoverable", "unrecoverable"]
    assert VD.check_dataset(rows) == [] and all(VD.check(r, str(out)) == [] for r in rows)
    # the same theta used as BOTH roles of one pair (equal after quantisation) is still two rows
    rows2, _ = _plain_and_pairs(tmp_path / "b", mock_scenes, [("q", 0, 0)])
    assert sorted(r["pair_role"] for r in rows2 if r["pair_id"] == "q") == ["recoverable", "unrecoverable"]


def test_validator_flags_half_pairs_and_split_violations(tmp_path, mock_scenes):
    rows, _ = _plain_and_pairs(tmp_path, mock_scenes, [("p0", 0, 0)])        # scene 1 stays a plain row
    half = [r for r in rows if not (r["pair_id"] and r["pair_role"] == "recoverable")]
    assert any("exactly one recoverable and one unrecoverable" in e for e in VD.check_dataset(half))
    bad = copy.deepcopy(rows)
    next(r for r in bad if r["pair_id"] is None)["split_group"] = "somewhere_else"
    assert any("split_group" in e for e in VD.check_dataset(bad))
    bad = copy.deepcopy(rows)
    for r in bad:
        if r["pair_id"] and r["pair_role"] == "recoverable":
            r["split_group"] = "other"
    assert any("different split groups" in e for e in VD.check_dataset(bad))
    a = copy.deepcopy(next(r for r in rows if r["pair_id"] is None))
    b = copy.deepcopy(a)
    a["id"], b["id"], b["split_group"] = "a", "b", "x"
    assert any("same non-distance factors" in e for e in VD.check_dataset([a, b]))


def test_split_group_rules():
    base = {"gap_m": 10.0, "v_mps": 12.0, "tau_s": 0.1, "prefix_s": 0.2, "b_mps2": 6.19}
    c1 = SP.split_group("corridor", base)
    c2 = SP.split_group("corridor", dict(base, gap_m=33.0))
    c3 = SP.split_group("corridor", dict(base, v_mps=12.5))
    assert c1 == c2 != c3                                     # all gap neighbours together, other factors split
    r = {"gap_m": 20.0, "v_mps": 18.0, "tau_s": 0.1, "b_mps2": 6.19}
    assert SP.split_group("rc03", r) == SP.split_group("rc03", dict(r, gap_m=40.0)) and SP.split_group("rc03", r).startswith("rc03:")
    i = {"gap_m": 20.0, "v_mps": 15.2, "t_reveal_s": 0.1}
    assert SP.split_group("infogap", i) == SP.split_group("infogap", dict(i, gap_m=31.0, v_mps=15.9, t_reveal_s=1.1))
    assert SP.split_group("infogap", i) != SP.split_group("infogap", dict(i, v_mps=16.0))


def test_pair_varying_a_nondistance_factor_stays_together(tmp_path, mock_scenes):
    """A pair that varies speed has members with different non-distance factors; both take the unrecoverable member's group."""
    th0, rec0 = mock_scenes[0]
    d = tmp_path / "mining" / "corridor"
    d.mkdir(parents=True)
    a, b = rec0.to_json(), copy.deepcopy(rec0.to_json())
    for m, v in ((a, 12.0), (b, 12.4)):
        m["family_id"] = m["split_group"] = "corridor"
        m["theta"] = {**m["theta"], "v_mps": v, "prefix_s": 0.0}
    with open(d / "pairs_bisect_s0.jsonl", "w") as f:
        f.write(json.dumps({"pair_id": "cp", "varied": "v_mps", "unrecoverable_side": a, "recoverable_side": b}) + "\n")
    EX.export_records(str(tmp_path / "mining"), str(tmp_path / "ds"), "mock", families=("corridor",))
    rows = [json.loads(line) for line in open(tmp_path / "ds" / "scenes.jsonl")]
    assert len(rows) == 2 and len({r["split_group"] for r in rows}) == 1
    assert rows[0]["split_group"] == SP.split_group("corridor", a["theta"])
    assert VD.check_dataset(rows) == []


def test_library_identity_is_family_level(tmp_path, mock_scenes):
    rows, _ = _plain_and_pairs(tmp_path, mock_scenes, [("p0", 0, 1)])
    assert {(r["policy_library_id"], r["policy_library_version"]) for r in rows} == {("occ_test_lib", rows[0]["policy_library_version"])}
    bad = copy.deepcopy(rows)
    bad[0]["policy_library_version"] = "deadbeef"
    assert any("library ids/versions" in e for e in VD.check_dataset(bad))
    bad = copy.deepcopy(rows)
    bad[0]["policy_library_id"] = "other"
    assert any("library ids/versions" in e for e in VD.check_dataset(bad))


def test_library_definition_drops_theta_valued_params_and_prefix_wrapper():
    inner = {"name": "brake_hold", "version": "0.1", "params": {}, "terminal": "protected_hold"}
    wrapped = {"name": "brake_hold@prefix", "version": "0.1", "params": {"prefix_s": 0.45}, "terminal": "protected_hold", "inner": inner}
    th = {"gap_m": 1.0, "prefix_s": 0.45}
    assert EX.library_definition([wrapped], th) == EX.library_definition([inner], {"gap_m": 1.0, "prefix_s": 0.0})
    other = dict(inner, params={"lc_time": 2.0})
    assert EX.library_definition([other], th) != EX.library_definition([inner], th)         # a real library parameter changes the version
    assert EX.library_definition([dict(other, params={"lc_time": 2.0, "prefix_s": 9})], th) == EX.library_definition([other], th)


def test_e1_fields_tied_to_required_and_every_required_field_is_checked(tmp_path, mock_scenes):
    assert set(EX.E1_FIELDS) <= set(VD.E1_REQUIRED)
    _, _, rows, out = _export(tmp_path, mock_scenes)
    assert set(EX.E1_FIELDS) | set(EX.E1_EXTRA) == set(rows[0]) and set(VD.E1_REQUIRED) <= set(rows[0])
    for k in VD.E1_REQUIRED:
        z = copy.deepcopy(rows[0])
        del z[k]
        assert f"missing {k}" in VD.check(z, str(out)), k
