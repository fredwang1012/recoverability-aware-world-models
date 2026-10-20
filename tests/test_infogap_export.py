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
        assert r["split_group"].startswith("infogap_D")
    # labels in the export are exactly the evidence class of the record
    assert {r["evidence"] for r in rows} == {"U_INFORMATION", "R_REF"}


def test_export_pairs_share_split_group_and_dedupe(tmp_path, mock_scenes):
    n, nh, rows, out = _export(tmp_path, mock_scenes, pairs=True)
    assert n == 2                                        # pair members replace the same-theta plain records
    assert {r["pair_role"] for r in rows} == {"unrecoverable", "recoverable"}
    assert all(r["pair_id"] == r["split_group"] == "infogap_cem_s0_pair000" and r["varied_factor"] == "gap_m" for r in rows)
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
