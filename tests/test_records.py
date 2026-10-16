import json

import records as RC
import validate_records as VR


def _rec(R, terminal=None, **kw):
    P, W = len(R), len(R[0])
    rr = RC.ExecutionRecord(case_id="RC01", family_id="f", split_group="f", backend="reference", theta={"gap": 18},
                            policies=[f"p{i}" for i in range(P)], worlds=[f"w{k}" for k in range(W)], R_raw=R,
                            R_norm=R, terminal_ok=terminal or [[True] * W for _ in range(P)],
                            margin_config={"eps": 0.0}, evidence_scope="reference_model", seed=0, **kw)
    return json.loads(json.dumps(rr.finalize().to_json()))


def test_valid_records_pass_and_classes_match():
    assert VR.check_record(_rec([[0.4, 0.3], [-0.2, 0.1]])) == []
    r = _rec([[-1.0, 1.0], [1.0, -1.0]])
    assert r["evidence_class"] == "U_INFORMATION" and r["V_hind"] >= r["V"] and VR.check_record(r) == []
    r = _rec([[-0.5, None], [-0.3, None]])         # each policy refuted in one resolved world
    assert r["evidence_class"] == "U_LIBRARY" and VR.check_record(r) == []
    r = _rec([[0.5, None], [None, None]])          # no full witness, no full refutation
    assert r["evidence_class"] == "UNRESOLVED" and r["coverage"] == {"run": 1, "total": 4}


def test_neg_inf_is_serialised_as_string_and_validates():
    r = _rec([[float("-inf"), 0.2]], terminal=[[False, True]])
    assert r["R_raw"][0][0] == "-inf" and VR.check_record(r) == []


def test_validator_catches_tampering():
    r = _rec([[0.4, 0.3], [-0.2, 0.1]])
    bad = json.loads(json.dumps(r)); bad["V"] += 0.1
    assert any("max_j min_w" in e for e in VR.check_record(bad))
    bad = json.loads(json.dumps(r)); bad["evidence_class"] = "U_LIBRARY"
    assert any("classify" in e for e in VR.check_record(bad))
    bad = json.loads(json.dumps(r)); bad["R_raw"][0][0] = float("nan")
    assert VR.check_record(bad)
    bad = json.loads(json.dumps(r)); bad["terminal_ok"][1][0] = False
    assert any("terminal" in e for e in VR.check_record(bad))


def test_invalid_run_requires_reason_and_is_not_a_label():
    r = _rec([[0.4, 0.3]], invalid_reasons=["spawn_overlap"])
    assert r["evidence_class"] == "INVALID_RUN" and VR.check_record(r) == []
    r["invalid_reasons"] = []
    assert any("INVALID_RUN" in e for e in VR.check_record(r))


def test_hidden_metadata_is_detected_in_features_only():
    ok = {"example_id": "e", "history": [{"t_capture": 0, "obs": {"v": 12.0, "objs": [{"dx": 30.0}]}}],
          "applied_controls": [], "contract_ids": {"world_set_id": "w1"}, "label": {"V": 0.1, "case_id": "RC01"}}
    assert VR.hidden_leaks(ok) == []                       # ids/labels are allowed outside the features
    assert VR.hidden_leaks(dict(ok, history=[{"obs": {"v": 12.0, "world_mode": "yield"}}]))
    assert VR.hidden_leaks(dict(ok, history=[{"obs": {"blueprint": "vehicle.audi.tt"}}]))
