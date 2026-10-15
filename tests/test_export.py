"""T7: exported records pass the validator; the validator rejects corrupted ones."""
import copy

import numpy as np

import core
import export
import validate_dataset as vd


def _rec(tmp_path, u):
    rec, theta = export.make_record(core.MockBackend(), u, core.DEFAULT_CFG, "two_lane", 0)
    (tmp_path / "traces").mkdir(exist_ok=True)
    export.trace_npz(core.MockBackend(), theta, rec, tmp_path / rec["trace_path"])
    return rec


def test_exported_record_valid(tmp_path):
    rng = np.random.default_rng(0)
    for _ in range(10):
        assert vd.check(_rec(tmp_path, rng.random(core.D)), str(tmp_path)) == []


def test_validator_catches_corruption(tmp_path):
    rec = _rec(tmp_path, np.full(core.D, 0.3))
    bad = copy.deepcopy(rec); bad["V"] += 0.5
    assert any("max_j" in e for e in vd.check(bad, str(tmp_path)))
    bad = copy.deepcopy(rec); bad["category"] = "safe" if rec["category"] != "safe" else "unrecoverable"
    assert any("inconsistent" in e for e in vd.check(bad, str(tmp_path)))
    bad = copy.deepcopy(rec); bad["R"][next(iter(bad["R"]))] = float("nan")
    assert vd.check(bad, str(tmp_path))
