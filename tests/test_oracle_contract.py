import os, sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "mining"))
import core


def test_full_oracle_matches_contract_and_pruned_V():
    b = core.MockBackend(); rng = np.random.default_rng(3)
    for u in rng.random((25, core.D)):
        th = core.decode(u)
        fast, full = core.oracle(b, th), core.oracle(b, th, full=True)
        assert abs(fast["V"] - full["V"]) < 1e-9                       # exact pruning never changes V
        R = np.array([[np.nan if x is None else x for x in full["R"][p]] for p in full["R"]])
        assert abs(np.nanmax(R.min(axis=1)) - full["V"]) < 1e-9
        assert full["V"] <= full["V_hind"] + 1e-9                       # robust <= hindsight
        assert (full["evidence"] == "R_REF") == (full["V"] >= core.EPS)
        if full["evidence"] == "U_INFORMATION":
            assert (R.max(axis=0) >= core.EPS).all()
