"""T6 (mock): occluded-actor family."""
import numpy as np
import pytest

import core
from core import MockBackend


@pytest.fixture(autouse=True)
def family():
    core.use_family("occluded")
    yield
    core.use_family("two_lane")


def _theta(**kw):
    th = core.decode(np.full(core.D, 0.5))
    th.update(kw)
    return th


def test_space_has_t_reveal():
    assert "t_reveal" in core.NAMES and core.D == 7


def test_observation_identical_before_reveal_across_worlds():
    class Rec(core.Policy):
        def reset(self): super().reset(); self.log = []
        def _act(self, obs): self.log.append((obs["t"], obs["v"], len(obs["objs"]))); return core.KEEP

    th = _theta(t_reveal=2.0, reaction_delay=0.0, obstacle_gap=100.0)
    logs = {}
    for w in core.WORLDS:
        p = Rec(); MockBackend().rollout(th, w, p); logs[w] = [l for l in p.log if l[0] < 2.0]
    assert logs["constant"] == logs["idm_reactive"] == logs["accelerating"]
    assert all(n == 1 for _, _, n in logs["constant"])           # only the obstacle is visible


def test_adjacent_visible_after_reveal():
    class Rec(core.Policy):
        def reset(self): super().reset(); self.n = []
        def _act(self, obs): self.n.append((obs["t"], len(obs["objs"]))); return core.KEEP

    p = Rec(); MockBackend().rollout(_theta(t_reveal=1.0), "constant", p)
    assert all(n == (1 if t < 1.0 else 2) for t, n in p.n)


def test_V_vs_t_reveal_on_sensitive_scenes():
    """Geometries (found by a 300-scene survey, tests/occluded_sensitive.json) where t_reveal changes V.
    Three show V decreasing with later reveal; one is a counter-example (hiding the faster car ahead makes a blind
    lane change succeed). The counter-example is documented in results/occluded_findings.md, not hidden."""
    import json, os
    cases = json.load(open(os.path.join(os.path.dirname(__file__), "occluded_sensitive.json")))
    be, dec, inc = MockBackend(), 0, 0
    for u, _ in cases:
        base = core.decode(np.array(u))
        Vs = [core.oracle(be, dict(base, t_reveal=float(t)))["V"] for t in (0, 1, 2, 3, 4)]
        if all(b <= a + 1e-9 for a, b in zip(Vs, Vs[1:])) and Vs[-1] < Vs[0] - 0.1:
            dec += 1
        elif Vs[-1] > Vs[0] + 0.1:
            inc += 1
    assert (dec, inc) == (3, 1)


def test_occlusion_rarely_changes_V_with_unconditional_brake_in_library():
    """Finding: emergency_brake needs no information, so it dominates V in most scenes."""
    rng = np.random.default_rng(1)
    be, changed = MockBackend(), 0
    for _ in range(60):
        b = core.decode(rng.random(core.D))
        Vs = {core.oracle(be, dict(b, t_reveal=float(t)))["V"] for t in (0, 4)}
        changed += len(Vs) > 1
    assert changed <= 6
