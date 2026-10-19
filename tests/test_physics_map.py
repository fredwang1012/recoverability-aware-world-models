"""Mock-only tests for the decel -> wheel parameter mapping (pure function on a profile dict; no CARLA server)."""
import pytest

from sim.plant_carla import wheel_params

NEW = {"knob": "max_brake_torque", "friction_fixed": 5.0, "torque": [350.0, 700.0, 1400.0, 2800.0], "decel": [2.0, 4.0, 8.0, 12.0]}
OLD = {"friction": [0.6, 1.0, 2.0], "decel": [0.7, 2.9, 4.8], "max_brake_torque": 700.0}


def test_torque_interpolates_and_sets_fixed_friction():
    mu, t = wheel_params(NEW, 6.0)
    assert mu == 5.0 and t == pytest.approx(1050.0)
    assert wheel_params(NEW, 4.0)[1] == pytest.approx(700.0)


def test_torque_monotone_in_decel_and_clipped():
    ts = [wheel_params(NEW, d)[1] for d in (1.0, 2.0, 3.0, 5.0, 8.0, 10.0, 20.0)]
    assert ts == sorted(ts)
    assert ts[0] == 350.0 and ts[-1] == 2800.0


def test_old_profile_backward_compatible():
    mu, t = wheel_params(OLD, 2.9)
    assert mu == pytest.approx(1.0) and t == 700.0
