"""Milestone 3 tests: PD controller behaviour."""
import pytest

from src.controller import pd_control, PDController


def test_zero_error_zero_control():
    assert pd_control(0.0, 0.0, 0.0, kp=10, kd=1) == 0.0


def test_above_target_negative_control():
    # q above q_desired -> negative error -> negative torque (pulls down)
    u = pd_control(0.5, 0.0, 0.0, kp=10, kd=1)
    assert u == pytest.approx(-10 * 0.5)


def test_below_target_positive_control():
    u = pd_control(-0.5, 0.0, 0.0, kp=10, kd=1)
    assert u == pytest.approx(10 * 0.5)


def test_damping_reduces_control_for_positive_velocity():
    u_rest = pd_control(0.5, 0.0, 0.0, kp=10, kd=2)
    u_moving = pd_control(0.5, 1.0, 0.0, kp=10, kd=2)
    assert u_moving == pytest.approx(u_rest - 2.0)


def test_damping_increases_control_for_negative_velocity():
    u_rest = pd_control(0.5, 0.0, 0.0, kp=10, kd=2)
    u_moving = pd_control(0.5, -1.0, 0.0, kp=10, kd=2)
    assert u_moving == pytest.approx(u_rest + 2.0)


def test_kp_scaling_is_linear():
    u1 = pd_control(0.3, 0.0, 0.0, kp=5, kd=0)
    u2 = pd_control(0.3, 0.0, 0.0, kp=10, kd=0)
    assert u2 == pytest.approx(2 * u1)


def test_kd_damps_at_desired_position():
    # at q = qd a non-zero velocity must be actively damped
    u = pd_control(0.0, 2.0, 0.0, kp=10, kd=3)
    assert u == pytest.approx(-6.0)


def test_pd_controller_callable():
    ctrl = PDController(kp=10, kd=2, q_desired=0.0)
    assert ctrl(0.1, 0.0, 0.0) == pytest.approx(-1.0)
