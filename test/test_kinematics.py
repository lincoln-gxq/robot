"""
Unit tests for differential-drive kinematics.
"""

import math
import sys
import os
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src", "state_estimator"))

from state_estimator.kinematics import (
    DifferentialDriveKinematics,
    RobotState,
    _wrap_angle,
)


class TestWrapAngle:
    def test_zero(self):
        assert _wrap_angle(0.0) == pytest.approx(0.0)

    def test_pi(self):
        assert abs(_wrap_angle(math.pi)) <= math.pi

    def test_wrap_positive(self):
        assert _wrap_angle(3 * math.pi) == pytest.approx(math.pi, abs=1e-9)

    def test_wrap_negative(self):
        # -3π wraps to ±π (both are equivalent representations of the same angle)
        assert abs(_wrap_angle(-3 * math.pi)) == pytest.approx(math.pi, abs=1e-9)


class TestWheelSpeedConversion:
    def setup_method(self):
        self.kin = DifferentialDriveKinematics(wheel_track_m=0.30, wheel_radius_m=0.05)

    def test_straight_motion(self):
        v, omega = self.kin.wheel_speeds_to_body(1.0, 1.0)
        assert v == pytest.approx(1.0)
        assert omega == pytest.approx(0.0)

    def test_left_turn(self):
        # Right wheel faster → turn left (positive omega)
        v, omega = self.kin.wheel_speeds_to_body(0.0, 1.0)
        assert omega > 0

    def test_right_turn(self):
        # Left wheel faster → turn right (negative omega)
        v, omega = self.kin.wheel_speeds_to_body(1.0, 0.0)
        assert omega < 0

    def test_spin_in_place(self):
        v, omega = self.kin.wheel_speeds_to_body(-0.5, 0.5)
        assert v == pytest.approx(0.0)
        assert omega > 0

    def test_round_trip(self):
        v_orig, omega_orig = 0.4, 0.3
        vl, vr = self.kin.body_to_wheel_speeds(v_orig, omega_orig)
        v_back, omega_back = self.kin.wheel_speeds_to_body(vl, vr)
        assert v_back == pytest.approx(v_orig)
        assert omega_back == pytest.approx(omega_orig)


class TestKinematicsStep:
    def setup_method(self):
        self.kin = DifferentialDriveKinematics(wheel_track_m=0.30)

    def test_straight_line(self):
        state = RobotState(x=0.0, y=0.0, theta=0.0)
        new_state = self.kin.step(state, v=1.0, omega=0.0, dt=1.0)
        assert new_state.x == pytest.approx(1.0, abs=1e-9)
        assert new_state.y == pytest.approx(0.0, abs=1e-9)
        assert new_state.theta == pytest.approx(0.0, abs=1e-9)

    def test_full_circle(self):
        """Robot driving in a circle should return close to origin."""
        state = RobotState()
        v, omega = 1.0, 1.0
        T = 2 * math.pi / omega  # period of one full circle
        dt = 0.001
        steps = int(T / dt)
        for _ in range(steps):
            state = self.kin.step(state, v, omega, dt)
        assert abs(state.x) < 0.05
        assert abs(state.y) < 0.05

    def test_heading_update(self):
        state = RobotState(theta=0.0)
        new_state = self.kin.step(state, v=0.0, omega=math.pi / 2, dt=1.0)
        assert new_state.theta == pytest.approx(math.pi / 2, abs=1e-9)

    def test_step_from_wheels(self):
        state = RobotState()
        new_state = self.kin.step_from_wheels(state, 0.5, 0.5, dt=1.0)
        assert new_state.x == pytest.approx(0.5, abs=1e-9)
        assert new_state.y == pytest.approx(0.0, abs=1e-9)

    def test_multiple_steps_accumulate(self):
        state = RobotState()
        for _ in range(10):
            state = self.kin.step(state, v=1.0, omega=0.0, dt=0.1)
        assert state.x == pytest.approx(1.0, abs=1e-6)
