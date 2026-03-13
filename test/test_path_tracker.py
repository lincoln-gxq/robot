"""
Unit tests for the Pure Pursuit path tracker.
"""

import math
import sys
import os
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src", "state_estimator"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src", "path_tracker"))

from state_estimator.kinematics import RobotState, DifferentialDriveKinematics
from path_tracker.pure_pursuit import PurePursuitController


class TestPurePursuitController:
    def _make_controller(self, **kwargs):
        defaults = dict(
            lookahead_distance_m=0.5,
            linear_velocity_m_s=0.3,
            max_angular_velocity=1.0,
            goal_tolerance_m=0.15,
        )
        defaults.update(kwargs)
        return PurePursuitController(**defaults)

    def test_no_path_returns_zero(self):
        ctrl = self._make_controller()
        state = RobotState()
        v, omega = ctrl.compute_control(state)
        assert v == 0.0
        assert omega == 0.0

    def test_finished_after_set_empty_path(self):
        ctrl = self._make_controller()
        ctrl.set_path([])
        assert ctrl.is_finished

    def test_stops_at_goal(self):
        ctrl = self._make_controller(goal_tolerance_m=0.5)
        # Goal right at the robot
        ctrl.set_path([(0.1, 0.0)])
        state = RobotState(x=0.0, y=0.0)
        v, omega = ctrl.compute_control(state)
        assert ctrl.is_finished
        assert v == 0.0

    def test_linear_velocity_returned(self):
        ctrl = self._make_controller(linear_velocity_m_s=0.4)
        ctrl.set_path([(2.0, 0.0), (4.0, 0.0), (6.0, 0.0)])
        state = RobotState(x=0.0, y=0.0, theta=0.0)
        v, omega = ctrl.compute_control(state)
        assert v == pytest.approx(0.4)

    def test_omega_clamped(self):
        ctrl = self._make_controller(max_angular_velocity=0.5)
        # Target far to the side – should saturate omega
        ctrl.set_path([(0.0, 5.0), (0.0, 10.0)])
        state = RobotState(x=0.0, y=0.0, theta=0.0)
        v, omega = ctrl.compute_control(state)
        assert abs(omega) <= 0.5 + 1e-9

    def test_straight_path_low_omega(self):
        ctrl = self._make_controller()
        # Path straight ahead along +X
        ctrl.set_path([(1.0, 0.0), (2.0, 0.0), (3.0, 0.0)])
        state = RobotState(x=0.0, y=0.0, theta=0.0)
        v, omega = ctrl.compute_control(state)
        assert abs(omega) < 0.1  # nearly straight

    def test_set_path_resets_finished(self):
        ctrl = self._make_controller()
        ctrl.set_path([(0.1, 0.0)])
        state = RobotState()
        ctrl.compute_control(state)  # should finish
        assert ctrl.is_finished
        ctrl.set_path([(5.0, 0.0)])
        assert not ctrl.is_finished

    def test_controller_simulation(self):
        """End-to-end: robot should eventually reach the goal."""
        ctrl = self._make_controller(
            lookahead_distance_m=0.3,
            linear_velocity_m_s=0.5,
            goal_tolerance_m=0.2,
        )
        kin = DifferentialDriveKinematics(wheel_track_m=0.30)
        path = [(1.0, 0.0), (2.0, 0.0), (3.0, 0.0), (4.0, 0.0), (5.0, 0.0)]
        ctrl.set_path(path)
        state = RobotState()
        dt = 0.05
        max_steps = 500
        for _ in range(max_steps):
            if ctrl.is_finished:
                break
            v, omega = ctrl.compute_control(state)
            state = kin.step(state, v, omega, dt)
        assert ctrl.is_finished, "Controller did not reach goal within time limit"
