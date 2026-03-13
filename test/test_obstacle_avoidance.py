"""
Unit tests for the VFH obstacle avoidance controller.
"""

import math
import sys
import os
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src", "obstacle_avoidance"))

from obstacle_avoidance.vfh import VFHController


def _make_clear_scan(n=360, r=5.0):
    """Return a laser scan with all ranges at r (no obstacles)."""
    return [r] * n


def _make_blocked_front_scan(n=360, r=5.0, blocked_range=0.25):
    """Block the sector directly in front (angles near 0)."""
    scan = [r] * n
    # Block the first ~30 degrees (front)
    for i in range(15):
        scan[i] = blocked_range
        scan[n - 1 - i] = blocked_range
    return scan


class TestVFHController:
    def _make_ctrl(self, **kwargs):
        defaults = dict(
            num_sectors=36,
            threshold=3.0,
            robot_radius_m=0.2,
            safety_distance_m=0.3,
            max_scan_range_m=5.0,
        )
        defaults.update(kwargs)
        return VFHController(**defaults)

    def test_clear_path_returns_nonzero_velocity(self):
        ctrl = self._make_ctrl()
        scan = _make_clear_scan()
        v, omega = ctrl.compute_command(
            scan_ranges=scan,
            scan_angle_min=-math.pi,
            scan_angle_increment=2 * math.pi / len(scan),
            goal_heading=0.0,
            linear_velocity=0.3,
        )
        assert v > 0.0

    def test_all_blocked_returns_zero_velocity(self):
        ctrl = self._make_ctrl(threshold=0.1)
        scan = [0.1] * 360  # very close obstacles everywhere
        v, omega = ctrl.compute_command(
            scan_ranges=scan,
            scan_angle_min=-math.pi,
            scan_angle_increment=2 * math.pi / 360,
            goal_heading=0.0,
            linear_velocity=0.3,
        )
        assert v == 0.0
        assert omega == 0.0

    def test_blocked_front_causes_steering(self):
        ctrl = self._make_ctrl(num_sectors=72)
        scan = _make_blocked_front_scan(n=360, blocked_range=0.20)
        v, omega = ctrl.compute_command(
            scan_ranges=scan,
            scan_angle_min=-math.pi,
            scan_angle_increment=2 * math.pi / 360,
            goal_heading=0.0,
        )
        # The robot should steer away (non-zero omega or reduced speed)
        assert abs(omega) > 0.0 or v < 0.3

    def test_omega_within_pi(self):
        ctrl = self._make_ctrl()
        scan = _make_clear_scan()
        _, omega = ctrl.compute_command(
            scan_ranges=scan,
            scan_angle_min=-math.pi,
            scan_angle_increment=2 * math.pi / len(scan),
            goal_heading=math.pi / 4,
        )
        assert -math.pi <= omega <= math.pi

    def test_speed_reduced_near_obstacle(self):
        ctrl = self._make_ctrl()
        # Close obstacle directly ahead
        scan = [0.4] * 360
        v_close, _ = ctrl.compute_command(
            scan_ranges=scan,
            scan_angle_min=-math.pi,
            scan_angle_increment=2 * math.pi / 360,
            goal_heading=0.0,
            linear_velocity=0.5,
        )
        # Far obstacle
        scan_far = [5.0] * 360
        v_far, _ = ctrl.compute_command(
            scan_ranges=scan_far,
            scan_angle_min=-math.pi,
            scan_angle_increment=2 * math.pi / 360,
            goal_heading=0.0,
            linear_velocity=0.5,
        )
        assert v_close <= v_far

    def test_invalid_ranges_ignored(self):
        ctrl = self._make_ctrl()
        scan = [0.0] * 360  # all zero (invalid)
        v, omega = ctrl.compute_command(
            scan_ranges=scan,
            scan_angle_min=-math.pi,
            scan_angle_increment=2 * math.pi / 360,
            goal_heading=0.0,
            linear_velocity=0.3,
        )
        # Should not crash
        assert isinstance(v, float)
        assert isinstance(omega, float)
