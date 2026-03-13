"""
Vector Field Histogram (VFH) obstacle avoidance.

VFH builds a polar histogram of obstacle densities around the robot
using laser scan data and selects the best candidate direction that
avoids obstacles while staying as close as possible to the goal
heading.

References
----------
Borenstein, J. & Koren, Y. (1991). The vector field histogram – fast
obstacle avoidance for mobile robots.  IEEE Transactions on Robotics
and Automation, 7(3), 278–288.
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import numpy as np


class VFHController:
    """Vector Field Histogram local obstacle avoidance.

    Parameters
    ----------
    num_sectors : int
        Number of angular sectors (resolution of the polar histogram).
    threshold : float
        Obstacle density threshold.  Sectors with density above this
        value are considered blocked.
    robot_radius_m : float
        Robot radius (m), used to enlarge obstacle vectors.
    safety_distance_m : float
        Minimum clearance from obstacles (m).
    max_scan_range_m : float
        Maximum range of the laser scanner (m).
    """

    def __init__(
        self,
        num_sectors: int = 72,
        threshold: float = 3.0,
        robot_radius_m: float = 0.2,
        safety_distance_m: float = 0.3,
        max_scan_range_m: float = 5.0,
    ) -> None:
        self.num_sectors = num_sectors
        self.threshold = threshold
        self.robot_radius_m = robot_radius_m
        self.safety_distance_m = safety_distance_m
        self.max_scan_range_m = max_scan_range_m
        self.sector_angle = 2.0 * math.pi / num_sectors

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def compute_command(
        self,
        scan_ranges: List[float],
        scan_angle_min: float,
        scan_angle_increment: float,
        goal_heading: float,
        linear_velocity: float = 0.3,
    ) -> Tuple[float, float]:
        """Compute (v, ω) command using VFH.

        Parameters
        ----------
        scan_ranges : list of float
            Range measurements from the laser scanner (m).
        scan_angle_min : float
            Angle of the first scan beam (rad).
        scan_angle_increment : float
            Angular step between consecutive beams (rad).
        goal_heading : float
            Desired heading toward the goal (rad, robot frame).
        linear_velocity : float
            Desired forward speed (m/s).

        Returns
        -------
        v : float
            Linear velocity (m/s).  Reduced in tight spaces.
        omega : float
            Angular velocity (rad/s).
        """
        hist = self._build_histogram(scan_ranges, scan_angle_min, scan_angle_increment)
        best_dir = self._select_direction(hist, goal_heading)

        if best_dir is None:
            # All directions blocked – stop
            return 0.0, 0.0

        # Steer toward the best direction
        omega = _wrap_angle(best_dir)
        # Clamp omega
        omega = max(-math.pi, min(math.pi, omega))

        # Slow down proportionally to obstacle proximity
        min_range = min(
            (r for r in scan_ranges if r < self.max_scan_range_m),
            default=self.max_scan_range_m,
        )
        speed_factor = min(1.0, (min_range - self.safety_distance_m) / self.max_scan_range_m)
        speed_factor = max(0.0, speed_factor)
        v = linear_velocity * speed_factor

        return v, omega

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _build_histogram(
        self,
        scan_ranges: List[float],
        angle_min: float,
        angle_inc: float,
    ) -> np.ndarray:
        """Build the polar obstacle density histogram."""
        hist = np.zeros(self.num_sectors, dtype=float)

        for i, r in enumerate(scan_ranges):
            if r <= 0.0 or r > self.max_scan_range_m:
                continue

            beam_angle = angle_min + i * angle_inc
            # Obstacle certainty value: closer → higher density
            certainty = (self.max_scan_range_m - r) / self.max_scan_range_m

            # Sector index
            sector = int(((beam_angle % (2 * math.pi)) / (2 * math.pi)) * self.num_sectors)
            sector = sector % self.num_sectors
            hist[sector] += certainty * certainty

        return hist

    def _select_direction(
        self, hist: np.ndarray, goal_heading: float
    ) -> Optional[float]:
        """Choose the steering angle closest to goal_heading that is unblocked."""
        goal_sector = int(
            ((goal_heading % (2 * math.pi)) / (2 * math.pi)) * self.num_sectors
        ) % self.num_sectors

        # Try sectors in order of angular distance from goal
        indices = sorted(range(self.num_sectors), key=lambda s: _sector_dist(s, goal_sector, self.num_sectors))

        for s in indices:
            if hist[s] < self.threshold:
                return (s + 0.5) * self.sector_angle

        return None  # all blocked


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _sector_dist(s1: int, s2: int, n: int) -> int:
    d = abs(s1 - s2)
    return min(d, n - d)
