"""
Pure Pursuit path tracking controller.

Pure pursuit is a geometric path-tracking algorithm.  The controller
selects a lookahead point on the reference path and computes the
angular velocity needed to steer toward it, while the linear velocity
is either constant or proportional to curvature.

References
----------
Coulter, R. C. (1992). Implementation of the Pure Pursuit Path Tracking
Algorithm.  Carnegie Mellon University Technical Report CMU-RI-TR-92-01.
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

from state_estimator.kinematics import RobotState, _wrap_angle

# Waypoint: (world_x, world_y)
Waypoint = Tuple[float, float]


class PurePursuitController:
    """Pure-pursuit path tracking controller.

    Parameters
    ----------
    lookahead_distance_m : float
        Distance ahead on the path to select the target point (m).
    linear_velocity_m_s : float
        Constant forward speed (m/s).
    max_angular_velocity : float
        Saturation limit for angular velocity (rad/s).
    goal_tolerance_m : float
        Distance from the final waypoint at which the path is declared
        complete (m).
    """

    def __init__(
        self,
        lookahead_distance_m: float = 0.5,
        linear_velocity_m_s: float = 0.3,
        max_angular_velocity: float = 1.0,
        goal_tolerance_m: float = 0.15,
    ) -> None:
        self.ld = lookahead_distance_m
        self.v = linear_velocity_m_s
        self.max_omega = max_angular_velocity
        self.goal_tol = goal_tolerance_m

        self._path: List[Waypoint] = []
        self._path_idx: int = 0
        self._finished: bool = True

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def set_path(self, waypoints: List[Waypoint]) -> None:
        """Assign a new reference path.  Resets internal state."""
        self._path = list(waypoints)
        self._path_idx = 0
        self._finished = len(waypoints) == 0

    @property
    def is_finished(self) -> bool:
        """True once the robot has reached the final waypoint."""
        return self._finished

    def compute_control(self, state: RobotState) -> Tuple[float, float]:
        """Return (v, ω) command for the current robot state.

        Returns
        -------
        v : float
            Linear velocity command (m/s).  0.0 if path is finished.
        omega : float
            Angular velocity command (rad/s).
        """
        if self._finished or not self._path:
            return 0.0, 0.0

        # Advance the nearest-point index so we never look backwards
        self._advance_index(state)

        # Check goal reached
        goal = self._path[-1]
        dist_to_goal = _dist(state.x, state.y, goal[0], goal[1])
        if dist_to_goal <= self.goal_tol:
            self._finished = True
            return 0.0, 0.0

        # Find the lookahead point
        lookahead = self._find_lookahead(state)
        if lookahead is None:
            # Robot is far off path; head toward the next waypoint directly
            lookahead = self._path[self._path_idx]

        # Compute angular velocity using the pure-pursuit formula:
        #   ω = 2 · v · sin(α) / L_d
        # where α is the angle to the lookahead in the robot frame.
        alpha = _wrap_angle(
            math.atan2(lookahead[1] - state.y, lookahead[0] - state.x) - state.theta
        )
        omega = 2.0 * self.v * math.sin(alpha) / self.ld
        omega = max(-self.max_omega, min(self.max_omega, omega))

        return self.v, omega

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _advance_index(self, state: RobotState) -> None:
        """Move _path_idx to the closest waypoint that is not behind the robot."""
        while self._path_idx < len(self._path) - 1:
            wp = self._path[self._path_idx]
            d = _dist(state.x, state.y, wp[0], wp[1])
            if d < self.ld * 0.5:
                self._path_idx += 1
            else:
                break

    def _find_lookahead(self, state: RobotState) -> Optional[Waypoint]:
        """Return the first waypoint on the path at distance ≥ lookahead."""
        for i in range(self._path_idx, len(self._path)):
            wp = self._path[i]
            d = _dist(state.x, state.y, wp[0], wp[1])
            if d >= self.ld:
                return wp
        # If no waypoint is far enough, return the last one
        return self._path[-1]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _dist(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
