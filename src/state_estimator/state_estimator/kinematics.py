"""
Differential-drive robot kinematics.

Implements the standard unicycle model:

    ẋ  = v · cos(θ)
    ẏ  = v · sin(θ)
    θ̇  = ω

with wheel-speed to body-velocity conversion:

    v = (v_r + v_l) / 2
    ω = (v_r - v_l) / L

where ``L`` is the wheel track (distance between left and right wheels).

The ``DifferentialDriveKinematics`` class performs numerical integration
of the state (x, y, θ) given either (v, ω) or (v_l, v_r) commands.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Tuple


@dataclass
class RobotState:
    """2-D pose and velocities of a differential-drive robot."""

    x: float = 0.0         # World X position (m)
    y: float = 0.0         # World Y position (m)
    theta: float = 0.0     # Heading (rad), measured from +X axis

    vx: float = 0.0        # Linear velocity (m/s)
    vy: float = 0.0        # (always 0 for diff-drive)
    omega: float = 0.0     # Angular velocity (rad/s)

    def __repr__(self) -> str:  # pragma: no cover
        return (
            f"RobotState(x={self.x:.3f}, y={self.y:.3f}, "
            f"theta={math.degrees(self.theta):.1f}°, "
            f"v={self.vx:.3f} m/s, ω={math.degrees(self.omega):.1f}°/s)"
        )


class DifferentialDriveKinematics:
    """Forward kinematics for a differential-drive robot.

    Parameters
    ----------
    wheel_track_m : float
        Distance between left and right wheels (metres).
    wheel_radius_m : float
        Radius of each wheel (metres).  Used only when converting
        angular wheel velocities to linear speeds.
    """

    def __init__(
        self,
        wheel_track_m: float = 0.4,
        wheel_radius_m: float = 0.1,
    ) -> None:
        self.L = wheel_track_m
        self.R = wheel_radius_m

    # ------------------------------------------------------------------
    # Wheel-speed to body-velocity conversions
    # ------------------------------------------------------------------

    def wheel_speeds_to_body(
        self, v_left: float, v_right: float
    ) -> Tuple[float, float]:
        """Convert left/right wheel linear speeds to (v, ω).

        Parameters
        ----------
        v_left, v_right : float
            Linear speed of the left and right wheels (m/s).

        Returns
        -------
        v : float
            Forward linear velocity (m/s).
        omega : float
            Angular velocity (rad/s).
        """
        v = (v_right + v_left) / 2.0
        omega = (v_right - v_left) / self.L
        return v, omega

    def body_to_wheel_speeds(self, v: float, omega: float) -> Tuple[float, float]:
        """Convert (v, ω) to left/right wheel linear speeds."""
        v_left = v - omega * self.L / 2.0
        v_right = v + omega * self.L / 2.0
        return v_left, v_right

    # ------------------------------------------------------------------
    # State integration
    # ------------------------------------------------------------------

    def step(
        self,
        state: RobotState,
        v: float,
        omega: float,
        dt: float,
    ) -> RobotState:
        """Integrate the unicycle model over one time step.

        Uses the exact arc-length integration for numerical accuracy:

            Δθ = ω · dt
            if |ω| ≈ 0: straight-line motion
            else:
                Δx = (v / ω) · (sin(θ + Δθ) - sin(θ))
                Δy = (v / ω) · (-cos(θ + Δθ) + cos(θ))

        Parameters
        ----------
        state : RobotState
            Current robot state.
        v : float
            Commanded linear velocity (m/s).
        omega : float
            Commanded angular velocity (rad/s).
        dt : float
            Time step (seconds).

        Returns
        -------
        RobotState
            Updated robot state after ``dt`` seconds.
        """
        theta = state.theta
        dtheta = omega * dt

        if abs(omega) < 1e-9:
            # Straight-line approximation
            dx = v * math.cos(theta) * dt
            dy = v * math.sin(theta) * dt
        else:
            r = v / omega  # turning radius
            dx = r * (math.sin(theta + dtheta) - math.sin(theta))
            dy = r * (-math.cos(theta + dtheta) + math.cos(theta))

        new_theta = _wrap_angle(theta + dtheta)

        return RobotState(
            x=state.x + dx,
            y=state.y + dy,
            theta=new_theta,
            vx=v * math.cos(new_theta),
            vy=v * math.sin(new_theta),
            omega=omega,
        )

    def step_from_wheels(
        self,
        state: RobotState,
        v_left: float,
        v_right: float,
        dt: float,
    ) -> RobotState:
        """Integrate the model from individual wheel speeds."""
        v, omega = self.wheel_speeds_to_body(v_left, v_right)
        return self.step(state, v, omega, dt)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _wrap_angle(angle: float) -> float:
    """Wrap angle to [-π, π]."""
    return math.atan2(math.sin(angle), math.cos(angle))
