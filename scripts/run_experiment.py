#!/usr/bin/env python3
"""
run_experiment.py – Standalone simulation and evaluation script.

Runs the sweeping robot coverage algorithms on a set of test environments
and reports performance metrics (coverage rate, repetition rate, miss rate,
path length, estimated task time).

Usage:
    python scripts/run_experiment.py
    python scripts/run_experiment.py --scenario all
    python scripts/run_experiment.py --scenario empty_room --algorithm boustrophedon

No ROS2 installation is required to run this script.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from typing import Dict, List, Tuple

# Add src packages to path
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_REPO_ROOT, "src", "map_tools"))
sys.path.insert(0, os.path.join(_REPO_ROOT, "src", "coverage_planner"))
sys.path.insert(0, os.path.join(_REPO_ROOT, "src", "path_tracker"))
sys.path.insert(0, os.path.join(_REPO_ROOT, "src", "state_estimator"))

from map_tools.grid_map import (
    GridMap,
    create_empty_room,
    create_multi_room,
    create_room_with_obstacles,
)
from coverage_planner.algorithms.boustrophedon import (
    BoustrophedonPlanner,
    compute_metrics,
)
from coverage_planner.algorithms.spiral import SpiralPlanner
from path_tracker.pure_pursuit import PurePursuitController
from state_estimator.kinematics import DifferentialDriveKinematics, RobotState

# ---------------------------------------------------------------------------
# Scenario definitions
# ---------------------------------------------------------------------------

SCENARIOS: Dict[str, GridMap] = {}


def _build_scenarios() -> None:
    SCENARIOS["empty_room"] = create_empty_room(
        10.0, 10.0, resolution=0.05, wall_thickness_m=0.2
    )
    SCENARIOS["room_with_obstacles"] = create_room_with_obstacles(
        10.0,
        10.0,
        resolution=0.05,
        wall_thickness_m=0.2,
        obstacles=[
            {"x": 3.0, "y": 3.0, "w": 1.0, "h": 1.0},
            {"x": 7.0, "y": 7.0, "w": 0.8, "h": 0.8},
            {"x": 5.0, "y": 5.0, "w": 1.5, "h": 0.4},
            {"x": 2.0, "y": 7.5, "w": 0.8, "h": 0.8},
            {"x": 8.0, "y": 3.0, "w": 0.6, "h": 0.6},
        ],
    )
    SCENARIOS["multi_room"] = create_multi_room(
        15.0, 10.0, resolution=0.05, wall_thickness_m=0.2
    )
    # Narrow corridor: tall thin room
    SCENARIOS["narrow_corridor"] = create_empty_room(
        2.0, 12.0, resolution=0.05, wall_thickness_m=0.1
    )


# ---------------------------------------------------------------------------
# Simulation helpers
# ---------------------------------------------------------------------------


def simulate_path_execution(
    waypoints: List[Tuple[float, float]],
    linear_velocity: float = 0.3,
    control_dt: float = 0.05,
    max_steps: int = 100_000,
) -> Tuple[float, List[Tuple[float, float]]]:
    """Simulate the robot executing the coverage path.

    Returns (elapsed_time_s, executed_positions).
    """
    if not waypoints:
        return 0.0, []

    ctrl = PurePursuitController(
        lookahead_distance_m=0.4,
        linear_velocity_m_s=linear_velocity,
        max_angular_velocity=1.5,
        goal_tolerance_m=0.15,
    )
    ctrl.set_path(waypoints)
    kin = DifferentialDriveKinematics(wheel_track_m=0.30, wheel_radius_m=0.05)
    state = RobotState(x=waypoints[0][0], y=waypoints[0][1], theta=0.0)

    elapsed = 0.0
    positions = [(state.x, state.y)]

    for _ in range(max_steps):
        if ctrl.is_finished:
            break
        v, omega = ctrl.compute_control(state)
        state = kin.step(state, v, omega, control_dt)
        elapsed += control_dt
        positions.append((state.x, state.y))

    return elapsed, positions


# ---------------------------------------------------------------------------
# Experiment runner
# ---------------------------------------------------------------------------


def run_single_experiment(
    scenario_name: str,
    algorithm: str,
    robot_radius_m: float = 0.20,
    linear_velocity: float = 0.30,
    verbose: bool = True,
) -> dict:
    gm = SCENARIOS[scenario_name]

    t0 = time.perf_counter()
    if algorithm == "spiral":
        planner = SpiralPlanner(gm, robot_radius_m=robot_radius_m)
    else:
        planner = BoustrophedonPlanner(gm, robot_radius_m=robot_radius_m)

    waypoints = planner.plan()
    plan_time = time.perf_counter() - t0

    # Compute planning-based metrics
    metrics = compute_metrics(waypoints, gm, robot_radius_m=robot_radius_m)

    # Simulate execution to get task time
    task_time, _ = simulate_path_execution(
        waypoints, linear_velocity=linear_velocity
    )

    result = {
        "scenario": scenario_name,
        "algorithm": algorithm,
        "waypoints": len(waypoints),
        "plan_time_s": round(plan_time, 4),
        "task_time_s": round(task_time, 2),
        **{k: round(v, 4) if isinstance(v, float) else v for k, v in metrics.items()},
    }

    if verbose:
        _print_result(result)

    return result


def _print_result(r: dict) -> None:
    print(
        f"\n{'─'*60}\n"
        f"  Scenario : {r['scenario']}\n"
        f"  Algorithm: {r['algorithm']}\n"
        f"  Waypoints: {r['waypoints']}\n"
        f"  Coverage rate    : {r['coverage_rate']*100:.1f}%\n"
        f"  Repetition rate  : {r['repetition_rate']*100:.1f}%\n"
        f"  Miss rate        : {r['miss_rate']*100:.1f}%\n"
        f"  Path length      : {r['path_length_m']:.2f} m\n"
        f"  Plan time        : {r['plan_time_s']*1000:.1f} ms\n"
        f"  Task time (sim)  : {r['task_time_s']:.1f} s\n"
        f"  Free cells       : {r['total_free_cells']}\n"
        f"  Covered cells    : {r['covered_cells']}\n"
        f"  Missed cells     : {r['missed_cells']}\n"
    )


def run_all_experiments(
    scenarios=None,
    algorithms=None,
    robot_radius_m=0.20,
    linear_velocity=0.30,
) -> List[dict]:
    if scenarios is None:
        scenarios = list(SCENARIOS.keys())
    if algorithms is None:
        algorithms = ["boustrophedon", "spiral"]

    results = []
    for scenario in scenarios:
        for algo in algorithms:
            r = run_single_experiment(
                scenario, algo, robot_radius_m, linear_velocity
            )
            results.append(r)

    _print_summary(results)
    return results


def _print_summary(results: List[dict]) -> None:
    print(f"\n{'═'*70}")
    print(f"  {'Scenario':<22} {'Algorithm':<16} {'Coverage':>9} {'Miss':>7} {'Length':>10} {'Time':>8}")
    print(f"{'─'*70}")
    for r in results:
        print(
            f"  {r['scenario']:<22} {r['algorithm']:<16} "
            f"{r['coverage_rate']*100:>8.1f}% {r['miss_rate']*100:>6.1f}% "
            f"{r['path_length_m']:>9.1f}m {r['task_time_s']:>7.0f}s"
        )
    print(f"{'═'*70}\n")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main() -> None:
    _build_scenarios()

    parser = argparse.ArgumentParser(
        description="Sweeping robot coverage path planning experiment runner."
    )
    parser.add_argument(
        "--scenario",
        choices=["all"] + list(SCENARIOS.keys()),
        default="all",
        help="Test scenario to run (default: all)",
    )
    parser.add_argument(
        "--algorithm",
        choices=["all", "boustrophedon", "spiral"],
        default="all",
        help="Coverage algorithm (default: all)",
    )
    parser.add_argument(
        "--robot-radius",
        type=float,
        default=0.20,
        metavar="M",
        help="Robot radius in metres (default: 0.20)",
    )
    parser.add_argument(
        "--velocity",
        type=float,
        default=0.30,
        metavar="M/S",
        help="Robot forward speed in m/s (default: 0.30)",
    )
    args = parser.parse_args()

    scenarios = list(SCENARIOS.keys()) if args.scenario == "all" else [args.scenario]
    algorithms = (
        ["boustrophedon", "spiral"] if args.algorithm == "all" else [args.algorithm]
    )

    print(
        f"\n{'═'*70}\n"
        f"  Sweeping Robot Coverage Path Planning – Experiment Results\n"
        f"  Robot radius: {args.robot_radius} m  |  Speed: {args.velocity} m/s\n"
        f"{'═'*70}"
    )

    run_all_experiments(
        scenarios=scenarios,
        algorithms=algorithms,
        robot_radius_m=args.robot_radius,
        linear_velocity=args.velocity,
    )


if __name__ == "__main__":
    main()
