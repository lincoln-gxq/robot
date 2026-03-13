"""
Unit tests for coverage path planning algorithms.
"""

import math
import sys
import os
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src", "map_tools"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src", "coverage_planner"))

from map_tools.grid_map import create_empty_room, create_room_with_obstacles
from coverage_planner.algorithms.boustrophedon import (
    BoustrophedonPlanner,
    compute_metrics,
)
from coverage_planner.algorithms.spiral import SpiralPlanner


class TestBoustrophedonPlanner:
    def _make_small_room(self):
        return create_empty_room(3.0, 3.0, resolution=0.1, wall_thickness_m=0.1)

    def test_returns_waypoints(self):
        gm = self._make_small_room()
        planner = BoustrophedonPlanner(gm, robot_radius_m=0.15)
        wps = planner.plan()
        assert len(wps) > 0

    def test_waypoints_are_tuples(self):
        gm = self._make_small_room()
        wps = BoustrophedonPlanner(gm, robot_radius_m=0.15).plan()
        for wp in wps:
            assert len(wp) == 2
            assert isinstance(wp[0], float)
            assert isinstance(wp[1], float)

    def test_high_coverage_rate(self):
        gm = create_empty_room(5.0, 5.0, resolution=0.1, wall_thickness_m=0.1)
        planner = BoustrophedonPlanner(gm, robot_radius_m=0.15)
        wps = planner.plan()
        metrics = compute_metrics(wps, gm, robot_radius_m=0.15)
        # Expect at least 85% coverage for boustrophedon in open room
        assert metrics["coverage_rate"] >= 0.85, (
            f"Coverage rate too low: {metrics['coverage_rate']:.2%}"
        )

    def test_metrics_keys(self):
        gm = self._make_small_room()
        wps = BoustrophedonPlanner(gm, robot_radius_m=0.15).plan()
        metrics = compute_metrics(wps, gm)
        expected_keys = {
            "coverage_rate",
            "repetition_rate",
            "miss_rate",
            "path_length_m",
            "total_free_cells",
            "covered_cells",
            "repeated_cells",
            "missed_cells",
        }
        assert expected_keys == set(metrics.keys())

    def test_coverage_rate_plus_miss_rate(self):
        gm = self._make_small_room()
        wps = BoustrophedonPlanner(gm, robot_radius_m=0.15).plan()
        m = compute_metrics(wps, gm, robot_radius_m=0.15)
        # coverage_rate + miss_rate should equal 1.0
        assert abs(m["coverage_rate"] + m["miss_rate"] - 1.0) < 1e-6

    def test_empty_waypoints_metrics(self):
        gm = self._make_small_room()
        m = compute_metrics([], gm)
        assert m["coverage_rate"] == 0.0
        assert m["path_length_m"] == 0.0

    def test_path_length_positive(self):
        gm = create_empty_room(4.0, 4.0, resolution=0.1, wall_thickness_m=0.1)
        wps = BoustrophedonPlanner(gm, robot_radius_m=0.15).plan()
        m = compute_metrics(wps, gm)
        assert m["path_length_m"] > 0.0


class TestSpiralPlanner:
    def _make_room(self):
        return create_empty_room(4.0, 4.0, resolution=0.1, wall_thickness_m=0.1)

    def test_returns_waypoints(self):
        gm = self._make_room()
        wps = SpiralPlanner(gm, robot_radius_m=0.15).plan()
        assert len(wps) > 0

    def test_waypoints_are_tuples(self):
        gm = self._make_room()
        wps = SpiralPlanner(gm, robot_radius_m=0.15).plan()
        for wp in wps:
            assert len(wp) == 2

    def test_coverage_rate_reasonable(self):
        gm = self._make_room()
        wps = SpiralPlanner(gm, robot_radius_m=0.15).plan()
        m = compute_metrics(wps, gm, robot_radius_m=0.15)
        # Spiral should cover a reasonable fraction of the room
        assert m["coverage_rate"] >= 0.70, (
            f"Spiral coverage rate too low: {m['coverage_rate']:.2%}"
        )

    def test_all_waypoints_within_bounds(self):
        gm = self._make_room()
        wps = SpiralPlanner(gm, robot_radius_m=0.15).plan()
        for wx, wy in wps:
            col, row = gm.world_to_cell(wx, wy)
            assert gm.is_valid(col, row), f"Waypoint ({wx},{wy}) out of bounds"

    def test_room_with_obstacles(self):
        obstacles = [{"x": 2.0, "y": 2.0, "w": 0.5, "h": 0.5}]
        gm = create_room_with_obstacles(4.0, 4.0, resolution=0.1, obstacles=obstacles)
        planner = SpiralPlanner(gm, robot_radius_m=0.15)
        wps = planner.plan()
        assert len(wps) > 0
