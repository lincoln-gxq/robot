"""
Boustrophedon (lawnmower / zigzag) coverage path planner.

The planner decomposes the free area of a GridMap into horizontal
strip rows and generates a back-and-forth waypoint path that covers
every free cell.

References
----------
Choset, H. (2001). Coverage for robotics – A survey of recent results.
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import numpy as np

from map_tools.grid_map import GridMap

# Waypoint: (world_x, world_y)
Waypoint = Tuple[float, float]


class BoustrophedonPlanner:
    """Generate a boustrophedon (zigzag) coverage path.

    Parameters
    ----------
    grid_map : GridMap
        The environment map.  Obstacle-free cells are covered.
    robot_radius_m : float
        Physical radius of the robot.  Used to inflate obstacles and to
        set the strip width so that adjacent passes share the robot
        footprint.
    start_col : int | None
        Starting column (grid index).  Defaults to the left-most free
        column.
    start_row : int | None
        Starting row (grid index).  Defaults to the bottom-most free row.
    """

    def __init__(
        self,
        grid_map: GridMap,
        robot_radius_m: float = 0.2,
        start_col: Optional[int] = None,
        start_row: Optional[int] = None,
    ) -> None:
        self.grid_map = grid_map
        self.robot_radius_m = robot_radius_m

        # Inflate obstacles by the robot radius to get the configuration-space map
        self._cspace = grid_map.inflate_obstacles(robot_radius_m)

        # Strip width: one robot diameter ensures full coverage without gaps
        self._strip_width = max(
            1, int(math.ceil(2 * robot_radius_m / grid_map.resolution))
        )

        # Determine start cell
        free_cols, free_rows = self._free_indices()
        if len(free_cols) == 0:
            self._start_col = 0
            self._start_row = 0
        else:
            self._start_col = start_col if start_col is not None else int(free_cols[0])
            self._start_row = start_row if start_row is not None else int(free_rows[0])

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def plan(self) -> List[Waypoint]:
        """Return an ordered list of (x, y) world-coordinate waypoints.

        The path visits every free cell in the configuration-space map
        using a boustrophedon scan.
        """
        waypoints: List[Waypoint] = []
        gm = self._cspace
        visited = np.zeros((gm.height, gm.width), dtype=bool)

        # Build row groups (strips)
        row = self._start_row
        direction_right = True  # current sweep direction

        while row < gm.height:
            row_waypoints: List[Waypoint] = []

            col_range = range(gm.width) if direction_right else range(gm.width - 1, -1, -1)

            for col in col_range:
                if gm.is_free(col, row) and not visited[row, col]:
                    wx, wy = gm.cell_to_world(col, row)
                    row_waypoints.append((wx, wy))
                    visited[row, col] = True

            if row_waypoints:
                waypoints.extend(row_waypoints)
                direction_right = not direction_right  # flip for next strip

            row += self._strip_width

        return waypoints

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _free_indices(self):
        free_mask = self._cspace.data == GridMap.FREE
        rows_idx, cols_idx = np.where(free_mask)
        if len(rows_idx) == 0:
            return np.array([], dtype=int), np.array([], dtype=int)
        # Sort by (row, col) ascending
        order = np.lexsort((cols_idx, rows_idx))
        return cols_idx[order], rows_idx[order]


# ---------------------------------------------------------------------------
# Performance metrics
# ---------------------------------------------------------------------------


def compute_metrics(
    waypoints: List[Waypoint],
    grid_map: GridMap,
    robot_radius_m: float = 0.2,
) -> dict:
    """Compute standard coverage performance metrics.

    Parameters
    ----------
    waypoints : list of (x, y)
        The executed path.
    grid_map : GridMap
        The reference map (without inflation).
    robot_radius_m : float
        Robot cleaning radius.

    Returns
    -------
    dict with keys:
        coverage_rate, repetition_rate, miss_rate,
        path_length_m, total_free_cells, covered_cells,
        repeated_cells, missed_cells
    """
    if not waypoints:
        return {
            "coverage_rate": 0.0,
            "repetition_rate": 0.0,
            "miss_rate": 1.0,
            "path_length_m": 0.0,
            "total_free_cells": grid_map.free_cells,
            "covered_cells": 0,
            "repeated_cells": 0,
            "missed_cells": grid_map.free_cells,
        }

    coverage_count = np.zeros((grid_map.height, grid_map.width), dtype=int)
    radius_cells = max(1, int(math.ceil(robot_radius_m / grid_map.resolution)))

    for wx, wy in waypoints:
        col, row = grid_map.world_to_cell(wx, wy)
        r_min = max(0, row - radius_cells)
        r_max = min(grid_map.height, row + radius_cells + 1)
        c_min = max(0, col - radius_cells)
        c_max = min(grid_map.width, col + radius_cells + 1)
        for r in range(r_min, r_max):
            for c in range(c_min, c_max):
                if grid_map.is_free(c, r):
                    coverage_count[r, c] += 1

    free_mask = grid_map.data == GridMap.FREE
    total_free = int(np.sum(free_mask))
    covered_mask = coverage_count > 0
    covered = int(np.sum(covered_mask & free_mask))
    repeated = int(np.sum((coverage_count > 1) & free_mask))
    missed = total_free - covered

    # Path length
    path_length = 0.0
    for i in range(1, len(waypoints)):
        dx = waypoints[i][0] - waypoints[i - 1][0]
        dy = waypoints[i][1] - waypoints[i - 1][1]
        path_length += math.sqrt(dx * dx + dy * dy)

    coverage_rate = covered / total_free if total_free > 0 else 0.0
    repetition_rate = repeated / covered if covered > 0 else 0.0
    miss_rate = missed / total_free if total_free > 0 else 0.0

    return {
        "coverage_rate": coverage_rate,
        "repetition_rate": repetition_rate,
        "miss_rate": miss_rate,
        "path_length_m": path_length,
        "total_free_cells": total_free,
        "covered_cells": covered,
        "repeated_cells": repeated,
        "missed_cells": missed,
    }
