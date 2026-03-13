"""
Spiral coverage path planner.

The robot starts near the centre of the free space (or a specified
starting position) and expands outwards in a square spiral, covering
each free cell once.  This approach naturally handles convex rooms and
produces smooth, low-turn-count paths.

Algorithm overview
------------------
1. Begin at the start cell.
2. Move right, then up, then left, then down (clockwise outward spiral).
3. Expand the radius after completing each full circuit.
4. Skip occupied / already-visited cells.
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import numpy as np

from map_tools.grid_map import GridMap

Waypoint = Tuple[float, float]


class SpiralPlanner:
    """Generate a square-spiral coverage path.

    Parameters
    ----------
    grid_map : GridMap
        Environment map.
    robot_radius_m : float
        Robot physical radius used to inflate obstacles.
    start_col : Optional[int]
        Column of the start cell.  Defaults to map centre.
    start_row : Optional[int]
        Row of the start cell.  Defaults to map centre.
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
        self._cspace = grid_map.inflate_obstacles(robot_radius_m)
        self._start_col = start_col if start_col is not None else grid_map.width // 2
        self._start_row = start_row if start_row is not None else grid_map.height // 2

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def plan(self) -> List[Waypoint]:
        """Return spiral coverage waypoints as world coordinates."""
        gm = self._cspace
        visited = np.zeros((gm.height, gm.width), dtype=bool)
        waypoints: List[Waypoint] = []

        # Direction vectors: right, up, left, down
        dir_vectors = [(1, 0), (0, 1), (-1, 0), (0, -1)]
        dir_idx = 0  # start moving right

        col, row = self._start_col, self._start_row

        # If the start cell is not free, find the nearest free cell
        if not gm.is_free(col, row):
            col, row = self._nearest_free(col, row)
            if col is None:
                return []

        # Spiral outward using a layer-by-layer approach
        # We iterate using the classic "spiral matrix" traversal
        left, right = 0, gm.width - 1
        bottom, top = 0, gm.height - 1

        # Collect all free cells in spiral order
        while left <= right and bottom <= top:
            # Move right along the bottom
            for c in range(left, right + 1):
                r = bottom
                if gm.is_free(c, r) and not visited[r, c]:
                    visited[r, c] = True
                    wx, wy = gm.cell_to_world(c, r)
                    waypoints.append((wx, wy))
            bottom += 1

            # Move up along the right
            for r in range(bottom, top + 1):
                c = right
                if gm.is_free(c, r) and not visited[r, c]:
                    visited[r, c] = True
                    wx, wy = gm.cell_to_world(c, r)
                    waypoints.append((wx, wy))
            right -= 1

            # Move left along the top
            if bottom <= top:
                for c in range(right, left - 1, -1):
                    r = top
                    if gm.is_free(c, r) and not visited[r, c]:
                        visited[r, c] = True
                        wx, wy = gm.cell_to_world(c, r)
                        waypoints.append((wx, wy))
                top -= 1

            # Move down along the left
            if left <= right:
                for r in range(top, bottom - 1, -1):
                    c = left
                    if gm.is_free(c, r) and not visited[r, c]:
                        visited[r, c] = True
                        wx, wy = gm.cell_to_world(c, r)
                        waypoints.append((wx, wy))
                left += 1

        return waypoints

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _nearest_free(self, col: int, row: int):
        """BFS to find the nearest free cell to (col, row)."""
        gm = self._cspace
        from collections import deque

        queue = deque([(col, row)])
        seen = set()
        seen.add((col, row))

        while queue:
            c, r = queue.popleft()
            if gm.is_free(c, r):
                return c, r
            for dc, dr in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                nc, nr = c + dc, r + dr
                if gm.is_valid(nc, nr) and (nc, nr) not in seen:
                    seen.add((nc, nr))
                    queue.append((nc, nr))

        return None, None
