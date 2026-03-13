"""
GridMap: 2-D occupancy grid used throughout the sweeping-robot system.

Cells are stored as a flat NumPy uint8 array that follows the ROS
OccupancyGrid convention:
  0   – free
  100 – occupied
  255 – unknown

The class is intentionally kept free of any ROS2 dependency so that it
can be unit-tested without a ROS2 installation.
"""

from __future__ import annotations

import csv
import json
import math
import os
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import numpy as np


class GridMap:
    """Axis-aligned 2-D occupancy grid.

    Parameters
    ----------
    width_cells : int
        Number of columns.
    height_cells : int
        Number of rows.
    resolution : float
        Side length of one cell in metres (default 0.05 m = 5 cm).
    origin_x : float
        X coordinate (metres) of the lower-left corner of cell (0, 0).
    origin_y : float
        Y coordinate (metres) of the lower-left corner of cell (0, 0).
    """

    FREE = 0
    OCCUPIED = 100
    UNKNOWN = 255

    def __init__(
        self,
        width_cells: int,
        height_cells: int,
        resolution: float = 0.05,
        origin_x: float = 0.0,
        origin_y: float = 0.0,
    ) -> None:
        self.width = width_cells
        self.height = height_cells
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        # Row-major: data[row, col]
        self.data: np.ndarray = np.full(
            (height_cells, width_cells), self.FREE, dtype=np.uint8
        )

    # ------------------------------------------------------------------
    # Coordinate conversions
    # ------------------------------------------------------------------

    def world_to_cell(self, wx: float, wy: float) -> Tuple[int, int]:
        """Convert world coordinates (m) to (col, row) grid indices."""
        col = int(math.floor((wx - self.origin_x) / self.resolution))
        row = int(math.floor((wy - self.origin_y) / self.resolution))
        return col, row

    def cell_to_world(self, col: int, row: int) -> Tuple[float, float]:
        """Convert (col, row) grid indices to world coordinates (centre of cell)."""
        wx = self.origin_x + (col + 0.5) * self.resolution
        wy = self.origin_y + (row + 0.5) * self.resolution
        return wx, wy

    def is_valid(self, col: int, row: int) -> bool:
        """Return True if (col, row) is inside the grid."""
        return 0 <= col < self.width and 0 <= row < self.height

    # ------------------------------------------------------------------
    # Cell access
    # ------------------------------------------------------------------

    def get_cell(self, col: int, row: int) -> int:
        if not self.is_valid(col, row):
            raise IndexError(f"Cell ({col}, {row}) is out of bounds.")
        return int(self.data[row, col])

    def set_cell(self, col: int, row: int, value: int) -> None:
        if not self.is_valid(col, row):
            raise IndexError(f"Cell ({col}, {row}) is out of bounds.")
        self.data[row, col] = value

    def is_free(self, col: int, row: int) -> bool:
        return self.is_valid(col, row) and self.data[row, col] == self.FREE

    def is_occupied(self, col: int, row: int) -> bool:
        return self.is_valid(col, row) and self.data[row, col] == self.OCCUPIED

    # ------------------------------------------------------------------
    # Inflation / dilation for obstacle safety margin
    # ------------------------------------------------------------------

    def inflate_obstacles(self, radius_m: float) -> "GridMap":
        """Return a *new* GridMap with obstacles dilated by ``radius_m``."""
        radius_cells = int(math.ceil(radius_m / self.resolution))
        inflated = GridMap(
            self.width,
            self.height,
            self.resolution,
            self.origin_x,
            self.origin_y,
        )
        inflated.data = self.data.copy()

        rows, cols = np.where(self.data == self.OCCUPIED)
        for r, c in zip(rows, cols):
            r_min = max(0, r - radius_cells)
            r_max = min(self.height, r + radius_cells + 1)
            c_min = max(0, c - radius_cells)
            c_max = min(self.width, c + radius_cells + 1)
            inflated.data[r_min:r_max, c_min:c_max] = self.OCCUPIED

        return inflated

    # ------------------------------------------------------------------
    # Statistics
    # ------------------------------------------------------------------

    @property
    def total_cells(self) -> int:
        return self.width * self.height

    @property
    def free_cells(self) -> int:
        return int(np.sum(self.data == self.FREE))

    @property
    def occupied_cells(self) -> int:
        return int(np.sum(self.data == self.OCCUPIED))

    # ------------------------------------------------------------------
    # Serialisation helpers
    # ------------------------------------------------------------------

    def to_dict(self) -> dict:
        return {
            "width": self.width,
            "height": self.height,
            "resolution": self.resolution,
            "origin_x": self.origin_x,
            "origin_y": self.origin_y,
            "data": self.data.tolist(),
        }

    @classmethod
    def from_dict(cls, d: dict) -> "GridMap":
        gm = cls(
            d["width"],
            d["height"],
            d.get("resolution", 0.05),
            d.get("origin_x", 0.0),
            d.get("origin_y", 0.0),
        )
        gm.data = np.array(d["data"], dtype=np.uint8)
        return gm

    def save_json(self, path: str) -> None:
        with open(path, "w") as fh:
            json.dump(self.to_dict(), fh)

    @classmethod
    def load_json(cls, path: str) -> "GridMap":
        with open(path) as fh:
            return cls.from_dict(json.load(fh))

    # ------------------------------------------------------------------
    # ASCII visualisation
    # ------------------------------------------------------------------

    def __str__(self) -> str:
        lines = []
        for row in reversed(range(self.height)):
            line = ""
            for col in range(self.width):
                v = self.data[row, col]
                if v == self.FREE:
                    line += "."
                elif v == self.OCCUPIED:
                    line += "#"
                else:
                    line += "?"
            lines.append(line)
        return "\n".join(lines)


# ---------------------------------------------------------------------------
# Factory helpers
# ---------------------------------------------------------------------------


def create_empty_room(
    width_m: float = 10.0,
    height_m: float = 10.0,
    resolution: float = 0.05,
    wall_thickness_m: float = 0.2,
) -> GridMap:
    """Return a GridMap representing a single empty rectangular room.

    The outer boundary cells are marked as OCCUPIED (walls).
    """
    width_cells = int(math.ceil(width_m / resolution))
    height_cells = int(math.ceil(height_m / resolution))
    gm = GridMap(width_cells, height_cells, resolution)

    wall_cells = max(1, int(math.ceil(wall_thickness_m / resolution)))

    # Top and bottom walls
    gm.data[:wall_cells, :] = GridMap.OCCUPIED
    gm.data[-wall_cells:, :] = GridMap.OCCUPIED
    # Left and right walls
    gm.data[:, :wall_cells] = GridMap.OCCUPIED
    gm.data[:, -wall_cells:] = GridMap.OCCUPIED

    return gm


def create_room_with_obstacles(
    width_m: float = 10.0,
    height_m: float = 10.0,
    resolution: float = 0.05,
    wall_thickness_m: float = 0.2,
    obstacles: Optional[List[dict]] = None,
) -> GridMap:
    """Return an empty room with rectangular obstacles added.

    Each obstacle is a dict with keys ``x``, ``y`` (centre, metres),
    ``w`` (width, metres), ``h`` (height, metres).
    """
    gm = create_empty_room(width_m, height_m, resolution, wall_thickness_m)

    if obstacles:
        for obs in obstacles:
            cx, cy = obs["x"], obs["y"]
            ow, oh = obs["w"], obs["h"]
            col0, row0 = gm.world_to_cell(cx - ow / 2, cy - oh / 2)
            col1, row1 = gm.world_to_cell(cx + ow / 2, cy + oh / 2)
            col0 = max(0, col0)
            row0 = max(0, row0)
            col1 = min(gm.width - 1, col1)
            row1 = min(gm.height - 1, row1)
            gm.data[row0 : row1 + 1, col0 : col1 + 1] = GridMap.OCCUPIED

    return gm


def create_multi_room(
    width_m: float = 15.0,
    height_m: float = 10.0,
    resolution: float = 0.05,
    wall_thickness_m: float = 0.2,
) -> GridMap:
    """Two-room layout connected by a doorway in the shared wall."""
    gm = create_empty_room(width_m, height_m, resolution, wall_thickness_m)

    wall_cells = max(1, int(math.ceil(wall_thickness_m / resolution)))
    mid_col = gm.width // 2

    # Vertical dividing wall, except for a doorway in the middle
    door_cells = max(2, int(math.ceil(1.0 / resolution)))
    door_start = gm.height // 2 - door_cells // 2
    door_end = door_start + door_cells

    for row in range(wall_cells, gm.height - wall_cells):
        if door_start <= row < door_end:
            continue
        for dc in range(wall_cells):
            gm.data[row, mid_col + dc] = GridMap.OCCUPIED

    return gm


# ---------------------------------------------------------------------------
# PGM loader (ROS-style occupancy maps)
# ---------------------------------------------------------------------------


def load_pgm(pgm_path: str, resolution: float = 0.05) -> GridMap:
    """Load a PGM file (as produced by ROS map_saver) into a GridMap.

    Pixel value 205 → UNKNOWN, 0 → OCCUPIED, 254 → FREE.
    """
    pgm_path = Path(pgm_path)
    with open(pgm_path, "rb") as fh:
        magic = fh.readline().strip()
        if magic not in (b"P5", b"P2"):
            raise ValueError(f"Unsupported PGM magic: {magic}")

        # Skip comments
        line = fh.readline()
        while line.startswith(b"#"):
            line = fh.readline()

        width, height = (int(v) for v in line.split())
        max_val = int(fh.readline().strip())

        if magic == b"P5":
            raw = np.frombuffer(fh.read(), dtype=np.uint8).reshape(height, width)
        else:
            values = []
            for tok in fh.read().split():
                values.append(int(tok))
            raw = np.array(values, dtype=np.uint8).reshape(height, width)

    gm = GridMap(width, height, resolution)
    # ROS convention: white (254/255) = free, black (0) = occupied, grey = unknown
    occ = np.full_like(raw, GridMap.UNKNOWN)
    occ[raw >= 250] = GridMap.FREE
    occ[raw <= 10] = GridMap.OCCUPIED
    # PGM rows are top-to-bottom, GridMap rows are bottom-to-top
    gm.data = np.flipud(occ)
    return gm
