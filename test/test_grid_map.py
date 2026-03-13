"""
Unit tests for the GridMap and map factory functions.
"""

import math
import sys
import os
import pytest

# Allow imports from src without installing
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src", "map_tools"))

from map_tools.grid_map import (
    GridMap,
    create_empty_room,
    create_multi_room,
    create_room_with_obstacles,
)


class TestGridMap:
    def test_create_basic(self):
        gm = GridMap(20, 10)
        assert gm.width == 20
        assert gm.height == 10
        assert gm.data.shape == (10, 20)

    def test_default_free(self):
        gm = GridMap(5, 5)
        assert gm.free_cells == 25
        assert gm.occupied_cells == 0

    def test_cell_access(self):
        gm = GridMap(10, 10)
        gm.set_cell(3, 4, GridMap.OCCUPIED)
        assert gm.get_cell(3, 4) == GridMap.OCCUPIED
        assert gm.is_occupied(3, 4)
        assert not gm.is_free(3, 4)

    def test_out_of_bounds_raises(self):
        gm = GridMap(5, 5)
        with pytest.raises(IndexError):
            gm.get_cell(5, 0)
        with pytest.raises(IndexError):
            gm.set_cell(0, 5, GridMap.FREE)

    def test_is_valid(self):
        gm = GridMap(5, 5)
        assert gm.is_valid(0, 0)
        assert gm.is_valid(4, 4)
        assert not gm.is_valid(-1, 0)
        assert not gm.is_valid(5, 0)
        assert not gm.is_valid(0, 5)

    def test_world_to_cell_and_back(self):
        gm = GridMap(100, 100, resolution=0.05)
        wx, wy = 2.5, 3.75
        col, row = gm.world_to_cell(wx, wy)
        cx, cy = gm.cell_to_world(col, row)
        assert abs(cx - (wx + 0.025)) < 0.051  # within one cell
        assert abs(cy - (wy + 0.025)) < 0.051

    def test_inflate_obstacles(self):
        gm = GridMap(20, 20, resolution=0.1)
        gm.set_cell(10, 10, GridMap.OCCUPIED)
        inflated = gm.inflate_obstacles(0.15)
        # The cell itself and neighbours within 1-2 cells should be occupied
        assert inflated.is_occupied(10, 10)
        assert inflated.is_occupied(11, 10)
        assert inflated.is_occupied(9, 10)

    def test_serialisation_round_trip(self, tmp_path):
        gm = GridMap(8, 6, resolution=0.1, origin_x=1.0, origin_y=2.0)
        gm.set_cell(3, 2, GridMap.OCCUPIED)
        path = str(tmp_path / "test_map.json")
        gm.save_json(path)
        gm2 = GridMap.load_json(path)
        assert gm2.width == gm.width
        assert gm2.height == gm.height
        assert gm2.resolution == gm.resolution
        assert gm2.get_cell(3, 2) == GridMap.OCCUPIED

    def test_str(self):
        gm = GridMap(3, 3)
        gm.set_cell(1, 1, GridMap.OCCUPIED)
        s = str(gm)
        assert "#" in s
        assert "." in s


class TestMapFactories:
    def test_empty_room_walls(self):
        gm = create_empty_room(10.0, 10.0, resolution=0.1)
        # Corners should be occupied (wall)
        assert gm.is_occupied(0, 0)
        # Centre should be free
        assert gm.is_free(gm.width // 2, gm.height // 2)

    def test_room_with_obstacles(self):
        obstacles = [{"x": 5.0, "y": 5.0, "w": 1.0, "h": 1.0}]
        gm = create_room_with_obstacles(10.0, 10.0, resolution=0.1, obstacles=obstacles)
        # Obstacle centre should be occupied
        col, row = gm.world_to_cell(5.0, 5.0)
        assert gm.is_occupied(col, row)

    def test_multi_room_doorway(self):
        gm = create_multi_room(15.0, 10.0, resolution=0.1)
        # The overall grid has free cells
        assert gm.free_cells > 0
        # Mid column has some occupied cells (dividing wall)
        mid = gm.width // 2
        occupied_in_mid_col = sum(
            1 for r in range(gm.height) if gm.is_occupied(mid, r)
        )
        assert occupied_in_mid_col > 0

    def test_free_cells_positive(self):
        gm = create_empty_room(5.0, 5.0, resolution=0.1)
        assert gm.free_cells > 0
        assert gm.total_cells == gm.free_cells + gm.occupied_cells
