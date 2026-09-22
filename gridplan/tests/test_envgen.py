import numpy as np
import pytest
from gridplan.envgen import Environment


class TestValidateGrid:

    def _make_env(self):
        """Create an Environment without triggering grid generation."""
        env = Environment.__new__(Environment)
        return env

    def test_clear_grid_is_valid(self):
        env = self._make_env()
        grid = np.zeros((10, 10), dtype=np.uint8)
        start = (0, 0)
        target = (9, 9)
        assert env._validate_grid(grid, start, target) is True

    def test_straight_path_along_edge(self):
        env = self._make_env()
        grid = np.ones((10, 10), dtype=np.uint8)
        grid[0, :] = 0  # top row clear
        grid[:, 9] = 0  # right column clear
        start = (0, 0)
        target = (9, 9)
        assert env._validate_grid(grid, start, target) is True

    def test_wall_blocks_path(self):
        env = self._make_env()
        grid = np.zeros((10, 10), dtype=np.uint8)
        grid[:, 5] = 1  # solid wall across column 5
        start = (0, 0)
        target = (0, 9)
        assert env._validate_grid(grid, start, target) is False

    def test_wall_with_gap_is_valid(self):
        env = self._make_env()
        grid = np.zeros((10, 10), dtype=np.uint8)
        grid[:, 5] = 1
        grid[3, 5] = 0  # one gap in the wall
        start = (0, 0)
        target = (0, 9)
        assert env._validate_grid(grid, start, target) is True

    def test_start_equals_target(self):
        env = self._make_env()
        grid = np.zeros((10, 10), dtype=np.uint8)
        start = (5, 5)
        target = (5, 5)
        assert env._validate_grid(grid, start, target) is True

    def test_target_surrounded_by_obstacles(self):
        env = self._make_env()
        grid = np.zeros((10, 10), dtype=np.uint8)
        # wall around (5,5)
        grid[4, 4:7] = 1
        grid[6, 4:7] = 1
        grid[5, 4] = 1
        grid[5, 6] = 1
        start = (0, 0)
        target = (5, 5)
        assert env._validate_grid(grid, start, target) is False

    def test_narrow_corridor(self):
        env = self._make_env()
        grid = np.ones((10, 10), dtype=np.uint8)
        # carve a winding path
        grid[0, 0:8] = 0
        grid[0:3, 7] = 0
        grid[2, 3:8] = 0
        grid[2:5, 3] = 0
        grid[4, 3:10] = 0
        grid[4:7, 9] = 0
        grid[6, 5:10] = 0
        grid[6:10, 5] = 0
        grid[9, 5:10] = 0
        start = (0, 0)
        target = (9, 9)
        assert env._validate_grid(grid, start, target) is True

    def test_two_disconnected_islands(self):
        env = self._make_env()
        grid = np.ones((10, 10), dtype=np.uint8)
        # island 1: top-left 3x3
        grid[0:3, 0:3] = 0
        # island 2: bottom-right 3x3
        grid[7:10, 7:10] = 0
        start = (0, 0)
        target = (9, 9)
        assert env._validate_grid(grid, start, target) is False
