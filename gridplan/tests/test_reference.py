import numpy as np
import pytest
from gridplan.reference import dijkstra


class TestDijkstra:

    def test_adjacent_start_and_target(self):
        grid = np.zeros((10, 10), dtype=np.uint8)
        result = dijkstra(grid, (0, 0), (0, 1))
        assert result.cost == 1
        assert result.path[0] == (0, 0)
        assert result.path[-1] == (0, 1)
        assert len(result.path) == 2

    def test_straight_line_no_obstacles(self):
        grid = np.zeros((10, 10), dtype=np.uint8)
        result = dijkstra(grid, (0, 0), (0, 9))
        assert result.cost == 9
        assert result.path[0] == (0, 0)
        assert result.path[-1] == (0, 9)
        assert len(result.path) == 10

    def test_start_equals_target(self):
        grid = np.zeros((10, 10), dtype=np.uint8)
        result = dijkstra(grid, (5, 5), (5, 5))
        assert result.cost == 0
        assert result.path == [(5, 5)]

    def test_obstacle_forces_detour(self):
        grid = np.zeros((10, 10), dtype=np.uint8)
        # wall across row 5, columns 0-8
        grid[5, 0:9] = 1
        result = dijkstra(grid, (4, 4), (6, 4))
        # can't go straight down, must go around the wall
        assert result.cost > 2
        assert result.path[0] == (4, 4)
        assert result.path[-1] == (6, 4)

    def test_corner_to_corner_empty_grid(self):
        grid = np.zeros((10, 10), dtype=np.uint8)
        result = dijkstra(grid, (0, 0), (9, 9))
        # manhattan distance is 18, optimal cost on 4-connected is 18
        assert result.cost == 18
        assert result.path[0] == (0, 0)
        assert result.path[-1] == (9, 9)

    def test_narrow_corridor(self):
        grid = np.ones((10, 10), dtype=np.uint8)
        # carve an L-shaped corridor
        grid[0, 0:10] = 0  # top row
        grid[0:10, 9] = 0  # right column
        result = dijkstra(grid, (0, 0), (9, 9))
        assert result.cost == 18
        assert result.path[0] == (0, 0)
        assert result.path[-1] == (9, 9)

    def test_path_is_contiguous(self):
        """Each step in the returned path should be exactly one cell apart."""
        grid = np.zeros((10, 10), dtype=np.uint8)
        grid[3, 1:9] = 1
        result = dijkstra(grid, (0, 0), (9, 9))
        for i in range(len(result.path) - 1):
            r1, c1 = result.path[i]
            r2, c2 = result.path[i + 1]
            assert abs(r1 - r2) + abs(c1 - c2) == 1

    def test_path_avoids_obstacles(self):
        """No cell in the path should be an obstacle."""
        grid = np.zeros((10, 10), dtype=np.uint8)
        grid[2, 0:8] = 1
        grid[5, 2:10] = 1
        result = dijkstra(grid, (0, 0), (9, 9))
        for r, c in result.path:
            assert grid[r, c] == 0

    def test_path_length_equals_cost_plus_one(self):
        """On a 4-connected grid with uniform cost 1, path length = cost + 1."""
        grid = np.zeros((10, 10), dtype=np.uint8)
        result = dijkstra(grid, (0, 0), (9, 9))
        assert len(result.path) == result.cost + 1

    def test_nodes_expanded_is_positive(self):
        grid = np.zeros((10, 10), dtype=np.uint8)
        result = dijkstra(grid, (0, 0), (9, 9))
        assert result.n_nodes_expanded > 0
