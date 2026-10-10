import numpy as np
import pytest
import gridplan


class TestGridBindings:

    def test_create_2d_grid(self):
        data = [0] * 16
        grid = gridplan.Grid(4, 4, data, gridplan.Connectivity.FOUR)
        assert grid.size() > 0

    def test_create_3d_grid(self):
        data = [0] * 64
        grid = gridplan.Grid(4, 4, 4, data, gridplan.Connectivity.SIX)
        assert grid.size() > 0

    def test_wrong_type_for_grid_data(self):
        with pytest.raises(TypeError):
            gridplan.Grid(4, 4, "not a list", gridplan.Connectivity.FOUR)

    def test_wrong_type_for_rows(self):
        with pytest.raises(TypeError):
            gridplan.Grid("four", 4, [0] * 16, gridplan.Connectivity.FOUR)

    def test_wrong_type_for_connectivity(self):
        with pytest.raises(TypeError):
            gridplan.Grid(4, 4, [0] * 16, "FOUR")

    def test_get_neighbors_returns_list(self):
        data = [0] * 16
        grid = gridplan.Grid(4, 4, data, gridplan.Connectivity.FOUR)
        # padded index for (1,1): (1+1)*(4+2) + (1+1) = 14
        neighbors = grid.get_neighbors(14)
        assert isinstance(neighbors, list)
        assert len(neighbors) > 0

    def test_is_obstacle(self):
        data = [0] * 16
        data[0] = 1  # (0,0) is obstacle
        grid = gridplan.Grid(4, 4, data, gridplan.Connectivity.FOUR)
        # padded index for (0,0): (0+1)*(4+2) + (0+1) = 7
        assert grid.is_obstacle(7) is True
        # padded index for (0,1): (0+1)*(4+2) + (1+1) = 8
        assert grid.is_obstacle(8) is False

    def test_index_to_coords(self):
        data = [0] * 16
        grid = gridplan.Grid(4, 4, data, gridplan.Connectivity.FOUR)
        # padded index for (2,3): (2+1)*(4+2) + (3+1) = 22
        coords = grid.index_to_coords(22)
        assert coords == (2, 3, 0)


class TestPlannerConfigBindings:

    def test_default_config(self):
        cfg = gridplan.PlannerConfig()
        assert cfg.tie_break is False
        assert cfg.record_expansions is False
        assert cfg.weight == 1.0
        assert cfg.heuristic == gridplan.Heuristic.MANHATTAN

    def test_custom_config(self):
        cfg = gridplan.PlannerConfig(
            tie_break=True,
            record_expansions=True,
            weight=2.5,
            heuristic=gridplan.Heuristic.OCTILE
        )
        assert cfg.tie_break is True
        assert cfg.weight == 2.5
        assert cfg.heuristic == gridplan.Heuristic.OCTILE

    def test_modify_config(self):
        cfg = gridplan.PlannerConfig()
        cfg.weight = 3.0
        assert cfg.weight == 3.0

    def test_wrong_type_for_heuristic(self):
        with pytest.raises(TypeError):
            gridplan.PlannerConfig(heuristic="MANHATTAN")


class TestPlannerResultBindings:

    def test_result_fields(self):
        data = [0] * 16
        grid = gridplan.Grid(4, 4, data, gridplan.Connectivity.FOUR)
        cfg = gridplan.PlannerConfig()
        bfs = gridplan.BFS()

        # padded indices for (0,0) and (3,3)
        start = (0 + 1) * (4 + 2) + (0 + 1)
        goal = (3 + 1) * (4 + 2) + (3 + 1)
        result = bfs.plan(grid, start, goal, cfg)

        assert hasattr(result, 'path')
        assert hasattr(result, 'cost')
        assert hasattr(result, 'nodes_expanded')
        assert hasattr(result, 'expansion_order')
        assert hasattr(result, 'time_ms')
        assert isinstance(result.path, list)
        assert isinstance(result.cost, float)
        assert isinstance(result.nodes_expanded, int)
        assert isinstance(result.time_ms, float)

    def test_expansion_order_none_by_default(self):
        data = [0] * 16
        grid = gridplan.Grid(4, 4, data, gridplan.Connectivity.FOUR)
        cfg = gridplan.PlannerConfig(record_expansions=False)
        bfs = gridplan.BFS()

        start = (0 + 1) * (4 + 2) + (0 + 1)
        goal = (3 + 1) * (4 + 2) + (3 + 1)
        result = bfs.plan(grid, start, goal, cfg)
        assert result.expansion_order is None

    def test_expansion_order_recorded(self):
        data = [0] * 16
        grid = gridplan.Grid(4, 4, data, gridplan.Connectivity.FOUR)
        cfg = gridplan.PlannerConfig(record_expansions=True)
        bfs = gridplan.BFS()

        start = (0 + 1) * (4 + 2) + (0 + 1)
        goal = (3 + 1) * (4 + 2) + (3 + 1)
        result = bfs.plan(grid, start, goal, cfg)
        assert result.expansion_order is not None
        assert len(result.expansion_order) > 0


class TestPlannerBindings:

    def _padded_index(self, r, c, cols=4):
        return (r + 1) * (cols + 2) + (c + 1)

    def test_bfs_plan(self):
        data = [0] * 16
        grid = gridplan.Grid(4, 4, data, gridplan.Connectivity.FOUR)
        cfg = gridplan.PlannerConfig()
        bfs = gridplan.BFS()

        result = bfs.plan(grid, self._padded_index(0, 0), self._padded_index(3, 3), cfg)
        assert result.cost == 6.0
        assert len(result.path) == 7

    def test_dijkstra_plan(self):
        data = [0] * 16
        grid = gridplan.Grid(4, 4, data, gridplan.Connectivity.FOUR)
        cfg = gridplan.PlannerConfig()
        dijkstra = gridplan.Dijkstra()

        result = dijkstra.plan(grid, self._padded_index(0, 0), self._padded_index(3, 3), cfg)
        assert result.cost == 6.0

    def test_astar_plan(self):
        data = [0] * 16
        grid = gridplan.Grid(4, 4, data, gridplan.Connectivity.FOUR)
        cfg = gridplan.PlannerConfig()
        astar = gridplan.AStar()

        result = astar.plan(grid, self._padded_index(0, 0), self._padded_index(3, 3), cfg)
        assert result.cost == 6.0

    def test_wrong_type_for_start(self):
        data = [0] * 16
        grid = gridplan.Grid(4, 4, data, gridplan.Connectivity.FOUR)
        cfg = gridplan.PlannerConfig()
        bfs = gridplan.BFS()

        with pytest.raises(TypeError):
            bfs.plan(grid, "start", 7, cfg)

    def test_wrong_type_for_config(self):
        data = [0] * 16
        grid = gridplan.Grid(4, 4, data, gridplan.Connectivity.FOUR)
        bfs = gridplan.BFS()

        with pytest.raises(TypeError):
            bfs.plan(grid, 7, 25, "not a config")


class TestEnumBindings:

    def test_connectivity_values(self):
        assert gridplan.Connectivity.FOUR is not None
        assert gridplan.Connectivity.EIGHT is not None
        assert gridplan.Connectivity.SIX is not None
        assert gridplan.Connectivity.EIGHTEEN is not None
        assert gridplan.Connectivity.TWENTYSIX is not None

    def test_heuristic_values(self):
        assert gridplan.Heuristic.MANHATTAN is not None
        assert gridplan.Heuristic.OCTILE is not None
        assert gridplan.Heuristic.EUCLIDEAN is not None
        assert gridplan.Heuristic.ZERO is not None

    def test_invalid_enum_string(self):
        with pytest.raises(TypeError):
            gridplan.Grid(4, 4, [0] * 16, "INVALID")
