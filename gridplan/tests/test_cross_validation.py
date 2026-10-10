import numpy as np
import pytest
import gridplan
from gridplan.reference import dijkstra as py_dijkstra, a_star as py_a_star
from gridplan.envgen import Environment


def padded_index(row, col, cols):
    """Convert (row, col) to padded linear index for the C++ grid."""
    return (row + 1) * (cols + 2) + (col + 1)


def run_cpp_dijkstra(grid_array, start, target):
    """Run C++ Dijkstra on a 2D numpy grid."""
    rows, cols = grid_array.shape
    flat = grid_array.flatten().tolist()
    cpp_grid = gridplan.Grid(rows, cols, flat, gridplan.Connectivity.FOUR)
    cfg = gridplan.PlannerConfig()

    start_idx = padded_index(start[0], start[1], cols)
    goal_idx = padded_index(target[0], target[1], cols)
    return gridplan.Dijkstra().plan(cpp_grid, start_idx, goal_idx, cfg)


def run_cpp_astar(grid_array, start, target):
    """Run C++ A* on a 2D numpy grid."""
    rows, cols = grid_array.shape
    flat = grid_array.flatten().tolist()
    cpp_grid = gridplan.Grid(rows, cols, flat, gridplan.Connectivity.FOUR)
    cfg = gridplan.PlannerConfig(heuristic=gridplan.Heuristic.MANHATTAN)

    start_idx = padded_index(start[0], start[1], cols)
    goal_idx = padded_index(target[0], target[1], cols)
    return gridplan.AStar().plan(cpp_grid, start_idx, goal_idx, cfg)


# ==================== Dijkstra cross-validation ====================

class TestCrossValidationDijkstra:

    @pytest.mark.parametrize("seed", range(200))
    def test_cost_matches_python_reference_uniform(self, seed):
        """C++ Dijkstra cost matches Python reference on uniform random grids."""
        try:
            env = Environment((32, 32), 0.2, seed, option="uniform")
        except Exception:
            pytest.skip("Environment generation failed for this seed")

        py_result = py_dijkstra(env.grid, env.start, env.target)
        cpp_result = run_cpp_dijkstra(env.grid, env.start, env.target)

        assert abs(cpp_result.cost - py_result.cost) < 0.01, (
            f"Seed {seed}: C++ cost={cpp_result.cost}, Python cost={py_result.cost}"
        )

    @pytest.mark.parametrize("seed", range(100))
    def test_cost_matches_python_reference_box(self, seed):
        """C++ Dijkstra cost matches Python reference on box obstacle grids."""
        try:
            env = Environment((32, 32), 0.2, seed, option="box")
        except Exception:
            pytest.skip("Environment generation failed for this seed")

        py_result = py_dijkstra(env.grid, env.start, env.target)
        cpp_result = run_cpp_dijkstra(env.grid, env.start, env.target)

        assert abs(cpp_result.cost - py_result.cost) < 0.01, (
            f"Seed {seed}: C++ cost={cpp_result.cost}, Python cost={py_result.cost}"
        )

    @pytest.mark.parametrize("density", [0.0, 0.1, 0.2, 0.3])
    def test_cost_matches_across_densities(self, density):
        """C++ Dijkstra matches Python reference across obstacle densities."""
        for seed in range(50):
            try:
                env = Environment((32, 32), density, seed, option="uniform")
            except Exception:
                continue

            py_result = py_dijkstra(env.grid, env.start, env.target)
            cpp_result = run_cpp_dijkstra(env.grid, env.start, env.target)

            assert abs(cpp_result.cost - py_result.cost) < 0.01, (
                f"Density {density}, Seed {seed}: C++ cost={cpp_result.cost}, Python cost={py_result.cost}"
            )


# ==================== A* cross-validation ====================

class TestCrossValidationAStar:

    @pytest.mark.parametrize("seed", range(200))
    def test_cost_matches_python_reference_uniform(self, seed):
        """C++ A* cost matches Python reference on uniform random grids."""
        try:
            env = Environment((32, 32), 0.2, seed, option="uniform")
        except Exception:
            pytest.skip("Environment generation failed for this seed")

        py_result = py_a_star(env.grid, env.start, env.target)
        cpp_result = run_cpp_astar(env.grid, env.start, env.target)

        assert abs(cpp_result.cost - py_result.cost) < 0.01, (
            f"Seed {seed}: C++ cost={cpp_result.cost}, Python cost={py_result.cost}"
        )

    @pytest.mark.parametrize("seed", range(100))
    def test_cost_matches_python_reference_box(self, seed):
        """C++ A* cost matches Python reference on box obstacle grids."""
        try:
            env = Environment((32, 32), 0.2, seed, option="box")
        except Exception:
            pytest.skip("Environment generation failed for this seed")

        py_result = py_a_star(env.grid, env.start, env.target)
        cpp_result = run_cpp_astar(env.grid, env.start, env.target)

        assert abs(cpp_result.cost - py_result.cost) < 0.01, (
            f"Seed {seed}: C++ cost={cpp_result.cost}, Python cost={py_result.cost}"
        )


# ==================== Dijkstra vs A* cross-validation ====================

class TestCrossValidationDijkstraVsAStar:

    @pytest.mark.parametrize("seed", range(200))
    def test_cpp_dijkstra_and_astar_agree(self, seed):
        """C++ Dijkstra and A* find the same optimal cost."""
        try:
            env = Environment((32, 32), 0.2, seed, option="uniform")
        except Exception:
            pytest.skip("Environment generation failed for this seed")

        d_result = run_cpp_dijkstra(env.grid, env.start, env.target)
        a_result = run_cpp_astar(env.grid, env.start, env.target)

        assert abs(d_result.cost - a_result.cost) < 0.01, (
            f"Seed {seed}: Dijkstra cost={d_result.cost}, A* cost={a_result.cost}"
        )
