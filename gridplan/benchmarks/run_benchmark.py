"""
Benchmark sweep for gridplan.

Runs all planner/grid/density/connectivity combinations and writes results to CSV.
Usage: python benchmarks/run_benchmark.py [--output results.csv] [--seeds 5]
"""

import argparse
import csv
import time
import itertools
import numpy as np
import gridplan
from gridplan.reference import dijkstra as py_dijkstra, a_star as py_a_star


# ---- Helpers ----

def padded_index_2d(row, col, cols):
    return (row + 1) * (cols + 2) + (col + 1)


def padded_index_3d(row, col, depth, cols, rows):
    padded_cols = cols + 2
    z_stride = (rows + 2) * padded_cols
    return (depth + 1) * z_stride + (row + 1) * padded_cols + (col + 1)


def generate_2d_grid(size, density, seed):
    """Generate a 2D grid with given density. Returns (grid_array, start, target)."""
    rng = np.random.default_rng(seed=seed)
    n_cells = size * size
    n_obstacles = int(n_cells * density)

    grid = np.zeros(n_cells, dtype=np.uint8)
    grid[:n_obstacles] = 1
    rng.shuffle(grid)
    grid = grid.reshape((size, size))

    # Clear corners for start/goal
    grid[0, 0] = 0
    grid[size - 1, size - 1] = 0

    start = (0, 0)
    target = (size - 1, size - 1)
    return grid, start, target


def generate_3d_grid(size, density, seed):
    """Generate a 3D grid with given density. Returns (grid_array, start, target)."""
    rng = np.random.default_rng(seed=seed)
    n_cells = size * size * size
    n_obstacles = int(n_cells * density)

    grid = np.zeros(n_cells, dtype=np.uint8)
    grid[:n_obstacles] = 1
    rng.shuffle(grid)
    grid = grid.reshape((size, size, size))

    grid[0, 0, 0] = 0
    grid[size - 1, size - 1, size - 1] = 0

    start = (0, 0, 0)
    target = (size - 1, size - 1, size - 1)
    return grid, start, target


def is_reachable_2d(grid, start, target):
    """Quick BFS reachability check."""
    from collections import deque
    rows, cols = grid.shape
    visited = set()
    queue = deque([start])
    visited.add(start)
    while queue:
        r, c = queue.popleft()
        if (r, c) == target:
            return True
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and grid[nr, nc] == 0 and (nr, nc) not in visited:
                visited.add((nr, nc))
                queue.append((nr, nc))
    return False


# ---- Configuration ----

GRID_SIZES_2D = [64, 256, 1024]
GRID_SIZES_3D = [32, 64]
DENSITIES = [0.0, 0.1, 0.2, 0.3]

CONNECTIVITY_2D = [
    ("FOUR", gridplan.Connectivity.FOUR),
    ("EIGHT", gridplan.Connectivity.EIGHT),
]
CONNECTIVITY_3D = [
    ("SIX", gridplan.Connectivity.SIX),
    ("TWENTYSIX", gridplan.Connectivity.TWENTYSIX),
]

HEURISTIC_MAP = {
    "FOUR": ("MANHATTAN", gridplan.Heuristic.MANHATTAN),
    "EIGHT": ("OCTILE", gridplan.Heuristic.OCTILE),
    "SIX": ("MANHATTAN", gridplan.Heuristic.MANHATTAN),
    "TWENTYSIX": ("OCTILE", gridplan.Heuristic.OCTILE),
}

WEIGHTS = [1.0, 1.5, 2.0, 5.0]
TIE_BREAK_OPTIONS = [False, True]

# Python reference only on small grids
PYTHON_MAX_SIZE_2D = 256

CSV_HEADER = [
    "dim", "grid_size", "density", "connectivity", "algorithm", "heuristic",
    "weight", "tie_break", "seed", "path_cost", "nodes_expanded",
    "cpp_time_ms", "python_e2e_time_ms", "implementation"
]


# ---- Run functions ----

def run_cpp_planner(planner_class, cpp_grid, start_idx, goal_idx, config):
    """Run a C++ planner and return (result, python_e2e_time_ms)."""
    planner = planner_class()
    t0 = time.perf_counter()
    result = planner.plan(cpp_grid, start_idx, goal_idx, config)
    t1 = time.perf_counter()
    python_e2e_ms = (t1 - t0) * 1000
    return result, python_e2e_ms


def run_python_dijkstra(grid_2d, start, target):
    """Run Python reference Dijkstra. Returns (cost, nodes_expanded, time_ms)."""
    t0 = time.perf_counter()
    result = py_dijkstra(grid_2d, start, target)
    t1 = time.perf_counter()
    return result.cost, result.n_nodes_expanded, (t1 - t0) * 1000


def run_python_astar(grid_2d, start, target):
    """Run Python reference A*. Returns (cost, nodes_expanded, time_ms)."""
    t0 = time.perf_counter()
    result = py_a_star(grid_2d, start, target)
    t1 = time.perf_counter()
    return result.cost, result.n_nodes_expanded, (t1 - t0) * 1000


# ---- Sweep ----

def run_2d_sweep(writer, num_seeds):
    """Run all 2D benchmark configurations."""
    for size in GRID_SIZES_2D:
        for density in DENSITIES:
            for conn_name, conn in CONNECTIVITY_2D:
                heur_name, heur = HEURISTIC_MAP[conn_name]

                for seed in range(num_seeds):
                    grid_2d, start, target = generate_2d_grid(size, density, seed)

                    if not is_reachable_2d(grid_2d, start, target):
                        continue

                    flat = grid_2d.flatten().tolist()
                    cpp_grid = gridplan.Grid(size, size, flat, conn)
                    start_idx = padded_index_2d(start[0], start[1], size)
                    goal_idx = padded_index_2d(target[0], target[1], size)

                    # Get optimal cost from Dijkstra first
                    dijkstra_config = gridplan.PlannerConfig(
                        tie_break=False, record_expansions=False,
                        weight=1.0, heuristic=heur
                    )
                    dijkstra_result, dijkstra_e2e = run_cpp_planner(
                        gridplan.Dijkstra, cpp_grid, start_idx, goal_idx, dijkstra_config
                    )
                    if dijkstra_result.cost < 0:
                        continue

                    # -- BFS --
                    bfs_config = gridplan.PlannerConfig(
                        tie_break=False, record_expansions=False,
                        weight=1.0, heuristic=heur
                    )
                    bfs_result, bfs_e2e = run_cpp_planner(
                        gridplan.BFS, cpp_grid, start_idx, goal_idx, bfs_config
                    )
                    writer.writerow([
                        "2D", size, density, conn_name, "BFS", "N/A",
                        1.0, False, seed, bfs_result.cost, bfs_result.nodes_expanded,
                        bfs_result.time_ms, bfs_e2e, "cpp"
                    ])

                    # -- Dijkstra (already computed) --
                    writer.writerow([
                        "2D", size, density, conn_name, "Dijkstra", "N/A",
                        1.0, False, seed, dijkstra_result.cost, dijkstra_result.nodes_expanded,
                        dijkstra_result.time_ms, dijkstra_e2e, "cpp"
                    ])

                    # -- A* and Weighted A* with tie-breaking options --
                    for weight in WEIGHTS:
                        for tie_break in TIE_BREAK_OPTIONS:
                            algo_name = "AStar" if weight == 1.0 else f"WeightedAStar"
                            config = gridplan.PlannerConfig(
                                tie_break=tie_break, record_expansions=False,
                                weight=weight, heuristic=heur
                            )
                            result, e2e = run_cpp_planner(
                                gridplan.AStar, cpp_grid, start_idx, goal_idx, config
                            )
                            writer.writerow([
                                "2D", size, density, conn_name, algo_name, heur_name,
                                weight, tie_break, seed, result.cost, result.nodes_expanded,
                                result.time_ms, e2e, "cpp"
                            ])

                    # -- Python reference (small grids only) --
                    if size <= PYTHON_MAX_SIZE_2D and conn_name == "FOUR":
                        py_cost, py_expanded, py_time = run_python_dijkstra(grid_2d, start, target)
                        writer.writerow([
                            "2D", size, density, conn_name, "Dijkstra", "N/A",
                            1.0, False, seed, py_cost, py_expanded,
                            0.0, py_time, "python"
                        ])

                        py_cost, py_expanded, py_time = run_python_astar(grid_2d, start, target)
                        writer.writerow([
                            "2D", size, density, conn_name, "AStar", "MANHATTAN",
                            1.0, False, seed, py_cost, py_expanded,
                            0.0, py_time, "python"
                        ])

                print(f"  Done: 2D {size}x{size}, density={density}, {conn_name}")


def run_3d_sweep(writer, num_seeds):
    """Run all 3D benchmark configurations."""
    for size in GRID_SIZES_3D:
        for density in DENSITIES:
            for conn_name, conn in CONNECTIVITY_3D:
                heur_name, heur = HEURISTIC_MAP[conn_name]

                for seed in range(num_seeds):
                    grid_3d = generate_3d_grid(size, density, seed)[0]
                    flat = grid_3d.flatten().tolist()
                    cpp_grid = gridplan.Grid(size, size, size, flat, conn)

                    start_idx = padded_index_3d(0, 0, 0, size, size)
                    goal_idx = padded_index_3d(size - 1, size - 1, size - 1, size, size)

                    # -- Dijkstra --
                    config = gridplan.PlannerConfig(
                        tie_break=False, record_expansions=False,
                        weight=1.0, heuristic=heur
                    )
                    result, e2e = run_cpp_planner(
                        gridplan.Dijkstra, cpp_grid, start_idx, goal_idx, config
                    )
                    if result.cost < 0:
                        continue

                    writer.writerow([
                        "3D", size, density, conn_name, "Dijkstra", "N/A",
                        1.0, False, seed, result.cost, result.nodes_expanded,
                        result.time_ms, e2e, "cpp"
                    ])

                    # -- A* and Weighted A* --
                    for weight in WEIGHTS:
                        algo_name = "AStar" if weight == 1.0 else "WeightedAStar"
                        config = gridplan.PlannerConfig(
                            tie_break=False, record_expansions=False,
                            weight=weight, heuristic=heur
                        )
                        result, e2e = run_cpp_planner(
                            gridplan.AStar, cpp_grid, start_idx, goal_idx, config
                        )
                        writer.writerow([
                            "3D", size, density, conn_name, algo_name, heur_name,
                            weight, False, seed, result.cost, result.nodes_expanded,
                            result.time_ms, e2e, "cpp"
                        ])

                print(f"  Done: 3D {size}^3, density={density}, {conn_name}")


def main():
    parser = argparse.ArgumentParser(description="Gridplan benchmark sweep")
    parser.add_argument("--output", default="benchmarks/results.csv", help="Output CSV path")
    parser.add_argument("--seeds", type=int, default=5, help="Number of seeds per configuration")
    parser.add_argument("--skip-3d", action="store_true", help="Skip 3D benchmarks")
    args = parser.parse_args()

    print(f"Starting benchmark sweep (seeds={args.seeds})")
    print(f"Output: {args.output}")

    with open(args.output, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(CSV_HEADER)

        print("\n=== 2D Sweep ===")
        run_2d_sweep(writer, args.seeds)

        if not args.skip_3d:
            print("\n=== 3D Sweep ===")
            run_3d_sweep(writer, args.seeds)

    print(f"\nDone. Results written to {args.output}")


if __name__ == "__main__":
    main()
