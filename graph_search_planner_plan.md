# Project 1: Graph-Search Planner Benchmark (Hybrid C++/Python)

**Duration:** 12 to 14 working days
**Core library:** C++20, CMake, pybind11, GoogleTest, Google Benchmark
**Python layer:** Python 3.11+, NumPy, SciPy, Matplotlib, pytest; Open3D or Plotly for 3D visualization
**Packaging:** scikit-build-core (installable with `pip install -e .`)
**Position in the drone track:** Foundation for trajectory generation (Project 3), incremental replanning (Project 5), and SITL integration (Project 6)

---

## 1. Objectives

1. Implement BFS, Dijkstra, A*, and weighted A* in C++ against a single, shared planner interface.
2. Support 2D occupancy grids (4- and 8-connectivity) and 3D voxel grids (6-, 18-, and 26-connectivity).
3. Expose the planners to Python through pybind11, accepting NumPy arrays as input and returning results as NumPy arrays.
4. Keep environment generation, visualization, benchmarking, and analysis in Python.
5. Maintain a small pure-Python reference implementation that serves as a correctness oracle and as the baseline for measuring C++ speedup.
6. Benchmark all algorithms across reproducible, randomly generated environments and report the results quantitatively.

## 2. Architectural Division

| Responsibility | Language | Rationale |
|---|---|---|
| Grid storage, neighbor generation, heuristics | C++ | Inner-loop performance |
| BFS, Dijkstra, A*, weighted A* | C++ | Inner-loop performance; industry-relevant practice |
| Timing of the search itself | C++ | Excludes binding and conversion overhead from measurements |
| Environment generation and obstacle inflation | Python | Simple with NumPy and `scipy.ndimage`; not performance-critical |
| Reference planners (Dijkstra, A*) | Python | Independent oracle for cross-validation |
| Visualization, benchmark sweeps, analysis | Python | Matplotlib and pandas are far more productive than C++ equivalents |

The boundary between the two layers should be narrow: an occupancy array, start and goal coordinates, and configuration options pass into C++; a result object passes back. Keeping the interface small is what keeps the bindings maintainable.

## 3. Deliverables

- A C++ library (`gridplan_core`) with a GoogleTest suite and Google Benchmark microbenchmarks.
- A Python package (`gridplan`) that wraps the library and contains the reference planners, environment generation, and visualization.
- A pytest suite that cross-validates the C++ planners against the Python reference.
- A benchmark script that emits a CSV of results and summary plots.
- A written report (a `README.md` is sufficient) presenting results and conclusions.
- At least one animation of node expansion for each algorithm in 2D, and one static 3D rendering of a planned path.

## 4. Proposed Repository Structure

```
gridplan/
  CMakeLists.txt
  CMakePresets.json          # debug, release, asan-ubsan presets
  pyproject.toml             # scikit-build-core configuration
  cpp/
    include/gridplan/
      grid.hpp               # Grid storage, index <-> coordinate conversion
      connectivity.hpp       # Neighbor offset and cost tables
      heuristics.hpp
      planner.hpp            # Planner interface, PlanResult, PlannerConfig
      bfs.hpp
      dijkstra.hpp
      astar.hpp              # A* and weighted A*
    src/
      grid.cpp
      bfs.cpp
      dijkstra.cpp
      astar.cpp
    tests/                   # GoogleTest
      test_grid.cpp
      test_planners.cpp
    bench/                   # Google Benchmark
      bench_planners.cpp
  bindings/
    module.cpp               # pybind11 module definition
  python/gridplan/
    __init__.py              # Re-exports the compiled module
    reference.py             # Pure-Python Dijkstra and A* (oracle)
    envgen.py                # Seeded environment generation, inflation
    viz2d.py
    viz3d.py
  benchmarks/
    run_benchmark.py
    analyze.py
  tests/                     # pytest
    test_bindings.py
    test_cross_validation.py
  README.md
```

Dependencies (pybind11, GoogleTest, Google Benchmark) can be obtained through CMake `FetchContent`, which avoids requiring system-level installation.

## 5. Technical Notes

### 5.1 Grid representation in C++

- Store occupancy as a flat `std::vector<uint8_t>` in row-major order, matching the memory layout of a C-contiguous NumPy array so that data can be copied (or viewed) without reordering.
- Represent cells internally as linear indices (`int32_t` is sufficient up to roughly two billion cells). Convert to and from coordinates only at the boundaries of the API.
- Store per-search state in flat arrays indexed by cell: `std::vector<float> g`, `std::vector<int32_t> parent`, and `std::vector<uint8_t> closed`. This is substantially faster than hash maps and is the standard technique in performant grid planners.
- Precompute, for each connectivity mode, a table of index offsets and step costs. Boundary handling requires care: a linear offset can wrap from the end of one row to the start of the next, so either validate coordinates for each neighbor or pad the grid with a one-cell occupied border, which removes bounds checks from the inner loop entirely. The padded-border approach is recommended.
- Decide whether to template on dimension (`Grid<2>`, `Grid<3>`) or to use one runtime-parameterized implementation. Document the choice and its trade-offs in the report.

### 5.2 Memory considerations

At 256³ (about 16.8 million cells), the occupancy grid requires about 17 MB, while float g-scores and int32 parents add about 134 MB. This is acceptable on a workstation but is a useful figure to report, since it illustrates why onboard planners use sparser representations such as octrees.

### 5.3 Movement costs and corner cutting

- Axis-aligned steps cost 1, face-diagonal steps cost √2, and 3D body-diagonal steps cost √3.
- Adopt an explicit corner-cutting rule. The conservative choice forbids a diagonal step unless every axis-aligned cell it passes between is free. Encode this in the precomputed neighbor tables as a list of cells that must be free for each diagonal offset.

### 5.4 Heuristics

| Connectivity | Heuristic | Form (with sorted absolute deltas d1 ≤ d2 ≤ d3) |
|---|---|---|
| 2D, 4-connected | Manhattan | dx + dy |
| 2D, 8-connected | Octile | (dx + dy) + (√2 − 2)·min(dx, dy) |
| 3D, 6-connected | Manhattan | dx + dy + dz |
| 3D, 26-connected | 3D octile | √3·d1 + √2·(d2 − d1) + (d3 − d2) |
| Any | Euclidean | Admissible but weaker for grid motion |

With a consistent heuristic, A* never needs to reopen a closed node. Verify consistency for your cost model as part of the project.

### 5.5 Search implementation details

- Use `std::priority_queue` with lazy deletion: push duplicate entries and discard stale ones when popped. The queue is a max-heap by default, so supply a comparator that orders by smallest `f`.
- Implement tie-breaking in the comparator: on equal `f`, prefer larger `g`. Make this a configuration option so that its effect can be measured.
- Weighted A* uses `f = g + w·h`. For `w ≥ 1` and an admissible `h`, path cost is at most `w` times optimal.
- BFS is optimal only when all step costs are equal; on 8- and 26-connected grids it is a baseline, not an optimal planner.
- Recording expansion order is optional through `PlannerConfig`, since it adds measurable overhead. Disable it during timing runs and enable it for visualization.

### 5.6 Binding design

- Accept the occupancy grid as `py::array_t<uint8_t, py::array::c_style | py::array::forcecast>`, and validate its dimensionality and shape before use.
- Return a `PlanResult` whose array fields (path, expansion order) are exposed as NumPy arrays. Returning a `std::vector` converted to a NumPy array is sufficient; zero-copy return is an optimization, not a requirement.
- Release the GIL during the search with `py::gil_scoped_release`. This costs nothing and allows parallel benchmark runs from Python threads later.
- Measure search time inside C++ with `std::chrono::steady_clock`, and separately measure end-to-end time from Python. The difference quantifies binding overhead, which is itself a useful result.

## 6. Testing Strategy

Testing occurs at two layers, and both matter.

**C++ (GoogleTest):**
- Index and coordinate conversion round-trips, neighbor tables, and corner-cutting rules on small hand-constructed grids.
- BFS and Dijkstra return equal costs on uniform-cost 4-connected grids.
- A* with a zero heuristic behaves identically to Dijkstra.
- A* cost equals Dijkstra cost for every admissible heuristic.
- Weighted A* cost is within the `w` bound.
- Unreachable goals, start equal to goal, and start or goal inside an obstacle are handled explicitly.

**Python (pytest):**
- Cross-validation: across several hundred random seeds, C++ Dijkstra and A* path costs match the pure-Python reference within floating-point tolerance.
- Binding behavior: incorrect dtypes, non-contiguous arrays, and wrong dimensionality raise clear exceptions rather than crashing.

**Sanitizers:** run the GoogleTest suite at least once under the `asan-ubsan` preset (`-fsanitize=address,undefined`). Out-of-bounds indexing in flat-array grid code is common and frequently silent without them.

## 7. Schedule

### Day 1: Toolchain
- Create the CMake project, presets, and `pyproject.toml`.
- Build a trivial pybind11 function, install it with `pip install -e .`, and call it from Python.
- Wire in GoogleTest and confirm that a placeholder test runs.
- The goal is a working build pipeline before any algorithmic code is written, since build problems are far harder to diagnose once real code is involved.

### Day 2: Python reference and environment generation
- Implement seeded environment generation: uniform random obstacles, random boxes, and a 2D maze generator.
- Implement inflation with `scipy.ndimage.binary_dilation`.
- Guarantee free start and goal cells, and reject unreachable configurations.
- Implement the pure-Python Dijkstra and A* reference planners. Keep them simple and obviously correct; they are an oracle, not a product.

### Day 3: C++ grid
- Implement the padded flat grid, index conversion, and neighbor tables for all five connectivity modes, including corner-cutting constraints.
- Write the GoogleTest grid tests.

### Day 4: C++ BFS and Dijkstra
- Define the planner interface, `PlannerConfig`, and `PlanResult`.
- Implement BFS and Dijkstra with optional expansion recording and internal timing.
- Write the corresponding GoogleTest cases.

### Day 5: C++ A* and weighted A*
- Implement the heuristics and A* with configurable weight and tie-breaking.
- Write the optimality and bounded-suboptimality tests.

### Day 6: Bindings and cross-validation
- Bind the grid construction, planners, configuration, and result types.
- Implement the pytest binding tests and the cross-validation suite.

### Day 7: Correctness hardening and profiling
- Run the full test suite under sanitizers and resolve any findings.
- Add Google Benchmark microbenchmarks for each planner on fixed grids.
- Profile the inner loop (`perf` on Linux is sufficient) and address any obvious inefficiencies, such as unnecessary allocations or bounds checks.

### Day 8: 2D visualization
- Expansion-order heatmaps with the path overlaid, a side-by-side comparison of all planners on one map, and an expansion animation exported as GIF or MP4.

### Day 9: 3D visualization
- Render occupied voxels (surface voxels or a point cloud for large grids) and the planned path.

### Days 10 and 11: Benchmark sweep
Run a sweep with a fixed set of seeds. Suggested variables:

- Grid size: 64², 256², 1024², 4096² in 2D; 32³, 64³, 128³, 256³ in 3D
- Obstacle density: 0.0, 0.1, 0.2, 0.3
- Connectivity: every supported mode
- Heuristic weight: 1.0, 1.5, 2.0, 5.0
- Tie-breaking: on and off
- Implementation: C++ and Python reference (the Python reference only on smaller grids, where it completes in reasonable time)

Record path cost, suboptimality relative to Dijkstra, nodes expanded, peak open-set size, internal C++ search time, and end-to-end Python time. Report medians and variance over seeds.

### Days 12 to 14: Analysis, report, and optional extension
- Generate summary plots and write the report addressing the questions in Section 8.
- If time permits, begin one extension from Section 9.

## 8. Questions the Report Should Answer

1. By what factor does A* reduce expansions relative to Dijkstra, and how does that factor change with obstacle density? Why does the advantage shrink in cluttered or maze-like environments?
2. How does the empirical suboptimality of weighted A* compare with its theoretical bound, and why is it typically far below it?
3. How much does tie-breaking matter on open grids versus cluttered ones?
4. How do runtime and memory scale from 2D to 3D at comparable resolution, and what does this imply for planning over a large outdoor volume?
5. What speedup does the C++ implementation achieve over the Python reference, and how does it vary with grid size? What fraction of end-to-end time is binding and conversion overhead, and at what problem size does that overhead become negligible?
6. What does profiling show about where the C++ search spends its time (heap operations, neighbor expansion, memory access)?
7. What do the paths look like geometrically? Characterize the unnecessary heading changes that Project 3 will need to remove.

## 9. Extensions

- **Theta\* (any-angle planning):** Permit parent pointers to skip intermediate cells when line of sight exists, using Bresenham traversal in 2D and its 3D analogue. This yields much smoother paths and sets up Project 3. Recommended as the first extension.
- **Jump Point Search:** Prunes the open set dramatically on uniform-cost 8-connected grids. A 2D implementation is well contained; the 3D generalization is considerably more involved.
- **Heap alternatives:** Compare `std::priority_queue` against a bucket queue or a d-ary heap, and measure the effect on large grids.
- **Bidirectional A\*:** Search from start and goal simultaneously, with careful attention to the termination condition, which is a common source of subtle errors.
- **Continuous integration:** A GitHub Actions workflow that builds the package, runs both test suites, and runs the sanitizer build strengthens the project as a portfolio piece.

## 10. Completion Criteria

The project is complete when the C++ planners pass the GoogleTest suite cleanly under sanitizers, match the Python reference across several hundred random seeds, the package installs and runs from a single `pip install -e .`, the benchmark sweep reproduces identical results for identical seeds from a single command, and the report answers the questions in Section 8 with supporting figures.

## 11. References

- Hart, Nilsson, and Raphael (1968), *A Formal Basis for the Heuristic Determination of Minimum Cost Paths*: the original A* paper.
- LaValle, *Planning Algorithms* (Cambridge University Press, 2006; freely available online), Chapter 2 on discrete planning.
- Amit Patel, *Red Blob Games*: articles on A* and grid heuristics, excellent for implementation details and visualization ideas.
- Harabor and Grastien (2011), *Online Graph Pruning for Pathfinding on Grid Maps*: Jump Point Search.
- Nash, Daniel, Koenig, and Felner (2007), *Theta\*: Any-Angle Path Planning on Grids*.
- pybind11 documentation, particularly the sections on NumPy arrays and the GIL.
- scikit-build-core documentation, for building and packaging CMake-based Python extensions.
