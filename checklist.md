# Graph-Search Planner Benchmark — Daily Checklist

---

## Day 1: Toolchain

- [x] Create top-level directory structure (`cpp/`, `bindings/`, `python/gridplan/`, `benchmarks/`, `tests/`)
- [x] Write `CMakeLists.txt` with FetchContent for pybind11, GoogleTest, Google Benchmark
- [x] Write `CMakePresets.json` (debug, release, asan-ubsan presets)
- [x] Write `pyproject.toml` with scikit-build-core configuration
- [x] Create a trivial pybind11 function (e.g., `add(a, b)`)
- [x] Verify `pip install -e .` builds and installs successfully
- [x] Call the trivial binding from Python to confirm it works
- [x] Wire in GoogleTest with a placeholder test
- [x] Confirm `ctest` runs the placeholder test successfully

---

## Day 2: Python Reference and Environment Generation

- [x] Implement seeded environment generation in `envgen.py`
  - [x] Uniform random obstacles
  - [ ] Random box obstacles
  - [ ] 2D maze generator
- [x] Implement obstacle inflation with `scipy.ndimage.binary_dilation`
- [x] Guarantee free start and goal cells; reject unreachable configurations
- [ ] Implement pure-Python Dijkstra in `reference.py`
- [ ] Implement pure-Python A* in `reference.py`
- [ ] Write basic tests for environment generation and reference planners

---

## Day 3: C++ Grid

- [ ] Implement flat `std::vector<uint8_t>` grid storage (row-major)
- [ ] Implement padded-border construction
- [ ] Implement linear index ↔ coordinate conversion
- [ ] Implement neighbor offset/cost tables for all connectivity modes:
  - [ ] 2D 4-connected
  - [ ] 2D 8-connected
  - [ ] 3D 6-connected
  - [ ] 3D 18-connected
  - [ ] 3D 26-connected
- [ ] Implement corner-cutting constraints for diagonal moves
- [ ] Write GoogleTest cases for grid:
  - [ ] Index/coordinate round-trips
  - [ ] Neighbor tables correctness
  - [ ] Corner-cutting rules on small hand-constructed grids
  - [ ] Padded border behavior

---

## Day 4: C++ BFS and Dijkstra

- [ ] Define `PlannerConfig` (tie-breaking, expansion recording, weight)
- [ ] Define `PlanResult` (path, cost, nodes expanded, expansion order, timing)
- [ ] Define the shared planner interface
- [ ] Implement BFS
- [ ] Implement Dijkstra with lazy-deletion priority queue
- [ ] Implement optional expansion-order recording
- [ ] Implement internal `std::chrono::steady_clock` timing
- [ ] Write GoogleTest cases:
  - [ ] BFS and Dijkstra return equal costs on uniform-cost 4-connected grids
  - [ ] Unreachable goal handled correctly
  - [ ] Start == goal handled correctly
  - [ ] Start or goal inside obstacle handled correctly

---

## Day 5: C++ A* and Weighted A*

- [ ] Implement heuristics:
  - [ ] Manhattan (2D and 3D)
  - [ ] Octile (2D)
  - [ ] 3D octile
  - [ ] Euclidean
- [ ] Implement A* with configurable heuristic
- [ ] Implement weighted A* (`f = g + w·h`)
- [ ] Implement tie-breaking (prefer larger `g` on equal `f`) as a config option
- [ ] Write GoogleTest cases:
  - [ ] A* with zero heuristic behaves identically to Dijkstra
  - [ ] A* cost equals Dijkstra cost for every admissible heuristic
  - [ ] Weighted A* cost is within the `w` bound
  - [ ] Verify heuristic consistency for each cost model

---

## Day 6: Bindings and Cross-Validation

- [ ] Bind grid construction accepting `py::array_t<uint8_t>`
- [ ] Bind all planners, `PlannerConfig`, and `PlanResult`
- [ ] Expose result path and expansion order as NumPy arrays
- [ ] Release GIL during search with `py::gil_scoped_release`
- [ ] Measure search time inside C++ and end-to-end from Python
- [ ] Write pytest binding tests:
  - [ ] Incorrect dtypes raise clear exceptions
  - [ ] Non-contiguous arrays handled or rejected
  - [ ] Wrong dimensionality raises clear exceptions
- [ ] Write cross-validation suite:
  - [ ] C++ Dijkstra cost matches Python reference across hundreds of random seeds
  - [ ] C++ A* cost matches Python reference across hundreds of random seeds
  - [ ] Test both 2D and 3D grids

---

## Day 7: Correctness Hardening and Profiling

- [ ] Run full GoogleTest suite under asan-ubsan preset
- [ ] Resolve all sanitizer findings
- [ ] Run pytest suite under sanitized build
- [ ] Add Google Benchmark microbenchmarks for each planner on fixed grids:
  - [ ] BFS on 2D and 3D grids
  - [ ] Dijkstra on 2D and 3D grids
  - [ ] A* on 2D and 3D grids
  - [ ] Weighted A* on 2D and 3D grids
- [ ] Profile inner loop (e.g., `perf`, Instruments)
- [ ] Address obvious inefficiencies (unnecessary allocations, bounds checks)

---

## Day 8: 2D Visualization

- [ ] Implement expansion-order heatmaps with path overlay in `viz2d.py`
- [ ] Side-by-side comparison of all planners on one map
- [ ] Expansion animation exported as GIF or MP4
- [ ] Generate at least one animation per algorithm

---

## Day 9: 3D Visualization

- [ ] Render occupied voxels (surface voxels or point cloud) in `viz3d.py`
- [ ] Render planned path through 3D grid
- [ ] Generate at least one static 3D rendering of a planned path
- [ ] Handle large grids gracefully (surface-only rendering or downsampling)

---

## Day 10: Benchmark Sweep (Part 1)

- [ ] Write `run_benchmark.py` sweep script
- [ ] Configure sweep variables:
  - [ ] Grid sizes: 64², 256², 1024², 4096² (2D); 32³, 64³, 128³, 256³ (3D)
  - [ ] Obstacle density: 0.0, 0.1, 0.2, 0.3
  - [ ] Connectivity: all supported modes
  - [ ] Heuristic weight: 1.0, 1.5, 2.0, 5.0
  - [ ] Tie-breaking: on and off
  - [ ] Implementation: C++ and Python reference (small grids only for Python)
- [ ] Use fixed seeds for reproducibility
- [ ] Record per-run metrics: path cost, suboptimality, nodes expanded, peak open-set size, C++ search time, Python end-to-end time
- [ ] Output results to CSV
- [ ] Begin running the sweep

---

## Day 11: Benchmark Sweep (Part 2)

- [ ] Complete any remaining sweep runs
- [ ] Verify reproducibility (identical seeds → identical results)
- [ ] Compute medians and variance over seeds
- [ ] Spot-check results for anomalies or errors
- [ ] Organize raw CSV data for analysis

---

## Day 12: Analysis and Plots

- [ ] Write `analyze.py` to generate summary plots
- [ ] Plot: A* expansion reduction vs. Dijkstra by obstacle density
- [ ] Plot: Weighted A* empirical suboptimality vs. theoretical bound
- [ ] Plot: Tie-breaking effect on open vs. cluttered grids
- [ ] Plot: Runtime and memory scaling from 2D to 3D
- [ ] Plot: C++ speedup over Python reference by grid size
- [ ] Plot: Binding/conversion overhead fraction by problem size
- [ ] Plot: Profiling breakdown (heap ops, neighbor expansion, memory access)

---

## Day 13: Report

- [ ] Write `README.md` report addressing all Section 8 questions:
  - [ ] Q1: A* expansion reduction vs. Dijkstra; effect of obstacle density
  - [ ] Q2: Weighted A* empirical vs. theoretical suboptimality
  - [ ] Q3: Tie-breaking impact on different grid types
  - [ ] Q4: 2D to 3D runtime/memory scaling and implications
  - [ ] Q5: C++ vs. Python speedup; binding overhead analysis
  - [ ] Q6: Profiling results for C++ search
  - [ ] Q7: Path geometry characterization; heading changes for Project 3
- [ ] Include supporting figures in the report
- [ ] Document design decisions (e.g., template vs. runtime dimension)
- [ ] Document build and usage instructions

---

## Day 14: Polish and Optional Extension

- [ ] Final review of all tests (GoogleTest + pytest)
- [ ] Verify `pip install -e .` works cleanly from scratch
- [ ] Verify benchmark sweep reproduces from a single command
- [ ] Clean up code and remove dead code
- [ ] (Optional) Begin Theta* extension
- [ ] (Optional) Begin Jump Point Search extension
- [ ] (Optional) Begin heap alternatives comparison
- [ ] (Optional) Set up GitHub Actions CI
