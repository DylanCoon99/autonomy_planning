cmake_minimum_required(VERSION 3.24)
project(gridplan LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ---------- Dependencies via FetchContent ----------
include(FetchContent)

FetchContent_Declare(
  pybind11
  GIT_REPOSITORY https://github.com/pybind/pybind11.git
  GIT_TAG        v2.13.6
)

FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        v1.15.2
)

FetchContent_Declare(
  googlebenchmark
  GIT_REPOSITORY https://github.com/google/benchmark.git
  GIT_TAG        v1.9.1
)

# Prevent benchmark from building its own tests
set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(pybind11 googletest googlebenchmark)

# ---------- Core C++ library ----------
add_library(gridplan_core
  cpp/src/grid.cpp
  cpp/src/bfs.cpp
  cpp/src/dijkstra.cpp
  cpp/src/astar.cpp
)
target_include_directories(gridplan_core PUBLIC cpp/include)
target_compile_features(gridplan_core PUBLIC cxx_std_20)

# ---------- pybind11 module ----------
pybind11_add_module(_gridplan_core bindings/module.cpp)
target_link_libraries(_gridplan_core PRIVATE gridplan_core)
install(TARGETS _gridplan_core DESTINATION .)

# ---------- GoogleTest ----------
enable_testing()

add_executable(test_grid cpp/tests/test_grid.cpp)
target_link_libraries(test_grid PRIVATE gridplan_core GTest::gtest_main)
add_test(NAME test_grid COMMAND test_grid)

add_executable(test_planners cpp/tests/test_planners.cpp)
target_link_libraries(test_planners PRIVATE gridplan_core GTest::gtest_main)
add_test(NAME test_planners COMMAND test_planners)

# ---------- Sandbox ----------
add_executable(sandbox cpp/src/main.cpp)
target_link_libraries(sandbox PRIVATE gridplan_core)

# ---------- Google Benchmark ----------
add_executable(bench_planners cpp/bench/bench_planners.cpp)
target_link_libraries(bench_planners PRIVATE gridplan_core benchmark::benchmark_main)
