#include <benchmark/benchmark.h>
#include "gridplan/grid.hpp"
#include "gridplan/planner.hpp"
#include "gridplan/bfs.hpp"
#include "gridplan/dijkstra.hpp"
#include "gridplan/astar.hpp"

static uint32_t padded_index(uint32_t row, uint32_t col, uint32_t cols) {
    return (row + 1) * (cols + 2) + (col + 1);
}

static gridplan::PlannerConfig config_no_record(gridplan::Heuristic h = gridplan::Heuristic::MANHATTAN) {
    return {false, false, 1.0f, h};
}

static gridplan::PlannerConfig config_weighted(float w, gridplan::Heuristic h = gridplan::Heuristic::MANHATTAN) {
    return {false, false, w, h};
}

// ==================== 2D BFS ====================

static void BM_BFS_2D_Four(benchmark::State& state) {
    int size = state.range(0);
    std::vector<uint8_t> data(size * size, 0);
    gridplan::Grid grid(size, size, data, gridplan::Connectivity::FOUR);
    gridplan::BFS bfs;
    auto cfg = config_no_record();
    uint32_t start = padded_index(0, 0, size);
    uint32_t goal = padded_index(size - 1, size - 1, size);

    for (auto _ : state) {
        auto result = bfs.plan(grid, start, goal, cfg);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_BFS_2D_Four)->Arg(64)->Arg(256)->Arg(1024);

static void BM_BFS_2D_Eight(benchmark::State& state) {
    int size = state.range(0);
    std::vector<uint8_t> data(size * size, 0);
    gridplan::Grid grid(size, size, data, gridplan::Connectivity::EIGHT);
    gridplan::BFS bfs;
    auto cfg = config_no_record();
    uint32_t start = padded_index(0, 0, size);
    uint32_t goal = padded_index(size - 1, size - 1, size);

    for (auto _ : state) {
        auto result = bfs.plan(grid, start, goal, cfg);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_BFS_2D_Eight)->Arg(64)->Arg(256)->Arg(1024);

// ==================== 2D Dijkstra ====================

static void BM_Dijkstra_2D_Four(benchmark::State& state) {
    int size = state.range(0);
    std::vector<uint8_t> data(size * size, 0);
    gridplan::Grid grid(size, size, data, gridplan::Connectivity::FOUR);
    gridplan::Dijkstra dijkstra;
    auto cfg = config_no_record();
    uint32_t start = padded_index(0, 0, size);
    uint32_t goal = padded_index(size - 1, size - 1, size);

    for (auto _ : state) {
        auto result = dijkstra.plan(grid, start, goal, cfg);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_Dijkstra_2D_Four)->Arg(64)->Arg(256)->Arg(1024);

static void BM_Dijkstra_2D_Eight(benchmark::State& state) {
    int size = state.range(0);
    std::vector<uint8_t> data(size * size, 0);
    gridplan::Grid grid(size, size, data, gridplan::Connectivity::EIGHT);
    gridplan::Dijkstra dijkstra;
    auto cfg = config_no_record(gridplan::Heuristic::OCTILE);
    uint32_t start = padded_index(0, 0, size);
    uint32_t goal = padded_index(size - 1, size - 1, size);

    for (auto _ : state) {
        auto result = dijkstra.plan(grid, start, goal, cfg);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_Dijkstra_2D_Eight)->Arg(64)->Arg(256)->Arg(1024);

// ==================== 2D A* ====================

static void BM_AStar_2D_Four_Manhattan(benchmark::State& state) {
    int size = state.range(0);
    std::vector<uint8_t> data(size * size, 0);
    gridplan::Grid grid(size, size, data, gridplan::Connectivity::FOUR);
    gridplan::AStar astar;
    auto cfg = config_no_record(gridplan::Heuristic::MANHATTAN);
    uint32_t start = padded_index(0, 0, size);
    uint32_t goal = padded_index(size - 1, size - 1, size);

    for (auto _ : state) {
        auto result = astar.plan(grid, start, goal, cfg);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_AStar_2D_Four_Manhattan)->Arg(64)->Arg(256)->Arg(1024);

static void BM_AStar_2D_Eight_Octile(benchmark::State& state) {
    int size = state.range(0);
    std::vector<uint8_t> data(size * size, 0);
    gridplan::Grid grid(size, size, data, gridplan::Connectivity::EIGHT);
    gridplan::AStar astar;
    auto cfg = config_no_record(gridplan::Heuristic::OCTILE);
    uint32_t start = padded_index(0, 0, size);
    uint32_t goal = padded_index(size - 1, size - 1, size);

    for (auto _ : state) {
        auto result = astar.plan(grid, start, goal, cfg);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_AStar_2D_Eight_Octile)->Arg(64)->Arg(256)->Arg(1024);

static void BM_AStar_2D_Four_Euclidean(benchmark::State& state) {
    int size = state.range(0);
    std::vector<uint8_t> data(size * size, 0);
    gridplan::Grid grid(size, size, data, gridplan::Connectivity::FOUR);
    gridplan::AStar astar;
    auto cfg = config_no_record(gridplan::Heuristic::EUCLIDEAN);
    uint32_t start = padded_index(0, 0, size);
    uint32_t goal = padded_index(size - 1, size - 1, size);

    for (auto _ : state) {
        auto result = astar.plan(grid, start, goal, cfg);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_AStar_2D_Four_Euclidean)->Arg(64)->Arg(256)->Arg(1024);

// ==================== 2D Weighted A* ====================

static void BM_WeightedAStar_2D_W1_5(benchmark::State& state) {
    int size = state.range(0);
    std::vector<uint8_t> data(size * size, 0);
    gridplan::Grid grid(size, size, data, gridplan::Connectivity::FOUR);
    gridplan::AStar astar;
    auto cfg = config_weighted(1.5f);
    uint32_t start = padded_index(0, 0, size);
    uint32_t goal = padded_index(size - 1, size - 1, size);

    for (auto _ : state) {
        auto result = astar.plan(grid, start, goal, cfg);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_WeightedAStar_2D_W1_5)->Arg(64)->Arg(256)->Arg(1024);

static void BM_WeightedAStar_2D_W2(benchmark::State& state) {
    int size = state.range(0);
    std::vector<uint8_t> data(size * size, 0);
    gridplan::Grid grid(size, size, data, gridplan::Connectivity::FOUR);
    gridplan::AStar astar;
    auto cfg = config_weighted(2.0f);
    uint32_t start = padded_index(0, 0, size);
    uint32_t goal = padded_index(size - 1, size - 1, size);

    for (auto _ : state) {
        auto result = astar.plan(grid, start, goal, cfg);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_WeightedAStar_2D_W2)->Arg(64)->Arg(256)->Arg(1024);

static void BM_WeightedAStar_2D_W5(benchmark::State& state) {
    int size = state.range(0);
    std::vector<uint8_t> data(size * size, 0);
    gridplan::Grid grid(size, size, data, gridplan::Connectivity::FOUR);
    gridplan::AStar astar;
    auto cfg = config_weighted(5.0f);
    uint32_t start = padded_index(0, 0, size);
    uint32_t goal = padded_index(size - 1, size - 1, size);

    for (auto _ : state) {
        auto result = astar.plan(grid, start, goal, cfg);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_WeightedAStar_2D_W5)->Arg(64)->Arg(256)->Arg(1024);

// ==================== 3D Dijkstra ====================

static void BM_Dijkstra_3D_Six(benchmark::State& state) {
    int size = state.range(0);
    std::vector<uint8_t> data(size * size * size, 0);
    gridplan::Grid grid(size, size, size, data, gridplan::Connectivity::SIX);
    gridplan::Dijkstra dijkstra;
    auto cfg = config_no_record();

    uint32_t padded_cols = size + 2;
    uint32_t z_stride = (size + 2) * (size + 2);
    uint32_t start = 1 * z_stride + 1 * padded_cols + 1;
    uint32_t goal = size * z_stride + size * padded_cols + size;

    for (auto _ : state) {
        auto result = dijkstra.plan(grid, start, goal, cfg);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_Dijkstra_3D_Six)->Arg(32)->Arg(64);

// ==================== 3D A* ====================

static void BM_AStar_3D_Six_Manhattan(benchmark::State& state) {
    int size = state.range(0);
    std::vector<uint8_t> data(size * size * size, 0);
    gridplan::Grid grid(size, size, size, data, gridplan::Connectivity::SIX);
    gridplan::AStar astar;
    auto cfg = config_no_record(gridplan::Heuristic::MANHATTAN);

    uint32_t padded_cols = size + 2;
    uint32_t z_stride = (size + 2) * (size + 2);
    uint32_t start = 1 * z_stride + 1 * padded_cols + 1;
    uint32_t goal = size * z_stride + size * padded_cols + size;

    for (auto _ : state) {
        auto result = astar.plan(grid, start, goal, cfg);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_AStar_3D_Six_Manhattan)->Arg(32)->Arg(64);

static void BM_AStar_3D_TwentySix_Octile(benchmark::State& state) {
    int size = state.range(0);
    std::vector<uint8_t> data(size * size * size, 0);
    gridplan::Grid grid(size, size, size, data, gridplan::Connectivity::TWENTYSIX);
    gridplan::AStar astar;
    auto cfg = config_no_record(gridplan::Heuristic::OCTILE);

    uint32_t padded_cols = size + 2;
    uint32_t z_stride = (size + 2) * (size + 2);
    uint32_t start = 1 * z_stride + 1 * padded_cols + 1;
    uint32_t goal = size * z_stride + size * padded_cols + size;

    for (auto _ : state) {
        auto result = astar.plan(grid, start, goal, cfg);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_AStar_3D_TwentySix_Octile)->Arg(32)->Arg(64);
