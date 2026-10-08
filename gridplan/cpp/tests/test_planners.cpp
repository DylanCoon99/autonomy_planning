#include <gtest/gtest.h>
#include <set>
#include <cmath>
#include "gridplan/bfs.hpp"
#include "gridplan/dijkstra.hpp"
#include "gridplan/astar.hpp"

// Helper: convert (row, col) to padded linear index
static uint32_t pi(uint32_t row, uint32_t col, uint32_t cols) {
    return (row + 1) * (cols + 2) + (col + 1);
}

static gridplan::PlannerConfig default_config() {
    return {.tie_break = false, .record_expansions = false, .weight = 1.0f, .heuristic = gridplan::Heuristic::MANHATTAN};
}

// ==================== BFS tests ====================

TEST(BFS, AdjacentStartAndGoal) {
    std::vector<uint8_t> data(16, 0);
    gridplan::Grid grid(4, 4, data, gridplan::Connectivity::FOUR);
    gridplan::BFS bfs;

    auto result = bfs.plan(grid, pi(0, 0, 4), pi(0, 1, 4), default_config());
    EXPECT_EQ(result.path.front(), pi(0, 0, 4));
    EXPECT_EQ(result.path.back(), pi(0, 1, 4));
    EXPECT_EQ(result.path.size(), 2);
    EXPECT_FLOAT_EQ(result.cost, 1.0f);
}

TEST(BFS, StartEqualsGoal) {
    std::vector<uint8_t> data(16, 0);
    gridplan::Grid grid(4, 4, data, gridplan::Connectivity::FOUR);
    gridplan::BFS bfs;

    auto result = bfs.plan(grid, pi(1, 1, 4), pi(1, 1, 4), default_config());
    EXPECT_EQ(result.path.size(), 1);
    EXPECT_FLOAT_EQ(result.cost, 0.0f);
}

TEST(BFS, CornerToCornerEmptyGrid) {
    std::vector<uint8_t> data(16, 0);
    gridplan::Grid grid(4, 4, data, gridplan::Connectivity::FOUR);
    gridplan::BFS bfs;

    auto result = bfs.plan(grid, pi(0, 0, 4), pi(3, 3, 4), default_config());
    EXPECT_EQ(result.path.front(), pi(0, 0, 4));
    EXPECT_EQ(result.path.back(), pi(3, 3, 4));
    // Manhattan distance on 4-connected 4x4 grid
    EXPECT_FLOAT_EQ(result.cost, 6.0f);
}

TEST(BFS, ObstacleForcesDetour) {
    // Wall across row 1, columns 0-2
    //   . . . .
    //   1 1 1 .
    //   . . . .
    //   . . . .
    std::vector<uint8_t> data(16, 0);
    data[1 * 4 + 0] = 1;
    data[1 * 4 + 1] = 1;
    data[1 * 4 + 2] = 1;
    gridplan::Grid grid(4, 4, data, gridplan::Connectivity::FOUR);
    gridplan::BFS bfs;

    auto result = bfs.plan(grid, pi(0, 0, 4), pi(2, 0, 4), default_config());
    EXPECT_EQ(result.path.front(), pi(0, 0, 4));
    EXPECT_EQ(result.path.back(), pi(2, 0, 4));
    // Must go around the wall
    EXPECT_GT(result.cost, 2.0f);
}

TEST(BFS, NarrowCorridor) {
    // All walls except top row and right column
    std::vector<uint8_t> data(16, 1);
    data[0 * 4 + 0] = 0;
    data[0 * 4 + 1] = 0;
    data[0 * 4 + 2] = 0;
    data[0 * 4 + 3] = 0;
    data[1 * 4 + 3] = 0;
    data[2 * 4 + 3] = 0;
    data[3 * 4 + 3] = 0;
    gridplan::Grid grid(4, 4, data, gridplan::Connectivity::FOUR);
    gridplan::BFS bfs;

    auto result = bfs.plan(grid, pi(0, 0, 4), pi(3, 3, 4), default_config());
    EXPECT_EQ(result.path.front(), pi(0, 0, 4));
    EXPECT_EQ(result.path.back(), pi(3, 3, 4));
    EXPECT_FLOAT_EQ(result.cost, 6.0f);
}

TEST(BFS, PathIsContiguous) {
    std::vector<uint8_t> data(16, 0);
    data[1 * 4 + 1] = 1;
    data[1 * 4 + 2] = 1;
    gridplan::Grid grid(4, 4, data, gridplan::Connectivity::FOUR);
    gridplan::BFS bfs;

    auto result = bfs.plan(grid, pi(0, 0, 4), pi(3, 3, 4), default_config());
    uint32_t stride = 4 + 2;
    for (size_t i = 0; i < result.path.size() - 1; ++i) {
        int32_t diff = std::abs((int32_t)result.path[i + 1] - (int32_t)result.path[i]);
        // Adjacent in flat padded array: diff should be 1 (horizontal) or stride (vertical)
        EXPECT_TRUE(diff == 1 || diff == (int32_t)stride);
    }
}

TEST(BFS, PathAvoidsObstacles) {
    std::vector<uint8_t> data(16, 0);
    data[1 * 4 + 0] = 1;
    data[1 * 4 + 1] = 1;
    data[1 * 4 + 2] = 1;
    gridplan::Grid grid(4, 4, data, gridplan::Connectivity::FOUR);
    gridplan::BFS bfs;

    auto result = bfs.plan(grid, pi(0, 0, 4), pi(3, 3, 4), default_config());
    std::set<uint32_t> obstacles = {pi(1, 0, 4), pi(1, 1, 4), pi(1, 2, 4)};
    for (uint32_t idx : result.path) {
        EXPECT_FALSE(obstacles.count(idx));
    }
}

TEST(BFS, NodesExpandedIsPositive) {
    std::vector<uint8_t> data(16, 0);
    gridplan::Grid grid(4, 4, data, gridplan::Connectivity::FOUR);
    gridplan::BFS bfs;

    auto result = bfs.plan(grid, pi(0, 0, 4), pi(3, 3, 4), default_config());
    EXPECT_GT(result.nodes_expanded, 0u);
}

TEST(BFS, PathLengthEqualsCostPlusOne) {
    // On 4-connected with uniform cost, path length = cost + 1
    std::vector<uint8_t> data(16, 0);
    gridplan::Grid grid(4, 4, data, gridplan::Connectivity::FOUR);
    gridplan::BFS bfs;

    auto result = bfs.plan(grid, pi(0, 0, 4), pi(3, 3, 4), default_config());
    EXPECT_EQ(result.path.size(), (size_t)(result.cost + 1));
}

// ==================== Dijkstra tests ====================

TEST(Dijkstra, AdjacentStartAndGoal) {
    std::vector<uint8_t> data(16, 0);
    gridplan::Grid grid(4, 4, data, gridplan::Connectivity::FOUR);
    gridplan::Dijkstra dijkstra;

    auto result = dijkstra.plan(grid, pi(0, 0, 4), pi(0, 1, 4), default_config());
    EXPECT_EQ(result.path.front(), pi(0, 0, 4));
    EXPECT_EQ(result.path.back(), pi(0, 1, 4));
    EXPECT_EQ(result.path.size(), 2);
    EXPECT_FLOAT_EQ(result.cost, 1.0f);
}

TEST(Dijkstra, StartEqualsGoal) {
    std::vector<uint8_t> data(16, 0);
    gridplan::Grid grid(4, 4, data, gridplan::Connectivity::FOUR);
    gridplan::Dijkstra dijkstra;

    auto result = dijkstra.plan(grid, pi(1, 1, 4), pi(1, 1, 4), default_config());
    EXPECT_EQ(result.path.size(), 1);
    EXPECT_FLOAT_EQ(result.cost, 0.0f);
}

TEST(Dijkstra, CornerToCornerFourConnected) {
    std::vector<uint8_t> data(16, 0);
    gridplan::Grid grid(4, 4, data, gridplan::Connectivity::FOUR);
    gridplan::Dijkstra dijkstra;

    auto result = dijkstra.plan(grid, pi(0, 0, 4), pi(3, 3, 4), default_config());
    EXPECT_EQ(result.path.front(), pi(0, 0, 4));
    EXPECT_EQ(result.path.back(), pi(3, 3, 4));
    EXPECT_FLOAT_EQ(result.cost, 6.0f);
}

TEST(Dijkstra, CornerToCornerEightConnected) {
    // On 8-connected, diagonal cost is sqrt(2). Optimal path from (0,0) to (3,3)
    // is 3 diagonal steps, cost = 3*sqrt(2)
    std::vector<uint8_t> data(16, 0);
    gridplan::Grid grid(4, 4, data, gridplan::Connectivity::EIGHT);
    gridplan::Dijkstra dijkstra;

    auto result = dijkstra.plan(grid, pi(0, 0, 4), pi(3, 3, 4), default_config());
    EXPECT_EQ(result.path.front(), pi(0, 0, 4));
    EXPECT_EQ(result.path.back(), pi(3, 3, 4));
    EXPECT_NEAR(result.cost, 3.0f * 1.414f, 0.01f);
}

TEST(Dijkstra, CostMatchesBFSOnFourConnected) {
    // On uniform 4-connected, Dijkstra and BFS should find the same cost
    std::vector<uint8_t> data(16, 0);
    data[1 * 4 + 1] = 1;
    data[1 * 4 + 2] = 1;
    gridplan::Grid grid(4, 4, data, gridplan::Connectivity::FOUR);
    gridplan::Dijkstra dijkstra;
    gridplan::BFS bfs;

    auto d_result = dijkstra.plan(grid, pi(0, 0, 4), pi(3, 3, 4), default_config());
    auto b_result = bfs.plan(grid, pi(0, 0, 4), pi(3, 3, 4), default_config());
    EXPECT_FLOAT_EQ(d_result.cost, b_result.cost);
}

TEST(Dijkstra, ObstacleForcesDetour) {
    std::vector<uint8_t> data(16, 0);
    data[1 * 4 + 0] = 1;
    data[1 * 4 + 1] = 1;
    data[1 * 4 + 2] = 1;
    gridplan::Grid grid(4, 4, data, gridplan::Connectivity::FOUR);
    gridplan::Dijkstra dijkstra;

    auto result = dijkstra.plan(grid, pi(0, 0, 4), pi(2, 0, 4), default_config());
    EXPECT_EQ(result.path.front(), pi(0, 0, 4));
    EXPECT_EQ(result.path.back(), pi(2, 0, 4));
    EXPECT_GT(result.cost, 2.0f);
}

TEST(Dijkstra, NarrowCorridor) {
    std::vector<uint8_t> data(16, 1);
    data[0 * 4 + 0] = 0;
    data[0 * 4 + 1] = 0;
    data[0 * 4 + 2] = 0;
    data[0 * 4 + 3] = 0;
    data[1 * 4 + 3] = 0;
    data[2 * 4 + 3] = 0;
    data[3 * 4 + 3] = 0;
    gridplan::Grid grid(4, 4, data, gridplan::Connectivity::FOUR);
    gridplan::Dijkstra dijkstra;

    auto result = dijkstra.plan(grid, pi(0, 0, 4), pi(3, 3, 4), default_config());
    EXPECT_FLOAT_EQ(result.cost, 6.0f);
}

TEST(Dijkstra, PathAvoidsObstacles) {
    std::vector<uint8_t> data(16, 0);
    data[1 * 4 + 0] = 1;
    data[1 * 4 + 1] = 1;
    data[1 * 4 + 2] = 1;
    gridplan::Grid grid(4, 4, data, gridplan::Connectivity::FOUR);
    gridplan::Dijkstra dijkstra;

    auto result = dijkstra.plan(grid, pi(0, 0, 4), pi(3, 3, 4), default_config());
    std::set<uint32_t> obstacles = {pi(1, 0, 4), pi(1, 1, 4), pi(1, 2, 4)};
    for (uint32_t idx : result.path) {
        EXPECT_FALSE(obstacles.count(idx));
    }
}

TEST(Dijkstra, PathIsContiguous) {
    std::vector<uint8_t> data(16, 0);
    data[1 * 4 + 1] = 1;
    gridplan::Grid grid(4, 4, data, gridplan::Connectivity::FOUR);
    gridplan::Dijkstra dijkstra;

    auto result = dijkstra.plan(grid, pi(0, 0, 4), pi(3, 3, 4), default_config());
    uint32_t stride = 4 + 2;
    for (size_t i = 0; i < result.path.size() - 1; ++i) {
        int32_t diff = std::abs((int32_t)result.path[i + 1] - (int32_t)result.path[i]);
        EXPECT_TRUE(diff == 1 || diff == (int32_t)stride);
    }
}

TEST(Dijkstra, NodesExpandedIsPositive) {
    std::vector<uint8_t> data(16, 0);
    gridplan::Grid grid(4, 4, data, gridplan::Connectivity::FOUR);
    gridplan::Dijkstra dijkstra;

    auto result = dijkstra.plan(grid, pi(0, 0, 4), pi(3, 3, 4), default_config());
    EXPECT_GT(result.nodes_expanded, 0u);
}

TEST(Dijkstra, EightConnectedCheaperThanFourConnected) {
    // 8-connected can take diagonals, so cost should be <= 4-connected
    std::vector<uint8_t> data(16, 0);
    gridplan::Grid grid4(4, 4, data, gridplan::Connectivity::FOUR);
    gridplan::Grid grid8(4, 4, data, gridplan::Connectivity::EIGHT);
    gridplan::Dijkstra dijkstra;

    auto r4 = dijkstra.plan(grid4, pi(0, 0, 4), pi(3, 3, 4), default_config());
    auto r8 = dijkstra.plan(grid8, pi(0, 0, 4), pi(3, 3, 4), default_config());
    EXPECT_LE(r8.cost, r4.cost);
}

// ==================== Edge case tests ====================
// These test unreachable goals and start/goal inside obstacles.
// Expected behavior: empty path and cost of -1.0f.

TEST(BFS, UnreachableGoal) {
    // Wall splits the grid in half vertically
    std::vector<uint8_t> data(16, 0);
    data[0 * 4 + 2] = 1;
    data[1 * 4 + 2] = 1;
    data[2 * 4 + 2] = 1;
    data[3 * 4 + 2] = 1;
    gridplan::Grid grid(4, 4, data, gridplan::Connectivity::FOUR);
    gridplan::BFS bfs;

    auto result = bfs.plan(grid, pi(0, 0, 4), pi(0, 3, 4), default_config());
    EXPECT_TRUE(result.path.empty());
    EXPECT_FLOAT_EQ(result.cost, -1.0f);
}

TEST(BFS, StartInsideObstacle) {
    std::vector<uint8_t> data(16, 0);
    data[0 * 4 + 0] = 1;  // (0,0) is obstacle
    gridplan::Grid grid(4, 4, data, gridplan::Connectivity::FOUR);
    gridplan::BFS bfs;

    auto result = bfs.plan(grid, pi(0, 0, 4), pi(3, 3, 4), default_config());
    EXPECT_TRUE(result.path.empty());
    EXPECT_FLOAT_EQ(result.cost, -1.0f);
}

TEST(BFS, GoalInsideObstacle) {
    std::vector<uint8_t> data(16, 0);
    data[3 * 4 + 3] = 1;  // (3,3) is obstacle
    gridplan::Grid grid(4, 4, data, gridplan::Connectivity::FOUR);
    gridplan::BFS bfs;

    auto result = bfs.plan(grid, pi(0, 0, 4), pi(3, 3, 4), default_config());
    EXPECT_TRUE(result.path.empty());
    EXPECT_FLOAT_EQ(result.cost, -1.0f);
}

TEST(Dijkstra, UnreachableGoal) {
    std::vector<uint8_t> data(16, 0);
    data[0 * 4 + 2] = 1;
    data[1 * 4 + 2] = 1;
    data[2 * 4 + 2] = 1;
    data[3 * 4 + 2] = 1;
    gridplan::Grid grid(4, 4, data, gridplan::Connectivity::FOUR);
    gridplan::Dijkstra dijkstra;

    auto result = dijkstra.plan(grid, pi(0, 0, 4), pi(0, 3, 4), default_config());
    EXPECT_TRUE(result.path.empty());
    EXPECT_FLOAT_EQ(result.cost, -1.0f);
}

TEST(Dijkstra, StartInsideObstacle) {
    std::vector<uint8_t> data(16, 0);
    data[0 * 4 + 0] = 1;
    gridplan::Grid grid(4, 4, data, gridplan::Connectivity::FOUR);
    gridplan::Dijkstra dijkstra;

    auto result = dijkstra.plan(grid, pi(0, 0, 4), pi(3, 3, 4), default_config());
    EXPECT_TRUE(result.path.empty());
    EXPECT_FLOAT_EQ(result.cost, -1.0f);
}

TEST(Dijkstra, GoalInsideObstacle) {
    std::vector<uint8_t> data(16, 0);
    data[3 * 4 + 3] = 1;
    gridplan::Grid grid(4, 4, data, gridplan::Connectivity::FOUR);
    gridplan::Dijkstra dijkstra;

    auto result = dijkstra.plan(grid, pi(0, 0, 4), pi(3, 3, 4), default_config());
    EXPECT_TRUE(result.path.empty());
    EXPECT_FLOAT_EQ(result.cost, -1.0f);
}

// ==================== A* tests ====================

TEST(AStar, AdjacentStartAndGoal) {
    std::vector<uint8_t> data(16, 0);
    gridplan::Grid grid(4, 4, data, gridplan::Connectivity::FOUR);
    gridplan::AStar astar;

    auto result = astar.plan(grid, pi(0, 0, 4), pi(0, 1, 4), default_config());
    EXPECT_EQ(result.path.front(), pi(0, 0, 4));
    EXPECT_EQ(result.path.back(), pi(0, 1, 4));
    EXPECT_EQ(result.path.size(), 2);
    EXPECT_FLOAT_EQ(result.cost, 1.0f);
}

TEST(AStar, StartEqualsGoal) {
    std::vector<uint8_t> data(16, 0);
    gridplan::Grid grid(4, 4, data, gridplan::Connectivity::FOUR);
    gridplan::AStar astar;

    auto result = astar.plan(grid, pi(1, 1, 4), pi(1, 1, 4), default_config());
    EXPECT_EQ(result.path.size(), 1);
    EXPECT_FLOAT_EQ(result.cost, 0.0f);
}

TEST(AStar, CornerToCornerFourConnected) {
    std::vector<uint8_t> data(16, 0);
    gridplan::Grid grid(4, 4, data, gridplan::Connectivity::FOUR);
    gridplan::AStar astar;

    auto result = astar.plan(grid, pi(0, 0, 4), pi(3, 3, 4), default_config());
    EXPECT_FLOAT_EQ(result.cost, 6.0f);
}

TEST(AStar, CostMatchesDijkstraFourConnected) {
    std::vector<uint8_t> data(16, 0);
    data[1 * 4 + 1] = 1;
    data[1 * 4 + 2] = 1;
    data[2 * 4 + 0] = 1;
    gridplan::Grid grid(4, 4, data, gridplan::Connectivity::FOUR);
    gridplan::AStar astar;
    gridplan::Dijkstra dijkstra;

    auto a_result = astar.plan(grid, pi(0, 0, 4), pi(3, 3, 4), default_config());
    auto d_result = dijkstra.plan(grid, pi(0, 0, 4), pi(3, 3, 4), default_config());
    EXPECT_FLOAT_EQ(a_result.cost, d_result.cost);
}

TEST(AStar, CostMatchesDijkstraEightConnected) {
    std::vector<uint8_t> data(16, 0);
    data[1 * 4 + 1] = 1;
    data[2 * 4 + 2] = 1;
    gridplan::Grid grid(4, 4, data, gridplan::Connectivity::EIGHT);
    gridplan::AStar astar;
    gridplan::Dijkstra dijkstra;

    auto cfg = default_config();
    cfg.heuristic = gridplan::Heuristic::OCTILE;

    auto a_result = astar.plan(grid, pi(0, 0, 4), pi(3, 3, 4), cfg);
    auto d_result = dijkstra.plan(grid, pi(0, 0, 4), pi(3, 3, 4), cfg);
    EXPECT_NEAR(a_result.cost, d_result.cost, 0.01f);
}

TEST(AStar, FewerExpansionsThanDijkstra) {
    std::vector<uint8_t> data(16, 0);
    gridplan::Grid grid(4, 4, data, gridplan::Connectivity::FOUR);
    gridplan::AStar astar;
    gridplan::Dijkstra dijkstra;

    auto a_result = astar.plan(grid, pi(0, 0, 4), pi(3, 3, 4), default_config());
    auto d_result = dijkstra.plan(grid, pi(0, 0, 4), pi(3, 3, 4), default_config());
    EXPECT_LE(a_result.nodes_expanded, d_result.nodes_expanded);
}

TEST(AStar, ZeroHeuristicMatchesDijkstra) {
    std::vector<uint8_t> data(16, 0);
    data[1 * 4 + 0] = 1;
    data[1 * 4 + 1] = 1;
    data[1 * 4 + 2] = 1;
    gridplan::Grid grid(4, 4, data, gridplan::Connectivity::FOUR);
    gridplan::AStar astar;
    gridplan::Dijkstra dijkstra;

    auto cfg = default_config();
    cfg.heuristic = gridplan::Heuristic::ZERO;

    auto a_result = astar.plan(grid, pi(0, 0, 4), pi(3, 3, 4), cfg);
    auto d_result = dijkstra.plan(grid, pi(0, 0, 4), pi(3, 3, 4), default_config());
    EXPECT_FLOAT_EQ(a_result.cost, d_result.cost);
}

TEST(AStar, PathAvoidsObstacles) {
    std::vector<uint8_t> data(16, 0);
    data[1 * 4 + 0] = 1;
    data[1 * 4 + 1] = 1;
    data[1 * 4 + 2] = 1;
    gridplan::Grid grid(4, 4, data, gridplan::Connectivity::FOUR);
    gridplan::AStar astar;

    auto result = astar.plan(grid, pi(0, 0, 4), pi(3, 3, 4), default_config());
    std::set<uint32_t> obstacles = {pi(1, 0, 4), pi(1, 1, 4), pi(1, 2, 4)};
    for (uint32_t idx : result.path) {
        EXPECT_FALSE(obstacles.count(idx));
    }
}

TEST(AStar, PathIsContiguous) {
    std::vector<uint8_t> data(16, 0);
    data[1 * 4 + 1] = 1;
    gridplan::Grid grid(4, 4, data, gridplan::Connectivity::FOUR);
    gridplan::AStar astar;

    auto result = astar.plan(grid, pi(0, 0, 4), pi(3, 3, 4), default_config());
    uint32_t stride = 4 + 2;
    for (size_t i = 0; i < result.path.size() - 1; ++i) {
        int32_t diff = std::abs((int32_t)result.path[i + 1] - (int32_t)result.path[i]);
        EXPECT_TRUE(diff == 1 || diff == (int32_t)stride);
    }
}

TEST(AStar, UnreachableGoal) {
    std::vector<uint8_t> data(16, 0);
    data[0 * 4 + 2] = 1;
    data[1 * 4 + 2] = 1;
    data[2 * 4 + 2] = 1;
    data[3 * 4 + 2] = 1;
    gridplan::Grid grid(4, 4, data, gridplan::Connectivity::FOUR);
    gridplan::AStar astar;

    auto result = astar.plan(grid, pi(0, 0, 4), pi(0, 3, 4), default_config());
    EXPECT_TRUE(result.path.empty());
    EXPECT_FLOAT_EQ(result.cost, -1.0f);
}

TEST(AStar, StartInsideObstacle) {
    std::vector<uint8_t> data(16, 0);
    data[0 * 4 + 0] = 1;
    gridplan::Grid grid(4, 4, data, gridplan::Connectivity::FOUR);
    gridplan::AStar astar;

    auto result = astar.plan(grid, pi(0, 0, 4), pi(3, 3, 4), default_config());
    EXPECT_TRUE(result.path.empty());
    EXPECT_FLOAT_EQ(result.cost, -1.0f);
}

// ==================== Weighted A* tests ====================

TEST(WeightedAStar, CostWithinWBound) {
    // Weighted A* cost should be <= w * optimal cost
    std::vector<uint8_t> data(16, 0);
    data[1 * 4 + 1] = 1;
    data[2 * 4 + 2] = 1;
    gridplan::Grid grid(4, 4, data, gridplan::Connectivity::FOUR);
    gridplan::AStar astar;
    gridplan::Dijkstra dijkstra;

    float w = 2.0f;
    auto cfg = default_config();
    cfg.weight = w;

    auto optimal = dijkstra.plan(grid, pi(0, 0, 4), pi(3, 3, 4), default_config());
    auto weighted = astar.plan(grid, pi(0, 0, 4), pi(3, 3, 4), cfg);

    EXPECT_LE(weighted.cost, w * optimal.cost);
    EXPECT_GE(weighted.cost, optimal.cost);
}

TEST(WeightedAStar, Weight5CostWithinBound) {
    std::vector<uint8_t> data(16, 0);
    data[1 * 4 + 0] = 1;
    data[1 * 4 + 1] = 1;
    data[1 * 4 + 2] = 1;
    gridplan::Grid grid(4, 4, data, gridplan::Connectivity::FOUR);
    gridplan::AStar astar;
    gridplan::Dijkstra dijkstra;

    float w = 5.0f;
    auto cfg = default_config();
    cfg.weight = w;

    auto optimal = dijkstra.plan(grid, pi(0, 0, 4), pi(3, 3, 4), default_config());
    auto weighted = astar.plan(grid, pi(0, 0, 4), pi(3, 3, 4), cfg);

    EXPECT_LE(weighted.cost, w * optimal.cost);
}

TEST(WeightedAStar, Weight1MatchesAStar) {
    // weight=1 should give the same optimal cost as regular A*
    std::vector<uint8_t> data(16, 0);
    data[1 * 4 + 1] = 1;
    gridplan::Grid grid(4, 4, data, gridplan::Connectivity::FOUR);
    gridplan::AStar astar;

    auto cfg_w1 = default_config();
    cfg_w1.weight = 1.0f;

    auto r1 = astar.plan(grid, pi(0, 0, 4), pi(3, 3, 4), cfg_w1);
    auto r2 = astar.plan(grid, pi(0, 0, 4), pi(3, 3, 4), default_config());
    EXPECT_FLOAT_EQ(r1.cost, r2.cost);
}
