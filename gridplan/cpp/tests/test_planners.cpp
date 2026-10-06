#include <gtest/gtest.h>
#include <set>
#include "gridplan/bfs.hpp"

// Helper: convert (row, col) to padded linear index
static uint32_t pi(uint32_t row, uint32_t col, uint32_t cols) {
    return (row + 1) * (cols + 2) + (col + 1);
}

static gridplan::PlannerConfig default_config() {
    return {.tie_break = false, .record_expansions = false, .weight = 1.0f};
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
