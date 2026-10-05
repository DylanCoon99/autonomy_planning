#include <gtest/gtest.h>
#include "gridplan/grid.hpp"
#include <algorithm>
#include <set>

// Helper: convert (row, col) to padded linear index
static uint32_t padded_index(uint32_t row, uint32_t col, uint32_t cols) {
    uint32_t stride = cols + 2;
    return (row + 1) * stride + (col + 1);
}

// Helper: extract just the indices from get_neighbors result
static std::set<uint32_t> neighbor_indices(const std::vector<std::pair<int32_t, float>>& neighbors) {
    std::set<uint32_t> indices;
    for (const auto& [idx, cost] : neighbors) {
        indices.insert(idx);
    }
    return indices;
}

// ==================== 4-connected tests ====================

TEST(GetNeighbors4, CenterOfOpenGrid) {
    // 4x4 grid, all free
    std::vector<uint8_t> data(16, 0);
    gridplan::Grid g(4, 4, data, gridplan::Connectivity::FOUR);

    auto neighbors = g.get_neighbors(padded_index(1, 1, 4));
    EXPECT_EQ(neighbors.size(), 4);

    auto indices = neighbor_indices(neighbors);
    EXPECT_TRUE(indices.count(padded_index(0, 1, 4)));  // up
    EXPECT_TRUE(indices.count(padded_index(2, 1, 4)));  // down
    EXPECT_TRUE(indices.count(padded_index(1, 0, 4)));  // left
    EXPECT_TRUE(indices.count(padded_index(1, 2, 4)));  // right
}

TEST(GetNeighbors4, CornerCell) {
    // (0,0) should have only 2 neighbors (right and down); up and left hit padding
    std::vector<uint8_t> data(16, 0);
    gridplan::Grid g(4, 4, data, gridplan::Connectivity::FOUR);

    auto neighbors = g.get_neighbors(padded_index(0, 0, 4));
    EXPECT_EQ(neighbors.size(), 2);

    auto indices = neighbor_indices(neighbors);
    EXPECT_TRUE(indices.count(padded_index(0, 1, 4)));  // right
    EXPECT_TRUE(indices.count(padded_index(1, 0, 4)));  // down
}

TEST(GetNeighbors4, EdgeCell) {
    // (0,1) top edge: 3 neighbors (left, right, down)
    std::vector<uint8_t> data(16, 0);
    gridplan::Grid g(4, 4, data, gridplan::Connectivity::FOUR);

    auto neighbors = g.get_neighbors(padded_index(0, 1, 4));
    EXPECT_EQ(neighbors.size(), 3);
}

TEST(GetNeighbors4, ObstacleBlocksNeighbor) {
    // Place obstacle to the right of (1,1)
    std::vector<uint8_t> data(16, 0);
    data[1 * 4 + 2] = 1;  // (1,2) is obstacle
    gridplan::Grid g(4, 4, data, gridplan::Connectivity::FOUR);

    auto neighbors = g.get_neighbors(padded_index(1, 1, 4));
    EXPECT_EQ(neighbors.size(), 3);

    auto indices = neighbor_indices(neighbors);
    EXPECT_FALSE(indices.count(padded_index(1, 2, 4)));  // blocked
}

TEST(GetNeighbors4, SurroundedByObstacles) {
    // (1,1) surrounded on all 4 sides
    std::vector<uint8_t> data(16, 0);
    data[0 * 4 + 1] = 1;  // (0,1)
    data[2 * 4 + 1] = 1;  // (2,1)
    data[1 * 4 + 0] = 1;  // (1,0)
    data[1 * 4 + 2] = 1;  // (1,2)
    gridplan::Grid g(4, 4, data, gridplan::Connectivity::FOUR);

    auto neighbors = g.get_neighbors(padded_index(1, 1, 4));
    EXPECT_EQ(neighbors.size(), 0);
}

TEST(GetNeighbors4, AllCostsAreOne) {
    std::vector<uint8_t> data(16, 0);
    gridplan::Grid g(4, 4, data, gridplan::Connectivity::FOUR);

    auto neighbors = g.get_neighbors(padded_index(1, 1, 4));
    for (const auto& [idx, cost] : neighbors) {
        EXPECT_FLOAT_EQ(cost, 1.0f);
    }
}

// ==================== 8-connected tests ====================

TEST(GetNeighbors8, CenterOfOpenGrid) {
    std::vector<uint8_t> data(16, 0);
    gridplan::Grid g(4, 4, data, gridplan::Connectivity::EIGHT);

    auto neighbors = g.get_neighbors(padded_index(1, 1, 4));
    EXPECT_EQ(neighbors.size(), 8);
}

TEST(GetNeighbors8, CornerCell) {
    // (0,0): right, down, and down-right
    std::vector<uint8_t> data(16, 0);
    gridplan::Grid g(4, 4, data, gridplan::Connectivity::EIGHT);

    auto neighbors = g.get_neighbors(padded_index(0, 0, 4));
    EXPECT_EQ(neighbors.size(), 3);
}

TEST(GetNeighbors8, DiagonalCostsAreSqrt2) {
    std::vector<uint8_t> data(16, 0);
    gridplan::Grid g(4, 4, data, gridplan::Connectivity::EIGHT);

    auto neighbors = g.get_neighbors(padded_index(1, 1, 4));
    int diagonal_count = 0;
    for (const auto& [idx, cost] : neighbors) {
        if (cost > 1.0f) {
            EXPECT_NEAR(cost, 1.414f, 0.001f);
            diagonal_count++;
        }
    }
    EXPECT_EQ(diagonal_count, 4);
}

// ==================== Corner-cutting tests ====================

TEST(CornerCutting8, BlockedByAdjacentObstacle) {
    // Obstacle at (0,2) should block diagonal from (1,1) to (0,2)...
    // but also block diagonal to (0,2) area.
    // More precisely: obstacle at (0,1) blocks up-left diagonal from (1,2)
    //
    //   . 1 .       row 0
    //   . X .       row 1: X is (1,1)
    //   . . .       row 2
    //
    // Up is blocked -> diagonals up-left and up-right should be blocked
    std::vector<uint8_t> data(16, 0);
    data[0 * 4 + 1] = 1;  // (0,1) obstacle directly above (1,1)
    gridplan::Grid g(4, 4, data, gridplan::Connectivity::EIGHT);

    auto neighbors = g.get_neighbors(padded_index(1, 1, 4));
    auto indices = neighbor_indices(neighbors);

    // up is blocked
    EXPECT_FALSE(indices.count(padded_index(0, 1, 4)));
    // up-left and up-right should be blocked by corner cutting
    EXPECT_FALSE(indices.count(padded_index(0, 0, 4)));
    EXPECT_FALSE(indices.count(padded_index(0, 2, 4)));
}

TEST(CornerCutting8, DiagonalAllowedWhenBothAxesFree) {
    //   . 0 .       row 0
    //   0 X .       row 1
    //   . . .       row 2
    //
    // Both up (0,1) and left (1,0) are free -> up-left diagonal should be allowed
    std::vector<uint8_t> data(16, 0);
    gridplan::Grid g(4, 4, data, gridplan::Connectivity::EIGHT);

    auto neighbors = g.get_neighbors(padded_index(1, 1, 4));
    auto indices = neighbor_indices(neighbors);
    EXPECT_TRUE(indices.count(padded_index(0, 0, 4)));  // up-left allowed
}

TEST(CornerCutting8, DiagonalBlockedByOneAxis) {
    //   . . .       row 0
    //   1 X .       row 1: (1,0) is obstacle
    //   . . .       row 2
    //
    // Left is blocked -> down-left and up-left diagonals should be blocked
    std::vector<uint8_t> data(16, 0);
    data[1 * 4 + 0] = 1;  // (1,0) obstacle
    gridplan::Grid g(4, 4, data, gridplan::Connectivity::EIGHT);

    auto neighbors = g.get_neighbors(padded_index(1, 1, 4));
    auto indices = neighbor_indices(neighbors);

    EXPECT_FALSE(indices.count(padded_index(0, 0, 4)));  // up-left blocked
    EXPECT_FALSE(indices.count(padded_index(2, 0, 4)));  // down-left blocked
    // down-right and up-right should still be fine
    EXPECT_TRUE(indices.count(padded_index(0, 2, 4)));
    EXPECT_TRUE(indices.count(padded_index(2, 2, 4)));
}

TEST(CornerCutting8, DiagonalFreeButTargetBlocked) {
    //   1 . .       row 0: (0,0) is obstacle
    //   . X .       row 1
    //   . . .       row 2
    //
    // Up (0,1) and left (1,0) are free, but (0,0) itself is obstacle
    // Diagonal should be blocked because the target cell is an obstacle
    std::vector<uint8_t> data(16, 0);
    data[0 * 4 + 0] = 1;  // (0,0) obstacle
    gridplan::Grid g(4, 4, data, gridplan::Connectivity::EIGHT);

    auto neighbors = g.get_neighbors(padded_index(1, 1, 4));
    auto indices = neighbor_indices(neighbors);
    EXPECT_FALSE(indices.count(padded_index(0, 0, 4)));  // target itself blocked
}
