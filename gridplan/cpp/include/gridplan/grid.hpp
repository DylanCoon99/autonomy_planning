#pragma once

#include <cstdint>
#include <vector>
#include <utility>

namespace gridplan {


struct NeighborEntry { int32_t dr; int32_t dc; float cost; };

static const std::vector<NeighborEntry> FOUR_DIRS = {
	{-1, 0, 1.0f}, {1, 0, 1.0f}, {0, -1, 1.0f}, {0, 1, 1.0f}
};

static const std::vector<NeighborEntry> DIAG_DIRS = {
	{-1, -1, 1.414f}, {-1, 1, 1.414f}, {1, -1, 1.414f}, {1, 1, 1.414f}
};

enum class Connectivity {
	FOUR,   // 2D
	EIGHT,  // 2D
	SIX,    // 3D
	EIGHTEEN, // 3D
	TWENTYSIX // 3D
};

// Placeholder — full implementation in Day 3
class Grid {
public:
	Grid() = default;
	Grid(uint32_t rows, uint32_t cols, const std::vector<uint8_t>& grid, Connectivity connectivity);

	uint8_t index_to_coordinate(uint32_t idx, uint32_t idy);

	std::vector<uint32_t> get_neighbors(uint32_t index);

	void print_grid();

private:
	uint32_t rows_, cols_;
	std::vector<uint8_t> grid_;
	uint8_t padding_;
	std::vector<std::pair<int32_t, float>> table_;

};

}  // namespace gridplan
