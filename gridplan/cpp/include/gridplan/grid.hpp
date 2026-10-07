#pragma once

#include <cstdint>
#include <vector>
#include <utility>

namespace gridplan {


struct NeighborEntry {
	int32_t dr; int32_t dc; int32_t dz; float cost;
	std::vector<int32_t> corner_checks;  // empty for axis-aligned moves
};

// ---- 2D directions ----
static const std::vector<NeighborEntry> FOUR_DIRS = {
	{-1, 0, 0, 1.0f}, {1, 0, 0, 1.0f}, {0, -1, 0, 1.0f}, {0, 1, 0, 1.0f}
};

static const std::vector<NeighborEntry> DIAG_2D_DIRS = {
	{-1, -1, 0, 1.414f}, {-1, 1, 0, 1.414f}, {1, -1, 0, 1.414f}, {1, 1, 0, 1.414f}
};

// ---- 3D directions ----
// 6-connected: axis-aligned only (up/down/left/right/above/below)
static const std::vector<NeighborEntry> SIX_DIRS = {
	{-1, 0, 0, 1.0f}, {1, 0, 0, 1.0f},   // row
	{0, -1, 0, 1.0f}, {0, 1, 0, 1.0f},   // col
	{0, 0, -1, 1.0f}, {0, 0, 1, 1.0f}    // depth
};

// 18-connected: axis-aligned + face diagonals (no body diagonals)
static const std::vector<NeighborEntry> FACE_DIAG_3D_DIRS = {
	// row-col diagonals (same z-plane)
	{-1, -1, 0, 1.414f}, {-1, 1, 0, 1.414f}, {1, -1, 0, 1.414f}, {1, 1, 0, 1.414f},
	// row-depth diagonals (same col)
	{-1, 0, -1, 1.414f}, {-1, 0, 1, 1.414f}, {1, 0, -1, 1.414f}, {1, 0, 1, 1.414f},
	// col-depth diagonals (same row)
	{0, -1, -1, 1.414f}, {0, -1, 1, 1.414f}, {0, 1, -1, 1.414f}, {0, 1, 1, 1.414f}
};

// 26-connected: adds body diagonals (all three axes change)
static const std::vector<NeighborEntry> BODY_DIAG_3D_DIRS = {
	{-1, -1, -1, 1.732f}, {-1, -1, 1, 1.732f}, {-1, 1, -1, 1.732f}, {-1, 1, 1, 1.732f},
	{ 1, -1, -1, 1.732f}, { 1, -1, 1, 1.732f}, { 1, 1, -1, 1.732f}, { 1, 1, 1, 1.732f}
};

enum class Connectivity {
	FOUR,   // 2D
	EIGHT,  // 2D
	SIX,    // 3D
	EIGHTEEN, // 3D
	TWENTYSIX // 3D
};


class Grid {
public:
	Grid() = default;
	// 2D constructor
	Grid(uint32_t rows, uint32_t cols, const std::vector<uint8_t>& grid, Connectivity connectivity);
	// 3D constructor
	Grid(uint32_t rows, uint32_t cols, uint32_t depth, const std::vector<uint8_t>& grid, Connectivity connectivity);

	uint8_t index_to_coordinate(uint32_t idx, uint32_t idy);

	std::vector<std::pair<int32_t, float>> get_neighbors(uint32_t index);

	void print_grid();

	uint32_t size();

	bool is_obstacle(uint32_t index) const { return grid_[index] != 0; }

private:
	uint32_t rows_, cols_, depth_ = 1;
	std::vector<uint8_t> grid_;
	struct TableEntry {
		int32_t offset;
		float cost;
		std::vector<int32_t> corner_checks;  // component offsets that must be free for diagonal moves
	};
	std::vector<TableEntry> table_;

	void build_table(Connectivity connectivity, int32_t row_stride, int32_t z_stride);
};

}  // namespace gridplan
