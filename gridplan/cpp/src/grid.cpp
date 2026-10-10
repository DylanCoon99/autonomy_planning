#include "gridplan/grid.hpp"


namespace gridplan {

// Placeholder — full implementation in Day 3

// 2D constructor
Grid::Grid(uint32_t rows, uint32_t cols, const std::vector<uint8_t>& grid, Connectivity connectivity)
		: rows_(rows), cols_(cols), grid_((rows + 2) * (cols + 2), 1) {

			// copy all rows from grid to grid_; account for padding
			int32_t row_stride = cols + 2;
			for (uint32_t i = 0; i < rows; ++i) {
				for (uint32_t j = 0; j < cols; ++j) {
					grid_[(i + 1) * row_stride + (j + 1)] = grid[i * cols + j];
				}
			}

			build_table(connectivity, row_stride, 0);
		}

// 3D constructor
Grid::Grid(uint32_t rows, uint32_t cols, uint32_t depth, const std::vector<uint8_t>& grid, Connectivity connectivity)
		: rows_(rows), cols_(cols), depth_(depth),
		  grid_((rows + 2) * (cols + 2) * (depth + 2), 1) {

			int32_t row_stride = cols + 2;
			int32_t z_stride = (rows + 2) * (cols + 2);

			// copy grid into padded interior
			for (uint32_t z = 0; z < depth; ++z) {
				for (uint32_t i = 0; i < rows; ++i) {
					for (uint32_t j = 0; j < cols; ++j) {
						int32_t padded_idx = (z + 1) * z_stride + (i + 1) * row_stride + (j + 1);
						int32_t src_idx = z * (rows * cols) + i * cols + j;
						grid_[padded_idx] = grid[src_idx];
					}
				}
			}

			build_table(connectivity, row_stride, z_stride);
		}

void Grid::build_table(Connectivity connectivity, int32_t row_stride, int32_t z_stride) {
			std::vector<NeighborEntry> selected_dirs;

			switch(connectivity) {
				case Connectivity::FOUR:
					selected_dirs = FOUR_DIRS;
					break;
				case Connectivity::EIGHT:
					selected_dirs = FOUR_DIRS;
					selected_dirs.insert(selected_dirs.end(), DIAG_2D_DIRS.begin(), DIAG_2D_DIRS.end());
					break;
				case Connectivity::SIX:
					selected_dirs = SIX_DIRS;
					break;
				case Connectivity::EIGHTEEN:
					selected_dirs = SIX_DIRS;
					selected_dirs.insert(selected_dirs.end(), FACE_DIAG_3D_DIRS.begin(), FACE_DIAG_3D_DIRS.end());
					break;
				case Connectivity::TWENTYSIX:
					selected_dirs = SIX_DIRS;
					selected_dirs.insert(selected_dirs.end(), FACE_DIAG_3D_DIRS.begin(), FACE_DIAG_3D_DIRS.end());
					selected_dirs.insert(selected_dirs.end(), BODY_DIAG_3D_DIRS.begin(), BODY_DIAG_3D_DIRS.end());
					break;
			}

			for (auto& entry : selected_dirs) {
				int32_t offset = entry.dr * row_stride + entry.dc + entry.dz * z_stride;

				// compute corner checks from component axes
				std::vector<int32_t> checks;
				int32_t axes[3] = {entry.dr, entry.dc, entry.dz};
				int32_t strides[3] = {row_stride, 1, z_stride};

				// count how many axes are non-zero
				int n_axes = (entry.dr != 0) + (entry.dc != 0) + (entry.dz != 0);

				if (n_axes >= 2) {
					// for each non-zero axis, add the single-axis offset as a corner check
					for (int a = 0; a < 3; ++a) {
						if (axes[a] != 0) {
							checks.push_back(axes[a] * strides[a]);
						}
					}
				}

				table_.push_back({offset, entry.cost, std::move(checks)});
			}
		}


uint8_t Grid::index_to_coordinate(uint32_t row, uint32_t col) {
	// grid is stored in row major
	return grid_[((row + 1) * (cols_ + 2)) + col + 1];
}



std::vector<std::pair<int32_t, float>> Grid::get_neighbors(uint32_t index) {
	// returns all valid and reachable neighbors for a node at a given index

	std::vector<std::pair<int32_t, float>> neighbors;

	for (const auto& entry : table_) {
		if (grid_[index + entry.offset] != 0) continue;

		// corner-cutting check: all component axes must be free
		bool passable = true;
		for (int32_t check : entry.corner_checks) {
			if (grid_[index + check] != 0) {
				passable = false;
				break;
			}
		}

		if (passable) {
			neighbors.emplace_back(index + entry.offset, entry.cost);
		}
	}

	return neighbors;
}


uint32_t Grid::size() {
	return grid_.size();
}

std::tuple<uint32_t, uint32_t, uint32_t> Grid::index_to_coords(uint32_t index) const {
	uint32_t padded_cols = cols_ + 2;
	uint32_t padded_rows = rows_ + 2;
	uint32_t z_stride = padded_rows * padded_cols;

	uint32_t z = index / z_stride;
	uint32_t rem = index % z_stride;
	uint32_t r = rem / padded_cols;
	uint32_t c = rem % padded_cols;

	// subtract 1 to remove padding offset; z is 0 for 2D grids
	return {r - 1, c - 1, depth_ == 1 ? 0 : z - 1};
}

}  // namespace gridplan
