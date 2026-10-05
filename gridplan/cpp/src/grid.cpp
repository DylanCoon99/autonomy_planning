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
				table_.emplace_back(offset, entry.cost);
			}
		}


uint8_t Grid::index_to_coordinate(uint32_t row, uint32_t col) {
	// grid is stored in row major
	return grid_[((row + 1) * (cols_ + 2)) + col + 1];
}



std::vector<uint32_t> Grid::get_neighbors(uint32_t index) {
	// returns all valid neighbors for a node at a given index

	// first get all the neighbors
	for (auto entry : table_) {
		
	}

}



}  // namespace gridplan
