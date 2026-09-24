#include "gridplan/grid.hpp"


namespace gridplan {

// Placeholder — full implementation in Day 3

Grid::Grid(uint32_t rows, uint32_t cols, const std::vector<uint8_t>& grid, Connectivity connectivity)
		: rows_(rows), cols_(cols), grid_((rows + 2) * (cols + 2), 1) {
			
			// copy all rows from grid to grid_; account for padding
			for (int i = 0; i < rows; ++i) {
				for (int j = 0; j < cols; ++j) {
					grid_[((i + 1) * (cols + 2)) + (j + 1)] = grid[i * cols + j];
				}
			}

			uint32_t stride = cols_ + 2;

			// compute the neighbor offset/cost table
			std::vector<NeighborEntry> selected_dirs;

			switch(connectivity) {
				case Connectivity::FOUR:
					selected_dirs = FOUR_DIRS;
					break;
				case Connectivity::EIGHT:
					selected_dirs = FOUR_DIRS;
					selected_dirs.insert(selected_dirs.end(), DIAG_DIRS.begin(), DIAG_DIRS.end());
					break;
				default:
					std::cerr << "Error: Unrecognized Connectivity Setting...\n";
					break;
			}



			for (auto& entry : selected_dirs) {                                                                                                                            
				int32_t offset = entry.dr * stride + entry.dc;                                                                                                             
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
