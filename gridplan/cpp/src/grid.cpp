#include "gridplan/grid.hpp"

namespace gridplan {

// Placeholder — full implementation in Day 3

Grid::Grid(uint32_t size_x, uint32_t size_y, const std::vector<uint8_t>& grid)
        : size_x_(size_x), size_y_(size_y), grid_((size_x + 2) * (size_y + 2), 1) {
            
            // copy all rows from grid to grid_; account for padding
            for (int i = 0; i < size_x; ++i) {
                for (int j = 0; j < size_y; ++j) {
                    grid_[((i + 1) * (size_y + 2)) + (j + 1)] = grid[i * size_y + j];
                }
            }

        }


uint8_t Grid::index_to_coordinate(uint32_t row, uint32_t col) {
    // grid is stored in row major
    return grid_[((row + 1) * (size_y_ + 2)) + col + 1];
}




}  // namespace gridplan
