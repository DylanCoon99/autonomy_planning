#pragma once

#include <cstdint>
#include <vector>

namespace gridplan {

// Placeholder — full implementation in Day 3
class Grid {
public:
    Grid() = default;
    Grid(uint32_t size_x, uint32_t size_y, const std::vector<uint8_t>& grid);

    
    uint8_t index_to_coordinate(uint32_t idx, uint32_t idy);
    
    void print_grid();

private:
    uint32_t size_x_, size_y_;
    std::vector<uint8_t> grid_;
    uint8_t padding_;

};

}  // namespace gridplan
