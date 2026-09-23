#include "gridplan/grid.hpp"
#include <iostream>

int main() {
    std::vector<uint8_t> data = {0, 0, 1, 0,
                                  0, 0, 0, 0,
                                  1, 0, 0, 1,
                                  0, 0, 0, 0};
    gridplan::Grid g(4, 4, data);

    

    // print each cell to verify padding
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            std::cout << (int)g.index_to_coordinate(r, c) << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}
