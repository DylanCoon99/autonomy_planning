#pragma once

#include <cstdint>
#include <cmath>
#include <algorithm>
#include "planner.hpp"
#include "grid.hpp"

namespace gridplan {

inline float compute_heuristic(Heuristic h, uint32_t from, uint32_t to, Grid& grid) {
    auto [r1, c1, z1] = grid.index_to_coords(from);
    auto [r2, c2, z2] = grid.index_to_coords(to);

    float dr = std::abs((float)r1 - (float)r2);
    float dc = std::abs((float)c1 - (float)c2);
    float dz = std::abs((float)z1 - (float)z2);

    switch (h) {
        case Heuristic::ZERO:
            return 0.0f;

        case Heuristic::MANHATTAN:
            return dr + dc + dz;

        case Heuristic::EUCLIDEAN:
            return std::sqrt(dr * dr + dc * dc + dz * dz);

        case Heuristic::OCTILE: {
            if (grid.get_depth() == 1) {
                // 2D octile: (dx + dy) + (sqrt(2) - 2) * min(dx, dy)
                float mn = std::min(dr, dc);
                return (dr + dc) + (1.414f - 2.0f) * mn;
            } else {
                // 3D octile: sort deltas d1 <= d2 <= d3
                float d[3] = {dr, dc, dz};
                std::sort(d, d + 3);
                // sqrt(3)*d1 + sqrt(2)*(d2 - d1) + (d3 - d2)
                return 1.732f * d[0] + 1.414f * (d[1] - d[0]) + (d[2] - d[1]);
            }
        }
    }

    return 0.0f;
}

}  // namespace gridplan
