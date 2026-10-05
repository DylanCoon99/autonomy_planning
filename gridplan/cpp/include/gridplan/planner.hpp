#pragma once

#include <cstdint>
#include <vector>
#include <chrono>

namespace gridplan {

struct PlannerConfig {
    bool tie_break;
    bool record_expansions;
    float weight;
};


struct PlannerResult {
    std::vector<uint32_t> path;
    float cost;
    uint32_t nodes_expanded;
    std::optional<std::vector<uint32_t>> expansion_order;
    std::chrono::duration<double, std::milli> time;
};



}  // namespace gridplan
