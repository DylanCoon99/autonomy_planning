#pragma once
#include "planner.hpp"

namespace gridplan {


class AStar : public PlannerInterface {
public:
    PlannerResult plan(Grid& grid, uint32_t start, uint32_t goal, const PlannerConfig& config) override;
};


}  // namespace gridplan
