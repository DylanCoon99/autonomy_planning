#pragma once

#include <cstdint>
#include <vector>
#include <chrono>
#include <optional>
#include "grid.hpp"

namespace gridplan {

enum class Heuristic {                                                                                                                                         
	MANHATTAN,                                                                                                                                                 
	OCTILE,                                                                                                                                                    
	EUCLIDEAN,                                                                                                                                                 
	ZERO  // makes A* behave like Dijkstra                                                                                                                     
};

struct PlannerConfig {
	bool tie_break;
	bool record_expansions;
	float weight;
	Heuristic heuristic;
};


struct PlannerResult {
	std::vector<uint32_t> path;
	float cost;
	uint32_t nodes_expanded;
	std::optional<std::vector<uint32_t>> expansion_order;
	std::chrono::duration<double, std::milli> time;
};


class PlannerInterface {

public:
	virtual PlannerResult plan(Grid& grid, uint32_t start, uint32_t goal, const PlannerConfig& config) = 0;

	virtual ~PlannerInterface() = default;

};



}  // namespace gridplan
