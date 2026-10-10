#include "gridplan/bfs.hpp"
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <algorithm>
#include <chrono>

namespace gridplan {


PlannerResult BFS::plan(Grid& grid, uint32_t start, uint32_t goal, const PlannerConfig& config) {

    auto start_time = std::chrono::steady_clock::now();

    // early-out: start or goal is an obstacle
    if (grid.is_obstacle(start) || grid.is_obstacle(goal)) {
        auto end_time = std::chrono::steady_clock::now();
        return PlannerResult{
            .path = {},
            .cost = -1.0f,
            .nodes_expanded = 0,
            .expansion_order = std::nullopt,
            .time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time)
        };
    }

    std::unordered_set<uint32_t> visited;
    std::unordered_map<uint32_t, uint32_t> parents;
    std::queue<uint32_t> q;

    uint32_t nodes_expanded = 0;
    std::vector<uint32_t> expansion_order;
    bool found = false;

    visited.insert(start);
    q.push(start);

    // perform bfs algorithm
    uint32_t current;

    while (!q.empty()) {
        current = q.front();
        q.pop();
        ++nodes_expanded;
        if (config.record_expansions) {
            expansion_order.push_back(current);
        }

        if (current == goal) {
            found = true;
            break;
        }

        // get all neighbors for this node; (neighbor, cost) pairs
        std::vector<std::pair<int32_t, float>> neighbors = grid.get_neighbors(current);

        // for each neighbor that is not in visited -> mark visited, add to queue
        for (auto pair : neighbors) {
            uint32_t neighbor = pair.first;
            if (!visited.contains(neighbor)) {
                visited.insert(neighbor);
                q.push(neighbor);
                parents[neighbor] = current;
            }
        }
    }

    auto end_time = std::chrono::steady_clock::now();
    auto diff_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    // unreachable goal
    if (!found) {
        return PlannerResult{
            .path = {},
            .cost = -1.0f,
            .nodes_expanded = nodes_expanded,
            .expansion_order = config.record_expansions ? std::optional(expansion_order) : std::nullopt,
            .time = diff_ms
        };
    }

    // build the path
    std::vector<uint32_t> path;
    current = goal;

    while (current != start) {
        path.push_back(current);
        current = parents[current];
    }

    path.push_back(start);
    std::reverse(path.begin(), path.end());

    PlannerResult result{
        .path = path,
        .cost = static_cast<float>(path.size() - 1),
        .nodes_expanded = nodes_expanded,
        .expansion_order = config.record_expansions ? std::optional(expansion_order) : std::nullopt,
        .time = diff_ms
    };

    return result;
}



}  // namespace gridplan
