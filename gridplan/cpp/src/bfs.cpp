#include "gridplan/bfs.hpp"
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <algorithm>
#include <chrono>

namespace gridplan {

// Placeholder — full implementation in Day 4

// should implement the plan method for the planner interface


PlannerResult BFS::plan(Grid& grid, uint32_t start, uint32_t goal, const PlannerConfig& config) {

    auto start_time = std::chrono::steady_clock::now();

    std::unordered_set<uint32_t> visited;
    std::unordered_map<uint32_t, uint32_t> parents;
    std::queue<uint32_t> q;

    uint32_t nodes_expanded = 0;

    visited.insert(start);
    q.push(start);

    // perform bfs algorithm
    uint32_t current;

   

    while (!q.empty()) {
        current = q.front();
        q.pop();
        ++nodes_expanded;
        // get all neighbors for this node; (neighbor, cost) pairs
        std::vector<std::pair<int32_t, float>> neighbors = grid.get_neighbors(current);

        if (current == goal) {
            // found the target
            break;
        }

        // for each neighbor that is not in vistid -> mark visited, add to queue
        for (auto pair : neighbors) {
            uint32_t neighbor = pair.first;
            if (!visited.contains(neighbor)) {
                visited.insert(neighbor);
                q.push(neighbor);
                parents[neighbor] = current;
            }
        }
    }

    // starting at target, find the parent for each node, put in a vector
    std::vector<uint32_t> path;

    current = goal;

    while (current != start) {
        path.push_back(current);
        current = parents[current];
    }

    path.push_back(start);
    std::reverse(path.begin(), path.end());

    auto end_time = std::chrono::steady_clock::now();

    auto diff_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    PlannerResult result{
        .path = path,
        .cost = static_cast<float>(path.size() - 1),
        .nodes_expanded = nodes_expanded,
        .expansion_order = std::nullopt,
        .time = diff_ms
    };

    return result;
}



}  // namespace gridplan
