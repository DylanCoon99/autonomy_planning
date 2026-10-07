#include "gridplan/dijkstra.hpp"
#include <queue>
#include <unordered_map>
#include <limits>
#include <utility>
#include <chrono>
#include <algorithm>
#include <optional>

namespace gridplan {



PlannerResult Dijkstra::plan(Grid& grid, uint32_t start, uint32_t goal, const PlannerConfig& config) {

    
    /*
    Dijkstra Algorithm

    Store unvisited nodes in a priority queue; stores pairs (cost, index)

    Keep track of the parent of each node

    Keep track of shortest distances to nodes

    Algorithm
        - put start in priority queue
        - while priority queue is not empty
            - pop from queue
            - if the cost is greater than the the current shortest distance -> skip
            - get all reachable neighbors for the node
            - for each neighbor
                - compute the cost to reach that node
                - if cost is less than the current shortest distance
                    - update the shortest distance
                    - update the parent
                    - push node to the queue

        - build the path using the parent map
        - build the result
        - return the result

    */

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

    // priority queue stores a pair/tuple (cost, index)
    std::priority_queue<std::pair<uint32_t, uint32_t>, std::vector<std::pair<uint32_t, uint32_t>>, std::greater<std::pair<uint32_t, uint32_t>>> priority_queue;
    std::unordered_map<uint32_t, std::optional<uint32_t>> parents;
    std::vector<float> distances(grid.size(), std::numeric_limits<float>::infinity());
    uint32_t nodes_expanded = 0;
    std::vector<uint32_t> expansion_order;
    distances[start] = 0.0f;
    parents[start] = std::nullopt;

    // put start in the priority queue
    priority_queue.push({0, start});
    bool found = false;

    while (!priority_queue.empty()) {
        // pop the node with least distance
        auto [cost, node] = priority_queue.top();
        priority_queue.pop();

        ++nodes_expanded;
        if (config.record_expansions) {
            expansion_order.push_back(node);
        }

        if (node == goal) {
            found = true;
            break;
        }

        if (cost > distances[node]) {
            continue;
        }

        // look at neighbors
        std::vector<std::pair<int32_t, float>> neighbors = grid.get_neighbors(node);

        for (auto pair : neighbors) {
            // for each neighbor, check if the distance is smaller than current distance
            auto [neighbor_cost, neighbor_node] = pair;
            uint32_t new_cost = neighbor_cost + cost;
            if (new_cost < distances[neighbor_node]) {
                // update the shortest path for neighbor node
                distances[neighbor_node] = new_cost;
                parents[neighbor_node] = node;
                priority_queue.push({new_cost, neighbor_node});
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
    uint32_t current = goal;

    while (current != start) {
        path.push_back(current);
        current = parents[current].value();
    }

    path.push_back(start);
    std::reverse(path.begin(), path.end());

    PlannerResult result{
        .path = path,
        .cost = distances[goal],
        .nodes_expanded = nodes_expanded,
        .expansion_order = config.record_expansions ? std::optional(expansion_order) : std::nullopt,
        .time = diff_ms
    };

    return result;


}


}  // namespace gridplan
