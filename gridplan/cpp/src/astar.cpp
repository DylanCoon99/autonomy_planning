#include "gridplan/astar.hpp"
#include "gridplan/heuristics.hpp"
#include <unordered_set>

namespace gridplan {

PlannerResult AStar::plan(Grid& grid, uint32_t start, uint32_t goal, const PlannerConfig& config) {

    // works the same way as Djikstra, but with a heuristic
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

    // priority queue stores (f, index) where f = g + w*h
    std::priority_queue<std::pair<float, uint32_t>, std::vector<std::pair<float, uint32_t>>, std::greater<std::pair<float, uint32_t>>> priority_queue;
    std::unordered_map<uint32_t, std::optional<uint32_t>> parents;
    // distances stores g values (actual cost from start)
    std::vector<float> distances(grid.size(), std::numeric_limits<float>::infinity());
    uint32_t nodes_expanded = 0;
    std::vector<uint32_t> expansion_order;
    std::unordered_set<uint32_t> closed;
    distances[start] = 0.0f;
    parents[start] = std::nullopt;

    // put start in the priority queue
    float h_start = compute_heuristic(config.heuristic, start, goal, grid);
    priority_queue.push({config.weight * h_start, start});
    bool found = false;

    while (!priority_queue.empty()) {
        auto [f, node] = priority_queue.top();
        priority_queue.pop();

        // skip if already expanded
        if (closed.count(node)) {
            continue;
        }
        closed.insert(node);

        ++nodes_expanded;
        if (config.record_expansions) {
            expansion_order.push_back(node);
        }

        if (node == goal) {
            found = true;
            break;
        }

        // look at neighbors
        std::vector<std::pair<int32_t, float>> neighbors = grid.get_neighbors(node);

        for (auto pair : neighbors) {
            auto [neighbor_idx, edge_cost] = pair;
            float g_new = distances[node] + edge_cost;

            if (g_new < distances[neighbor_idx]) {
                distances[neighbor_idx] = g_new;
                parents[neighbor_idx] = node;
                float h = compute_heuristic(config.heuristic, neighbor_idx, goal, grid);
                float f_new = g_new + config.weight * h;
                priority_queue.push({f_new, neighbor_idx});
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
