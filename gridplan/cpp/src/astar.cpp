#include "gridplan/astar.hpp"
#include "gridplan/heuristics.hpp"
#include <unordered_set>

namespace gridplan {

struct QueueEntry {
    float f;
    float g;
    uint32_t node;
};

struct CompareDefault {
    bool operator()(const QueueEntry& a, const QueueEntry& b) {
        return a.f > b.f;  // min-heap by f
    }
};

struct CompareWithTieBreak {
    bool operator()(const QueueEntry& a, const QueueEntry& b) {
        if (a.f != b.f) return a.f > b.f;  // min-heap by f
        return a.g < b.g;  // on equal f, prefer larger g
    }
};

PlannerResult AStar::plan(Grid& grid, uint32_t start, uint32_t goal, const PlannerConfig& config) {

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

    std::unordered_map<uint32_t, std::optional<uint32_t>> parents;
    std::vector<float> distances(grid.size(), std::numeric_limits<float>::infinity());
    uint32_t nodes_expanded = 0;
    std::vector<uint32_t> expansion_order;
    std::unordered_set<uint32_t> closed;
    distances[start] = 0.0f;
    parents[start] = std::nullopt;

    float h_start = compute_heuristic(config.heuristic, start, goal, grid);
    bool found = false;

    // Use a lambda to run the search with either comparator
    auto run_search = [&](auto& pq) {
        pq.push({config.weight * h_start, 0.0f, start});

        while (!pq.empty()) {
            auto entry = pq.top();
            pq.pop();

            if (closed.count(entry.node)) continue;
            closed.insert(entry.node);

            ++nodes_expanded;
            if (config.record_expansions) {
                expansion_order.push_back(entry.node);
            }

            if (entry.node == goal) {
                found = true;
                break;
            }

            auto neighbors = grid.get_neighbors(entry.node);

            for (auto [neighbor_idx, edge_cost] : neighbors) {
                float g_new = distances[entry.node] + edge_cost;

                if (g_new < distances[neighbor_idx]) {
                    distances[neighbor_idx] = g_new;
                    parents[neighbor_idx] = entry.node;
                    float h = compute_heuristic(config.heuristic, neighbor_idx, goal, grid);
                    float f_new = g_new + config.weight * h;
                    pq.push({f_new, g_new, static_cast<uint32_t>(neighbor_idx)});
                }
            }
        }
    };

    if (config.tie_break) {
        std::priority_queue<QueueEntry, std::vector<QueueEntry>, CompareWithTieBreak> pq;
        run_search(pq);
    } else {
        std::priority_queue<QueueEntry, std::vector<QueueEntry>, CompareDefault> pq;
        run_search(pq);
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
