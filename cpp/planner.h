#pragma once

#include "utils.h"
#include <vector>

namespace OrderPlanner {
    // Type alias for cost maps
    using CostMap = std::vector<std::vector<double>>;

    // Cost = density + LAMBDA * length
    // LAMBDA controls step cost relative to density (default 0.5)
    extern double LAMBDA;

    // Compute heuristic: min cost from each cell to goal using Dijkstra
    CostMap compute_cost_map(const Map &map, const Site &goal);

    // A* result
    struct AStarResult {
        std::vector<Site> path;
        double total_density_cost = 0;
        int total_length = 0;
        bool success = false;
    };

    // Single-goal A* with budget constraint
    AStarResult astar_search(
        const Map& map,
        const Site& start,
        const Site& goal,
        const CostMap& heuristic,
        int max_steps
    );

    // Multi-goal A* (finds path to any goal)
    AStarResult astar_search_multi_goal(
        const Map& map,
        const Site& start,
        const std::vector<Site>& goals,
        const std::vector<CostMap>& heuristics,
        int max_steps
    );

    // Two-leg A*: start → waypoint → any goal
    AStarResult astar_search_with_waypoint(
        const Map& map,
        const Site& start,
        const Site& waypoint,
        const std::vector<Site>& goals,
        const CostMap& waypoint_heuristic,
        const std::vector<CostMap>& goal_heuristics,
        int max_steps
    );

    // Legacy functions
    std::vector<Site> plan(const Map &map, const Site &start, const Site &end);
    Path find_path(const Map &map, const Order &order);
    
    // Get the lambda value that was used for the last successful find_path call
    double get_last_used_lambda();
}
