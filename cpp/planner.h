#pragma once

#include "utils.h"
#include <vector>

namespace OrderPlanner {
    // Type alias for cost maps
    using CostMap = std::vector<std::vector<double>>;

    // Cost = density + LAMBDA * length
    // LAMBDA controls step cost relative to density
    extern double LAMBDA;
    
    // Lambda search parameters
    constexpr double LAMBDA_START = 0.05;
    constexpr double LAMBDA_INCREMENT = 0.05;
    constexpr double LAMBDA_MAX = 2.0;

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

    // Two-leg A*: start → waypoint → any goal (legacy)
    AStarResult astar_search_with_waypoint(
        const Map& map,
        const Site& start,
        const Site& waypoint,
        const std::vector<Site>& goals,
        const CostMap& waypoint_heuristic,
        const std::vector<CostMap>& goal_heuristics,
        int max_steps
    );
    
    // Unified A*: start → delivery → any dock (joint optimization)
    AStarResult astar_search_unified(
        const Map& map,
        const Site& start,
        const Site& delivery,
        const std::vector<Site>& docks,
        const CostMap& h_delivery,
        const CostMap& h_docks_min,
        double h_delivery_to_docks,
        int max_steps
    );
    
    // Compute minimum cost from each position to any dock
    CostMap compute_min_cost_to_any_dock(const Map& map, const std::vector<Site>& docks);
    
    // Helper functions for unified A*
    bool is_dock_position(const Site& pos, const std::vector<Site>& docks);
    double compute_unified_heuristic(
        const Site& pos,
        bool visited_delivery,
        const Site& delivery,
        const CostMap& h_delivery,
        const CostMap& h_docks_min,
        double h_delivery_to_docks
    );

    // Legacy functions
    std::vector<Site> plan(const Map &map, const Site &start, const Site &end);
    Path find_path(const Map &map, const Order &order);
    
    // Get the lambda value that was used for the last successful find_path call
    double get_last_used_lambda();
}
