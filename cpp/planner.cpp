#include "planner.h"
#include "utils.h"

#include <queue>
#include <set>
#include <map>
#include <cmath>
#include <algorithm>
#include <memory>

using namespace OrderPlanner;

// Default lambda value (can be set from command line)
double OrderPlanner::LAMBDA = 0.2;

// ========== Dijkstra: Compute Heuristic ==========
// h(s) = min cost from s to goal, where cost = density + LAMBDA * length

CostMap OrderPlanner::compute_cost_map(const Map &map, const Site &goal) {
    size_t rows = get_map_rows(map), cols = get_map_cols(map);
    CostMap h(rows, std::vector<double>(cols, 1e18));
    
    struct Node { size_t e, n; double cost; };
    auto cmp = [](const Node& a, const Node& b) { return a.cost > b.cost; };
    std::priority_queue<Node, std::vector<Node>, decltype(cmp)> pq(cmp);
    
    h[goal.e][goal.n] = get_density_at(map, goal);
    pq.push({goal.e, goal.n, h[goal.e][goal.n]});
    
    while (!pq.empty()) {
        auto [e, n, cost] = pq.top(); pq.pop();
        if (cost > h[e][n]) continue;
        
        for (int i = 0; i < NUM_DIRECTIONS; i++) {
            int ne = (int)e + DIRECTION_DE[i];
            int nn = (int)n + DIRECTION_DN[i];
            if (!is_site_valid(map, ne, nn)) continue;
            
            double edge = get_density_at(map, {(size_t)ne, (size_t)nn}) + LAMBDA;
            double new_cost = cost + edge;
            if (new_cost < h[ne][nn]) {
                h[ne][nn] = new_cost;
                pq.push({(size_t)ne, (size_t)nn, new_cost});
            }
        }
    }
    return h;
}

// ========== A* with OPEN/CLOSED Sets ==========
// Cost: g(s) = Σ(density + LAMBDA) from start to s
// Heuristic: h(s) = min cost from s to goal (from Dijkstra)
// f(s) = g(s) + h(s)

struct AStarNode {
    Site cell;
    double g, f;
    int length;
    std::shared_ptr<AStarNode> parent;
};

AStarResult OrderPlanner::astar_search(
    const Map& map,
    const Site& start,
    const Site& goal,
    const CostMap& h,
    int max_steps
) {
    size_t cols = get_map_cols(map);
    auto key = [cols](const Site& s) { return s.e * cols + s.n; };
    
    // OPEN: priority queue ordered by f
    auto cmp = [](const AStarNode& a, const AStarNode& b) { return a.f > b.f; };
    std::priority_queue<AStarNode, std::vector<AStarNode>, decltype(cmp)> OPEN(cmp);
    
    // CLOSED: expanded nodes
    std::set<size_t> CLOSED;
    
    // g-values: best g seen for each cell
    std::map<size_t, double> g_val;
    
    // Initialize start
    double g0 = get_density_at(map, start);
    double f0 = g0 + h[start.e][start.n] - get_density_at(map, start); // avoid double-count
    g_val[key(start)] = g0;
    OPEN.push({start, g0, f0, 0, nullptr});
    
    while (!OPEN.empty()) {
        AStarNode s = OPEN.top(); OPEN.pop();
        size_t s_key = key(s.cell);
        
        // Skip if already expanded
        if (CLOSED.count(s_key)) continue;
        
        // Goal EXPANDED? Done!
        if (sites_equal(s.cell, goal)) {
            AStarResult result;
            result.success = true;
            result.total_length = s.length;
            result.total_density_cost = s.g - LAMBDA * s.length; // extract pure density
            
            // Reconstruct path
            auto cur = std::make_shared<AStarNode>(s);
            while (cur) {
                result.path.push_back(cur->cell);
                cur = cur->parent;
            }
            std::reverse(result.path.begin(), result.path.end());
            return result;
        }
        
        // Add to CLOSED
        CLOSED.insert(s_key);
        
        // Expand successors
        for (int i = 0; i < NUM_DIRECTIONS; i++) {
            int ne = (int)s.cell.e + DIRECTION_DE[i];
            int nn = (int)s.cell.n + DIRECTION_DN[i];
            if (!is_site_valid(map, ne, nn)) continue;
            
            Site s_prime = {(size_t)ne, (size_t)nn};
            size_t s_prime_key = key(s_prime);
            
            // Skip if in CLOSED
            if (CLOSED.count(s_prime_key)) continue;
            
            // Check budget
            int new_len = s.length + 1;
            if (new_len > max_steps) continue;
            
            // Edge cost: density + LAMBDA
            double edge = get_density_at(map, s_prime) + LAMBDA;
            double g_new = s.g + edge;
            
            // If g(s') > g(s) + c(s,s'), update
            if (!g_val.count(s_prime_key) || g_new < g_val[s_prime_key]) {
                g_val[s_prime_key] = g_new;
                double h_prime = h[s_prime.e][s_prime.n] - get_density_at(map, s_prime);
                double f_new = g_new + h_prime;
                
                auto parent = std::make_shared<AStarNode>(s);
                OPEN.push({s_prime, g_new, f_new, new_len, parent});
            }
        }
    }
    
    return AStarResult(); // No path found
}

// ========== Multi-Goal A* ==========

AStarResult OrderPlanner::astar_search_multi_goal(
    const Map& map,
    const Site& start,
    const std::vector<Site>& goals,
    const std::vector<CostMap>& heuristics,
    int max_steps
) {
    if (goals.empty()) return AStarResult();
    
    size_t cols = get_map_cols(map);
    auto key = [cols](const Site& s) { return s.e * cols + s.n; };
    
    auto is_goal = [&](const Site& s) {
        for (const auto& g : goals) if (sites_equal(s, g)) return true;
        return false;
    };
    
    auto min_h = [&](const Site& s) {
        double best = 1e18;
        for (const auto& h : heuristics) best = std::min(best, h[s.e][s.n]);
        return best;
    };
    
    auto cmp = [](const AStarNode& a, const AStarNode& b) { return a.f > b.f; };
    std::priority_queue<AStarNode, std::vector<AStarNode>, decltype(cmp)> OPEN(cmp);
    std::set<size_t> CLOSED;
    std::map<size_t, double> g_val;
    
    double g0 = get_density_at(map, start);
    double f0 = g0 + min_h(start) - get_density_at(map, start);
    g_val[key(start)] = g0;
    OPEN.push({start, g0, f0, 0, nullptr});
    
    while (!OPEN.empty()) {
        AStarNode s = OPEN.top(); OPEN.pop();
        size_t s_key = key(s.cell);
        
        if (CLOSED.count(s_key)) continue;
        
        if (is_goal(s.cell)) {
            AStarResult result;
            result.success = true;
            result.total_length = s.length;
            result.total_density_cost = s.g - LAMBDA * s.length;
            auto cur = std::make_shared<AStarNode>(s);
            while (cur) { result.path.push_back(cur->cell); cur = cur->parent; }
            std::reverse(result.path.begin(), result.path.end());
            return result;
        }
        
        CLOSED.insert(s_key);
        
        for (int i = 0; i < NUM_DIRECTIONS; i++) {
            int ne = (int)s.cell.e + DIRECTION_DE[i];
            int nn = (int)s.cell.n + DIRECTION_DN[i];
            if (!is_site_valid(map, ne, nn)) continue;
            
            Site s_prime = {(size_t)ne, (size_t)nn};
            size_t s_prime_key = key(s_prime);
            if (CLOSED.count(s_prime_key)) continue;
            
            int new_len = s.length + 1;
            if (new_len > max_steps) continue;
            
            double edge = get_density_at(map, s_prime) + LAMBDA;
            double g_new = s.g + edge;
            
            if (!g_val.count(s_prime_key) || g_new < g_val[s_prime_key]) {
                g_val[s_prime_key] = g_new;
                double h_prime = min_h(s_prime) - get_density_at(map, s_prime);
                double f_new = g_new + h_prime;
                OPEN.push({s_prime, g_new, f_new, new_len, std::make_shared<AStarNode>(s)});
            }
        }
    }
    
    return AStarResult();
}

// ========== Waypoint A* (Long-Horizon) ==========

AStarResult OrderPlanner::astar_search_with_waypoint(
    const Map& map,
    const Site& start,
    const Site& waypoint,
    const std::vector<Site>& goals,
    const CostMap& h_waypoint,
    const std::vector<CostMap>& h_goals,
    int max_steps
) {
    // Leg 1: start → waypoint
    AStarResult leg1 = astar_search(map, start, waypoint, h_waypoint, max_steps);
    if (!leg1.success) return AStarResult();
    
    // Leg 2: waypoint → any goal (remaining budget)
    int remaining = max_steps - leg1.total_length;
    AStarResult leg2 = astar_search_multi_goal(map, waypoint, goals, h_goals, remaining);
    if (!leg2.success) return AStarResult();
    
    // Combine paths
    AStarResult result;
    result.success = true;
    result.path = leg1.path;
    for (size_t i = 1; i < leg2.path.size(); i++) result.path.push_back(leg2.path[i]);
    result.total_length = leg1.total_length + leg2.total_length;
    result.total_density_cost = leg1.total_density_cost + leg2.total_density_cost 
                                 - get_density_at(map, waypoint); // avoid double-count
    return result;
}

// ========== Legacy Plan (Simple Diagonal) ==========

std::vector<Site> OrderPlanner::plan(const Map&, const Site& start, const Site& end) {
    std::vector<Site> path;
    size_t e = start.e, n = start.n;
    path.push_back({e, n});
    while (e != end.e || n != end.n) {
        if (e < end.e) e++; else if (e > end.e) e--;
        if (n < end.n) n++; else if (n > end.n) n--;
        path.push_back({e, n});
    }
    return path;
}

// ========== Main Entry Point ==========

Path OrderPlanner::find_path(const Map &map, const Order &order) {
    // Precompute heuristics
    CostMap h_delivery = compute_cost_map(map, order.delivery);
    std::vector<CostMap> h_docks;
    for (const auto& dock : map.docks) {
        h_docks.push_back(compute_cost_map(map, dock));
    }
    
    // Full route: origin → delivery → any dock
    AStarResult result = astar_search_with_waypoint(
        map, order.origin_dock, order.delivery, map.docks,
        h_delivery, h_docks, MAX_FLIGHT_LENGTH
    );
    
    if (!result.success) return Path{{}, {}};
    
    // Split at delivery
    std::vector<Site> outbound, inbound;
    bool at_delivery = false;
    for (const auto& s : result.path) {
        if (!at_delivery) {
            outbound.push_back(s);
            if (sites_equal(s, order.delivery)) at_delivery = true;
        } else {
            inbound.push_back(s);
        }
    }
    
    // Add delivery to inbound start
    if (!inbound.empty()) {
        inbound.insert(inbound.begin(), order.delivery);
    }
    
    return Path{outbound, inbound};
}
