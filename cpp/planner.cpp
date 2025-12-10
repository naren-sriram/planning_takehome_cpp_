#include "planner.h"
#include "utils.h"

#include <queue>
#include <set>
#include <map>
#include <cmath>
#include <algorithm>
#include <memory>

using namespace OrderPlanner;

double OrderPlanner::LAMBDA = 0.5;
static double last_used_lambda = LAMBDA_START;

// ========== Dijkstra: Compute Heuristic ==========
// h(s) = min cost from s to goal, where cost = density + LAMBDA * length

CostMap OrderPlanner::compute_cost_map(const Map &map, const Site &goal) {
    const size_t rows = get_map_rows(map);
    const size_t cols = get_map_cols(map);
    
    std::priority_queue<DijkstraNode, std::vector<DijkstraNode>, std::greater<DijkstraNode>> pq;
    CostMap h(rows, std::vector<double>(cols, 1e18));
    
    h[goal.e][goal.n] = 0.0;
    pq.push({goal.e, goal.n, 0.0});
    
    while (!pq.empty()) {
        const DijkstraNode current = pq.top();
        pq.pop();
        
        if (current.cost > h[current.e][current.n]) {
            continue;
        }
        
        for (int i = 0; i < NUM_DIRECTIONS; i++) {
            const int ne = static_cast<int>(current.e) + DIRECTION_DE[i];
            const int nn = static_cast<int>(current.n) + DIRECTION_DN[i];
            
            if (!is_site_valid(map, ne, nn)) {
                continue;
            }
            
            const double edge_cost = get_density_at(map, {current.e, current.n}) + LAMBDA;
            const double new_cost = current.cost + edge_cost;
            
            if (new_cost < h[ne][nn]) {
                h[ne][nn] = new_cost;
                pq.push({static_cast<size_t>(ne), static_cast<size_t>(nn), new_cost});
            }
        }
    }
    return h;
}

CostMap OrderPlanner::compute_min_cost_to_any_dock(const Map& map, const std::vector<Site>& docks) {
    const size_t rows = get_map_rows(map);
    const size_t cols = get_map_cols(map);
    CostMap h_min(rows, std::vector<double>(cols, 1e18));
    
    std::priority_queue<DijkstraNode, std::vector<DijkstraNode>, std::greater<DijkstraNode>> pq;
    
    for (const auto& dock : docks) {
        h_min[dock.e][dock.n] = 0.0;
        pq.push({dock.e, dock.n, 0.0});
    }
    
    while (!pq.empty()) {
        const DijkstraNode current = pq.top();
        pq.pop();
        
        if (current.cost > h_min[current.e][current.n]) {
            continue;
        }
        
        for (int i = 0; i < NUM_DIRECTIONS; i++) {
            const int ne = static_cast<int>(current.e) + DIRECTION_DE[i];
            const int nn = static_cast<int>(current.n) + DIRECTION_DN[i];
            
            if (!is_site_valid(map, ne, nn)) {
                continue;
            }
            
            const double edge_cost = get_density_at(map, {current.e, current.n}) + LAMBDA;
            const double new_cost = current.cost + edge_cost;
            
            if (new_cost < h_min[ne][nn]) {
                h_min[ne][nn] = new_cost;
                pq.push({static_cast<size_t>(ne), static_cast<size_t>(nn), new_cost});
            }
        }
    }
    return h_min;
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

// ========== Helper Functions for Unified A* ==========

bool OrderPlanner::is_dock_position(const Site& pos, const std::vector<Site>& docks) {
    for (const auto& dock : docks) {
        if (sites_equal(pos, dock)) {
            return true;
        }
    }
    return false;
}

double OrderPlanner::compute_unified_heuristic(
    const Site& pos,
    bool visited_delivery,
    const Site& delivery,
    const CostMap& h_delivery,
    const CostMap& h_docks_min,
    double h_delivery_to_docks
) {
    if (visited_delivery) {
        return h_docks_min[pos.e][pos.n];
    }
    
    if (sites_equal(pos, delivery)) {
        return h_docks_min[pos.e][pos.n];
    }
    
    return h_delivery[pos.e][pos.n] + h_delivery_to_docks - get_density_at(map, delivery);
}

// ========== Unified A* State Structures ==========

struct StateKey {
    Site position;
    bool visited_delivery;
    
    bool operator<(const StateKey& other) const {
        if (position.e != other.position.e) {
            return position.e < other.position.e;
        }
        if (position.n != other.position.n) {
            return position.n < other.position.n;
        }
        return visited_delivery < other.visited_delivery;
    }
};

struct UnifiedState {
    Site position;
    bool visited_delivery;
    double g;
    double f;
    int length;
    std::shared_ptr<UnifiedState> parent;
    
    bool operator>(const UnifiedState& other) const {
        return f > other.f;
    }
};

AStarResult OrderPlanner::astar_search_unified(
    const Map& map,
    const Site& start,
    const Site& delivery,
    const std::vector<Site>& docks,
    const CostMap& h_delivery,
    const CostMap& h_docks_min,
    double h_delivery_to_docks,
    int max_steps
) {
    std::priority_queue<UnifiedState, std::vector<UnifiedState>, std::greater<UnifiedState>> OPEN;
    std::set<StateKey> CLOSED;
    std::map<StateKey, double> g_val;
    
    const double g0 = get_density_at(map, start);
    const double h0 = compute_unified_heuristic(
        start, false, delivery, h_delivery, h_docks_min, h_delivery_to_docks
    );
    const double f0 = g0 + h0;
    const StateKey start_key = {start, false};
    
    g_val[start_key] = g0;
    OPEN.push({start, false, g0, f0, 0, nullptr});
    
    while (!OPEN.empty()) {
        const UnifiedState s = OPEN.top();
        OPEN.pop();
        const StateKey s_key = {s.position, s.visited_delivery};
        
        if (CLOSED.count(s_key) > 0) {
            continue;
        }
        
        if (s.visited_delivery && is_dock_position(s.position, docks) && s.length <= max_steps) {
            AStarResult result;
            result.success = true;
            result.total_length = s.length;
            result.total_density_cost = s.g - LAMBDA * s.length;
            
            auto cur = std::make_shared<UnifiedState>(s);
            while (cur) {
                result.path.push_back(cur->position);
                cur = cur->parent;
            }
            std::reverse(result.path.begin(), result.path.end());
            return result;
        }
        
        CLOSED.insert(s_key);
        
        bool new_visited = s.visited_delivery;
        if (sites_equal(s.position, delivery) && !s.visited_delivery) {
            new_visited = true;
        }
        
        for (int i = 0; i < NUM_DIRECTIONS; i++) {
            const int ne = static_cast<int>(s.position.e) + DIRECTION_DE[i];
            const int nn = static_cast<int>(s.position.n) + DIRECTION_DN[i];
            
            if (!is_site_valid(map, ne, nn)) {
                continue;
            }
            
            const Site s_prime = {static_cast<size_t>(ne), static_cast<size_t>(nn)};
            const StateKey s_prime_key = {s_prime, new_visited};
            
            if (CLOSED.count(s_prime_key) > 0) {
                continue;
            }
            
            const int new_len = s.length + 1;
            if (new_len > max_steps) {
                continue;
            }
            
            const double edge_cost = get_density_at(map, s_prime) + LAMBDA;
            const double g_new = s.g + edge_cost;
            
            if (g_val.count(s_prime_key) == 0 || g_new < g_val[s_prime_key]) {
                g_val[s_prime_key] = g_new;
                const double h_prime = compute_unified_heuristic(
                    s_prime, new_visited, delivery, h_delivery, h_docks_min, h_delivery_to_docks
                );
                const double f_new = g_new + h_prime;
                OPEN.push({s_prime, new_visited, g_new, f_new, new_len, std::make_shared<UnifiedState>(s)});
            }
        }
    }
    
    return AStarResult();
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
    AStarResult result;
    double lambda_used = LAMBDA_START;
    
    while (lambda_used <= LAMBDA_MAX) {
        LAMBDA = lambda_used;
        
        // Precompute heuristics
        CostMap h_delivery = compute_cost_map(map, order.delivery);
        CostMap h_docks_min = compute_min_cost_to_any_dock(map, map.docks);
        
        // Min cost from delivery to any dock (h_docks_min already has this)
        double h_delivery_to_docks = h_docks_min[order.delivery.e][order.delivery.n];
        
        result = astar_search_unified(
            map, order.origin_dock, order.delivery, map.docks,
            h_delivery, h_docks_min, h_delivery_to_docks, MAX_FLIGHT_LENGTH
        );
        
        if (result.success) {
            last_used_lambda = lambda_used;
            break;
        }
        lambda_used += LAMBDA_INCREMENT;
    }
    
    if (!result.success) {
        last_used_lambda = lambda_used;
        return Path{{}, {}};
    }
    
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
    
    if (!inbound.empty() && !sites_equal(inbound.front(), order.delivery)) {
        inbound.insert(inbound.begin(), order.delivery);
    }
    
    return Path{outbound, inbound};
}

double OrderPlanner::get_last_used_lambda() {
    return last_used_lambda;
}
