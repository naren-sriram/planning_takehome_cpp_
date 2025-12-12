#include "planner.h"
#include "utils.h"

#include <queue>
#include <memory>
#include <algorithm>
#include <climits>

using namespace OrderPlanner;

static double last_used_lambda = 0.0;

//  Distance to Nearest Dock (BFS) 

std::vector<std::vector<int>> OrderPlanner::compute_distance_to_nearest_dock(
    const Map& map,
    const std::vector<Site>& docks
) {
    const size_t rows = get_map_rows(map);
    const size_t cols = get_map_cols(map);
    std::vector<std::vector<int>> dist(rows, std::vector<int>(cols, INT_MAX));
    
    std::queue<std::pair<size_t, size_t>> q;
    
    for (const auto& dock : docks) {
        dist[dock.e][dock.n] = 0;
        q.push({dock.e, dock.n});
    }
    
    while (!q.empty()) {
        const auto [e, n] = q.front();
        q.pop();
        
        for (int i = 0; i < NUM_DIRECTIONS; i++) {
            const int ne = static_cast<int>(e) + DIRECTION_DE[i];
            const int nn = static_cast<int>(n) + DIRECTION_DN[i];
            
            if (!is_site_valid(map, ne, nn)) {
                continue;
            }
            
            if (dist[ne][nn] == INT_MAX) {
                dist[ne][nn] = dist[e][n] + 1;
                q.push({static_cast<size_t>(ne), static_cast<size_t>(nn)});
            }
        }
    }
    
    return dist;
}

//  Outbound Search 

OutboundResult OrderPlanner::search_outbound(
    const Map& map,
    const Site& start,
    const Site& delivery,
    std::vector<std::vector<std::vector<std::shared_ptr<PathNode>>>>& parent_map
) {
    const size_t rows = get_map_rows(map);
    const size_t cols = get_map_cols(map);
    
    std::vector<std::vector<std::vector<int>>> min_risk(
        rows, std::vector<std::vector<int>>(cols, std::vector<int>(PROFILE_SIZE, INT_MAX))
    );
    
    parent_map.assign(rows, std::vector<std::vector<std::shared_ptr<PathNode>>>(
        cols, std::vector<std::shared_ptr<PathNode>>(PROFILE_SIZE, nullptr)
    ));
    
    auto heuristic = [&](size_t e, size_t n) -> int {
        const int de = static_cast<int>(e) - static_cast<int>(delivery.e);
        const int dn = static_cast<int>(n) - static_cast<int>(delivery.n);
        return std::max(std::abs(de), std::abs(dn));
    };
    
    std::priority_queue<AugmentedState, std::vector<AugmentedState>, std::greater<AugmentedState>> pq;
    
    min_risk[start.e][start.n][0] = get_density_at(map, start);
    pq.push({start, 0, get_density_at(map, start)});
    
    RiskProfile profile(PROFILE_SIZE, INT_MAX);
    OutboundResult result;
    result.goal_state = delivery;
    result.goal_steps = -1;
    
    while (!pq.empty()) {
        const AugmentedState current = pq.top();
        pq.pop();
        
        if (current.risk > min_risk[current.position.e][current.position.n][current.steps]) {
            continue;
        }
        
        if (sites_equal(current.position, delivery)) {
            if (current.risk < profile[current.steps]) {
                profile[current.steps] = current.risk;
                if (result.goal_steps == -1 || current.steps < result.goal_steps) {
                    result.goal_steps = current.steps;
                }
            }
        }
        
        if (current.steps + heuristic(current.position.e, current.position.n) > MAX_STEPS) {
            continue;
        }
        
        for (int i = 0; i < NUM_DIRECTIONS; i++) {
            const int ne = static_cast<int>(current.position.e) + DIRECTION_DE[i];
            const int nn = static_cast<int>(current.position.n) + DIRECTION_DN[i];
            
            if (!is_site_valid(map, ne, nn)) {
                continue;
            }
            
            const int new_steps = current.steps + 1;
            if (new_steps > MAX_STEPS) {
                continue;
            }
            
            const Site neighbor = {static_cast<size_t>(ne), static_cast<size_t>(nn)};
            const int new_risk = current.risk + get_density_at(map, neighbor);
            
            if (new_risk < min_risk[ne][nn][new_steps]) {
                min_risk[ne][nn][new_steps] = new_risk;
                
                auto parent_node = std::make_shared<PathNode>();
                parent_node->position = current.position;
                parent_node->steps = current.steps;
                parent_map[ne][nn][new_steps] = parent_node;
                
                pq.push({{static_cast<size_t>(ne), static_cast<size_t>(nn)}, new_steps, new_risk});
            }
        }
    }
    
    result.profile = profile;
    return result;
}

// Inbound Search 

InboundResult OrderPlanner::search_inbound(
    const Map& map,
    const Site& delivery,
    const std::vector<Site>& docks,
    const std::vector<std::vector<int>>& dist_to_docks,
    std::vector<std::vector<std::vector<std::shared_ptr<PathNode>>>>& parent_map
) {
    const size_t rows = get_map_rows(map);
    const size_t cols = get_map_cols(map);
    
    std::vector<std::vector<std::vector<int>>> min_risk(
        rows, std::vector<std::vector<int>>(cols, std::vector<int>(PROFILE_SIZE, INT_MAX))
    );
    
    parent_map.assign(rows, std::vector<std::vector<std::shared_ptr<PathNode>>>(
        cols, std::vector<std::shared_ptr<PathNode>>(PROFILE_SIZE, nullptr)
    ));
    
    auto is_dock = [&](size_t e, size_t n) -> bool {
        for (const auto& dock : docks) {
            if (dock.e == e && dock.n == n) {
                return true;
            }
        }
        return false;
    };
    
    auto heuristic = [&](size_t e, size_t n) -> int {
        return dist_to_docks[e][n];
    };
    
    std::priority_queue<AugmentedState, std::vector<AugmentedState>, std::greater<AugmentedState>> pq;
    
    min_risk[delivery.e][delivery.n][0] = get_density_at(map, delivery);
    pq.push({delivery, 0, get_density_at(map, delivery)});
    
    RiskProfile profile(PROFILE_SIZE, INT_MAX);
    InboundResult result;
    result.goal_states.assign(PROFILE_SIZE, delivery);
    
    while (!pq.empty()) {
        const AugmentedState current = pq.top();
        pq.pop();
        
        if (current.risk > min_risk[current.position.e][current.position.n][current.steps]) {
            continue;
        }
        
        if (is_dock(current.position.e, current.position.n)) {
            if (current.risk < profile[current.steps]) {
                profile[current.steps] = current.risk;
                result.goal_states[current.steps] = current.position;
            }
        }
        
        if (current.steps + heuristic(current.position.e, current.position.n) > MAX_STEPS) {
            continue;
        }
        
        for (int i = 0; i < NUM_DIRECTIONS; i++) {
            const int ne = static_cast<int>(current.position.e) + DIRECTION_DE[i];
            const int nn = static_cast<int>(current.position.n) + DIRECTION_DN[i];
            
            if (!is_site_valid(map, ne, nn)) {
                continue;
            }
            
            const int new_steps = current.steps + 1;
            if (new_steps > MAX_STEPS) {
                continue;
            }
            
            const Site neighbor = {static_cast<size_t>(ne), static_cast<size_t>(nn)};
            const int new_risk = current.risk + get_density_at(map, neighbor);
            
            if (new_risk < min_risk[ne][nn][new_steps]) {
                min_risk[ne][nn][new_steps] = new_risk;
                
                auto parent_node = std::make_shared<PathNode>();
                parent_node->position = current.position;
                parent_node->steps = current.steps;
                parent_map[ne][nn][new_steps] = parent_node;
                
                pq.push({{static_cast<size_t>(ne), static_cast<size_t>(nn)}, new_steps, new_risk});
            }
        }
    }
    
    result.profile = profile;
    return result;
}

// Merge Profiles 

MergeResult OrderPlanner::merge_profiles(
    const RiskProfile& outbound_profile,
    const RiskProfile& inbound_profile
) {
    MergeResult result = {-1, -1, INT_MAX, false};
    
    for (int i = 0; i < PROFILE_SIZE; i++) {
        if (outbound_profile[i] == INT_MAX) {
            continue;
        }
        
        for (int j = 0; j < MAX_STEPS - i; j++) {
            if (inbound_profile[j] == INT_MAX) {
                continue;
            }
            
            const int total_risk = outbound_profile[i] + inbound_profile[j];
            if (total_risk < result.total_risk) {
                result.total_risk = total_risk;
                result.outbound_steps = i;
                result.inbound_steps = j;
                result.success = true;
            }
        }
    }
    
    return result;
}

// Path Reconstruction 

std::vector<Site> OrderPlanner::reconstruct_path(
    const Site& goal,
    int goal_steps,
    const std::vector<std::vector<std::vector<std::shared_ptr<PathNode>>>>& parent_map
) {
    std::vector<Site> path;
    
    Site current = goal;
    int current_steps = goal_steps;
    
    while (current_steps >= 0) {
        path.push_back(current);
        
        const auto& parent = parent_map[current.e][current.n][current_steps];
        if (!parent) {
            break;
        }
        
        current = parent->position;
        current_steps = parent->steps;
    }
    
    std::reverse(path.begin(), path.end());
    return path;
}



Path OrderPlanner::find_path(const Map& map, const Order& order) {
    last_used_lambda = 0.0;  // Not used in this approach, but kept for compatibility
    
    // Precompute distance to nearest dock for heuristic
    const auto dist_to_docks = compute_distance_to_nearest_dock(map, map.docks);
    
    // Note that outbound search and inbound search are independent of each other. Result of one
    // does not affect the result of the other. They both affect the final path.

    // Outbound search
    std::vector<std::vector<std::vector<std::shared_ptr<PathNode>>>> outbound_parents;
    const auto outbound_result = search_outbound(map, order.origin_dock, order.delivery, outbound_parents);
    
    // Inbound search
    std::vector<std::vector<std::vector<std::shared_ptr<PathNode>>>> inbound_parents;
    const auto inbound_result = search_inbound(
        map, order.delivery, map.docks, dist_to_docks, inbound_parents
    );
    
    // Merge profiles
    const auto merge_result = merge_profiles(outbound_result.profile, inbound_result.profile);
    
    if (!merge_result.success) {
        return Path{{}, {}};
    }
    
    // Reconstruct outbound path: origin → delivery
    std::vector<Site> outbound_path;
    if (merge_result.outbound_steps > 0) {
        outbound_path = reconstruct_path(order.delivery, merge_result.outbound_steps, outbound_parents);
    } else {
        outbound_path = {order.origin_dock, order.delivery};
    }
    
    // Reconstruct inbound path: delivery → dock
    std::vector<Site> inbound_path;
    if (merge_result.inbound_steps > 0) {
        const Site& goal_dock = inbound_result.goal_states[merge_result.inbound_steps];
        inbound_path = reconstruct_path(goal_dock, merge_result.inbound_steps, inbound_parents);
    } else {
        inbound_path = {order.delivery};
    }
    
    return Path{outbound_path, inbound_path};
}

double OrderPlanner::get_last_used_lambda() {
    return last_used_lambda;
}
