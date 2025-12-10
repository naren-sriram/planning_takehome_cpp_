#pragma once

#include "utils.h"
#include <vector>

namespace OrderPlanner {
    // ========== Constants ==========
    
    constexpr int MAX_STEPS = MAX_FLIGHT_LENGTH;
    constexpr int PROFILE_SIZE = MAX_STEPS + 1;  // 0..110 inclusive
    
    // ========== Data Structures ==========
    
    // Augmented state: (row, col, steps_taken)
    struct AugmentedState {
        size_t e;
        size_t n;
        int steps;
        int risk;
        
        bool operator>(const AugmentedState& other) const {
            if (risk != other.risk) return risk > other.risk;
            return steps > other.steps;
        }
    };
    
    // Search profile: minimum risk for each step count (0..110)
    using RiskProfile = std::vector<int>;
    
    // Path reconstruction data
    struct PathNode {
        Site position;
        int steps;
        std::shared_ptr<PathNode> parent;
    };
    
    // ========== Core Functions ==========
    
    // Compute distance from all cells to nearest dock (BFS) for heuristic
    std::vector<std::vector<int>> compute_distance_to_nearest_dock(
        const Map& map,
        const std::vector<Site>& docks
    );
    
    // Phase A: Outbound search (Start → Delivery)
    // Returns profile[111] where profile[k] = min risk to reach delivery in exactly k steps
    // Also stores the goal state for path reconstruction
    struct OutboundResult {
        RiskProfile profile;
        Site goal_state;
        int goal_steps;
    };
    
    OutboundResult search_outbound(
        const Map& map,
        const Site& start,
        const Site& delivery,
        std::vector<std::vector<std::vector<std::shared_ptr<PathNode>>>>& parent_map
    );
    
    // Phase B: Inbound search (Delivery → Any Dock)
    // Returns profile[111] where profile[k] = min risk to reach any dock in exactly k steps
    // Also stores which dock was reached at each step count
    struct InboundResult {
        RiskProfile profile;
        std::vector<Site> goal_states;  // goal_states[k] = dock reached at step k
    };
    
    InboundResult search_inbound(
        const Map& map,
        const Site& delivery,
        const std::vector<Site>& docks,
        const std::vector<std::vector<int>>& dist_to_docks,
        std::vector<std::vector<std::vector<std::shared_ptr<PathNode>>>>& parent_map
    );
    
    // Phase C: Merge profiles and find optimal (i, j) where i + j <= 110
    struct MergeResult {
        int outbound_steps;
        int inbound_steps;
        int total_risk;
        bool success;
    };
    
    MergeResult merge_profiles(
        const RiskProfile& outbound_profile,
        const RiskProfile& inbound_profile
    );
    
    // Reconstruct path from parent map
    std::vector<Site> reconstruct_path(
        const Site& start,
        int steps,
        const std::vector<std::vector<std::vector<std::shared_ptr<PathNode>>>>& parent_map
    );
    
    // Main entry point
    Path find_path(const Map& map, const Order& order);
    
    // For ablation study compatibility
    double get_last_used_lambda();
}
