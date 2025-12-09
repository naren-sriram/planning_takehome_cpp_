#pragma once

#include <string>
#include <vector>
#include <iostream>

namespace OrderPlanner {
    // ========== Core Data Structures ==========

    struct Site {
        // The East coordinate.
        size_t e;
        // The North coordinate.
        size_t n;
    };

    struct Order {
        // The dock where the drone takes off from.
        Site origin_dock;
        // The site where the order is to be delivered.
        Site delivery;
    };

    struct Map {
        // The docks are the locations where the drones can pick up and drop off orders.
        std::vector<Site> docks;
        // A 2D grid of population density values. The density value at (e, n) is stored in
        // density[e][n].
        std::vector<std::vector<uint8_t> > density;
    };

    struct Path {
        std::vector<Site> outbound;
        std::vector<Site> inbound;
    };

    // ========== Constants ==========

    // Maximum total flight length in steps
    constexpr int MAX_FLIGHT_LENGTH = 110;

    // 8-connected grid directions (N, S, E, W, NE, NW, SE, SW)
    constexpr int NUM_DIRECTIONS = 8;
    constexpr int DIRECTION_DE[NUM_DIRECTIONS] = {-1, 1, 0, 0, -1, -1, 1, 1};
    constexpr int DIRECTION_DN[NUM_DIRECTIONS] = {0, 0, -1, 1, -1, 1, -1, 1};

    // ========== Helper Functions ==========

    // Check if two sites are equal
    bool sites_equal(const Site& a, const Site& b);

    // Get map dimensions
    inline size_t get_map_rows(const Map& map) { return map.density.size(); }
    inline size_t get_map_cols(const Map& map) { return map.density.empty() ? 0 : map.density[0].size(); }

    // Check if a site is within map bounds
    bool is_site_valid(const Map& map, int e, int n);

    // Get density value at a site (returns 0 if out of bounds)
    uint8_t get_density_at(const Map& map, const Site& site);

    // ========== I/O Functions ==========

    std::vector<Site> read_sites(std::string line);
    Map read_map(std::string filename);
    std::vector<Order> read_orders(std::string filename);
    void print_path(const Path &path, std::ostream& out = std::cout);
    
    // ========== Ablation Study ==========
    
    // Compute total density cost of a path
    int compute_path_density(const Map& map, const Path& path);
    
    // Log result to CSV file for ablation study
    void log_ablation_result(
        const std::string& filename,
        double lambda,
        int order_num,
        int outbound_steps,
        int inbound_steps,
        int total_density
    );
}
