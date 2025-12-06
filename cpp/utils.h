#pragma once

#include <string>
#include <vector>

namespace OrderPlanner {
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

    std::vector<Site> read_sites(std::string line);
    Map read_map(std::string filename);
    std::vector<Order> read_orders(std::string filename);
    void print_path(const Path &path);
}
