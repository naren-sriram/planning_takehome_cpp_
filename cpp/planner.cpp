#include "planner.h"
#include "utils.h"

#include <random>
#include <vector>

using namespace OrderPlanner;

// Plan a path between two sites. The path includes the start and end sites.
std::vector<Site> OrderPlanner::plan(const Map &map, const Site &start, const Site &end) {
    // TODO: We should consider the density map when planning the path.
    // TODO: We should make sure the total path length doesn't exceed the maximum range of the zip.
    std::vector<Site> path;
    path.push_back(start);

    size_t e = start.e;
    size_t n = start.n;

    while (e != end.e || n != end.n) {
        if (e < end.e) {
            e += 1;
        } else if (e > end.e) {
            e -= 1;
        }

        if (n < end.n) {
            n += 1;
        } else if (n > end.n) {
            n -= 1;
        }

        path.push_back({e, n});
    }

    return path;
}

// Plan a complete path for an order.
Path OrderPlanner::find_path(const Map &map, const Order &order) {
    // TODO: Can we do better than this?
    const Site destination_dock = map.docks[std::rand() % map.docks.size()];

    Path path = {
        .outbound = plan(map, order.origin_dock, order.delivery),
        .inbound = plan(map, order.delivery, destination_dock),
    };

    return path;
}
