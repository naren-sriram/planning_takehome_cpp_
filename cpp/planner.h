#pragma once

#include "utils.h"

#include <vector>

namespace OrderPlanner {
    std::vector<Site> plan(const Map &map, const Site &start, const Site &end);
    Path find_path(const Map &map, const Order &order);
}
