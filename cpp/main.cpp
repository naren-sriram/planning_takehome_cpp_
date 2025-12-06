#include "utils.h"
#include "planner.h"

#include <iostream>
#include <string>
#include <vector>

using namespace OrderPlanner;

int main(int argc, char *argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <map> <orders>" << std::endl;
        return 1;
    }

    const std::string map_filename = argv[1]; 
    const std::string orders_filename = argv[2];

    const Map map = read_map(map_filename);
    const std::vector<Order> orders = read_orders(orders_filename);

    for (const Order &order : orders) {
        Path path = find_path(map, order);
        print_path(path);
    }

    return 0;
}
