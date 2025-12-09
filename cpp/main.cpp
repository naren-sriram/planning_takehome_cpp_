#include "utils.h"
#include "planner.h"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

using namespace OrderPlanner;

int main(int argc, char *argv[]) {
    if (argc < 3 || argc > 4) {
        std::cerr << "Usage: " << argv[0] << " <map> <orders> [lambda]" << std::endl;
        return 1;
    }

    const std::string map_filename = argv[1]; 
    const std::string orders_filename = argv[2];
    if (argc == 4) LAMBDA = std::stod(argv[3]);

    const Map map = read_map(map_filename);
    const std::vector<Order> orders = read_orders(orders_filename);

    std::ofstream paths_file("paths.txt");
    int order_num = 0;
    for (const Order &order : orders) {
        Path path = find_path(map, order);
        print_path(path, paths_file);
        
        int density = compute_path_density(map, path);
        log_ablation_result("ablation_results.csv", LAMBDA, order_num++,
            path.outbound.size(), path.inbound.size(), density);
    }

    return 0;
}
