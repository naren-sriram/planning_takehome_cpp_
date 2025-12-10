#include "utils.h"
#include "planner.h"

#include <iostream>
#include <fstream>
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

    std::ofstream paths_file("paths.txt");
    int order_num = 0;
    for (const Order &order : orders) {
        Path path = find_path(map, order);
        print_path(path, paths_file);
        
        int density = compute_path_density(map, path);
        double lambda_used = get_last_used_lambda();
        // Steps = nodes - 1 for each segment
        int outbound_steps = path.outbound.empty() ? 0 : path.outbound.size() - 1;
        int inbound_steps = path.inbound.empty() ? 0 : path.inbound.size() - 1;
        log_ablation_result("ablation_results.csv", lambda_used, order_num++,
            outbound_steps, inbound_steps, density);
    }

    return 0;
}
