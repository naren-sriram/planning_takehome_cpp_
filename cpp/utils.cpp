#include "utils.h"

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

using namespace OrderPlanner;

std::vector<Site> OrderPlanner::read_sites(std::string line) {
    std::vector<Site> sites;

    std::istringstream ss(line);
    std::string token;
    while (std::getline(ss, token, ' ')) {
        std::istringstream ss2(token);
        std::string e_str, n_str;
        std::getline(ss2, e_str, ',');
        std::getline(ss2, n_str);
        Site site = {static_cast<size_t>(std::stoi(e_str)), static_cast<size_t>(std::stoi(n_str))};
        sites.push_back(site);
    }

    return sites;
}

// Read the density map from a file
Map OrderPlanner::read_map(std::string filename) {
    // The file format is as follows:
    //
    // <number of rows m> <number of columns n>
    // <density value at (0, 0)> <density value at (0, 1)> ... <density value at (0, n-1)>
    // <density value at (1, 0)> <density value at (1, 1)> ... <density value at (1, n-1)>
    // ...
    // <density value at (m-1, 0)> <density value at (m-1, 1)> ... <density value at (m-1, n-1)>
    // <dock_0_e>,<dock_0_n> <dock_1_e>,<dock_1_n> ... <dock_m_e>,<dock_m_n>

    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: could not open file " << filename << std::endl;
        exit(1);
    }

    // The first two values are the number of rows and columns
    size_t rows, cols;
    file >> rows >> cols;

    // The next rows * cols values are the density values
    std::vector<std::vector<uint8_t> > density(rows, std::vector<uint8_t>(cols));
    for (size_t e = 0; e < rows; e++) {
        for (size_t n = 0; n < cols; n++) {
            int value;
            file >> value;
            density[e][n] = value;
        }
    }

    std::string line;
    std::getline(file, line); // Skip the newline

    // Parse the docks
    std::getline(file, line);
    const std::vector<Site> docks = read_sites(line);

    return {docks, density};
}

// Read a list of orders from a file
std::vector<Order> OrderPlanner::read_orders(std::string filename) {
    // The file format is as follows:
    //
    // <origin dock e>,<origin dock n> <delivery e>,<delivery n> <destination dock e>,<destination dock n>

    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: could not open file " << filename << std::endl;
        exit(1);
    }

    std::vector<Order> orders;

    std::string line;
    while (std::getline(file, line)) {
        std::vector<Site> waypoints = read_sites(line);

        const Order order = {
            .origin_dock = waypoints[0],
            .delivery = waypoints[1],
        };
        orders.push_back(order);
    }

    return orders;
}

void OrderPlanner::print_path(const Path &path) {
    for (const Site &site : path.outbound) {
        std::cout << site.e << "," << site.n << " ";
    }
    std::cout << "// ";
    for (const Site &site : path.inbound) {
        std::cout << site.e << "," << site.n << " ";
    }
    std::cout << std::endl;
}
