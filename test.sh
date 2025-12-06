#!/usr/bin/env bash

set -e

MAP=sf.txt
NUM_ORDERS=5

# Build the order_planner binary
echo "Building..."
make -C cpp
echo "Done building."

# Generate some random orders based on population density
echo "Generating orders..."
python3 analysis.py $MAP --generate_random_orders $NUM_ORDERS > orders.txt
echo "Done generating orders."

# Run the planner to come up with paths for the orders
echo "Running..."
./cpp/order_planner $MAP orders.txt > paths.txt
echo "Done running."

# Evaluate the risk and lengths of the paths
echo "Evaluating paths..."
python3 analysis.py $MAP --paths paths.txt --plot
echo "Done evaluating paths."
