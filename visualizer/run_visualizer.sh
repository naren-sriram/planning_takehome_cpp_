#!/bin/bash
# Convenient script to run the visualizer

cd "$(dirname "$0")"

if [ ! -f "../sf.txt" ] || [ ! -f "../orders.txt" ] || [ ! -f "../paths.txt" ]; then
    echo "Error: Required files not found!"
    echo "Please ensure sf.txt, orders.txt, and paths.txt exist in the parent directory."
    exit 1
fi

echo "Starting Drone Path Visualizer..."
echo ""

# Check if we're in a virtual environment
if [ -d "../planning_env" ]; then
    echo "Activating virtual environment..."
    source ../planning_env/bin/activate
fi

python path_visualizer.py ../sf.txt ../orders.txt ../paths.txt
