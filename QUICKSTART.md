# Quick Start Guide

## Generate and Visualize in 3 Steps

### 1️⃣ Generate Random Orders
```bash
source planning_env/bin/activate
python analysis.py --generate_random_orders 5 sf.txt > orders.txt
```
✅ Creates `orders.txt` with 5 random delivery orders

### 2️⃣ Plan Optimal Paths
```bash
./cpp/order_planner sf.txt orders.txt > paths.txt
```
✅ Creates `paths.txt` with A*-optimized paths

### 3️⃣ Visualize
```bash
cd visualizer
./run_visualizer.sh
```
✅ Opens interactive animation showing:
- Color-coded density map
- Robot moving step-by-step
- Path statistics
- Pause/Replay controls

## That's It!

Close each order's window to see the next one.

---

## Want Different Number of Orders?

Change the number in step 1:
```bash
# Just 1 order
python analysis.py --generate_random_orders 1 sf.txt > orders.txt

# 20 orders
python analysis.py --generate_random_orders 20 sf.txt > orders.txt
```

Then repeat steps 2 and 3.

---

## File Flow

```
orders.txt  ──┐
sf.txt      ──┼──> cpp/order_planner ──> paths.txt ──> visualizer
              │
              └──────────────────────────────────────────┘
```

**Yes, once you have `paths.txt`, the visualizer works!**
