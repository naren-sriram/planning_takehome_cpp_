# Visualizer Demo

## Quick Start

### Option 1: Using the run script (Recommended)

```bash
cd visualizer
./run_visualizer.sh
```

### Option 2: Direct execution

```bash
cd visualizer
source ../planning_env/bin/activate
python path_visualizer.py ../sf.txt ../orders.txt ../paths.txt
```

## What to Expect

### Order 1 Visualization
- **Origin**: (50, 15) - marked with green circle
- **Delivery**: (34, 49) - marked with green circle
- **Destination Dock**: (55, 23) - marked with purple circle
- **Blue robot** animates moving:
  - Leg 1: 56 steps from origin to delivery
  - Leg 2: 46 steps from delivery to dock
- **Path traced** in blue behind the robot
- **Final summary** displayed showing total steps and density

### Animation Features
- **Color map**: Yellow (low density 0) → Red (high density 9)
- **Grid**: Shows all cells with density values (sampled)
- **All docks**: Shown as small black squares
- **Speed**: ~100ms per step (adjustable in code)

### Statistics Display
After each leg completes, you'll see:
```
ORDER 1 COMPLETE!
─────────────────────────
Start:  (50, 15)
Goal:   (34, 49)
Dock:   (55, 23)
─────────────────────────
Total Steps:   102
Total Density: 20.0
─────────────────────────
Leg 1: 56 steps, density=10.0
Leg 2: 46 steps, density=10.0
```

## Interactive Controls

### Button Controls (Bottom of Window)

1. **⏸️ Pause/Play Button** (Red/Green)
   - Click to pause the animation
   - Click again to resume from the same position
   - Button color: Red = Playing, Green = Paused
   - Great for examining specific positions!

2. **🔄 Replay Button** (Blue)
   - Click to restart the current order from the beginning
   - Animation resets to the origin dock
   - Perfect for reviewing a path multiple times

### Tips

1. **Pause frequently**: Pause to examine density values and the robot's position
2. **Replay for clarity**: Watch the same order multiple times to understand the strategy
3. **Study the path**: Notice how A* avoids high-density (red) areas
4. **Compare legs**: See how budget is allocated between leg 1 and leg 2
5. **Save**: Use matplotlib's save button (💾) in the toolbar to export as PNG

## Troubleshooting

### "No module named matplotlib"
```bash
source ../planning_env/bin/activate
pip install matplotlib numpy
```

### "File not found" errors
Make sure you're in the visualizer directory and that:
- `../sf.txt` exists (map file)
- `../orders.txt` exists (orders)
- `../paths.txt` exists (computed paths)

Generate paths if missing:
```bash
cd ..
./cpp/order_planner sf.txt orders.txt > paths.txt
```

## Example Session

```bash
$ cd visualizer
$ ./run_visualizer.sh

Starting Drone Path Visualizer...

Drone Path Visualizer
==================================================
Map:    ../sf.txt
Orders: ../orders.txt
Paths:  ../paths.txt
==================================================
Close the window to proceed to the next order...

[Window opens with animated visualization]
[Close window when done viewing]
[Next order begins automatically]
```

Enjoy watching your optimized paths in action! 🚁
