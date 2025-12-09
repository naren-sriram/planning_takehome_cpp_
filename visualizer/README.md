# Drone Path Visualizer

A simple, effective visualizer for the drone delivery path planning system.

## Features

- **Color-coded density map**: Yellow to red gradient (0-9 intensity)
- **Density numbers**: Displayed inside grid cells
- **Highlighted markers**:
  - Green circles: Start (origin dock) and Goal (delivery site)
  - Purple circle: Destination dock
  - Black squares: All available docks
- **Animated robot**: Blue circle moving step-by-step along the path
- **Two-leg visualization**:
  - Leg 1: Origin dock → Delivery site
  - Leg 2: Delivery site → Destination dock
- **Summary display**: Shows complete statistics after each order

### Interactive Controls

- **▶️ Play/Pause Button**: Pause and resume the animation at any time
  - Red = Playing
  - Green = Paused
- **🔄 Replay Button**: Restart the current order from the beginning
  - Resets robot to origin dock
  - Clears path trace

## Requirements

```bash
pip install matplotlib numpy
```

## Usage

### Basic Usage

```bash
cd visualizer
python path_visualizer.py ../sf.txt ../orders.txt ../paths.txt
```

### From Project Root

```bash
python visualizer/path_visualizer.py sf.txt orders.txt paths.txt
```

## What You'll See

1. **Grid Display**:
   - Color intensity increases with population density
   - Density values shown in cells (sampled for clarity)

2. **Animation**:
   - Blue circle (robot) moves from grid to grid
   - Blue line traces the path taken
   - Status shows current leg and step count

3. **Order Summary** (after completion):
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

4. **Multiple Orders**: Close the window to proceed to the next order

## Controls

### Interactive Buttons (Bottom of Window)
- **Pause/Play**: Click to pause or resume the animation
  - Button color changes: Red (playing) → Green (paused)
- **Replay**: Restart the current order from the beginning
  - Resets animation to first step

### Window Controls
- **Close window**: Proceed to next order (or exit if last order)
- **Animation speed**: Default 100ms per step (adjustable in code if needed)

## File Format

### Expected Inputs

- **Map file** (`sf.txt`): Grid dimensions, density values, dock locations
- **Orders file** (`orders.txt`): Origin dock and delivery locations
- **Paths file** (`paths.txt`): Computed paths (outbound // inbound)

## Customization

Edit `path_visualizer.py` to adjust:
- `animation_speed`: Animation speed (milliseconds per step)
- Color maps: Change `cmap='YlOrRd'` to other matplotlib colormaps
- Marker sizes and colors
- Grid sampling rate for density numbers
