# Interactive Controls Quick Reference

## Control Panel Layout

```
┌─────────────────────────────────────────────────────────┐
│                  DENSITY MAP VISUALIZATION               │
│                 (Blue robot moving on grid)              │
│                                                          │
│  [Green circles: Start & Goal]                          │
│  [Purple circle: Destination Dock]                      │
│  [Blue line: Path trace]                                │
└─────────────────────────────────────────────────────────┘

┌─────────┬─────────┬────────────────────────────────────┐
│  Pause  │ Replay  │  Speed: [====o====]  100 ms/step  │
│  (Red)  │ (Blue)  │       10ms ←→ 500ms               │
└─────────┴─────────┴────────────────────────────────────┘
```

## Button Functions

### ⏸️ Pause/Play Button
**Location**: Bottom-left
**Colors**:
- Red = Playing
- Green = Paused

**Usage**:
```
Click → Pause animation
Click again → Resume from current position
```

### 🔄 Replay Button
**Location**: Bottom-center-left
**Color**: Blue

**Usage**:
```
Click → Restart current order from beginning
       → Resets to origin dock
       → Clears path trace
```

### ⚡ Speed Slider
**Location**: Bottom-center-right
**Color**: Green
**Range**: 10ms - 500ms per step

**Usage**:
```
Drag left  → Faster (10ms)   [Quick overview]
Center     → Default (100ms) [Normal viewing]
Drag right → Slower (500ms)  [Detailed analysis]
```

**Real-time**: Speed changes immediately while animation is running!

## Typical Workflow

### First Viewing (Understand the Path)
1. Start with **slow speed** (300-500ms)
2. Watch each step carefully
3. Use **Pause** to examine specific positions
4. Note how A* avoids high-density areas

### Quick Review (See Overall Strategy)
1. Click **Replay** to restart
2. Set **fast speed** (10-50ms)
3. Watch the complete path quickly
4. Compare both legs

### Detailed Analysis (Study Specific Sections)
1. Set **slow speed** (400-500ms)
2. **Pause** when robot reaches interesting positions
3. Examine density values around the robot
4. **Resume** to continue
5. Use **Replay** to watch again if needed

## Keyboard Shortcuts

None currently implemented. Use mouse for all controls.

## Tips for Best Experience

1. **Start slow**: Begin with 400-500ms to understand the path
2. **Pause frequently**: Pause to read density values on the map
3. **Speed up for overview**: Use 10-50ms to see the big picture
4. **Replay multiple times**: Watch at different speeds to see different aspects
5. **Compare legs**: Notice how leg 2 budget depends on leg 1 usage

## What to Look For

### During Animation:
- **Robot position**: Current grid cell (blue circle)
- **Path trace**: Where the robot has been (blue line)
- **Density values**: Numbers in each cell
- **Color intensity**: Red = high density, Yellow = low density
- **Detours**: Robot taking longer paths to avoid red areas

### In Summary (After Completion):
- **Total steps**: Did it use most of the 110-step budget?
- **Density cost**: How low compared to naive approach?
- **Dock choice**: Which dock was selected and why?
- **Leg comparison**: How were steps allocated between legs?

## Common Questions

**Q: Can I go backwards?**
A: No, but use Replay to watch from the beginning.

**Q: Can I skip to the next order?**
A: Close the window to proceed to the next order.

**Q: Can I pause between orders?**
A: Yes, each order is a separate window.

**Q: Can I save the animation?**
A: Use matplotlib's save button (💾) in the toolbar for static images.
For videos, use screen recording software.

**Q: Why does the speed slider show "ms/step"?**
A: Milliseconds per step. Lower = faster, higher = slower.

**Q: Can I change the speed while paused?**
A: Yes! Adjust anytime, even when paused.
