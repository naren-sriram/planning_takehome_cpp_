#!/usr/bin/env python3
"""
Drone Path Visualizer
Visualizes the drone path on a density map with step-by-step animation.
"""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, Slider
import numpy as np
import sys
from pathlib import Path

class PathVisualizer:
    def __init__(self, map_file, orders_file, paths_file):
        """Initialize the visualizer with map, orders, and paths."""
        self.map_data = self.load_map(map_file)
        self.orders = self.load_orders(orders_file)
        self.paths = self.load_paths(paths_file)

        self.current_order = 0
        self.current_step = 0
        self.current_leg = 0  # 0 = outbound, 1 = inbound
        self.animation_speed = 100  # milliseconds per step
        self.is_paused = False
        self.animation = None

    def load_map(self, map_file):
        """Load the density map and dock locations."""
        with open(map_file, 'r') as f:
            lines = f.readlines()

        # First line: dimensions
        rows, cols = map(int, lines[0].strip().split())

        # Next rows: density values
        density = []
        for i in range(1, rows + 1):
            row = list(map(int, lines[i].strip().split()))
            density.append(row)

        # Last line: dock locations
        docks = []
        dock_line = lines[-1].strip().split()
        for dock_str in dock_line:
            e, n = map(int, dock_str.split(','))
            docks.append((e, n))

        return {
            'rows': rows,
            'cols': cols,
            'density': np.array(density),
            'docks': docks
        }

    def load_orders(self, orders_file):
        """Load delivery orders."""
        orders = []
        with open(orders_file, 'r') as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) >= 2:
                    origin_e, origin_n = map(int, parts[0].split(','))
                    delivery_e, delivery_n = map(int, parts[1].split(','))
                    orders.append({
                        'origin': (origin_e, origin_n),
                        'delivery': (delivery_e, delivery_n)
                    })
        return orders

    def load_paths(self, paths_file):
        """Load computed paths."""
        paths = []
        with open(paths_file, 'r') as f:
            for line in f:
                parts = line.strip().split(' // ')
                if len(parts) == 2:
                    outbound_str, inbound_str = parts

                    # Parse outbound path
                    outbound = []
                    if outbound_str.strip():
                        for point in outbound_str.strip().split():
                            e, n = map(int, point.split(','))
                            outbound.append((e, n))

                    # Parse inbound path
                    inbound = []
                    if inbound_str.strip():
                        for point in inbound_str.strip().split():
                            e, n = map(int, point.split(','))
                            inbound.append((e, n))

                    # Mark as failed if either path is empty
                    if not outbound or not inbound:
                        paths.append(None)  # Failed path
                    else:
                        paths.append({
                            'outbound': outbound,
                            'inbound': inbound
                        })
        return paths

    def calculate_path_density(self, path):
        """Calculate total density cost for a path."""
        total = 0
        for e, n in path:
            total += self.map_data['density'][e, n]
        return total

    def setup_plot(self):
        """Set up the matplotlib figure and axes."""
        self.fig = plt.figure(figsize=(16, 10))

        # Main plot area (adjusted to make room for controls)
        self.ax = plt.subplot2grid((20, 20), (0, 0), colspan=20, rowspan=17, fig=self.fig)
        self.fig.suptitle('Drone Path Visualizer - Interactive Controls', fontsize=16, fontweight='bold')

        # Plot density map with color intensity
        density = self.map_data['density']
        im = self.ax.imshow(density.T, cmap='YlOrRd', origin='lower',
                           interpolation='nearest', alpha=0.7, vmin=0, vmax=9)

        # Add colorbar
        cbar = plt.colorbar(im, ax=self.ax, label='Population Density')
        cbar.set_ticks(range(10))

        # Add density numbers to cells (sample some to avoid clutter)
        step = max(1, min(self.map_data['rows'], self.map_data['cols']) // 20)
        for i in range(0, self.map_data['rows'], step):
            for j in range(0, self.map_data['cols'], step):
                self.ax.text(i, j, str(density[i, j]),
                           ha='center', va='center',
                           fontsize=6, color='black', alpha=0.5)

        # Mark all docks with small markers
        for dock in self.map_data['docks']:
            self.ax.plot(dock[0], dock[1], 'ks', markersize=8, alpha=0.5)

        # Set up grid
        self.ax.set_xlim(-0.5, self.map_data['rows'] - 0.5)
        self.ax.set_ylim(-0.5, self.map_data['cols'] - 0.5)
        self.ax.set_xlabel('East →', fontsize=12)
        self.ax.set_ylabel('North →', fontsize=12)
        self.ax.grid(True, alpha=0.3, linewidth=0.5)
        self.ax.set_aspect('equal')

        # Create robot (blue circle) - initially invisible
        self.robot = plt.Circle((0, 0), 0.6, color='blue', zorder=10, visible=False)
        self.ax.add_patch(self.robot)

        # Create markers for start, goal, and destination dock
        self.start_marker = plt.Circle((0, 0), 0.8, color='green', fill=False,
                                       linewidth=3, zorder=5, visible=False)
        self.goal_marker = plt.Circle((0, 0), 0.8, color='green', fill=False,
                                      linewidth=3, zorder=5, visible=False)
        self.dock_marker = plt.Circle((0, 0), 0.8, color='purple', fill=False,
                                      linewidth=3, zorder=5, visible=False)
        self.ax.add_patch(self.start_marker)
        self.ax.add_patch(self.goal_marker)
        self.ax.add_patch(self.dock_marker)

        # Path line
        self.path_line, = self.ax.plot([], [], 'b-', linewidth=2, alpha=0.5, zorder=3)

        # Info text box
        self.info_text = self.ax.text(0.02, 0.98, '', transform=self.ax.transAxes,
                                      verticalalignment='top', fontsize=10,
                                      bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

        # Status text
        self.status_text = self.ax.text(0.98, 0.98, '', transform=self.ax.transAxes,
                                        verticalalignment='top', horizontalalignment='right',
                                        fontsize=10, fontweight='bold',
                                        bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))

        # Add interactive controls at the bottom
        self.setup_controls()

    def setup_controls(self):
        """Set up interactive control buttons."""
        # Play/Pause button (centered-left)
        ax_pause = plt.subplot2grid((20, 20), (18, 7), colspan=3, rowspan=1, fig=self.fig)
        self.btn_pause = Button(ax_pause, 'Pause', color='lightcoral', hovercolor='coral')
        self.btn_pause.on_clicked(self.toggle_pause)

        # Replay button (centered-right)
        ax_replay = plt.subplot2grid((20, 20), (18, 11), colspan=3, rowspan=1, fig=self.fig)
        self.btn_replay = Button(ax_replay, 'Replay', color='lightblue', hovercolor='skyblue')
        self.btn_replay.on_clicked(self.replay_order)

    def toggle_pause(self, event):
        """Toggle pause/play state."""
        self.is_paused = not self.is_paused
        if self.is_paused:
            self.btn_pause.label.set_text('Play')
            self.btn_pause.ax.set_facecolor('lightgreen')
        else:
            self.btn_pause.label.set_text('Pause')
            self.btn_pause.ax.set_facecolor('lightcoral')

    def replay_order(self, event):
        """Replay the current order from the beginning."""
        self.current_step = 0
        self.current_leg = 0
        self.is_paused = False
        self.btn_pause.label.set_text('Pause')
        self.btn_pause.ax.set_facecolor('lightcoral')
        # Re-initialize the display
        self.init_animation()

    def init_animation(self):
        """Initialize animation for a new order."""
        self.current_step = 0
        self.current_leg = 0

        order = self.orders[self.current_order]
        path = self.paths[self.current_order]

        # Check if path failed
        if path is None:
            print(f"\n⚠️  Order {self.current_order + 1} FAILED: No valid path within 110-step budget")
            print(f"   Origin: {order['origin']}, Delivery: {order['delivery']}")
            # Skip to next order after a brief pause
            return

        # Set markers
        self.start_marker.center = order['origin']
        self.start_marker.set_visible(True)

        self.goal_marker.center = order['delivery']
        self.goal_marker.set_visible(True)

        # Destination dock is the last point of inbound path
        dest_dock = path['inbound'][-1]
        self.dock_marker.center = dest_dock
        self.dock_marker.set_visible(True)

        # Show robot at start
        self.robot.center = order['origin']
        self.robot.set_visible(True)

        # Clear path line
        self.path_line.set_data([], [])

    def animate(self, frame):
        """Animation update function."""
        # Check if paused
        if self.is_paused:
            return

        order = self.orders[self.current_order]
        path = self.paths[self.current_order]

        # Check if path failed - skip to next order
        if path is None:
            self.animation.event_source.stop()
            plt.close()
            self.current_order += 1
            if self.current_order < len(self.orders):
                self.run()
            return

        # Determine which leg we're on and which path segment
        if self.current_leg == 0:
            current_path = path['outbound']
            leg_name = "Leg 1: Origin → Delivery"
        else:
            current_path = path['inbound']
            leg_name = "Leg 2: Delivery → Dock"

        # Update robot position
        if self.current_step < len(current_path):
            pos = current_path[self.current_step]
            self.robot.center = pos

            # Update path line to show trajectory so far
            path_so_far = current_path[:self.current_step + 1]
            xs, ys = zip(*path_so_far) if path_so_far else ([], [])
            self.path_line.set_data(xs, ys)

            # Update status
            self.status_text.set_text(f'{leg_name}\nStep {self.current_step + 1}/{len(current_path)}')

            self.current_step += 1
        else:
            # Finished current leg
            if self.current_leg == 0:
                # Move to leg 2
                self.current_leg = 1
                self.current_step = 0
                self.path_line.set_data([], [])  # Clear path for new leg
            else:
                # Finished entire order - show summary
                outbound_density = self.calculate_path_density(path['outbound'])
                inbound_density = self.calculate_path_density(path['inbound'])
                total_density = outbound_density + inbound_density
                total_steps = len(path['outbound']) + len(path['inbound']) - 2

                dest_dock = path['inbound'][-1]

                summary = (f"ORDER {self.current_order + 1} COMPLETE!\n"
                          f"─────────────────────────\n"
                          f"Start:  {order['origin']}\n"
                          f"Goal:   {order['delivery']}\n"
                          f"Dock:   {dest_dock}\n"
                          f"─────────────────────────\n"
                          f"Total Steps:   {total_steps}\n"
                          f"Total Density: {total_density:.1f}\n"
                          f"─────────────────────────\n"
                          f"Leg 1: {len(path['outbound']) - 1} steps, "
                          f"density={outbound_density:.1f}\n"
                          f"Leg 2: {len(path['inbound']) - 1} steps, "
                          f"density={inbound_density:.1f}")

                self.status_text.set_text(summary)

        # Update info box
        info = (f"Order {self.current_order + 1} of {len(self.orders)}\n"
               f"Origin: {order['origin']}\n"
               f"Delivery: {order['delivery']}")
        self.info_text.set_text(info)

    def run(self):
        """Run the visualization for all orders."""
        for i in range(len(self.orders)):
            self.current_order = i
            self.setup_plot()
            self.init_animation()

            # Calculate total frames needed
            path = self.paths[i]
            total_frames = len(path['outbound']) + len(path['inbound']) + 100  # +100 for pause at end

            # Store animation object
            self.animation = FuncAnimation(self.fig, self.animate,
                                          init_func=self.init_animation,
                                          interval=self.animation_speed,
                                          blit=False, repeat=True, cache_frame_data=False)

            plt.tight_layout(rect=[0, 0.12, 1, 0.96])  # Make room for controls
            plt.show()

def main():
    if len(sys.argv) < 4:
        print("Usage: python path_visualizer.py <map_file> <orders_file> <paths_file>")
        print("Example: python path_visualizer.py ../sf.txt ../orders.txt ../paths.txt")
        sys.exit(1)

    map_file = sys.argv[1]
    orders_file = sys.argv[2]
    paths_file = sys.argv[3]

    print("Drone Path Visualizer")
    print("=" * 50)
    print(f"Map:    {map_file}")
    print(f"Orders: {orders_file}")
    print(f"Paths:  {paths_file}")
    print("=" * 50)
    print("Close the window to proceed to the next order...")
    print()

    visualizer = PathVisualizer(map_file, orders_file, paths_file)
    visualizer.run()

if __name__ == "__main__":
    main()
