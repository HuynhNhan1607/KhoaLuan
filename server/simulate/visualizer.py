"""
Formation Control Visualizer
=============================
Real-time animation with payload polygon and ETC visualization
Centroid is computed from actual robot positions
Includes speed slider for animation control
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle, Polygon
from matplotlib.widgets import Slider
from config import (
    CORNERS, PATH_LENGTH, PATH_HEIGHT, NUM_ROBOTS,
    FORMATION_STAMP_INTERVAL, ROBOT_MARKER_SIZE,
    ROBOT_COLORS, ROBOT_LABELS, FLASH_RADIUS, FLASH_DURATION,
    SENSOR_UPDATE_PERIOD, VELOCITY_MAX, TRIGGER_THRESHOLD,
    HEADING_BIAS_STD, MEASUREMENT_NOISE_STD, FORMATION_SPACING,
    SPEED_FACTOR
)


class FormationVisualizer:
    """
    Real-time animation for Virtual Structure Formation Control.
    
    Features:
    - Cyan payload polygon connecting all robots
    - Centroid marked with 'X' (computed from robot positions)
    - Yellow flashes for ETC broadcasts
    - Ghost formation lines
    - Statistics panel
    - Speed slider for animation control
    """
    
    def __init__(self, robots, controller):
        self.robots = robots
        self.controller = controller
        
        # Ghost lines for formation history
        self.ghost_lines = []
        self.ghost_stamp_timer = 0.0
        
        # Speed control
        self.speed_factor = SPEED_FACTOR
        
        # Setup plot
        self.setup_plot()
    
    def setup_plot(self):
        """Setup matplotlib figure with speed slider."""
        # Create figure with extra space at bottom for slider
        self.fig = plt.figure(figsize=(13, 11))
        
        # Main axes for simulation
        self.ax = self.fig.add_axes([0.08, 0.15, 0.87, 0.78])
        
        self.ax.set_facecolor('white')
        self.ax.grid(True, linestyle='-', linewidth=0.5, alpha=0.7, color='gray')
        self.ax.set_axisbelow(True)
        
        # Reference path
        path = np.vstack([CORNERS, CORNERS[0]])
        self.ax.plot(path[:, 0], path[:, 1], 
                    'k--', linewidth=2, alpha=0.5, label='Reference Path')
        
        # Corner markers
        for i, corner in enumerate(CORNERS):
            self.ax.plot(corner[0], corner[1], 'ks', markersize=12,
                        markerfacecolor='yellow', markeredgecolor='black',
                        markeredgewidth=2)
            self.ax.annotate(f'C{i+1}', corner, textcoords="offset points",
                            xytext=(12, 12), fontsize=11, fontweight='bold')
        
        # Trajectory lines for robots
        self.trail_lines = []
        for i in range(NUM_ROBOTS):
            line, = self.ax.plot([], [], color=ROBOT_COLORS[i],
                                linewidth=2, alpha=0.8, label=ROBOT_LABELS[i])
            self.trail_lines.append(line)
        
        # Centroid trajectory (computed)
        self.centroid_trail, = self.ax.plot([], [], 'k-', linewidth=1.5, 
                                             alpha=0.4, label='Centroid Path')
        
        # Robot markers
        initial_positions = np.array([robot.position for robot in self.robots])
        self.robot_markers = self.ax.scatter(
            initial_positions[:, 0], initial_positions[:, 1],
            s=ROBOT_MARKER_SIZE, c=ROBOT_COLORS,
            edgecolors='black', linewidths=2, zorder=10
        )
        
        # Centroid marker (X) - computed from robot positions
        initial_centroid = self.controller.get_current_centroid()
        self.centroid_marker, = self.ax.plot(
            initial_centroid[0], initial_centroid[1],
            'kX', markersize=15, markeredgewidth=3, zorder=11, label='Centroid (computed)'
        )
        
        # === PAYLOAD POLYGON ===
        self.payload_polygon = Polygon(
            initial_positions, closed=True,
            fill=True, facecolor='cyan', alpha=0.3,
            edgecolor='darkcyan', linewidth=2.5, zorder=8
        )
        self.ax.add_patch(self.payload_polygon)
        
        # Flash circles
        self.flash_circles = []
        for i in range(NUM_ROBOTS):
            circle = Circle((0, 0), FLASH_RADIUS,
                           fill=False, edgecolor='gold',
                           linewidth=4, alpha=0, zorder=9)
            self.ax.add_patch(circle)
            self.flash_circles.append(circle)
        
        # Ghost lines container
        self.ghost_line_objects = []
        
        # Information displays
        self.time_text = self.ax.text(
            0.02, 0.98, '', transform=self.ax.transAxes,
            fontsize=10, fontweight='bold', verticalalignment='top',
            family='monospace',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.9)
        )
        
        self.etc_text = self.ax.text(
            0.02, 0.78, '', transform=self.ax.transAxes,
            fontsize=10, verticalalignment='top', family='monospace',
            bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.9)
        )
        
        self.formation_text = self.ax.text(
            0.02, 0.55, '', transform=self.ax.transAxes,
            fontsize=9, verticalalignment='top', family='monospace',
            bbox=dict(boxstyle='round', facecolor='lightcyan', alpha=0.9)
        )
        
        # Labels and title
        self.ax.set_xlabel('X Position (m)', fontsize=12, fontweight='bold')
        self.ax.set_ylabel('Y Position (m)', fontsize=12, fontweight='bold')
        self.ax.set_title(
            'Virtual Structure (Centroid-Based) with ETC\n'
            f'Trigger: {TRIGGER_THRESHOLD*100:.0f}cm | '
            f'Heading Bias: ±{np.degrees(HEADING_BIAS_STD):.1f}° | '
            f'Noise: {MEASUREMENT_NOISE_STD*100:.0f}cm',
            fontsize=12, fontweight='bold'
        )
        
        # Legend
        self.ax.legend(loc='upper right', fontsize=10, framealpha=0.9,
                      edgecolor='black', fancybox=False)
        
        # Axis limits
        padding = 1.0
        self.ax.set_xlim(-padding, PATH_LENGTH + padding)
        self.ax.set_ylim(-padding, PATH_HEIGHT * 2 + padding)
        self.ax.set_aspect('equal')
        
        # === SPEED SLIDER ===
        slider_ax = self.fig.add_axes([0.2, 0.03, 0.6, 0.03])
        self.speed_slider = Slider(
            ax=slider_ax,
            label='Speed',
            valmin=1,
            valmax=20,
            valinit=self.speed_factor,
            valstep=1,
            color='steelblue'
        )
        self.speed_slider.on_changed(self._on_speed_change)
        
        # Speed label
        self.speed_label = self.fig.text(
            0.85, 0.035, f'{self.speed_factor}x', 
            fontsize=12, fontweight='bold',
            verticalalignment='center'
        )
    
    def _on_speed_change(self, val):
        """Callback when speed slider is changed."""
        self.speed_factor = int(val)
        self.speed_label.set_text(f'{self.speed_factor}x')
    
    def get_speed_factor(self):
        """Get current speed factor from slider."""
        return self.speed_factor
    
    def stamp_ghost_line(self):
        """Stamp current formation as ghost line."""
        positions = [robot.position.copy() for robot in self.robots]
        self.ghost_lines.append(positions)
    
    def update_frame(self, simulation_time, current_step, total_steps):
        """Update animation frame."""
        # Update ghost stamp timer
        self.ghost_stamp_timer += 0.05
        if self.ghost_stamp_timer >= FORMATION_STAMP_INTERVAL:
            self.ghost_stamp_timer = 0.0
            self.stamp_ghost_line()
        
        # Update robot trails
        for i, robot in enumerate(self.robots):
            trajectory = np.array(robot.trajectory)
            self.trail_lines[i].set_data(trajectory[:, 0], trajectory[:, 1])
        
        # Update centroid trail (from controller history)
        if len(self.controller.centroid_trajectory) > 0:
            centroid_traj = np.array(self.controller.centroid_trajectory)
            self.centroid_trail.set_data(centroid_traj[:, 0], centroid_traj[:, 1])
        
        # Update robot markers
        current_positions = np.array([robot.position for robot in self.robots])
        self.robot_markers.set_offsets(current_positions)
        
        # Update centroid marker (computed from robot positions)
        current_centroid = self.controller.get_current_centroid()
        self.centroid_marker.set_data([current_centroid[0]], [current_centroid[1]])
        
        # Update payload polygon
        self.payload_polygon.set_xy(current_positions)
        
        # Update flash circles
        for i, robot in enumerate(self.robots):
            if robot.is_flashing:
                self.flash_circles[i].center = robot.position
                alpha = robot.flash_timer / FLASH_DURATION * 0.8
                self.flash_circles[i].set_alpha(alpha)
            else:
                self.flash_circles[i].set_alpha(0)
        
        # Draw ghost lines
        while len(self.ghost_line_objects) < len(self.ghost_lines):
            idx = len(self.ghost_line_objects)
            positions = np.array(self.ghost_lines[idx])
            
            ghost_poly = Polygon(
                positions, closed=True,
                fill=False, edgecolor='gray', alpha=0.3,
                linewidth=1, linestyle='-', zorder=5
            )
            self.ax.add_patch(ghost_poly)
            
            for pos in positions:
                self.ax.plot(pos[0], pos[1], 'ko', markersize=2, alpha=0.3, zorder=5)
            
            self.ghost_line_objects.append(ghost_poly)
        
        # Calculate statistics
        total_packets = sum(robot.total_broadcasts for robot in self.robots)
        expected_tt_packets = NUM_ROBOTS * (simulation_time / SENSOR_UPDATE_PERIOD)
        savings = (1 - total_packets / expected_tt_packets) * 100 if expected_tt_packets > 0 else 0
        
        # Formation errors
        formation_errors = self.controller.get_formation_error() * 100
        
        # Update text displays
        progress = current_step / total_steps * 100
        corner_idx = self.controller.current_corner_idx + 1
        
        self.time_text.set_text(
            f'Time: {simulation_time:.1f}s\n'
            f'Progress: {progress:.0f}%\n'
            f'Corner: {corner_idx}/{len(CORNERS)-1}\n'
            f'Speed: {self.speed_factor}x'
        )
        
        self.etc_text.set_text(
            f'== ETC Statistics ==\n'
            f'Total Packets: {total_packets}\n'
            f'Expected (TT): {int(expected_tt_packets)}\n'
            f'Bandwidth Saved: {savings:.1f}%\n'
            f'───────────────────\n'
            f'R1: {self.robots[0].total_broadcasts} pkts\n'
            f'R2: {self.robots[1].total_broadcasts} pkts\n'
            f'R3: {self.robots[2].total_broadcasts} pkts'
        )
        
        self.formation_text.set_text(
            f'== Formation Errors ==\n'
            f'R1: {formation_errors[0]:.1f}cm\n'
            f'R2: {formation_errors[1]:.1f}cm\n'
            f'R3: {formation_errors[2]:.1f}cm\n'
            f'Mean: {np.mean(formation_errors):.1f}cm'
        )
    
    def get_figure(self):
        """Return figure for animation."""
        return self.fig

