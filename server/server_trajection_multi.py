from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import tkinter as tk
from tkinter import ttk
import numpy as np
import time
import threading


from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

class TrajectoryVisualizer:
    # Robot IDs to display - change this list to show/hide robots
    ROBOT_IDS = [1, 2, 3]
    
    def __init__(self):
        # Multi-robot trajectory storage - keyed by robot_id
        self.trajectory_points = {}  # {robot_id: [(x, y), ...]}
        self.x = {}                  # {robot_id: x_position}
        self.y = {}                  # {robot_id: y_position}
        self.theta = {}              # {robot_id: theta_angle}
        
        # Ground truth (target) paths for trajectory supervision
        self.ground_truth_paths = {}  # {robot_id: [(x, y), ...]}
        
        # Robot colors
        self.robot_colors = {
            1: 'red',
            2: 'green',
            3: 'blue'
        }
        
        # Initialize state for robots
        for robot_id in self.ROBOT_IDS:
            self.trajectory_points[robot_id] = []
            self.x[robot_id] = 0.0
            self.y[robot_id] = 0.0
            self.theta[robot_id] = 0.0
        
        self.parent_frame = None
        self.canvas = None
        self.fig = None
        self.ax = None
        self.is_active = False
        self.lock = threading.Lock()
        
        # Throttle update parameters
        self.update_interval = 200  # ms
        self.points_threshold = 5
        self.pending_points = {rid: [] for rid in self.ROBOT_IDS}
        self.last_update_time = time.time() * 1000
        self.update_scheduled = False
        
        # Plot limits and grid
        self.PLOT_X_MIN = -1
        self.PLOT_X_MAX = 7.0
        self.PLOT_Y_MIN = -1
        self.PLOT_Y_MAX = 7.0
        self.GRID_SIZE = 0.6  # m
        
        # Robot parameters
        self.robot_radius = 0.1543  # m
        
        # Status panel variables (now per robot)
        self.ekf_labels = {}        # {robot_id: {"x": label, "y": label}}
        self.bno055_labels = {}     # {robot_id: {"x": label, "y": label, ...}}
        self.odometry_labels = {}   # {robot_id: {"x": label, "y": label, ...}}
        self.localization_labels = {}  # {robot_id: {"x": label, "y": label}}
        
        # ========== APPROACH PHASE VISUALIZATION ==========
        self.object_position = None   # (x, y) object center
        self.object_length = 0.2      # Object length in meters (X-axis direction)
        self.object_width = 0.2       # Object width in meters (Y-axis direction)
        self.grip_positions = {}      # {robot_id: (x, y)}
        self.obstacles = []           # List of obstacle dicts
        self.show_obstacles = True    # Toggle obstacle display
        
        # ========== PHASE 2: TRANSPORT VISUALIZATION ==========
        self.destination_position = None   # (x, y) destination center
        self.centroid_path = []            # [(x, y), ...] centroid trajectory
        self.formation_circle = None       # (x, y, radius) - circle showing unified body size
        self.show_corridor = False         # Toggle to show corridor/tube around path (default OFF)
        
    def initialize_plot(self, parent_frame):
        """Initialize the trajectory visualization embedded in a parent frame"""
        self.parent_frame = parent_frame
        
        # Create main container
        main_container = ttk.PanedWindow(self.parent_frame, orient=tk.HORIZONTAL)
        main_container.pack(fill=tk.BOTH, expand=True)
        
        # Create left panel for status frames
        left_panel = ttk.Frame(main_container, width=250)
        main_container.add(left_panel, weight=1)
        
        # Create right panel for plot
        right_panel = ttk.Frame(main_container)
        main_container.add(right_panel, weight=4)
        
        # Configure the left panel with status frames for 3 robots
        left_panel.columnconfigure(0, weight=1)
        
        # Create status frames for each robot
        self._create_status_frames(left_panel)
        
        # Create matplotlib figure and canvas in the right panel
        self.fig = Figure(figsize=(8, 6), dpi=100)
        self.ax = self.fig.add_subplot(111)
        
        # Set up grid
        grid_size = self.GRID_SIZE
        
        # Set fixed ticks at grid intervals
        x_ticks = np.arange(self.PLOT_X_MIN, self.PLOT_X_MAX + grid_size/2, grid_size)
        y_ticks = np.arange(self.PLOT_Y_MIN, self.PLOT_Y_MAX + grid_size/2, grid_size)
        self.ax.set_xticks(x_ticks)
        self.ax.set_yticks(y_ticks)
        
        # Grid styling
        self.ax.grid(True, linewidth=0.8)
        self.ax.set_facecolor('#f8f8f8')
        
        self.ax.set_aspect('equal')
        self.ax.set_xlabel('X Position (m)', fontsize=12)
        self.ax.set_ylabel('Y Position (m)', fontsize=12)
        self.ax.set_title('Multi-Robot Trajectory (Red=R1, Green=R2, Blue=R3)', fontsize=14)
        
        # Set initial plot limits
        self.ax.set_xlim(self.PLOT_X_MIN, self.PLOT_X_MAX)
        self.ax.set_ylim(self.PLOT_Y_MIN, self.PLOT_Y_MAX)
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=right_panel)
        self.canvas.draw()

        # Add navigation toolbar for zoom/pan functionality
        toolbar_frame = ttk.Frame(right_panel)
        toolbar_frame.pack(side=tk.TOP, fill=tk.X)
        self.toolbar = NavigationToolbar2Tk(self.canvas, toolbar_frame)
        self.toolbar.update()

        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
        
        # Add control buttons
        button_frame = ttk.Frame(right_panel)
        
        ttk.Button(button_frame, text="Reset View", command=self.reset_view).pack(side=tk.RIGHT, padx=5, pady=5)
        ttk.Button(button_frame, text="Clear All", command=self.clear_all_trajectories).pack(side=tk.RIGHT, padx=5, pady=5)
        ttk.Button(button_frame, text="Clear Robot 1", command=lambda: self.clear_trajectory(1)).pack(side=tk.LEFT, padx=5, pady=5)
        ttk.Button(button_frame, text="Clear Robot 2", command=lambda: self.clear_trajectory(2)).pack(side=tk.LEFT, padx=5, pady=5)
        ttk.Button(button_frame, text="Clear Robot 3", command=lambda: self.clear_trajectory(3)).pack(side=tk.LEFT, padx=5, pady=5)
        
        # Draw initial robot bodies
        for robot_id in self.ROBOT_IDS:
            self._draw_robot_body(robot_id, 0, 0)
        
        self.canvas.draw()
        self.is_active = True

    def _create_status_frames(self, parent):
        """Create status frames for all 3 robots"""
        # Create a notebook for the 3 robots in the left panel
        robot_notebook = ttk.Notebook(parent)
        robot_notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        for robot_id in self.ROBOT_IDS:
            # Create a tab for each robot
            robot_tab = ttk.Frame(robot_notebook)
            robot_notebook.add(robot_tab, text=f"Robot {robot_id}")
            
            # Configure rows
            for i in range(4):
                robot_tab.rowconfigure(i, weight=1)
            robot_tab.columnconfigure(0, weight=1)
            
            # Frame 1: EKF
            ekf_frame = ttk.LabelFrame(robot_tab, text="EKF")
            ekf_frame.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
            
            ttk.Label(ekf_frame, text="X Position:").grid(row=0, column=0, padx=5, pady=2, sticky="w")
            ttk.Label(ekf_frame, text="Y Position:").grid(row=1, column=0, padx=5, pady=2, sticky="w")
            
            ekf_x = ttk.Label(ekf_frame, text="0.00 m")
            ekf_x.grid(row=0, column=1, padx=5, pady=2, sticky="e")
            ekf_y = ttk.Label(ekf_frame, text="0.00 m")
            ekf_y.grid(row=1, column=1, padx=5, pady=2, sticky="e")
            
            self.ekf_labels[robot_id] = {"x": ekf_x, "y": ekf_y}
            
            # Frame 2: BNO055 Position
            bno055_frame = ttk.LabelFrame(robot_tab, text="BNO055 Position")
            bno055_frame.grid(row=1, column=0, padx=5, pady=5, sticky="nsew")
            
            ttk.Label(bno055_frame, text="X Position:").grid(row=0, column=0, padx=5, pady=2, sticky="w")
            ttk.Label(bno055_frame, text="Y Position:").grid(row=1, column=0, padx=5, pady=2, sticky="w")
            ttk.Label(bno055_frame, text="X Velocity:").grid(row=2, column=0, padx=5, pady=2, sticky="w")
            ttk.Label(bno055_frame, text="Y Velocity:").grid(row=3, column=0, padx=5, pady=2, sticky="w")
            
            bno055_x = ttk.Label(bno055_frame, text="0.00 m")
            bno055_x.grid(row=0, column=1, padx=5, pady=2, sticky="e")
            bno055_y = ttk.Label(bno055_frame, text="0.00 m")
            bno055_y.grid(row=1, column=1, padx=5, pady=2, sticky="e")
            bno055_vx = ttk.Label(bno055_frame, text="0.00 m/s")
            bno055_vx.grid(row=2, column=1, padx=5, pady=2, sticky="e")
            bno055_vy = ttk.Label(bno055_frame, text="0.00 m/s")
            bno055_vy.grid(row=3, column=1, padx=5, pady=2, sticky="e")
            
            self.bno055_labels[robot_id] = {"x": bno055_x, "y": bno055_y, "vx": bno055_vx, "vy": bno055_vy}
            
            # Frame 3: Odometry
            odometry_frame = ttk.LabelFrame(robot_tab, text="Odometry")
            odometry_frame.grid(row=2, column=0, padx=5, pady=5, sticky="nsew")
            
            ttk.Label(odometry_frame, text="X Position:").grid(row=0, column=0, padx=5, pady=2, sticky="w")
            ttk.Label(odometry_frame, text="Y Position:").grid(row=1, column=0, padx=5, pady=2, sticky="w")
            ttk.Label(odometry_frame, text="X Velocity:").grid(row=2, column=0, padx=5, pady=2, sticky="w")
            ttk.Label(odometry_frame, text="Y Velocity:").grid(row=3, column=0, padx=5, pady=2, sticky="w")
            
            odom_x = ttk.Label(odometry_frame, text="0.00 m")
            odom_x.grid(row=0, column=1, padx=5, pady=2, sticky="e")
            odom_y = ttk.Label(odometry_frame, text="0.00 m")
            odom_y.grid(row=1, column=1, padx=5, pady=2, sticky="e")
            odom_vx = ttk.Label(odometry_frame, text="0.00 m/s")
            odom_vx.grid(row=2, column=1, padx=5, pady=2, sticky="e")
            odom_vy = ttk.Label(odometry_frame, text="0.00 m/s")
            odom_vy.grid(row=3, column=1, padx=5, pady=2, sticky="e")
            
            self.odometry_labels[robot_id] = {"x": odom_x, "y": odom_y, "vx": odom_vx, "vy": odom_vy}
            
            # Frame 4: Localization
            localization_frame = ttk.LabelFrame(robot_tab, text="Localization")
            localization_frame.grid(row=3, column=0, padx=5, pady=5, sticky="nsew")
            
            ttk.Label(localization_frame, text="X Position:").grid(row=0, column=0, padx=5, pady=2, sticky="w")
            ttk.Label(localization_frame, text="Y Position:").grid(row=1, column=0, padx=5, pady=2, sticky="w")
            
            loc_x = ttk.Label(localization_frame, text="0.00 m")
            loc_x.grid(row=0, column=1, padx=5, pady=2, sticky="e")
            loc_y = ttk.Label(localization_frame, text="0.00 m")
            loc_y.grid(row=1, column=1, padx=5, pady=2, sticky="e")
            
            self.localization_labels[robot_id] = {"x": loc_x, "y": loc_y}
    
    def _draw_robot_body(self, robot_id, x, y):
        """Draw a single robot body at position (x, y) with robot_id color"""
        color = self.robot_colors.get(robot_id, 'gray')
        
        # Draw robot body (circle)
        robot_body = plt.Circle((x, y), self.robot_radius, fill=False, color=color, linewidth=2)
        self.ax.add_patch(robot_body)
        
        # Draw 4 Mecanum wheels
        wheel_offset = self.robot_radius * 0.7
        wheel_positions = [
            (wheel_offset, wheel_offset),
            (-wheel_offset, wheel_offset),
            (-wheel_offset, -wheel_offset),
            (wheel_offset, -wheel_offset)
        ]
        
        for rel_x, rel_y in wheel_positions:
            wheel_x = x + rel_x
            wheel_y = y + rel_y
            wheel = plt.Circle((wheel_x, wheel_y), 0.02, fill=True, color=color, alpha=0.6)
            self.ax.add_patch(wheel)
        
        # Draw robot ID text
        self.ax.text(x, y, f"R{robot_id}", ha='center', va='center', 
                    fontsize=10, color=color, weight='bold')
    
    def queue_ui_update(self, update_func, *args, **kwargs):
        """Queue a UI update to be executed on the main thread"""
        if self.parent_frame and self.parent_frame.winfo_exists():
            self.parent_frame.after(0, lambda: update_func(*args, **kwargs))
    
    # Update functions for multi-source position data
    def update_ekf(self, robot_id, x, y):
        """Update EKF position for a specific robot"""
        def _update():
            if robot_id in self.ekf_labels:
                self.ekf_labels[robot_id]["x"].config(text=f"{x:.2f} m")
                self.ekf_labels[robot_id]["y"].config(text=f"{y:.2f} m")
        
        self.queue_ui_update(_update)

    def update_bno055(self, robot_id, x, y, vx, vy):
        """Update BNO055 position for a specific robot"""
        def _update():
            if robot_id in self.bno055_labels:
                self.bno055_labels[robot_id]["x"].config(text=f"{x:.2f} m")
                self.bno055_labels[robot_id]["y"].config(text=f"{y:.2f} m")
                self.bno055_labels[robot_id]["vx"].config(text=f"{vx:.2f} m/s")
                self.bno055_labels[robot_id]["vy"].config(text=f"{vy:.2f} m/s")
        
        self.queue_ui_update(_update)

    def update_odometry(self, robot_id, x, y, vx, vy):
        """Update odometry position for a specific robot"""
        def _update():
            if robot_id in self.odometry_labels:
                self.odometry_labels[robot_id]["x"].config(text=f"{x:.2f} m")
                self.odometry_labels[robot_id]["y"].config(text=f"{y:.2f} m")
                self.odometry_labels[robot_id]["vx"].config(text=f"{vx:.2f} m/s")
                self.odometry_labels[robot_id]["vy"].config(text=f"{vy:.2f} m/s")
        
        self.queue_ui_update(_update)

    def update_localization(self, robot_id, x, y):
        """Update localization position for a specific robot"""
        def _update():
            if robot_id in self.localization_labels:
                self.localization_labels[robot_id]["x"].config(text=f"{x:.2f} m")
                self.localization_labels[robot_id]["y"].config(text=f"{y:.2f} m")
        
        self.queue_ui_update(_update)
    
    def update_position(self, robot_id, x, y, theta):
        """
        Update position for a specific robot from position data.
        This is called from server_multi.py when receiving position updates.
        
        Args:
            robot_id (int): Robot identifier (1, 2, or 3)
            x, y: Position in meters
            theta: Heading angle in radians
        """
        def _update():
            with self.lock:
                # Update stored position
                self.x[robot_id] = x
                self.y[robot_id] = y
                self.theta[robot_id] = theta
                
                # Add point to trajectory
                self.trajectory_points[robot_id].append((x, y))
                
                # Limit trajectory length to prevent memory issues
                # max_points = 1000
                # if len(self.trajectory_points[robot_id]) > max_points:
                #     self.trajectory_points[robot_id] = self.trajectory_points[robot_id][-max_points:]
            
            # Refresh plot
            self._update_plot()
        
        self.queue_ui_update(_update)
    
    def set_ground_truth_path(self, robot_id, path_points_meters):
        """
        Set the ground truth (target) path for a specific robot.
        
        Args:
            robot_id (int): Robot identifier (1, 2, or 3)
            path_points_meters (list): List of (x, y) tuples in meters
        """
        with self.lock:
            self.ground_truth_paths[robot_id] = path_points_meters
        self._update_plot()
    
    def clear_ground_truth_path(self, robot_id):
        """
        Clear the ground truth (target) path for a specific robot.
        
        Args:
            robot_id (int): Robot identifier (1, 2, or 3)
        """
        with self.lock:
            if robot_id in self.ground_truth_paths:
                del self.ground_truth_paths[robot_id]
        self._update_plot()
    
    # ========== APPROACH PHASE VISUALIZATION METHODS ==========
    
    def set_object_position(self, x, y, length=0.2, width=0.2):
        """
        Set the position of the rectangular object to be grasped.
        
        Args:
            x, y: Object center position in meters
            length: Object length in meters (X-axis direction)
            width: Object width in meters (Y-axis direction)
        """
        with self.lock:
            self.object_position = (float(x), float(y))
            self.object_length = float(length)
            self.object_width = float(width)
        self._update_plot()
    
    def clear_object_position(self):
        """Clear the object position marker."""
        with self.lock:
            self.object_position = None
        self._update_plot()
    
    def set_grip_positions(self, grip_positions):
        """
        Set the grip positions for robots.
        
        Args:
            grip_positions: Dict {robot_id: (x, y)}
        """
        with self.lock:
            self.grip_positions = grip_positions.copy() if grip_positions else {}
        self._update_plot()
    
    def clear_grip_positions(self):
        """Clear all grip position markers."""
        with self.lock:
            self.grip_positions.clear()
        self._update_plot()
    
    def set_obstacles(self, obstacles):
        """
        Set the obstacles for visualization.
        
        Args:
            obstacles: List of obstacle dicts from VectorFieldPlanner
                      Each dict has 'type' ('circle' or 'rectangle') and params
        """
        with self.lock:
            self.obstacles = obstacles.copy() if obstacles else []
        self._update_plot()
    
    def clear_obstacles(self):
        """Clear all obstacle markers."""
        with self.lock:
            self.obstacles.clear()
        self._update_plot()
    
    def toggle_obstacles(self, show):
        """Toggle obstacle display on/off."""
        self.show_obstacles = show
        self._update_plot()
    
    # ========== PHASE 2: TRANSPORT VISUALIZATION METHODS ==========
    
    def set_destination_position(self, x, y):
        """
        Set the destination position for Phase 2 transport.
        
        Args:
            x, y: Destination center position in meters
        """
        with self.lock:
            self.destination_position = (float(x), float(y))
        self._update_plot()
    
    def clear_destination_position(self):
        """Clear the destination position marker."""
        with self.lock:
            self.destination_position = None
        self._update_plot()
    
    def set_centroid_path(self, path_points):
        """
        Set the centroid trajectory path for Phase 2 transport visualization.
        
        Args:
            path_points: List of (x, y) tuples representing centroid path
        """
        with self.lock:
            self.centroid_path = list(path_points) if path_points else []
        self._update_plot()
    
    def clear_centroid_path(self):
        """Clear the centroid trajectory path."""
        with self.lock:
            self.centroid_path = []
        self._update_plot()
    
    def set_formation_circle(self, x, y, radius):
        """
        Set the formation circle to show unified body size at Phase 2 start.
        
        Args:
            x, y: Center position (usually object position)
            radius: Effective radius (grip_radius + robot_radius)
        """
        print(f"[TrajectoryViz] Setting formation circle at ({x:.2f}, {y:.2f}), radius={radius:.2f}")
        with self.lock:
            self.formation_circle = (float(x), float(y), float(radius))
        self._update_plot()
        # Also update popup window if open
        if hasattr(self, 'popup_ax') and self.popup_ax:
            self._update_popup()
    
    def clear_formation_circle(self):
        """Clear the formation circle."""
        with self.lock:
            self.formation_circle = None
        self._update_plot()
        # Also update popup window if open
        if hasattr(self, 'popup_ax') and self.popup_ax:
            self._update_popup()
    
    def _update_plot(self):
        """Update the main plot with current data"""
        if not self.is_active or not self.canvas:
            return
            
        with self.lock:
            self.ax.clear()
            
            # Reset grid and labels
            grid_size = self.GRID_SIZE
            x_ticks = np.arange(self.PLOT_X_MIN, self.PLOT_X_MAX + grid_size/2, grid_size)
            y_ticks = np.arange(self.PLOT_Y_MIN, self.PLOT_Y_MAX + grid_size/2, grid_size)
            self.ax.set_xticks(x_ticks)
            self.ax.set_yticks(y_ticks)
            self.ax.grid(True, linewidth=0.8)
            self.ax.set_facecolor('#f8f8f8')
            self.ax.set_aspect('equal')
            self.ax.set_xlabel('X Position (m)', fontsize=12)
            self.ax.set_ylabel('Y Position (m)', fontsize=12)
            self.ax.set_title('Multi-Robot Trajectory (Red=R1, Green=R2, Blue=R3)', fontsize=14)
            self.ax.set_xlim(self.PLOT_X_MIN, self.PLOT_X_MAX)
            self.ax.set_ylim(self.PLOT_Y_MIN, self.PLOT_Y_MAX)
            
            # ========== DRAW OBSTACLES ==========
            if self.show_obstacles and self.obstacles:
                for obs in self.obstacles:
                    if obs['type'] == 'circle':
                        circle = plt.Circle(
                            (obs['cx'], obs['cy']), obs['radius'],
                            fill=True, color='gray', alpha=0.4, edgecolor='black', linewidth=1
                        )
                        self.ax.add_patch(circle)
                    elif obs['type'] == 'rectangle':
                        width = obs['x2'] - obs['x1']
                        height = obs['y2'] - obs['y1']
                        rect = patches.Rectangle(
                            (obs['x1'], obs['y1']), width, height,
                            fill=True, color='gray', alpha=0.4, edgecolor='black', linewidth=1
                        )
                        self.ax.add_patch(rect)
            
            # ========== DRAW OBJECT ==========
            if self.object_position is not None:
                obj_x, obj_y = self.object_position
                obj_length = self.object_length
                obj_width = self.object_width
                # Draw rectangular object centered at (obj_x, obj_y)
                rect_x = obj_x - obj_length / 2
                rect_y = obj_y - obj_width / 2
                object_rect = plt.Rectangle(
                    (rect_x, rect_y), obj_length, obj_width,
                    fill=True, color='orange', alpha=0.7, edgecolor='darkorange', linewidth=2
                )
                self.ax.add_patch(object_rect)
                self.ax.plot(obj_x, obj_y, 'k+', markersize=10, markeredgewidth=2)
                self.ax.text(obj_x, obj_y + obj_width/2 + 0.12, f"Object\n{obj_length:.2f}x{obj_width:.2f}m",
                           ha='center', va='bottom', fontsize=9, color='darkorange', weight='bold')
            
            # ========== DRAW GRIP POSITIONS ==========
            for robot_id, (gx, gy) in self.grip_positions.items():
                color = self.robot_colors.get(robot_id, 'purple')
                
                # Draw grip target as star marker
                self.ax.plot(gx, gy, marker='*', markersize=15, color=color,
                           markeredgecolor='black', markeredgewidth=0.5)
                
                # Draw dashed line from object to grip position
                if self.object_position is not None:
                    obj_x, obj_y = self.object_position
                    self.ax.plot([obj_x, gx], [obj_y, gy], '--', color=color, 
                               linewidth=1, alpha=0.5)
            
            # ========== DRAW DESTINATION (PHASE 2) ==========
            if self.destination_position is not None:
                dest_x, dest_y = self.destination_position
                
                # Draw destination as diamond marker
                self.ax.plot(dest_x, dest_y, marker='D', markersize=15, color='magenta',
                           markeredgecolor='black', markeredgewidth=1.5, alpha=0.8)
                
                # Label
                self.ax.text(dest_x, dest_y + 0.2, "Destination",
                           ha='center', va='bottom', fontsize=9, color='magenta', weight='bold')
                
                # Draw dashed line from object to destination if both exist
                if self.object_position is not None:
                    obj_x, obj_y = self.object_position
                    self.ax.plot([obj_x, dest_x], [obj_y, dest_y], ':', color='magenta', 
                               linewidth=2, alpha=0.5)
            
            # ========== DRAW CENTROID PATH (PHASE 2) ==========
            if self.centroid_path and len(self.centroid_path) > 1:
                centroid_path_arr = np.array(self.centroid_path)
                self.ax.plot(centroid_path_arr[:, 0], centroid_path_arr[:, 1], 
                           color='magenta', linestyle='-', linewidth=2.5, alpha=0.8,
                           label="Centroid Path")
                # Mark waypoints
                self.ax.scatter(centroid_path_arr[:, 0], centroid_path_arr[:, 1], 
                              color='magenta', s=25, alpha=0.6, marker='s')
            
            # ========== DRAW FORMATION CIRCLE (PHASE 2 - Unified Body Size) ==========
            if self.formation_circle is not None:
                fc_x, fc_y, fc_radius = self.formation_circle
                print(f"[TrajectoryViz] Drawing formation circle at ({fc_x:.2f}, {fc_y:.2f}), r={fc_radius:.2f}")
                formation_ring = plt.Circle(
                    (fc_x, fc_y), fc_radius,
                    fill=False, color='cyan', linewidth=2.5, linestyle='--', alpha=0.8
                )
                self.ax.add_patch(formation_ring)
                # Label
                self.ax.text(fc_x + fc_radius + 0.1, fc_y, 
                           f"r={fc_radius:.2f}m",
                           ha='left', va='center', fontsize=9, color='cyan', weight='bold')
            
            # Draw ground truth (target) paths for all robots
            for robot_id in self.ROBOT_IDS:
                if robot_id in self.ground_truth_paths and self.ground_truth_paths[robot_id]:
                    color = self.robot_colors[robot_id]
                    gt_path = np.array(self.ground_truth_paths[robot_id])
                    self.ax.plot(gt_path[:, 0], gt_path[:, 1], color=color, 
                               linestyle='--', linewidth=2, alpha=0.7, 
                               label=f"R{robot_id} Target")
                    # Mark waypoints with circles
                    self.ax.scatter(gt_path[:, 0], gt_path[:, 1], color=color, 
                                  s=30, alpha=0.5, marker='o')
            
            # Draw actual trajectories for all robots
            for robot_id in self.ROBOT_IDS:
                if self.trajectory_points[robot_id]:
                    color = self.robot_colors[robot_id]
                    traj = np.array(self.trajectory_points[robot_id])
                    self.ax.plot(traj[:, 0], traj[:, 1], color=color, 
                               linestyle='-', linewidth=1, alpha=0.6, 
                               label=f"R{robot_id} Actual")
            
            # Draw current robot positions
            for robot_id in self.ROBOT_IDS:
                x = self.x[robot_id]
                y = self.y[robot_id]
                theta = self.theta[robot_id]
                color = self.robot_colors[robot_id]
                
                # Draw robot body
                robot_body = plt.Circle((x, y), self.robot_radius, fill=False, color=color, linewidth=2)
                self.ax.add_patch(robot_body)
                
                # Draw direction arrow
                arrow_length = self.robot_radius * 0.8
                # Y-axis heading convention: theta=0 points North (+Y)
                dx = arrow_length * np.sin(theta)
                dy = arrow_length * np.cos(theta)
                self.ax.arrow(x, y, dx, dy, head_width=0.05, head_length=0.07, 
                            fc=color, ec=color, linewidth=2)
                
                # Draw robot ID
                self.ax.text(x, y - self.robot_radius - 0.15, f"R{robot_id}", 
                           ha='center', va='top', fontsize=10, color=color, weight='bold')
                
                # Draw wheels
                wheel_offset = self.robot_radius * 0.7
                wheel_positions = [
                    (wheel_offset, wheel_offset),
                    (-wheel_offset, wheel_offset),
                    (-wheel_offset, -wheel_offset),
                    (wheel_offset, -wheel_offset)
                ]
                
                for rel_x, rel_y in wheel_positions:
                    # Rotate wheel position by theta
                    rotated_x = x + rel_x * np.cos(theta) - rel_y * np.sin(theta)
                    rotated_y = y + rel_x * np.sin(theta) + rel_y * np.cos(theta)
                    wheel = plt.Circle((rotated_x, rotated_y), 0.02, fill=True, color=color, alpha=0.6)
                    self.ax.add_patch(wheel)
            
            # Add legend
            # Add legend
            handles, labels = self.ax.get_legend_handles_labels()
            if handles:
                self.ax.legend(loc='upper right', fontsize=9)
            
            # Redraw canvas
            self.canvas.draw_idle()
            
            # Also update popup if open
            if hasattr(self, 'popup_canvas') and self.popup_canvas:
                try:
                    self.popup_canvas.draw_idle()
                except:
                    pass
    
    def show(self):
        """Show is not needed as the plot is embedded"""
        if not self.is_active:
            print("Trajectory visualizer not initialized. Call initialize_plot() with a parent frame first.")
    
    def reset_view(self):
        """Reset the plot view to default limits"""
        self.ax.set_xlim(self.PLOT_X_MIN, self.PLOT_X_MAX)
        self.ax.set_ylim(self.PLOT_Y_MIN, self.PLOT_Y_MAX)
        self.canvas.draw()        
    def clear_trajectory(self, robot_id):
        """Clear trajectory for a specific robot"""
        with self.lock:
            if robot_id in self.trajectory_points:
                self.trajectory_points[robot_id].clear()
                self.pending_points[robot_id].clear()
        self._update_plot()
    
    def clear_all_trajectories(self):
        """Clear all robot trajectories"""
        with self.lock:
            for robot_id in self.ROBOT_IDS:
                self.trajectory_points[robot_id].clear()
                self.pending_points[robot_id].clear()
        self._update_plot()
    
    def close(self):
        """Cleanup when closing"""
        self.is_active = False
    
    # ========== POPUP WINDOW FOR LARGER MAP VIEW ==========
    
    def open_popup_window(self):
        """
        Open the trajectory map in a separate popup window for better viewing.
        This creates a larger, resizable window with the full map.
        """
        # Check if popup already exists
        if hasattr(self, 'popup_window') and self.popup_window:
            try:
                self.popup_window.lift()  # Bring to front
                self.popup_window.focus_force()
                return
            except tk.TclError:
                pass  # Window was closed, create new one
        
        # Create popup window
        self.popup_window = tk.Toplevel()
        self.popup_window.title("Multi-Robot Trajectory Map")
        self.popup_window.geometry("1200x900")
        self.popup_window.minsize(800, 600)
        
        # Create main frame
        main_frame = ttk.Frame(self.popup_window, padding=5)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Create matplotlib figure for popup (larger)
        self.popup_fig = Figure(figsize=(12, 9), dpi=100)
        self.popup_ax = self.popup_fig.add_subplot(111)
        
        # Setup popup axis
        self._setup_popup_axis()
        
        # Create canvas for popup
        self.popup_canvas = FigureCanvasTkAgg(self.popup_fig, master=main_frame)
        
        # Add toolbar
        toolbar_frame = ttk.Frame(main_frame)
        toolbar_frame.pack(side=tk.TOP, fill=tk.X)
        self.popup_toolbar = NavigationToolbar2Tk(self.popup_canvas, toolbar_frame)
        self.popup_toolbar.update()
        
        # Pack canvas
        self.popup_canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        
        # Control buttons
        button_frame = ttk.Frame(main_frame)
        button_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=5)
        
        # Left side - Clear buttons
        ttk.Button(button_frame, text="Clear R1", 
                  command=lambda: self.clear_trajectory(1)).pack(side=tk.LEFT, padx=3)
        ttk.Button(button_frame, text="Clear R2", 
                  command=lambda: self.clear_trajectory(2)).pack(side=tk.LEFT, padx=3)
        ttk.Button(button_frame, text="Clear R3", 
                  command=lambda: self.clear_trajectory(3)).pack(side=tk.LEFT, padx=3)
        ttk.Button(button_frame, text="Clear All", 
                  command=self.clear_all_trajectories).pack(side=tk.LEFT, padx=5)
        
        # Right side - View controls
        ttk.Button(button_frame, text="Reset View", 
                  command=self._reset_popup_view).pack(side=tk.RIGHT, padx=5)
        ttk.Button(button_frame, text="Refresh", 
                  command=self._update_popup).pack(side=tk.RIGHT, padx=5)
        
        # Corridor toggle button (Phase 2)
        self.corridor_button = ttk.Button(button_frame, text="Corridor: OFF", 
                                         command=self._toggle_corridor)
        self.corridor_button.pack(side=tk.RIGHT, padx=5)
        
        # Add periodic update
        self._schedule_popup_update()
        
        # Handle close event
        self.popup_window.protocol("WM_DELETE_WINDOW", self._on_popup_close)
        
        # Initial draw
        self._update_popup()
    
    def _setup_popup_axis(self):
        """Setup the popup window axis"""
        ax = self.popup_ax
        grid_size = self.GRID_SIZE
        
        x_ticks = np.arange(self.PLOT_X_MIN, self.PLOT_X_MAX + grid_size/2, grid_size)
        y_ticks = np.arange(self.PLOT_Y_MIN, self.PLOT_Y_MAX + grid_size/2, grid_size)
        ax.set_xticks(x_ticks)
        ax.set_yticks(y_ticks)
        ax.grid(True, linewidth=0.8)
        ax.set_facecolor('#f8f8f8')
        ax.set_aspect('equal')
        ax.set_xlabel('X Position (m)', fontsize=12)
        ax.set_ylabel('Y Position (m)', fontsize=12)
        ax.set_title('Multi-Robot Trajectory Map (Red=R1, Green=R2, Blue=R3)', fontsize=14)
        ax.set_xlim(self.PLOT_X_MIN, self.PLOT_X_MAX)
        ax.set_ylim(self.PLOT_Y_MIN, self.PLOT_Y_MAX)
    
    def _toggle_corridor(self):
        """Toggle the corridor/tube visualization for Phase 2 path."""
        self.show_corridor = not self.show_corridor
        
        # Update button text
        if hasattr(self, 'corridor_button'):
            status = "ON" if self.show_corridor else "OFF"
            self.corridor_button.config(text=f"Corridor: {status}")
        
        # Refresh popup
        self._update_popup()
    
    def _update_popup(self):
        """Update the popup window plot"""
        if not hasattr(self, 'popup_ax') or not self.popup_ax:
            return
        
        ax = self.popup_ax
        
        # Save current view limits
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        
        ax.clear()
        self._setup_popup_axis()
        ax.set_xlim(xlim)
        ax.set_ylim(ylim)
        
        # Draw obstacles
        if self.show_obstacles and self.obstacles:
            for obs in self.obstacles:
                if obs['type'] == 'circle':
                    circle = plt.Circle(
                        (obs['cx'], obs['cy']), obs['radius'],
                        fill=True, color='gray', alpha=0.4, edgecolor='black', linewidth=1
                    )
                    ax.add_patch(circle)
                elif obs['type'] == 'rectangle':
                    width = obs['x2'] - obs['x1']
                    height = obs['y2'] - obs['y1']
                    rect = patches.Rectangle(
                        (obs['x1'], obs['y1']), width, height,
                        fill=True, color='gray', alpha=0.4, edgecolor='black', linewidth=1
                    )
                    ax.add_patch(rect)
        
        # Draw object (rectangular)
        if self.object_position is not None:
            obj_x, obj_y = self.object_position
            obj_length = self.object_length
            obj_width = self.object_width
            # Draw rectangular object centered at (obj_x, obj_y)
            rect_x = obj_x - obj_length / 2
            rect_y = obj_y - obj_width / 2
            object_rect = plt.Rectangle(
                (rect_x, rect_y), obj_length, obj_width,
                fill=True, color='orange', alpha=0.7, edgecolor='darkorange', linewidth=2
            )
            ax.add_patch(object_rect)
            ax.plot(obj_x, obj_y, 'k+', markersize=12, markeredgewidth=2)
            ax.text(obj_x, obj_y + obj_width/2 + 0.15, f"Object\n{obj_length:.2f}x{obj_width:.2f}m",
                   ha='center', va='bottom', fontsize=10, color='darkorange', weight='bold')
        
        # Draw grip positions
        for robot_id, (gx, gy) in self.grip_positions.items():
            color = self.robot_colors.get(robot_id, 'purple')
            ax.plot(gx, gy, marker='*', markersize=20, color=color,
                   markeredgecolor='black', markeredgewidth=0.5)
            if self.object_position is not None:
                obj_x, obj_y = self.object_position
                ax.plot([obj_x, gx], [obj_y, gy], '--', color=color, linewidth=1, alpha=0.5)
        
        # ========== DRAW DESTINATION (PHASE 2) ==========
        if self.destination_position is not None:
            dest_x, dest_y = self.destination_position
            ax.plot(dest_x, dest_y, marker='D', markersize=18, color='magenta',
                   markeredgecolor='black', markeredgewidth=1.5, alpha=0.8)
            ax.text(dest_x, dest_y + 0.25, "Destination",
                   ha='center', va='bottom', fontsize=10, color='magenta', weight='bold')
            if self.object_position is not None:
                obj_x, obj_y = self.object_position
                ax.plot([obj_x, dest_x], [obj_y, dest_y], ':', color='magenta', 
                       linewidth=2, alpha=0.5)
        
        # ========== DRAW CENTROID PATH (PHASE 2) ==========
        if self.centroid_path and len(self.centroid_path) > 1:
            centroid_path_arr = np.array(self.centroid_path)
            ax.plot(centroid_path_arr[:, 0], centroid_path_arr[:, 1], 
                   color='magenta', linestyle='-', linewidth=3, alpha=0.8,
                   label="Centroid Path")
            ax.scatter(centroid_path_arr[:, 0], centroid_path_arr[:, 1], 
                      color='magenta', s=30, alpha=0.6, marker='s')
        
        # ========== DRAW FORMATION CIRCLE (PHASE 2 - Unified Body Size) ==========
        if self.formation_circle is not None:
            fc_x, fc_y, fc_radius = self.formation_circle
            formation_ring = plt.Circle(
                (fc_x, fc_y), fc_radius,
                fill=False, color='cyan', linewidth=3, linestyle='--', alpha=0.9
            )
            ax.add_patch(formation_ring)
            # Label
            ax.text(fc_x + fc_radius + 0.15, fc_y, 
                   f"r={fc_radius:.2f}m",
                   ha='left', va='center', fontsize=10, color='cyan', weight='bold')
            
            # ========== DRAW CORRIDOR (Tube around path) ==========
            if self.show_corridor and self.centroid_path and len(self.centroid_path) > 1:
                # Draw circles along the path to create corridor effect
                path_arr = np.array(self.centroid_path)
                
                # Sample fewer points for cleaner visualization
                step = max(1, len(path_arr) // 20)  # ~20 circles along path
                
                for i in range(0, len(path_arr), step):
                    circle = plt.Circle(
                        (path_arr[i, 0], path_arr[i, 1]), fc_radius,
                        fill=False, color='cyan', linewidth=1, linestyle=':', alpha=0.4
                    )
                    ax.add_patch(circle)
                
                # Always draw at end point
                if len(path_arr) > 0:
                    end_circle = plt.Circle(
                        (path_arr[-1, 0], path_arr[-1, 1]), fc_radius,
                        fill=False, color='cyan', linewidth=2, linestyle='--', alpha=0.7
                    )
                    ax.add_patch(end_circle)
        
        # Draw ground truth paths
        for robot_id in self.ROBOT_IDS:
            if robot_id in self.ground_truth_paths and self.ground_truth_paths[robot_id]:
                color = self.robot_colors[robot_id]
                gt_path = np.array(self.ground_truth_paths[robot_id])
                ax.plot(gt_path[:, 0], gt_path[:, 1], color=color, 
                       linestyle='--', linewidth=2, alpha=0.7, label=f"R{robot_id} Target")
                ax.scatter(gt_path[:, 0], gt_path[:, 1], color=color, s=40, alpha=0.5, marker='o')
        
        # Draw actual trajectories
        for robot_id in self.ROBOT_IDS:
            if self.trajectory_points[robot_id]:
                color = self.robot_colors[robot_id]
                traj = np.array(self.trajectory_points[robot_id])
                ax.plot(traj[:, 0], traj[:, 1], color=color, 
                       linestyle='-', linewidth=1.5, alpha=0.6, label=f"R{robot_id} Actual")
        
        # Draw current robot positions
        for robot_id in self.ROBOT_IDS:
            x = self.x[robot_id]
            y = self.y[robot_id]
            theta = self.theta[robot_id]
            color = self.robot_colors[robot_id]
            
            robot_body = plt.Circle((x, y), self.robot_radius, fill=False, color=color, linewidth=2)
            ax.add_patch(robot_body)
            
            arrow_length = self.robot_radius * 0.8
            # Y-axis heading convention: theta=0 points North (+Y)
            dx = arrow_length * np.sin(theta)
            dy = arrow_length * np.cos(theta)
            ax.arrow(x, y, dx, dy, head_width=0.06, head_length=0.08, fc=color, ec=color, linewidth=2)
            
            ax.text(x, y - self.robot_radius - 0.18, f"R{robot_id}", 
                   ha='center', va='top', fontsize=11, color=color, weight='bold')
        
        handles, labels = ax.get_legend_handles_labels()
        if handles:
            ax.legend(loc='upper right', fontsize=10)
        
        try:
            self.popup_canvas.draw_idle()
        except:
            pass
    
    def _reset_popup_view(self):
        """Reset popup view to default"""
        if hasattr(self, 'popup_ax') and self.popup_ax:
            self.popup_ax.set_xlim(self.PLOT_X_MIN, self.PLOT_X_MAX)
            self.popup_ax.set_ylim(self.PLOT_Y_MIN, self.PLOT_Y_MAX)
            self._update_popup()
    
    def _schedule_popup_update(self):
        """Schedule periodic popup updates"""
        if hasattr(self, 'popup_window') and self.popup_window:
            try:
                self._update_popup()
                self.popup_window.after(500, self._schedule_popup_update)
            except tk.TclError:
                pass  # Window closed
    
    def _on_popup_close(self):
        """Handle popup window close"""
        if hasattr(self, 'popup_window') and self.popup_window:
            self.popup_window.destroy()
            self.popup_window = None
            self.popup_canvas = None
            self.popup_ax = None

# Create a singleton instance
trajectory_visualizer = TrajectoryVisualizer()

