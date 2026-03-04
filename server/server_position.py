import numpy as np
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox
import math
import queue
import os

# Fixed path to map file - change this path if needed
MAP_FILE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "house_map.npy")

ORIGINAL_CELL = 0.1  # 10cm
NEW_CELL = 0.02      # 1cm

class RobotPositionVisualizer:
    def __init__(self):
        # UI components
        self.position_window = None
        self.canvas = None
        self.fig = None
        self.ax = None
        self.is_active = False
        self.x_entry = None  # Entry for X coordinate
        self.y_entry = None  # Entry for Y coordinate
        
        # Thread and update queue
        self.lock = threading.Lock()
        self.update_queue = queue.Queue()
        self.update_interval = 500  # ms
        self.last_plot_update = 0
        
        # Map data
        self.grid = None
        self.grid_width = None
        self.grid_height = None
        self.start_pos = None
        
        # Position data (in grid cells)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.trajectory_points = []
        
        # Destination point
        self.destination = None
        
        # Conversion ratio from grid cells to meters
        self.cell_to_meter = NEW_CELL  # Default: 1 cell = 1cm
        
        # Robot physical parameters
        self.robot_radius_meters = 0.1543  # Robot radius in meters
        
        # Destination callback
        self.send_destination_callback = None
        
    def initialize_plot(self):
        """Initialize window displaying robot position"""
        if self.position_window is not None:
            return
        
        self.position_window = tk.Toplevel()
        self.position_window.title("Robot Position")
        self.position_window.geometry("1000x800")
        self.position_window.protocol("WM_DELETE_WINDOW", self.close)
        
        # Create main panel with left control panel and right map panel
        main_panel = ttk.PanedWindow(self.position_window, orient=tk.HORIZONTAL)
        main_panel.pack(fill=tk.BOTH, expand=True)
        
        # Left control panel
        control_frame = ttk.Frame(main_panel, width=200)
        control_frame.pack_propagate(False)  # Don't shrink
        
        # Add coordinate input section at top of control panel
        input_frame = ttk.LabelFrame(control_frame, text="Nhập tọa độ đích (cm)")
        input_frame.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Label(input_frame, text="X (cm):").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        self.x_entry = ttk.Entry(input_frame, width=8)
        self.x_entry.grid(row=0, column=1, padx=5, pady=5, sticky=tk.W)
        
        ttk.Label(input_frame, text="Y (cm):").grid(row=1, column=0, padx=5, pady=5, sticky=tk.W)
        self.y_entry = ttk.Entry(input_frame, width=8)
        self.y_entry.grid(row=1, column=1, padx=5, pady=5, sticky=tk.W)
        
        ttk.Button(input_frame, text="Đặt vị trí đích", 
                  command=self.on_coord_submit).grid(row=2, column=0, columnspan=2, 
                                                    padx=5, pady=5, sticky=tk.EW)
        
        # Add empty frame to push buttons to top
        filler = ttk.Frame(control_frame)
        filler.pack(fill=tk.BOTH, expand=True)
        
        # Add control panel to main panel
        main_panel.add(control_frame, weight=1)
        
        # Right map panel
        map_frame = ttk.Frame(main_panel)
        
        # Create plot and map
        self.fig = Figure(figsize=(8, 8))
        self.ax = self.fig.add_subplot(111)
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=map_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        
        # Matplotlib toolbar
        self.toolbar = NavigationToolbar2Tk(self.canvas, map_frame)
        self.toolbar.update()
        
        # Add map panel to main panel
        main_panel.add(map_frame, weight=5)
        
        self.is_active = True
        
        # Load map
        self.load_map()
        
        # Start update queue check
        self.check_update_queue()
    
    def load_map(self):
        """Load map from fixed path and scale to desired resolution"""
        try:
            if os.path.exists(MAP_FILE_PATH):
                data = np.load(MAP_FILE_PATH, allow_pickle=True).item()
                original_grid = data["grid"]
                self.start_pos = data.get("start_pos", (0, 0))
                
                # Original grid dimensions
                original_height, original_width = original_grid.shape
                
                # Calculate scaling factor
                scaling_factor = int(ORIGINAL_CELL / NEW_CELL)
                
                # Create the high-resolution grid by repeating each cell
                self.grid = np.repeat(np.repeat(original_grid, scaling_factor, axis=0), 
                                      scaling_factor, axis=1)
                
                # Update grid dimensions
                self.grid_height, self.grid_width = self.grid.shape
                
                # Set cell_to_meter for 1cm per cell (0.01m)
                self.cell_to_meter = NEW_CELL  # 1cm per cell
                
                # Scale the start position
                if self.start_pos is not None:
                    self.start_pos = (self.start_pos[0] * scaling_factor, 
                                     self.start_pos[1] * scaling_factor)
                else:
                    self.start_pos = (0, 0)
                    
                print(f"Original map: {original_width}x{original_height}")
                print(f"Scaled map: {self.grid_width}x{self.grid_height}")
                print(f"Start position: {self.start_pos}")
                print(f"Cell size: {self.cell_to_meter*100}cm")
                
                # Update robot position if not already set
                if len(self.trajectory_points) == 0:
                    self.x = self.start_pos[0]
                    self.y = self.start_pos[1]
                
                self._update_plot()
            else:
                print(f"Map file not found at: {MAP_FILE_PATH}")
                messagebox.showerror("Error", f"Map file not found at:\n{MAP_FILE_PATH}")
        except Exception as e:
            print(f"Error loading map: {e}")
            messagebox.showerror("Error", f"Cannot load map: {str(e)}")
    
    def on_coord_submit(self):
        """Handle manual coordinate input in centimeters"""
        try:
            # Get coordinates from input fields (in centimeters)
            x_cm = float(self.x_entry.get())
            y_cm = float(self.y_entry.get())
            
            # Convert to meters
            x_meters = x_cm / 100.0
            y_meters = y_cm / 100.0
            
            # Convert to grid coordinates
            x_grid = int(x_meters / self.cell_to_meter)
            y_grid = int(y_meters / self.cell_to_meter)
            
            # Check if coordinates are within grid bounds
            if 0 <= x_grid < self.grid_width and 0 <= y_grid < self.grid_height:
                # Check if position is not an obstacle
                if self.grid[y_grid, x_grid] == 0:
                    self.destination = (x_grid, y_grid)
                    print(f"Destination set at: ({x_cm:.1f}cm, {y_cm:.1f}cm) = cell ({x_grid}, {y_grid})")
                    
                    # Update plot
                    self._update_plot()
                    
                    # Send destination to client if callback exists
                    if self.send_destination_callback:
                        # Send in meters
                        self.send_destination_callback(x_meters, y_meters)
                else:
                    messagebox.showwarning("Vị trí không hợp lệ", "Vị trí đã chọn là vật cản!")
            else:
                max_x_cm = self.grid_width * self.cell_to_meter * 100
                max_y_cm = self.grid_height * self.cell_to_meter * 100
                messagebox.showwarning("Tọa độ không hợp lệ", 
                                      f"Tọa độ nằm ngoài phạm vi. Phạm vi hợp lệ: X: 0-{max_x_cm:.1f}cm, Y: 0-{max_y_cm:.1f}cm")
        except ValueError:
            messagebox.showerror("Đầu vào không hợp lệ", "Vui lòng nhập số hợp lệ cho tọa độ X và Y.")
    
    def set_destination_callback(self, callback):
        """Set callback function when destination is selected"""
        self.send_destination_callback = callback
    
    def clear_destination(self):
        """Clear selected destination"""
        self.destination = None
        self._update_plot()
        
        # Clear entry fields
        if self.x_entry and self.y_entry:
            self.x_entry.delete(0, tk.END)
            self.y_entry.delete(0, tk.END)
    
    def clear_trajectory(self):
        """Clear robot trajectory"""
        self.trajectory_points = []
        self._update_plot()
    
    def check_update_queue(self):
        """Check update queue and apply updates"""
        try:
            # Process position updates
            update_needed = False
            while not self.update_queue.empty():
                x, y, theta = self.update_queue.get_nowait()
                
                # Convert from meters to grid coordinates
                if self.cell_to_meter > 0:
                    grid_x = x / self.cell_to_meter
                    grid_y = y / self.cell_to_meter
                else:
                    grid_x = x
                    grid_y = y
                
                self.x = grid_x
                self.y = grid_y
                self.theta = theta
                self.trajectory_points.append((grid_x, grid_y))
                update_needed = True
            
            # Update plot if needed
            current_time = time.time() * 1000  # ms
            if update_needed and (current_time - self.last_plot_update > self.update_interval):
                self._update_plot()
                self.last_plot_update = current_time
                
        except Exception as e:
            print(f"Error in position update queue: {e}")
        
        # Schedule next check
        if self.position_window:
            self.position_window.after(50, self.check_update_queue)
    
    def update_robot_position(self, x, y, theta):
        """Update robot position with data from robot"""
        if not self.is_active:
            return
        # x, y are real coordinates (meters)
        self.update_queue.put((x, y, theta))
    
    def _update_plot(self):
        """Update plot with current position data"""
        if not self.is_active or not self.ax:
            return
        
        with self.lock:
            # Clear and redraw
            self.ax.clear()
            
            # Draw grid
            if self.grid is not None:
                # Draw map - origin='upper' for correct coordinate direction
                self.ax.imshow(self.grid, cmap='binary', origin='upper',
                              extent=[0, self.grid_width, 0, self.grid_height])
                
                # Draw grid lines
                for i in range(self.grid_width + 1):
                    alpha = 1.0 if i % (6 * (ORIGINAL_CELL/NEW_CELL)) == 0 else 0.2
                    self.ax.axvline(i, color='gray', alpha=alpha)
                for i in range(self.grid_height + 1):
                    alpha = 1.0 if i % (6 * (ORIGINAL_CELL/NEW_CELL))== 0 else 0.2
                    self.ax.axhline(i, color='gray', alpha=alpha)
                
                # Draw start position
                if self.start_pos:
                    self.ax.plot(self.start_pos[0] + 0.5, self.start_pos[1] + 0.5, 'go', 
                                markersize=6, markeredgecolor='k', label='Start')
                    
                if self.destination:     
                    x_dest, y_dest = self.destination
                    self.ax.plot(x_dest + 0.5, y_dest + 0.5, 'ro', markersize=6, 
                                markeredgecolor='k', label='Target')
                    
                    # Show coordinate in cm
                    dest_x_cm = x_dest * self.cell_to_meter * 100
                    dest_y_cm = y_dest * self.cell_to_meter * 100
                    self.ax.text(x_dest + 1, y_dest + 0.5, 
                                f"({dest_x_cm:.1f}, {dest_y_cm:.1f}) cm",
                                fontsize=6, fontweight='bold',
                                bbox=dict(facecolor='white', alpha=0.7, boxstyle='round'))
            
            # Draw trajectory
            if self.trajectory_points:
                x_points = [p[0] + 0.5 for p in self.trajectory_points]
                y_points = [p[1] + 0.5 for p in self.trajectory_points]
                self.ax.plot(x_points, y_points, 'b-', linewidth=2, alpha=0.7)
            
            # Draw robot at current position
            theta_rad = math.radians(self.theta)
            self._draw_robot(self.x, self.y, theta_rad)
            
            # Format plot
            self.ax.set_aspect('equal')
            self.ax.set_xlabel(f'Width: {self.grid_width} cells ({self.grid_width*self.cell_to_meter*100:.1f}cm)')
            self.ax.set_ylabel(f'Height: {self.grid_height} cells ({self.grid_height*self.cell_to_meter*100:.1f}cm)')
            self.ax.set_title('Robot Position - Enter coordinates to set destination')
            
            # Set normal grid limits
            self.ax.set_xlim(0, self.grid_width)
            self.ax.set_ylim(0, self.grid_height)
            
            # Display current position in cm
            pos_x_cm = self.x * self.cell_to_meter * 100
            pos_y_cm = self.y * self.cell_to_meter * 100
            self.ax.text(self.grid_width * 1.05, self.grid_height * 0.95,
                        f'Position: ({pos_x_cm:.1f}, {pos_y_cm:.1f}) cm\n'
                        f'Angle: {self.theta:.1f}°', 
                        fontsize=10, fontweight='bold',
                        bbox=dict(facecolor='yellow', alpha=0.8, boxstyle='round,pad=0.5'))
            
            # Add legend
            # Add legend
            handles, labels = self.ax.get_legend_handles_labels()
            if handles:
                self.ax.legend(loc='upper right', fontsize=8)
            
            self.fig.tight_layout()
            self.canvas.draw()
    
    def _draw_robot(self, x, y, theta):
        """Draw robot at specified position and orientation using real-world radius"""
        # Convert robot radius from meters to grid cells
        robot_radius_cells = self.robot_radius_meters / self.cell_to_meter
        
        # Draw robot center
        self.ax.plot(x + 0.5, y + 0.5, 'bo', markersize=4, label='Robot')
        
        # Draw robot body as a circle with actual radius
        robot_circle = plt.Circle((x + 0.5, y + 0.5), robot_radius_cells, 
                                  fill=False, color='blue', linestyle='-', linewidth=1.5, alpha=0.7)
        self.ax.add_patch(robot_circle)
        
        # Draw direction arrow - scale based on robot size
        arrow_length = robot_radius_cells * 1.2  # Arrow extends beyond the robot body
        # Y-axis heading convention: theta=0 points North (+Y)
        dx = arrow_length * math.sin(theta)
        dy = arrow_length * math.cos(theta)
        
        self.ax.arrow(x + 0.5, y + 0.5, dx, dy, 
                     head_width=robot_radius_cells * 0.2, 
                     head_length=robot_radius_cells * 0.3, 
                     fc='blue', ec='blue', linewidth=1.5)
        
        # Draw wheels at the perimeter of the robot
        wheel_angles = [0, 2*np.pi/3, 4*np.pi/3]  # 0°, 120°, 240°
        
        for i, angle in enumerate(wheel_angles):
            # Calculate global wheel angle
            global_angle = theta + angle
            
            # Calculate wheel position at the edge of the robot circle
            wheel_x = x + 0.5 + robot_radius_cells * np.cos(global_angle)
            wheel_y = y + 0.5 + robot_radius_cells * np.sin(global_angle)
            
            # Number the wheels
            self.ax.text(wheel_x, wheel_y, str(i+1), fontsize=8, 
                        ha='center', va='center', color='black', 
                        bbox=dict(facecolor='white', alpha=0.7, boxstyle='circle'))
    
    def show(self):
        """Show position window"""
        if not self.is_active or not self.position_window:
            self.initialize_plot()
        else:
            self.position_window.deiconify()  # Show hidden window
            self.is_active = True
    
    def close(self):
        """Close position window"""
        if self.position_window:
            self.position_window.withdraw()  # Hide window instead of completely closing
            #self.is_active = False

# Create singleton instance
robot_position_visualizer = RobotPositionVisualizer()

# # Example function to send destination to client
# def send_destination_to_client(x, y):
#     """Example function to send destination to client"""
#     print(f"Sending destination to client: X={x}m, Y={y}m")

# # Set callback function to send destination
# robot_position_visualizer.set_destination_callback(send_destination_to_client)