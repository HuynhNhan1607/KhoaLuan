import tkinter as tk
from tkinter import ttk, messagebox
import math
import time
import numpy as np

ANGLE_ROTATION = 2 * math.pi / 3  # (Rad/s)
TIME_ROTATION = 9  # 1s
RPM = 65
M_PER_ROUND = 0.06 * math.pi

class ControlGUI:
    def __init__(self, root, robot_id=None):
        self.root = root
        self.server = None
        self.robot_id = robot_id  # Multi-robot: store robot_id
        self.orientation = 0  # Hướng của robot (góc quay)
        # self.max_speed = RPM * M_PER_ROUND / 60
        self.max_speed = 0.15
        self.control_window = None
        self.recording = False
        self.trajectory = []
        self.running_trajectory = False
        self.trajectory_index = 0
        
        # Rectangle movement parameters
        self.rect_running = False
        self.rect_step = 0
        self.rect_length_time = 16.0  # Time for length (vy direction) in seconds
        self.rect_width_time = 16.0   # Time for width (vx direction) in seconds
        self.rect_velocity = 0.3     # Fixed velocity m/s
        self.rect_pause_time = 5000   # Pause at vertices in ms
        
        # Hourglass movement parameters
        self.hourglass_running = False
        self.hourglass_step = 0
        self.hourglass_horizontal_time = 16.0  # Time for horizontal (vx) in seconds
        self.hourglass_diagonal_time = 11.0    # Time for diagonal in seconds
        self.hourglass_velocity = 0.3 # Fixed velocity m/s
        self.hourglass_angle = 52    # Diagonal angle in degrees
        self.hourglass_pause_time = 5000  # Pause at vertices in ms
        
        # Circle movement parameters
        self.circle_running = False
        self.circle_step = 0
        self.circle_sub_step = 0      # Sub-step within each quarter
        self.circle_radius = 1.5      # Radius in meters
        self.circle_velocity = 0.3    # Linear velocity m/s
        self.circle_pause_time = 5000 # Pause between quarters in ms (5s)
        self.circle_steps_per_quarter = 20  # Number of sub-steps per quarter for smooth motion
        
        # Game-like control: track which keys are currently pressed
        self.active_keys = set()
        
        # Continuous update loop
        self.update_interval = 50  # ms (20 FPS update rate)
        self.update_job = None
        
        # Track last command to avoid sending duplicates
        self._last_command = (0, 0, 0)
        
        # Debounce special keys (e.g., R) to avoid auto-repeat toggling
        self._r_down = False
        
        # X11 auto-repeat detection: track key press timestamps
        # When KeyRelease is followed immediately by KeyPress, it's auto-repeat
        self._key_press_time = {}  # key -> timestamp of last KeyPress
        self._pending_release = {}  # key -> scheduled release callback id
        
        # Danh sách lưu trữ biến tốc độ động cơ
        self.motor_speed_vars = []  # Đảm bảo list này được khởi tạo trước khi tạo widgets

    def run(self):
        if self.control_window is not None and self.control_window.winfo_exists():
            self.control_window.focus_force()
            return
            
        self.control_window = tk.Toplevel(self.root)
        # Multi-robot: Add robot_id to title
        title = f"Robot {self.robot_id} Control Interface" if self.robot_id else "Robot Control Interface"
        self.control_window.title(title)
        self.control_window.geometry("400x300")
        self.control_window.resizable(True, True)
        
        # Đảm bảo cửa sổ điều khiển được tách ra khỏi cửa sổ chính
        self.control_window.transient(self.root)
        
        # Vùng hiển thị thông tin
        info_frame = ttk.LabelFrame(self.control_window, text="Robot Information", padding=10)
        info_frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        # Hiển thị hướng dẫn sử dụng
        controls_text = """
        Control Keys (Game-like, hold keys for continuous movement):
        W: Move Forward
        S: Move Backward
        A: Move Left
        D: Move Right
        Q: Rotate Left
        E: Rotate Right
        
        Combine keys for diagonal movement:
        W+A: Forward-Left    W+D: Forward-Right
        S+A: Backward-Left   S+D: Backward-Right
        W+Q: Forward + Rotate Left
        
        Other keys:
        R: Toggle Recording
        T: Run Trajectory
        O: Rectangle Movement (vy→vx→-vy→-vx)
        P: Hourglass Movement (6 segments)
        L: Circle Movement (4 quarters)
        C: Emergency Stop
        Space: Stop Movement
        ESC: Close Window
        """
        
        ttk.Label(info_frame, text=controls_text, justify="left").pack(anchor="w")
        
        # Hiển thị trạng thái
        self.status_frame = ttk.Frame(self.control_window, padding=5)
        self.status_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(self.status_frame, text="Status:").pack(side="left")
        self.status_label = ttk.Label(self.status_frame, text="Ready")
        self.status_label.pack(side="left", padx=5)
        
        # Hiển thị nút điều khiển
        control_buttons = ttk.Frame(self.control_window, padding=5)
        control_buttons.pack(fill="x", padx=10, pady=5)
        
        ttk.Button(control_buttons, text="Record Path", command=self.toggle_recording).pack(side="left", padx=5)
        ttk.Button(control_buttons, text="Run Path", command=self.run_trajectory).pack(side="left", padx=5)
        ttk.Button(control_buttons, text="Rectangle", command=self.show_rectangle_dialog).pack(side="left", padx=5)
        ttk.Button(control_buttons, text="Hourglass", command=self.show_hourglass_dialog).pack(side="left", padx=5)
        ttk.Button(control_buttons, text="Circle", command=self.show_circle_dialog).pack(side="left", padx=5)
        ttk.Button(control_buttons, text="Stop", command=self.stop_robot).pack(side="left", padx=5)
        ttk.Button(control_buttons, text="Close", command=self.close_control).pack(side="right", padx=5)
        
        # Khung điều khiển động cơ
        motor_control_frame = ttk.LabelFrame(self.control_window, text="Motor Control", padding=10)
        motor_control_frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        # Tạo các widget điều khiển động cơ
        for i in range(4):  
            motor_frame = tk.Frame(motor_control_frame)
            motor_frame.pack(fill=tk.X, padx=5, pady=2)
            
            tk.Label(motor_frame, text=f"Motor {i}:").pack(side=tk.LEFT)
            
            speed_var = tk.StringVar(value="0")
            self.motor_speed_vars.append(speed_var)
            
            speed_entry = tk.Entry(motor_frame, textvariable=speed_var, width=8)
            speed_entry.pack(side=tk.LEFT, padx=5)
            
            set_button = tk.Button(motor_frame, text="Set", 
                                  command=lambda idx=i: self.set_motor_speed(idx))
            set_button.pack(side=tk.LEFT, padx=5)
        
        # Đăng ký các sự kiện bàn phím
        self.control_window.bind("<KeyPress>", self.on_key_press)
        self.control_window.bind("<KeyRelease>", self.on_key_release)
        
        # Đăng ký sự kiện đóng cửa sổ
        self.control_window.protocol("WM_DELETE_WINDOW", self.close_control)
        
        # Đặt focus để nhận sự kiện bàn phím
        self.control_window.focus_set()
        
        # Start continuous update loop for game-like control
        self.start_continuous_update()

    def start_continuous_update(self):
        """Start continuous update loop for responsive control"""
        self.update_movement()
    
    def update_movement(self):
        """Continuously update robot movement based on active keys"""
        if self.control_window is None or not self.control_window.winfo_exists():
            return
        
        # Calculate movement based on active keys
        dot_x, dot_y, dot_theta = 0, 0, 0
        status = "Ready"
        
        # Movement keys
        if "w" in self.active_keys:
            dot_y += self.max_speed
            status = "Forward"
        if "s" in self.active_keys:
            dot_y -= self.max_speed
            status = "Backward"
        if "a" in self.active_keys:
            dot_x -= self.max_speed
            status = "Left"
        if "d" in self.active_keys:
            dot_x += self.max_speed
            status = "Right"
        
        # Rotation keys
        if "q" in self.active_keys:
            dot_theta = ANGLE_ROTATION / TIME_ROTATION
            self.orientation += dot_theta * 0.1
            status += " + Rot Left"
        if "e" in self.active_keys:
            dot_theta = -(ANGLE_ROTATION / TIME_ROTATION)
            self.orientation += dot_theta * 0.1
            status += " + Rot Right"
        
        # Combine movements for diagonal (normalize speed)
        if (dot_x != 0 and dot_y != 0):
            # Normalize to maintain constant speed when moving diagonally
            magnitude = math.sqrt(dot_x**2 + dot_y**2)
            dot_x = (dot_x / magnitude) * self.max_speed
            dot_y = (dot_y / magnitude) * self.max_speed
            status = "Diagonal"
        
        # Create current command tuple
        current_command = (round(dot_x, 4), round(dot_y, 4), round(dot_theta, 4))
        
        # Only send command if it's different from the last one
        if current_command != self._last_command:
            self.send_kinematic_command(dot_x, dot_y, dot_theta)
            self._last_command = current_command
            
            # Update status only when command changes
            if dot_x != 0 or dot_y != 0 or dot_theta != 0:
                self.update_status(status)
                
                # Record trajectory if recording is active
                if self.recording:
                    self.trajectory.append((dot_x, dot_y, dot_theta))
            else:
                self.update_status("Ready")
        
        # Schedule next update
        self.update_job = self.control_window.after(self.update_interval, self.update_movement)

    def on_key_press(self, event):
        """Handle key press events - add to active keys set"""
        key = event.keysym.lower()
        
        # Special command keys (execute once on press)
        if key == "escape":
            self.close_control()
            return
        elif key == "r":
            # Debounce: only toggle once per physical press
            if self._r_down:
                return
            self._r_down = True
            self.toggle_recording()
            return
        elif key == "t":
            self.run_trajectory()
            return
        elif key == "o":
            self.show_rectangle_dialog()
            return
        elif key == "p":
            self.show_hourglass_dialog()
            return
        elif key == "l":
            self.show_circle_dialog()
            return
        elif key == "space":
            self.stop_robot()
            return
        elif key == "c":
            # Emergency stop shortcut
            self.stop_robot()
            return
        
        # Movement keys - add to active set and record timestamp
        if key in ["w", "s", "a", "d", "q", "e"]:
            self.active_keys.add(key)
            # Record timestamp of this KeyPress for auto-repeat detection
            self._key_press_time[key] = time.time()
            # Cancel any pending release for this key (from auto-repeat)
            if key in self._pending_release:
                self.control_window.after_cancel(self._pending_release[key])
                del self._pending_release[key]
    
    def on_key_release(self, event):
        """Handle key release events - remove from active keys set
        
        Uses X11 auto-repeat filtering: when holding a key, X11 sends
        alternating KeyRelease and KeyPress events very close together.
        We detect this by delaying the actual key removal and checking
        if a new KeyPress came in during that window.
        """
        key = event.keysym.lower()
        
        # Filter out fake KeyRelease from X11 auto-repeat:
        # Schedule the actual key release after a short delay (20ms)
        # If a KeyPress for the same key fires during this window,
        # it will cancel the pending release (auto-repeat case)
        if key in ["w", "s", "a", "d", "q", "e"]:
            # Record the release time for comparison
            release_time = time.time()
            # Schedule delayed release check
            callback_id = self.control_window.after(
                20,  # 20ms delay - enough to catch auto-repeat KeyPress
                lambda k=key, t=release_time: self._delayed_key_release(k, t)
            )
            self._pending_release[key] = callback_id
            return
        
        # Release debounce flags
        if key == "r":
            self._r_down = False
    
    def _delayed_key_release(self, key, release_time):
        """Process key release after delay for auto-repeat filtering.
        
        Only removes the key if no new KeyPress occurred after the release.
        Args:
            key: The key that was released
            release_time: Timestamp when KeyRelease was received
        """
        # Clean up pending release tracker
        if key in self._pending_release:
            del self._pending_release[key]
        
        # Check if a new KeyPress occurred AFTER this KeyRelease
        # If so, it was auto-repeat and we should NOT remove the key
        last_press_time = self._key_press_time.get(key, 0)
        
        if last_press_time > release_time:
            # A new KeyPress occurred after this release - it's auto-repeat
            # Keep the key in active_keys
            return
        
        # Real key release - remove from active set
        self.active_keys.discard(key)
    
    def update_status(self, message):
        if self.status_label:
            self.status_label.config(text=message)
    
    def close_control(self):
        """Close control window and cleanup"""
        # Cancel the update loop
        if self.update_job:
            self.control_window.after_cancel(self.update_job)
            self.update_job = None
        
        # Stop the robot
        self.stop_robot()
        
        # Close window
        if self.control_window:
            self.control_window.destroy()
            self.control_window = None
    
    def toggle_recording(self):
        self.recording = not self.recording
        if self.recording:
            self.update_status("Recording Path")
            self.trajectory = []
        else:
            self.update_status("Path Recorded")
            self.save_trajectory()
    
    def save_trajectory(self):
        if not self.trajectory:
            return
        
        try:
            # Multi-robot: Include robot_id in filename
            filename = f"trajectory_robot{self.robot_id}.txt" if self.robot_id else "trajectory.txt"
            with open(filename, "w") as f:
                for cmd in self.trajectory:
                    f.write(f"{cmd[0]},{cmd[1]},{cmd[2]}\n")
            print(f"Trajectory saved to {filename}")
        except Exception as e:
            print(f"Error saving trajectory: {e}")
    
    def load_trajectory(self):
        try:
            self.trajectory = []
            # Multi-robot: Include robot_id in filename
            filename = f"trajectory_robot{self.robot_id}.txt" if self.robot_id else "trajectory.txt"
            with open(filename, "r") as f:
                for line in f:
                    values = line.strip().split(",")
                    if len(values) == 3:
                        self.trajectory.append((float(values[0]), float(values[1]), float(values[2])))
            print(f"Loaded {len(self.trajectory)} commands from trajectory")
            return True
        except Exception as e:
            print(f"Error loading trajectory: {e}")
            return False
    
    def run_trajectory(self):
        if self.running_trajectory:
            return
            
        if self.load_trajectory():
            self.running_trajectory = True
            self.trajectory_index = 0
            self.update_status("Running Path")
            self.execute_trajectory()
        else:
            self.update_status("No Path Available")
    
    def execute_trajectory(self):
        if not self.running_trajectory or self.trajectory_index >= len(self.trajectory):
            self.running_trajectory = False
            self.update_status("Path Completed")
            return
            
        if self.control_window is None or not self.control_window.winfo_exists():
            self.running_trajectory = False
            return
            
        cmd = self.trajectory[self.trajectory_index]
        dot_x, dot_y, dot_theta = cmd
        
        self.send_kinematic_command(dot_x, dot_y, dot_theta)
        self.trajectory_index += 1
        
        # Lập lịch chạy lệnh tiếp theo sau một khoảng thời gian
        delay = 200  # ms
        self.control_window.after(delay, self.execute_trajectory)
    
    def stop_robot(self):
        """Stop robot immediately and clear all active keys"""
        self.active_keys.clear()
        self.send_kinematic_command(0, 0, 0)
        self.running_trajectory = False
        self.rect_running = False
        self.hourglass_running = False
        self.circle_running = False
        self.update_status("Stopped")
    
    def show_rectangle_dialog(self):
        """Show dialog to input rectangle movement parameters"""
        if self.rect_running:
            self.update_status("Rectangle already running")
            return
            
        dialog = tk.Toplevel(self.control_window)
        dialog.title("Rectangle Movement Parameters")
        dialog.geometry("300x200")
        dialog.transient(self.control_window)
        dialog.grab_set()
        
        # Length time input (vy direction)
        ttk.Label(dialog, text="Thời gian chiều dài (s):").pack(pady=5)
        length_var = tk.StringVar(value="2.0")
        ttk.Entry(dialog, textvariable=length_var, width=10).pack()
        
        # Width time input (vx direction)
        ttk.Label(dialog, text="Thời gian chiều rộng (s):").pack(pady=5)
        width_var = tk.StringVar(value="1.0")
        ttk.Entry(dialog, textvariable=width_var, width=10).pack()
        
        # Info label
        ttk.Label(dialog, text=f"Vận tốc cố định: {self.rect_velocity} m/s").pack(pady=10)
        
        def start_rectangle():
            try:
                self.rect_length_time = float(length_var.get())
                self.rect_width_time = float(width_var.get())
                dialog.destroy()
                self.start_rectangle_movement()
            except ValueError:
                messagebox.showerror("Error", "Vui lòng nhập số hợp lệ")
        
        ttk.Button(dialog, text="Bắt đầu", command=start_rectangle).pack(pady=10)
        ttk.Button(dialog, text="Hủy", command=dialog.destroy).pack()
    
    def start_rectangle_movement(self):
        """Start rectangle movement pattern: vy -> pause -> vx -> pause -> -vy -> pause -> -vx -> pause"""
        self.rect_running = True
        self.rect_step = 0
        self.update_status("Rectangle: Starting")
        self.execute_rectangle_step()
    
    def execute_rectangle_step(self):
        """Execute one step of rectangle movement pattern"""
        if not self.rect_running:
            return
            
        if self.control_window is None or not self.control_window.winfo_exists():
            self.rect_running = False
            return
        
        v = self.rect_velocity
        
        # Rectangle movement pattern:
        # Step 0: Move +vy (forward) for length_time
        # Step 1: Pause at vertex
        # Step 2: Move +vx (right) for width_time
        # Step 3: Pause at vertex
        # Step 4: Move -vy (backward) for length_time
        # Step 5: Pause at vertex
        # Step 6: Move -vx (left) for width_time
        # Step 7: Pause at vertex (complete)
        
        if self.rect_step == 0:
            # Move +vy (forward)
            self.send_kinematic_command(0, v, 0)
            self.update_status("Rectangle: +vy (Forward)")
            delay_ms = int(self.rect_length_time * 1000)
            self.rect_step = 1
            self.control_window.after(delay_ms, self.execute_rectangle_step)
            
        elif self.rect_step == 1:
            # Pause at vertex 1
            self.send_kinematic_command(0, 0, 0)
            self.update_status("Rectangle: Pause (Vertex 1)")
            self.rect_step = 2
            self.control_window.after(self.rect_pause_time, self.execute_rectangle_step)
            
        elif self.rect_step == 2:
            # Move +vx (right)
            self.send_kinematic_command(v, 0, 0)
            self.update_status("Rectangle: +vx (Right)")
            delay_ms = int(self.rect_width_time * 1000)
            self.rect_step = 3
            self.control_window.after(delay_ms, self.execute_rectangle_step)
            
        elif self.rect_step == 3:
            # Pause at vertex 2
            self.send_kinematic_command(0, 0, 0)
            self.update_status("Rectangle: Pause (Vertex 2)")
            self.rect_step = 4
            self.control_window.after(self.rect_pause_time, self.execute_rectangle_step)
            
        elif self.rect_step == 4:
            # Move -vy (backward)
            self.send_kinematic_command(0, -v, 0)
            self.update_status("Rectangle: -vy (Backward)")
            delay_ms = int(self.rect_length_time * 1000)
            self.rect_step = 5
            self.control_window.after(delay_ms, self.execute_rectangle_step)
            
        elif self.rect_step == 5:
            # Pause at vertex 3
            self.send_kinematic_command(0, 0, 0)
            self.update_status("Rectangle: Pause (Vertex 3)")
            self.rect_step = 6
            self.control_window.after(self.rect_pause_time, self.execute_rectangle_step)
            
        elif self.rect_step == 6:
            # Move -vx (left)
            self.send_kinematic_command(-v, 0, 0)
            self.update_status("Rectangle: -vx (Left)")
            delay_ms = int(self.rect_width_time * 1000)
            self.rect_step = 7
            self.control_window.after(delay_ms, self.execute_rectangle_step)
            
        elif self.rect_step == 7:
            # Pause at vertex 4 (back to start)
            self.send_kinematic_command(0, 0, 0)
            self.update_status("Rectangle: Complete")
            self.rect_running = False
            self.rect_step = 0
    
    def show_hourglass_dialog(self):
        """Show dialog to input hourglass movement parameters"""
        if self.hourglass_running:
            self.update_status("Hourglass already running")
            return
            
        dialog = tk.Toplevel(self.control_window)
        dialog.title("Hourglass Movement Parameters")
        dialog.geometry("300x300")
        dialog.transient(self.control_window)
        dialog.grab_set()
        
        # Time for horizontal segment (vx)
        ttk.Label(dialog, text="Thời gian ngang - vx (s):").pack(pady=5)
        horiz_time_var = tk.StringVar(value="2.0")
        ttk.Entry(dialog, textvariable=horiz_time_var, width=10).pack()
        
        # Time for diagonal segment
        ttk.Label(dialog, text="Thời gian chéo (s):").pack(pady=5)
        diag_time_var = tk.StringVar(value="2.0")
        ttk.Entry(dialog, textvariable=diag_time_var, width=10).pack()
        
        # Diagonal angle
        ttk.Label(dialog, text="Góc chéo (độ):").pack(pady=5)
        angle_var = tk.StringVar(value="54")
        ttk.Entry(dialog, textvariable=angle_var, width=10).pack()
        
        # Info label
        ttk.Label(dialog, text=f"Vận tốc cố định: {self.hourglass_velocity} m/s").pack(pady=5)
        ttk.Label(dialog, text="Pattern: vx → xéo lên → vx → xéo lui").pack(pady=5)
        
        def start_hourglass():
            try:
                self.hourglass_horizontal_time = float(horiz_time_var.get())
                self.hourglass_diagonal_time = float(diag_time_var.get())
                self.hourglass_angle = float(angle_var.get())
                dialog.destroy()
                self.start_hourglass_movement()
            except ValueError:
                messagebox.showerror("Error", "Vui lòng nhập số hợp lệ")
        
        ttk.Button(dialog, text="Bắt đầu", command=start_hourglass).pack(pady=10)
        ttk.Button(dialog, text="Hủy", command=dialog.destroy).pack()
    
    def start_hourglass_movement(self):
        """Start hourglass movement pattern: vx -> diagonal up 54° -> vx -> diagonal back 54°"""
        self.hourglass_running = True
        self.hourglass_step = 0
        self.update_status("Hourglass: Starting")
        self.execute_hourglass_step()
    
    def execute_hourglass_step(self):
        """Execute one step of hourglass movement pattern"""
        if not self.hourglass_running:
            return
            
        if self.control_window is None or not self.control_window.winfo_exists():
            self.hourglass_running = False
            return
        
        v = self.hourglass_velocity
        angle = self.hourglass_angle
        
        # Calculate diagonal velocity components for all 4 directions
        # Up-Left: angle = 180 - input_angle (e.g., 126°)
        up_left_rad = math.radians(180 - angle)
        up_left_vx = v * math.cos(up_left_rad)   # negative (left)
        up_left_vy = v * math.sin(up_left_rad)   # positive (forward)
        
        # Up-Right: angle = input_angle (e.g., 54°)
        up_right_rad = math.radians(angle)
        up_right_vx = v * math.cos(up_right_rad)  # positive (right)
        up_right_vy = v * math.sin(up_right_rad)  # positive (forward)
        
        # Down-Left: angle = -(180 - input_angle) = -126° (or 234°)
        down_left_rad = math.radians(-angle)
        down_left_vx = v * math.cos(down_left_rad)   # negative (left)
        down_left_vy = v * math.sin(down_left_rad)   # negative (backward)
        
        # Down-Right: angle = -input_angle (e.g., -54°)
        down_right_rad = math.radians(-(180 - angle))
        down_right_vx = v * math.cos(down_right_rad)  # positive (right)
        down_right_vy = v * math.sin(down_right_rad)  # negative (backward)
        
        # Hourglass movement pattern (6 segments):
        # Step 0: Move +vx (right) for horizontal_time
        # Step 1: Pause
        # Step 2: Move diagonal UP-LEFT (126°) for diagonal_time
        # Step 3: Pause
        # Step 4: Move diagonal UP-RIGHT (54°) for diagonal_time
        # Step 5: Pause
        # Step 6: Move -vx (left) for horizontal_time
        # Step 7: Pause
        # Step 8: Move diagonal DOWN-LEFT (-126°) for diagonal_time
        # Step 9: Pause
        # Step 10: Move diagonal DOWN-RIGHT (-54°) for diagonal_time
        # Step 11: Complete
        
        horiz_delay_ms = int(self.hourglass_horizontal_time * 1000)
        diag_delay_ms = int(self.hourglass_diagonal_time * 1000)
        
        if self.hourglass_step == 0:
            # Move +vx (right)
            self.send_kinematic_command(v, 0, 0)
            self.update_status("Hourglass: +vx (Right)")
            self.hourglass_step = 1
            self.control_window.after(horiz_delay_ms, self.execute_hourglass_step)
            
        elif self.hourglass_step == 1:
            # Pause
            self.send_kinematic_command(0, 0, 0)
            self.update_status("Hourglass: Pause (1)")
            self.hourglass_step = 2
            self.control_window.after(self.hourglass_pause_time, self.execute_hourglass_step)
            
        elif self.hourglass_step == 2:
            # Move diagonal UP-LEFT
            self.send_kinematic_command(up_left_vx, up_left_vy, 0)
            self.update_status(f"Hourglass: Up-Left ({180 - angle}°)")
            self.hourglass_step = 3
            self.control_window.after(diag_delay_ms, self.execute_hourglass_step)
            
        elif self.hourglass_step == 3:
            # Pause
            self.send_kinematic_command(0, 0, 0)
            self.update_status("Hourglass: Pause (2)")
            self.hourglass_step = 4
            self.control_window.after(self.hourglass_pause_time, self.execute_hourglass_step)
            
        elif self.hourglass_step == 4:
            # Move diagonal UP-RIGHT
            self.send_kinematic_command(up_right_vx, up_right_vy, 0)
            self.update_status(f"Hourglass: Up-Right ({angle}°)")
            self.hourglass_step = 5
            self.control_window.after(diag_delay_ms, self.execute_hourglass_step)
            
        elif self.hourglass_step == 5:
            # Pause
            self.send_kinematic_command(0, 0, 0)
            self.update_status("Hourglass: Pause (3)")
            self.hourglass_step = 6
            self.control_window.after(self.hourglass_pause_time, self.execute_hourglass_step)
            
        elif self.hourglass_step == 6:
            # Move -vx (left)
            self.send_kinematic_command(-v, 0, 0)
            self.update_status("Hourglass: -vx (Left)")
            self.hourglass_step = 7
            self.control_window.after(horiz_delay_ms, self.execute_hourglass_step)
            
        elif self.hourglass_step == 7:
            # Pause
            self.send_kinematic_command(0, 0, 0)
            self.update_status("Hourglass: Pause (4)")
            self.hourglass_step = 8
            self.control_window.after(self.hourglass_pause_time, self.execute_hourglass_step)
            
        elif self.hourglass_step == 8:
            # Move diagonal DOWN-LEFT
            self.send_kinematic_command(down_left_vx, down_left_vy, 0)
            self.update_status(f"Hourglass: Down-Left ({-(180 - angle)}°)")
            self.hourglass_step = 9
            self.control_window.after(diag_delay_ms, self.execute_hourglass_step)
            
        elif self.hourglass_step == 9:
            # Pause
            self.send_kinematic_command(0, 0, 0)
            self.update_status("Hourglass: Pause (5)")
            self.hourglass_step = 10
            self.control_window.after(self.hourglass_pause_time, self.execute_hourglass_step)
            
        elif self.hourglass_step == 10:
            # Move diagonal DOWN-RIGHT
            self.send_kinematic_command(down_right_vx, down_right_vy, 0)
            self.update_status(f"Hourglass: Down-Right (-{angle}°)")
            self.hourglass_step = 11
            self.control_window.after(diag_delay_ms, self.execute_hourglass_step)
            
        elif self.hourglass_step == 11:
            # Complete
            self.send_kinematic_command(0, 0, 0)
            self.update_status("Hourglass: Complete")
            self.hourglass_running = False
            self.hourglass_step = 0
    
    def show_circle_dialog(self):
        """Show dialog to input circle movement parameters"""
        if self.circle_running:
            self.update_status("Circle already running")
            return
            
        dialog = tk.Toplevel(self.control_window)
        dialog.title("Circle Movement Parameters")
        dialog.geometry("350x250")
        dialog.transient(self.control_window)
        dialog.grab_set()
        
        # Radius input
        ttk.Label(dialog, text="Bán kính R (m):").pack(pady=5)
        radius_var = tk.StringVar(value="1.5")
        ttk.Entry(dialog, textvariable=radius_var, width=10).pack()
        
        # Velocity is fixed
        ttk.Label(dialog, text=f"Vận tốc cố định: {self.circle_velocity} m/s").pack(pady=5)
        
        # Calculate and show info
        def update_info(*args):
            try:
                r = float(radius_var.get())
                quarter_arc = math.pi * r / 2  # Quarter circumference
                quarter_time = quarter_arc / self.circle_velocity
                omega = self.circle_velocity / r  # Angular velocity
                info_label.config(text=f"Chu vi 1/4 vòng: {quarter_arc:.2f}m\n"
                                      f"Thời gian 1/4 vòng: {quarter_time:.2f}s\n"
                                      f"Vận tốc góc ω: {omega:.4f} rad/s\n"
                                      f"Pause giữa các phần: 5s")
            except:
                info_label.config(text="Nhập bán kính hợp lệ")
        
        radius_var.trace_add("write", update_info)
        
        info_label = ttk.Label(dialog, text="", justify="center")
        info_label.pack(pady=10)
        update_info()
        
        def start_circle():
            try:
                self.circle_radius = float(radius_var.get())
                dialog.destroy()
                self.start_circle_movement()
            except ValueError:
                messagebox.showerror("Error", "Vui lòng nhập số hợp lệ")
        
        ttk.Button(dialog, text="Bắt đầu", command=start_circle).pack(pady=10)
        ttk.Button(dialog, text="Hủy", command=dialog.destroy).pack()
    
    def start_circle_movement(self):
        """Start circle movement: 4 quarters with pauses between"""
        self.circle_running = True
        self.circle_step = 0
        self.update_status("Circle: Starting")
        self.execute_circle_step()
    
    def execute_circle_step(self):
        """Execute one step of circle movement for mecanum robot.
        
        For mecanum robot, we trace a circle by varying vx and vy
        while keeping heading constant (dot_theta = 0).
        
        Velocity direction is always tangent to the circle:
        - At angle θ on circle: vx = v * sin(θ), vy = v * cos(θ)
        
        Circle is divided into 4 quarters, each with sub-steps for smooth motion.
        """
        if not self.circle_running:
            return
            
        if self.control_window is None or not self.control_window.winfo_exists():
            self.circle_running = False
            return
        
        v = self.circle_velocity
        r = self.circle_radius
        steps_per_quarter = self.circle_steps_per_quarter
        
        # Calculate time for each sub-step
        # Quarter arc length = π * R / 2
        # Quarter time = arc / v = π * R / (2 * v)
        quarter_time = (math.pi * r) / (2 * v)
        step_time = quarter_time / steps_per_quarter
        step_delay_ms = int(step_time * 1000)
        
        # Angle per sub-step = 90° / steps_per_quarter
        angle_per_step = (math.pi / 2) / steps_per_quarter
        
        # Current quarter (0-3) and sub-step within quarter
        current_quarter = self.circle_step
        current_sub_step = self.circle_sub_step
        
        # Calculate current angle on circle
        # Quarter 0: 0° to 90°, Quarter 1: 90° to 180°, etc.
        base_angle = current_quarter * (math.pi / 2)
        current_angle = base_angle + current_sub_step * angle_per_step
        
        # For a circle traced counterclockwise starting from +Y direction:
        # At angle θ, tangent velocity direction is:
        # vx = v * sin(θ)  (positive = right)
        # vy = v * cos(θ)  (positive = forward)
        # This makes the robot move tangent to the circle
        vx = v * math.sin(current_angle)
        vy = v * math.cos(current_angle)
        
        if current_quarter < 4:  # Still moving through quarters
            if current_sub_step < steps_per_quarter:
                # Execute this sub-step
                self.send_kinematic_command(vx, vy, 0)  # dot_theta = 0 for mecanum
                progress = (current_quarter * steps_per_quarter + current_sub_step + 1) / (4 * steps_per_quarter)
                self.update_status(f"Circle: Q{current_quarter+1}/4 [{current_sub_step+1}/{steps_per_quarter}] ({progress*100:.0f}%)")
                
                self.circle_sub_step += 1
                self.control_window.after(step_delay_ms, self.execute_circle_step)
            else:
                # Quarter complete, pause
                self.send_kinematic_command(0, 0, 0)
                self.update_status(f"Circle: Pause after Q{current_quarter+1}/4 (5s)")
                
                # Move to next quarter
                self.circle_step += 1
                self.circle_sub_step = 0
                self.control_window.after(self.circle_pause_time, self.execute_circle_step)
        else:
            # All 4 quarters complete
            self.send_kinematic_command(0, 0, 0)
            self.update_status("Circle: Complete!")
            self.circle_running = False
            self.circle_step = 0
            self.circle_sub_step = 0
    
    def send_kinematic_command(self, dot_x, dot_y, dot_theta):
        # PATCH 1B: Multi-robot - Pass robot_id to server
        if self.server and self.robot_id:
            self.server.send_kinematic_command(self.robot_id, dot_x, dot_y, dot_theta)
        elif self.server:
            # Fallback for backward compatibility
            self.server.send_kinematic_command(dot_x, dot_y, dot_theta)
        else:
            print("Server not connected!")
    
    def set_server(self, server):
        self.server = server
