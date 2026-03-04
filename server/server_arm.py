"""
Robot Arm Control GUI Module
Provides UI for sending IK coordinates, servo angles, and receiving computed angles
"""

import tkinter as tk
from tkinter import ttk, messagebox


class ArmControlGUI:
    """GUI class for controlling robot arm for a specific robot"""
    
    def __init__(self, parent, robot_id):
        self.parent = parent
        self.robot_id = robot_id
        self.server = None
        
        # UI elements storage
        self.coord_entries = {}  # x, y, z, pitch
        self.servo_entries = {}  # j0, j1, j2, j3, j4, j5
        self.received_labels = {}  # Labels to display received angles
        
    def set_server(self, server):
        """Set the server reference for communication"""
        self.server = server
        
    def setup_ui(self, parent_frame):
        """Setup the arm control UI within the given frame"""
        
        # Configure grid
        parent_frame.columnconfigure(0, weight=1)
        parent_frame.columnconfigure(1, weight=1)
        
        # === LEFT COLUMN: Send Controls ===
        left_frame = ttk.Frame(parent_frame)
        left_frame.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        
        # --- Coordinate Input Section ---
        coord_frame = ttk.LabelFrame(left_frame, text="Send IK Coordinates", padding=10)
        coord_frame.pack(fill="x", pady=5)
        
        coord_grid = ttk.Frame(coord_frame)
        coord_grid.pack(fill="x", pady=5)
        
        # X, Y, Z, Pitch entries
        coord_labels = ["X (mm):", "Y (mm):", "Z (mm):", "Pitch (°):"]
        coord_keys = ["x", "y", "z", "pitch"]
        default_values = ["150", "0", "100", "0"]
        
        for i, (label, key, default) in enumerate(zip(coord_labels, coord_keys, default_values)):
            ttk.Label(coord_grid, text=label).grid(row=i, column=0, padx=5, pady=3, sticky="w")
            var = tk.StringVar(value=default)
            entry = ttk.Entry(coord_grid, textvariable=var, width=12)
            entry.grid(row=i, column=1, padx=5, pady=3, sticky="ew")
            self.coord_entries[key] = entry
        
        # Send Coordinates button
        ttk.Button(coord_frame, text="Send Coordinates", 
                  command=self.send_coordinates).pack(pady=10)
        
        # --- Servo Direct Control Section ---
        servo_frame = ttk.LabelFrame(left_frame, text="Send Servo Angles (Verify)", padding=10)
        servo_frame.pack(fill="x", pady=5)
        
        servo_grid = ttk.Frame(servo_frame)
        servo_grid.pack(fill="x", pady=5)
        
        # J0-J5 entries
        for i in range(6):
            joint_name = f"J{i}"
            ttk.Label(servo_grid, text=f"{joint_name}:").grid(row=i // 3, column=(i % 3) * 2, padx=5, pady=3, sticky="w")
            var = tk.StringVar(value="90")
            entry = ttk.Entry(servo_grid, textvariable=var, width=8)
            entry.grid(row=i // 3, column=(i % 3) * 2 + 1, padx=5, pady=3, sticky="ew")
            self.servo_entries[f"j{i}"] = entry
        
        # Send Servo Angles button
        ttk.Button(servo_frame, text="Send Servo Angles", 
                  command=self.send_servo_angles).pack(pady=10)
        
        # --- ARM Commands Section ---
        cmd_frame = ttk.LabelFrame(left_frame, text="ARM Commands", padding=10)
        cmd_frame.pack(fill="x", pady=5)
        
        # Pick/Place coordinate inputs
        pick_grid = ttk.Frame(cmd_frame)
        pick_grid.pack(fill="x", pady=5)
        
        pick_labels = ["X (mm):", "Y (mm):", "Z (mm):"]
        pick_keys = ["cmd_x", "cmd_y", "cmd_z"]
        pick_defaults = ["100", "150", "-150"]
        
        self.cmd_entries = {}
        for i, (label, key, default) in enumerate(zip(pick_labels, pick_keys, pick_defaults)):
            ttk.Label(pick_grid, text=label).grid(row=0, column=i*2, padx=2, pady=3, sticky="w")
            var = tk.StringVar(value=default)
            entry = ttk.Entry(pick_grid, textvariable=var, width=8)
            entry.grid(row=0, column=i*2+1, padx=2, pady=3, sticky="ew")
            self.cmd_entries[key] = entry
        
        # Pick and Place buttons row
        pick_btn_row = ttk.Frame(cmd_frame)
        pick_btn_row.pack(fill="x", pady=5)
        
        ttk.Button(pick_btn_row, text="Pick", 
                  command=self.send_pick_command).pack(side="left", padx=5, expand=True, fill="x")
        ttk.Button(pick_btn_row, text="Place", 
                  command=self.send_place_command).pack(side="left", padx=5, expand=True, fill="x")
        
        # Gripper buttons row
        gripper_row = ttk.Frame(cmd_frame)
        gripper_row.pack(fill="x", pady=5)
        
        ttk.Label(gripper_row, text="Gripper:").pack(side="left", padx=5)
        ttk.Button(gripper_row, text="Open", 
                  command=lambda: self.send_gripper_command("open")).pack(side="left", padx=5, expand=True, fill="x")
        ttk.Button(gripper_row, text="Close", 
                  command=lambda: self.send_gripper_command("close")).pack(side="left", padx=5, expand=True, fill="x")
        
        # Rest button
        ttk.Button(cmd_frame, text="Rest Position", 
                  command=self.send_rest_command).pack(pady=10, fill="x")
        
        # === RIGHT COLUMN: Received Angles Display ===
        right_frame = ttk.Frame(parent_frame)
        right_frame.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
        
        # --- Received Angles Section ---
        received_frame = ttk.LabelFrame(right_frame, text="Received IK Result", padding=10)
        received_frame.pack(fill="x", pady=5)
        
        received_grid = ttk.Frame(received_frame)
        received_grid.pack(fill="x", pady=5)
        
        # Labels for J0-J5 received values
        for i in range(6):
            joint_name = f"J{i}"
            row = i // 2
            col = (i % 2) * 2
            
            ttk.Label(received_grid, text=f"{joint_name}:").grid(row=row, column=col, padx=5, pady=5, sticky="w")
            
            # Create a styled label for displaying the value
            value_label = tk.Label(received_grid, text="---.--°", 
                                  bg="#2b2b2b", fg="#00ff00",
                                  font=("Consolas", 12, "bold"),
                                  width=10, relief="sunken", anchor="center")
            value_label.grid(row=row, column=col + 1, padx=5, pady=5, sticky="ew")
            self.received_labels[f"j{i}"] = value_label
        
        # Status information
        status_frame = ttk.LabelFrame(right_frame, text="Status", padding=10)
        status_frame.pack(fill="x", pady=5)
        
        self.status_label = ttk.Label(status_frame, text="Ready", foreground="gray")
        self.status_label.pack(anchor="w")
        
        # Help text
        help_frame = ttk.LabelFrame(right_frame, text="Info", padding=10)
        help_frame.pack(fill="x", pady=5)
        
        help_text = """• Send Coordinates: Gửi X, Y, Z, Pitch để 
  client tính IK và trả về các góc servo

• Send Servo Angles: Gửi trực tiếp góc 
  servo J0-J5 (dùng để verify)

• Received IK Result: Hiển thị góc servo 
  nhận được từ client sau khi tính IK"""
        
        ttk.Label(help_frame, text=help_text, justify="left").pack(anchor="w")
    
    def send_coordinates(self):
        """Send X, Y, Z, Pitch coordinates to client for IK calculation"""
        try:
            x = float(self.coord_entries["x"].get())
            y = float(self.coord_entries["y"].get())
            z = float(self.coord_entries["z"].get())
            pitch = float(self.coord_entries["pitch"].get())
            
            if self.server:
                self.server.send_arm_coordinates(self.robot_id, x, y, z, pitch)
                self.update_status(f"Sent: X={x}, Y={y}, Z={z}, Pitch={pitch}")
            else:
                messagebox.showwarning("Not Connected", "Server not available")
                
        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter valid numbers for coordinates")
    
    def send_servo_angles(self):
        """Send direct servo angles to client"""
        try:
            angles = {}
            for i in range(6):
                key = f"j{i}"
                angles[key] = float(self.servo_entries[key].get())
                
                # Validate range
                if not (0 <= angles[key] <= 180):
                    messagebox.showerror("Invalid Angle", 
                                        f"J{i} must be between 0 and 180 degrees")
                    return
            
            if self.server:
                self.server.send_arm_servo_angles(self.robot_id, angles)
                self.update_status(f"Sent servo angles: {[f'{v:.1f}' for v in angles.values()]}")
            else:
                messagebox.showwarning("Not Connected", "Server not available")
                
        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter valid numbers for angles")
    
    def update_received_angles(self, angles):
        """Update the display with received angles from client
        
        Args:
            angles: dict with keys j0, j1, j2, j3, j4, j5 containing angle values
        """
        for i in range(6):
            key = f"j{i}"
            if key in angles:
                value = angles[key]
                self.received_labels[key].config(text=f"{value:.2f}°")
        
        self.update_status("IK result received")
    
    def update_status(self, message):
        """Update the status label"""
        if hasattr(self, 'status_label'):
            self.status_label.config(text=message)
    
    def clear_received_angles(self):
        """Clear all received angle displays"""
        for i in range(6):
            key = f"j{i}"
            self.received_labels[key].config(text="---.--°")
    
    def send_pick_command(self):
        """Send Pick command with x, y, z coordinates"""
        try:
            x = float(self.cmd_entries["cmd_x"].get())
            y = float(self.cmd_entries["cmd_y"].get())
            z = float(self.cmd_entries["cmd_z"].get())
            
            if self.server:
                self.server.send_arm_pick(self.robot_id, x, y, z)
                self.update_status(f"Pick: X={x}, Y={y}, Z={z}")
            else:
                messagebox.showwarning("Not Connected", "Server not available")
                
        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter valid numbers for coordinates")
    
    def send_place_command(self):
        """Send Place command with x, y, z coordinates"""
        try:
            x = float(self.cmd_entries["cmd_x"].get())
            y = float(self.cmd_entries["cmd_y"].get())
            z = float(self.cmd_entries["cmd_z"].get())
            
            if self.server:
                self.server.send_arm_place(self.robot_id, x, y, z)
                self.update_status(f"Place: X={x}, Y={y}, Z={z}")
            else:
                messagebox.showwarning("Not Connected", "Server not available")
                
        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter valid numbers for coordinates")
    
    def send_gripper_command(self, action):
        """Send Gripper open/close command
        
        Args:
            action: "open" or "close"
        """
        if self.server:
            self.server.send_arm_gripper(self.robot_id, action)
            self.update_status(f"Gripper: {action}")
        else:
            messagebox.showwarning("Not Connected", "Server not available")
    
    def send_rest_command(self):
        """Send Rest command to return arm to rest position"""
        if self.server:
            self.server.send_arm_rest(self.robot_id)
            self.update_status("Rest position command sent")
        else:
            messagebox.showwarning("Not Connected", "Server not available")
