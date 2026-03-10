from tkinter import filedialog, ttk, messagebox
from tkinter import scrolledtext
import tkinter as tk
from server_multi import Server
from server_control_multi import ControlGUI
from server_arm import ArmControlGUI
from server_trajection_multi import trajectory_visualizer
from server_rpm_plot import get_rpm_plotter
class ServerGUI:
    def __init__(self, root):
        self.root = root
        self.server = Server(self)
        
        # Multi-robot UI elements - indexed by robot_id
        self.rpm_labels = {}      # {robot_id: [label1, label2, label3, label4]}
        self.speed_entries = {}   # {robot_id: [entry1, entry2, entry3, entry4]}
        self.pid_entries = {}     # {robot_id: [[(p, i, d)], ...]}
        self.status_labels = {}   # {robot_id: label}
        self.calib_indicators = {}  # {robot_id: label}
        self.heading_labels = {}  # {robot_id: label}
        self.connect_buttons = {}  # {robot_id: button}
        self.disconnect_buttons = {}  # {robot_id: button}
        self.reset_esp_buttons = {}
        self.firmware_labels = {}  # {robot_id: label} - CHANGED: per-robot firmware labels
        
        # Control GUI instances per robot
        self.control_guis = {}
        for robot_id in [1, 2, 3]:
            control_gui = ControlGUI(root, robot_id)  # PATCH 1A: Pass robot_id to ControlGUI
            control_gui.set_server(self.server)
            self.control_guis[robot_id] = control_gui
        
        # Arm Control GUI instances per robot
        self.arm_guis = {}
        for robot_id in [1, 2, 3]:
            arm_gui = ArmControlGUI(root, robot_id)
            arm_gui.set_server(self.server)
            self.arm_guis[robot_id] = arm_gui
        
        # Approach phase UI variables
        self.approach_vars = {}
        self.manual_pos_entries = {}
        self.arrival_indicators = {}
        
        self.setup_gui()
        self.initialize_plots()
    # THÊM HÀM MỚI NÀY VÀO CLASS ServerGUI
    def initialize_plots(self):
        """Khởi tạo tất cả các đối tượng Matplotlib trong luồng chính."""
        try:
            for robot_id in [1, 2, 3]:
                # Lấy (hoặc tạo) plotter
                plotter = get_rpm_plotter(robot_id)
                # Gọi hàm initialize_figure
                plotter.initialize_figure()
            self.update_monitor("Khởi tạo 3 đồ thị RPM thành công.")
        except Exception as e:
            self.update_monitor(f"LỖI: Không thể khởi tạo đồ thị Matplotlib: {e}")
    def setup_gui(self):
        self.root.title("Multi-Robot Omni Server Control")
        self.root.geometry("1200x900")
        
        # Configure styles
        style = ttk.Style()
        
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill="both", expand=True)
        
        # Create main notebook with tabs
        self.main_notebook = ttk.Notebook(main_frame)
        self.main_notebook.pack(fill="both", expand=True)
        
        # Tab 1: Global Dashboard
        global_tab = ttk.Frame(self.main_notebook, padding=10)
        self.main_notebook.add(global_tab, text="Global Dashboard")
        
        # Tab 2: Robot Control (with nested tabs)
        control_tab = ttk.Frame(self.main_notebook, padding=10)
        self.main_notebook.add(control_tab, text="Robot Control")
        
        # Tab 2.5: Robot Arm Control
        arm_tab = ttk.Frame(self.main_notebook, padding=10)
        self.main_notebook.add(arm_tab, text="Robot Arm")
        
        # Tab 3: Connection Management
        connection_tab = ttk.Frame(self.main_notebook, padding=10)
        self.main_notebook.add(connection_tab, text="Connection")
        
        # Tab 4: Settings
        settings_tab = ttk.Frame(self.main_notebook, padding=10)
        self.main_notebook.add(settings_tab, text="Settings")
        
        # Setup each tab (Approach controls now integrated into Global Dashboard)
        self._setup_global_dashboard(global_tab)
        self._setup_robot_control_tab(control_tab)
        self._setup_arm_control_tab(arm_tab)
        self._setup_connection_tab(connection_tab)
        self._setup_settings_tab(settings_tab)
        
        # Monitor panel at bottom
        self._setup_monitor_panel(main_frame)

    def _setup_global_dashboard(self, parent):
        """Setup the global dashboard with approach phase controls and popup map"""
        # Title bar
        title_frame = ttk.Frame(parent)
        title_frame.pack(fill="x", pady=(0, 10))
        
        ttk.Label(title_frame, text="Global Dashboard - Approach Phase Control", 
                 font=("Arial", 14, "bold")).pack(side="left")
        
        # Buttons - right side
        ttk.Button(title_frame, text="🧪 Test Mode", 
                  command=self._open_test_mode).pack(side="right", padx=5)
        ttk.Button(title_frame, text="🗺️ Open Map", 
                  command=self._open_map_popup).pack(side="right", padx=5)
        ttk.Button(title_frame, text="Emergency Stop All", 
                  command=lambda: self.server.emergency_stop()).pack(side="right", padx=5)
        
        # Main content - split into left (status) and right (controls)
        main_paned = ttk.PanedWindow(parent, orient=tk.HORIZONTAL)
        main_paned.pack(fill=tk.BOTH, expand=True)
        
        # Left panel - Robot Status (EKF, BNO055, etc for LIVE monitoring)
        left_panel = ttk.Frame(main_paned, width=280)
        main_paned.add(left_panel, weight=1)
        trajectory_visualizer.parent_frame = parent
        trajectory_visualizer._create_status_frames(left_panel)
        trajectory_visualizer.is_active = True
        
        # Right panel - Approach Phase Controls
        right_panel = ttk.Frame(main_paned)
        main_paned.add(right_panel, weight=3)
        
        # Create scrollable frame for controls
        main_canvas = tk.Canvas(right_panel)
        scrollbar = ttk.Scrollbar(right_panel, orient="vertical", command=main_canvas.yview)
        scrollable_frame = ttk.Frame(main_canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: main_canvas.configure(scrollregion=main_canvas.bbox("all"))
        )
        
        main_canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        main_canvas.configure(yscrollcommand=scrollbar.set)
        
        main_canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        # ========== OBJECT POSITION ==========
        obj_frame = ttk.LabelFrame(scrollable_frame, text="Object Position & Dimensions", padding=5)
        obj_frame.pack(fill="x", padx=5, pady=5)
        
        obj_grid = ttk.Frame(obj_frame)
        obj_grid.pack(fill="x")
        
        ttk.Label(obj_grid, text="X:").grid(row=0, column=0, padx=3)
        self.approach_vars['obj_x'] = tk.StringVar(value="3.6")
        ttk.Entry(obj_grid, textvariable=self.approach_vars['obj_x'], width=8).grid(row=0, column=1, padx=2)
        
        ttk.Label(obj_grid, text="Y:").grid(row=0, column=2, padx=3)
        self.approach_vars['obj_y'] = tk.StringVar(value="3.18")
        ttk.Entry(obj_grid, textvariable=self.approach_vars['obj_y'], width=8).grid(row=0, column=3, padx=2)
        
        ttk.Label(obj_grid, text="Length:").grid(row=0, column=4, padx=3)
        self.approach_vars['obj_length'] = tk.StringVar(value="0.2")
        ttk.Entry(obj_grid, textvariable=self.approach_vars['obj_length'], width=6).grid(row=0, column=5, padx=2)
        
        ttk.Label(obj_grid, text="Width:").grid(row=0, column=6, padx=3)
        self.approach_vars['obj_width'] = tk.StringVar(value="0.1")
        ttk.Entry(obj_grid, textvariable=self.approach_vars['obj_width'], width=6).grid(row=0, column=7, padx=2)
        
        ttk.Button(obj_grid, text="Set", command=self._set_object_position).grid(row=0, column=8, padx=5)
        
        # ========== ROBOT CONFIGURATION ==========
        config_frame = ttk.LabelFrame(scrollable_frame, text="Robot Configuration", padding=5)
        config_frame.pack(fill="x", padx=5, pady=5)
        
        config_row = ttk.Frame(config_frame)
        config_row.pack(fill="x")
        
        ttk.Label(config_row, text="Number of Robots:").pack(side="left", padx=3)
        self.approach_vars['num_robots'] = tk.StringVar(value="1")
        num_combo = ttk.Combobox(config_row, textvariable=self.approach_vars['num_robots'],
                                 values=["1", "2", "3"], state="readonly", width=3)
        num_combo.pack(side="left", padx=3)
        num_combo.bind("<<ComboboxSelected>>", lambda e: self._update_num_robots())
        
        ttk.Label(config_row, text="Gripper Length:").pack(side="left", padx=10)
        self.approach_vars['gripper_length'] = tk.StringVar(value="0.05")
        ttk.Entry(config_row, textvariable=self.approach_vars['gripper_length'], width=6).pack(side="left", padx=3)
        ttk.Label(config_row, text="m (default 5cm)").pack(side="left")

        
        ttk.Button(config_row, text="Apply", command=self._apply_config).pack(side="left", padx=10)
        
        # ========== OBSTACLES ==========
        obs_frame = ttk.LabelFrame(scrollable_frame, text="Obstacles", padding=5)
        obs_frame.pack(fill="x", padx=5, pady=5)
        
        obs_row = ttk.Frame(obs_frame)
        obs_row.pack(fill="x")
        
        ttk.Label(obs_row, text="X:").pack(side="left")
        self.approach_vars['obs_x'] = tk.StringVar(value="0.95")
        ttk.Entry(obs_row, textvariable=self.approach_vars['obs_x'], width=6).pack(side="left", padx=2)
        
        ttk.Label(obs_row, text="Y:").pack(side="left", padx=(5, 0))
        self.approach_vars['obs_y'] = tk.StringVar(value="2.98")
        ttk.Entry(obs_row, textvariable=self.approach_vars['obs_y'], width=6).pack(side="left", padx=2)
        
        ttk.Label(obs_row, text="R:").pack(side="left", padx=(5, 0))
        self.approach_vars['obs_r'] = tk.StringVar(value="0.3")
        ttk.Entry(obs_row, textvariable=self.approach_vars['obs_r'], width=5).pack(side="left", padx=2)
        
        ttk.Button(obs_row, text="Add", command=self._add_obstacle).pack(side="left", padx=5)
        ttk.Button(obs_row, text="Clear", command=self._clear_obstacles).pack(side="left", padx=3)
        
        # ========== ACTIONS (Production - requires connected robots) ==========
        action_frame = ttk.LabelFrame(scrollable_frame, text="Actions (Production)", padding=5)
        action_frame.pack(fill="x", padx=5, pady=5)
        
        self.approach_vars['use_vf'] = tk.BooleanVar(value=True)
        ttk.Checkbutton(action_frame, text="Use Vector Field (obstacle avoidance)",
                       variable=self.approach_vars['use_vf']).pack(anchor="w")
        
        action_row = ttk.Frame(action_frame)
        action_row.pack(fill="x", pady=5)
        
        start_btn = ttk.Button(action_row, text="START APPROACH PHASE", 
                              command=self._start_approach_phase)
        start_btn.pack(side="left", padx=5, expand=True, fill="x")
        self.approach_start_btn = start_btn
        
        ttk.Button(action_row, text="ABORT", 
                  command=self._abort_approach).pack(side="left", padx=5, expand=True, fill="x")
        
        # ========== ARRIVAL STATUS ==========
        status_frame = ttk.LabelFrame(scrollable_frame, text="Arrival Status", padding=5)
        status_frame.pack(fill="x", padx=5, pady=5)
        
        status_row = ttk.Frame(status_frame)
        status_row.pack(fill="x")
        
        for robot_id in [1, 2, 3]:
            ttk.Label(status_row, text=f"Robot {robot_id}:").pack(side="left", padx=5)
            indicator = tk.Label(status_row, text="Wait", bg="#9E9E9E", fg="white", width=8)
            indicator.pack(side="left", padx=2)
            self.arrival_indicators[robot_id] = indicator
        
        self.phase2_ready_label = ttk.Label(status_frame, text="", font=("Arial", 10, "bold"))
        self.phase2_ready_label.pack(pady=3)

        # ========== PHASE 2: TRANSPORT CONTROLS ==========
        transport_frame = ttk.LabelFrame(scrollable_frame, text="Phase 2: Transport", padding=5)
        transport_frame.pack(fill="x", padx=5, pady=5)
        
        # Destination input row
        dest_row = ttk.Frame(transport_frame)
        dest_row.pack(fill="x", pady=3)
        
        ttk.Label(dest_row, text="Destination:").pack(side="left", padx=3)
        ttk.Label(dest_row, text="X:").pack(side="left", padx=3)
        self.approach_vars['dest_x'] = tk.StringVar(value="2.0")
        ttk.Entry(dest_row, textvariable=self.approach_vars['dest_x'], width=8).pack(side="left", padx=2)
        
        ttk.Label(dest_row, text="Y:").pack(side="left", padx=3)
        self.approach_vars['dest_y'] = tk.StringVar(value="4.0")
        ttk.Entry(dest_row, textvariable=self.approach_vars['dest_y'], width=8).pack(side="left", padx=2)
        
        ttk.Button(dest_row, text="Set", command=self._set_destination).pack(side="left", padx=5)
        
        # Phase 2 action row
        transport_action_row = ttk.Frame(transport_frame)
        transport_action_row.pack(fill="x", pady=5)
        
        self.phase2_start_btn = ttk.Button(transport_action_row, text="START PHASE 2", 
                                           command=self._start_transport_phase, state="disabled")
        self.phase2_start_btn.pack(side="left", padx=5, expand=True, fill="x")
        
        ttk.Button(transport_action_row, text="ABORT PHASE 2", 
                  command=self._abort_transport).pack(side="left", padx=5, expand=True, fill="x")
        
        # Phase 2 status indicator
        self.phase2_status_label = ttk.Label(transport_frame, text="Status: Waiting for Phase 1 completion", 
                                             foreground="gray")
        self.phase2_status_label.pack(pady=3)


    def _setup_robot_control_tab(self, parent):
        """Setup the robot control tab with nested tabs for 3 robots"""
        # Create nested notebook for 3 robots
        self.robot_notebook = ttk.Notebook(parent)
        self.robot_notebook.pack(fill="both", expand=True)
        
        # Create a tab for each robot
        for robot_id in [1, 2, 3]:
            robot_frame = ttk.Frame(self.robot_notebook, padding=10)
            self.robot_notebook.add(robot_frame, text=f"Robot {robot_id}")
            self._setup_robot_control_content(robot_frame, robot_id)

    def _setup_arm_control_tab(self, parent):
        """Setup the robot arm control tab with nested tabs for 3 robots"""
        # Create nested notebook for 3 robots
        self.arm_notebook = ttk.Notebook(parent)
        self.arm_notebook.pack(fill="both", expand=True)
        
        # Create a tab for each robot's arm control
        for robot_id in [1, 2, 3]:
            arm_frame = ttk.Frame(self.arm_notebook, padding=10)
            self.arm_notebook.add(arm_frame, text=f"Robot {robot_id}")
            # Setup arm GUI in this frame
            if robot_id in self.arm_guis:
                self.arm_guis[robot_id].setup_ui(arm_frame)

    def _setup_robot_control_content(self, parent, robot_id):
        """Setup the control content for a specific robot (BỐ CỤC ĐÃ SẮP XẾP LẠI)"""
        
        # --- BỐ CỤC MỚI (DÙNG GRID 2 CỘT) ---
        
        parent.rowconfigure(0, weight=0) # Hàng 0: Status (không co giãn)
        parent.rowconfigure(1, weight=1) # Hàng 1: Nội dung chính (co giãn)
        
        # Cột Trái (weight=1) hẹp hơn, Cột Phải (weight=2) rộng hơn
        parent.columnconfigure(0, weight=1) # Cột Trái (Tuning)
        parent.columnconfigure(1, weight=2) # Cột Phải (Actions)

        # Hàng 0: Status bar (chạy dài 2 cột)
        status_frame = ttk.Frame(parent, relief="sunken", padding="2")
        status_frame.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 10))
        
        ttk.Label(status_frame, text=f"Robot {robot_id} Status:").pack(side="left")
        status_label = ttk.Label(status_frame, text="Disconnected")
        status_label.pack(side="left", padx=(5, 0))
        self.status_labels[robot_id] = status_label
        
        # --- CỘT TRÁI (column=0) ---
        left_column_frame = ttk.Frame(parent)
        left_column_frame.grid(row=1, column=0, sticky="nsew", padx=(0, 5))

        # Motor control frame
        motor_frame = ttk.LabelFrame(left_column_frame, text="Motor Control", padding=10)
        motor_frame.pack(fill="x", pady=5)
        
        motor_grid = ttk.Frame(motor_frame)
        motor_grid.pack(fill="x", pady=5)

        motor_grid.columnconfigure(0, weight=0) # Label "Motor X:"
        motor_grid.columnconfigure(1, weight=0) # Label RPM (KHÔNG co giãn)
        motor_grid.columnconfigure(2, weight=0) # Label "Set Speed:"
        motor_grid.columnconfigure(3, weight=0) # Entry Speed (KHÔNG co giãn)
        motor_grid.columnconfigure(4, weight=0) # Button "Set"
        
        rpm_labels = []
        speed_entries = []
        
        for i in range(4):
            ttk.Label(motor_grid, text=f"Motor {i+1}:").grid(row=i, column=0, padx=(5, 2), pady=5, sticky="w")
            rpm_label = ttk.Label(motor_grid, text="0.00 RPM", width=12, relief="sunken", anchor="e")
            rpm_label.grid(row=i, column=1, padx=(0, 10), pady=5, sticky="ew")
            rpm_labels.append(rpm_label)
            ttk.Label(motor_grid, text=f"Set Speed:").grid(row=i, column=2, padx=(10, 2), pady=5, sticky="w")
            speed_var = tk.StringVar(value="0")
            speed_entry = ttk.Entry(motor_grid, textvariable=speed_var, width=10)
            speed_entry.grid(row=i, column=3, padx=(0, 5), pady=5, sticky="ew")
            speed_entries.append(speed_entry)
            ttk.Button(motor_grid, text="Set", 
                      command=lambda idx=i, rid=robot_id, se=speed_entry: self.set_motor_speed(rid, idx, se)).grid(
                          row=i, column=4, padx=5, pady=5)
        
        self.rpm_labels[robot_id] = rpm_labels
        self.speed_entries[robot_id] = speed_entries

        # PID control frame
        pid_frame = ttk.LabelFrame(left_column_frame, text="PID Settings", padding=10)
        pid_frame.pack(fill="x", pady=5)

        # PID Auto tuning frame
        pid_auto_frame = ttk.Frame(pid_frame)
        pid_auto_frame.pack(fill="x", pady=(0, 5))
        
        ttk.Label(pid_auto_frame, text="PID Auto:").pack(side="left", padx=5)
        ttk.Label(pid_auto_frame, text="RPM:").pack(side="left", padx=(10, 2))
        rpm_var = tk.StringVar(value="70")
        rpm_entry = ttk.Entry(pid_auto_frame, textvariable=rpm_var, width=8)
        rpm_entry.pack(side="left", padx=2)
        
        ttk.Label(pid_auto_frame, text="PWM:").pack(side="left", padx=(5, 2))
        pwm_var = tk.StringVar(value="100")
        pwm_entry = ttk.Entry(pid_auto_frame, textvariable=pwm_var, width=8)
        pwm_entry.pack(side="left", padx=2)
        
        ttk.Button(pid_auto_frame, text="Start Auto Tune", 
                  command=lambda rid=robot_id, rv=rpm_var, pv=pwm_var: self.send_pid_auto(rid, rv.get(), pv.get())).pack(side="left", padx=5)

        pid_button_frame = ttk.Frame(pid_frame)
        pid_button_frame.pack(fill="x", pady=5)
        ttk.Button(pid_button_frame, text="PID Auto", 
                  command=lambda rid=robot_id: self.server.send_command_to_robot(rid, "pid_auto")).pack(side="left", padx=5)
        ttk.Button(pid_button_frame, text="Save PID Config", 
                  command=lambda: self.server.save_pid_config(robot_id)).pack(side="left", padx=5)
        ttk.Button(pid_button_frame, text="Load PID Config", 
                  command=lambda: self.server.load_pid_config(robot_id)).pack(side="left", padx=5)
        
        pid_entries_robot = []
        for i in range(4):
            motor_frame_pid = ttk.Frame(pid_frame) 
            motor_frame_pid.pack(fill="x", pady=2)
            ttk.Label(motor_frame_pid, text=f"Motor {i+1}:").pack(side="left", padx=5)
            p_var = tk.StringVar(value="1.0")
            i_var = tk.StringVar(value="0.0")
            d_var = tk.StringVar(value="0.0")
            ttk.Label(motor_frame_pid, text="P:").pack(side="left", padx=2)
            p_entry = ttk.Entry(motor_frame_pid, textvariable=p_var, width=8)
            p_entry.pack(side="left", padx=2)
            ttk.Label(motor_frame_pid, text="I:").pack(side="left", padx=2)
            i_entry = ttk.Entry(motor_frame_pid, textvariable=i_var, width=8)
            i_entry.pack(side="left", padx=2)
            ttk.Label(motor_frame_pid, text="D:").pack(side="left", padx=2)
            d_entry = ttk.Entry(motor_frame_pid, textvariable=d_var, width=8)
            d_entry.pack(side="left", padx=2)
            ttk.Button(motor_frame_pid, text="Set", 
                      command=lambda idx=i, rid=robot_id: self.set_pid(rid, idx)).pack(side="left", padx=5)
            pid_entries_robot.append((p_entry, i_entry, d_entry))
        self.pid_entries[robot_id] = pid_entries_robot
        


        # --- CỘT PHẢI (column=1) ---
        right_column_frame = ttk.Frame(parent)
        right_column_frame.grid(row=1, column=1, sticky="nsew", padx=(5, 0))

        # Control buttons
        control_frame = ttk.LabelFrame(right_column_frame, text="Robot Control", padding=10)
        control_frame.pack(fill="x", pady=5)
        
        # Configure grid columns for control_frame
        for i in range(4):
            control_frame.columnconfigure(i, weight=1)
        
        # Row 0: Basic control buttons
        ttk.Button(control_frame, text="Manual Control", 
                  command=lambda: self.manual_control(robot_id), 
                  state="normal").grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        ttk.Button(control_frame, text="Emergency Stop", 
                  command=lambda: self.server.emergency_stop(robot_id), 
                  style="Red.TButton").grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        reset_btn = ttk.Button(control_frame, text="Reset ESP", 
                               command=lambda: self.server.send_command_to_robot(robot_id, "reset"), 
                               state="disabled")
        reset_btn.grid(row=0, column=2, padx=5, pady=5, sticky="ew")
        self.reset_esp_buttons[robot_id] = reset_btn # <-- Dòng này lưu lại nút
        ttk.Button(control_frame, text="Show RPM Plot", 
                  command=lambda: self.server.show_rpm_plot(robot_id)).grid(row=0, column=3, padx=5, pady=5, sticky="ew")
        
        # Row 1: Test trajectory buttons
        ttk.Button(control_frame, text="Run Square Test", 
                  command=lambda rid=robot_id: self.server.start_test_trajectory(rid, 'square')).grid(
                      row=1, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
        ttk.Button(control_frame, text="Run Circle Test", 
                  command=lambda rid=robot_id: self.server.start_test_trajectory(rid, 'circle')).grid(
                      row=1, column=2, columnspan=2, padx=5, pady=5, sticky="ew")

        # BNO055 sensor frame
        bno055_frame = ttk.LabelFrame(right_column_frame, text="BNO055 Sensor", padding=10)
        bno055_frame.pack(fill="x", pady=5)
        calib_frame = ttk.Frame(bno055_frame)
        calib_frame.pack(fill="x", pady=5)
        ttk.Label(calib_frame, text="Calibration Status:").pack(side="left", padx=5)
        calib_indicator = tk.Label(calib_frame, text="Not Calibrated", 
                                   bg="#F44336", fg="white", 
                                   width=15, relief="flat")
        calib_indicator.pack(side="left", padx=10)
        self.calib_indicators[robot_id] = calib_indicator
        
        heading_frame = ttk.Frame(bno055_frame)
        heading_frame.pack(fill="x", pady=5)
        ttk.Label(heading_frame, text="Heading:").pack(side="left", padx=5)
        heading_label = tk.Label(heading_frame, text="0.0°", 
                                bg="#FFD700", fg="black", 
                                width=10, relief="sunken", font=("Arial", 12, "bold"))
        heading_label.pack(side="left", padx=10)
        self.heading_labels[robot_id] = heading_label

        # THAY ĐỔI 3: Sắp xếp Firmware Update nằm ngang theo thứ tự mới
        firmware_frame = ttk.LabelFrame(right_column_frame, text="Firmware Update", padding=10)
        firmware_frame.pack(fill="x", pady=5)
        
        # Cấu hình grid cho firmware_frame
        firmware_frame.columnconfigure(0, weight=0) # Button (Upgrade)
        firmware_frame.columnconfigure(1, weight=0) # Button (Send)
        firmware_frame.columnconfigure(2, weight=0) # Button (Choose File)
        firmware_frame.columnconfigure(3, weight=1) # Label (File path - co giãn)
        
        # Hàng 0:
        
        # CỘT 0: Nút Upgrade Mode
        upgrade_btn = ttk.Button(firmware_frame, text="Upgrade Mode",
                                 command=lambda rid=robot_id: self.server.send_upgrade_command(rid),
                                 state="disabled")
        upgrade_btn.grid(row=0, column=0, padx=5, pady=5, sticky="w")
        
        # CỘT 1: Nút Send Firmware
        send_fw_btn = ttk.Button(firmware_frame, text="Send Firmware",
                                 command=lambda rid=robot_id: self.server.send_firmware(rid),
                                 state="disabled")
        send_fw_btn.grid(row=0, column=1, padx=5, pady=5, sticky="w")
        
        # CỘT 2: Nút Choose File
        ttk.Button(firmware_frame, text="Choose File",
                  command=lambda rid=robot_id: self.choose_file_for_robot(rid)).grid(row=0, column=2, padx=(10, 5), pady=5, sticky="w")
        
        # CỘT 3: Label File (co giãn)
        fw_label = ttk.Label(firmware_frame, text="No file selected", width=20, anchor="w") 
        fw_label.grid(row=0, column=3, padx=5, pady=5, sticky="ew")
        
        if not hasattr(self, 'firmware_labels'):
            self.firmware_labels = {}
        self.firmware_labels[robot_id] = fw_label
        
        # (Lưu các nút)
        if not hasattr(self, 'send_fw_buttons'):
            self.send_fw_buttons = {}
        if not hasattr(self, 'upgrade_buttons'):
            self.upgrade_buttons = {}
        self.send_fw_buttons[robot_id] = send_fw_btn
        self.upgrade_buttons[robot_id] = upgrade_btn

    def _setup_connection_tab(self, parent):
        """Setup the connection management tab for all 3 robots"""
        # Read connection profiles
        self.connection_profiles = self.load_connection_profiles()
        
        # Create connection controls for each robot
        for robot_id in [1, 2, 3]:
            robot_frame = ttk.LabelFrame(parent, text=f"Robot {robot_id} Connection", padding=10)
            robot_frame.pack(fill="x", pady=5)
            
            ttk.Label(robot_frame, text="Profile:").grid(row=0, column=0, padx=5, pady=5)
            
            profile_names = list(self.connection_profiles.keys())
            default_profile_name = f"Robot {robot_id}"
            if default_profile_name not in profile_names:
                default_profile_name = profile_names[0] if profile_names else ""
            selected_profile = tk.StringVar(value=default_profile_name)
            profile_combo = ttk.Combobox(robot_frame, textvariable=selected_profile, 
                                        values=profile_names, state="readonly", width=15)
            profile_combo.grid(row=0, column=1, padx=5, pady=5)
            
            connect_btn = ttk.Button(robot_frame, text="Connect",
                                    command=lambda rid=robot_id, pvar=selected_profile: self.connect_to_profile(rid, pvar.get()))
            connect_btn.grid(row=0, column=2, padx=5, pady=5)
            self.connect_buttons[robot_id] = connect_btn
            
            disconnect_btn = ttk.Button(robot_frame, text="Disconnect",
                                       command=lambda rid=robot_id: self.server.disconnect_from_robot(rid),
                                       state="disabled")
            disconnect_btn.grid(row=0, column=3, padx=5, pady=5)
            self.disconnect_buttons[robot_id] = disconnect_btn
            
            # Manual connection fields
            ttk.Label(robot_frame, text="Host:").grid(row=1, column=0, padx=5, pady=5)
            host_var = tk.StringVar(value="192.168.1.211")
            host_entry = ttk.Entry(robot_frame, textvariable=host_var, width=20)
            host_entry.grid(row=1, column=1, padx=5, pady=5)
            
            ttk.Label(robot_frame, text="Port:").grid(row=1, column=2, padx=5, pady=5)
            port_var = tk.StringVar(value="2004")
            port_entry = ttk.Entry(robot_frame, textvariable=port_var, width=10)
            port_entry.grid(row=1, column=3, padx=5, pady=5)
            
            ttk.Button(robot_frame, text="Connect Manual",
                      command=lambda rid=robot_id, hv=host_var, pv=port_var: self.connect_manual(rid, hv.get(), pv.get())).grid(
                          row=1, column=4, padx=5, pady=5)
        
        # Disconnect all button
        ttk.Button(parent, text="Disconnect All Robots", 
                  command=self.server.disconnect_all_robots,
                  style="Red.TButton").pack(pady=10)

    def _setup_settings_tab(self, parent):
        """Setup the settings tab"""
        # Logging settings
        log_frame = ttk.LabelFrame(parent, text="Logging Settings", padding=10)
        log_frame.pack(fill="x", pady=5)
        
        self.log_enabled = tk.BooleanVar(value=True)
        ttk.Checkbutton(log_frame, text="Enable Data Logging", 
                       variable=self.log_enabled,
                       command=self.toggle_logging).pack(anchor="w", pady=5)
        
        # Firmware update
        firmware_frame = ttk.LabelFrame(parent, text="Firmware Management", padding=10)
        firmware_frame.pack(fill="x", pady=5)
        
        ttk.Button(firmware_frame, text="Choose Firmware File",
                  command=self.choose_file).pack(side="left", padx=5, pady=5)
        
        self.firmware_label = ttk.Label(firmware_frame, text="No file selected")
        self.firmware_label.pack(side="left", padx=5, pady=5)
        
        # Progress bar for firmware upload
        self.progress_var = tk.DoubleVar()
        self.progress_frame = ttk.Frame(parent)
        ttk.Label(self.progress_frame, text="Upload progress:").pack(side="left")
        self.progress_bar = ttk.Progressbar(self.progress_frame, 
                                           variable=self.progress_var,
                                           maximum=100, length=400)
        self.progress_bar.pack(side="left", padx=5)
    
    # ========== APPROACH PHASE UI CALLBACKS ==========
    
    def _set_object_position(self):
        """Set object position from UI inputs"""
        try:
            x = float(self.approach_vars['obj_x'].get())
            y = float(self.approach_vars['obj_y'].get())
            length = float(self.approach_vars['obj_length'].get())
            width = float(self.approach_vars['obj_width'].get())
            self.server.set_object_position(x, y, length, width)
        except ValueError:
            messagebox.showerror("Invalid Input", "Object position and dimension values must be numbers")
    
    def _update_num_robots(self):
        """Update number of robots"""
        num = int(self.approach_vars['num_robots'].get())
        self.server.set_num_robots(num)
    
    def _apply_config(self):
        """Apply robot configuration"""
        try:
            num = int(self.approach_vars['num_robots'].get())
            gripper_length = float(self.approach_vars['gripper_length'].get())
            self.server.set_num_robots(num)
            self.server.set_gripper_length(gripper_length)
        except ValueError:
            messagebox.showerror("Invalid Input", "Configuration values must be numbers")

    
    def _toggle_position_mode(self):
        """Toggle between manual and EKF positions"""
        use_manual = self.approach_vars['use_manual'].get()
        self.server.set_use_manual_positions(use_manual)
    
    def _apply_manual_positions(self):
        """Apply manual robot positions"""
        try:
            for robot_id, entries in self.manual_pos_entries.items():
                x = float(entries['x'].get())
                y = float(entries['y'].get())
                theta = float(entries['theta'].get())
                self.server.set_manual_robot_position(robot_id, x, y, theta)
        except ValueError:
            messagebox.showerror("Invalid Input", "Position values must be numbers")
    
    def _add_obstacle(self):
        """Add circular obstacle"""
        try:
            x = float(self.approach_vars['obs_x'].get())
            y = float(self.approach_vars['obs_y'].get())
            r = float(self.approach_vars['obs_r'].get())
            self.server.add_obstacle(x, y, r)
        except ValueError:
            messagebox.showerror("Invalid Input", "Obstacle values must be numbers")
    
    def _clear_obstacles(self):
        """Clear all obstacles"""
        self.server.clear_obstacles()
    
    def _compute_trajectories(self):
        """Compute approach trajectories"""
        use_vf = self.approach_vars['use_vf'].get()
        result = self.server.compute_approach_trajectories(use_vector_field=use_vf)
        if result is None:
            messagebox.showwarning("Planning Failed", 
                                 "Could not generate approach trajectories.\n"
                                 "Target might be unreachable or inside an obstacle.")
    
    def _start_approach_phase(self):
        """Start the approach phase"""
        use_vf = self.approach_vars['use_vf'].get()
        
        # Check position mode and apply manual positions if needed
        use_manual = self.approach_vars.get('use_manual', tk.BooleanVar(value=False)).get()
        if use_manual:
            self._apply_manual_positions()
            self.server.gui.update_monitor("📍 Position mode: MANUAL (Applied positions from GUI)")
        else:
            self.server.gui.update_monitor("📍 Position mode: EKF (Using robot positions)")
        
        # Log starting positions for debugging
        for rid in [1, 2, 3]:
            pos = self.server.get_robot_position(rid)
            if pos:
                self.server.gui.update_monitor(f"  Robot {rid} start: ({pos[0]:.3f}, {pos[1]:.3f})")
        
        # Reset arrival indicators
        for robot_id, indicator in self.arrival_indicators.items():
            indicator.config(text="In Progress", bg="#FFC107")
        self.phase2_ready_label.config(text="")
        
        if self.server.start_approach_phase(use_vector_field=use_vf) is False:
             messagebox.showwarning("Approach Failed", 
                                  "Failed to start approach phase.\n"
                                  "Check log for details.")
    
    def _abort_approach(self):
        """Abort the approach phase"""
        self.server.abort_approach_phase()
        
        # Reset indicators
        for indicator in self.arrival_indicators.values():
            indicator.config(text="Aborted", bg="#F44336")
        self.phase2_ready_label.config(text="")
    
    def update_arrival_status(self, robot_id, arrived):
        """Update arrival status indicator for a robot"""
        if robot_id in self.arrival_indicators:
            if arrived:
                self.arrival_indicators[robot_id].config(text="Arrived", bg="#4CAF50")
            else:
                self.arrival_indicators[robot_id].config(text="In Progress", bg="#FFC107")
        
        # Check if all arrived
        if self.server.check_all_arrived():
            self.phase2_ready_label.config(text="✓ ALL ARRIVED - READY FOR PHASE 2", 
                                          foreground="green")
            # Enable Phase 2 button
            if hasattr(self, 'phase2_start_btn'):
                self.phase2_start_btn.config(state="normal")
                self.phase2_status_label.config(text="Status: Ready to start", foreground="blue")
    
    # ========== PHASE 2: TRANSPORT CALLBACKS ==========
    
    def _set_destination(self):
        """Set destination position for Phase 2 transport"""
        try:
            x = float(self.approach_vars['dest_x'].get())
            y = float(self.approach_vars['dest_y'].get())
            self.server.set_destination_position(x, y)
        except ValueError:
            messagebox.showerror("Invalid Input", "Destination values must be numbers")
    
    def _start_transport_phase(self):
        """Start Phase 2: Transport the object to destination"""
        # First set destination
        self._set_destination()
        
        # Update status
        self.phase2_status_label.config(text="Status: Computing trajectory...", foreground="orange")
        self.phase2_start_btn.config(state="disabled")
        
        # Start transport phase
        if self.server.start_transport_phase():
            self.phase2_status_label.config(text="Status: Transport in progress", foreground="green")
        else:
            self.phase2_status_label.config(text="Status: Failed to start", foreground="red")
            self.phase2_start_btn.config(state="normal")
            messagebox.showwarning("Transport Failed", 
                                  "Failed to start transport phase.\n"
                                  "Check log for details.")
    
    def _abort_transport(self):
        """Abort Phase 2 transport"""
        self.server.abort_transport_phase()
        self.phase2_status_label.config(text="Status: Aborted", foreground="red")
        self.phase2_start_btn.config(state="normal")
    
    def update_phase2_status(self, status, completed=False):
        """Update Phase 2 status from server"""
        if completed:
            self.phase2_status_label.config(text="Status: Transport complete!", foreground="green")
            self.phase2_start_btn.config(state="normal")
        else:
            self.phase2_status_label.config(text=f"Status: {status}", foreground="blue")
    
    def _open_map_popup(self):
        """Open the trajectory map in a separate popup window"""
        trajectory_visualizer.open_popup_window()
    
    def _open_test_mode(self):
        """Open the Test Mode popup window for trajectory testing without connected robots"""
        # Check if window already exists
        if hasattr(self, 'test_window') and self.test_window:
            try:
                self.test_window.lift()
                self.test_window.focus_force()
                return
            except tk.TclError:
                pass
        
        # Create popup window
        self.test_window = tk.Toplevel()
        self.test_window.title("🧪 Test Mode - Trajectory Planning")
        self.test_window.geometry("500x600")
        self.test_window.minsize(400, 500)
        
        main_frame = ttk.Frame(self.test_window, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        ttk.Label(main_frame, text="Test Mode - Manual Position Testing", 
                 font=("Arial", 12, "bold")).pack(pady=(0, 10))
        
        ttk.Label(main_frame, text="Test trajectory computation WITHOUT connected robots",
                 foreground="gray").pack()
        
        ttk.Separator(main_frame, orient="horizontal").pack(fill="x", pady=10)
        
        # Manual Positions Frame
        pos_frame = ttk.LabelFrame(main_frame, text="Manual Robot Positions", padding=10)
        pos_frame.pack(fill="x", pady=5)
        
        # Initialize manual_pos_entries if needed
        if not hasattr(self, 'manual_pos_entries') or not self.manual_pos_entries:
            self.manual_pos_entries = {}
        
        for robot_id in [1, 2, 3]:
            row = ttk.Frame(pos_frame)
            row.pack(fill="x", pady=3)
            
            ttk.Label(row, text=f"Robot {robot_id}:", width=10).pack(side="left")
            
            # Distinct start positions for testing
            default_x = {1: "1.55", 2: "2.0", 3: "2.5"}.get(robot_id, "1.55")
            default_y = {1: "1.78", 2: "1.78", 3: "1.78"}.get(robot_id, "1.78")
            
            x_var = tk.StringVar(value=default_x)
            ttk.Label(row, text="X:").pack(side="left", padx=3)
            ttk.Entry(row, textvariable=x_var, width=8).pack(side="left")
            
            y_var = tk.StringVar(value=default_y)
            ttk.Label(row, text="Y:").pack(side="left", padx=(10, 3))
            ttk.Entry(row, textvariable=y_var, width=8).pack(side="left")
            
            theta_var = tk.StringVar(value="0.0")
            ttk.Label(row, text="θ:").pack(side="left", padx=(10, 3))
            ttk.Entry(row, textvariable=theta_var, width=6).pack(side="left")
            
            self.manual_pos_entries[robot_id] = {'x': x_var, 'y': y_var, 'theta': theta_var}
        
        # Initialize approach_vars if needed
        if 'use_manual' not in self.approach_vars:
            self.approach_vars['use_manual'] = tk.BooleanVar(value=True)
        
        # Actions Frame
        action_frame = ttk.LabelFrame(main_frame, text="Test Actions", padding=10)
        action_frame.pack(fill="x", pady=10)
        
        btn_row = ttk.Frame(action_frame)
        btn_row.pack(fill="x", pady=5)
        
        ttk.Button(btn_row, text="Apply Positions", 
                  command=self._test_apply_positions).pack(side="left", padx=5)
        ttk.Button(btn_row, text="Compute Trajectories", 
                  command=self._test_compute_trajectories).pack(side="left", padx=5)
        ttk.Button(btn_row, text="Open Map", 
                  command=self._open_map_popup).pack(side="left", padx=5)
        
        # Output Frame
        output_frame = ttk.LabelFrame(main_frame, text="Output (Trajectory JSON)", padding=10)
        output_frame.pack(fill="both", expand=True, pady=5)
        
        self.test_output = scrolledtext.ScrolledText(output_frame, height=10, width=50)
        self.test_output.pack(fill="both", expand=True)
        
        # ========== PHASE 2 TESTING SECTION ==========
        ttk.Separator(main_frame, orient="horizontal").pack(fill="x", pady=10)
        
        phase2_frame = ttk.LabelFrame(main_frame, text="Phase 2 Testing", padding=10)
        phase2_frame.pack(fill="x", pady=5)
        
        ttk.Label(phase2_frame, text="Simulate Phase 1 completion to test Phase 2:",
                 foreground="gray").pack(anchor="w")
        
        phase2_btn_row = ttk.Frame(phase2_frame)
        phase2_btn_row.pack(fill="x", pady=5)
        
        ttk.Button(phase2_btn_row, text="🔧 Simulate Phase 1 Complete", 
                  command=self._test_simulate_phase1_complete).pack(side="left", padx=5)
        
        ttk.Button(phase2_btn_row, text="🚀 Test Phase 2 Transport", 
                  command=self._test_start_phase2).pack(side="left", padx=5)
        
        # Handle close
        self.test_window.protocol("WM_DELETE_WINDOW", self._on_test_close)
    
    def _test_apply_positions(self):
        """Apply manual positions from test mode"""
        try:
            self.server.set_use_manual_positions(True)
            for robot_id, entries in self.manual_pos_entries.items():
                x = float(entries['x'].get())
                y = float(entries['y'].get())
                theta = float(entries['theta'].get())
                self.server.set_manual_robot_position(robot_id, x, y, theta)
            self.test_output.insert(tk.END, "✓ Manual positions applied\n")
        except ValueError as e:
            self.test_output.insert(tk.END, f"✗ Error: {e}\n")
    
    def _test_compute_trajectories(self):
        """Compute trajectories in test mode (without sending to robots)"""
        import json
        import traceback
        
        # First apply positions
        self._test_apply_positions()
        
        try:
            # Compute trajectories
            use_vf = self.approach_vars.get('use_vf', tk.BooleanVar(value=True)).get()
            self.test_output.insert(tk.END, f"\nComputing trajectories (vector_field={use_vf})...\n")
            
            trajectories = self.server.compute_approach_trajectories(use_vector_field=use_vf)
            
            if trajectories:
                self.test_output.insert(tk.END, "\n=== Computed Trajectories ===\n")
                for robot_id, traj in trajectories.items():
                    self.test_output.insert(tk.END, f"\nRobot {robot_id}: {len(traj)} waypoints\n")
                    # Show first 3 and last 1 points
                    if len(traj) > 4:
                        for wp in traj[:3]:
                            self.test_output.insert(tk.END, f"  {json.dumps(wp)}\n")
                        self.test_output.insert(tk.END, f"  ... ({len(traj)-4} more points)\n")
                        self.test_output.insert(tk.END, f"  {json.dumps(traj[-1])}\n")
                    else:
                        for wp in traj:
                            self.test_output.insert(tk.END, f"  {json.dumps(wp)}\n")
                self.test_output.insert(tk.END, "\n✓ Trajectories displayed on map\n")
            else:
                self.test_output.insert(tk.END, "\n✗ Failed to generate trajectories (Unreachable)\n")
                messagebox.showwarning("Planning Failed", "Targets are unreachable!")
                self.test_output.insert(tk.END, "✗ Failed to compute trajectories (returned None)\n")
                self.test_output.insert(tk.END, "Check System Monitor for error details\n")
        except Exception as e:
            self.test_output.insert(tk.END, f"\n✗ Exception: {e}\n")
            self.test_output.insert(tk.END, traceback.format_exc())
        
        self.test_output.see(tk.END)
    
    def _on_test_close(self):
        """Handle test window close"""
        if hasattr(self, 'test_window') and self.test_window:
            self.test_window.destroy()
            self.test_window = None
    
    def _test_simulate_phase1_complete(self):
        """
        Simulate Phase 1 completion for testing Phase 2.
        
        This sets up the server state as if robots have arrived at grip positions
        without actually connecting to real robots.
        """
        import json
        
        # First apply positions and compute trajectories if not done
        self._test_apply_positions()
        
        try:
            # Get number of robots from config
            num_robots = int(self.approach_vars.get('num_robots', tk.StringVar(value='1')).get())
            
            # Set object position if not set
            if self.server.object_position is None:
                try:
                    obj_x = float(self.approach_vars['obj_x'].get())
                    obj_y = float(self.approach_vars['obj_y'].get())
                    obj_size = float(self.approach_vars['obj_size'].get())
                    self.server.set_object_position(obj_x, obj_y, obj_size, obj_size)
                except:
                    self.server.set_object_position(2.0, 4.0, 0.2, 0.2)
            
            # Compute grip positions
            self.server.formation_planner.set_num_robots(num_robots)
            # Use robot IDs from the planner's robot_angles (supports 1, 2, 3 robots correctly)
            active_robots = list(self.server.formation_planner.robot_angles.keys())
            self.server.formation_planner.set_active_robots(active_robots)
            
            grip_positions = self.server.formation_planner.compute_grip_positions(
                self.server.object_position,
                self.server.object_length,
                self.server.object_width
            )
            self.server.grip_positions = grip_positions
            
            # Set robot positions to grip positions (simulating arrival)
            for robot_id, grip_pos in grip_positions.items():
                self.server.robot_positions[robot_id] = (grip_pos[0], grip_pos[1], 0.0)
                self.server.manual_positions[robot_id] = (grip_pos[0], grip_pos[1], 0.0)
            
            # Mark all robots as arrived via approach_manager (proper delegation)
            self.server.approach_manager.arrived_status = {rid: True for rid in active_robots}
            self.server.approach_manager.approach_phase_active = False
            
            # Update GUI arrival indicators
            for robot_id in active_robots:
                if robot_id in self.arrival_indicators:
                    self.arrival_indicators[robot_id].config(text="Arrived", bg="#4CAF50")
            
            # Enable Phase 2 button
            self.phase2_ready_label.config(text="✓ ALL ARRIVED - READY FOR PHASE 2", 
                                          foreground="green")
            self.phase2_start_btn.config(state="normal")
            self.phase2_status_label.config(text="Status: Ready to start", foreground="blue")
            
            # Update visualization
            from server_trajection_multi import trajectory_visualizer
            trajectory_visualizer.set_grip_positions(grip_positions)
            
            # Log output
            self.test_output.insert(tk.END, "\n" + "="*50 + "\n")
            self.test_output.insert(tk.END, "✓ PHASE 1 SIMULATED COMPLETE\n")
            self.test_output.insert(tk.END, "="*50 + "\n")
            self.test_output.insert(tk.END, f"Object position: {self.server.object_position}\n")
            self.test_output.insert(tk.END, f"Active robots: {active_robots}\n")
            self.test_output.insert(tk.END, f"Grip positions:\n")
            for rid, pos in grip_positions.items():
                self.test_output.insert(tk.END, f"  Robot {rid}: ({pos[0]:.3f}, {pos[1]:.3f})\n")
            self.test_output.insert(tk.END, "\n→ You can now set destination and click 'START PHASE 2'\n")
            self.test_output.insert(tk.END, "  Or click 'Test Phase 2 Transport' to compute trajectory\n")
            self.test_output.see(tk.END)
            
        except Exception as e:
            self.test_output.insert(tk.END, f"\n✗ Error: {e}\n")
            import traceback
            self.test_output.insert(tk.END, traceback.format_exc())
            self.test_output.see(tk.END)
    
    def _test_start_phase2(self):
        """
        Test Phase 2 transport trajectory computation without sending to robots.
        """
        import json
        
        # Get destination
        try:
            dest_x = float(self.approach_vars['dest_x'].get())
            dest_y = float(self.approach_vars['dest_y'].get())
            self.server.set_destination_position(dest_x, dest_y)
        except ValueError:
            self.test_output.insert(tk.END, "✗ Invalid destination values\n")
            return
        
        self.test_output.insert(tk.END, f"\nComputing Phase 2 transport trajectory...\n")
        self.test_output.insert(tk.END, f"Destination: ({dest_x:.2f}, {dest_y:.2f})\n")
        
        try:
            # Check if Phase 1 is simulated
            if not self.server.approach_manager.arrived_status or not all(self.server.approach_manager.arrived_status.values()):
                self.test_output.insert(tk.END, "✗ Phase 1 not complete. Click 'Simulate Phase 1 Complete' first.\n")
                self.test_output.see(tk.END)
                return
            
            # Compute transport trajectory
            trajectory = self.server.compute_transport_trajectories()
            
            if trajectory:
                self.test_output.insert(tk.END, f"\n=== Centroid Trajectory ({len(trajectory)} waypoints) ===\n")
                
                # Show first 3 and last 1 points
                if len(trajectory) > 4:
                    for wp in trajectory[:3]:
                        self.test_output.insert(tk.END, f"  {json.dumps(wp)}\n")
                    self.test_output.insert(tk.END, f"  ... ({len(trajectory)-4} more points)\n")
                    self.test_output.insert(tk.END, f"  {json.dumps(trajectory[-1])}\n")
                else:
                    for wp in trajectory:
                        self.test_output.insert(tk.END, f"  {json.dumps(wp)}\n")
                
                # Show formation offsets
                arrived_robots = [rid for rid, arr in self.server.approach_manager.arrived_status.items() if arr]
                self.server.formation_planner.set_active_robots(arrived_robots)
                offsets = self.server.formation_planner.compute_formation_offsets()
                
                self.test_output.insert(tk.END, f"\n=== Formation Offsets ===\n")
                for rid, offset in offsets.items():
                    self.test_output.insert(tk.END, f"  Robot {rid}: [{offset[0]:.3f}, {offset[1]:.3f}]\n")
                
                self.test_output.insert(tk.END, "\n✓ Trajectory displayed on map (magenta line)\n")
                
                # Update visualization
                from server_trajection_multi import trajectory_visualizer
                path_points = [(p['x'], p['y']) for p in trajectory]
                trajectory_visualizer.set_centroid_path(path_points)
            else:
                self.test_output.insert(tk.END, "✗ Failed to compute transport trajectory\n")
                self.test_output.insert(tk.END, "Check if destination is reachable\n")
        
        except Exception as e:
            self.test_output.insert(tk.END, f"\n✗ Exception: {e}\n")
            import traceback
            self.test_output.insert(tk.END, traceback.format_exc())
        
        self.test_output.see(tk.END)

    def _setup_monitor_panel(self, parent):
        """Setup the monitor/log panel at the bottom"""
        monitor_frame = ttk.LabelFrame(parent, text="System Monitor", padding=5)
        monitor_frame.pack(fill="both", expand=True, pady=(10, 0))
        
        # Scrolled text for logs
        self.monitor_text = scrolledtext.ScrolledText(monitor_frame, height=10, wrap=tk.WORD)
        self.monitor_text.pack(fill="both", expand=True)
        
        # Monitor control buttons
        button_frame = ttk.Frame(monitor_frame)
        button_frame.pack(fill="x", pady=(5, 0))
        
        ttk.Button(button_frame, text="Clear Monitor", 
                  command=self.clear_monitor).pack(side="left", padx=5)

    # Connection methods
    def load_connection_profiles(self):
        """Load connection profiles from config file"""
        profiles = {}
        try:
            with open("connection_config.txt", "r") as f:
                for line in f:
                    line = line.strip()
                    if line and not line.startswith("#"):
                        parts = line.split(":")
                        if len(parts) == 3:
                            name = parts[0].strip()
                            host = parts[1].strip()
                            port = int(parts[2].strip())
                            profiles[name] = (host, port)
        except FileNotFoundError:
            # Create default config
            profiles = {
                "Robot1_Local": ("192.168.1.101", 8080),
                "Robot2_Local": ("192.168.1.102", 8080),
                "Robot3_Local": ("192.168.1.103", 8080)
            }
        return profiles

    def connect_to_profile(self, robot_id, profile_name):
        """Connect to a robot using a profile"""
        if profile_name in self.connection_profiles:
            host, port = self.connection_profiles[profile_name]
            if self.server.connect_to_robot(robot_id, host, port):
                self.connect_buttons[robot_id].config(state="disabled")
                self.disconnect_buttons[robot_id].config(state="normal")

    def connect_manual(self, robot_id, host, port):
        """Connect to a robot manually"""
        try:
            port_int = int(port)
            if self.server.connect_to_robot(robot_id, host, port_int):
                self.connect_buttons[robot_id].config(state="disabled")
                self.disconnect_buttons[robot_id].config(state="normal")
        except ValueError:
            messagebox.showerror("Invalid Port", "Port must be a number")

    # UI Update methods (now with robot_id parameter)
    def update_status(self, robot_id, status):
        """Update connection status for a specific robot"""
        if robot_id in self.status_labels:
            self.status_labels[robot_id].config(text=status)

    def enable_control_buttons(self, robot_id):
        """Enable control buttons for a specific robot"""
        # SỬA Ở ĐÂY: Thêm logic kích hoạt các nút
        if robot_id in self.upgrade_buttons:
            self.upgrade_buttons[robot_id].config(state="normal")
        
        if robot_id in self.send_fw_buttons:
            self.send_fw_buttons[robot_id].config(state="normal")
        
        if robot_id in self.reset_esp_buttons:
            self.reset_esp_buttons[robot_id].config(state="normal")

    def disable_control_buttons(self, robot_id):
        """Disable control buttons for a specific robot"""
        if robot_id in self.connect_buttons:
            self.connect_buttons[robot_id].config(state="normal")
        if robot_id in self.disconnect_buttons:
            self.disconnect_buttons[robot_id].config(state="disabled")

        if robot_id in self.reset_esp_buttons:
            self.reset_esp_buttons[robot_id].config(state="disabled")
        if robot_id in self.upgrade_buttons:
            self.upgrade_buttons[robot_id].config(state="disabled")
        if robot_id in self.send_fw_buttons:
            self.send_fw_buttons[robot_id].config(state="disabled")

    def update_encoders(self, robot_id, encoders):
        """Update RPM display for a specific robot"""
        if robot_id in self.rpm_labels:
            for i, rpm_label in enumerate(self.rpm_labels[robot_id]):
                rpm_label.config(text=f"{encoders[i]:.2f} RPM")

    def update_heading(self, robot_id, heading_value):
        """Update heading display for a specific robot"""
        if robot_id in self.heading_labels:
            self.heading_labels[robot_id].config(text=f"{heading_value:.1f}°")

    def update_calibration_status(self, robot_id, is_calibrated):
        """Update calibration status for a specific robot"""
        if robot_id in self.calib_indicators:
            if is_calibrated:
                self.calib_indicators[robot_id].config(text="Calibrated", bg="#4CAF50")
            else:
                self.calib_indicators[robot_id].config(text="Not Calibrated", bg="#F44336")

    def update_monitor(self, message):
        """Update the system monitor with a message"""
        self.monitor_text.insert(tk.END, f"{message}\n")
        self.monitor_text.see(tk.END)

    def clear_monitor(self):
        """Clear the monitor text"""
        self.monitor_text.delete(1.0, tk.END)

    # Control methods
    def manual_control(self, robot_id):
        """Open manual control window for a specific robot"""
        if robot_id in self.control_guis:
            self.control_guis[robot_id].run()

    def set_motor_speed(self, robot_id, motor_index, entry):
        """Set motor speed for a specific robot"""
        try:
            speed = float(entry.get())
            self.server.set_speed(robot_id, motor_index, speed)
        except ValueError:
            messagebox.showerror("Invalid Speed", "Speed must be a number")

    def set_pid(self, robot_id, motor_index):
        """Set PID values for a specific motor on a specific robot"""
        try:
            p_entry, i_entry, d_entry = self.pid_entries[robot_id][motor_index]
            p = float(p_entry.get())
            i = float(i_entry.get())
            d = float(d_entry.get())
            self.server.set_pid_values(robot_id, motor_index, p, i, d)
            self.update_monitor(f"Robot {robot_id}, Motor {motor_index}: PID set to P={p}, I={i}, D={d}")
        except ValueError:
            messagebox.showerror("Invalid PID", "PID values must be numbers")

    def update_pid_entries(self, robot_id, motor_index, p, i, d):
        """Update PID entry fields for a specific robot"""
        if robot_id in self.pid_entries:
            p_entry, i_entry, d_entry = self.pid_entries[robot_id][motor_index]
            p_entry.delete(0, tk.END)
            p_entry.insert(0, str(p))
            i_entry.delete(0, tk.END)
            i_entry.insert(0, str(i))
            d_entry.delete(0, tk.END)
            d_entry.insert(0, str(d))

    def send_pid_auto(self, robot_id, rpm_str, pwm_str):
        """Send PID auto tune command with RPM and PWM parameters"""
        try:
            rpm = float(rpm_str)
            pwm = float(pwm_str)
            command = f"RPM:{rpm:.2f} PWM:{pwm:.2f}"
            self.server.send_command_to_robot(robot_id, command)
            self.update_monitor(f"Robot {robot_id}: PID Auto tune started with RPM={rpm:.2f}, PWM={pwm:.2f}")
        except ValueError:
            messagebox.showerror("Invalid Input", "RPM and PWM must be valid numbers")

    # Settings methods
    def toggle_logging(self):
        """Toggle data logging on/off"""
        self.server.log_data = self.log_enabled.get()
        status = "enabled" if self.server.log_data else "disabled"
        self.update_monitor(f"Data logging {status}")

    def choose_file(self):
        """Choose a firmware file"""
        file_path = filedialog.askopenfilename(
            title="Select Firmware File",
            filetypes=[("Binary files", "*.bin"), ("All files", "*.*")]
        )
        if file_path:
            self.server.file_path = file_path
            self.firmware_label.config(text=f"Selected: {file_path.split('/')[-1]}")
            self.update_monitor(f"Firmware file selected: {file_path}")
    
    def choose_file_for_robot(self, robot_id):
        """Choose a firmware file for a specific robot"""
        file_path = filedialog.askopenfilename(
            title=f"Select Firmware File for Robot {robot_id}",
            filetypes=[("Binary files", "*.bin"), ("All files", "*.*")]
        )
        if file_path:
            self.server.file_paths[robot_id] = file_path
            if robot_id in self.firmware_labels:
                filename = file_path.split('/')[-1]
                if len(filename) > 25:
                    filename = filename[:22] + "..."
                self.firmware_labels[robot_id].config(text=filename)
            self.update_monitor(f"Robot {robot_id}: Firmware file selected: {file_path}")

    def setup_progress_bar(self, total_size):
        """Setup progress bar for firmware upload"""
        self.progress_var.set(0)
        self.progress_frame.pack(fill="x", pady=5)

    def update_progress(self, value):
        """Update progress bar value"""
        self.progress_var.set(value)
        self.root.update_idletasks()

    def hide_progress_bar(self):
        """Hide the progress bar"""
        self.progress_frame.pack_forget()
