"""
Interactive Workspace Debugger for Robot Arm
Real-time visualization and IK/FK testing

Usage:
    python workspace_visualizer.py
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import TextBox, Button

# Import robot parameters from config.py (single source of truth)
from config import (
    d1, a2, a3, d5,
    SERVO_MAPPING_CONFIG,
    PICK_PLACE_CONFIG
)

# ===== Robot Parameters (from config.py) =====
USE_GRIPPER = True  # Set to True when using gripper

ARM_D1 = d1          # Base height
ARM_A2 = a2          # Link 1 length
ARM_A3 = a3          # Link 2 length
GRIPPER_LENGTH = d5  # Physical gripper length (from config)
ARM_D5 = GRIPPER_LENGTH if USE_GRIPPER else 0.0  # Tool length for IK

# Height compensation from server config
LIFT_HEIGHT = PICK_PLACE_CONFIG.get("LIFT_HEIGHT", 10)  # mm offset for pick/place
DEFAULT_PITCH = PICK_PLACE_CONFIG.get("PITCH", -90)  # Default gripper pitch

# TCP workspace visualization (shows potential reach when gripper attached)
# Only displayed when USE_GRIPPER=False (calibrating wrist mode)
SHOW_TCP_WORKSPACE = True  # Toggle TCP workspace visualization

# ===== Servo Mapping Config (from config.py) =====
SERVO_CONFIG = [
    {"offset": SERVO_MAPPING_CONFIG["j0"]["offset"], "dir": SERVO_MAPPING_CONFIG["j0"]["dir"]},
    {"offset": SERVO_MAPPING_CONFIG["j1"]["offset"], "dir": SERVO_MAPPING_CONFIG["j1"]["dir"]},
    {"offset": SERVO_MAPPING_CONFIG["j2"]["offset"], "dir": SERVO_MAPPING_CONFIG["j2"]["dir"]},
    {"offset": SERVO_MAPPING_CONFIG["j3"]["offset"], "dir": SERVO_MAPPING_CONFIG["j3"]["dir"]},
    {"offset": SERVO_MAPPING_CONFIG["j4"]["offset"], "dir": SERVO_MAPPING_CONFIG["j4"]["dir"]},
    {"offset": SERVO_MAPPING_CONFIG["j5"]["offset"], "dir": SERVO_MAPPING_CONFIG["j5"]["dir"]},
]

# ===== Gravity Compensation Config =====
GRAVITY_GAINS = {
    1: 5.0,  # J1 - Shoulder
    2: 3.0,  # J2 - Elbow
    3: 2.0,  # J3 - Wrist Pitch
}


def map_math_to_servo(joint_idx, math_angle):
    if joint_idx < 0 or joint_idx > 5:
        return 90.0
    cfg = SERVO_CONFIG[joint_idx]
    servo_angle = cfg["offset"] + (math_angle * cfg["dir"])
    return np.clip(servo_angle, 0.0, 180.0)


def unmap_servo_to_math(joint_idx, servo_angle):
    if joint_idx < 0 or joint_idx > 5:
        return 0.0
    cfg = SERVO_CONFIG[joint_idx]
    return (servo_angle - cfg["offset"]) / cfg["dir"]


def apply_gravity_compensation(ch, servo_angle, global_angle):
    """Apply gravity compensation based on global angle (angle from horizontal floor)
    
    Args:
        ch: Joint channel (1, 2, or 3)
        servo_angle: Current servo angle (0-180)
        global_angle: Link angle from horizontal floor (from FK)
        
    Returns:
        Compensated servo angle (0-180)
    
    Physics:
        - When link is horizontal (global=0°), gravity has max effect → max compensation
        - When link is vertical (global=±90°), gravity has no effect → no compensation
    """
    gain = GRAVITY_GAINS.get(ch, 0.0)
    if gain == 0.0:
        return servo_angle
    
    # Use global angle (from horizontal floor) for compensation
    global_rad = np.radians(global_angle)
    compensation = gain * np.cos(global_rad)
    
    compensated = servo_angle + compensation
    return np.clip(compensated, 0.0, 180.0)


def inverse_kinematics_exact(target_r, target_z, target_phi):
    phi_rad = np.radians(target_phi)
    wrist_r = target_r - ARM_D5 * np.cos(phi_rad)
    wrist_z = target_z - ARM_D5 * np.sin(phi_rad)
    r_rel = wrist_r
    z_rel = wrist_z - ARM_D1
    dist_sq = r_rel**2 + z_rel**2
    dist = np.sqrt(dist_sq)
    max_reach = ARM_A2 + ARM_A3
    min_reach = abs(ARM_A2 - ARM_A3)
    
    debug = {
        "wrist_r": wrist_r, "wrist_z": wrist_z,
        "dist": dist, "max_reach": max_reach, "min_reach": min_reach,
    }
    
    if dist > max_reach:
        debug["fail_reason"] = f"OUT OF REACH: dist={dist:.1f} > max={max_reach:.1f}"
        return False, None, debug
    if dist < min_reach:
        debug["fail_reason"] = f"TOO CLOSE: dist={dist:.1f} < min={min_reach:.1f}"
        return False, None, debug
    
    cos_q2 = (dist_sq - ARM_A2**2 - ARM_A3**2) / (2 * ARM_A2 * ARM_A3)
    cos_q2 = np.clip(cos_q2, -1.0, 1.0)
    q2_rad = np.arccos(cos_q2)
    
    alpha = np.arctan2(z_rel, r_rel)
    beta = np.arctan2(ARM_A3 * np.sin(q2_rad), ARM_A2 + ARM_A3 * np.cos(q2_rad))
    q1_polar = alpha + beta
    
    math_q1 = np.degrees(q1_polar) - 90.0
    math_q2 = np.degrees(q2_rad)
    math_q3 = (math_q1 + 90.0) - math_q2 - target_phi
    
    sv1 = map_math_to_servo(1, math_q1)
    sv2 = map_math_to_servo(2, math_q2)
    sv3 = map_math_to_servo(3, math_q3)
    
    debug.update({"math_q1": math_q1, "math_q2": math_q2, "math_q3": math_q3})
    
    if not (0 <= sv1 <= 180 and 0 <= sv2 <= 180 and 0 <= sv3 <= 180):
        debug["fail_reason"] = f"SERVO LIMITS: J1={sv1:.1f}, J2={sv2:.1f}, J3={sv3:.1f}"
        return False, None, debug
    
    return True, {"j1": sv1, "j2": sv2, "j3": sv3}, debug


def inverse_kinematics_full(x, y, z, phi_deg):
    theta0_rad = np.arctan2(y, x)
    theta0_deg = np.degrees(theta0_rad)
    
    debug = {"theta0_deg": theta0_deg, "x": x, "y": y, "z": z, "phi": phi_deg}
    
    if theta0_deg < 0.0 or theta0_deg > 180.0:
        debug["fail_reason"] = f"J0 OUT OF RANGE: {theta0_deg:.1f} deg"
        return False, None, debug
    
    sv0 = map_math_to_servo(0, theta0_deg)
    r = np.sqrt(x**2 + y**2)
    debug["r"] = r
    
    success, angles, ik_debug = inverse_kinematics_exact(r, z, phi_deg)
    debug.update(ik_debug)
    
    if success:
        angles["j0"] = sv0
        
        # VERIFICATION: Use FK to check if result actually reaches target
        fk_result, _ = forward_kinematics(sv0, angles["j1"], angles["j2"], angles["j3"])
        actual_x, actual_y, actual_z = fk_result["x"], fk_result["y"], fk_result["z"]
        actual_phi = fk_result["phi"]
        
        # Calculate position error
        pos_error = np.sqrt((actual_x - x)**2 + (actual_y - y)**2 + (actual_z - z)**2)
        phi_error = abs(actual_phi - phi_deg)
        
        debug["fk_verify"] = {
            "actual_x": actual_x, "actual_y": actual_y, "actual_z": actual_z,
            "actual_phi": actual_phi, "pos_error": pos_error, "phi_error": phi_error
        }
        
        # Tolerance: 1mm for position, 1 degree for phi
        if pos_error > 1.0:
            debug["fail_reason"] = f"POSITION MISMATCH: error={pos_error:.1f}mm (target ({x:.1f},{y:.1f},{z:.1f}) vs actual ({actual_x:.1f},{actual_y:.1f},{actual_z:.1f}))"
            return False, None, debug
        if phi_error > 1.0:
            debug["fail_reason"] = f"PHI MISMATCH: target={phi_deg:.1f}° vs actual={actual_phi:.1f}° (error={phi_error:.1f}°)"
            return False, None, debug
        
        return True, angles, debug
    
    return False, None, debug


def forward_kinematics(j0_servo, j1_servo, j2_servo, j3_servo):
    theta0_deg = unmap_servo_to_math(0, j0_servo)
    theta0_rad = np.radians(theta0_deg)
    
    m1 = unmap_servo_to_math(1, j1_servo)
    m2 = unmap_servo_to_math(2, j2_servo)
    m3 = unmap_servo_to_math(3, j3_servo)
    
    t1, t2, t3 = np.radians(m1), np.radians(m2), np.radians(m3)
    
    angle_1_plot = t1 + np.pi / 2.0
    p1_r = ARM_A2 * np.cos(angle_1_plot)
    p1_z = ARM_D1 + ARM_A2 * np.sin(angle_1_plot)
    
    angle_2_plot = angle_1_plot - t2
    p2_r = p1_r + ARM_A3 * np.cos(angle_2_plot)
    p2_z = p1_z + ARM_A3 * np.sin(angle_2_plot)
    
    angle_3_plot = angle_2_plot - t3
    p3_r = p2_r + ARM_D5 * np.cos(angle_3_plot)
    p3_z = p2_z + ARM_D5 * np.sin(angle_3_plot)
    
    r = p3_r
    z = p3_z
    x = r * np.cos(theta0_rad)
    y = r * np.sin(theta0_rad)
    phi = np.degrees(angle_3_plot)
    
    points_rz = [(0, 0), (0, ARM_D1), (p1_r, p1_z), (p2_r, p2_z), (p3_r, p3_z)]
    
    link_angles = {
        "link1_geo": np.degrees(angle_1_plot),
        "link2_geo": np.degrees(angle_2_plot),
        "link3_geo": np.degrees(angle_3_plot),
    }
    
    return {"x": x, "y": y, "z": z, "phi": phi, "r": r, "link_angles": link_angles}, points_rz


def convert_to_inclinometer(geo_angle_deg):
    """Convert geometric angle to inclinometer reading (-90 to +90)"""
    angle = geo_angle_deg % 360
    if angle > 180:
        angle -= 360
    if angle > 90:
        angle = 180 - angle
    elif angle < -90:
        angle = -180 - angle
    return angle


class InteractiveDebugger:
    def __init__(self):
        self.fig = plt.figure(figsize=(18, 10))
        self.fig.canvas.manager.set_window_title('Robot Arm Workspace Debugger')
        
        self.ax_rz = self.fig.add_axes([0.03, 0.35, 0.40, 0.60])
        self.ax_text = self.fig.add_axes([0.45, 0.35, 0.25, 0.60])
        self.ax_text.axis('off')
        self.ax_incl = self.fig.add_axes([0.72, 0.35, 0.26, 0.60])
        self.ax_incl.axis('off')
        
        # IK Mode state
        self.current_x = 100.0
        self.current_y = 0.0
        self.current_z = 100.0
        self.current_phi = float(DEFAULT_PITCH)  # Use config default pitch
        
        # FK Mode state
        self.current_j0 = 90.0
        self.current_j1 = 90.0
        self.current_j2 = 90.0
        self.current_j3 = 90.0
        
        self.mode = 'IK'
        
        self._create_widgets()
        self.update_visualization()
        
    def _create_widgets(self):
        # Row 1: IK inputs
        self.fig.text(0.02, 0.26, 'IK Mode:', fontsize=10, fontweight='bold')
        
        ax_x = self.fig.add_axes([0.10, 0.22, 0.10, 0.05])
        self.text_x = TextBox(ax_x, 'X:', initial=str(self.current_x))
        self.text_x.on_submit(lambda t: setattr(self, 'current_x', float(t)) if t else None)
        
        ax_y = self.fig.add_axes([0.25, 0.22, 0.10, 0.05])
        self.text_y = TextBox(ax_y, 'Y:', initial=str(self.current_y))
        self.text_y.on_submit(lambda t: setattr(self, 'current_y', float(t)) if t else None)
        
        ax_z = self.fig.add_axes([0.40, 0.22, 0.10, 0.05])
        self.text_z = TextBox(ax_z, 'Z:', initial=str(self.current_z))
        self.text_z.on_submit(lambda t: setattr(self, 'current_z', float(t)) if t else None)
        
        ax_phi = self.fig.add_axes([0.55, 0.22, 0.10, 0.05])
        self.text_phi = TextBox(ax_phi, 'Phi:', initial=str(int(self.current_phi)))
        self.text_phi.on_submit(lambda t: setattr(self, 'current_phi', float(t)) if t else None)
        
        ax_calc_ik = self.fig.add_axes([0.70, 0.22, 0.12, 0.05])
        self.btn_calc_ik = Button(ax_calc_ik, 'Calculate IK', color='lightgreen')
        self.btn_calc_ik.on_clicked(self._on_calc_ik)
        
        # Row 2: FK inputs
        self.fig.text(0.02, 0.18, 'FK Mode:', fontsize=10, fontweight='bold')
        
        ax_j0 = self.fig.add_axes([0.10, 0.14, 0.10, 0.05])
        self.text_j0 = TextBox(ax_j0, 'J0:', initial=str(self.current_j0))
        self.text_j0.on_submit(lambda t: setattr(self, 'current_j0', float(t)) if t else None)
        
        ax_j1 = self.fig.add_axes([0.25, 0.14, 0.10, 0.05])
        self.text_j1 = TextBox(ax_j1, 'J1:', initial=str(self.current_j1))
        self.text_j1.on_submit(lambda t: setattr(self, 'current_j1', float(t)) if t else None)
        
        ax_j2 = self.fig.add_axes([0.40, 0.14, 0.10, 0.05])
        self.text_j2 = TextBox(ax_j2, 'J2:', initial=str(self.current_j2))
        self.text_j2.on_submit(lambda t: setattr(self, 'current_j2', float(t)) if t else None)
        
        ax_j3 = self.fig.add_axes([0.55, 0.14, 0.10, 0.05])
        self.text_j3 = TextBox(ax_j3, 'J3:', initial=str(self.current_j3))
        self.text_j3.on_submit(lambda t: setattr(self, 'current_j3', float(t)) if t else None)
        
        ax_calc_fk = self.fig.add_axes([0.70, 0.14, 0.12, 0.05])
        self.btn_calc_fk = Button(ax_calc_fk, 'Calculate FK', color='lightcyan')
        self.btn_calc_fk.on_clicked(self._on_calc_fk)
        
        # Row 3: Presets - MUST store references to prevent garbage collection
        ax_p1 = self.fig.add_axes([0.10, 0.06, 0.12, 0.05])
        self.btn_home = Button(ax_p1, 'Home(90,90,90)', color='lightyellow')
        self.btn_home.on_clicked(lambda e: self._set_fk(90, 90, 90, 90))
        
        ax_p2 = self.fig.add_axes([0.25, 0.06, 0.12, 0.05])
        self.btn_forward = Button(ax_p2, 'Forward', color='lightyellow')
        self.btn_forward.on_clicked(lambda e: self._set_fk(90, 45, 90, 135))
        
        ax_p3 = self.fig.add_axes([0.40, 0.06, 0.12, 0.05])
        self.btn_up = Button(ax_p3, 'Up', color='lightyellow')
        self.btn_up.on_clicked(lambda e: self._set_fk(90, 0, 90, 180))
        
        ax_p4 = self.fig.add_axes([0.55, 0.06, 0.12, 0.05])
        self.btn_down = Button(ax_p4, 'Down', color='lightyellow')
        self.btn_down.on_clicked(lambda e: self._set_fk(90, 120, 60, 60))
        
        ax_p5 = self.fig.add_axes([0.70, 0.06, 0.12, 0.05])
        self.btn_stretch = Button(ax_p5, 'Stretch', color='lightyellow')
        self.btn_stretch.on_clicked(lambda e: self._set_fk(90, 90, 0, 90))
        
        self.fig.text(0.05, 0.01, 
            f"Robot: d1={ARM_D1}, a2={ARM_A2}, a3={ARM_A3}, d5={ARM_D5}mm | "
            f"Gripper: {GRIPPER_LENGTH}mm | "
            f"Lift: {LIFT_HEIGHT}mm | "
            f"Gravity: J1={GRAVITY_GAINS.get(1,0)}, J2={GRAVITY_GAINS.get(2,0)}, J3={GRAVITY_GAINS.get(3,0)}",
            fontsize=9, color='gray')
    
    def _on_calc_ik(self, event):
        self.mode = 'IK'
        self.update_visualization()
    
    def _on_calc_fk(self, event):
        self.mode = 'FK'
        self.update_visualization()
    
    def _set_ik(self, x, y, z, phi):
        self.current_x, self.current_y, self.current_z, self.current_phi = x, y, z, phi
        self.text_x.set_val(str(x))
        self.text_y.set_val(str(y))
        self.text_z.set_val(str(z))
        self.text_phi.set_val(str(phi))
        self.mode = 'IK'
        self.update_visualization()
    
    def _set_fk(self, j0, j1, j2, j3):
        self.current_j0, self.current_j1, self.current_j2, self.current_j3 = j0, j1, j2, j3
        self.text_j0.set_val(str(j0))
        self.text_j1.set_val(str(j1))
        self.text_j2.set_val(str(j2))
        self.text_j3.set_val(str(j3))
        self.mode = 'FK'
        self.update_visualization()
    
    def update_visualization(self):
        self.ax_rz.clear()
        self.ax_text.clear()
        self.ax_text.axis('off')
        self.ax_incl.clear()
        self.ax_incl.axis('off')
        
        self._draw_workspace()
        
        if self.mode == 'IK':
            text_lines, incl_lines = self._process_ik()
        else:
            text_lines, incl_lines = self._process_fk()
        
        self.ax_text.text(0, 1, '\n'.join(text_lines), transform=self.ax_text.transAxes,
                          fontsize=9, fontfamily='monospace', verticalalignment='top')
        
        if incl_lines:
            self.ax_incl.text(0, 1, '\n'.join(incl_lines), transform=self.ax_incl.transAxes,
                              fontsize=9, fontfamily='monospace', verticalalignment='top')
        
        self.ax_rz.set_xlabel('R (mm)')
        self.ax_rz.set_ylabel('Z (mm)')
        target_type = 'TCP' if USE_GRIPPER else 'Wrist'
        self.ax_rz.set_title(f'R-Z Plane ({self.mode} Mode) - Target: {target_type}')
        self.ax_rz.set_aspect('equal')
        self.ax_rz.grid(True, alpha=0.3)
        self.ax_rz.legend(loc='upper right', fontsize=8)
        # Expand limits to fit TCP workspace
        max_extent = ARM_A2 + ARM_A3 + GRIPPER_LENGTH + 50
        self.ax_rz.set_xlim(-100, max_extent)
        self.ax_rz.set_ylim(-150, max_extent)
        
        self.fig.canvas.draw_idle()
    
    def _draw_workspace(self):
        max_r, min_r = ARM_A2 + ARM_A3, abs(ARM_A2 - ARM_A3)
        theta = np.linspace(-np.pi/2, np.pi, 100)
        self.ax_rz.plot(max_r * np.cos(theta), ARM_D1 + max_r * np.sin(theta),
                        'r--', alpha=0.4, linewidth=1, label=f'Max ({max_r}mm)')
        self.ax_rz.plot(min_r * np.cos(theta), ARM_D1 + min_r * np.sin(theta),
                        'orange', linestyle='--', alpha=0.4, linewidth=1, label=f'Min ({min_r}mm)')
        self.ax_rz.plot([0, 0], [0, ARM_D1], 'k-', linewidth=4)
        self.ax_rz.axhline(y=0, color='brown', linewidth=2, label='Ground')
    
    def _draw_tcp_workspace(self, wrist_r, wrist_z, link2_geo_angle):
        """Draw TCP workspace arc at wrist position showing reachable TCP area.
        
        ONLY shown when USE_GRIPPER=False (calibrating wrist position).
        Shows where TCP would go if gripper is attached.
        
        The J3 servo range is 0-180, so from the current link2 angle,
        TCP can sweep a range determined by J3's limits.
        
        Args:
            wrist_r, wrist_z: Wrist position in R-Z plane
            link2_geo_angle: Geometric angle of link2 (degrees from horizontal)
        """
        # Only show TCP workspace when calibrating wrist (USE_GRIPPER=False)
        if not SHOW_TCP_WORKSPACE or GRIPPER_LENGTH <= 0 or USE_GRIPPER:
            return
        
        # J3 servo angle determines TCP pitch relative to link2
        # Servo J3: 0-180, offset 90, so math_j3 = -90 to +90
        # TCP pitch = link2_geo - math_j3
        # Therefore TCP can swing from (link2_geo + 90) to (link2_geo - 90)
        
        link2_rad = np.radians(link2_geo_angle)
        
        # J3 math angle range: -90 to +90
        # But we need to consider servo limits (0-180 after offset)
        j3_math_min = -90.0  # servo = 0
        j3_math_max = 90.0   # servo = 180
        
        # TCP geo angle = link2_geo - j3_math
        # When j3_math = -90: TCP points most upward (link2_geo + 90)
        # When j3_math = +90: TCP points most downward (link2_geo - 90)
        tcp_angle_min = np.radians(link2_geo_angle - j3_math_max)  # lowest TCP angle
        tcp_angle_max = np.radians(link2_geo_angle - j3_math_min)  # highest TCP angle
        
        # Draw the arc showing possible TCP positions
        angles = np.linspace(tcp_angle_min, tcp_angle_max, 50)
        tcp_r = wrist_r + GRIPPER_LENGTH * np.cos(angles)
        tcp_z = wrist_z + GRIPPER_LENGTH * np.sin(angles)
        
        # Draw TCP workspace arc (faded, showing possible range)
        self.ax_rz.plot(tcp_r, tcp_z, 'c-', linewidth=2, alpha=0.5, label=f'TCP Range ({GRIPPER_LENGTH}mm)')
        
        # Fill the area lightly
        polygon_r = np.concatenate([[wrist_r], tcp_r, [wrist_r]])
        polygon_z = np.concatenate([[wrist_z], tcp_z, [wrist_z]])
        self.ax_rz.fill(polygon_r, polygon_z, color='cyan', alpha=0.1)
        
        # Draw lines at pitch extremes
        self.ax_rz.plot([wrist_r, wrist_r + GRIPPER_LENGTH * np.cos(tcp_angle_min)],
                        [wrist_z, wrist_z + GRIPPER_LENGTH * np.sin(tcp_angle_min)],
                        'c--', linewidth=1, alpha=0.4)
        self.ax_rz.plot([wrist_r, wrist_r + GRIPPER_LENGTH * np.cos(tcp_angle_max)],
                        [wrist_z, wrist_z + GRIPPER_LENGTH * np.sin(tcp_angle_max)],
                        'c--', linewidth=1, alpha=0.4)
        
        # Annotate limits
        self.ax_rz.annotate(f'J3=0\n({np.degrees(tcp_angle_max):.0f}°)',
                           (wrist_r + GRIPPER_LENGTH * np.cos(tcp_angle_max) * 0.7,
                            wrist_z + GRIPPER_LENGTH * np.sin(tcp_angle_max) * 0.7),
                           fontsize=7, color='teal', alpha=0.7)
        self.ax_rz.annotate(f'J3=180\n({np.degrees(tcp_angle_min):.0f}°)',
                           (wrist_r + GRIPPER_LENGTH * np.cos(tcp_angle_min) * 0.7,
                            wrist_z + GRIPPER_LENGTH * np.sin(tcp_angle_min) * 0.7),
                           fontsize=7, color='teal', alpha=0.7)
    
    def _process_ik(self):
        x, y, z, phi = self.current_x, self.current_y, self.current_z, self.current_phi
        r = np.sqrt(x**2 + y**2)
        
        success, angles, debug = inverse_kinematics_full(x, y, z, phi)
        
        if success:
            j0, j1, j2, j3 = angles["j0"], angles["j1"], angles["j2"], angles["j3"]
            
            fk, pts = forward_kinematics(j0, j1, j2, j3)
            self.ax_rz.plot([p[0] for p in pts], [p[1] for p in pts], 'g-', linewidth=3, label='Arm')
            self.ax_rz.plot([p[0] for p in pts], [p[1] for p in pts], 'go', markersize=8)
            
            # Get global angles from FK for gravity compensation
            link_angles = fk.get("link_angles", {})
            link1_geo = link_angles.get("link1_geo", 0)
            link2_geo = link_angles.get("link2_geo", 0)
            link3_geo = link_angles.get("link3_geo", 0)
            
            # Apply gravity compensation using GLOBAL angles (from horizontal floor)
            j1_c = apply_gravity_compensation(1, j1, link1_geo)
            j2_c = apply_gravity_compensation(2, j2, link2_geo)
            j3_c = apply_gravity_compensation(3, j3, link3_geo)
            
            fk_c, pts_c = forward_kinematics(j0, j1_c, j2_c, j3_c)
            self.ax_rz.plot([p[0] for p in pts_c], [p[1] for p in pts_c], 'b--', linewidth=2, alpha=0.7, label='+Gravity')
            
            # Draw target point and wrist
            target_label = 'TCP Target' if USE_GRIPPER else 'Wrist Target'
            self.ax_rz.plot(r, z, 'r*', markersize=20, label=f'{target_label} ({r:.0f}, {z:.0f})')
            self.ax_rz.plot(debug['wrist_r'], debug['wrist_z'], 'mo', markersize=10, label='Wrist')
            
            # Draw TCP workspace arc at wrist position (only when USE_GRIPPER=False)
            # Pass link2_geo angle so arc can calculate J3 sweep range
            link_angles = fk.get("link_angles", {})
            link2_geo = link_angles.get("link2_geo", 0)
            self._draw_tcp_workspace(debug['wrist_r'], debug['wrist_z'], link2_geo)
            
            return self._gen_ik_text(x, y, z, phi, r, angles, debug, j1_c, j2_c, j3_c, fk_c)
        else:
            # IK failed - do NOT draw target point, only show error
            return self._gen_fail_text(x, y, z, phi, r, debug), []
    
    def _process_fk(self):
        j0, j1, j2, j3 = self.current_j0, self.current_j1, self.current_j2, self.current_j3
        
        fk, pts = forward_kinematics(j0, j1, j2, j3)
        self.ax_rz.plot([p[0] for p in pts], [p[1] for p in pts], 'g-', linewidth=3, label='Arm')
        self.ax_rz.plot([p[0] for p in pts], [p[1] for p in pts], 'go', markersize=8)
        
        # Get global angles from FK for gravity compensation
        link_angles = fk.get("link_angles", {})
        link1_geo = link_angles.get("link1_geo", 0)
        link2_geo = link_angles.get("link2_geo", 0)
        link3_geo = link_angles.get("link3_geo", 0)
        
        # Apply gravity compensation using GLOBAL angles (from horizontal floor)
        j1_c = apply_gravity_compensation(1, j1, link1_geo)
        j2_c = apply_gravity_compensation(2, j2, link2_geo)
        j3_c = apply_gravity_compensation(3, j3, link3_geo)
        
        fk_c, pts_c = forward_kinematics(j0, j1_c, j2_c, j3_c)
        self.ax_rz.plot([p[0] for p in pts_c], [p[1] for p in pts_c], 'b--', linewidth=2, alpha=0.7, label='+Gravity')
        
        # Wrist position is pts[-2] (before TCP), TCP is pts[-1]
        # When ARM_D5=0, pts[-1] IS the wrist
        if ARM_D5 == 0:
            wrist_r, wrist_z = pts[-1][0], pts[-1][1]
            self.ax_rz.plot(wrist_r, wrist_z, 'mo', markersize=10, label='Wrist')
        else:
            wrist_r, wrist_z = pts[-2][0], pts[-2][1]
            self.ax_rz.plot(pts[-1][0], pts[-1][1], 'r*', markersize=20, label='TCP')
            self.ax_rz.plot(wrist_r, wrist_z, 'mo', markersize=10, label='Wrist')
        
        # Draw TCP workspace arc (only when USE_GRIPPER=False)
        # link2_geo already defined above for gravity compensation
        self._draw_tcp_workspace(wrist_r, wrist_z, link2_geo)
        
        return self._gen_fk_text(j0, j1, j2, j3, j1_c, j2_c, j3_c, fk, fk_c)
    
    def _gen_ik_text(self, x, y, z, phi, r, angles, debug, j1_c, j2_c, j3_c, fk_c):
        # Get inclinometer for BOTH Xavier (no gravity) and with Gravity
        # First calculate FK without gravity for Xavier inclinometer
        fk_xavier, _ = forward_kinematics(angles['j0'], angles['j1'], angles['j2'], angles['j3'])
        link_ang_xavier = fk_xavier.get("link_angles", {})
        incl1_xavier = convert_to_inclinometer(link_ang_xavier.get("link1_geo", 0))
        incl2_xavier = convert_to_inclinometer(link_ang_xavier.get("link2_geo", 0))
        incl3_xavier = convert_to_inclinometer(link_ang_xavier.get("link3_geo", 0))
        
        # With gravity compensation
        link_ang_grav = fk_c.get("link_angles", {})
        incl1_grav = convert_to_inclinometer(link_ang_grav.get("link1_geo", 0))
        incl2_grav = convert_to_inclinometer(link_ang_grav.get("link2_geo", 0))
        incl3_grav = convert_to_inclinometer(link_ang_grav.get("link3_geo", 0))
        
        main = [
            "===== IK MODE =====", "",
            f"INPUT: X={x:.1f} Y={y:.1f} Z={z:.1f}",
            f"Phi={phi:.1f} R={r:.1f}mm", "",
            "XAVIER OUTPUT:",
            f"  J0={angles['j0']:.1f} J1={angles['j1']:.1f}",
            f"  J2={angles['j2']:.1f} J3={angles['j3']:.1f}", "",
            "GRAVITY COMP:",
            f"  J1: {angles['j1']:.1f} -> {j1_c:.1f}",
            f"  J2: {angles['j2']:.1f} -> {j2_c:.1f}",
            f"  J3: {angles['j3']:.1f} -> {j3_c:.1f}", "",
            f"Wrist: R={debug['wrist_r']:.1f} Z={debug['wrist_z']:.1f}",
        ]
        
        incl = self._gen_incl_text_both(
            angles['j1'], angles['j2'], angles['j3'],
            j1_c, j2_c, j3_c,
            incl1_xavier, incl2_xavier, incl3_xavier,
            incl1_grav, incl2_grav, incl3_grav)
        return main, incl
    
    def _gen_fk_text(self, j0, j1, j2, j3, j1_c, j2_c, j3_c, fk, fk_c):
        # Xavier (no gravity) inclinometer
        link_ang_xavier = fk.get("link_angles", {})
        incl1_xavier = convert_to_inclinometer(link_ang_xavier.get("link1_geo", 0))
        incl2_xavier = convert_to_inclinometer(link_ang_xavier.get("link2_geo", 0))
        incl3_xavier = convert_to_inclinometer(link_ang_xavier.get("link3_geo", 0))
        
        # With gravity compensation
        link_ang_grav = fk_c.get("link_angles", {})
        incl1_grav = convert_to_inclinometer(link_ang_grav.get("link1_geo", 0))
        incl2_grav = convert_to_inclinometer(link_ang_grav.get("link2_geo", 0))
        incl3_grav = convert_to_inclinometer(link_ang_grav.get("link3_geo", 0))
        
        main = [
            "===== FK MODE =====", "",
            "INPUT JOINTS:",
            f"  J0={j0:.1f} J1={j1:.1f}",
            f"  J2={j2:.1f} J3={j3:.1f}", "",
            "GRAVITY COMP:",
            f"  J1: {j1:.1f} -> {j1_c:.1f}",
            f"  J2: {j2:.1f} -> {j2_c:.1f}",
            f"  J3: {j3:.1f} -> {j3_c:.1f}", "",
            "TCP (no gravity):",
            f"  X={fk['x']:.1f} Y={fk['y']:.1f}",
            f"  Z={fk['z']:.1f} R={fk['r']:.1f}",
            f"  Phi={fk['phi']:.1f}", "",
            "TCP (+gravity):",
            f"  X={fk_c['x']:.1f} Y={fk_c['y']:.1f}",
            f"  Z={fk_c['z']:.1f}",
        ]
        
        incl = self._gen_incl_text_both(
            j1, j2, j3,
            j1_c, j2_c, j3_c,
            incl1_xavier, incl2_xavier, incl3_xavier,
            incl1_grav, incl2_grav, incl3_grav)
        return main, incl
    
    def _gen_incl_text_both(self, j1, j2, j3, j1_c, j2_c, j3_c,
                            incl1_x, incl2_x, incl3_x,
                            incl1_g, incl2_g, incl3_g):
        """Generate inclinometer table showing BOTH Xavier and Gravity values"""
        return [
            "===== INCLINOMETER =====",
            "(0=ngang +90=len -90=xuong)", "",
            "Servo angles:",
            "         Xavier  | +Gravity",
            f"  J1:   {j1:6.1f}  | {j1_c:6.1f}",
            f"  J2:   {j2:6.1f}  | {j2_c:6.1f}",
            f"  J3:   {j3:6.1f}  | {j3_c:6.1f}", "",
            "+-------------------------------+",
            "|       | Xavier  | +Gravity   |",
            "+-------------------------------+",
            f"| Link1 | {incl1_x:+6.1f}  | {incl1_g:+6.1f} deg |",
            "+-------------------------------+",
            f"| Link2 | {incl2_x:+6.1f}  | {incl2_g:+6.1f} deg |",
            "+-------------------------------+",
            f"| Link3 | {incl3_x:+6.1f}  | {incl3_g:+6.1f} deg |",
            "+-------------------------------+", "",
            "Xavier = IK output (green)",
            "+Gravity = compensated (blue)",
        ]
    
    def _gen_fail_text(self, x, y, z, phi, r, debug):
        return [
            "===== IK FAILED =====", "",
            f"INPUT: X={x:.1f} Y={y:.1f} Z={z:.1f}",
            f"Phi={phi:.1f} R={r:.1f}mm", "",
            "REASON:",
            f"  {debug.get('fail_reason', 'Unknown')}", "",
            "Try different coordinates.",
        ]
    
    def run(self):
        plt.show()


def main():
    print("=" * 50)
    print("  Robot Arm Workspace Debugger")
    print("=" * 50)
    print(f"Robot: d1={ARM_D1}, a2={ARM_A2}, a3={ARM_A3}, d5={ARM_D5}")
    print(f"Lift Height: {LIFT_HEIGHT}mm | Default Pitch: {DEFAULT_PITCH}°")
    print(f"Gravity: J1={GRAVITY_GAINS.get(1,0)}, J2={GRAVITY_GAINS.get(2,0)}, J3={GRAVITY_GAINS.get(3,0)}")
    print("-" * 50)
    
    debugger = InteractiveDebugger()
    debugger.run()


if __name__ == "__main__":
    main()
