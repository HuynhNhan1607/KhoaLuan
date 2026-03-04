"""
Kinematics Module
Contains forward kinematics, inverse kinematics, and angle mapping functions
"""

import numpy as np
from config import (
    d1, a2, a3, d5,
    JOINT_LIMITS_DEG,
    SERVO_MAPPING_CONFIG
)


def map_angle(joint_name, math_angle):
    """
    Convert math/DH angle to servo angle
    
    Args:
        joint_name: Joint key (j0, j1, j2, j3, j4, j5)
        math_angle: Math/DH angle in degrees
        
    Returns:
        Servo angle (0-180 degrees)
    """
    cfg = SERVO_MAPPING_CONFIG.get(joint_name, {"offset": 90, "dir": 1})
    servo_angle = cfg["offset"] + (math_angle * cfg["dir"])
    
    # Safety Clamp
    if not (0 <= servo_angle <= 180):
        print(f"[WARNING] Clamping {joint_name}: {servo_angle:.1f} -> {max(0, min(180, servo_angle))}")
        servo_angle = max(0, min(180, servo_angle))
        
    print(f"[MAP-ANGLE] {joint_name}: math={math_angle:.1f}° -> servo={servo_angle:.1f}° (offset={cfg['offset']}, dir={cfg['dir']})")
    return servo_angle


def unmap_angle(joint_name, servo_angle):
    """
    Convert servo angle to math/DH angle
    
    Args:
        joint_name: Joint key (j0, j1, j2, j3, j4, j5)
        servo_angle: Servo angle (0-180 degrees)
        
    Returns:
        Math/DH angle in degrees
    """
    cfg = SERVO_MAPPING_CONFIG.get(joint_name, {"offset": 90, "dir": 1})
    # Servo = Offset + (Math * Dir) => Math = (Servo - Offset) / Dir
    return (servo_angle - cfg["offset"]) / cfg["dir"]


def map_servo_to_math(joint_key, servo_angle):
    """Helper function for mapping servo to math angle"""
    cfg = SERVO_MAPPING_CONFIG[joint_key]
    return (servo_angle - cfg["offset"]) / cfg["dir"]


def map_math_to_servo(joint_key, math_angle):
    """Helper function for mapping math to servo angle"""
    cfg = SERVO_MAPPING_CONFIG[joint_key]
    return cfg["offset"] + (math_angle * cfg["dir"])


def fk_geometric(j0_deg, j2_servo, j3_servo, j4_servo):
    """
    Forward Kinematics (Geometric)
    
    Args:
        j0_deg: Base rotation (servo angle)
        j2_servo: Shoulder angle (servo angle) 
        j3_servo: Elbow angle (servo angle)
        j4_servo: Wrist pitch angle (servo angle)
        
    Returns:
        tuple: ([x, y, z], phi_deg, None)
            - [x, y, z]: TCP position in mm
            - phi_deg: End effector pitch angle in degrees
            - None: placeholder for compatibility
    """
    # 1. Base Rotation (J0)
    theta0_deg = map_servo_to_math("j0", j0_deg)
    theta0_rad = np.radians(theta0_deg)

    # 2. Planar Arm (R-Z plane)
    m2 = map_servo_to_math("j1", j2_servo)
    m3 = map_servo_to_math("j2", j3_servo)
    m4 = map_servo_to_math("j3", j4_servo)
    
    t2, t3, t4 = np.radians(m2), np.radians(m3), np.radians(m4)
    
    # Coordinates in R-Z plane (relative to J2 axis)
    # J2 (Dir -1) -> theta2 + 90
    angle_2_plot = t2 + np.pi/2
    p2_r = 0 + a2 * np.cos(angle_2_plot)
    p2_z = d1 + a2 * np.sin(angle_2_plot)
    
    # J3
    angle_3_plot = angle_2_plot - t3
    p3_r = p2_r + a3 * np.cos(angle_3_plot)
    p3_z = p2_z + a3 * np.sin(angle_3_plot)
    
    # J4
    angle_4_plot = angle_3_plot - t4
    p4_r = p3_r + d5 * np.cos(angle_4_plot)
    p4_z = p3_z + d5 * np.sin(angle_4_plot)  # Tip Position
    
    # 3. Rotate 2D (R, Z) into 3D (X, Y, Z)
    r = p4_r
    z = p4_z
    
    x = r * np.cos(theta0_rad)
    y = r * np.sin(theta0_rad)
    
    current_phi_deg = np.degrees(angle_4_plot)
    
    return [x, y, z], current_phi_deg, None


def calculate_link_angles(j0_servo, j1_servo, j2_servo, j3_servo):
    """
    Calculate geometric angles of each link relative to horizontal plane.
    
    Args:
        j0_servo: J0 servo angle (base rotation)
        j1_servo: J1 servo angle (shoulder)
        j2_servo: J2 servo angle (elbow)
        j3_servo: J3 servo angle (wrist)
        
    Returns:
        list: [j0_geo, j1_geo, j2_geo, j3_geo, j4_geo, j5_geo]
              Link angles in degrees relative to horizontal (0=horizontal, +90=up, -90=down)
    """
    # Convert servo angles to math angles
    m1 = map_servo_to_math("j1", j1_servo)
    m2 = map_servo_to_math("j2", j2_servo)
    m3 = map_servo_to_math("j3", j3_servo)
    
    t1, t2, t3 = np.radians(m1), np.radians(m2), np.radians(m3)
    
    # Calculate geometric angles (same as workspace_visualizer.py)
    # angle_1_plot = math angle + 90° (link1 angle from horizontal)
    link1_geo_rad = t1 + np.pi / 2.0
    
    # link2 = link1 - math_q2
    link2_geo_rad = link1_geo_rad - t2
    
    # link3 = link2 - math_q3
    link3_geo_rad = link2_geo_rad - t3
    
    # Convert to degrees
    link1_geo = np.degrees(link1_geo_rad)
    link2_geo = np.degrees(link2_geo_rad)
    link3_geo = np.degrees(link3_geo_rad)
    
    # Return array: [J0, J1, J2, J3, J4, J5]
    # J0, J4, J5 don't need gravity compensation, set to 0
    return [0.0, link1_geo, link2_geo, link3_geo, 0.0, 0.0]


def within_limits(angle_deg, joint_name, eps=1e-6):
    """
    Check if angle is within joint limits
    
    Args:
        angle_deg: Angle in degrees
        joint_name: Joint name (theta1, theta2, etc.)
        eps: Tolerance epsilon
        
    Returns:
        bool: True if within limits
    """
    amin, amax = JOINT_LIMITS_DEG[joint_name]
    if joint_name == "theta1":
        angle_deg = angle_deg % 360.0
    return (amin - eps) <= angle_deg <= (amax + eps)


def inverse_kinematics_exact(target_r, target_z, target_phi):
    """
    Exact Inverse Kinematics (Geometric) for 3-link planar arm
    
    Args:
        target_r: Distance from Z-axis to TCP (R-Z plane)
        target_z: TCP height
        target_phi: Pitch angle in degrees from horizontal
        
    Returns:
        tuple: (sv2, sv3, sv4) servo angles (0-180) or None if unreachable
    """
    phi_rad = np.radians(target_phi)
    
    # Calculate Wrist Center
    wrist_r = target_r - d5 * np.cos(phi_rad)
    wrist_z = target_z - d5 * np.sin(phi_rad)
    
    r_rel = wrist_r
    z_rel = wrist_z - d1
    
    dist_sq = r_rel**2 + z_rel**2
    dist = np.sqrt(dist_sq)
    
    max_reach = a2 + a3
    # Check simple reach
    if dist > max_reach: 
        return None
    if dist < abs(a2 - a3):  # Triangle constraint
        return None 
    
    # Law of Cosines for J3 (Elbow)
    cos_q3 = (dist_sq - a2**2 - a3**2) / (2 * a2 * a3)
    cos_q3 = np.clip(cos_q3, -1.0, 1.0)
    q3_rad = np.arccos(cos_q3)
    
    # Calculate J2 (Shoulder)
    alpha = np.arctan2(z_rel, r_rel)
    beta = np.arctan2(a3 * np.sin(q3_rad), a2 + a3 * np.cos(q3_rad))
    q2_polar = alpha + beta 
    
    # Convert to Math Angle Definition
    math_q2 = np.degrees(q2_polar) - 90
    math_q3 = np.degrees(q3_rad)
    
    # Calculate J4 (Wrist)
    math_q4 = (math_q2 + 90) - math_q3 - target_phi
    
    # Map to Servo Values
    sv2 = map_math_to_servo("j1", math_q2)
    sv3 = map_math_to_servo("j2", math_q3)
    sv4 = map_math_to_servo("j3", math_q4)
    
    # Check Limits (Hardware 0-180)
    if not (0 <= sv2 <= 180 and 0 <= sv3 <= 180 and 0 <= sv4 <= 180):
        return None
        
    return sv2, sv3, sv4


def find_best_solution(target_r, target_z, target_phi, tol_pos=10, tol_phi=90):
    """
    Fuzzy IK Scanner - Find alternative solution near target if exact IK fails
    
    Args:
        target_r: Target radial distance
        target_z: Target height
        target_phi: Target pitch angle
        tol_pos: Position tolerance in mm
        tol_phi: Angle tolerance in degrees
        
    Returns:
        tuple: (best_solution, best_info) or (None, {})
    """
    best_sol = None
    min_score = float('inf')
    best_info = {}

    # Position search grid (5mm steps)
    pos_step = 5.0 
    search_pos = np.arange(-tol_pos, tol_pos + pos_step, pos_step)
    
    # Angle search grid (3 degree steps)
    phi_step = 3.0
    search_phi = np.arange(-tol_phi, tol_phi + phi_step, phi_step)
    
    for d_phi in search_phi:
        curr_phi = target_phi + d_phi
        for dr in search_pos:
            for dz in search_pos:
                curr_r = target_r + dr
                curr_z = target_z + dz
                
                # Calculate IK
                sol = inverse_kinematics_exact(curr_r, curr_z, curr_phi)
                
                if sol:
                    s2, s3, s4 = sol
                    
                    # Cost Function
                    dist_error = np.sqrt(dr**2 + dz**2)
                    phi_error = abs(d_phi)
                    
                    # Servo Comfort: prefer 90 deg (center)
                    servo_cost = (abs(s2-90) + abs(s3-90) + abs(s4-90)) / 180.0
                    
                    # Weighting: Pos >> Angle >> ServoComfort
                    total_score = (dist_error * 1.0) + (phi_error * 0.5) + (servo_cost * 1.0)
                    
                    if total_score < min_score:
                        min_score = total_score
                        best_sol = sol
                        best_info = {
                            "r": curr_r, "z": curr_z, "phi": curr_phi, 
                            "score": min_score
                        }

    return best_sol, best_info


def ik_5dof_optimized(x, y, z, phi_deg=0.0, tol_pos=10, tol_phi=90):
    """
    Main IK Entry Point - 5-DOF Inverse Kinematics
    Converts 3D (x,y,z) -> 2D (r,z) + Base Rotation
    
    Physical Constraints:
    - J0 servo range: 0° to 180°
    - J0=0° → arm points to +X axis (y=0)
    - J0=180° → arm points to -X axis (y=0)
    - Workspace: only Y≥0 (upper half-plane)
    - Q3 (x<0, y<0) and Q4 (x>0, y<0) are UNREACHABLE
    
    Args:
        x, y, z: Target TCP position in mm
        phi_deg: Target pitch angle in degrees
        tol_pos: Position tolerance for fuzzy search
        tol_phi: Angle tolerance for fuzzy search
        
    Returns:
        list: [j0, j1, j2, j3, j4] servo angles or None if unreachable
    """
    # 1. Calculate Base Rotation (J0)
    # arctan2 returns angle in range [-180°, +180°]
    theta0_rad = np.arctan2(y, x)
    theta0_deg = np.degrees(theta0_rad)
    
    # Check if angle is within physical servo range [0°, 180°]
    # If y<0, angle will be negative or >180° → unreachable
    if theta0_deg < 0 or theta0_deg > 180:
        print(f"[IK-FAIL] Target (x={x:.1f}, y={y:.1f}) requires J0={theta0_deg:.1f}° (out of range [0°, 180°])")
        print(f"[IK-FAIL] Workspace limited to Y≥0 (upper half-plane only)")
        return None
    
    # Map to Servo J0 (with offset=0, dir=1, this is direct mapping)
    sv0 = map_math_to_servo("j0", theta0_deg)
    
    if not (0 <= sv0 <= 180):
        print(f"[IK-FAIL] Base J0 servo out of bounds: {sv0:.1f}°")
        return None
        
    # 2. Calculate Cylindrical Coordinates (R, Z)
    r = np.sqrt(x**2 + y**2)
    
    # 3. Try Exact IK first
    sol = inverse_kinematics_exact(r, z, phi_deg)
    
    if sol:
        sv2, sv3, sv4 = sol
        return [sv0, sv2, sv3, sv4, 90.0]  # j4(roll) default 90
        
    # 4. If Exact Fails, Try Fuzzy
    fuzzy_sol, fuzzy_info = find_best_solution(r, z, phi_deg, tol_pos=tol_pos, tol_phi=tol_phi)
    
    if fuzzy_sol:
        sv2, sv3, sv4 = fuzzy_sol
        print(f"[IK-FUZZY] Found solution: dist_err={fuzzy_info['r']-r:.1f}mm")
        return [sv0, sv2, sv3, sv4, 90.0]
        
    return None
