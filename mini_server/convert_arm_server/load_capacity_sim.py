#!/usr/bin/env python3
"""
Robot Arm Load Capacity Simulator
Simulates and plots the load-carrying capacity (10g - 500g) of the robot arm joints
at various coordinates with a constant pitch of -90 degrees.
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

# Add current directory to path to ensure config and kinematics can be imported
sys.path.append(os.path.abspath(os.path.dirname(__file__)))

try:
    from config import d1, a2, a3, d5
except ImportError:
    print("[WARNING] Could not import config.py. Using default DH parameters.")
    d1 = 32.0      # mm
    a2 = 105.0     # mm
    a3 = 130.0     # mm
    d5 = 140.0     # mm

# ==========================================
# CONFIGURATION & PHYSICAL PARAMETERS
# ==========================================
# Gravity constant (m/s^2)
g = 9.81

# Servo Specs (Cheap $8-$10 "20kg" Servo)
TORQUE_LIMIT_KG_CM = 16.0  # Real peak stall torque (actually ~16kg*cm at 6V)
SAFE_CONTINUOUS_KG_CM = 4.0  # Safe continuous thermal holding limit (~25% of real stall torque)
# Convert kg*cm to N*m: 1 kg*cm = 0.0980665 N*m
T_limit = TORQUE_LIMIT_KG_CM * 0.0980665  # ~1.569 N*m

# Component Masses (in kg)
m_servo = 0.068   # Weight of one servo (68g)
m_link1 = 0.150   # Structural bracket weight of Link 1 (150g split of 400g)
m_link2 = 0.200   # Structural bracket weight of Link 2 (200g split of 400g)
m_tool = 0.186    # Gripper assembly weight (186g, including J5 gripper servo)

# Point masses at joints:
# J1 (Shoulder) is at (0, d1) - supported by base, doesn't add gravity torque to J1.
m_s2 = m_servo     # J2 servo at elbow (68g)
m_s3 = m_servo     # J3 servo at wrist (68g, wrist pitch)
m_s4 = m_servo     # J4 servo at wrist (68g, wrist roll)
m_wrist_joints = m_s3 + m_s4  # Total wrist point mass (136g)

# Link lengths in meters
a2_m = a2 / 1000.0
a3_m = a3 / 1000.0
d5_m = d5 / 1000.0
d1_m = d1 / 1000.0

# Base Box height (mm)
base_height = 170.0

# Pre-calculate base torque terms for J2 and J1 to optimize computation:
# T2 = cos(phi_2) * [ g * a3_m * (m_wrist_joints + m_tool + m_link2/2) + g * a3_m * m_load ]
T2_base = g * a3_m * (m_wrist_joints + m_tool + m_link2 / 2.0)
T2_coeff = g * a3_m

# T1 = cos(phi_1) * [ g * a2_m * (m_link1/2 + m_s2 + m_link2 + m_wrist_joints + m_tool) + g * a2_m * m_load ] + T2
T1_base_1 = g * a2_m * (m_link1 / 2.0 + m_s2 + m_link2 + m_wrist_joints + m_tool)
T1_coeff_1 = g * a2_m


def ik_pitch_90(r, z):
    """
    Exact Inverse Kinematics for a 3-link planar arm under pitch = -90 deg.
    
    Args:
        r: radial distance from Z-axis to TCP (mm)
        z: TCP height relative to J0 (mm) (z_world - base_height)
        
    Returns:
        list of dict: Valid solutions, each containing:
            - 'sv2': Shoulder servo angle (deg, 0-180)
            - 'sv3': Elbow servo angle (deg, 0-180)
            - 'sv4': Wrist servo angle (deg, 0-180)
            - 'phi1': Link 1 geometric angle (rad)
            - 'phi2': Link 2 geometric angle (rad)
            - 'type': 'Elbow Up' or 'Elbow Down'
    """
    # Wrist center position
    # Since pitch is -90 deg, Link 3 points straight down.
    # r_wrist = r, z_wrist = z + d5
    r_w = r
    z_w = z + d5
    
    # Coordinates relative to Shoulder joint (0, d1)
    r_rel = r_w
    z_rel = z_w - d1
    
    dist_sq = r_rel**2 + z_rel**2
    dist = np.sqrt(dist_sq)
    
    # Triangle inequality check
    if dist > (a2 + a3) or dist < np.abs(a2 - a3):
        return []
        
    cos_q3 = (dist_sq - a2**2 - a3**2) / (2.0 * a2 * a3)
    cos_q3 = np.clip(cos_q3, -1.0, 1.0)
    q3_rad = np.arccos(cos_q3)
    
    solutions = []
    
    # Case 1: Elbow Up (positive q3)
    alpha = np.arctan2(z_rel, r_rel)
    beta = np.arctan2(a3 * np.sin(q3_rad), a2 + a3 * np.cos(q3_rad))
    q2_polar_up = alpha + beta
    
    math_q2_up = np.degrees(q2_polar_up) - 90.0
    math_q3_up = np.degrees(q3_rad)
    math_q4_up = (math_q2_up + 90.0) - math_q3_up - (-90.0)
    
    sv2_up = math_q2_up + 90.0
    sv3_up = math_q3_up + 90.0
    sv4_up = math_q4_up + 90.0
    
    if (0.0 <= sv2_up <= 180.0) and (0.0 <= sv3_up <= 180.0) and (0.0 <= sv4_up <= 180.0):
        # Calculate geometric angles relative to horizontal plane
        phi1 = np.radians(sv2_up)
        phi2 = np.radians(sv2_up - sv3_up + 90.0)
        solutions.append({
            'sv2': sv2_up, 'sv3': sv3_up, 'sv4': sv4_up,
            'phi1': phi1, 'phi2': phi2, 'type': 'Elbow Up'
        })
        
    # Case 2: Elbow Down (negative q3)
    beta_down = np.arctan2(a3 * np.sin(-q3_rad), a2 + a3 * np.cos(-q3_rad))
    q2_polar_down = alpha + beta_down
    
    math_q2_down = np.degrees(q2_polar_down) - 90.0
    math_q3_down = np.degrees(-q3_rad)
    math_q4_down = (math_q2_down + 90.0) - math_q3_down - (-90.0)
    
    sv2_down = math_q2_down + 90.0
    sv3_down = math_q3_down + 90.0
    sv4_down = math_q4_down + 90.0
    
    if (0.0 <= sv2_down <= 180.0) and (0.0 <= sv3_down <= 180.0) and (0.0 <= sv4_down <= 180.0):
        # Ensure distinct from elbow up
        is_distinct = True
        if len(solutions) > 0:
            if np.abs(sv3_down - sv3_up) < 1e-2 and np.abs(sv2_down - sv2_up) < 1e-2:
                is_distinct = False
        if is_distinct:
            phi1 = np.radians(sv2_down)
            phi2 = np.radians(sv2_down - sv3_down + 90.0)
            solutions.append({
                'sv2': sv2_down, 'sv3': sv3_down, 'sv4': sv4_down,
                'phi1': phi1, 'phi2': phi2, 'type': 'Elbow Down'
            })
            
    return solutions


def compute_max_payload(phi1, phi2):
    """
    Computes the maximum payload (in kg) the arm can carry before exceeding
    the torque limit on J1 or J2.
    
    Args:
        phi1: Link 1 angle relative to horizontal (rad)
        phi2: Link 2 angle relative to horizontal (rad)
        
    Returns:
        float: Max payload in kg (0.0 if arm cannot support its own weight)
    """
    cos_p1 = np.cos(phi1)
    cos_p2 = np.cos(phi2)
    
    # 1. Elbow Joint (J2) Torque Constraint:
    # T2 = cos(phi_2) * (T2_base + T2_coeff * m_load)
    # We require |T2| <= T_limit
    abs_cos_p2 = np.abs(cos_p2)
    if abs_cos_p2 < 1e-6:
        # Elbow is vertical, payload doesn't create J2 torque
        max_load_j2 = float('inf')
    else:
        # T2_base + T2_coeff * m_load <= T_limit / |cos(phi_2)|
        max_load_j2 = (T_limit / abs_cos_p2 - T2_base) / T2_coeff
        if max_load_j2 < 0:
            return 0.0  # self-weight exceeds limits
            
    # 2. Shoulder Joint (J1) Torque Constraint:
    # T1 = m_load * A1 + B1
    # A1 = g * r_tcp (horizontal distance)
    # B1 = g * cos(phi_1) * a2 * M1 + g * cos(phi_2) * a3 * M2
    r_tcp_m = a2_m * cos_p1 + a3_m * cos_p2
    A1 = g * r_tcp_m
    B1 = cos_p1 * T1_base_1 + cos_p2 * T2_base
    
    if np.abs(A1) < 1e-6:
        max_load_j1 = float('inf')
    else:
        # We need |A1 * m_load + B1| <= T_limit
        # Since A1 > 0:
        # -T_limit - B1 <= A1 * m_load <= T_limit - B1
        if B1 > T_limit or B1 < -T_limit:
            return 0.0  # self-weight exceeds limits
        max_load_j1 = (T_limit - B1) / A1
        if max_load_j1 < 0:
            return 0.0
            
    return min(max_load_j1, max_load_j2)


def get_arm_joints_xy(phi1, phi2):
    """
    Computes coordinates of arm joints relative to J0 for stick figure drawing.
    """
    # Shoulder joint J1 (r=0, z=d1)
    # Elbow joint J2
    # Wrist joint J3
    # TCP
    j1_r, j1_z = 0.0, d1
    j2_r, j2_z = a2 * np.cos(phi1), d1 + a2 * np.sin(phi1)
    j3_r, j3_z = j2_r + a3 * np.cos(phi2), j2_z + a3 * np.sin(phi2)
    tcp_r, tcp_z = j3_r, j3_z - d5  # Pitch is -90, straight down
    
    return (
        [j1_r, j2_r, j3_r, tcp_r],
        [j1_z, j2_z, j3_z, tcp_z]
    )


def run_simulation():
    # Grid definition in R-Z plane (Z relative to J0)
    # Constraints: r > 12cm (120mm), Z in [-200, 0] mm
    r_min, r_max = 120.0, 240.0  # mm
    z_min, z_max = -200.0, 0.0   # mm (relative to J0)
    
    # Resolution (1mm steps)
    r_grid = np.arange(r_min, r_max + 1.0, 1.0)
    z_grid = np.arange(z_min, z_max + 1.0, 1.0)
    
    R_mesh, Z_mesh = np.meshgrid(r_grid, z_grid)
    
    # Arrays to store max payload (in grams)
    # -1.0 means unreachable or invalid
    payload_up = np.full_like(R_mesh, -1.0, dtype=float)
    payload_down = np.full_like(R_mesh, -1.0, dtype=float)
    
    # Keep track of numerical statistics
    stats = {
        'up_reachable': 0,
        'down_reachable': 0,
        'up_capacities': [],
        'down_capacities': [],
    }
    
    for i in range(len(z_grid)):
        for j in range(len(r_grid)):
            r = r_grid[j]
            z = z_grid[i]
            
            sols = ik_pitch_90(r, z)
            
            for sol in sols:
                phi1 = sol['phi1']
                phi2 = sol['phi2']
                max_payload_kg = compute_max_payload(phi1, phi2)
                max_payload_g = max_payload_kg * 1000.0
                
                if sol['type'] == 'Elbow Up':
                    payload_up[i, j] = max_payload_g
                    stats['up_reachable'] += 1
                    stats['up_capacities'].append(max_payload_g)
                elif sol['type'] == 'Elbow Down':
                    payload_down[i, j] = max_payload_g
                    stats['down_reachable'] += 1
                    stats['down_capacities'].append(max_payload_g)
                    
    print("\nSimulation completed successfully!")
    print(f"Elbow Up Configuration: {stats['up_reachable']} reachable positions.")
    if stats['up_reachable'] > 0:
        valid_caps = [c for c in stats['up_capacities'] if c >= 0]
        print(f"  - Max Payload: {max(valid_caps):.1f}g")
        print(f"  - Min Payload: {min(valid_caps):.1f}g")
        print(f"  - Mean Payload: {np.mean(valid_caps):.1f}g")
        print(f"  - Support self-weight: {np.sum(np.array(valid_caps) > 0) / len(valid_caps) * 100:.1f}% of workspace")
        
    print(f"Elbow Down Configuration: {stats['down_reachable']} reachable positions.")
    if stats['down_reachable'] > 0:
        valid_caps = [c for c in stats['down_capacities'] if c >= 0]
        print(f"  - Max Payload: {max(valid_caps):.1f}g")
        print(f"  - Min Payload: {min(valid_caps):.1f}g")
        print(f"  - Mean Payload: {np.mean(valid_caps):.1f}g")
        print(f"  - Support self-weight: {np.sum(np.array(valid_caps) > 0) / len(valid_caps) * 100:.1f}% of workspace")

    # ==========================================
    # PLOTTING
    # ==========================================
    # Setup styling: Light Mode, White Background
    plt.rcParams['figure.facecolor'] = 'white'
    plt.rcParams['axes.facecolor'] = 'white'
    plt.rcParams['savefig.facecolor'] = 'white'
    plt.rcParams['font.family'] = 'sans-serif'
    
    # Custom color map: Red (unstable/fails) -> Yellow (low load) -> Green (high load) -> Dark Blue (>500g)
    # We will represent:
    # < 10g: Red (cannot carry nominal load)
    # 10g - 100g: Orange
    # 100g - 300g: Yellow/Light Green
    # 300g - 500g: Pure Green
    # > 500g: Blue-Green/Blue
    colors = [
        (0.85, 0.2, 0.2),   # Red (Fails)
        (1.0, 0.6, 0.2),   # Orange (10-100g)
        (0.9, 0.9, 0.2),   # Yellow (100-300g)
        (0.2, 0.8, 0.2),   # Green (300-500g)
        (0.1, 0.4, 0.8)    # Blue (>500g)
    ]
    cmap_name = 'load_capacity_cmap'
    custom_cmap = LinearSegmentedColormap.from_list(cmap_name, colors, N=256)
    
    # Create single plot
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # Create a masked array for plotting (hide unreachable points)
    masked_data = np.ma.masked_where(payload_up < 0, payload_up)
    
    # Plot heatmap
    # Set vmin to 0g and vmax to 500g (as payload is evaluated 10g-500g)
    cax = ax.pcolormesh(R_mesh, Z_mesh, masked_data, cmap=custom_cmap, vmin=0, vmax=500, shading='auto', alpha=0.95)
    
    # Grid lines and border
    ax.grid(color='gray', linestyle='--', linewidth=0.5, alpha=0.3)
    ax.axhline(0, color='black', linewidth=1.2, label='J0 Level (Z=0)')
    ax.axhline(-170, color='darkgray', linestyle='--', linewidth=1.2, label='Ground (Z=-170mm)')
    ax.axvline(120, color='red', linestyle=':', linewidth=1.5, label='Constraint r=12cm')
    
    # Draw robot base column (sketch)
    ax.fill_between([-10, 10], [-170, -170], [d1, d1], color='lightgray', alpha=0.5, edgecolor='gray', zorder=2)
    ax.plot([0, 0], [-170, d1], color='black', linestyle='--', linewidth=1.0, zorder=3)
    
    # Title and Labels
    ax.set_title('Elbow Up Configuration - Reachable Workspace Load Capacity', fontsize=14, fontweight='bold', pad=15)
    ax.set_xlabel('Radial Reach R (mm)', fontsize=12)
    ax.set_ylabel('Height Z relative to J0 (mm)', fontsize=12)
    ax.set_xlim(0, 260)
    ax.set_ylim(-210, 50)
    
    # Plot stick figures of the arm at representative positions
    # Let's select two radial distances for representation: r=140mm, r=200mm at z=-100mm
    sample_rs = [140.0, 200.0]
    sample_z = -100.0
    
    for sr in sample_rs:
        sols = ik_pitch_90(sr, sample_z)
        for sol in sols:
            if sol['type'] == 'Elbow Up':
                # Get joint positions
                xs, ys = get_arm_joints_xy(sol['phi1'], sol['phi2'])
                
                # Plot links
                ax.plot(xs, ys, color='black', linewidth=3.5, zorder=5, alpha=0.85)
                # Plot joint pivots
                ax.scatter(xs[:-1], ys[:-1], color='dimgray', s=50, edgecolor='black', zorder=6)
                # Plot TCP
                ax.scatter(xs[-1], ys[-1], color='red', s=60, marker='X', zorder=6, label='TCP' if sr == sample_rs[0] else "")
                
                # Compute max payload at this sample point
                payload_kg = compute_max_payload(sol['phi1'], sol['phi2'])
                payload_g = payload_kg * 1000.0
                ax.text(sr, sample_z + 15, f"{payload_g:.0f}g", color='black', fontweight='bold',
                        fontsize=9, bbox=dict(facecolor='white', alpha=0.8, edgecolor='none', boxstyle='round,pad=0.2'),
                        ha='center', zorder=7)
    
    # Add labels to base box and joints
    ax.text(0, -85.0, "Base Box\n(170mm)", ha='center', va='center', fontsize=9, color='dimgray', rotation=90)
    ax.scatter([0], [d1], color='black', s=80, zorder=10, label='J1 Shoulder')
    
    # Legend
    ax.legend(loc='upper right')
    
    # Colorbar
    cbar = fig.colorbar(cax, ax=ax, shrink=0.8, extend='max', pad=0.04)
    cbar.set_label('Max Load Capacity (grams)', fontsize=12, fontweight='bold', labelpad=10)
    cbar.ax.tick_params(labelsize=10)
    
    # Main Title
    fig.suptitle('Robot Arm Load Capacity Simulation (Pitch = -90°)', 
                 fontsize=16, fontweight='bold', y=0.97)
    
    plt.tight_layout()
    plt.subplots_adjust(top=0.88)
    
    # Save the plot
    image_path = os.path.join(os.path.dirname(__file__), 'workspace_load_capacity.png')
    plt.savefig(image_path, dpi=300, bbox_inches='tight')
    print(f"Saved plot visualization to: {image_path}")
    
    # Show the plot if running in windowed environment
    if 'show' in sys.argv:
        plt.show()


# ==========================================
# 2D PLOT OF TORQUE VS REACH
# ==========================================
def plot_torque_vs_reach():
    """
    Plots the static torque on J1 and J2 as a function of radial reach r
    for different load values (10g, 100g, 200g, 500g) at a constant height.
    """
    reach_r = np.linspace(120.0, 235.0, 100)
    z_target = -100.0  # mm relative to J0
    
    loads = [0.010, 0.100, 0.200, 0.500]  # kg
    colors = ['blue', 'orange', 'green', 'red']
    
    fig, axes = plt.subplots(2, 1, figsize=(10, 10), sharex=True)
    fig.patch.set_facecolor('white')
    
    j1_torques = {ld: [] for ld in loads}
    j2_torques = {ld: [] for ld in loads}
    valid_rs = []
    
    for r in reach_r:
        sols = ik_pitch_90(r, z_target)
        sol = None
        for s in sols:
            if s['type'] == 'Elbow Up':
                sol = s
                break
        
        if sol is None:
            continue
            
        valid_rs.append(r)
        phi1 = sol['phi1']
        phi2 = sol['phi2']
        
        cos_p1 = np.cos(phi1)
        cos_p2 = np.cos(phi2)
        
        for ld in loads:
            # Torque J2
            t2 = cos_p2 * (T2_base + T2_coeff * ld)
            # Torque J1
            t1 = cos_p1 * (T1_base_1 + T1_coeff_1 * ld) + t2
            
            # Convert to kg*cm absolute value
            j1_torques[ld].append(np.abs(t1) / 0.0980665)
            j2_torques[ld].append(np.abs(t2) / 0.0980665)
            
    ax_j1 = axes[0]
    ax_j2 = axes[1]
    
    ax_j1.set_title("Joint 1 (Shoulder) Torque", fontsize=12, fontweight='bold')
    ax_j1.grid(color='gray', linestyle=':', linewidth=0.5, alpha=0.5)
    ax_j1.set_ylabel('Torque |T| (kg·cm)', fontsize=10)
    
    ax_j2.set_title("Joint 2 (Elbow) Torque", fontsize=12, fontweight='bold')
    ax_j2.grid(color='gray', linestyle=':', linewidth=0.5, alpha=0.5)
    ax_j2.set_ylabel('Torque |T| (kg·cm)', fontsize=10)
    ax_j2.set_xlabel('Radial Reach R (mm)', fontsize=10)
    
    if len(valid_rs) == 0:
        ax_j1.text(0.5, 0.5, "No valid configurations\nwithin servo limits\n(0° to 180°)", 
                   ha='center', va='center', color='crimson', fontsize=12, transform=ax_j1.transAxes,
                   bbox=dict(facecolor='#FFF0F0', alpha=0.9, edgecolor='crimson', boxstyle='round,pad=0.5'))
        ax_j2.text(0.5, 0.5, "No valid configurations\nwithin servo limits\n(0° to 180°)", 
                   ha='center', va='center', color='crimson', fontsize=12, transform=ax_j2.transAxes,
                   bbox=dict(facecolor='#FFF0F0', alpha=0.9, edgecolor='crimson', boxstyle='round,pad=0.5'))
    else:
        # Plot limits
        ax_j1.axhline(TORQUE_LIMIT_KG_CM, color='crimson', linestyle='--', linewidth=1.5, label=f'Real Stall Limit ({TORQUE_LIMIT_KG_CM:.1f} kg·cm)')
        ax_j1.axhline(SAFE_CONTINUOUS_KG_CM, color='darkorange', linestyle=':', linewidth=1.5, label=f'Safe Continuous Limit ({SAFE_CONTINUOUS_KG_CM:.1f} kg·cm)')
        ax_j2.axhline(TORQUE_LIMIT_KG_CM, color='crimson', linestyle='--', linewidth=1.5, label=f'Real Stall Limit ({TORQUE_LIMIT_KG_CM:.1f} kg·cm)')
        ax_j2.axhline(SAFE_CONTINUOUS_KG_CM, color='darkorange', linestyle=':', linewidth=1.5, label=f'Safe Continuous Limit ({SAFE_CONTINUOUS_KG_CM:.1f} kg·cm)')
        
        for ld, color in zip(loads, colors):
            ax_j1.plot(valid_rs, j1_torques[ld], color=color, linewidth=2.0, label=f"Load = {ld*1000:.0f}g")
            ax_j2.plot(valid_rs, j2_torques[ld], color=color, linewidth=2.0, label=f"Load = {ld*1000:.0f}g")
            
        ax_j1.legend(loc='upper left')
        ax_j2.legend(loc='upper left')
        
    fig.suptitle(f'Static Joint Torque vs Reach R (Elbow Up, Z = {z_target}mm, Pitch = -90°)', 
                 fontsize=14, fontweight='bold')
    
    plt.tight_layout()
    torque_image_path = os.path.join(os.path.dirname(__file__), 'workspace_torque_curves.png')
    plt.savefig(torque_image_path, dpi=300, bbox_inches='tight')
    print(f"Saved joint torque curve visualization to: {torque_image_path}")


if __name__ == "__main__":
    run_simulation()
    plot_torque_vs_reach()
