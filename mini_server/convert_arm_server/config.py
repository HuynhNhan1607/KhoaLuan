"""
Robot Configuration Module
Contains all robot parameters, servo mapping, and default settings
"""

import os
import json

# ===== Robot Kinematics Parameters (Paper-based Model) =====
# Units: mm, degrees
d1 = 32.0      # Vertical offset (base height)
a2 = 105.0     # Link 1 length (shoulder to elbow)
a3 = 130.0      # Link 2 length (elbow to wrist)
d5 = 140.0     # Tool length (wrist to TCP)

# Legacy names for compatibility
D1 = d1
L1 = a2
L2 = a3
LT = d5

# ===== Joint Limits (DH Angles) =====
JOINT_LIMITS_DEG = {
    "theta1": (0.0, 360.0),   # base yaw
    "theta2": (-180, 180),    # shoulder
    "theta3": (-180, 180),    # elbow
    "theta4": (-180, 180),    # wrist pitch
    "theta5": (0.0, 180.0),   # wrist roll
}

# ===== File Paths =====
DEFAULT_ANGLES_FILE = "default_angles.json"
JOINT_LIMITS_FILE = "joint_limits.json"

# ===== Default Joint Angles (degrees) =====
DEFAULT_JOINT_ANGLES = {
    "j0": 90,
    "j1": 90,
    "j2": 135,
    "j3": 90,
    "j4": 90,
    "j5": 90
}

# ===== Default Joint Limits (Servo Angles) =====
DEFAULT_JOINT_LIMITS = {
    "j0": {"min": 0, "max": 180, "min_us": 450, "max_us": 2500, "name": "Base Rotation"},
    "j1": {"min": 0, "max": 180, "min_us": 450, "max_us": 2500, "name": "Shoulder Pitch"},
    "j2": {"min": 0, "max": 180, "min_us": 450, "max_us": 2500, "name": "Elbow"},
    "j3": {"min": 0, "max": 180, "min_us": 450, "max_us": 2500, "name": "Wrist Pitch"},
    "j4": {"min": 0, "max": 180, "min_us": 450, "max_us": 2500, "name": "Wrist Roll"},
    "j5": {"min": 0, "max": 180, "min_us": 450, "max_us": 2500, "name": "Gripper"}
}

# ===== Servo Mapping Configuration =====
# Maps DH/Math angles to physical servo angles
SERVO_MAPPING_CONFIG = {
    "j0": {"offset": 0,  "dir": 1},    # Base - offset 90 allows -90° to +90° math range
    "j1": {"offset": 90, "dir": 1},   # Shoulder
    "j2": {"offset": 90, "dir": 1},   # Elbow
    "j3": {"offset": 90, "dir": 1},   # Wrist Pitch
    "j4": {"offset": 0,  "dir": 1},    # Wrist Roll
    "j5": {"offset": 0,  "dir": 1}     # Gripper
}

# ===== Helper Functions for Config =====
def load_default_angles():
    """Load default angles from file or return defaults"""
    angles = DEFAULT_JOINT_ANGLES.copy()
    if os.path.exists(DEFAULT_ANGLES_FILE):
        try:
            with open(DEFAULT_ANGLES_FILE, 'r') as f:
                angles = json.load(f)
        except Exception:
            pass
    return angles

def load_joint_limits():
    """Load joint limits from file or return defaults"""
    limits = DEFAULT_JOINT_LIMITS.copy()
    if os.path.exists(JOINT_LIMITS_FILE):
        try:
            with open(JOINT_LIMITS_FILE, 'r') as f:
                limits = json.load(f)
        except Exception:
            pass
    return limits

def save_default_angles(angles):
    """Save default angles to file"""
    with open(DEFAULT_ANGLES_FILE, 'w') as f:
        json.dump(angles, f, indent=2)

def save_joint_limits(limits):
    """Save joint limits to file"""
    with open(JOINT_LIMITS_FILE, 'w') as f:
        json.dump(limits, f, indent=2)

# ===== Pick and Place Configuration =====
PICK_PLACE_CONFIG = {
    # Rest position (SERVO angles directly, NOT math angles)
    # J0=90° faces Y-axis, J1-J3=170° folds arm back
    "J0_REST": 90,              # Base rotation to face Y-axis
    "REST_ANGLES_SERVO": {      # These are SERVO angles (0-180°)
        "j1": 170,
        "j2": 170,
        "j3": 170
    },
    
    # Gripper parameters
    "GRIPPER_OPEN": 40,        # J5 angle for fully open
    "GRIPPER_CLOSED": 90,     # J5 angle for fully closed
    "GRIPPER_WAIT_MS": 500,    # ms to wait after closing gripper
    
    # Movement parameters
    "LIFT_HEIGHT": 50,         # mm - lift height after picking
    "APPROACH_HEIGHT": 200,     # mm - height to hover ABOVE object before descending
    
    # Fixed angles
    "PITCH": -90,              # Gripper always points straight down
    
    # Timing
    "MOVE_DELAY_MS": 300,      # Delay between major steps
    "JOINT_DELAY_MS": 200,     # Delay between individual joint movements
}

