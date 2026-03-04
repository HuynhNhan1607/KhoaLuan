"""
Configuration Parameters for Multi-Agent Formation Control
===========================================================
Virtual Structure Control with Event-Triggered Communication
"""

import numpy as np

# =============================================================================
# TIME PARAMETERS
# =============================================================================
DT = 0.01  # Physics timestep (s) - 100Hz
SIMULATION_TIME = 45.0  # Total simulation time (s)

# =============================================================================
# SENSOR MODEL
# =============================================================================
SENSOR_UPDATE_PERIOD = 0.05  # 50ms = 20Hz update rate

# =============================================================================
# REAL-WORLD FAILURE MODE A: Variable Latency (Time Sync Jitter)
# =============================================================================
# Chrony sync drifts and network jitter causes variable latency
# Instead of fixed 10ms, latency varies between 10ms and 100ms
LATENCY_MIN = 0.01   # 10ms minimum latency
LATENCY_MAX = 0.1    # 100ms maximum latency (10x worse case)

# =============================================================================
# REAL-WORLD FAILURE MODE B: Sensor Outliers (Glitches)
# =============================================================================
# UWB multipath or vision glare causes sudden position spikes
# Simulates "teleportation" glitch in sensor reading
OUTLIER_PROBABILITY = 0.01   # 1% chance per sensor update
OUTLIER_MAGNITUDE = 0.5      # 50cm sudden jump

# =============================================================================
# EVENT-TRIGGERED COMMUNICATION
# =============================================================================
TRIGGER_THRESHOLD = 0.05  # 5cm - Trigger broadcast if prediction error exceeds this

# =============================================================================
# ERROR MODEL (Heading Drift - Rotational Bias)
# =============================================================================
# EKF trajectory is accurate in robot frame but rotated in global frame
# This simulates IMU/Odometry heading bias
HEADING_BIAS_STD = np.radians(0.3)  # 0.3 degrees standard deviation
MEASUREMENT_NOISE_STD = 0.02  # 10cm Gaussian noise (smooth EKF)

# =============================================================================
# VELOCITY CONSTRAINTS
# =============================================================================
VELOCITY_NOMINAL = 0.2  # Base/Feedforward speed (m/s)
VELOCITY_MAX = 0.3  # Hard maximum velocity (m/s)
VELOCITY_DEADZONE = 0.02  # Below this, robot stops (m/s)

# =============================================================================
# FORMATION PARAMETERS
# =============================================================================
NUM_ROBOTS = 3
FORMATION_SPACING = 0.4  # 40cm spacing from center to each robot

# Formation offsets in FIXED world frame (relative to virtual center)
# Triangle formation pointing UP
FORMATION_OFFSETS = [
    np.array([0.0, 0.4]),    # Robot 1: Top (0.4m above center)
    np.array([-0.35, -0.2]), # Robot 2: Bottom-Left
    np.array([0.35, -0.2]),  # Robot 3: Bottom-Right
]

# =============================================================================
# PATH PARAMETERS (S-Curve / Snake)
# =============================================================================
PATH_LENGTH = 5.0  # Horizontal segment length (m)
PATH_HEIGHT = 2.5  # Vertical segment height (m)

CORNERS = np.array([
    [5.0, 5.0],   # Start: top right
    [0.0, 5.0],   # Go left
    [0.0, 2.5],   # Go down
    [5.0, 2.5],   # Go right
    [5.0, 0.0],   # Go down
    [0.0, 0.0],   # Go left (end: bottom left)
])

# =============================================================================
# CONTROLLER GAINS
# =============================================================================
KP_POSITION = 2.5  # Proportional gain for position control

# =============================================================================
# ANIMATION PARAMETERS
# =============================================================================
ANIMATION_INTERVAL = 50  # Animation frame interval (ms) - 20 FPS
SPEED_FACTOR = 5  # Number of physics steps per animation frame

# =============================================================================
# VISUALIZATION
# =============================================================================
FLASH_DURATION = 0.1  # Duration of yellow flash (seconds)
FLASH_RADIUS = 0.25  # Radius of flash circle (m)
FORMATION_STAMP_INTERVAL = 0.5  # Stamp formation ghost line every 0.5 seconds
ROBOT_MARKER_SIZE = 150  # Robot marker size for scatter plot

# Robot colors
ROBOT_COLORS = ['#0072BD', '#77AC30', '#D95319']  # Blue, Green, Red
ROBOT_LABELS = ['Robot 1', 'Robot 2', 'Robot 3']
