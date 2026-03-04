"""
Robot Model with Event-Triggered Communication
===============================================
Mecanum Robot with Heading Drift Error Model, Variable Latency, and Sensor Outliers

REAL-WORLD FAILURE MODES:
A) Variable Latency Jitter: Latency varies 10-100ms (not fixed)
B) Sensor Outliers: 1% chance of 50cm position glitch
"""

import numpy as np
from config import (
    DT, SENSOR_UPDATE_PERIOD, 
    LATENCY_MIN, LATENCY_MAX,
    HEADING_BIAS_STD, MEASUREMENT_NOISE_STD,
    TRIGGER_THRESHOLD, FLASH_DURATION,
    OUTLIER_PROBABILITY, OUTLIER_MAGNITUDE
)


class MecanumRobotETC:
    """
    Mecanum Robot with Event-Triggered Communication (ETC).
    
    Error Models:
    1. Heading Bias (Rotational Drift) - IMU/Odometry heading drift
    2. Variable Latency (Jitter) - Network/Chrony sync jitter
    3. Sensor Outliers (Glitches) - UWB multipath, vision glare
    
    Predictor-Corrector Scheme:
    - Prediction: Dead reckoning with last broadcasted velocity
    - Trigger: Broadcast when |Sensor - Predicted| > threshold
    """
    
    def __init__(self, robot_id, initial_position):
        self.id = robot_id
        self.position = np.array(initial_position, dtype=float)
        self.velocity = np.array([0.0, 0.0])
        self.heading = 0.0  # Fixed (Mecanum)
        
        # Store initial position for heading bias calculation
        self.initial_position = np.array(initial_position, dtype=float)
        
        # Trajectory history
        self.trajectory = [self.position.copy()]
        
        # === ERROR MODEL: HEADING BIAS (Rotational Drift) ===
        self.heading_bias = np.random.normal(0, HEADING_BIAS_STD)
        
        # Pre-compute rotation matrix
        cos_bias = np.cos(-self.heading_bias)
        sin_bias = np.sin(-self.heading_bias)
        self.rotation_matrix = np.array([
            [cos_bias, -sin_bias],
            [sin_bias, cos_bias]
        ])
        
        # === JITTER BUFFER (Variable Latency) ===
        # Store (timestamp, position) pairs instead of fixed deque
        # This allows retrieving position at arbitrary past time
        self.position_history = []  # List of (time, position)
        self.max_history_duration = LATENCY_MAX * 2  # Keep 2x max latency
        
        # === SENSOR MODEL (20Hz with Zero-Order Hold) ===
        self.sensor_update_timer = 0.0
        self.measured_position = self.position.copy()
        
        # === EVENT-TRIGGERED COMMUNICATION ===
        self.last_broadcast_pos = self.position.copy()
        self.last_broadcast_vel = np.array([0.0, 0.0])
        self.last_broadcast_time = 0.0
        
        # Communication statistics
        self.total_broadcasts = 0
        self.broadcast_events = []
        
        # Outlier statistics
        self.total_outliers = 0
        
        # Flash state for visualization
        self.flash_timer = 0.0
        self.is_flashing = False
        
        # Initial broadcast
        self._broadcast(0.0)
    
    def _add_to_history(self, current_time):
        """Add current position to timestamped history."""
        self.position_history.append((current_time, self.position.copy()))
        
        # Clean old entries (keep only recent history)
        cutoff_time = current_time - self.max_history_duration
        self.position_history = [
            (t, p) for t, p in self.position_history if t >= cutoff_time
        ]
    
    def _get_delayed_position(self, current_time):
        """
        Get position with VARIABLE latency (Jitter).
        
        Instead of fixed 10ms latency, randomly sample 10-100ms latency
        and interpolate position from history.
        """
        # Random latency in range [LATENCY_MIN, LATENCY_MAX]
        random_latency = np.random.uniform(LATENCY_MIN, LATENCY_MAX)
        target_time = current_time - random_latency
        
        if len(self.position_history) == 0:
            return self.position.copy()
        
        # Find positions around target_time and interpolate
        # If target_time is before all history, use oldest
        if target_time <= self.position_history[0][0]:
            return self.position_history[0][1].copy()
        
        # If target_time is after all history, use newest
        if target_time >= self.position_history[-1][0]:
            return self.position_history[-1][1].copy()
        
        # Linear interpolation between two closest points
        for i in range(len(self.position_history) - 1):
            t1, p1 = self.position_history[i]
            t2, p2 = self.position_history[i + 1]
            
            if t1 <= target_time <= t2:
                # Interpolate
                alpha = (target_time - t1) / (t2 - t1) if t2 > t1 else 0
                return p1 + alpha * (p2 - p1)
        
        # Fallback
        return self.position_history[-1][1].copy()
    
    def _update_measurement(self, current_time):
        """
        Update sensor measurement (called at 20Hz).
        
        Includes:
        1. Variable Latency (Jitter): 10-100ms random delay
        2. Heading Bias: Rotational drift
        3. Measurement Noise: Gaussian noise
        4. Outlier Injection: 1% chance of 50cm glitch
        """
        # Get delayed position (with jitter)
        delayed_pos = self._get_delayed_position(current_time)
        
        # Vector from initial position
        displacement = delayed_pos - self.initial_position
        
        # Apply rotation (heading bias)
        rotated_displacement = self.rotation_matrix @ displacement
        rotated_pos = self.initial_position + rotated_displacement
        
        # Add measurement noise
        noise = np.random.normal(0, MEASUREMENT_NOISE_STD, 2)
        self.measured_position = rotated_pos + noise
        
        # === FAILURE MODE B: SENSOR OUTLIERS (Glitches) ===
        if np.random.random() < OUTLIER_PROBABILITY:
            # Add random 50cm jump in random direction
            outlier_angle = np.random.uniform(0, 2 * np.pi)
            outlier_vector = OUTLIER_MAGNITUDE * np.array([
                np.cos(outlier_angle),
                np.sin(outlier_angle)
            ])
            self.measured_position += outlier_vector
            self.total_outliers += 1
            # Note: This will likely trigger ETC broadcast!
        
        # Check trigger condition
        self._check_trigger(current_time)
    
    def get_predicted_position(self, current_time):
        """Get predicted position using dead reckoning."""
        dt = current_time - self.last_broadcast_time
        predicted = self.last_broadcast_pos + self.last_broadcast_vel * dt
        return predicted
    
    def _check_trigger(self, current_time):
        """Check if broadcast should be triggered."""
        predicted_pos = self.get_predicted_position(current_time)
        prediction_error = np.linalg.norm(self.measured_position - predicted_pos)
        
        if prediction_error > TRIGGER_THRESHOLD:
            self._broadcast(current_time)
    
    def _broadcast(self, current_time):
        """Broadcast current position and velocity."""
        self.last_broadcast_pos = self.measured_position.copy()
        self.last_broadcast_vel = self.velocity.copy()
        self.last_broadcast_time = current_time
        self.total_broadcasts += 1
        self.broadcast_events.append((current_time, self.position.copy()))
        
        # Trigger flash
        self.flash_timer = FLASH_DURATION
        self.is_flashing = True
    
    def get_broadcast_state(self):
        """Get the last broadcasted state."""
        return self.last_broadcast_pos.copy(), self.last_broadcast_vel.copy()
    
    def set_velocity(self, vx, vy):
        """Set robot velocity command."""
        self.velocity = np.array([vx, vy])
    
    def update(self, dt, current_time):
        """Update robot state (100Hz physics)."""
        # Update position
        self.position += self.velocity * dt
        
        # Add to timestamped history (for jitter buffer)
        self._add_to_history(current_time)
        
        # Check sensor update (20Hz)
        self.sensor_update_timer += dt
        if self.sensor_update_timer >= SENSOR_UPDATE_PERIOD:
            self.sensor_update_timer = 0.0
            self._update_measurement(current_time)
        
        # Update flash timer
        if self.is_flashing:
            self.flash_timer -= dt
            if self.flash_timer <= 0:
                self.is_flashing = False
                self.flash_timer = 0.0
        
        # Store trajectory
        self.trajectory.append(self.position.copy())
