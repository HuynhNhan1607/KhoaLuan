"""
Virtual Structure Formation Controller
=======================================
Centroid computed from actual robot positions
"""

import numpy as np
from config import (
    CORNERS, FORMATION_OFFSETS, FORMATION_SPACING,
    VELOCITY_NOMINAL, VELOCITY_MAX, VELOCITY_DEADZONE,
    KP_POSITION, DT
)


def get_path_direction(current_corner_idx):
    """Get unit direction vector for current path segment."""
    if current_corner_idx >= len(CORNERS) - 1:
        return np.array([1.0, 0.0])
    
    start = CORNERS[current_corner_idx]
    end = CORNERS[current_corner_idx + 1]
    direction = end - start
    norm = np.linalg.norm(direction)
    if norm > 0:
        return direction / norm
    return np.array([1.0, 0.0])


def clip_velocity(velocity, max_speed=VELOCITY_MAX):
    """Clip velocity magnitude to [0, max_speed]."""
    speed = np.linalg.norm(velocity)
    if speed < VELOCITY_DEADZONE:
        return np.array([0.0, 0.0])
    if speed > max_speed:
        return velocity / speed * max_speed
    return velocity


def compute_centroid(positions):
    """
    Compute the geometric centroid (center of mass) of positions.
    For a triangle, this is the average of the 3 vertices.
    """
    return np.mean(positions, axis=0)


class VirtualStructureController:
    """
    Virtual Structure Formation Controller.
    
    CORRECT APPROACH:
    - Centroid is COMPUTED from actual robot positions (not predefined)
    - Centroid = geometric center of triangle formed by 3 robots
    - Target: Move centroid along the path
    - Constraints: Each robot must stay within distance limit from centroid
    
    This is realistic - if robots fail to maintain formation, 
    centroid will drift and simulation will show the failure.
    """
    
    def __init__(self, robots):
        self.robots = robots
        self.current_corner_idx = 0
        
        # Desired formation offsets (from centroid)
        # These are the IDEAL offsets, actual positions may deviate
        self.desired_offsets = FORMATION_OFFSETS.copy()
        
        # Distance constraint: max distance from robot to centroid
        self.max_distance_from_centroid = FORMATION_SPACING * 1.5  # 60cm max
        
        # Centroid trajectory history
        self.centroid_trajectory = []
        
        # Initial centroid from actual robot positions
        initial_positions = np.array([r.position for r in robots])
        initial_centroid = compute_centroid(initial_positions)
        self.centroid_trajectory.append(initial_centroid.copy())
        
        print(f"\nVirtual Structure (Centroid-Based):")
        print(f"  Initial Centroid: ({initial_centroid[0]:.2f}, {initial_centroid[1]:.2f})")
        print(f"  Max Distance from Centroid: {self.max_distance_from_centroid*100:.0f}cm")
        print(f"  Desired Offsets:")
        for i, offset in enumerate(self.desired_offsets):
            print(f"    Robot {i+1}: ({offset[0]:.2f}, {offset[1]:.2f}) m")
    
    def get_current_centroid(self):
        """Compute centroid from ACTUAL robot positions."""
        positions = np.array([r.position for r in self.robots])
        return compute_centroid(positions)
    
    def get_measured_centroid(self):
        """Compute centroid from MEASURED robot positions (with sensor error)."""
        positions = np.array([r.measured_position for r in self.robots])
        return compute_centroid(positions)
    
    def update_target(self):
        """Update target corner based on centroid progress."""
        if self.current_corner_idx >= len(CORNERS) - 1:
            return
        
        centroid = self.get_current_centroid()
        current_target = CORNERS[self.current_corner_idx + 1]
        distance_to_target = np.linalg.norm(centroid - current_target)
        
        if distance_to_target < 0.2 and self.current_corner_idx < len(CORNERS) - 2:
            self.current_corner_idx += 1
    
    def check_constraints(self):
        """
        Check if formation constraints are satisfied.
        Returns: (satisfied, violations)
        """
        centroid = self.get_current_centroid()
        violations = []
        
        for i, robot in enumerate(self.robots):
            distance = np.linalg.norm(robot.position - centroid)
            if distance > self.max_distance_from_centroid:
                violations.append({
                    'robot': i,
                    'distance': distance,
                    'limit': self.max_distance_from_centroid
                })
        
        return len(violations) == 0, violations
    
    def compute_velocities(self, current_time):
        """
        Compute velocity commands for all robots.
        
        Strategy:
        1. Compute ACTUAL centroid from robot measured positions
        2. Determine target centroid position (along path)
        3. Each robot: move toward (target_centroid + own_offset)
        4. Apply constraints
        """
        velocities = []
        
        # Update corner target
        self.update_target()
        
        # Path direction
        path_direction = get_path_direction(self.current_corner_idx)
        
        # Target corner for centroid
        if self.current_corner_idx < len(CORNERS) - 1:
            centroid_target = CORNERS[self.current_corner_idx + 1]
        else:
            centroid_target = CORNERS[-1]
        
        # Current centroid from MEASURED positions (what robots perceive)
        current_centroid = self.get_measured_centroid()
        
        # Store for visualization
        self.centroid_trajectory.append(self.get_current_centroid())
        
        # Distance to corner for feedforward scaling
        dist_to_corner = np.linalg.norm(current_centroid - centroid_target)
        ff_scale = np.clip(dist_to_corner / 0.5, 0.0, 1.0)
        
        for i, robot in enumerate(self.robots):
            # Robot's own measured position
            own_measured = robot.measured_position
            
            # Target position = where robot SHOULD be
            # = current_centroid + desired_offset
            # (centroid moves, robot maintains offset)
            
            # We want the centroid to move toward centroid_target
            # So each robot should move toward:
            # target_centroid + own_offset
            
            # Simple approach: robot's target is its ideal position
            # relative to current centroid, plus movement toward path target
            robot_target = current_centroid + self.desired_offsets[i]
            
            # Add feedforward toward path direction
            # (shift the target in path direction)
            ff_shift = VELOCITY_NOMINAL * path_direction * ff_scale * DT * 10
            robot_target = robot_target + ff_shift
            
            # Position error
            position_error = robot_target - own_measured
            error_magnitude = np.linalg.norm(position_error)
            
            # Adaptive gain
            if error_magnitude > FORMATION_SPACING:
                kp_effective = KP_POSITION * 2.0
            else:
                kp_effective = KP_POSITION
            
            # Feedforward velocity (towards path direction)
            v_feedforward = VELOCITY_NOMINAL * path_direction * ff_scale
            
            # Feedback velocity (towards robot's target position)
            v_feedback = kp_effective * position_error
            
            # Combined velocity
            v_cmd = v_feedforward + v_feedback
            v_cmd_clipped = clip_velocity(v_cmd, VELOCITY_MAX)
            
            velocities.append(v_cmd_clipped)
        
        return velocities
    
    def get_formation_error(self):
        """Calculate formation error: deviation from desired offsets."""
        centroid = self.get_current_centroid()
        
        errors = []
        for i, robot in enumerate(self.robots):
            expected = centroid + self.desired_offsets[i]
            actual = robot.position
            error = np.linalg.norm(actual - expected)
            errors.append(error)
        
        return np.array(errors)
