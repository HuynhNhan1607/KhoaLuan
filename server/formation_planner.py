"""
Formation Planner Module

Calculates grip positions for 2 or 3 robots around an object.
Provides formation offsets for Virtual Structure control.
"""

import math
from typing import Tuple, Dict, List, Optional


class FormationPlanner:
    """
    Calculate grip positions for robots around an object.
    
    Supports flexible configurations for 2 or 3 robots.
    
    Formation layouts:
    - 3 robots: Equilateral triangle
        - Robot 1: Top (90°)
        - Robot 2: Bottom-left (210°)
        - Robot 3: Bottom-right (330°)
    
    - 2 robots: Opposite sides
        - Robot 1: Top (90°)
        - Robot 2: Bottom (270°)
    
    Attributes:
        num_robots: Number of robots (2 or 3)
        grip_radius: Distance from object center to grip positions
        robot_angles: Dict of robot_id -> angle in degrees
    """
    
    # Default formation angles (degrees from positive X-axis, counter-clockwise)
    DEFAULT_ANGLES_3 = {
        1: 90.0,    # Top
        2: 210.0,   # Bottom-left
        3: 330.0    # Bottom-right
    }
    
    DEFAULT_ANGLES_2 = {
        1: 90.0,    # Top
        2: 270.0    # Bottom
    }
    
    # 1 robot: use robot_id=1 at 270° (bottom) - approaches from below, faces up (θ=0)
    # No rotation needed since initial heading (North) matches target heading
    DEFAULT_ANGLES_1 = {
        1: 180.0    # Bottom - go straight to object, no rotation required
    }
    
    def __init__(self, num_robots: int = 3, grip_radius: float = 0.4):
        """
        Initialize the Formation Planner.
        
        Args:
            num_robots: Number of robots (2 or 3)
            grip_radius: Distance from object center to grip positions (meters)
        """
        self._num_robots = 0
        self.grip_radius = grip_radius
        self.robot_angles = {}
        
        # Set number of robots (validates and sets angles)
        self.set_num_robots(num_robots)
        
        # Active robot IDs (can be subset of configured robots)
        self.active_robots = set()
    
    @property
    def num_robots(self) -> int:
        """Get the number of configured robots."""
        return self._num_robots
    
    def set_num_robots(self, num_robots: int):
        """
        Set the number of robots.
        
        Args:
            num_robots: Number of robots (1, 2, or 3)
            
        Raises:
            ValueError: If num_robots is not 1, 2, or 3
        """
        if num_robots not in (1, 2, 3):
            raise ValueError(f"num_robots must be 1, 2, or 3, got {num_robots}")
        
        self._num_robots = num_robots
        
        # Set default angles based on robot count
        if num_robots == 3:
            self.robot_angles = self.DEFAULT_ANGLES_3.copy()
        elif num_robots == 2:
            self.robot_angles = self.DEFAULT_ANGLES_2.copy()
        else:  # num_robots == 1
            # Single robot: use robot_id=2 at 270° (bottom position)
            # Robot approaches from below → target heading θ=0 (North) → no rotation needed
            self.robot_angles = self.DEFAULT_ANGLES_1.copy()
        
        # Reset active robots to all
        self.active_robots = set(self.robot_angles.keys())
    
    def set_robot_angle(self, robot_id: int, angle_degrees: float):
        """
        Set custom angle for a specific robot.
        
        Args:
            robot_id: Robot identifier
            angle_degrees: Angle in degrees from positive X-axis
        """
        if robot_id not in self.robot_angles:
            raise ValueError(f"Robot {robot_id} not configured. "
                           f"Valid IDs: {list(self.robot_angles.keys())}")
        self.robot_angles[robot_id] = angle_degrees
    
    def set_active_robots(self, robot_ids: List[int]):
        """
        Set which robots are active (connected).
        
        Only active robots will have grip positions computed.
        
        Args:
            robot_ids: List of active robot IDs
        """
        self.active_robots = set(robot_ids)
    
    def compute_grip_positions(self, object_pos: Tuple[float, float],
                               object_length: Optional[float] = None,
                               object_width: Optional[float] = None) -> Dict[int, Tuple[float, float]]:
        """
        Compute grip positions for all active robots.
        
        Args:
            object_pos: (x, y) center position of object in meters
            object_length: Optional object length in meters (X-axis, for future rectangular support)
            object_width: Optional object width in meters (Y-axis, for future rectangular support)
            
        Returns:
            Dict mapping robot_id to (x, y) grip position
        """
        obj_x, obj_y = object_pos
        grip_positions = {}
        
        for robot_id in self.active_robots:
            if robot_id not in self.robot_angles:
                continue
            
            angle_deg = self.robot_angles[robot_id]
            angle_rad = math.radians(angle_deg)
            
            # Calculate grip position at grip_radius from object center
            grip_x = obj_x + self.grip_radius * math.cos(angle_rad)
            grip_y = obj_y + self.grip_radius * math.sin(angle_rad)
            
            grip_positions[robot_id] = (grip_x, grip_y)
        
        return grip_positions
    
    def compute_formation_offsets(self) -> Dict[int, Tuple[float, float]]:
        """
        Compute formation offsets relative to centroid.
        
        These offsets are used in Virtual Structure control (Phase 2).
        The centroid is the average position of all robots in formation.
        
        Returns:
            Dict mapping robot_id to (dx, dy) offset from centroid
        """
        # First, compute positions around origin (object at 0,0)
        positions = self.compute_grip_positions((0.0, 0.0))
        
        if not positions:
            return {}
        
        # Calculate centroid
        n = len(positions)
        cx = sum(pos[0] for pos in positions.values()) / n
        cy = sum(pos[1] for pos in positions.values()) / n
        
        # Compute offsets from centroid
        offsets = {}
        for robot_id, (px, py) in positions.items():
            offsets[robot_id] = (px - cx, py - cy)
        
        return offsets
    
    def compute_approach_angles(self, object_pos: Tuple[float, float], heading_convention: str = "Y") -> Dict[int, float]:
        """
        Compute the angles robots should face when at grip positions.
        
        Robots should face toward the object center.
        
        Args:
            object_pos: (x, y) object center position
            heading_convention: 'X' for standard (0=East), 'Y' for Y-axis heading (0=North)
            
        Returns:
            Dict mapping robot_id to approach angle in radians
        """
        obj_x, obj_y = object_pos
        grip_positions = self.compute_grip_positions(object_pos)
        
        approach_angles = {}
        for robot_id, (gx, gy) in grip_positions.items():
            # Angle from grip position toward object
            dx = obj_x - gx
            dy = obj_y - gy
            angle = math.atan2(dy, dx)
            
            if heading_convention == "Y":
                # If Heading coincides with Y-axis (Theta=0 -> North)
                # Standard atan2 gives angle from East (0)
                # We need to subtract 90 degrees (pi/2)
                angle = angle - (math.pi / 2.0)
            
            # Normalize to -pi..pi
            while angle > math.pi: angle -= 2 * math.pi
            while angle < -math.pi: angle += 2 * math.pi
            
            approach_angles[robot_id] = angle
        
        return approach_angles
    
    def get_formation_info(self) -> Dict:
        """
        Get current formation configuration info.
        
        Returns:
            Dict with formation details for debugging/display
        """
        return {
            'num_robots': self._num_robots,
            'grip_radius': self.grip_radius,
            'robot_angles': self.robot_angles.copy(),
            'active_robots': list(self.active_robots),
            'offsets': self.compute_formation_offsets()
        }
    
    def validate_formation(self, grip_positions: Dict[int, Tuple[float, float]],
                          map_bounds: Tuple[float, float, float, float] = (0, 0, 10, 10)) -> Dict[int, bool]:
        """
        Validate that grip positions are within map bounds.
        
        Args:
            grip_positions: Dict of robot_id to (x, y) positions
            map_bounds: (x_min, y_min, x_max, y_max) map boundaries
            
        Returns:
            Dict mapping robot_id to validity (True if valid)
        """
        x_min, y_min, x_max, y_max = map_bounds
        validity = {}
        
        for robot_id, (x, y) in grip_positions.items():
            valid = (x_min <= x <= x_max) and (y_min <= y <= y_max)
            validity[robot_id] = valid
        
        return validity


# Singleton instance
_default_planner = None

def get_formation_planner(num_robots: int = 3, grip_radius: float = 0.4) -> FormationPlanner:
    """
    Get or create the default FormationPlanner instance.
    
    Args:
        num_robots: Number of robots
        grip_radius: Grip radius in meters
        
    Returns:
        FormationPlanner instance
    """
    global _default_planner
    if _default_planner is None:
        _default_planner = FormationPlanner(num_robots, grip_radius)
    return _default_planner
