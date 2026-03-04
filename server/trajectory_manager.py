"""
Trajectory Manager Module
Provides path generation utilities for robot navigation.
"""
import numpy as np
import math
from typing import Tuple, List, Optional


# ========== GEOMETRIC OBJECT AVOIDANCE HELPERS ==========

def line_intersects_circle(p1: Tuple[float, float], 
                           p2: Tuple[float, float],
                           center: Tuple[float, float], 
                           radius: float) -> bool:
    """
    Check if line segment from p1 to p2 intersects a circle.
    
    Uses parametric line equation and quadratic formula.
    
    Args:
        p1: Start point (x, y)
        p2: End point (x, y)
        center: Circle center (x, y)
        radius: Circle radius
        
    Returns:
        True if line segment intersects circle
    """
    x1, y1 = p1
    x2, y2 = p2
    cx, cy = center
    
    # Direction vector
    dx = x2 - x1
    dy = y2 - y1
    
    # Vector from p1 to circle center
    fx = x1 - cx
    fy = y1 - cy
    
    # Quadratic coefficients: at² + bt + c = 0
    a = dx*dx + dy*dy
    
    if a < 1e-10:  # p1 == p2
        # Check if point is inside circle
        return (fx*fx + fy*fy) < radius*radius
    
    b = 2 * (fx*dx + fy*dy)
    c = fx*fx + fy*fy - radius*radius
    
    discriminant = b*b - 4*a*c
    
    if discriminant < 0:
        return False  # No intersection
    
    # Check if intersection is within segment (0, 1) with epsilon tolerance
    # Exclude exact endpoints to avoid false positives when goal is on boundary
    sqrt_disc = math.sqrt(discriminant)
    t1 = (-b - sqrt_disc) / (2*a)  # Entry point
    t2 = (-b + sqrt_disc) / (2*a)  # Exit point
    
    # Use epsilon to exclude touches at segment endpoints
    eps = 0.02  # 2% tolerance
    
    # Line CROSSES THROUGH circle if:
    # 1. Entry point is strictly inside segment (not at endpoints)
    # 2. OR line starts inside circle and exits within segment
    enters_inside = (eps < t1 < 1 - eps)
    starts_inside = (t1 < 0) and (eps < t2 < 1 - eps)
    
    return enters_inside or starts_inside



def compute_tangent_waypoints(start: Tuple[float, float],
                               goal: Tuple[float, float],
                               center: Tuple[float, float],
                               radius: float) -> List[Tuple[float, float]]:
    """
    Compute waypoint(s) to navigate around a circular obstacle.
    
    Generates multiple waypoints along the arc to prevent the robot from
    cutting corners and entering the avoidance zone.
    
    Args:
        start: Starting position (x, y)
        goal: Goal position (x, y)
        center: Obstacle center (x, y)
        radius: Avoidance radius (object_radius + robot_radius)
        
    Returns:
        List of waypoint(s) along the arc to navigate around the obstacle
    """
    cx, cy = center
    sx, sy = start
    gx, gy = goal
    
    # Calculate angles from center to start and goal
    angle_start = math.atan2(sy - cy, sx - cx)
    angle_goal = math.atan2(gy - cy, gx - cx)
    
    # Determine angular difference (shortest path)
    angle_diff = angle_goal - angle_start
    while angle_diff > math.pi: angle_diff -= 2*math.pi
    while angle_diff < -math.pi: angle_diff += 2*math.pi
    
    waypoints = []
    
    # Small margin to prevent corner-cutting (10% extra + 5cm fixed)
    # Keep it close to actual avoidance radius for smoother path to approach_entry
    safe_radius = radius * 1.10 + 0.05

    
    # Calculate how many waypoints needed based on arc length
    # More waypoints = smoother path = less corner-cutting
    # Use waypoint every ~30 degrees (0.52 radians) to prevent cutting into zone
    max_angle_per_segment = math.pi / 6  # 30 degrees
    
    abs_diff = abs(angle_diff)
    num_segments = max(1, int(math.ceil(abs_diff / max_angle_per_segment)))
    
    # Generate waypoints along the arc
    for i in range(1, num_segments + 1):
        # Interpolate angle (skip first = start, skip last because goal is added separately)
        t = i / (num_segments + 1)
        angle = angle_start + angle_diff * t
        
        wp_x = cx + safe_radius * math.cos(angle)
        wp_y = cy + safe_radius * math.sin(angle)
        waypoints.append((wp_x, wp_y))
    
    return waypoints



def generate_path_avoiding_object(start_pos: Tuple[float, float],
                                   goal_pos: Tuple[float, float],
                                   object_center: Optional[Tuple[float, float]],
                                   object_radius: float,
                                   robot_radius: float = 0.2,
                                   gripper_length: float = 0.15) -> List[Tuple[float, float]]:
    """
    Generate a path from start to goal that avoids the object using two-phase approach.
    
    Phase 1 (Transit): Stay outside (object_radius + robot_radius) during movement
    Phase 2 (Final Approach): Straight line into the grip position
    
    This handles the case where grip position is INSIDE the transit avoidance zone
    but the robot needs to reach it for grasping.
    
    Args:
        start_pos: Starting position (robot center)
        goal_pos: Goal/grip position (robot center)
        object_center: Object center position (can be None)
        object_radius: Object radius
        robot_radius: Robot body radius (default 0.2m = 20cm)
        gripper_length: Distance from robot center to gripper tip (default 0.15m = 15cm)
        
    Returns:
        List of waypoints [(x, y), ...] from start to goal
    """
    if object_center is None:
        return [start_pos, goal_pos]
    
    cx, cy = object_center
    gx, gy = goal_pos
    sx, sy = start_pos
    
    # Transit avoidance radius (robot body must not touch object during transit)
    transit_avoid_radius = object_radius + robot_radius
    
    # Calculate distance from goal to object center
    goal_dist = math.sqrt((gx - cx)**2 + (gy - cy)**2)
    
    # Calculate distance from start to object center
    start_dist = math.sqrt((sx - cx)**2 + (sy - cy)**2)
    
    # DEBUG: Print key values
    print(f"[ObjAvoid] object_radius={object_radius:.3f}, robot_radius={robot_radius:.3f}")
    print(f"[ObjAvoid] transit_avoid_radius={transit_avoid_radius:.3f}")
    print(f"[ObjAvoid] goal_dist={goal_dist:.3f}, start_dist={start_dist:.3f}")
    
    # Check if start is already inside transit zone (should not happen normally)
    if start_dist < transit_avoid_radius:
        # Start is too close to object - just go straight
        print(f"[ObjAvoid] WARNING: Start inside transit zone! Returning direct path.")
        return [start_pos, goal_pos]

    
    # Check if goal is inside transit avoidance zone (typical for grip positions)
    if goal_dist < transit_avoid_radius:
        # TWO-PHASE APPROACH
        print(f"[ObjAvoid] Goal INSIDE transit zone - using two-phase approach")
        
        # Protect against very small goal_dist (goal too close to object center)
        if goal_dist < 0.1:  # Less than 10cm from object center
            # Goal is essentially at object center - this is unusual, just go direct
            print(f"[ObjAvoid] WARNING: goal_dist < 0.1m, returning direct path")
            return [start_pos, goal_pos]
        
        # Calculate approach entry point: on the edge of transit zone,
        # aligned with the direction from object center to goal
        direction_x = (gx - cx) / goal_dist
        direction_y = (gy - cy) / goal_dist
        
        # Approach entry point is on the transit zone boundary
        approach_entry = (
            cx + direction_x * transit_avoid_radius,
            cy + direction_y * transit_avoid_radius
        )
        print(f"[ObjAvoid] approach_entry={approach_entry}")

        
        # Check if direct path from start to approach_entry crosses object
        crosses = line_intersects_circle(start_pos, approach_entry, object_center, transit_avoid_radius)
        print(f"[ObjAvoid] Path crosses transit zone: {crosses}")
        
        if crosses:
            # Need to go around - generate tangent waypoints
            tangent_points = compute_tangent_waypoints(
                start_pos, approach_entry, object_center, transit_avoid_radius
            )
            # Path: Start → Tangent points → Approach Entry → Goal
            result = [start_pos] + tangent_points + [approach_entry, goal_pos]
            print(f"[ObjAvoid] Generated {len(result)} waypoints (with tangents)")
            return result
        else:
            # Direct path to approach entry is clear
            # Path: Start → Approach Entry → Goal
            result = [start_pos, approach_entry, goal_pos]
            print(f"[ObjAvoid] Generated {len(result)} waypoints (direct to entry)")
            return result
    
    else:
        # Goal is outside transit zone - simple avoidance
        print(f"[ObjAvoid] Goal OUTSIDE transit zone - checking if path crosses")
        crosses = line_intersects_circle(start_pos, goal_pos, object_center, transit_avoid_radius)
        print(f"[ObjAvoid] Path crosses transit zone: {crosses}")
        
        if crosses:
            # Need to go around
            tangent_points = compute_tangent_waypoints(
                start_pos, goal_pos, object_center, transit_avoid_radius
            )
            result = [start_pos] + tangent_points + [goal_pos]
            print(f"[ObjAvoid] Generated {len(result)} waypoints (with tangents)")
            return result
        else:
            # Direct path is clear
            print(f"[ObjAvoid] Direct path is clear, returning 2 waypoints")
            return [start_pos, goal_pos]







def get_test_trajectory(shape, scale=1.0, center=(2.0, 2.0), points=8):
    """
    Generate a test trajectory based on the specified shape.
    
    Args:
        shape (str): Shape type - 'square' or 'circle'
        scale (float): Scaling factor for the trajectory (default: 1.0)
        center (tuple): Center point (x, y) in meters (default: (2.0, 2.0))
        points (int): Number of waypoints to generate (default: 8)
    
    Returns:
        list: List of (x, y) tuples representing waypoints in meters
    """
    trajectory = []
    center_x, center_y = center
    
    if shape == 'square':
        # Generate a square path
        # Calculate square size based on scale
        side_length = scale * 2.0  # 2 meters per side at scale=1.0
        half_side = side_length / 2.0
        
        # Generate points along the perimeter of the square
        points_per_side = max(2, points // 4)
        
        # Top side (left to right)
        for i in range(points_per_side):
            t = i / (points_per_side - 1) if points_per_side > 1 else 0
            x = center_x - half_side + t * side_length
            y = center_y + half_side
            trajectory.append((x, y))
        
        # Right side (top to bottom)
        for i in range(1, points_per_side):
            t = i / (points_per_side - 1) if points_per_side > 1 else 0
            x = center_x + half_side
            y = center_y + half_side - t * side_length
            trajectory.append((x, y))
        
        # Bottom side (right to left)
        for i in range(1, points_per_side):
            t = i / (points_per_side - 1) if points_per_side > 1 else 0
            x = center_x + half_side - t * side_length
            y = center_y - half_side
            trajectory.append((x, y))
        
        # Left side (bottom to top)
        for i in range(1, points_per_side):
            t = i / (points_per_side - 1) if points_per_side > 1 else 0
            x = center_x - half_side
            y = center_y - half_side + t * side_length
            trajectory.append((x, y))
    
    elif shape == 'circle':
        # Generate a circular path
        radius = scale * 1.0  # 1 meter radius at scale=1.0
        
        for i in range(points):
            angle = 2 * math.pi * i / points
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            trajectory.append((x, y))
        
        # Close the circle by adding the first point at the end
        if trajectory:
            trajectory.append(trajectory[0])
    
    else:
        raise ValueError(f"Unknown shape: {shape}. Supported shapes: 'square', 'circle'")
    
    return trajectory


def timestamp_trajectory(path_points, start_time=None, dt=1.0):
    """
    Attach timestamps to a list of (x, y) waypoints.

    Args:
        path_points (list): List of (x, y) tuples
        start_time (float|None): Epoch seconds for the first waypoint. If None, time.time() is used.
        dt (float): Time interval (seconds) between consecutive waypoints.

    Returns:
        list: List of dicts: {"x": float, "y": float, "t": float}

    Notes/Assumptions:
        - This function assumes a simple fixed time spacing between waypoints (dt).
        - If you need per-segment timing based on velocities or distances, compute dt externally
          and pass an appropriate list of timestamps instead.
    """
    import time as _time

    if start_time is None:
        start_time = _time.time()

    stamped = []
    t = float(start_time)
    for (x, y) in path_points:
        stamped.append({"x": float(x), "y": float(y), "t": t})
        t += float(dt)

    return stamped


def generate_approach_trajectory(start_pos, goal_pos, 
                                  initial_heading=0.0, target_heading=0.0,
                                  vector_field_planner=None,
                                  velocity=0.2, start_time=None, min_waypoint_spacing=0.1,
                                  object_center=None, object_radius=0.0, robot_radius=0.2):
    """
    Generate a trajectory from current position to grip position.
    
    This function is used in Phase 1 (Approach) to create paths for robots
    to reach their grip positions around the object.
    
    Uses two-phase approach to avoid object during transit:
    - Phase 1: Stay outside (object_radius + robot_radius) during movement
    - Phase 2: Straight line into the grip position
    
    Args:
        start_pos (tuple): (x, y) starting position in meters
        goal_pos (tuple): (x, y) goal/grip position in meters
        initial_heading (float): Initial robot heading in radians (default: 0.0)
        target_heading (float): Target robot heading in radians (default: 0.0)
        vector_field_planner: Optional VectorFieldPlanner instance for obstacle avoidance
                             If None, generates straight line path
        velocity (float): Desired travel velocity in m/s (default: 0.2)
        start_time (float|None): Epoch timestamp for first waypoint. If None, uses time.time()
        min_waypoint_spacing (float): Minimum distance between waypoints in meters (default: 0.1)
        object_center (tuple): (x, y) object center position for avoidance (default: None)
        object_radius (float): Object radius in meters (default: 0.0)
        robot_radius (float): Robot body radius in meters (default: 0.2)
    
    Returns:
        list: List of dicts: [{"x": float, "y": float, "t": float, "theta": float}, ...]
              Timestamps are absolute (epoch seconds)
    
    Notes:
        - If object_center is provided, uses two-phase approach to avoid object
        - If vector_field_planner is provided, uses potential field path planning
        - Otherwise, generates straight line with evenly spaced waypoints
        - Timestamp intervals are computed based on velocity and distance
    """
    import time as _time
    
    if start_time is None:
        start_time = _time.time()
    
    sx, sy = start_pos
    gx, gy = goal_pos
    
    # Use A* Planner if available
    raw_path = None
    if vector_field_planner is not None:
        # Note: vector_field_planner argument is now expected to be a PathPlanner instance
        # Ensure obstacle is in the map if provided
        if object_center is not None and object_radius > 0:
            vector_field_planner.add_obstacle('circle', object_center[0], object_center[1], object_radius)
            
        raw_path = vector_field_planner.plan_path(start_pos, goal_pos)
        
        # Remove it after planning so it doesn't persist for other unrelated queries?
        # Ideally the map is consistent. For now let's assume it persists or use a temp approach.
        # But since we have multiple robots, we might want it to persist.
        # However, to be safe against duplicate adds, the planner handles it or we leave it.
    
    if raw_path:
        waypoints = raw_path
    else:
        # Fallback to straight line (or 2-phase if we kept the logic, but A* is preferred)
        waypoints = [start_pos, goal_pos]
    
    # Calculate timestamps based on velocity
    trajectory = []
    current_time = start_time
    
    # Convert constraints to detailed trajectory
    # If using A* path, it's already a list of points. We just need to timestamp them.
    # But A* points might be far apart or too close. We should interpolate to match min_waypoint_spacing.
    
    trajectory_points = []
    
    # First, flatten the path into evenly spaced points
    for i in range(len(waypoints) - 1):
        p1 = waypoints[i]
        p2 = waypoints[i+1]
        dist = math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
        
        num_steps = max(1, int(dist / min_waypoint_spacing))
        
        for s in range(num_steps):
            t = s / num_steps
            x = p1[0] + t * (p2[0] - p1[0])
            y = p1[1] + t * (p2[1] - p1[1])
            trajectory_points.append((x, y))
            
    # Add final point
    trajectory_points.append(waypoints[-1])
    
    # Now assign time and theta
    total_dist = 0
    for i in range(1, len(trajectory_points)):
        p1 = trajectory_points[i-1]
        p2 = trajectory_points[i]
        total_dist += math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
        
    total_duration = total_dist / velocity if velocity > 0 else 0
    
    # Shortest angle diff
    theta_diff = target_heading - initial_heading
    while theta_diff > math.pi: theta_diff -= 2*math.pi
    while theta_diff < -math.pi: theta_diff += 2*math.pi
    
    current_dist = 0
    t_start = start_time
    
    for i, p in enumerate(trajectory_points):
        if i > 0:
            p_prev = trajectory_points[i-1]
            dist_step = math.sqrt((p[0]-p_prev[0])**2 + (p[1]-p_prev[1])**2)
            current_dist += dist_step
            
        progress = current_dist / total_dist if total_dist > 0 else 1.0
        
        t = t_start + (current_dist / velocity) if velocity > 0 else t_start
        theta = initial_heading + theta_diff * progress
        
        trajectory.append({
            "x": float(p[0]),
            "y": float(p[1]),
            "theta": float(theta),
            "t": float(t)
        })
            
    return trajectory




def generate_multi_robot_approach_trajectories(robot_positions, grip_positions,
                                               initial_headings=None, target_headings=None,
                                               vector_field_planner=None,
                                               velocity=0.2, start_time=None,
                                               min_waypoint_spacing=0.1,
                                               object_center=None, object_radius=0.0, 
                                               robot_radius=0.2):
    """
    Generate approach trajectories for multiple robots.
    
    All robots start moving at the same time. Each robot's trajectory
    is computed independently with object avoidance.
    
    Args:
        robot_positions (dict): {robot_id: (x, y)} current positions
        grip_positions (dict): {robot_id: (x, y)} target grip positions
        initial_headings (dict): {robot_id: heading} initial headings in radians
        target_headings (dict): {robot_id: heading} target headings in radians
        vector_field_planner: Optional VectorFieldPlanner for obstacle avoidance
        velocity (float): Travel velocity in m/s
        start_time (float|None): Common start time for all robots
        min_waypoint_spacing (float): Minimum waypoint spacing in meters
        object_center (tuple): (x, y) object center for avoidance
        object_radius (float): Object radius in meters
        robot_radius (float): Robot body radius in meters
    
    Returns:
        dict: {robot_id: trajectory_list} where trajectory_list is
              [{"x": float, "y": float, "t": float, "theta": float}, ...]
    """
    import time as _time
    
    if start_time is None:
        start_time = _time.time()
    
    trajectories = {}
    if initial_headings is None: initial_headings = {}
    if target_headings is None: target_headings = {}
    
    for robot_id, start_pos in robot_positions.items():
        if robot_id not in grip_positions:
            continue
        
        goal_pos = grip_positions[robot_id]
        
        trajectory = generate_approach_trajectory(
            start_pos=start_pos,
            goal_pos=goal_pos,
            initial_heading=initial_headings.get(robot_id, 0.0),
            target_heading=target_headings.get(robot_id, 0.0),
            vector_field_planner=vector_field_planner,
            velocity=velocity,
            start_time=start_time,
            min_waypoint_spacing=min_waypoint_spacing,
            object_center=object_center,
            object_radius=object_radius,
            robot_radius=robot_radius
        )
        
        if trajectory is None:
            # Failed to generate path for this robot
            return None
        
        trajectories[robot_id] = trajectory
    
    return trajectories


def generate_safe_approach_path(planner, start_pos: Tuple[float, float], goal_pos: Tuple[float, float], 
                                object_pos: Tuple[float, float], object_size: float, 
                                robot_radius: float = 0.25) -> Optional[List[Tuple[float, float]]]:
    """
    Generate a safe approach path using Two-Phase Approach (Transit -> Grip).
    
    This function handles the geometric constraint where the grip position is inside
    the safety buffer of the object.
    
    Phases:
    1. Transit: A* path to an 'Approach Entry' point outside the collision zone.
       - The object obstacle is inflated by robot_radius to ensure body clearance.
    2. Final Approach: Straight line from Entry to Grip Position.
       - Ignores the object obstacle (assumes safe approach vector).
       
    Args:
        planner: PathPlanner instance used for A* planning
        start_pos: (x, y) Starting position
        goal_pos: (x, y) Target grip position
        object_pos: (x, y) Center of the object
        object_size: Diameter of the object
        robot_radius: Robot body radius for obstacle inflation (default 0.25m)
        
    Returns:
        List of (x, y) waypoints representing the full path, or None if failed.
    """
    obj_x, obj_y = object_pos
    obj_r = object_size / 2.0
    
    # 1. Add Obstacle
    # PathPlanner now handles inflation internally (C-Space)
    planner.add_circular_obstacle(obj_x, obj_y, obj_r)
    
    # Calculate effective radius for checking entry point (Obj + Robot + Margin)
    # The planner inflation is roughly obj_r + robot_radius
    inflated_radius = obj_r + robot_radius
    
    full_path = None
    
    try:
        # 2. Calculate Approach Entry Point
        dx = goal_pos[0] - obj_x
        dy = goal_pos[1] - obj_y
        dist_to_grip = math.sqrt(dx**2 + dy**2)
        
        # Safety radius for Entry Point needs to be slightly outside the inflated obstacle
        # to ensure A* can reach it without collision.
        entry_margin = 0.05
        safe_entry_radius = inflated_radius + entry_margin
        
        if dist_to_grip < safe_entry_radius:
            # Goal is inside safety, use 2-phase
            if dist_to_grip > 0.001:
                scale = safe_entry_radius / dist_to_grip
                entry_x = obj_x + dx * scale
                entry_y = obj_y + dy * scale
            else:
                entry_x, entry_y = goal_pos # Should not happen for grip
            
            entry_pos = (entry_x, entry_y)
            
            # 3. Plan to Entry (Transit Phase)
            print(f"[TrajMgr] Robot@{start_pos} -> Entry@{entry_pos} (Goal@{goal_pos})")
            
            # start_pos might be (x, y, theta), take first 2
            path_to_entry = planner.plan_path((start_pos[0], start_pos[1]), entry_pos)
            
            if path_to_entry:
                # 4. Append Final Approach (Approach Phase)
                full_path = path_to_entry + [goal_pos]
                print(f"[TrajMgr] Path found: {len(path_to_entry)} pts to entry + 1 to goal")
            else:
                # FAILURE CASE: Do NOT fallback to straight line which causes collisions.
                print(f"[TrajMgr] ERROR: No A* path found from {start_pos} to Entry {entry_pos}!")
                print(f"[TrajMgr] Likely blocked by obstacle or start/entry inside inflation zone.")
                full_path = None
        else:
            # Goal is safe (outside inflated radius), plan directly
            print(f"[TrajMgr] Direct Plan: Robot@{start_pos} -> Goal@{goal_pos}")
            full_path = planner.plan_path((start_pos[0], start_pos[1]), goal_pos)
            if not full_path:
                print(f"[TrajMgr] ERROR: No A* path found to Goal!")
            
    finally:
        # 5. Cleanup: Remove the temporary obstacle
        planner.remove_last_obstacle()
        
    return full_path
