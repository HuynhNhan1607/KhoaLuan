"""
Synchronized Trajectory Generator Module

Generates collision-free trajectories for multiple robots using Co-Simulation
with Dynamic Repulsive Force approach. This is optimized for holonomic robots
like Mecanum-wheeled platforms.

Key Features:
- Real-time collision detection between robots
- Dynamic Repulsive Force for smooth obstacle avoidance
- Priority-based collision resolution (lower ID = higher priority)
- Compatible with VectorFieldPlanner for static obstacle avoidance
- Two-phase object avoidance: transit around object, then approach grip position
"""

import math
import time
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass

# Import object avoidance helpers from trajectory_manager





@dataclass
class RobotState:
    """Represents the state of a robot at a given time."""
    x: float
    y: float
    theta: float = 0.0
    reached_goal: bool = False


class CollisionChecker:
    """
    Handles collision detection and repulsive force computation between robots.
    
    Uses a potential field approach where robots exert repulsive forces on
    each other when within the influence radius.
    """
    
    def __init__(self, safety_radius: float = 0.4, influence_radius: float = 0.6):
        """
        Initialize the collision checker.
        
        Args:
            safety_radius: Minimum allowed distance between robots (meters)
            influence_radius: Distance at which repulsive force starts (meters)
        """
        self.safety_radius = safety_radius
        self.influence_radius = influence_radius
    
    def get_distance(self, pos1: Tuple[float, float], pos2: Tuple[float, float]) -> float:
        """Calculate Euclidean distance between two positions."""
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
    
    def check_collision(self, pos1: Tuple[float, float], pos2: Tuple[float, float]) -> bool:
        """
        Check if two robots are in collision.
        
        Returns:
            True if distance < safety_radius (collision detected)
        """
        return self.get_distance(pos1, pos2) < self.safety_radius
    
    def compute_repulsive_force(self, 
                                  robot_pos: Tuple[float, float],
                                  other_pos: Tuple[float, float],
                                  repulsive_gain: float = 1.0) -> Tuple[float, float]:
        """
        Compute repulsive force on robot from another robot.
        
        Uses stronger exponential force that increases rapidly at close distances.
        Direction is from other robot toward this robot (pushing away).
        
        Args:
            robot_pos: Position of the robot being pushed
            other_pos: Position of the robot exerting the force
            repulsive_gain: Force multiplier (k in F = k/d²)
            
        Returns:
            Tuple (fx, fy) force components
        """
        dx = robot_pos[0] - other_pos[0]
        dy = robot_pos[1] - other_pos[1]
        distance = math.sqrt(dx**2 + dy**2)
        
        # No force if outside influence radius
        if distance >= self.influence_radius:
            return (0.0, 0.0)
        
        # Prevent division by zero
        if distance < 0.01:
            distance = 0.01
        
        # Stronger force formula: exponential increase at close distances
        # F = k * (influence_radius - d)² / d²
        # This creates much stronger forces when robots are close
        margin = self.influence_radius - distance
        force_magnitude = repulsive_gain * (margin * margin) / (distance * distance)
        
        # Extra strong force when inside safety radius (critical zone)
        if distance < self.safety_radius:
            critical_factor = (self.safety_radius / distance) ** 2
            force_magnitude *= critical_factor * 3.0  # Triple the force in critical zone
        
        # Normalize direction
        nx = dx / distance
        ny = dy / distance
        
        # Apply tangential component for smoother avoidance
        # Tangent direction helps robots pass around each other
        tx = -ny  # Perpendicular to radial direction
        ty = nx
        
        # Adaptive mixing: more tangential when close, more radial when far
        radial_ratio = 0.5 + 0.4 * (distance / self.influence_radius)
        tangent_ratio = 1.0 - radial_ratio
        
        fx = force_magnitude * (radial_ratio * nx + tangent_ratio * tx)
        fy = force_magnitude * (radial_ratio * ny + tangent_ratio * ty)
        
        return (fx, fy)


class CoSimulator:
    """
    Co-Simulation engine for multi-robot trajectory generation.
    
    Simulates all robots moving simultaneously, applying attractive forces
    toward goals and repulsive forces from other robots to ensure
    collision-free trajectories.
    
    Supports two-phase object avoidance: robots navigate around the object
    during transit, then approach their grip positions.
    """
    
    def __init__(self, 
                 static_paths: Dict[int, List[Tuple[float, float]]] = None,
                 path_planner=None,  # Added path_planner
                 safety_radius: float = 0.4,
                 influence_radius: float = 0.6,
                 repulsive_gain: float = 1.0, 
                 time_step: float = 0.1,
                 velocity: float = 0.2,
                 goal_tolerance: float = 1e-10,
                 object_center: Optional[Tuple[float, float]] = None,
                 object_radius: float = 0.0,
                 robot_radius: float = 0.2):
        
        self.static_paths = static_paths if static_paths else {}
        self.path_planner = path_planner  # Store planner
        self.safety_radius = safety_radius
        self.influence_radius = influence_radius
        self.time_step = time_step
        self.velocity = velocity
        self.goal_tolerance = goal_tolerance
        self.robot_radius = robot_radius
        
        # Waypoint tracking
        self.current_waypoint_idx: Dict[int, int] = {}
        self.collision_checker = CollisionChecker(safety_radius, influence_radius)
        
    def _compute_safe_velocity(self,
                               robot_id: int,
                               states: Dict[int, RobotState],
                               target_pos: Tuple[float, float]) -> Tuple[float, float]:
        """
        Compute safe velocity. 
        Priority: 
        1. Avoid Static Obstacles (Absolute)
        2. Avoid Dynamic Obstacles (Repulsive)
        3. Seek Goal (Attractive)
        """
        state = states[robot_id]
        pos = (state.x, state.y)
        
        # 1. Attractive Vector (Goal Seeking)
        dx = target_pos[0] - pos[0]
        dy = target_pos[1] - pos[1]
        dist = math.sqrt(dx**2 + dy**2)
        
        if dist < 0.001:
            return (0.0, 0.0)
            
        # Normalize
        dir_x = dx / dist
        dir_y = dy / dist
        
        # 2. Repulsive Vector (Dynamic Avoidance)
        # Only apply if we are actually moving
        rep_x, rep_y = 0.0, 0.0
        
        for other_id, other_state in states.items():
            if other_id == robot_id: continue
            other_pos = (other_state.x, other_state.y)
            fx, fy = self.collision_checker.compute_repulsive_force(pos, other_pos, repulsive_gain=0.5)
            rep_x += fx
            rep_y += fy
            
        # 3. Combine Vectors (Candidate 1: Steered)
        cmd_x = dir_x * self.velocity + rep_x
        cmd_y = dir_y * self.velocity + rep_y
        
        # Normalize candidate
        cmd_speed = math.sqrt(cmd_x**2 + cmd_y**2)
        if cmd_speed > 0.01:
            scale = self.velocity / cmd_speed
            if cmd_speed > self.velocity:
                 cmd_x *= scale
                 cmd_y *= scale
        else:
            cmd_x, cmd_y = 0.0, 0.0

        # 4. Check Static Obstacle Collision (Grid Map)
        # Project future position
        future_x = pos[0] + cmd_x * self.time_step
        future_y = pos[1] + cmd_y * self.time_step
        
        # If we have a planner, verify this candidate path
        if self.path_planner:
            # Check if center is in obstacle (or close to it)
            # Use is_obstacle logic which handles bounds
            # Note: PathPlanner stores inflated map, so checking center point is sufficient for 'robot_radius' clearance
            if self.path_planner.is_obstacle(future_x, future_y):
                # Steering pushes us into wall! 
                # Fallback: Ignore repulsive force, try pure A* direction (since A* is guaranteed safe statically)
                cmd_x = dir_x * self.velocity
                cmd_y = dir_y * self.velocity
                
                # Verify fallback
                future_x = pos[0] + cmd_x * self.time_step
                future_y = pos[1] + cmd_y * self.time_step
                if self.path_planner.is_obstacle(future_x, future_y):
                    # Even pure A* direction is blocked? (Maybe we drifted)
                    # Force Stop (Unreachable condition)
                    return (0.0, 0.0) # Will cause timeout/fail later

        # 5. Final Dynamic Safety Check
        # If resultant moves us INTO collision with another robot, we must stop.
        for other_id, other_state in states.items():
            if other_id == robot_id: continue
            other_pos = (other_state.x, other_state.y)
            future_dist = math.sqrt((future_x - other_pos[0])**2 + (future_y - other_pos[1])**2)
            
            if future_dist < self.safety_radius:
                # Collision detected -> STOP (Unreachable)
                # User Policy: "Don't stop, find another road". 
                # Since we can't re-plan here easily, returning (0,0) will effectively stop.
                # To trigger "Unreachable", the simulation loop needs to detect stuck state.
                return (0.0, 0.0)
                
        return (cmd_x, cmd_y)

    
    def simulate(self,
                 robot_positions: Dict[int, Tuple[float, float]],
                 goal_positions: Dict[int, Tuple[float, float]],
                 initial_headings: Dict[int, float] = None,
                 target_headings: Dict[int, float] = None,
                 start_time: Optional[float] = None,
                 max_steps: int = 2000,
                 min_waypoint_spacing: float = 0.05) -> Dict[int, List[Dict]]:
        """
        Run the co-simulation and generate synchronized trajectories.
        
        Args:
            robot_positions: {robot_id: (x, y)} starting positions
            goal_positions: {robot_id: (x, y)} goal positions
            start_time: Epoch timestamp for trajectory start
            max_steps: Maximum simulation steps
            
        Returns:
            Dict {robot_id: trajectory_list} where trajectory_list is
            [{"x": float, "y": float, "t": float}, ...]
        """
        if start_time is None:
            start_time = time.time()
        
        # Validate robot IDs
        valid_robot_ids = [rid for rid in robot_positions.keys() if rid in goal_positions]
        
        if not valid_robot_ids:
            return {}
        
        # Initialize robot states
        states: Dict[int, RobotState] = {}
        initial_dists: Dict[int, float] = {}
        
        # Use provided static paths or create 2-point paths (start->goal)
        active_paths = {}
        
        for robot_id in valid_robot_ids:
            pos = robot_positions[robot_id]
            goal = goal_positions[robot_id]
            theta = initial_headings.get(robot_id, 0.0) if initial_headings else 0.0
            states[robot_id] = RobotState(x=pos[0], y=pos[1], theta=theta)
            
            dist = math.sqrt((goal[0] - pos[0])**2 + (goal[1] - pos[1])**2)
            initial_dists[robot_id] = max(0.001, dist)
            
            # Setup path tracking
            if self.static_paths and robot_id in self.static_paths and self.static_paths[robot_id]:
                active_paths[robot_id] = self.static_paths[robot_id] # Use provided A* path
                print(f"[CoSim] R{robot_id}: Using A* path ({len(active_paths[robot_id])} pts)")
            else:
                active_paths[robot_id] = [pos, goal] # Fallback
                print(f"[CoSim] R{robot_id}: ⚠ FALLBACK to straight-line (no A* path provided)!")
                
            self.current_waypoint_idx[robot_id] = 0
            
            # Ensure path starts reasonably close to current pos? 
            # Ideally A* path starts at 'pos'. 
            # If path[0] == pos, we can look at path[1].
        
        # Initialize trajectories
        trajectories: Dict[int, List[Dict]] = {rid: [] for rid in valid_robot_ids}
        
        # Add initial positions
        current_time = 0.0
        for robot_id, state in states.items():
            trajectories[robot_id].append({
                "x": float(state.x),
                "y": float(state.y),
                "theta": float(state.theta),
                "t": float(start_time + current_time)
            })
            
        # Simulation loop
        for step in range(max_steps):
            # Check if all reached
            all_reached = all(state.reached_goal for state in states.values())
            if all_reached:
                break
            
            # Sort by priority for processing (Lower ID = processed first? Actually doesn't matter much for sync,
            # but priority logic is inside _compute_safe_velocity)
            
            next_states = {} # Store next positions to avoid order dependency in collision check?
            # Actually simpler to update in place for simple sim, but let's be careful.
            
            for robot_id in sorted(states.keys()):
                state = states[robot_id]
                if state.reached_goal:
                    continue
                    
                goal = goal_positions[robot_id]
                pos = (state.x, state.y)
                
                # Check goal reached
                dist_to_goal = math.sqrt((pos[0]-goal[0])**2 + (pos[1]-goal[1])**2)
                snap_threshold = 0.05
                if dist_to_goal < snap_threshold:
                    state.reached_goal = True
                    state.x, state.y = goal
                    if target_headings and robot_id in target_headings:
                        state.theta = target_headings[robot_id]
                    trajectories[robot_id].append({
                        "x": float(state.x), "y": float(state.y), "theta": float(state.theta),
                        "t": float(start_time + current_time + self.time_step)
                    })
                    continue
                
                # Get current target waypoint
                path = active_paths[robot_id]
                idx = self.current_waypoint_idx[robot_id]
                target_wp = path[idx]
                
                # Check if waypoint reached
                dist_wp = math.sqrt((pos[0]-target_wp[0])**2 + (pos[1]-target_wp[1])**2)
                if dist_wp < 0.1: # Waypoint tolerance
                    if idx < len(path) - 1:
                        self.current_waypoint_idx[robot_id] += 1
                        target_wp = path[self.current_waypoint_idx[robot_id]]
                
                # Compute Velocity
                vx, vy = self._compute_safe_velocity(robot_id, states, target_wp)
                
                # Update position
                state.x += vx * self.time_step
                state.y += vy * self.time_step
                
                # Update Theta (Interpolation)
                if initial_headings and target_headings and robot_id in initial_headings:
                    current_dist = math.sqrt((goal[0] - state.x)**2 + (goal[1] - state.y)**2)
                    progress = 1.0 - (current_dist / initial_dists[robot_id])
                    progress = max(0.0, min(1.0, progress))
                    
                    start_theta = initial_headings[robot_id]
                    end_theta = target_headings[robot_id]
                    
                    diff = end_theta - start_theta
                    while diff > math.pi: diff -= 2*math.pi
                    while diff < -math.pi: diff += 2*math.pi
                    
                    state.theta = start_theta + diff * progress

            
            # Advance time
            current_time += self.time_step
            
            # Record positions
            for robot_id, state in states.items():
                # Check if we should record this point
                should_record = False
                
                if len(trajectories[robot_id]) == 0:
                     should_record = True
                elif state.reached_goal and not trajectories[robot_id][-1].get("is_goal", False):
                     # Always record the exact moment we reach goal
                     # (Check previous point wasn't already the goal point to avoid dups)
                     should_record = True
                elif not state.reached_goal:
                     # Check spacing
                     last_x = trajectories[robot_id][-1]["x"]
                     last_y = trajectories[robot_id][-1]["y"]
                     dist = math.sqrt((state.x - last_x)**2 + (state.y - last_y)**2)
                     if dist >= min_waypoint_spacing:
                         should_record = True

                if should_record:
                    # Check timestamp uniqueness (simple check)
                    if trajectories[robot_id] and trajectories[robot_id][-1]["t"] >= start_time + current_time - 0.001:
                        # Update existing point logic if needed, or skip?
                        # Actually if we filter by space, time difference will naturally be large enough
                        pass
                    
                    point = {
                        "x": float(state.x),
                        "y": float(state.y),
                        "theta": float(state.theta),
                        "t": float(start_time + current_time)
                    }
                    if state.reached_goal:
                        point["is_goal"] = True
                    trajectories[robot_id].append(point)
        
        # Verify if all robots reached their goals
        all_reached = all(states[rid].reached_goal for rid in trajectories)
        if not all_reached:
            # Simulation ended without all robots reaching goals
            print(f"[CoSim] WARNING: Not all robots reached goals!")
            for rid in trajectories:
                state = states[rid]
                goal = goal_positions[rid]
                dist = math.sqrt((state.x - goal[0])**2 + (state.y - goal[1])**2)
                print(f"[CoSim]   Robot {rid}: reached={state.reached_goal}, "
                      f"pos=({state.x:.3f}, {state.y:.3f}), "
                      f"goal=({goal[0]:.3f}, {goal[1]:.3f}), "
                      f"dist={dist:.3f}m")
            return None

            
        # Smooth trajectories to reduce zigzag
        for robot_id in trajectories:
            trajectories[robot_id] = self._smooth_trajectory(trajectories[robot_id])
            
        # Re-filter to ensure minimum spacing after smoothing
        if min_waypoint_spacing > 0:
            for robot_id in trajectories:
                trajectories[robot_id] = self._filter_trajectory_spacing(
                    trajectories[robot_id], min_waypoint_spacing
                )
        
        return trajectories
    
    def _smooth_trajectory(self, trajectory: List[Dict], window_size: int = 5) -> List[Dict]:
        """Apply moving average smoothing to trajectory."""
        if len(trajectory) < window_size:
            return trajectory
        
        smoothed = []
        half_window = window_size // 2
        
        for i in range(len(trajectory)):
            if i < half_window or i >= len(trajectory) - half_window:
                smoothed.append(trajectory[i])
            else:
                # Average positions in window
                avg_x = sum(trajectory[j]["x"] for j in range(i - half_window, i + half_window + 1)) / window_size
                avg_y = sum(trajectory[j]["y"] for j in range(i - half_window, i + half_window + 1)) / window_size
                
                # Average angles properly (using unit vectors)
                sin_sum = sum(math.sin(trajectory[j].get("theta", 0)) for j in range(i - half_window, i + half_window + 1))
                cos_sum = sum(math.cos(trajectory[j].get("theta", 0)) for j in range(i - half_window, i + half_window + 1))
                avg_theta = math.atan2(sin_sum, cos_sum)
                
                smoothed.append({
                    "x": float(avg_x),
                    "y": float(avg_y),
                    "theta": float(avg_theta),
                    "t": trajectory[i]["t"]
                })
        
        return smoothed

    def _filter_trajectory_spacing(self, trajectory: List[Dict], min_spacing: float) -> List[Dict]:
        """Filter trajectory to ensure minimum spacing between waypoints."""
        if not trajectory:
            return []
            
        filtered = [trajectory[0]]
        for i in range(1, len(trajectory)):
            # Always keep the last point (it might be the goal) if it's explicitly marked or just the end
            is_last = (i == len(trajectory) - 1)
            
            p1 = filtered[-1]
            p2 = trajectory[i]
            
            dist = math.sqrt((p1['x'] - p2['x'])**2 + (p1['y'] - p2['y'])**2)
            
            if dist >= min_spacing or is_last or p2.get('is_goal'):
                filtered.append(p2)
                
        return filtered


def generate_synchronized_trajectories(
    robot_positions: Dict[int, Tuple[float, float]],
    goal_positions: Dict[int, Tuple[float, float]],
    initial_headings: Dict[int, float] = None,
    target_headings: Dict[int, float] = None,
    static_paths: Dict[int, List[Tuple[float, float]]] = None,
    path_planner=None, # Added path_planner
    velocity: float = 0.2,
    safety_radius: float = 0.4,
    influence_radius: float = 0.6,
    time_step: float = 0.1,
    start_time: Optional[float] = None,
    min_waypoint_spacing: float = 0.05,
    object_center: Optional[Tuple[float, float]] = None,
    object_radius: float = 0.0,
    robot_radius: float = 0.2
) -> Dict[int, List[Dict]]:
    """
    Generate collision-free synchronized trajectories for multiple robots.
    
    Args:
        static_paths: Pre-computed A* paths for each robot
    """
    simulator = CoSimulator(
        static_paths=static_paths,
        path_planner=path_planner, # Pass planner
        safety_radius=safety_radius,
        influence_radius=influence_radius,
        time_step=time_step,
        velocity=velocity,
        object_center=object_center,
        object_radius=object_radius,
        robot_radius=robot_radius
    )
    
    return simulator.simulate(
        robot_positions=robot_positions,
        goal_positions=goal_positions,
        initial_headings=initial_headings,
        target_headings=target_headings,
        start_time=start_time,
        max_steps=2000,
        min_waypoint_spacing=min_waypoint_spacing
    )




def generate_non_intersecting_trajectories(
    robot_positions: Dict[int, Tuple[float, float, float]],
    goal_positions: Dict[int, Tuple[float, float]],
    initial_headings: Dict[int, float] = None,
    target_headings: Dict[int, float] = None,
    static_paths: Dict[int, List[Tuple[float, float]]] = None,
    path_planner = None,
    velocity: float = 0.2,
    start_time: Optional[float] = None,
    min_waypoint_spacing: float = 0.1,
    path_buffer: float = 0.4,
    object_center: Optional[Tuple[float, float]] = None,
    object_radius: float = 0.0
) -> Dict[int, List[Dict]]:
    """
    Generate trajectories where paths do NOT geometrically intersect.
    
    Unlike CoSimulation which uses timing-based collision avoidance,
    this function ensures paths are geometrically separated so even with
    trajectory tracking errors, robots will not collide.
    
    Algorithm:
    1. Sort robots by ID (lower = higher priority)
    2. For each robot in priority order:
       a. Use pre-computed static_path if available, else plan with A*
       b. Add this path as a "corridor obstacle" for remaining robots
       c. Lower priority robots must re-plan around the corridor
    3. Timestamp all paths with velocity

    Args:
        robot_positions: {robot_id: (x, y, theta)} current positions
        goal_positions: {robot_id: (x, y)} goal positions  
        initial_headings: {robot_id: theta} initial headings
        target_headings: {robot_id: theta} target headings
        static_paths: {robot_id: [(x,y), ...]} pre-computed A* paths (already handle object avoidance)
        path_planner: PathPlanner instance for re-planning blocked paths
        velocity: Travel velocity in m/s
        start_time: Start time for trajectories
        min_waypoint_spacing: Minimum spacing between waypoints
        path_buffer: Buffer radius around paths (should be >= 2*robot_radius)
        object_center: Object center position (for reference only)
        object_radius: Object radius (for reference only)
        
    Returns:
        Dict {robot_id: trajectory_list} where trajectory_list is
        [{x, y, theta, t}, ...] or None on error
    """
    if path_planner is None:
        print("[NonIntersect] ERROR: path_planner is required")
        return None
    
    if static_paths is None:
        static_paths = {}
    
    if start_time is None:
        start_time = time.time()
    
    if initial_headings is None:
        initial_headings = {}
    if target_headings is None:
        target_headings = {}
    
    # Sort robots by ID (lower ID = higher priority)
    sorted_robot_ids = sorted(robot_positions.keys())
    
    # Store final paths for each robot
    final_paths: Dict[int, List[Tuple[float, float]]] = {}
    
    # Track obstacles added for path corridors (to remove later)
    path_corridor_start_idx = len(path_planner.obstacles)
    
    try:
        for robot_id in sorted_robot_ids:
            if robot_id not in goal_positions:
                print(f"[NonIntersect] Robot {robot_id} has no goal position, skipping")
                continue
            
            start_pos = robot_positions[robot_id][:2]  # (x, y)
            goal_pos = goal_positions[robot_id]
            
            # Check if we have a pre-computed static path
            if robot_id in static_paths and static_paths[robot_id]:
                base_path = static_paths[robot_id]
                print(f"[NonIntersect] Robot {robot_id}: Using pre-computed path ({len(base_path)} pts)")
            else:
                # Fallback: plan with A* (may fail if goal is in obstacle zone)
                print(f"[NonIntersect] Robot {robot_id}: No static path, planning A*...")
                base_path = path_planner.plan_path(start_pos, goal_pos)
            
            if base_path is None or len(base_path) < 2:
                print(f"[NonIntersect] ERROR: No path found for Robot {robot_id}!")
                return None
            
            # Check if this path intersects any existing corridor
            # If so, we need to re-plan around the corridors
            if _path_intersects_obstacles(base_path, path_planner):
                print(f"[NonIntersect] Robot {robot_id}: Path blocked by higher-priority corridor, re-planning...")
                
                # Re-plan from start to goal, the corridors are already in obstacle map
                # Use entry-point approach: plan to a safe entry point, then straight to goal
                new_path = _plan_path_with_entry(
                    planner=path_planner,
                    start=start_pos,
                    goal=goal_pos,
                    object_center=object_center,
                    object_radius=object_radius
                )
                
                if new_path is None or len(new_path) < 2:
                    print(f"[NonIntersect] ERROR: Robot {robot_id} cannot find alternative path!")
                    print(f"[NonIntersect] All routes are blocked by higher-priority robot paths.")
                    return None
                
                base_path = new_path
                print(f"[NonIntersect] Robot {robot_id}: Alternative path found ({len(base_path)} pts)")
            
            final_paths[robot_id] = base_path
            
            # Add this path as corridor obstacle for remaining (lower priority) robots
            # Add FULL corridor so lower priority robots must find different paths
            # Exclude: goals (so robots can reach destinations) + OTHER robots' starts (so they can escape)
            all_goals = list(goal_positions.values())
            
            # Exclude starts of OTHER robots (not this one) so they can escape
            other_starts = [(pos[0], pos[1]) for rid, pos in robot_positions.items() if rid != robot_id]
            exclusion_zones = all_goals + other_starts
            
            _add_path_as_obstacle(path_planner, base_path, path_buffer,
                                  exclude_goals=exclusion_zones,
                                  goal_exclusion_radius=path_buffer,  # Exact match
                                  skip_first_n=0)
            print(f"[NonIntersect] Robot {robot_id}: Full corridor added (exclude {len(exclusion_zones)} zones)")
    
    finally:
        # Cleanup: Remove all path corridor obstacles
        while len(path_planner.obstacles) > path_corridor_start_idx:
            path_planner.remove_last_obstacle()
    
    # Convert paths to timestamped trajectories
    trajectories: Dict[int, List[Dict]] = {}
    
    for robot_id, path in final_paths.items():
        # Calculate total distance
        total_dist = 0
        for i in range(1, len(path)):
            dx = path[i][0] - path[i-1][0]
            dy = path[i][1] - path[i-1][1]
            total_dist += math.sqrt(dx*dx + dy*dy)
        
        trajectory = []
        current_dist = 0
        
        initial_theta = initial_headings.get(robot_id, 0.0)
        final_theta = target_headings.get(robot_id, 0.0)
        
        # Shortest angle difference
        theta_diff = final_theta - initial_theta
        while theta_diff > math.pi: theta_diff -= 2*math.pi
        while theta_diff < -math.pi: theta_diff += 2*math.pi
        
        for i, point in enumerate(path):
            if i > 0:
                dx = point[0] - path[i-1][0]
                dy = point[1] - path[i-1][1]
                current_dist += math.sqrt(dx*dx + dy*dy)
            
            progress = current_dist / total_dist if total_dist > 0 else 1.0
            t = start_time + (current_dist / velocity) if velocity > 0 else start_time
            theta = initial_theta + theta_diff * progress
            
            trajectory.append({
                "x": float(point[0]),
                "y": float(point[1]),
                "theta": float(theta),
                "t": float(t)
            })
        
        trajectories[robot_id] = trajectory
    
    print(f"[NonIntersect] Successfully generated non-intersecting trajectories for {len(trajectories)} robots")
    return trajectories


def _path_intersects_obstacles(path: List[Tuple[float, float]], planner, exclude_last_n: int = 5) -> bool:
    """
    Check if any point along the path is inside an obstacle.
    
    Args:
        path: List of waypoints
        planner: PathPlanner instance
        exclude_last_n: Number of points at the end to exclude from check
                       (goals may be close together legitimately)
    """
    # Only check the transit portion, not the final approach to goal
    check_until = max(1, len(path) - exclude_last_n)
    
    for i, point in enumerate(path[:check_until]):
        if planner.is_obstacle(point[0], point[1]):
            return True
    return False


def _plan_path_with_entry(planner, start, goal, object_center, object_radius) -> Optional[List[Tuple[float, float]]]:
    """
    Plan path using entry point approach to handle goals inside obstacle zones.
    Similar to generate_safe_approach_path but works with existing obstacle map.
    """
    # Calculate entry point outside the current obstacles
    if object_center and object_radius > 0:
        obj_x, obj_y = object_center
        gx, gy = goal
        
        dx = gx - obj_x
        dy = gy - obj_y
        dist = math.sqrt(dx*dx + dy*dy)
        
        if dist < 0.01:
            dist = 0.01
        
        # Safe entry radius (outside object + robot inflation)
        safe_radius = object_radius + planner.robot_radius + 0.15
        
        # Entry point on safe radius, aligned with goal direction
        entry_x = obj_x + (dx / dist) * safe_radius
        entry_y = obj_y + (dy / dist) * safe_radius
        entry_point = (entry_x, entry_y)
        
        # Plan to entry point first
        path_to_entry = planner.plan_path(start, entry_point)
        
        if path_to_entry:
            # Append final approach to goal
            return path_to_entry + [goal]
    
    # Fallback: try direct planning
    return planner.plan_path(start, goal)




def _add_path_as_obstacle(planner, path: List[Tuple[float, float]], buffer_radius: float,
                          exclude_goals: List[Tuple[float, float]] = None,
                          goal_exclusion_radius: float = 0.5,
                          skip_first_n: int = 5):
    """
    Add a path as obstacle corridor to the planner.
    
    Creates circles along the path to form a "tube" that blocks other paths
    from crossing.
    
    Args:
        planner: PathPlanner instance
        path: List of (x, y) waypoints
        buffer_radius: Radius of the corridor
        exclude_goals: List of goal positions to NOT block (allow other robots to reach their goals)
        goal_exclusion_radius: Radius around goals to exclude from corridor
        skip_first_n: Number of waypoints at the START to skip (allow other robots to escape their start)
    """
    if exclude_goals is None:
        exclude_goals = []
    
    # Add circles at each waypoint and between waypoints
    # Use smaller step to create continuous corridor
    step_size = buffer_radius / 2  # Overlap circles for continuity
    
    # Skip first N segments to allow other robots to escape their start positions
    start_segment = min(skip_first_n, len(path) - 2)
    
    for i in range(start_segment, len(path) - 1):
        p1 = path[i]
        p2 = path[i + 1]
        
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        dist = math.sqrt(dx*dx + dy*dy)
        
        if dist < 0.001:
            continue
        
        # Number of circles to place along this segment
        num_circles = max(1, int(dist / step_size))
        
        for j in range(num_circles + 1):
            t = j / num_circles if num_circles > 0 else 0
            cx = p1[0] + t * dx
            cy = p1[1] + t * dy
            
            # Check if this point is near any excluded goal
            near_goal = False
            for goal in exclude_goals:
                goal_dist = math.sqrt((cx - goal[0])**2 + (cy - goal[1])**2)
                if goal_dist < goal_exclusion_radius:
                    near_goal = True
                    break
            
            if near_goal:
                continue  # Skip adding obstacle near goals
            
            # Add as obstacle (without robot_radius inflation since we're using buffer_radius directly)
            # Use internal method to avoid double inflation
            planner._add_obs_circle_raw(cx, cy, buffer_radius - planner.robot_radius)


def verify_trajectories_collision_free(
    trajectories: Dict[int, List[Dict]],
    safety_radius: float = 0.4
) -> Tuple[bool, List[Dict]]:
    """
    Verify that generated trajectories are collision-free.
    
    Args:
        trajectories: Generated trajectories from generate_synchronized_trajectories
        safety_radius: Minimum allowed distance
        
    Returns:
        Tuple (is_safe, collision_list) where collision_list contains
        details of any detected collisions
    """
    collisions = []
    robot_ids = list(trajectories.keys())
    
    # Build time-indexed positions
    all_times = set()
    for traj in trajectories.values():
        for point in traj:
            all_times.add(point["t"])
    
    sorted_times = sorted(all_times)
    
    # Check each time step
    for t in sorted_times:
        positions = {}
        for robot_id, traj in trajectories.items():
            # Find position at time t (interpolate if needed)
            pos = None
            for i, point in enumerate(traj):
                if abs(point["t"] - t) < 0.001:
                    pos = (point["x"], point["y"])
                    break
                elif point["t"] > t and i > 0:
                    # Interpolate
                    prev = traj[i-1]
                    alpha = (t - prev["t"]) / (point["t"] - prev["t"])
                    pos = (
                        prev["x"] + alpha * (point["x"] - prev["x"]),
                        prev["y"] + alpha * (point["y"] - prev["y"])
                    )
                    break
            if pos is None and traj:
                pos = (traj[-1]["x"], traj[-1]["y"])
            positions[robot_id] = pos
        
        # Check pairwise distances
        for i, id1 in enumerate(robot_ids):
            for id2 in robot_ids[i+1:]:
                if positions[id1] is None or positions[id2] is None:
                    continue
                dist = math.sqrt(
                    (positions[id1][0] - positions[id2][0])**2 +
                    (positions[id1][1] - positions[id2][1])**2
                )
                if dist < safety_radius:
                    collisions.append({
                        "time": t,
                        "robots": (id1, id2),
                        "distance": dist,
                        "positions": {id1: positions[id1], id2: positions[id2]}
                    })
    
    return (len(collisions) == 0, collisions)
