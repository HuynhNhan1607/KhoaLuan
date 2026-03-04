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
"""

import math
import time
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass


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
    """
    
    def __init__(self, 
                 vector_field_planner=None,
                 safety_radius: float = 0.4,
                 influence_radius: float = 0.6,
                 repulsive_gain: float = 1.0,
                 time_step: float = 0.1,
                 velocity: float = 0.2,
                 goal_tolerance: float = 0.05):
        """
        Initialize the Co-Simulator.
        
        Args:
            vector_field_planner: VectorFieldPlanner for static obstacles
            safety_radius: Minimum distance between robots
            influence_radius: Distance at which repulsive force activates
            repulsive_gain: Strength of repulsive force
            time_step: Simulation time step (seconds)
            velocity: Desired robot velocity (m/s)
            goal_tolerance: Distance to consider goal reached
        """
        self.vector_field_planner = vector_field_planner
        self.collision_checker = CollisionChecker(safety_radius, influence_radius)
        self.repulsive_gain = repulsive_gain
        self.time_step = time_step
        self.velocity = velocity
        self.goal_tolerance = goal_tolerance
        self.safety_radius = safety_radius
        self.influence_radius = influence_radius
    
    def _compute_attractive_force(self, 
                                    pos: Tuple[float, float],
                                    goal: Tuple[float, float]) -> Tuple[float, float]:
        """
        Compute attractive force toward goal.
        
        Uses the VectorFieldPlanner if available, otherwise simple attraction.
        """
        if self.vector_field_planner is not None:
            # Use VectorFieldPlanner's force computation (includes static obstacles)
            fx_att, fy_att = self.vector_field_planner._compute_attractive_force(
                pos[0], pos[1], goal[0], goal[1]
            )
            fx_rep, fy_rep = self.vector_field_planner._compute_repulsive_force(
                pos[0], pos[1], goal[0], goal[1]
            )
            return (fx_att + fx_rep, fy_att + fy_rep)
        else:
            # Simple linear attraction
            dx = goal[0] - pos[0]
            dy = goal[1] - pos[1]
            dist = math.sqrt(dx**2 + dy**2)
            if dist > 0.01:
                return (dx / dist, dy / dist)
            return (0.0, 0.0)
    
    def _compute_total_force(self,
                              robot_id: int,
                              states: Dict[int, RobotState],
                              goals: Dict[int, Tuple[float, float]]) -> Tuple[float, float]:
        """
        Compute total force on a robot (attractive + repulsive from other robots).
        
        Priority system: robot with lower ID has higher priority.
        - Higher ID robots always get pushed by lower ID robots
        - When very close (inside safety radius), both robots push each other
          to ensure collision avoidance even in head-on scenarios
        """
        state = states[robot_id]
        pos = (state.x, state.y)
        goal = goals[robot_id]
        
        # Attractive force toward goal
        fx, fy = self._compute_attractive_force(pos, goal)
        
        # Repulsive forces from other robots
        for other_id, other_state in states.items():
            if other_id == robot_id:
                continue
            
            other_pos = (other_state.x, other_state.y)
            distance = self.collision_checker.get_distance(pos, other_pos)
            
            # Priority: lower ID = higher priority
            # Normal case: lower priority robot gets pushed
            if other_id < robot_id:
                rep_fx, rep_fy = self.collision_checker.compute_repulsive_force(
                    pos, other_pos, self.repulsive_gain
                )
                fx += rep_fx
                fy += rep_fy
            # Critical case: when inside safety radius, even higher priority robot
            # gets a weaker push to help avoid collision
            elif distance < self.safety_radius:
                # Weaker push for higher priority robot (0.3x strength)
                rep_fx, rep_fy = self.collision_checker.compute_repulsive_force(
                    pos, other_pos, self.repulsive_gain * 0.3
                )
                fx += rep_fx
                fy += rep_fy
        
        return (fx, fy)
    
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
        
        # Only simulate robots that have both position AND goal
        valid_robot_ids = set(robot_positions.keys()) & set(goal_positions.keys())
        
        if not valid_robot_ids:
            return {}
        
        # Initialize robot states (only for valid robots)
        states: Dict[int, RobotState] = {}
        initial_dists: Dict[int, float] = {}  # Store initial distance for interpolation
        
        for robot_id in valid_robot_ids:
            pos = robot_positions[robot_id]
            # Get initial theta if provided, else 0
            theta = initial_headings.get(robot_id, 0.0) if initial_headings else 0.0
            states[robot_id] = RobotState(x=pos[0], y=pos[1], theta=theta)
            
            # Calculate initial distance to goal for progress interpolation
            goal = goal_positions[robot_id]
            dist = math.sqrt((goal[0] - pos[0])**2 + (goal[1] - pos[1])**2)
            initial_dists[robot_id] = dist if dist > 0.001 else 0.001
        
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
            # Check if all robots have reached their goals
            all_reached = all(state.reached_goal for state in states.values())
            if all_reached:
                break
            
            # Update each robot
            for robot_id in sorted(states.keys()):  # Process in priority order
                state = states[robot_id]
                
                if state.reached_goal:
                    continue
                
                goal = goal_positions[robot_id]
                pos = (state.x, state.y)
                
                # Check if goal reached
                dist_to_goal = self.collision_checker.get_distance(pos, goal)
                if dist_to_goal < self.goal_tolerance:
                    state.x = goal[0]
                    state.y = goal[1]
                    # Snap orientation to target
                    if target_headings and robot_id in target_headings:
                        state.theta = target_headings[robot_id]
                    
                    trajectories[robot_id].append({
                        "x": float(goal[0]),
                        "y": float(goal[1]),
                        "theta": float(state.theta),
                        "t": float(start_time + current_time + self.time_step)
                    })
                    continue
                
                # Compute total force
                fx, fy = self._compute_total_force(robot_id, states, goal_positions)
                
                # Normalize and apply velocity
                magnitude = math.sqrt(fx**2 + fy**2)
                if magnitude > 0.001:
                    vx = (fx / magnitude) * self.velocity
                    vy = (fy / magnitude) * self.velocity
                else:
                    # Move directly toward goal if stuck
                    dx = goal[0] - state.x
                    dy = goal[1] - state.y
                    dist = math.sqrt(dx**2 + dy**2)
                    if dist > 0.001:
                        vx = (dx / dist) * self.velocity
                        vy = (dy / dist) * self.velocity
                    else:
                        vx, vy = 0.0, 0.0
                
                # Update position
                new_x = state.x + vx * self.time_step
                new_y = state.y + vy * self.time_step
                
                # Check for collision with static obstacles (if planner available)
                if self.vector_field_planner is not None:
                    if self.vector_field_planner.is_obstacle(new_x, new_y):
                        # Don't move into obstacle - stay in place
                        continue
                
                # Clamp to map bounds
                if self.vector_field_planner is not None:
                    # Use VectorFieldPlanner bounds if available
                    x_min = self.vector_field_planner.x_min
                    x_max = self.vector_field_planner.x_max
                    y_min = self.vector_field_planner.y_min
                    y_max = self.vector_field_planner.y_max
                    
                    padding = 0.01
                    new_x = max(x_min + padding, min(new_x, x_max - padding))
                    new_y = max(y_min + padding, min(new_y, y_max - padding))
                
                state.x = new_x
                state.y = new_y
                
                # Update Theta (Interpolation based on progress)
                # progress = 1 - (current_dist / initial_dist)
                if initial_headings and target_headings and robot_id in initial_headings and robot_id in target_headings:
                    current_dist = math.sqrt((goal[0] - state.x)**2 + (goal[1] - state.y)**2)
                    progress = 1.0 - (current_dist / initial_dists[robot_id])
                    progress = max(0.0, min(1.0, progress)) # Clamp 0..1
                    
                    start_theta = initial_headings[robot_id]
                    end_theta = target_headings[robot_id]
                    
                    # Handle angle wrapping for shortest path interpolation
                    diff = end_theta - start_theta
                    while diff > math.pi: diff -= 2*math.pi
                    while diff < -math.pi: diff += 2*math.pi
                    
                    # Calculate new theta
                    new_theta = start_theta + diff * progress
                    state.theta = new_theta
            
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
    vector_field_planner=None,
    velocity: float = 0.2,
    safety_radius: float = 0.4,
    influence_radius: float = 0.6,
    time_step: float = 0.1,
    repulsive_gain: float = 1.0,
    start_time: Optional[float] = None,
    min_waypoint_spacing: float = 0.05
) -> Dict[int, List[Dict]]:
    """
    Generate collision-free synchronized trajectories for multiple robots.
    
    Uses Co-Simulation with Dynamic Repulsive Force to ensure robots
    don't collide with each other while reaching their goals.
    
    Args:
        robot_positions: {robot_id: (x, y)} current positions
        goal_positions: {robot_id: (x, y)} target positions
        vector_field_planner: Optional VectorFieldPlanner for static obstacles
        velocity: Travel velocity in m/s
        safety_radius: Minimum distance between robots
        influence_radius: Distance at which repulsive force activates
        time_step: Simulation time step
        repulsive_gain: Strength of repulsive force
        start_time: Epoch timestamp for trajectory start
        
    Returns:
        Dict {robot_id: trajectory_list} where trajectory_list is
        [{"x": float, "y": float, "t": float}, ...]
        
    Example:
        >>> positions = {1: (1.0, 1.0), 2: (2.0, 1.0), 3: (1.5, 2.0)}
        >>> goals = {1: (5.0, 5.0), 2: (5.0, 5.0), 3: (5.0, 5.0)}
        >>> trajectories = generate_synchronized_trajectories(positions, goals)
    """
    simulator = CoSimulator(
        vector_field_planner=vector_field_planner,
        safety_radius=safety_radius,
        influence_radius=influence_radius,
        repulsive_gain=repulsive_gain,
        time_step=time_step,
        velocity=velocity
    )
    
    return simulator.simulate(
        robot_positions=robot_positions,
        goal_positions=goal_positions,
        initial_headings=initial_headings,
        target_headings=target_headings,
        start_time=start_time,
        min_waypoint_spacing=min_waypoint_spacing
    )


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
