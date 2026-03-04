"""
Approach Manager Module (Phase 1)

Handles all Phase 1 (Approach) operations:
- Computing trajectories from current positions to grip positions
- Starting/aborting approach phase  
- Handling arrival notifications
"""

import time
import json
from tkinter import messagebox

from trajectory_manager import generate_multi_robot_approach_trajectories, generate_safe_approach_path
from synchronized_trajectory import generate_synchronized_trajectories, generate_non_intersecting_trajectories, verify_trajectories_collision_free
from server_trajection_multi import trajectory_visualizer


# ================================================================
# SINGLE ROBOT TEST MODE - Set to True when testing with only 1 robot
# This bypasses the "all robots arrived" check
# ================================================================
SINGLE_ROBOT_TEST_MODE = True  # Set to True when testing with only 1 robot


class ApproachManager:
    """
    Manages Phase 1 (Approach) operations.
    
    Robots move from their current positions to grip positions around the object.
    Uses A* path planning and Co-Simulation for collision-free trajectories.
    """
    
    def __init__(self, server):
        """
        Initialize the Approach Manager.
        
        Args:
            server: Reference to main Server instance for accessing shared state
        """
        self.server = server
        
        # Approach phase state (moved from Server)
        self.approach_phase_active = False
        self.approach_trajectories = {}  # {robot_id: trajectory_list}
        self.arrived_status = {}         # {robot_id: bool}
        self.grip_sides = {}             # {robot_id: "top"|"bottom"|"left"|"right"}
    
    def compute_approach_trajectories(self, use_vector_field=True, use_collision_avoidance=True):
        """
        Compute approach trajectories from current positions to grip positions.
        
        Args:
            use_vector_field: Deprecated flag, now means "Use A* Planner".
            use_collision_avoidance: If True, use Co-Simulation.
        
        Returns:
            dict: {robot_id: trajectory_list} or None on error
        """
        server = self.server
        
        if server.object_position is None:
            server.gui.update_monitor("Error: Object position not set")
            return None
        
        # Determine which robots are connected
        connected_robots = list(server.robot_connections.keys())
        if not connected_robots:
            if server.use_manual_positions:
                # For testing without connections
                connected_robots = list(server.manual_positions.keys())
            else:
                server.gui.update_monitor("Error: No robots connected")
                return None
        
        # Set active robots in formation planner
        server.formation_planner.set_active_robots(connected_robots)
        
        # Compute grip positions
        server.grip_positions = server.formation_planner.compute_grip_positions(
            server.object_position, server.object_length, server.object_width
        )
        
        if not server.grip_positions:
            server.gui.update_monitor("Error: Failed to compute grip positions")
            return None
        
        # Get current robot positions
        robot_positions = {}
        for robot_id in connected_robots:
            pos = server.get_robot_position(robot_id)
            if pos is None:
                server.gui.update_monitor(f"Warning: No position for Robot {robot_id}")
                continue
            robot_positions[robot_id] = pos
        
        # Filter robot_positions to only include robots that have grip positions
        robot_positions = {
            rid: pos for rid, pos in robot_positions.items() 
            if rid in server.grip_positions
        }
        
        if not robot_positions:
            server.gui.update_monitor("Error: No robot positions match grip positions")
            return None
        
        # Use planner if requested (flag name preserved for compatibility)
        planner = server.path_planner if use_vector_field else None
        
        # Generate paths using A* first (Static Paths)
        static_paths = {}
        server.gui.update_monitor(f"📍 DEBUG: planner={'Available' if planner else 'None'}, use_vector_field={use_vector_field}")
        
        if planner:
            try:
                for rid, start_pos in robot_positions.items():
                    base_goal = server.grip_positions[rid]
                    server.gui.update_monitor(f"  R{rid}: Computing A* path from {start_pos[:2]} to {base_goal}")
                    
                    # Use max dimension for effective object size
                    max_object_dimension = max(server.object_length, server.object_width) if server.object_length and server.object_width else 0.2
                    
                    # Use the helper function from trajectory_manager
                    path = generate_safe_approach_path(
                        planner=planner,
                        start_pos=start_pos[:2],  # (x, y)
                        goal_pos=base_goal,
                        object_pos=server.object_position,
                        object_size=max_object_dimension,
                        robot_radius=0.25
                    )
                    
                    if path:
                        static_paths[rid] = path
                        server.gui.update_monitor(f"  R{rid}: ✓ A* path generated ({len(path)} pts)")
                    else:
                        # CRITICAL: If no path found (likely blocked), STOP everything.
                        msg = f"Target UNREACHABLE for Robot {rid}! Path blocked by obstacle."
                        server.gui.update_monitor(msg)
                        messagebox.showerror("Path Planning Error", msg)
                        return None

            except Exception as e:
                print(f"Error in path generation: {e}")
                import traceback
                traceback.print_exc()
                server.gui.update_monitor(f"❌ Exception in path generation: {e}")
        else:
            server.gui.update_monitor("⚠ No planner available - using straight-line fallback")

        # DEBUG: Show static_paths summary
        server.gui.update_monitor(f"📍 DEBUG: static_paths has {len(static_paths)} entries: {list(static_paths.keys())}")
        
        start_time = time.time() + 1.0
        
        # Extract initial headings
        initial_headings = {}
        for rid in robot_positions:
            initial_headings[rid] = robot_positions[rid][2] if len(robot_positions[rid]) >= 3 else 0.0
        
        # Calculate target headings
        target_headings = server.formation_planner.compute_approach_angles(
            server.object_position, heading_convention='Y'
        )
        
        if use_collision_avoidance:
            server.gui.update_monitor("Generating NON-INTERSECTING collision-free trajectories...")
            
            # Use max dimension for effective radius with rectangular objects
            max_object_dimension = max(server.object_length, server.object_width) if server.object_length and server.object_width else 0.0
            object_radius = max_object_dimension / 2
            
            # CRITICAL: safety_radius must account for BOTH robot radii + desired gap
            # Robot radius = 0.2m, so 2 robots touching = 0.4m center-to-center
            # To have 10cm GAP between robot bodies: 0.2 + 0.1 + 0.2 = 0.5m center-to-center
            robot_safety_radius = 2 * server.robot_radius + 0.2  # 2×20cm + 10cm gap = 50cm
            
            # Use Non-Intersecting Paths (geometry-based, no timing dependency)
            self.approach_trajectories = generate_non_intersecting_trajectories(
                robot_positions=robot_positions,
                goal_positions=server.grip_positions,
                initial_headings=initial_headings,
                target_headings=target_headings,
                static_paths=static_paths,  # Pre-computed A* paths with object avoidance
                path_planner=server.path_planner,
                velocity=server.approach_velocity,
                start_time=start_time,
                min_waypoint_spacing=0.1,
                path_buffer=robot_safety_radius,  # Buffer around each path
                object_center=server.object_position,
                object_radius=object_radius
            )
            
            if self.approach_trajectories is None:
                server.gui.update_monitor("Error: Failed to generate non-intersecting trajectories")
                server.gui.update_monitor("Hint: One robot's path may be completely blocking another. Try repositioning robots.")
                return None
            
            # Verify paths don't intersect (optional sanity check)
            is_safe, collisions = verify_trajectories_collision_free(
                self.approach_trajectories, safety_radius=robot_safety_radius
            )
            
            if is_safe:
                server.gui.update_monitor("✓ All trajectories verified safe (non-intersecting)")
            else:
                # With non-intersecting paths, this should rarely happen
                server.gui.update_monitor(f"⚠ Warning: {len(collisions)} potential timing collision(s) detected.")
                server.gui.update_monitor("  Paths are geometrically separate, but timing may overlap at edges.")
        else:
            # Independent paths (just timestamp the A* paths)
            server.gui.update_monitor("Generating independent trajectories...")
            max_object_dimension = max(server.object_length, server.object_width) if server.object_length and server.object_width else 0.0
            self.approach_trajectories = generate_multi_robot_approach_trajectories(
                robot_positions=robot_positions,
                grip_positions=server.grip_positions,
                initial_headings=initial_headings,
                target_headings=target_headings,
                vector_field_planner=planner,
                velocity=server.approach_velocity,
                start_time=start_time,
                min_waypoint_spacing=0.1,
                object_center=server.object_position,
                object_radius=max_object_dimension/2,
                robot_radius=server.robot_radius
            )

        # Update visualization
        trajectory_visualizer.set_grip_positions(server.grip_positions)
        for robot_id, trajectory in self.approach_trajectories.items():
            path_points = [(p['x'], p['y']) for p in trajectory]
            trajectory_visualizer.set_ground_truth_path(robot_id, path_points)
        
        return self.approach_trajectories
    
    def start_approach_phase(self, use_vector_field=True):
        """
        Start the approach phase: compute and send trajectories to all robots.
        
        Args:
            use_vector_field: If True, use vector field for obstacle avoidance.
        
        Returns:
            bool: True if started successfully
        """
        server = self.server
        
        # Compute trajectories
        trajectories = self.compute_approach_trajectories(use_vector_field)
        
        if not trajectories:
            server.gui.update_monitor("Failed to start approach phase: No trajectories")
            return False
        
        # Reset arrival status
        self.arrived_status = {robot_id: False for robot_id in trajectories.keys()}
        self.approach_phase_active = True
        
        # Calculate synchronized start time for all robots
        sync_start_time = time.time() + server.execution_time_offset
        
        # Offset all trajectory times by execution_time_offset
        for trajectory in trajectories.values():
            for point in trajectory:
                point['t'] += server.execution_time_offset
        
        # Send trajectories to robots
        success_count = 0
        for robot_id, trajectory in trajectories.items():
            if robot_id not in server.robot_connections:
                server.gui.update_monitor(f"Robot {robot_id} not connected - skipping")
                continue
            
            # Set decentralized execution mode
            server.decentralized_execution[robot_id] = True
            
            # Send trajectory (object info will be sent separately when all robots arrive)
            meta = {
                "phase": "approach"
            }
            
            if server.send_trajectory_to_robot(robot_id, trajectory, meta):
                # Small delay to ensure trajectory is received before execute command
                time.sleep(0.01)  # 10ms delay
                
                # Send execute command with synchronized start time
                exec_cmd = json.dumps({
                    "type": "control", 
                    "cmd": "execute_trajectory",
                    "time": sync_start_time  # Synchronized start time for all robots
                })
                server.send_command_to_robot(robot_id, exec_cmd)
                success_count += 1
        
        if success_count > 0:
            server.gui.update_monitor(
                f"Approach phase started for {success_count} robot(s)"
            )
            return True
        else:
            self.approach_phase_active = False
            server.gui.update_monitor("Approach phase failed: No trajectories sent")
            return False
    
    def handle_arrival_notification(self, robot_id):
        """
        Handle arrival notification from a robot.
        
        Called when robot sends status='arrived' message.
        
        Args:
            robot_id: Robot that has arrived
        """
        server = self.server
        
        # Only update if this robot was part of the approach phase
        if robot_id not in self.arrived_status:
            server.gui.update_monitor(f"Robot {robot_id}: Received 'arrived' but not in approach phase - ignoring")
            return
        
        self.arrived_status[robot_id] = True
        server.gui.update_monitor(f"Robot {robot_id}: Arrived at grip position")
        
        # Update GUI arrival indicator
        if hasattr(server.gui, 'update_arrival_status'):
            server.gui.update_arrival_status(robot_id, True)
        
        # Check if all robots have arrived
        if self.check_all_arrived():
            self.approach_phase_active = False
            server.gui.update_monitor(
                "=== ALL ROBOTS ARRIVED - SENDING SYNCHRONIZED GRIP COMMAND ==="
            )
            
            # Send synchronized grip command to all arrived robots
            self._send_synchronized_grip_command()
    
    def check_all_arrived(self):
        """
        Check if all active robots have arrived at their positions.
        
        Only checks robots that were actually sent trajectories (arrived_status keys),
        NOT the total num_robots configured. This allows 1 robot to work correctly.
        
        Returns:
            bool: True if all robots have arrived
        """
        if not self.arrived_status:
            return False
        
        # Check that ALL robots that received trajectories have arrived
        # arrived_status is set from trajectories.keys() → only connected robots
        # So this correctly handles 1, 2, or 3 robots automatically
        return all(self.arrived_status.values())
    
    def abort_approach_phase(self):
        """
        Abort the current approach phase and stop all robots.
        """
        server = self.server
        self.approach_phase_active = False
        
        # Stop all robots
        for robot_id in server.robot_connections.keys():
            server.emergency_stop(robot_id)
            server.decentralized_execution[robot_id] = False
            trajectory_visualizer.clear_ground_truth_path(robot_id)
        
        self.arrived_status.clear()
        self.approach_trajectories.clear()
        
        server.gui.update_monitor("Approach phase aborted")
    
    def get_approach_status(self):
        """
        Get current approach phase status.
        
        Returns:
            dict: Status information
        """
        server = self.server
        return {
            'active': self.approach_phase_active,
            'object_position': server.object_position,
            'object_length': server.object_length,
            'object_width': server.object_width,
            'grip_positions': server.grip_positions.copy(),
            'arrived_status': self.arrived_status.copy(),
            'all_arrived': self.check_all_arrived(),
            'use_manual_positions': server.use_manual_positions,
            'num_robots': server.formation_planner.num_robots,
            'grip_radius': server.formation_planner.grip_radius
        }
    
    def _send_synchronized_grip_command(self):
        """
        Send synchronized grip command to all arrived robots.
        
        This is called when all robots have arrived at their grip positions.
        The command includes:
        - Synchronized start time (in the future)
        - Object position and size
        - Each robot's grip position
        
        All robots will execute the grip action at the same time.
        """
        server = self.server
        
        # Get list of arrived robots
        arrived_robots = [rid for rid, arrived in self.arrived_status.items() if arrived]
        
        if not arrived_robots:
            server.gui.update_monitor("Error: No arrived robots to send grip command")
            return
        
        # Calculate synchronized start time (give robots time to prepare)
        sync_grip_time = time.time() + server.execution_time_offset
        
        # Determine robot positions relative to object (top/bottom based on Y coordinate)
        # Robot with higher Y is "top", robot with lower Y is "bottom"
        robot_positions_info = {}
        for rid in arrived_robots:
            grip_pos = server.grip_positions.get(rid)
            if grip_pos:
                robot_positions_info[rid] = {
                    "y": grip_pos[1],
                    "angle": server.formation_planner.robot_angles.get(rid, 0)
                }
        
        # Sort robots by Y position to determine top/bottom
        sorted_robots = sorted(robot_positions_info.keys(), 
                               key=lambda r: robot_positions_info[r]["y"], 
                               reverse=True)  # Higher Y first = top
        
        # Assign positions: "top", "bottom" (for 2 robots) or "top", "left", "right" (for 3 robots)
        grip_side = {}
        if len(sorted_robots) == 1:
            # Single robot: determine side based on Y position relative to object
            r = sorted_robots[0]
            obj_y = server.object_position[1]
            robot_y = robot_positions_info[r]["y"]
            grip_side[r] = "bottom" if robot_y < obj_y else "top"
        elif len(sorted_robots) == 2:
            grip_side[sorted_robots[0]] = "top"
            grip_side[sorted_robots[1]] = "bottom"
        elif len(sorted_robots) == 3:
            # Top robot (highest Y)
            grip_side[sorted_robots[0]] = "top"
            # For bottom two, determine left/right by X coordinate
            bottom_two = sorted_robots[1:]
            if server.grip_positions[bottom_two[0]][0] < server.grip_positions[bottom_two[1]][0]:
                grip_side[bottom_two[0]] = "left"
                grip_side[bottom_two[1]] = "right"
            else:
                grip_side[bottom_two[0]] = "right"
                grip_side[bottom_two[1]] = "left"
        
        # Send grip command to each robot
        success_count = 0
        
        # Save grip_sides for reuse in execute_place (Phase 2)
        self.grip_sides = grip_side.copy()
        
        for robot_id in arrived_robots:
            if robot_id not in server.robot_connections:
                continue
            
            grip_cmd = {
                "type": "control",
                "cmd": "execute_grip",
                "time": sync_grip_time,
                "object_pos": list(server.object_position),
                "object_size": [server.object_length, server.object_width],
                "grip_side": grip_side.get(robot_id, "unknown")  # "top", "bottom", "left", "right"
            }
            
            try:
                server.send_command_to_robot(robot_id, json.dumps(grip_cmd))
                success_count += 1
                server.gui.update_monitor(
                    f"Robot {robot_id}: Grip command sent (side={grip_side.get(robot_id)}, at {sync_grip_time:.3f})"
                )
            except Exception as e:
                server.gui.update_monitor(f"Robot {robot_id}: Failed to send grip command: {e}")
        
        if success_count > 0:
            server.gui.update_monitor(
                f"=== SYNCHRONIZED GRIP COMMAND SENT TO {success_count} ROBOTS ==="
            )
            server.gui.update_monitor(
                f"    Object: pos={server.object_position}, size=[{server.object_length}, {server.object_width}]"
            )
            server.gui.update_monitor(
                f"    Grip starts at: {sync_grip_time:.3f} (in {server.execution_time_offset}s)"
            )
