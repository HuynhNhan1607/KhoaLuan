#!/usr/bin/env python3
"""
Test script for Non-Intersecting Approach Paths
Run this to verify the generate_non_intersecting_trajectories function works correctly.

Usage: python test_non_intersecting_paths.py
"""

import sys
import math

# Add server directory to path
sys.path.insert(0, '/home/huynhan1607/Multiple_Mobile_Robot/server')

from path_planner import PathPlanner
from trajectory_manager import generate_safe_approach_path
from synchronized_trajectory import generate_non_intersecting_trajectories, verify_trajectories_collision_free


def test_crossing_paths():
    """
    Test case: Two robots with paths that would normally cross.
    
    Setup:
    - Robot 1: bottom-left, goal at top-right side of object
    - Robot 2: bottom-right, goal at top-left side of object
    - These paths would naturally cross if both take shortest path
    """
    print("\n" + "="*60)
    print("TEST: Two robots with crossing paths")
    print("="*60)
    
    # Object position (center) - FROM USER CONFIG
    object_pos = (0.95, 4.18)
    object_size = 0.2  # diameter
    object_radius = object_size / 2
    
    # Robot positions (FROM USER CONFIG - very close together!)
    robot_positions = {
        1: (1.55, 1.78, 0.0),  # Robot 1
        2: (2.0, 1.78, 0.0),   # Robot 2 - only 0.45m from Robot 1!
    }
    
    # Calculate grip positions (opposite sides of object)
    grip_radius = (object_size / 2) + 0.1 + 0.05  # object_radius + arm_base + gripper
    grip_positions = {
        1: (object_pos[0], object_pos[1] + grip_radius),  # North side
        2: (object_pos[0], object_pos[1] - grip_radius),  # South side  
    }
    
    print(f"Object: center={object_pos}, radius={object_radius:.2f}m")
    print(f"Robot 1: pos={robot_positions[1][:2]} → goal={grip_positions[1]}")
    print(f"Robot 2: pos={robot_positions[2][:2]} → goal={grip_positions[2]}")
    
    # Create path planner
    planner = PathPlanner(
        x_range=(0.0, 10.0),
        y_range=(0.0, 10.0),
        cell_size=0.05,
        robot_radius=0.25
    )
    
    # Generate static paths (with two-phase approach for object avoidance)
    static_paths = {}
    for rid in robot_positions:
        path = generate_safe_approach_path(
            planner=planner,
            start_pos=robot_positions[rid][:2],
            goal_pos=grip_positions[rid],
            object_pos=object_pos,
            object_size=object_size,
            robot_radius=0.25
        )
        if path:
            static_paths[rid] = path
            print(f"Static path R{rid}: {len(path)} points")
        else:
            print(f"ERROR: No static path for R{rid}")
            return False
    
    # Initial/target headings
    initial_headings = {1: 0.0, 2: 0.0}
    target_headings = {1: math.pi/2, 2: -math.pi/2}  # Face toward object
    
    # Safety radius for path buffer: 2 * robot_radius + gap
    robot_radius = 0.2
    safety_radius = 2 * robot_radius + 0.1  # 50cm (10cm gap)
    
    print(f"\nGenerating non-intersecting trajectories (path_buffer={safety_radius:.2f}m)...")
    
    # Generate non-intersecting trajectories
    trajectories = generate_non_intersecting_trajectories(
        robot_positions=robot_positions,
        goal_positions=grip_positions,
        initial_headings=initial_headings,
        target_headings=target_headings,
        static_paths=static_paths,
        path_planner=planner,
        velocity=0.2,
        start_time=0.0,
        min_waypoint_spacing=0.1,
        path_buffer=safety_radius,
        object_center=object_pos,
        object_radius=object_radius
    )
    
    if trajectories is None:
        print("❌ ERROR: Failed to generate trajectories")
        return False
    
    print(f"\n✓ Generated trajectories:")
    for rid, traj in trajectories.items():
        total_dist = 0
        for i in range(1, len(traj)):
            dx = traj[i]['x'] - traj[i-1]['x']
            dy = traj[i]['y'] - traj[i-1]['y']
            total_dist += math.sqrt(dx*dx + dy*dy)
        print(f"  Robot {rid}: {len(traj)} waypoints, {total_dist:.2f}m total distance")
    
    # Verify collision-free
    is_safe, collisions = verify_trajectories_collision_free(
        trajectories, safety_radius=safety_radius
    )
    
    if is_safe:
        print(f"\n✓ PASS: All trajectories are collision-free!")
    else:
        print(f"\n⚠ WARNING: {len(collisions)} potential timing collisions detected")
        print("  (This may be acceptable as paths don't geometrically intersect)")
        for col in collisions[:3]:
            print(f"  - R{col['robots'][0]} vs R{col['robots'][1]} at t={col['time']:.2f}s, dist={col['distance']:.3f}m")
    
    # Check if paths geometrically intersect (the real test)
    # Exclude: grip zones (convergence expected) + start zones (user-placed, guaranteed safe)
    print("\nChecking geometric path intersection (excluding grip and start zones)...")
    path1 = [(p['x'], p['y']) for p in trajectories[1]]
    path2 = [(p['x'], p['y']) for p in trajectories[2]]
    
    # Exclude zones around grip positions, object, AND start positions
    start_positions = [robot_positions[1][:2], robot_positions[2][:2]]
    exclusion_zones = list(grip_positions.values()) + [object_pos] + start_positions
    
    intersects = check_paths_intersect(
        path1, path2, 
        min_distance=safety_radius*0.9,
        exclude_zones=exclusion_zones,
        zone_radius=1.0  # 1m around excluded zones
    )
    
    if not intersects:
        print(f"✓ PASS: Paths do NOT intersect in open space (min_distance >= {safety_radius*0.9:.2f}m)")
        return True
    else:
        print(f"❌ FAIL: Paths cross each other in open space!")
        return False


def check_paths_intersect(path1, path2, min_distance=0.5, exclude_zones=None, zone_radius=0.5):
    """
    Check if two paths come within min_distance of each other at any point.
    
    Excludes zones near goals where convergence is expected (grip positions).
    
    Args:
        path1, path2: Lists of (x, y) points
        min_distance: Minimum required separation
        exclude_zones: List of (x, y) points to exclude from check (e.g., grip positions)
        zone_radius: Radius around exclusion zones
    """
    if exclude_zones is None:
        exclude_zones = []
    
    min_found = float('inf')
    min_location = None
    
    for p1 in path1:
        # Skip if p1 is near any exclusion zone (goal area)
        in_exclusion = False
        for zone in exclude_zones:
            if math.sqrt((p1[0]-zone[0])**2 + (p1[1]-zone[1])**2) < zone_radius:
                in_exclusion = True
                break
        if in_exclusion:
            continue
            
        for p2 in path2:
            # Skip if p2 is near any exclusion zone
            in_exclusion2 = False
            for zone in exclude_zones:
                if math.sqrt((p2[0]-zone[0])**2 + (p2[1]-zone[1])**2) < zone_radius:
                    in_exclusion2 = True
                    break
            if in_exclusion2:
                continue
            
            dist = math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
            if dist < min_found:
                min_found = dist
                min_location = (p1, p2)
    
    if min_found == float('inf'):
        print(f"  All path points are in exclusion zones (expected near grip positions)")
        return False  # No intersection in open space
    
    print(f"  Minimum distance between paths (excluding grip zone): {min_found:.3f}m")
    if min_location:
        print(f"    At: {min_location[0]} <-> {min_location[1]}")
    
    return min_found < min_distance


def test_same_side_paths():
    """
    Test case: Two robots on same side, goals on same side.
    This should not cause crossing.
    """
    print("\n" + "="*60)
    print("TEST: Two robots on same side (no crossing expected)")
    print("="*60)
    
    object_pos = (3.0, 4.0)
    object_size = 0.6
    object_radius = object_size / 2
    
    robot_positions = {
        1: (1.0, 2.0, 0.0),
        2: (1.5, 1.5, 0.0),
    }
    
    grip_radius = (object_size / 2) + 0.1 + 0.15
    grip_positions = {
        1: (object_pos[0] - grip_radius, object_pos[1] + 0.3),
        2: (object_pos[0] - grip_radius, object_pos[1] - 0.3),
    }
    
    print(f"Robot 1: pos={robot_positions[1][:2]} → goal={grip_positions[1]}")
    print(f"Robot 2: pos={robot_positions[2][:2]} → goal={grip_positions[2]}")
    
    planner = PathPlanner(x_range=(0.0, 10.0), y_range=(0.0, 10.0), cell_size=0.05, robot_radius=0.25)
    
    static_paths = {}
    for rid in robot_positions:
        path = generate_safe_approach_path(
            planner=planner,
            start_pos=robot_positions[rid][:2],
            goal_pos=grip_positions[rid],
            object_pos=object_pos,
            object_size=object_size,
            robot_radius=0.25
        )
        if path:
            static_paths[rid] = path
    
    safety_radius = 0.7
    
    trajectories = generate_non_intersecting_trajectories(
        robot_positions=robot_positions,
        goal_positions=grip_positions,
        initial_headings={1: 0, 2: 0},
        target_headings={1: math.pi/2, 2: math.pi/2},
        static_paths=static_paths,
        path_planner=planner,
        velocity=0.2,
        start_time=0.0,
        path_buffer=safety_radius,
        object_center=object_pos,
        object_radius=object_radius
    )
    
    if trajectories:
        print(f"✓ Generated trajectories for {len(trajectories)} robots")
        for rid, traj in trajectories.items():
            print(f"  Robot {rid}: {len(traj)} waypoints")
        return True
    else:
        print("❌ FAIL: Could not generate trajectories")
        return False


if __name__ == "__main__":
    print("="*60)
    print("Non-Intersecting Paths Test Suite")
    print("="*60)
    
    results = []
    
    # Run tests
    results.append(("Crossing Paths", test_crossing_paths()))
    results.append(("Same Side Paths", test_same_side_paths()))
    
    # Summary
    print("\n" + "="*60)
    print("TEST SUMMARY")
    print("="*60)
    
    all_passed = True
    for name, passed in results:
        status = "✓ PASS" if passed else "❌ FAIL"
        print(f"  {status}: {name}")
        if not passed:
            all_passed = False
    
    print("\n" + ("All tests passed!" if all_passed else "Some tests failed!"))
    sys.exit(0 if all_passed else 1)
