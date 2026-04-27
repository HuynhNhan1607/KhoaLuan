# Multi-Robot Server - Detailed Behavior Documentation

## 1. System Overview
The Server acts as the central orchestrator for a cooperative multi-robot system consisting of up to 3 Holonomic or Non-Holonomic robots. The system is designed to collaboratively navigate to an object, secure it using robotic arms, and transport it safely to a target destination without colliding. 

The server operations are primarily split into two main operational phases:
1. **Phase 1: Approach** - Robots navigate from independent starting positions to form a specific gripping geometry around the object.
2. **Phase 2: Transport** - Working as a rigid collective, the robots transport the object to the destination while maintaining a **Virtual Structure**.

## 2. Server Architecture & State Management
- **Core Component (`server_multi.py`)**: The `Server` class dynamically manages multi-robot states dynamically. State is tracked via dictionary structures indexed by Robot ID (`1`, `2`, `3`), tracking connections, physical parameters, live positions, connection status, and progress states.
- **Network Interface**: Network communication is managed using TCP sockets. The Server spawns an independent client thread for each newly connected robot to ensure non-blocking concurrent communication.
- **Configuration & Parameters**: Physical capabilities like `robot_radius`, `gripper_length`, `approach_velocity`, and `transport_velocity` are centrally configured on the server to adapt to varying scenarios.

## 3. Communication Protocol (JSON over TCP)
All data transmission between the Server and the Edge Robots happens via JSON payload encoding.

### Messages Sent to Robots (Downlink)
*   **Trajectories (`"type": "load_trajectory"`)**: Packets of sequence waypoints including coordinates `[x, y]` and relative timestamps.
*   **Action Controls (`"type": "control"`)**: Command execution triggers like:
    *   `"command": "execute_trajectory"`
    *   `"command": "execute_grip"`
    *   `"command": "execute_place"`
*   **Synchronizer Ping (`"type": "sync_position"`)**: Regular time-sync pings sent to align robot internal clocks with the server's logical timer.
*   **Arm / IK Overrides**: Manual dispatching of arm parameters (`"type": "arm_ik_request"`).

### Messages Received from Robots (Uplink)
*   **Status Indicators**: Important lifecycle events like `{"type": "status", "status": "arrived"}` or `{"type": "status", "status": "transport_complete"}`.
*   **Telemetry Realtime Stream**:
    *   `encoder`: Motor velocities and states.
    *   `bno055`: Orientation / IMU data.
    *   `position`: Assumed odometry or localized XY coordinate.
    *   `pid_data`: PID controller active statistics parameters.

---

## 4. Phase 1: Approach Phase (`approach_manager.py`)
This phase handles safely maneuvering the dispersed robots into a precise formation around the target object.

1. **Path Planning**: Once started, the approach manager uses an **A* (A-Star)** path-finding algorithm to build shortest-path routes avoiding static obstacles for each robot.
2. **Collision De-confliction**: The server runs a Co-Simulation via `generate_non_intersecting_trajectories`. Utilizing radial geometry constraints (`robot_radius`), the server enforces strict spatial-temporal guarantees that the robots' paths will not cross or intersect at the same timestamp.
3. **Execution Steps**:
   - The Server pushes approach trajectories to all robots.
   - Pushes an **execution trigger** utilizing a network time offset to ensure all robots start simultaneously securely bypassing network latency.
   - Monitors uplink statuses, suspending execution transition until every participating robot reports an `"arrived"` status.
   - Triggers synchronized `execute_grip` action.

---

## 5. Phase 2: Transport Phase (`transport_manager.py`)
After securing the object, the robots transition into acting as a single composite unit (Virtual Structure).

1. **Virtual Structure Control**: The server calculates the mathematical centroid of the formation (which is usually the center of the payload). The individual robot end-effectors act as rigid extensions of this centroid.
2. **Path Smoothing**: Unlike Phase 1, Phase 2 implements **Cubic Spline Smoothing** on the generated payload path. This ensures jerk-less smooth acceleration and deceleration, preventing dropping the payload or straining the robotic arms.
3. **Execution Steps**:
   - Server calculates the smooth Centroid trajectory, then extrapolates relative trajectories for Robot 1, Robot 2, and Robot 3 based on formation offsets.
   - Distributes the load payloads and sends the synchronized execute start time.
   - Constantly logs transport synchronized statuses.
   - Awaits `"transport_complete"`.
   - Triggers `"execute_place"` to open the grippers and decouple the virtual structure.

---

## 6. Execution Models & Sync Strategies
*   **Decentralized Timed Execution**: For higher accuracy, rather than the server spoon-feeding the robots waypoints continually (Send-and-Wait), the backend utilizes Model B: it dumps the entire trajectory cache on the robot edges, allowing the robot's local microcontroller to execute the time-series loop to prevent sub-millisecond network jitter.
*   **Execution Delay Compensation**: A parameter `execution_time_offset` is sent into the load command to tell the robots internally "delay start by exactly X milliseconds once you receive this". 

## 7. Logging & Instrumentation
- A high-performance detached thread `_sync_log_worker` manages disk I/O.
- Dumps high-resolution CSV streams tracking telemetry (`encoder.csv`, `imu.csv`, `pid.csv`).
- This non-blocking architecture ensures the primary Server execution pipelines do not stagger or lag during massive robotic data influx dumps.
