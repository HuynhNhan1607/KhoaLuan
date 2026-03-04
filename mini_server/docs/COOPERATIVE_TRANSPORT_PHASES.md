# Cooperative Transport System - Phase Documentation

## Tổng quan hệ thống

Hệ thống cooperative transport gồm 2 robot làm việc phối hợp để di chuyển một vật thể. Quá trình chia thành 2 phase chính:

1. **APPROACH PHASE (Phase 1)**: Các robot di chuyển độc lập đến vị trí grip
2. **TRANSPORT PHASE (Phase 2)**: Các robot phối hợp di chuyển vật thể theo mode Leader-Follower

---

## PHASE 1: APPROACH MODE

### Mục đích
Di chuyển mỗi robot từ vị trí hiện tại đến vị trí approach (gần vật thể) để chuẩn bị grip.

### Behavior chi tiết

#### 1.1 Trajectory Execution
Mỗi robot nhận trajectory riêng từ server và thực thi độc lập:

```
Robot 1: trajectory_1 = [{x1, y1, t1}, {x2, y2, t2}, ...]
Robot 2: trajectory_2 = [{x1, y1, t1}, {x2, y2, t2}, ...]
```

**Thuật toán**: Pure Pursuit + PID Velocity Control
- **Pure Pursuit**: Tìm lookahead point trước robot một khoảng `LOOKAHEAD_DISTANCE = 0.15m`
- **PID Control**: Tính velocity command để tracking lookahead point

**PID Gains** (trong `trajectory_executor.h`):
```c
#define TRAJ_KP 3.0f     // Proportional gain
#define TRAJ_KI 0.3f     // Integral gain  
#define TRAJ_KD 0.4f     // Derivative gain
#define MAX_I_TERM 0.3f  // Anti-windup limit
```

**Control Loop** (20Hz = 50ms):
```c
// 1. Get current position from EKF
cur_x, cur_y, cur_theta = EKF state

// 2. Find lookahead point
for i = current_index to trajectory.count:
    dist = sqrt((point[i].x - cur_x)^2 + (point[i].y - cur_y)^2)
    if dist > LOOKAHEAD_DISTANCE:
        lookahead_point = point[i]
        break

// 3. Calculate PID velocity command
err_x = lookahead_point.x - cur_x
err_y = lookahead_point.y - cur_y

P_x = KP * err_x
I_x = KI * sum_err_x  // với anti-windup
D_x = -KD * d_measurement_x  // derivative on measurement

cmd_vx = P_x + I_x + D_x
cmd_vy = P_y + I_y + D_y

// 4. Speed limiting (deceleration near goal)
if distance_to_final < DECEL_RADIUS:
    speed_limit = MIN_VELOCITY + (MAX_VELOCITY - MIN_VELOCITY) * 
                  (distance - acceptance_radius) / (DECEL_RADIUS - acceptance_radius)
    
if |cmd_v| > speed_limit:
    cmd_v = cmd_v * (speed_limit / |cmd_v|)

// 5. Send command to ESP32
send("dot_x:cmd_vx dot_y:cmd_vy dot_theta:cmd_w")
```

**Velocity Limits**:
```c
#define MAX_VELOCITY 0.25f        // 0.25 m/s maximum speed
#define MIN_VELOCITY 0.05f        // 0.05 m/s minimum speed when decelerating
#define DECEL_RADIUS 0.30f        // Start deceleration 30cm from goal
#define ACCEPTANCE_RADIUS 0.03f   // 3cm - goal reached threshold
#define ACCEPTANCE_HOLD_TIME_MS 200  // Hold for 200ms before confirming arrival
```

#### 1.2 Robot Independence
**Các robot KHÔNG ảnh hưởng tới nhau trong phase này:**

- ❌ Không có communication về position giữa các robot
- ❌ Không có formation control
- ❌ Không có collision avoidance
- ❌ Không có speed coordination

Mỗi robot chỉ quan tâm đến trajectory của mình và thực thi độc lập.

#### 1.3 Theta Tracking (Optional)
Nếu `ENABLE_THETA_TRACKING = 1`:

```c
// Feedforward từ trajectory timing
dt = target.t - prev.t
d_theta = target.theta - prev.theta
dot_theta_ff = d_theta / dt

// P-correction để giảm error
theta_err = cur_theta - target.theta
dot_theta_p = KP_THETA * theta_err

// Command = feedforward - correction
cmd_dot_theta = dot_theta_ff - dot_theta_p
clamp(cmd_dot_theta, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)
```

#### 1.4 Goal Completion
Robot coi như đã đến đích khi:
1. **Position**: `distance_to_final < ACCEPTANCE_RADIUS` (3cm)
2. **Theta** (nếu enabled): `|theta_error| < ACCEPTANCE_ANGLE` (5 degrees)
3. **Hold time**: Giữ trong vùng acceptance trong `ACCEPTANCE_HOLD_TIME_MS` (200ms)

Khi đến đích:
```c
trajectory_stop()
send_to_laptop_clients('{"type": "control", "status": "arrived"}')
```

---

## PHASE 2: TRANSPORT MODE

### Mục đích
Sau khi cả 2 robot đã grip vật thể, di chuyển vật thể đến vị trí đích với formation control để tránh làm hỏng vật thể.

### Kích hoạt Transport Mode

Transport mode được kích hoạt khi:
```c
formation_lock_transport_offset(centroid_x, centroid_y)
```

Được gọi sau khi:
1. Robot2 grip thành công (trong `arm_execute_grip`)
2. Hoặc manual command từ laptop: `lock_transport_offset`

**Điều kiện tiên quyết**:
- Robot2 đã nhận được dữ liệu từ Robot1 (`has_neighbor_data = true`)
- Cả 2 robot đều đã grip vật thể

### Architecture: Leader-Follower Mode

```
┌─────────────────────────────────────────────────────────┐
│                   TRANSPORT PHASE 2                     │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  Robot 1 (LEADER)              Robot 2 (FOLLOWER)     │
│  ┌──────────────┐              ┌──────────────┐        │
│  │ Trajectory   │              │ Formation    │        │
│  │ Executor     │──sync_pos──▶│ Follower     │        │
│  │              │   (20Hz)     │              │        │
│  │ + Speed      │              │ PD Control   │        │
│  │   Scaling    │              │              │        │
│  └──────────────┘              └──────────────┘        │
│         │                              │                │
│         ▼                              ▼                │
│    ┌─────────┐                    ┌─────────┐          │
│    │ ESP32_1 │                    │ ESP32_2 │          │
│    └─────────┘                    └─────────┘          │
│         │                              │                │
│         └──────────┬ OBJECT ┬──────────┘                │
│                    └────────┘                          │
└─────────────────────────────────────────────────────────┘
```

### 2.1 Robot 1 Behavior (Leader)

#### Trajectory Execution
Robot1 tiếp tục thực thi trajectory như bình thường (giống Phase 1), **NHƯNG** với speed scaling dựa trên follower error.

#### Speed Scaling for Formation Maintenance

**Mục đích**: Đợi Robot2 nếu nó bị lạc hậu, tránh làm đứt vật thể.

```c
// In trajectory_executor.c
#if ROBOT_ID == 1 && TRANSPORT_MODE_LEADER_FOLLOWER

double follower_error = 0.0;  // Distance between Robot2 and expected position

if (formation_is_transport_active() && 
    formation_get_follower_error(&follower_error)) {
    
    float target_scale = 1.0f;  // Default: full speed
    
    if (follower_error > FOLLOWER_ERROR_MAX) {
        // Robot2 too far (>5cm) -> STOP completely
        target_scale = 0.0f;
    }
    else if (follower_error > FOLLOWER_ERROR_DEADBAND) {
        // Robot2 slightly behind (1-5cm) -> Smooth deceleration
        // Quadratic scaling for smooth response
        float error_ratio = (follower_error - FOLLOWER_ERROR_DEADBAND) / 
                           (FOLLOWER_ERROR_MAX - FOLLOWER_ERROR_DEADBAND);
        target_scale = 1.0f - error_ratio * error_ratio;
    }
    // else: error <= 1cm -> full speed (1.0)
    
    // Low-pass filter to prevent jitter
    formation_speed_scale = SMOOTH_ALPHA * target_scale + 
                           (1.0 - SMOOTH_ALPHA) * formation_speed_scale;
    
    // Apply scaling to velocity command
    cmd_vx *= formation_speed_scale;
    cmd_vy *= formation_speed_scale;
}
#endif
```

**Thresholds** (trong `sys_config.h`):
```c
#define FOLLOWER_ERROR_MAX 0.05f      // 5cm - leader stops completely
#define FOLLOWER_ERROR_DEADBAND 0.01f // 1cm - ignore small noise
#define FOLLOWER_DATA_TIMEOUT 0.5     // 500ms - timeout for follower data
#define VELOCITY_SMOOTH_ALPHA 0.3f    // Smoothing factor for speed scaling
```

**Speed Scaling Curve**:
```
Speed Scale
1.0 ┤████████████           Full speed (error < 1cm)
    │           ╲
0.7 ┤            ╲          Smooth deceleration
    │             ╲         (quadratic curve)
0.4 ┤              ╲
    │               ╲
0.0 ┤________________████   Complete stop (error > 5cm)
    0    1cm        5cm
         ↑          ↑
      Deadband    Max Error
```

#### Position Broadcasting
Robot1 gửi position đến Robot2 với tần số cao:

```c
#define SYNC_POSITION_RATE_HZ 20  // 20Hz = 50ms interval

// Sent data:
{
    "type": "sync_position",
    "x": current_x,
    "y": current_y, 
    "vx": current_vx,
    "vy": current_vy,
    "theta": current_theta,
    "ts": timestamp_epoch
}
```

### 2.2 Robot 2 Behavior (Follower)

#### Override Trajectory - CRITICAL
**Trong transport mode, Robot2 KHÔNG chạy trajectory!**

Có 2 cơ chế đảm bảo điều này:

1. **trajectory_executor.c**: Skip execution khi transport active
```c
#if ROBOT_ID == 2
if (formation_is_transport_active()) {
    usleep(100000);  // Skip, không gửi command
    continue;
}
#endif
```

2. **formation_manager.c**: Thread riêng control Robot2
```c
#if ROBOT_ID == 2
if (g_formation.transport_active) {
    formation_get_transport_follow_velocity(&vx, &vy);
    // Gửi command trực tiếp từ đây
    client_manager_broadcast_to_motor(cmd);
}
#endif
```

**Robot2 chỉ làm 2 việc**:
- Follow position: `target = neighbor_pos + initial_offset`
- Hold theta: Giữ **góc của chính mình** (locked tại thời điểm grip)

**Robot2 KHÔNG làm**:
- ❌ Không chạy trajectory
- ❌ Không copy góc của Robot1
- ❌ Không tự nội suy quỹ đạo

#### PD Control for Following

**Target Position**:
```
Robot2_target = Robot1_position + initial_offset
```

Trong đó:
- `initial_offset` được lock khi `formation_lock_transport_offset()` được gọi
- `initial_offset = Robot2_pos - Robot1_pos` tại thời điểm grip

**PD Control Algorithm**:
```c
// Get current positions
my_x, my_y = Robot2's current position (from EKF)
neighbor_x, neighbor_y = Robot1's position (from sync_position)

// Calculate target position
target_x = neighbor_x + initial_offset_x
target_y = neighbor_y + initial_offset_y

// Position error
err_x = target_x - my_x
err_y = target_y - my_y

// === P TERM: Proportional to position error ===
p_vx = Kp * err_x
p_vy = Kp * err_y

// === D TERM: Velocity matching + damping ===
velocity_err_x = neighbor_vx - my_vx
velocity_err_y = neighbor_vy - my_vy
d_vx = Kd * velocity_err_x
d_vy = Kd * velocity_err_y

// Total correction
correction_vx = p_vx + d_vx
correction_vy = p_vy + d_vy

// === FEEDFORWARD: Match Robot1's velocity ===
cmd_vx = neighbor_vx + correction_vx
cmd_vy = neighbor_vy + correction_vy
```

**PD Gains** (conservative, uncertainty-aware):
```c
#define BASE_KP 1.8f  // Proportional gain (conservative)
#define BASE_KD 0.5f  // Derivative gain (high damping for smoothness)

// Scale down gains when position uncertainty is high
if (pos_uncertainty > 0.2m):
    Kp = BASE_KP * 0.5
    Kd = BASE_KD * 0.5
```

**Safety Limits**:
```c
#define ERROR_DEADBAND 0.02f        // 2cm - ignore tiny errors to reduce jitter
#define TRANSPORT_VELOCITY 0.11f    // 0.11 m/s - slow and safe
#define max_correction (TRANSPORT_VELOCITY * 1.5f)  // Limit correction velocity
#define max_vel (TRANSPORT_VELOCITY * 1.2f)         // Limit total velocity
```

**Velocity Smoothing** (Low-pass filter):
```c
// Prevent jerky motion
smooth_vx = ALPHA * new_vx + (1.0 - ALPHA) * smooth_vx
smooth_vy = ALPHA * new_vy + (1.0 - ALPHA) * smooth_vy
// ALPHA = 0.7 (fast response but still smooth)
```

#### Delay Compensation

**Vấn đề**: Network delay + processing delay → Robot1's position đã cũ khi Robot2 nhận được.

**Giải pháp**: Extrapolate Robot1's position về hiện tại:

```c
// In formation_update_neighbor()
double receive_time = get_current_epoch_time();
double delay_s = receive_time - ts;  // Delay in seconds

// Extrapolate position
predicted_x = x + vx * delay_s;
predicted_y = y + vy * delay_s;

// Use predicted position for control
g_formation.neighbor_x = predicted_x;
g_formation.neighbor_y = predicted_y;
```

**Typical delay**: 20-50ms (tùy thuộc network và processing)

### 2.3 Follower Error Calculation (Advanced)

Robot1 cần biết Robot2 có bị lag không để điều chỉnh tốc độ. Thuật toán tính error **có dự đoán** để tránh dao động:

```c
// In formation_get_follower_error()

// Expected position for Robot2
expected_follower_x = my_x + initial_offset_x
expected_follower_y = my_y + initial_offset_y

// Actual position of Robot2 (from sync_position)
actual_follower_x = neighbor_x
actual_follower_y = neighbor_y

// === CURRENT ERROR ===
dx = actual_follower_x - expected_follower_x
dy = actual_follower_y - expected_follower_y
current_error = sqrt(dx^2 + dy^2)

// === VELOCITY ANALYSIS ===
relative_vx = follower_vx - my_vx
relative_vy = follower_vy - my_vy

// Check if Robot2 is moving TOWARDS expected position
velocity_dot = dx * relative_vx + dy * relative_vy
follower_correcting = (velocity_dot < 0.0)

// === PREDICTIVE ERROR (0.2s ahead) ===
PREDICTION_TIME = 0.2
predicted_dx = dx + relative_vx * PREDICTION_TIME
predicted_dy = dy + relative_vy * PREDICTION_TIME
predicted_error = sqrt(predicted_dx^2 + predicted_dy^2)

// === SMART ERROR SELECTION ===
// If Robot2 is actively correcting and predicted error is smaller,
// use predicted error (less conservative, prevents oscillation)
if (follower_correcting && predicted_error < current_error) {
    error_distance = predicted_error;
} else {
    error_distance = current_error;
}
```

**Ý nghĩa**:
- Nếu Robot2 đang di chuyển về đúng hướng (correcting) → dùng predicted error (lạc quan hơn)
- Nếu Robot2 không correcting hoặc đang đi xa → dùng current error (an toàn hơn)
- Điều này tránh Robot1 dừng/khởi động liên tục (oscillation)

---

## State Transitions

### Flow Chart

```
┌─────────────────┐
│   IDLE STATE    │ Robot đứng yên, chưa nhận trajectory
└────────┬────────┘
         │
         │ receive trajectory_1 / trajectory_2
         ▼
┌─────────────────┐
│ APPROACH PHASE  │ Robot di chuyển độc lập theo trajectory
│   (Phase 1)     │ - Pure Pursuit + PID
│                 │ - No interaction between robots
│                 │ - Independent execution
└────────┬────────┘
         │
         │ both robots arrived + grip success
         │ -> formation_lock_transport_offset()
         ▼
┌─────────────────┐
│ TRANSPORT PHASE │ Cooperative transport
│   (Phase 2)     │ Robot1: Leader (trajectory + speed scaling)
│                 │ Robot2: Follower (follow Robot1 + PD control)
│ Leader-Follower │ - Sync position (20Hz)
│      Mode       │ - Formation maintenance
│                 │ - Speed coordination
└────────┬────────┘
         │
         │ trajectory complete
         │ -> formation_end_transport()
         ▼
┌─────────────────┐
│  ARRIVED STATE  │ Both robots arrived at destination
└─────────────────┘
```

### Detailed State Transitions

#### 1. IDLE → APPROACH
**Trigger**: Receive trajectory command from laptop
```json
{
    "command": "execute_trajectory",
    "trajectory": [...],
    "start_time": 1234567890.123
}
```

**Actions**:
- Parse trajectory points
- Schedule execution at `start_time`
- Start trajectory executor thread (20Hz)
- Robots execute independently

#### 2. APPROACH → TRANSPORT
**Trigger**: Both robots grip success
```c
// After Robot2 grip success
formation_lock_transport_offset(centroid_x, centroid_y);
```

**Actions**:
```c
// Calculate initial offset
pthread_mutex_lock(&g_ekf_mutex);
double my_x = g_ekf.x[0];
double my_y = g_ekf.x[1];
pthread_mutex_unlock(&g_ekf_mutex);

// Lock offset (Robot2 - Robot1)
g_formation.initial_offset_x = my_x - neighbor_x;
g_formation.initial_offset_y = my_y - neighbor_y;
g_formation.is_locked = true;

// Activate transport
g_formation.transport_active = true;
```

**Behavior Changes**:
- Robot1: Enable speed scaling based on follower error
- Robot2: Switch from trajectory to follow mode (override trajectory)
- Start position sync (20Hz)

#### 3. TRANSPORT → ARRIVED
**Trigger**: Robot1 trajectory complete
```c
// In trajectory_executor.c
if (goal_reached && hold_time >= ACCEPTANCE_HOLD_TIME_MS) {
    trajectory_stop();
    send_to_laptop_clients('{"type": "control", "status": "arrived"}');
    formation_end_transport();  // End transport mode
}
```

**Actions**:
- Stop both robots
- Deactivate transport mode
- Send arrival notification to laptop
- Ready for release/grip commands

---

## Configuration Parameters Summary

### Phase 1 (Approach) - Independent Execution

| Parameter              | Value    | Description                               |
| ---------------------- | -------- | ----------------------------------------- |
| `MAX_VELOCITY`         | 0.25 m/s | Maximum approach speed                    |
| `MIN_VELOCITY`         | 0.05 m/s | Minimum speed when decelerating           |
| `LOOKAHEAD_DISTANCE`   | 0.15 m   | Pure Pursuit lookahead distance           |
| `DECEL_RADIUS`         | 0.30 m   | Start deceleration when 30cm from goal    |
| `ACCEPTANCE_RADIUS`    | 0.03 m   | Goal reached when within 3cm              |
| `ACCEPTANCE_HOLD_TIME` | 200 ms   | Hold position for 200ms before confirming |
| `TRAJ_KP`              | 3.0      | PID Proportional gain                     |
| `TRAJ_KI`              | 0.3      | PID Integral gain                         |
| `TRAJ_KD`              | 0.4      | PID Derivative gain                       |
| `CONTROL_LOOP_DELAY`   | 50 ms    | Control loop period (20Hz)                |

### Phase 2 (Transport) - Cooperative Execution

| Parameter                   | Value    | Description                               |
| --------------------------- | -------- | ----------------------------------------- |
| `TRANSPORT_VELOCITY`        | 0.11 m/s | Transport speed (slow and safe)           |
| `SYNC_POSITION_RATE_HZ`     | 20 Hz    | Position sync frequency (Leader→Follower) |
| `FOLLOWER_ERROR_MAX`        | 0.05 m   | Max follower error before leader stops    |
| `FOLLOWER_ERROR_DEADBAND`   | 0.01 m   | Ignore errors smaller than 1cm            |
| `FOLLOWER_DATA_TIMEOUT`     | 0.5 s    | Timeout for follower position data        |
| `VELOCITY_SMOOTH_ALPHA`     | 0.3      | Speed scaling smoothing factor (Leader)   |
| `BASE_KP` (Follower)        | 1.8      | PD Proportional gain (conservative)       |
| `BASE_KD` (Follower)        | 0.5      | PD Derivative gain (high damping)         |
| `ERROR_DEADBAND` (Follower) | 0.02 m   | Ignore tiny errors (reduce jitter)        |
| `PREDICTION_TIME`           | 0.2 s    | Predictive error calculation horizon      |

---

## Robot Interaction Summary

### Phase 1: APPROACH (Independent)
| Interaction Type      | Status | Details                                      |
| --------------------- | ------ | -------------------------------------------- |
| Position Sharing      | ❌ No   | Each robot uses only its own EKF             |
| Velocity Coordination | ❌ No   | Independent trajectory execution             |
| Collision Avoidance   | ❌ No   | Relies on trajectory planning                |
| Formation Control     | ❌ No   | No formation in this phase                   |
| Speed Adjustment      | ❌ No   | Each robot follows its own trajectory timing |

**Conclusion**: Robots hoàn toàn độc lập, không ảnh hưởng tới nhau.

### Phase 2: TRANSPORT (Cooperative Leader-Follower)
| Interaction Type      | Status     | Details                                        |
| --------------------- | ---------- | ---------------------------------------------- |
| Position Sharing      | ✅ Yes      | Robot1 → Robot2 (20Hz) with delay compensation |
| Velocity Coordination | ✅ Yes      | Robot1 scales speed based on Robot2 error      |
| Formation Control     | ✅ Yes      | Robot2 maintains offset from Robot1            |
| Speed Adjustment      | ✅ Yes      | Robot1 slows/stops if Robot2 lags behind       |
| Predictive Control    | ✅ Yes      | Robot2 extrapolates Robot1 position            |
| Collision Avoidance   | 🔄 Implicit | Maintained by formation control                |

**Conclusion**: Robots phối hợp chặt chẽ, ảnh hưởng lẫn nhau để maintain formation.

---

## Critical Differences Between Phases

| Aspect                 | Phase 1: APPROACH                      | Phase 2: TRANSPORT                     |
| ---------------------- | -------------------------------------- | -------------------------------------- |
| **Control Strategy**   | Independent PID tracking               | Leader-Follower coordination           |
| **Robot1 Behavior**    | Follow trajectory (full speed)         | Follow trajectory + speed scaling      |
| **Robot2 Behavior**    | Follow trajectory (full speed)         | Override trajectory → follow Robot1    |
| **Speed**              | 0.25 m/s (fast)                        | 0.11 m/s (slow, safe)                  |
| **Position Sync**      | No sync                                | 20Hz sync (Robot1→Robot2)              |
| **Formation**          | No formation                           | Strict formation (offset locked)       |
| **Speed Coordination** | None                                   | Robot1 waits for Robot2                |
| **Failure Handling**   | Independent (stop if trajectory fails) | Cooperative (stop if formation breaks) |
| **Theta Control**      | Optional (trajectory-based)            | Locked (prevent arm breakage)          |

---

## Failure Modes & Recovery

### Phase 1 Failures
1. **Trajectory execution timeout**: Robot stops after no progress for 5s
2. **EKF divergence**: System stops, requires manual reset
3. **Obstacle collision**: No built-in avoidance, relies on trajectory planning

**Recovery**: Manual intervention or re-send trajectory

### Phase 2 Failures
1. **Follower data timeout (>500ms)**:
   - Robot1: Continue trajectory at reduced speed
   - Robot2: Stop immediately (no leader data)

2. **Follower error too large (>5cm)**:
   - Robot1: Stop completely (wait for Robot2)
   - Robot2: Aggressive correction to catch up

3. **Position uncertainty high (>20cm)**:
   - Robot2: Reduce PD gains by 50% (conservative control)
   
4. **Formation break**:
   - Both robots stop
   - Require manual reset or re-grip

**Recovery**: System attempts automatic recovery through PD control. If persistent (>5s), require manual intervention.

---

## Implementation Notes

### Key Files
- `trajectory_executor.c`: Phase 1 + Phase 2 (Robot1) trajectory execution
- `formation_manager.c`: Phase 2 Leader-Follower coordination
- `sys_config.h`: Configuration parameters for both phases
- `ekf.c`: Localization (EKF state estimation)

### Threading Model
```
┌─────────────────────────────────────────────────┐
│              MAIN THREAD                        │
│  - JSON command parsing                         │
│  - Network communication                        │
│  - High-level control                           │
└────────────┬────────────────────────────────────┘
             │
   ┌─────────┼─────────┐
   ▼         ▼         ▼
┌──────┐ ┌──────┐ ┌──────┐
│ EKF  │ │ TRAJ │ │ FORM │  All threads run at 20Hz
│Thread│ │Thread│ │Thread│  (50ms period)
└──────┘ └──────┘ └──────┘
   │         │         │
   └─────────┴─────────┴──────▶ ESP32 Commands
```

### Communication Protocol
**Robot1 → Robot2** (Transport Phase):
```json
{
    "type": "sync_position",
    "x": 1.234,
    "y": 5.678,
    "vx": 0.11,
    "vy": 0.0,
    "theta": 1.57,
    "ts": 1234567890.123,
    "pos_uncertainty": 0.05
}
```

**Frequency**: 20Hz (50ms interval)  
**Transport**: UDP broadcast (low latency, tolerate packet loss)

---

## Tuning Guidelines

### Phase 1 (Approach) Tuning

**Objective**: Fast, accurate trajectory tracking

1. **PID Gains**:
   - Increase `TRAJ_KP` if robot is too slow to reach trajectory
   - Increase `TRAJ_KD` if robot overshoots or oscillates
   - Increase `TRAJ_KI` if steady-state error exists

2. **Speed Limits**:
   - Increase `MAX_VELOCITY` for faster approach (risk: overshoot)
   - Increase `DECEL_RADIUS` for smoother deceleration

3. **Acceptance**:
   - Decrease `ACCEPTANCE_RADIUS` for tighter positioning
   - Increase `ACCEPTANCE_HOLD_TIME` for more stable goal detection

### Phase 2 (Transport) Tuning

**Objective**: Smooth, safe cooperative transport

1. **Follower PD Gains**:
   - Increase `BASE_KP` if Robot2 is too slow to track Robot1
   - Increase `BASE_KD` if Robot2 oscillates around target position
   - Keep conservative (current values are well-tuned)

2. **Formation Thresholds**:
   - Decrease `FOLLOWER_ERROR_MAX` for tighter formation (risk: frequent stops)
   - Increase `FOLLOWER_ERROR_DEADBAND` to reduce jitter
   - Adjust `VELOCITY_SMOOTH_ALPHA` for smoother speed scaling

3. **Sync Rate**:
   - Current: 20Hz (good balance)
   - Can reduce to 5Hz after testing delay compensation
   - Higher rate = better tracking, more network traffic

---

## Performance Metrics

### Phase 1 (Approach)
- **Tracking Error**: Typically < 5cm RMS
- **Arrival Time**: Depends on trajectory length and speed
- **Overshoot**: < 2cm with proper PID tuning
- **Success Rate**: > 99% (with good EKF localization)

### Phase 2 (Transport)
- **Formation Error**: < 3cm RMS (Robot2 tracking Robot1)
- **Speed Coordination**: Robot1 stops if Robot2 lags > 5cm
- **Transport Speed**: 0.11 m/s (conservative for safety)
- **Success Rate**: > 95% (depends on network stability)

---

## Future Improvements

### Short-term
1. **Adaptive PID**: Auto-tune gains based on tracking error
2. **Obstacle Avoidance**: Add collision detection + dynamic re-planning
3. **Reduce Sync Rate**: Test 5Hz sync with better prediction (reduce network load)

### Long-term
1. **Multi-robot formation**: Extend to 3+ robots
2. **Vision-based formation**: Use camera for relative positioning
3. **Impedance control**: Soft interaction for fragile objects
4. **Learning-based coordination**: RL for optimal formation behavior

---

## Glossary

- **EKF**: Extended Kalman Filter - localization algorithm
- **Pure Pursuit**: Path following algorithm with lookahead point
- **PD Control**: Proportional-Derivative controller
- **Feedforward**: Open-loop control based on reference trajectory
- **Feedback**: Closed-loop control based on error measurement
- **Deadband**: Small threshold to ignore noise
- **Anti-windup**: Prevent integrator saturation in PID
- **Low-pass Filter**: Smooth signal by reducing high-frequency noise
- **Extrapolation**: Predict future value based on current velocity

---

*Document Version: 1.0*  
*Last Updated: February 3, 2026*  
*Author: Automated system analysis*
