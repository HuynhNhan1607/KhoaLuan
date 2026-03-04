#ifndef SYS_CONFIG_H
#define SYS_CONFIG_H

// Robot ID (set at compile-time via build.sh)
// 1 = robot1, 2 = robot2
// Override bằng: ./build.sh robot1 hoặc ./build.sh robot2
#ifndef ROBOT_ID
#define ROBOT_ID 1 // Default to robot1
#endif

#define Cal_Freq 0
#define USE_GRIPPER 1
#define ENABLE_THETA_TRACKING 1 // 0 = disabled, 1 = enabled

// Test mode khi nhấc bánh xe lên:
// 0 = chế độ bình thường (có UWB + optical flow)
// 1 = chế độ test (chỉ odometry + IMU, tắt UWB + optical flow)
#define TEST_MODE_WHEEL_UP 0

// Optical Flow Enable/Disable:
// 0 = tắt hoàn toàn optical flow (khi cảm biến hư)
// 1 = bật optical flow
#define ENABLE_OPTICAL_FLOW 1

// Optical Flow EKF update mode:
// 0 = update velocity only (vx, vy) - tránh drift position
// 1 = update full state (x, y, vx, vy) - cần mat_inv 4x4
#define OPTICAL_FLOW_UPDATE_POSITION 0

// PID Logging for tuning:
// 0 = disabled
// 1 = enabled - ghi log 20Hz vào /tmp/pid_log.csv
#define ENABLE_PID_LOGGING 0

// Trajectory Position Source:
// 0 = dùng EKF (có sensor fusion: UWB + Odometry + Optical Flow)
// 1 = dùng Odometry trực tiếp (chỉ encoder, không có sensor fusion)
#define USE_ODOMETRY_FOR_TRAJECTORY 0

// ============ TRANSPORT (PHASE 2) CONFIG ============
// Transport velocity (m/s) - vận tốc di chuyển vật thể chậm để đảm bảo an toàn
#define TRANSPORT_VELOCITY 0.1f

// Transport mode: Leader-Follower only
// Robot1 = Leader (trajectory), Robot2 = Follower (follow Robot1 + offset)
#define TRANSPORT_MODE_LEADER_FOLLOWER 1 // Always enabled

// ============ LEADER-FOLLOWER SYNC CONFIG ============
// Sync position rate (Hz) - how often robots share their position
// Higher = better tracking but more network traffic
// Lower = more prediction needed, must have good delay compensation
#define SYNC_POSITION_RATE_HZ 20 // Current: 20Hz, Target: 5Hz after tuning

// Smooth speed scaling based on follower error (no sudden jumps)
// Uses continuous function: speed_scale = 1 - (error/max_error)^2
// This creates smooth deceleration from 0cm to max_error
// TRANSPORT MODE: Allow reasonable tolerance for sync errors
#define FOLLOWER_ERROR_MAX 0.09f      // 15cm - leader stops completely (relaxed for sync delay)
#define FOLLOWER_ERROR_DEADBAND 0.02f // 5cm - ignore small errors (noise + EKF uncertainty)
#define FOLLOWER_DATA_TIMEOUT 0.5     // 500ms - timeout for follower data

// Velocity smoothing (low-pass filter) to prevent jerky motion
// smooth_vel = alpha * new_vel + (1-alpha) * old_vel
// Higher alpha = faster response but more jerky
// Lower alpha = smoother but slower response
#define VELOCITY_SMOOTH_ALPHA 0.8f // Smoothing factor (0.0-1.0)

// ============ SINGLE ROBOT MODE ============
// Set to 1 when only Robot1 is operating (Robot2 is unavailable/broken)
// Effects:
//   - formation_lock_transport_offset: không cần neighbor data, lock ngay lập tức
//   - Transport phase: bỏ qua follower error check, chạy full speed
//   - sync_position thread: tắt hoàn toàn (không có peer để broadcast)
#define SINGLE_ROBOT_MODE 1

#endif