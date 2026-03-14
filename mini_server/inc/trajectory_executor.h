#ifndef TRAJECTORY_EXECUTOR_H
#define TRAJECTORY_EXECUTOR_H

#include <stdbool.h>
#include <stddef.h>

// Define maximum number of points in a trajectory to avoid dynamic allocation
// issues on embedded
#define MAX_TRAJECTORY_POINTS 500
#define CONTROL_LOOP_RATE_HZ 20
#define CONTROL_LOOP_DELAY_US (1000000 / CONTROL_LOOP_RATE_HZ)

// Pure Pursuit / Constant Velocity Mode parameters
#define LOOKAHEAD_DISTANCE \
  0.20f                                // Lookahead distance in meters (increased for stability)
#define MIN_LOOKAHEAD_DISTANCE 0.05f   // Minimum lookahead when near goal
#define ACCEPTANCE_RADIUS 0.02f        // Final goal acceptance radius (2cm)
#define ACCEPTANCE_ANGLE \
  0.015f                             // Final goal acceptance angle (approx 3 degrees)
#define ACCEPTANCE_HOLD_TIME_MS 1500 // Hold time in ms at goal before stopping (reduced)
#define MAX_VELOCITY 0.15f           // Constant cruise velocity in m/s (reduced for smoother motion)
#define MIN_VELOCITY 0.05f           // Minimum velocity during final approach (reduced)
#define DECEL_RADIUS \
  0.50f // Start decelerating at this distance from final goal

// PID Control Parameters for XY Tracking (Tuned for gentler response)
#define TRAJ_KP 1.5f     // P gain (reduced from 2.0 for softer response)
#define TRAJ_KI 0.01f    // Integral gain (reduced from 0.0146)
#define TRAJ_KD 0.15f    // D gain (increased from 0.0782 for more damping)
#define MAX_I_TERM 0.05f // Limit integral term influence (m/s)

// Theta Control Parameters (tuned for small robot 30x26cm)
#define KP_THETA 0.5f        // P-correction gain (feedforward is main controller)
#define MAX_ANGULAR_VEL 0.5f // Maximum angular velocity in rad/s

// Docking parameters (global frame commands)
#define DOCK_SCAN_VX 0.05f
#define DOCK_ALIGN_VX 0.05f
#define DOCK_APPROACH_VY 0.05f
#define DOCK_X_TOL 0.01f
#define DOCK_TARGET_Z 0.16f
#define DOCK_Y_TOL 0.01f
#define DOCK_SCAN_TIMEOUT_MS 2500
#define DOCK_TOTAL_TIMEOUT_MS 12000
#define VISION_STALE_MS 500 // Max age of camera data (ms); older -> treat as not found

typedef struct
{
  float x;
  float y;
  float theta; // Optional, can be used for orientation
  float t;     // Timestamp or duration
  bool has_theta;
} TrajectoryPoint;

typedef struct
{
  TrajectoryPoint points[MAX_TRAJECTORY_POINTS];
  int count;
  int current_index;
  bool active;
  long start_time_ms;
  double start_time_epoch; // Scheduled start time (epoch seconds), mandatory
} Trajectory;

// Initialize the trajectory executor resources
void trajectory_init(void);

// Load a trajectory from a JSON string
// Expected JSON format: {"trajectory": [{"x": 1.0, "y": 2.0, "t":
// 1234567890.0}, ...]}
bool trajectory_load(const char *json_str);

// Start executing the currently loaded trajectory at a specific epoch timestamp
// This is the ONLY way to start trajectory execution (no immediate start)
void trajectory_start_at(double start_time);

// Stop execution immediately
void trajectory_stop(void);

// Thread function to run the control loop
void *trajectory_thread_func(void *arg);

// Check if trajectory is currently running
bool trajectory_is_running(void);

// Helper to set current robot position for the controller
// This should be called by the localization thread or main loop updates
void trajectory_set_current_pose(float x, float y, float theta);

// Start docking test mode (no trajectory required)
bool trajectory_start_docking_test(void);

#endif // TRAJECTORY_EXECUTOR_H
