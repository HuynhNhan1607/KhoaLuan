#include "trajectory_executor.h"
#include "cJSON.h"
#include "client_manager.h"
#include "ekf.h"               // Include EKF header
#include "formation_manager.h" // For cooperative transport
#include "localize.h"          // For current pose if needed directly, though we might inject it
#include "socket.h"            // For send_to_laptop_clients
#include "sys_config.h"        // For ENABLE_THETA_TRACKING
#include "docking.h"           // For VL53L0X docking
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

// Global trajectory state
static Trajectory g_trajectory;
static pthread_mutex_t g_traj_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_t g_traj_thread;
static bool g_thread_running = false;

// Current robot pose (Using EKF)
extern ekf_t g_ekf;
extern pthread_mutex_t g_ekf_mutex;

// Current robot pose (Using Odometry) - for USE_ODOMETRY_FOR_TRAJECTORY=1
extern float g_odom_x, g_odom_y, g_odom_theta;
extern pthread_mutex_t g_odom_mutex;

// Global Last Command for Feedback Sign Correction (Unsigned Odom Fix)
float g_last_cmd_vx = 0.0f;
float g_last_cmd_vy = 0.0f;

// PID Logging for tuning (controlled by ENABLE_PID_LOGGING in sys_config.h)
#if ENABLE_PID_LOGGING
static FILE *g_pid_log_file = NULL;
static double g_pid_log_start_time = 0.0;
#define PID_LOG_FILE "./tools/pid_log.csv"
#endif

// Helper to get current time in ms
static long get_time_ms(void)
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
}

void trajectory_init(void)
{
  pthread_mutex_lock(&g_traj_mutex);
  g_trajectory.count = 0;
  g_trajectory.current_index = 0;
  g_trajectory.active = false;
  pthread_mutex_unlock(&g_traj_mutex);

  // Always start the execution thread so docking_update() runs
  // even when no trajectory is loaded (standalone docking test).
  if (!g_thread_running)
  {
    g_thread_running = true;
    if (pthread_create(&g_traj_thread, NULL, trajectory_thread_func, NULL) != 0)
    {
      printf("[TRAJ] Failed to create execution thread\n");
      g_thread_running = false;
    }
    else
    {
      pthread_detach(g_traj_thread);
      printf("[TRAJ] Execution thread started\n");
    }
  }
}

void trajectory_correct_heading(float target_theta)
{
  printf("[TRAJ] Correcting heading to %.2f rad (%.1f deg)...\n",
         target_theta, target_theta * 180.0f / (float)M_PI);

  const float KP = 1.0f;
  const float MAX_W = 0.5f;
  const float TOLERANCE = 0.05f; /* ~3 degrees */
  const int MAX_ITERS = 200;     /* ~10 s at 20 Hz */

  for (int i = 0; i < MAX_ITERS; i++)
  {
    pthread_mutex_lock(&g_ekf_mutex);
    float cur_theta = (float)g_ekf.x[4];
    pthread_mutex_unlock(&g_ekf_mutex);

    float err = target_theta - cur_theta;
    while (err > (float)M_PI)
      err -= 2.0f * (float)M_PI;
    while (err < -(float)M_PI)
      err += 2.0f * (float)M_PI;

    if (fabsf(err) < TOLERANCE)
      break;

    float cmd_w = KP * err;
    if (cmd_w > MAX_W)
      cmd_w = MAX_W;
    if (cmd_w < -MAX_W)
      cmd_w = -MAX_W;

    char rot_cmd[64];
    snprintf(rot_cmd, sizeof(rot_cmd),
             "dot_x:0.0000 dot_y:0.0000 dot_theta:%.4f\n", cmd_w);
    client_manager_broadcast_to_motor(rot_cmd, strlen(rot_cmd));

    usleep(CONTROL_LOOP_DELAY_US);
  }

  /* Stop rotation */
  const char *stop = "dot_x:0.0000 dot_y:0.0000 dot_theta:0.0000\n";
  client_manager_broadcast_to_motor(stop, strlen(stop));
  printf("[TRAJ] Heading correction complete\n");
}

void trajectory_set_current_pose(float x, float y, float theta)
{
  (void)x;
  (void)y;
  (void)theta;
  // pthread_mutex_lock(&g_pose_mutex);
  // g_current_x = x;
  // g_current_y = y;
  // g_current_theta = theta;
  // pthread_mutex_unlock(&g_pose_mutex);
}

bool trajectory_load(const char *json_str)
{
  // Parse JSON
  cJSON *root = cJSON_Parse(json_str);
  if (!root)
  {
    printf("[TRAJ] Failed to parse JSON\n");
    return false;
  }

  cJSON *traj_array = cJSON_GetObjectItem(root, "trajectory");
  if (!cJSON_IsArray(traj_array))
  {
    printf("[TRAJ] 'trajectory' field missing or not an array\n");
    cJSON_Delete(root);
    return false;
  }

  pthread_mutex_lock(&g_traj_mutex);
  g_trajectory.count = 0;
  g_trajectory.current_index = 0;
  g_trajectory.active =
      false; // Don't start yet, wait for explicit start or auto-start logic?
             // Let's assume load implies preparation.

  int size = cJSON_GetArraySize(traj_array);
  if (size > MAX_TRAJECTORY_POINTS)
  {
    printf("[TRAJ] Warning: Trajectory truncated from %d to %d points\n", size,
           MAX_TRAJECTORY_POINTS);
    size = MAX_TRAJECTORY_POINTS;
  }

  for (int i = 0; i < size; i++)
  {
    cJSON *point = cJSON_GetArrayItem(traj_array, i);
    if (point)
    {
      cJSON *x = cJSON_GetObjectItem(point, "x");
      cJSON *y = cJSON_GetObjectItem(point, "y");
      cJSON *t = cJSON_GetObjectItem(point, "t");
      cJSON *theta = cJSON_GetObjectItem(point, "theta");

      if (x && y)
      {
        g_trajectory.points[i].x = (float)x->valuedouble;
        g_trajectory.points[i].y = (float)y->valuedouble;
        // Use provided timestamp or default
        g_trajectory.points[i].t = t ? (float)t->valuedouble : 0.0f;

        if (theta)
        {
          g_trajectory.points[i].theta = (float)theta->valuedouble;
          g_trajectory.points[i].has_theta = true;
        }
        else
        {
          g_trajectory.points[i].theta = 0.0f;
          g_trajectory.points[i].has_theta = false;
        }
        g_trajectory.count++;
      }
    }
  }
  pthread_mutex_unlock(&g_traj_mutex);

  cJSON_Delete(root);
  printf("[TRAJ] Loaded %d points\n", g_trajectory.count);
  return true;
}

// Helper to get current epoch time in seconds (double)
static double get_time_epoch(void)
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (double)tv.tv_sec + (double)tv.tv_usec / 1000000.0;
}

void trajectory_start_at(double start_time)
{
  if (start_time <= 0.0)
  {
    printf("[TRAJ] ERROR: Invalid start_time (%.3f). Must be > 0!\n",
           start_time);
    return;
  }

  // Nếu đang ở transport phase, chỉnh heading về 0 trước khi chạy trajectory
  if (formation_is_transport_active())
  {
    printf("[TRAJ] Transport trajectory: correcting heading to 0 before start\n");
    trajectory_correct_heading(0.0f);
  }

  pthread_mutex_lock(&g_traj_mutex);
  if (g_trajectory.count > 0)
  {
    g_trajectory.active = true;
    g_trajectory.current_index = 0;
    g_trajectory.start_time_epoch = start_time;
    g_trajectory.start_time_ms = get_time_ms();
    double now = get_time_epoch();
    printf("[TRAJ] Execution scheduled at %.3f (current: %.3f, wait: %.3fs)\n",
           start_time, now, start_time - now);

#if ENABLE_PID_LOGGING
    // Open PID log file
    g_pid_log_file = fopen(PID_LOG_FILE, "w");
    if (g_pid_log_file)
    {
      g_pid_log_start_time = now;
      fprintf(g_pid_log_file, "time,tgt_x,tgt_y,cur_x,cur_y,err_x,err_y,cmd_vx,"
                              "cmd_vy,P_x,P_y,I_x,I_y,D_x,D_y\n");
      printf("[PID_LOG] Started logging to %s\n", PID_LOG_FILE);
    }
#endif
  }
  else
  {
    printf("[TRAJ] Cannot start: No trajectory loaded\n");
  }
  pthread_mutex_unlock(&g_traj_mutex);

  // Ensure thread is running
  if (!g_thread_running)
  {
    g_thread_running = true;
    if (pthread_create(&g_traj_thread, NULL, trajectory_thread_func, NULL) !=
        0)
    {
      printf("[TRAJ] Failed to create execution thread\n");
      g_thread_running = false;
    }
  }
}

void trajectory_stop(void)
{
  pthread_mutex_lock(&g_traj_mutex);
  g_trajectory.active = false;
  pthread_mutex_unlock(&g_traj_mutex);

  // Send 0 velocity to stop
  char cmd[64];
  snprintf(cmd, sizeof(cmd), "dot_x:0.0000 dot_y:0.0000 dot_theta:0.0000\n");
  client_manager_broadcast_to_motor(cmd, strlen(cmd));

  // Reset global command state
  g_last_cmd_vx = 0.0f;
  g_last_cmd_vy = 0.0f;

#if ENABLE_PID_LOGGING
  // Close PID log file
  if (g_pid_log_file)
  {
    fclose(g_pid_log_file);
    g_pid_log_file = NULL;
    printf("[PID_LOG] Stopped logging - file saved to %s\n", PID_LOG_FILE);
  }
#endif

  printf("[TRAJ] Execution stopped\n");
}

bool trajectory_is_running(void)
{
  bool active;
  pthread_mutex_lock(&g_traj_mutex);
  active = g_trajectory.active;
  pthread_mutex_unlock(&g_traj_mutex);
  return active;
}

void *trajectory_thread_func(void *arg)
{
  (void)arg;
  printf("[TRAJ] Thread started (Constant Velocity Mode)\n");

  // Pure Pursuit parameters (from trajectorsy_executor.h)
  const float lookahead_dist = LOOKAHEAD_DISTANCE;

  const float acceptance_radius = ACCEPTANCE_RADIUS;
  // const float max_vel = MAX_VELOCITY; // Used in PID logic directly now

  while (g_thread_running)
  {
    // PID State
    static float last_pos_x = 0.0f;
    static float last_pos_y = 0.0f;
    static float sum_err_x = 0.0f;
    static float sum_err_y = 0.0f;
    static double last_control_time = 0.0;
    static bool first_run_pid = true;

    // Velocity smoothing state (low-pass filter)
    static float smooth_cmd_vx = 0.0f;
    static float smooth_cmd_vy = 0.0f;

    // D-term filtering state
    static float filtered_d_meas_x = 0.0f;
    static float filtered_d_meas_y = 0.0f;

    // Reset hold timer if not running or just starting (handled by ensuring
    // it's 0 when condition fails)
    static long goal_reached_start_time = 0;

    pthread_mutex_lock(&g_traj_mutex);
    bool active = g_trajectory.active;
    int count = g_trajectory.count;
    int current_idx = g_trajectory.current_index;
    double start_time = g_trajectory.start_time_epoch;
    pthread_mutex_unlock(&g_traj_mutex);

    // ╔══════════════════════════════════════════════════════════════════════╗
    // ║  DOCKING BLOCK (A): Run docking loop if active                     ║
    // ║                                                                    ║
    // ║  Flow: trajectory PID → acceptance zone → docking_start() [Block B]║
    // ║        → THIS BLOCK runs docking_update() every cycle              ║
    // ║        → when docking completes, sends "arrived" to server         ║
    // ║        → server receives "arrived" → sends execute_grip            ║
    // ║                                                                    ║
    // ║  Also handles standalone docking test (no trajectory loaded):      ║
    // ║        → server sends start_docking command → docking_start()      ║
    // ║        → THIS BLOCK runs docking_update() every cycle              ║
    // ║        → when completes, sends "docking_complete" (UI only)        ║
    // ╚══════════════════════════════════════════════════════════════════════╝
#if ENABLE_DOCKING == 1
    if (docking_is_active())
    {
      // Get current heading for body→global frame conversion
      float dock_theta;
      pthread_mutex_lock(&g_ekf_mutex);
      dock_theta = (float)g_ekf.x[4];
      pthread_mutex_unlock(&g_ekf_mutex);

      // Run one cycle of the VL53L0X docking state machine
      docking_update(dock_theta);

      // Check if docking just completed
      if (docking_is_complete())
      {
        printf("[TRAJ] Docking complete!\n");

        if (count > 0)
        {
          // Full flow — send "arrived", then SHUTDOWN docking completely
          // so transport phase has zero interference from VL53L0X
          const char *msg =
              "{\"type\": \"control\", \"status\": \"arrived\"}\n";
          send_to_laptop_clients(msg, strlen(msg));
          printf("[TRAJ] Sent 'arrived' → server will send execute_grip\n");

          // Tắt hoàn toàn: đóng I2C, giải phóng GPIO, không đọc sensor nữa
          docking_shutdown();
          printf("[TRAJ] Docking shutdown — sensors released for transport\n");
        }
        else
        {
          // Standalone test — chỉ cập nhật UI, không shutdown
          const char *msg =
              "{\"type\": \"control\", \"status\": \"docking_complete\"}\n";
          send_to_laptop_clients(msg, strlen(msg));
          printf("[TRAJ] Sent 'docking_complete' (standalone test)\n");
        }

        // Reset PID state for next trajectory
        first_run_pid = true;
        sum_err_x = 0.0f;
        sum_err_y = 0.0f;
        smooth_cmd_vx = 0.0f;
        smooth_cmd_vy = 0.0f;
        filtered_d_meas_x = 0.0f;
        filtered_d_meas_y = 0.0f;
        goal_reached_start_time = 0;
      }

      usleep(DOCKING_LOOP_DELAY_US);
      continue; // Docking controls motors — skip PID entirely
    }
#endif // ENABLE_DOCKING

    // --- IDLE CHECK: No trajectory loaded ---
    if (!active || count == 0)
    {
      usleep(100000); // Sleep 100ms when idle
      continue;
    }

    // === ROBOT2: Skip trajectory execution during transport mode ===
#if ROBOT_ID == 2
    if (formation_is_transport_active())
    {
      usleep(100000);
      continue;
    }
#endif

    // --- CHECK START TIME (MANDATORY) ---
    if (start_time > 0.0)
    {
      double now = get_time_epoch();
      if (now < start_time)
      {
        // Still waiting for scheduled start time
        double diff = start_time - now;
        if (diff > 0.1)
        {
          usleep(100000); // Sleep 100ms if far from start time
        }
        else
        {
          usleep((useconds_t)(diff * 1000000)); // Sleep exact remaining time
        }
        continue;
      }
      else
      {
        // Start time reached! Clear it and begin execution
        pthread_mutex_lock(&g_traj_mutex);
        g_trajectory.start_time_epoch = 0.0;
        pthread_mutex_unlock(&g_traj_mutex);
        printf("[TRAJ] Scheduled start time reached! Beginning execution.\n");
      }
    }

    // Get current pose (Global Frame)
    float cur_x, cur_y, cur_theta;
#if USE_ODOMETRY_FOR_TRAJECTORY
    // Use raw odometry for x, y (no sensor fusion)
    pthread_mutex_lock(&g_odom_mutex);
    cur_x = g_odom_x;
    cur_y = g_odom_y;
    pthread_mutex_unlock(&g_odom_mutex);
    // But still use EKF for theta (IMU fused)
    pthread_mutex_lock(&g_ekf_mutex);
    cur_theta = (float)g_ekf.x[4];
    pthread_mutex_unlock(&g_ekf_mutex);
#else
    // Use EKF (sensor fusion: UWB + Odometry + Optical Flow)
    pthread_mutex_lock(&g_ekf_mutex);
    cur_x = (float)g_ekf.x[0];
    cur_y = (float)g_ekf.x[1];
    cur_theta = (float)g_ekf.x[4];
    pthread_mutex_unlock(&g_ekf_mutex);
#endif

    // === VIRTUAL STRUCTURE: For Pure Pursuit search, use centroid position ===
    float search_x = cur_x;
    float search_y = cur_y;
    double transport_offset_x = 0.0, transport_offset_y = 0.0;
    bool is_transport = formation_is_transport_active();
    bool use_transport_offset = is_transport;

    if (use_transport_offset)
    {
      if (formation_get_transport_offset(&transport_offset_x, &transport_offset_y))
      {
        // search_x = cur_x - (float)transport_offset_x;
        search_y = cur_y - (float)transport_offset_y;
      }
    }

    // --- PURE PURSUIT: Find Lookahead Point ---
    int lookahead_idx = current_idx;
    for (int i = current_idx; i < count; i++)
    {
      float dx_search = g_trajectory.points[i].x - search_x;
      float dy_search = g_trajectory.points[i].y - search_y;
      float dist_to_point =
          sqrtf(dx_search * dx_search + dy_search * dy_search);

      if (dist_to_point > lookahead_dist)
      {
        lookahead_idx = i;
        break;
      }
      lookahead_idx = i;
    }

    if (lookahead_idx >= count)
    {
      lookahead_idx = count - 1;
    }

    pthread_mutex_lock(&g_traj_mutex);
    if (lookahead_idx > g_trajectory.current_index)
    {
      g_trajectory.current_index = lookahead_idx;
    }
    pthread_mutex_unlock(&g_traj_mutex);

    TrajectoryPoint target = g_trajectory.points[lookahead_idx];

    // === VIRTUAL STRUCTURE: Convert centroid target to robot target ===
    if (use_transport_offset)
    {
      formation_set_centroid_target((double)target.x, (double)target.y);
      double robot_target_x, robot_target_y;
      if (formation_get_robot_target(&robot_target_x, &robot_target_y))
      {
        // target.x = (float)robot_target_x;
        target.y = (float)robot_target_y;
      }
    }

    // --- STOP CONDITION: Check POSITION (and THETA if enabled) ---
    TrajectoryPoint final_point = g_trajectory.points[count - 1];

    if (use_transport_offset)
    {
      // final_point.x += (float)transport_offset_x;
      final_point.y += (float)transport_offset_y;
    }

    float dx_final = final_point.x - cur_x;
    float dy_final = final_point.y - cur_y;
    float distance_to_final = sqrtf(dx_final * dx_final + dy_final * dy_final);

    bool goal_reached = false;
    if (distance_to_final < acceptance_radius)
    {
#if ENABLE_THETA_TRACKING
      // In transport mode skip theta check — position-only acceptance
      if (!is_transport && final_point.has_theta)
      {
        float theta_err = cur_theta - final_point.theta;
        if (fabs(theta_err) < ACCEPTANCE_ANGLE)
        {
          goal_reached = true;
        }
      }
      else
      {
        goal_reached = true;
      }
#else
      goal_reached = true;
#endif
    }

    if (goal_reached)
    {
      // ╔══════════════════════════════════════════════════════════════════╗
      // ║  DOCKING BLOCK (B): Start docking when entering acceptance zone║
      // ║  Only for phase 1 (not transport mode — docking already done)  ║
      // ╚══════════════════════════════════════════════════════════════════╝
#if ENABLE_DOCKING == 1
      if (!is_transport && !docking_is_active() && !docking_is_complete())
      {
        printf("[TRAJ] Entered acceptance zone — starting VL53L0X docking\n");
        printf("[TRAJ] Cur(%.2f, %.2f) Final(%.2f, %.2f) Dist:%.3f\n",
               cur_x, cur_y, final_point.x, final_point.y, distance_to_final);

        trajectory_stop(); // Dừng PID
        docking_start();   // Bật VL53L0X sensors
        goal_reached_start_time = 0;
        continue; // → Block (A) sẽ chạy docking_update() ở vòng sau
      }
      if (!is_transport && (docking_is_active() || docking_is_complete()))
      {
        // Docking đã active hoặc complete → Block (A) xử lý
        usleep(CONTROL_LOOP_DELAY_US);
        continue;
      }
      // is_transport == true: fall through to hold-then-arrived logic below
#endif // ENABLE_DOCKING

      // === HOLD THEN ARRIVED (transport phase 2, or no-docking build) ===
      if (goal_reached_start_time == 0)
      {
        goal_reached_start_time = get_time_ms();
        printf("[TRAJ] Entered acceptance zone. Holding for %dms...\n",
               ACCEPTANCE_HOLD_TIME_MS);
      }

      long elapsed = get_time_ms() - goal_reached_start_time;
      if (elapsed >= ACCEPTANCE_HOLD_TIME_MS)
      {
        trajectory_stop();
        printf("[TRAJ] Trajectory complete! Cur(%.2f, %.2f) Final(%.2f, %.2f) "
               "Dist:%.3f Radius:%.3f Held:%ldms\n",
               cur_x, cur_y, final_point.x, final_point.y, distance_to_final,
               acceptance_radius, elapsed);

        const char *arrived_msg =
            "{\"type\": \"control\", \"status\": \"arrived\"}\n";
        send_to_laptop_clients(arrived_msg, strlen(arrived_msg));
        printf("[TRAJ] Sent arrival notification to laptop\n");

        goal_reached_start_time = 0;
        continue;
      }

      // HOLDING: Send zero velocity while waiting in acceptance zone
      char hold_cmd[64];
      snprintf(hold_cmd, sizeof(hold_cmd), "dot_x:0.0000 dot_y:0.0000 dot_theta:0.0000\n");
      client_manager_broadcast_to_motor(hold_cmd, strlen(hold_cmd));
      usleep(CONTROL_LOOP_DELAY_US);
      continue;
    }
    else
    {
      // Left the acceptance zone or haven't reached it
      if (goal_reached_start_time != 0)
      {
        printf("[TRAJ] Left acceptance zone (Held: %ldms). Resetting timer.\n",
               get_time_ms() - goal_reached_start_time);
        goal_reached_start_time = 0;
      }
    }

    // --- ADAPTIVE PID VELOCITY CONTROL ---
    // Calculate dt for PID
    double cur_time = get_time_epoch();
    float dt = (float)(cur_time - last_control_time);
    if (last_control_time == 0.0 || dt <= 0.001f || dt > 0.2f)
    {
      dt = 0.05f; // Default 20Hz
    }
    last_control_time = cur_time;

    // Error vector (Global Frame)
    // Note: For Pure Pursuit, we target 'target' (Lookahead point).
    // Ideally PID should track the path, but tracking Lookahead Point is a
    // robust simplification.
    float err_x = target.x - cur_x;
    float err_y = target.y - cur_y;

    // P-Term with constant Kp (fixed gain)
    float P_x = TRAJ_KP * err_x;
    float P_y = TRAJ_KP * err_y;

    // I-Term
    // Only integrate if moving (active) and not stopped
    sum_err_x += err_x * dt;
    sum_err_y += err_y * dt;

    // Anti-windup (Clamping I-term magnitude)
    float i_mag = sqrtf(sum_err_x * sum_err_x + sum_err_y * sum_err_y);
    float max_i_accum = MAX_I_TERM / TRAJ_KI; // Back-calculate max accumulator
    if (i_mag > max_i_accum && max_i_accum > 0.0001f)
    {
      float scale = max_i_accum / i_mag;
      sum_err_x *= scale;
      sum_err_y *= scale;
    }
    float I_x = TRAJ_KI * sum_err_x;
    float I_y = TRAJ_KI * sum_err_y;

    if (first_run_pid)
    {
      last_pos_x = cur_x;
      last_pos_y = cur_y;
      first_run_pid = false;
    }

    // D-Term (Derivative on Measurement) with LOW-PASS FILTER
    // Avoids "Derivative Kick" when target changes abruptly
    // Filter to reduce noise and jerk
    float d_meas_x = (cur_x - last_pos_x) / dt;
    float d_meas_y = (cur_y - last_pos_y) / dt;

    // Low-pass filter for D-term (reduces jerk from noisy measurements)
    const float D_FILTER_ALPHA = 0.4f; // 0=max smoothing, 1=no filter (LOWER = smoother)
    filtered_d_meas_x = D_FILTER_ALPHA * d_meas_x + (1.0f - D_FILTER_ALPHA) * filtered_d_meas_x;
    filtered_d_meas_y = D_FILTER_ALPHA * d_meas_y + (1.0f - D_FILTER_ALPHA) * filtered_d_meas_y;

    float D_x = -TRAJ_KD * filtered_d_meas_x;
    float D_y = -TRAJ_KD * filtered_d_meas_y;

    last_pos_x = cur_x;
    last_pos_y = cur_y;

    // Total Command (raw PID output)
    float raw_cmd_vx = P_x + I_x + D_x;
    float raw_cmd_vy = P_y + I_y + D_y;

    // --- Speed Limiting (Deceleration Logic) ---
    // We still apply the speed limit based on distance to FINAL goal
    // This scales the PID vector down if it exceeds the desired speed

    // Calculate desired max speed at this point
    float speed_limit = MAX_VELOCITY;
    if (distance_to_final < DECEL_RADIUS)
    {
      float min_speed = MIN_VELOCITY;
      speed_limit = min_speed + (MAX_VELOCITY - min_speed) *
                                    (distance_to_final - acceptance_radius) /
                                    (DECEL_RADIUS - acceptance_radius);
      if (speed_limit < min_speed)
        speed_limit = min_speed;
    }

    // Clamp magnitude (smooth scaling instead of hard clipping)
    float raw_cmd_mag = sqrtf(raw_cmd_vx * raw_cmd_vx + raw_cmd_vy * raw_cmd_vy);
    float cmd_dot_x = raw_cmd_vx;
    float cmd_dot_y = raw_cmd_vy;
    if (raw_cmd_mag > speed_limit)
    {
      float scale = speed_limit / raw_cmd_mag;
      cmd_dot_x *= scale;
      cmd_dot_y *= scale;
    }

    // --- VELOCITY SMOOTHING (Low-pass filter) ---
    // Critical for reducing jerk! Smooths out sudden command changes
    const float VEL_SMOOTH_ALPHA = 0.8f; // 0=max smoothing, 1=no smoothing (TUNED: 0.6 = good balance)
    smooth_cmd_vx = VEL_SMOOTH_ALPHA * cmd_dot_x + (1.0f - VEL_SMOOTH_ALPHA) * smooth_cmd_vx;
    smooth_cmd_vy = VEL_SMOOTH_ALPHA * cmd_dot_y + (1.0f - VEL_SMOOTH_ALPHA) * smooth_cmd_vy;

    // --- ACCELERATION LIMITING (reduces jerk further) ---
    const float MAX_ACCEL = 0.5f; // m/s^2 - maximum acceleration
    float delta_vx = smooth_cmd_vx - g_last_cmd_vx;
    float delta_vy = smooth_cmd_vy - g_last_cmd_vy;
    float accel_mag = sqrtf(delta_vx * delta_vx + delta_vy * delta_vy) / dt;

    if (accel_mag > MAX_ACCEL)
    {
      float accel_scale = (MAX_ACCEL * dt) / sqrtf(delta_vx * delta_vx + delta_vy * delta_vy);
      smooth_cmd_vx = g_last_cmd_vx + delta_vx * accel_scale;
      smooth_cmd_vy = g_last_cmd_vy + delta_vy * accel_scale;
    }

    // Final command (smoothed + acceleration limited)
    cmd_dot_x = smooth_cmd_vx;
    cmd_dot_y = smooth_cmd_vy;

    // === COOPERATIVE TRANSPORT: Speed Scaling for Formation Maintenance ===
#if ROBOT_ID == 1 && !SINGLE_ROBOT_MODE
    // Robot1 (Leader): Scale down speed if Robot2 (Follower) is lagging behind
    // Priority: Formation maintenance > Trajectory execution
    // NOTE: Disabled in SINGLE_ROBOT_MODE - Robot1 luôn chạy full speed

    double follower_error = 0.0;
    static float formation_speed_scale = 1.0f; // Smooth scaling factor

    if (is_transport &&
        formation_get_follower_error(&follower_error))
    {
      // Calculate target speed scale with hysteresis
      float target_scale = 1.0f;

      if (follower_error > FOLLOWER_ERROR_MAX)
      {
        // Robot2 too far behind -> stop completely
        target_scale = 0.0f;
      }
      else if (follower_error > FOLLOWER_ERROR_DEADBAND)
      {
        // Smooth scaling between deadband and max error
        // scale = 1 - (error/max_error)^2 (quadratic for smooth deceleration)
        float error_ratio = (follower_error - FOLLOWER_ERROR_DEADBAND) /
                            (FOLLOWER_ERROR_MAX - FOLLOWER_ERROR_DEADBAND);
        target_scale = 1.0f - error_ratio * error_ratio;
      }
      // else: error <= deadband -> full speed (target_scale = 1.0)

      // Apply smooth filtering to prevent jitter (low-pass filter)
      // formation_speed_scale = α * target + (1-α) * old
      const float SMOOTH_ALPHA = 0.3f; // Faster response for quicker recovery
      formation_speed_scale = SMOOTH_ALPHA * target_scale +
                              (1.0f - SMOOTH_ALPHA) * formation_speed_scale;

      // Apply formation speed scaling
      cmd_dot_x *= formation_speed_scale;
      cmd_dot_y *= formation_speed_scale;

      // Debug log every 2 seconds (40 cycles at 20Hz)
      static int formation_debug_cnt = 0;
      if (++formation_debug_cnt >= 40)
      {
        printf("[COOPERATIVE] Follower error=%.1fcm scale=%.2f target_scale=%.2f\n",
               follower_error * 100.0, formation_speed_scale, target_scale);
        formation_debug_cnt = 0;
      }
    }
    else
    {
      // No formation active or no follower data -> reset to full speed
      formation_speed_scale = 1.0f;
    }
#endif // ROBOT_ID == 1 && !SINGLE_ROBOT_MODE

    // --- Update Global Command State (for Unsigned Odom Correction) ---
    g_last_cmd_vx = cmd_dot_x;
    g_last_cmd_vy = cmd_dot_y;

    // --- Orientation Control (Feedforward + P-Correction) ---
    // Use trajectory timestamps to calculate required angular velocity
    float cmd_dot_theta = 0.0f;

    // === TRANSPORT MODE: Lock theta to prevent arm breakage ===
    // During transport, trajectory theta is the OBJECT's theta (always 0),
    // NOT the robot's theta. Robot must maintain its locked theta.
#if ROBOT_ID == 1
    if (is_transport)
    {
      // Always hold heading at 0 during transport
      float theta_err = cur_theta - 0.0f;
      while (theta_err > (float)M_PI)
        theta_err -= 2.0f * (float)M_PI;
      while (theta_err < -(float)M_PI)
        theta_err += 2.0f * (float)M_PI;

      const float KP_THETA_HOLD = 1.0f;
      const float MAX_ANGULAR_VEL_HOLD = 0.5f;
      cmd_dot_theta = -KP_THETA_HOLD * theta_err;
      if (cmd_dot_theta > MAX_ANGULAR_VEL_HOLD)
        cmd_dot_theta = MAX_ANGULAR_VEL_HOLD;
      if (cmd_dot_theta < -MAX_ANGULAR_VEL_HOLD)
        cmd_dot_theta = -MAX_ANGULAR_VEL_HOLD;
    }
    else
#endif
    {
      // Normal mode: follow trajectory theta
#if ENABLE_THETA_TRACKING
      if (target.has_theta)
      {
        // Find previous point for feedforward calculation
        int prev_idx = (lookahead_idx > 0) ? lookahead_idx - 1 : 0;
        TrajectoryPoint prev_point = g_trajectory.points[prev_idx];

        // Calculate feedforward angular velocity from trajectory timing
        float dt = target.t - prev_point.t;
        float d_theta = target.theta - prev_point.theta;

        // Normalize d_theta to [-PI, PI] for shortest path
        while (d_theta > M_PI)
          d_theta -= 2.0f * M_PI;
        while (d_theta < -M_PI)
          d_theta += 2.0f * M_PI;

        float dot_theta_ff = 0.0f;
        if (dt > 0.01f)
        { // Avoid division by zero
          dot_theta_ff = d_theta / dt;
        }

        // Small P-correction to track actual theta
        float theta_err = cur_theta - target.theta;
        while (theta_err > M_PI)
          theta_err -= 2.0f * M_PI;
        while (theta_err < -M_PI)
          theta_err += 2.0f * M_PI;
        float dot_theta_p = theta_err * KP_THETA; // KP_THETA is now small (0.5)

        // Combine feedforward + P-correction
        // Feedforward: follow trajectory timing (d_theta/dt)
        // P-correction: reduce tracking error (negative feedback)
        cmd_dot_theta = dot_theta_ff - dot_theta_p;

        // Clamp to max angular velocity
        if (cmd_dot_theta > MAX_ANGULAR_VEL)
          cmd_dot_theta = MAX_ANGULAR_VEL;
        if (cmd_dot_theta < -MAX_ANGULAR_VEL)
          cmd_dot_theta = -MAX_ANGULAR_VEL;
      }
#endif
    }

    // --- Send Command to Robot ---
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "dot_x:%.4f dot_y:%.4f dot_theta:%.4f\n",
             cmd_dot_x, cmd_dot_y, cmd_dot_theta);
    client_manager_broadcast_to_motor(cmd, strlen(cmd));

#if ENABLE_PID_LOGGING
    // Log PID data at 20Hz to CSV file
    if (g_pid_log_file)
    {
      double now = get_time_epoch();
      double time_since_start = now - g_pid_log_start_time;
      fprintf(g_pid_log_file,
              "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%."
              "4f,%.4f,%.4f\n",
              time_since_start, target.x, target.y, cur_x, cur_y, err_x, err_y,
              cmd_dot_x, cmd_dot_y, P_x, P_y, I_x, I_y, D_x, D_y);
    }
#endif

    // Debug output (every 1 second at 20Hz) - ENABLED for PID tuning
    static int debug_cnt = 0;
    if (++debug_cnt >= 20)
    {
      printf("[TRAJ] Cur(%.3f,%.3f) -> Tgt(%.3f,%.3f) DistToFinal:%.3f SpeedLimit:%.3f\n",
             cur_x, cur_y, target.x, target.y, distance_to_final, speed_limit);
      // PID tuning debug - show individual contributions
      printf("[PID] Err(%.3f,%.3f) P(%.3f,%.3f) I(%.3f,%.3f) D(%.3f,%.3f) Cmd(%.3f,%.3f)\n",
             err_x, err_y, P_x, P_y, I_x, I_y, D_x, D_y, cmd_dot_x, cmd_dot_y);
      debug_cnt = 0;
    }

    usleep(CONTROL_LOOP_DELAY_US);
  }
  return NULL;
}
