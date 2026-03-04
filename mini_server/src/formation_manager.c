#include "formation_manager.h"
#include "client_manager.h"
#include "ekf.h"
#include "sys_config.h"
#include "trajectory_executor.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// External EKF (vị trí robot hiện tại)
extern ekf_t g_ekf;
extern pthread_mutex_t g_ekf_mutex;

// Formation state
typedef struct
{
  // =========== LEADER-FOLLOWER MODE ===========
  // Vị trí và vận tốc của neighbor (leader)
  double neighbor_x;
  double neighbor_y;
  double neighbor_vx;
  double neighbor_vy;
  double neighbor_theta;
  double neighbor_ts;

  // Offset ban đầu (this_robot - neighbor) tại thời điểm lock
  double initial_offset_x;
  double initial_offset_y;

  // Trạng thái leader-follower
  bool is_locked;         // Đã có offset ban đầu chưa
  bool follow_enabled;    // Có đang ở chế độ follow không
  bool has_neighbor_data; // Đã nhận được dữ liệu từ neighbor chưa

  // Thread control
  pthread_t follow_thread;
  bool thread_running;

  // Last user control timestamp
  double last_control_ts;

  // =========== TRANSPORT MODE ===========
  // Offset từ robot đến centroid (robot_pos - centroid)
  double transport_offset_x;
  double transport_offset_y;

  // Centroid target từ trajectory
  double centroid_target_x;
  double centroid_target_y;

  // Expected neighbor position (nếu là follower)
  // = centroid + neighbor_offset (tính từ formation_offset gửi từ server)
  double neighbor_offset_x;
  double neighbor_offset_y;
  bool neighbor_offset_valid; // True nếu đã tính neighbor_offset từ fresh data

  // Transport mode active
  bool transport_active;

  // Locked theta (heading) during transport - to prevent arm breakage
  double transport_locked_theta;

  // Mutex
  pthread_mutex_t mutex;
} formation_state_t;

static formation_state_t g_formation = {.is_locked = false,
                                        .follow_enabled =
                                            false, // Mặc định TẮT follow
                                        .has_neighbor_data = false,
                                        .thread_running = false,
                                        .last_control_ts = 0.0,
                                        .transport_offset_x = 0.0,
                                        .transport_offset_y = 0.0,
                                        .centroid_target_x = 0.0,
                                        .centroid_target_y = 0.0,
                                        .neighbor_offset_x = 0.0,
                                        .neighbor_offset_y = 0.0,
                                        .neighbor_offset_valid = false,
                                        .transport_active = false,
                                        .transport_locked_theta = 0.0,
                                        .mutex = PTHREAD_MUTEX_INITIALIZER};

// Lấy timestamp hiện tại (epoch seconds)
static double get_current_epoch_time(void)
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (double)tv.tv_sec + (double)tv.tv_usec / 1000000.0;
}

// Forward declaration
bool formation_get_follow_velocity(double *vx, double *vy);
bool formation_get_transport_follow_velocity(double *vx, double *vy);

// Follow thread: Gửi vận tốc mirror đến ESP32 khi không có trajectory
static void *formation_follow_thread_func(void *arg)
{
  (void)arg;
  printf("[FORMATION] Follow thread started (20Hz)\n");

  while (g_formation.thread_running)
  {
#if ROBOT_ID == 2
    // Robot 2 trong Leader-Follower transport mode:
    // Follow Robot 1's position + offset thay vì dùng trajectory
    if (g_formation.transport_active)
    {
      double follow_vx = 0.0, follow_vy = 0.0;

      if (formation_get_transport_follow_velocity(&follow_vx, &follow_vy))
      {
        // Calculate theta hold command (same as Robot1)
        // Must maintain locked theta to prevent arm breakage
        float cmd_dot_theta = 0.0f;

        pthread_mutex_lock(&g_formation.mutex);
        double locked_theta = g_formation.transport_locked_theta;
        pthread_mutex_unlock(&g_formation.mutex);

        pthread_mutex_lock(&g_ekf_mutex);
        float cur_theta = (float)g_ekf.x[4];
        pthread_mutex_unlock(&g_ekf_mutex);

        float theta_err = cur_theta - (float)locked_theta;
        // Normalize to [-PI, PI]
        while (theta_err > M_PI)
          theta_err -= 2.0f * M_PI;
        while (theta_err < -M_PI)
          theta_err += 2.0f * M_PI;

        // P-controller to hold theta
        const float KP_THETA_HOLD = 1.0f;
        const float MAX_ANGULAR_VEL_HOLD = 0.5f;
        cmd_dot_theta = -KP_THETA_HOLD * theta_err;
        if (cmd_dot_theta > MAX_ANGULAR_VEL_HOLD)
          cmd_dot_theta = MAX_ANGULAR_VEL_HOLD;
        if (cmd_dot_theta < -MAX_ANGULAR_VEL_HOLD)
          cmd_dot_theta = -MAX_ANGULAR_VEL_HOLD;

        // Có vận tốc hợp lệ -> gửi đến ESP32 (with theta hold)
        char cmd[128];
        snprintf(cmd, sizeof(cmd), "dot_x:%.4f dot_y:%.4f dot_theta:%.4f\n",
                 follow_vx, follow_vy, cmd_dot_theta);
        client_manager_broadcast_to_motor(cmd, strlen(cmd));

        // Debug log (mỗi 1 giây)
        static int debug_cnt = 0;
        if (++debug_cnt >= 20)
        {
          printf("[TRANSPORT-FOLLOW] vx=%.3f vy=%.3f theta_hold: locked=%.2f cur=%.2f err=%.3f\n",
                 follow_vx, follow_vy, locked_theta, cur_theta, theta_err);
          debug_cnt = 0;
        }
      }
      usleep(50000); // 20Hz = 50ms
      continue;      // Skip normal follow logic
    }
#endif

    // Gửi vận tốc follow khi:
    // 1. Normal mode: KHÔNG có trajectory running VÀ KHÔNG trong transport
    // 2. Leader-Follower transport: Robot2 MUST follow Robot1 (ignore trajectory)
    bool should_follow = false;
    if (!trajectory_is_running() && !g_formation.transport_active)
    {
      // Normal follow mode (không transport)
      should_follow = true;
    }
#if ROBOT_ID == 2
    else if (g_formation.transport_active)
    {
      // Robot2 trong Leader-Follower transport: override trajectory
      should_follow = false; // Handled by transport follow velocity above
    }
#endif

    if (should_follow)
    {
      double follow_vx = 0.0, follow_vy = 0.0;

      if (formation_get_follow_velocity(&follow_vx, &follow_vy))
      {
        // Có vận tốc hợp lệ từ neighbor -> gửi đến ESP32
        char cmd[128];
        snprintf(cmd, sizeof(cmd), "dot_x:%.4f dot_y:%.4f dot_theta:0.0000\n",
                 follow_vx, follow_vy);
        client_manager_broadcast_to_motor(cmd, strlen(cmd));

        // Debug log (mỗi 1 giây)
        static int debug_cnt = 0;
        if (++debug_cnt >= 20)
        {
          printf("[FORMATION] Following: vx=%.3f vy=%.3f\n", follow_vx,
                 follow_vy);
          debug_cnt = 0;
        }
      }
    }

    usleep(50000); // 20Hz = 50ms
  }

  printf("[FORMATION] Follow thread stopped\n");
  return NULL;
}

void formation_init(void)
{
  pthread_mutex_lock(&g_formation.mutex);
  g_formation.is_locked = false;
  g_formation.follow_enabled = false; // Mặc định TẮT - chỉ bật khi cần
  g_formation.has_neighbor_data = false;
  g_formation.neighbor_x = 0;
  g_formation.neighbor_y = 0;
  g_formation.neighbor_vx = 0;
  g_formation.neighbor_vy = 0;
  g_formation.neighbor_theta = 0;
  g_formation.neighbor_ts = 0;
  g_formation.initial_offset_x = 0;
  g_formation.initial_offset_y = 0;
  g_formation.last_control_ts = 0;
  g_formation.transport_active = false;
  g_formation.transport_offset_x = 0;
  g_formation.transport_offset_y = 0;
  g_formation.centroid_target_x = 0;
  g_formation.centroid_target_y = 0;
  g_formation.neighbor_offset_x = 0;
  g_formation.neighbor_offset_y = 0;
  g_formation.neighbor_offset_valid = false;
  g_formation.transport_locked_theta = 0;
  pthread_mutex_unlock(&g_formation.mutex);

  // Start follow thread
  g_formation.thread_running = true;
  if (pthread_create(&g_formation.follow_thread, NULL,
                     formation_follow_thread_func, NULL) != 0)
  {
    fprintf(stderr, "[FORMATION] Failed to create follow thread\n");
    g_formation.thread_running = false;
  }

  printf("[FORMATION] Initialized, waiting for neighbor data to lock offset\n");
}

void formation_update_neighbor(double x, double y, double vx, double vy,
                               double theta, double ts, double pos_uncertainty)
{
  (void)pos_uncertainty; // Không dùng trong code cũ

  double receive_time = get_current_epoch_time();
  double delay_s = receive_time - ts; // Delay in seconds

  // **DELAY COMPENSATION**: Extrapolate leader position to current time
  double predicted_x = x + vx * delay_s;
  double predicted_y = y + vy * delay_s;

  pthread_mutex_lock(&g_formation.mutex);

  // Cập nhật dữ liệu neighbor với vị trí đã compensate
  g_formation.neighbor_x = predicted_x;
  g_formation.neighbor_y = predicted_y;
  g_formation.neighbor_vx = vx;
  g_formation.neighbor_vy = vy;
  g_formation.neighbor_theta = theta;
  g_formation.neighbor_ts = receive_time; // Dùng thời gian LOCAL khi nhận
  g_formation.has_neighbor_data = true;

  // ========== DEFERRED OFFSET CALCULATION ==========
  // Nếu transport đã active nhưng neighbor_offset chưa valid,
  // thì tính neighbor_offset bây giờ với sync_position mới
  if (g_formation.transport_active && !g_formation.neighbor_offset_valid)
  {
    // Lấy vị trí robot hiện tại
    pthread_mutex_lock(&g_ekf_mutex);
    double my_x = g_ekf.x[0];
    double my_y = g_ekf.x[1];
    pthread_mutex_unlock(&g_ekf_mutex);

    // Tính neighbor_offset từ centroid đã lưu
    g_formation.neighbor_offset_x = predicted_x - g_formation.centroid_target_x;
    g_formation.neighbor_offset_y = predicted_y - g_formation.centroid_target_y;

    // Tính lại transport_offset (có thể đã bị outdated)
    g_formation.transport_offset_x = my_x - g_formation.centroid_target_x;
    g_formation.transport_offset_y = my_y - g_formation.centroid_target_y;

    g_formation.neighbor_offset_valid = true;
    g_formation.is_locked = true;

    printf("[FORMATION] Deferred offset calculation (fresh sync_position received):\n");
    printf("  My pos: (%.3f, %.3f)\n", my_x, my_y);
    printf("  Neighbor pos: (%.3f, %.3f)\n", predicted_x, predicted_y);
    printf("  Centroid: (%.3f, %.3f)\n", g_formation.centroid_target_x, g_formation.centroid_target_y);
    printf("  Transport offset: (%.3f, %.3f)\n", g_formation.transport_offset_x, g_formation.transport_offset_y);
    printf("  Neighbor offset: (%.3f, %.3f)\n", g_formation.neighbor_offset_x, g_formation.neighbor_offset_y);
  }

  pthread_mutex_unlock(&g_formation.mutex);
}

bool formation_get_follow_velocity(double *vx, double *vy)
{
  pthread_mutex_lock(&g_formation.mutex);

  // Kiểm tra điều kiện
  if (!g_formation.follow_enabled || !g_formation.is_locked)
  {
    pthread_mutex_unlock(&g_formation.mutex);
    *vx = 0;
    *vy = 0;
    return false;
  }

  // Kiểm tra dữ liệu có quá cũ không (timeout 1 giây)
  double current_time = get_current_epoch_time();
  if (current_time - g_formation.neighbor_ts > 1.0)
  {
    pthread_mutex_unlock(&g_formation.mutex);
    *vx = 0;
    *vy = 0;
    printf("[Timeout]\n");
    return false; // Dữ liệu quá cũ, dừng lại
  }

  // **FEEDBACK CONTROL**: Điều chỉnh velocity dựa trên sai lệch offset

  // Lấy vị trí hiện tại của follower
  pthread_mutex_lock(&g_ekf_mutex);
  double my_x = g_ekf.x[0];
  double my_y = g_ekf.x[1];
  pthread_mutex_unlock(&g_ekf_mutex);

  // Tính vị trí mục tiêu của follower (leader + offset mong muốn)
  double target_x = g_formation.neighbor_x + g_formation.initial_offset_x;
  double target_y = g_formation.neighbor_y + g_formation.initial_offset_y;

  // Tính error (sai lệch giữa vị trí thực tế và vị trí mục tiêu)
  double err_x = target_x - my_x;
  double err_y = target_y - my_y;

  // PID gains (bắt đầu với P-only control)
  const double Kp = 1.5; // Proportional gain - điều chỉnh độ mạnh của feedback

  // P-correction để kéo follower về vị trí mục tiêu
  double correction_vx = Kp * err_x;
  double correction_vy = Kp * err_y;

  // Giới hạn correction velocity (tránh quá mạnh)
  const double max_correction = 0.3; // m/s
  double correction_mag =
      sqrt(correction_vx * correction_vx + correction_vy * correction_vy);
  if (correction_mag > max_correction)
  {
    double scale = max_correction / correction_mag;
    correction_vx *= scale;
    correction_vy *= scale;
  }

  // Velocity = feedforward (neighbor velocity) + feedback (PID correction)
  // Feedforward giúp không có steady-state error khi leader di chuyển đều
  *vx = g_formation.neighbor_vx + correction_vx;
  *vy = g_formation.neighbor_vy + correction_vy;

  pthread_mutex_unlock(&g_formation.mutex);
  return true;
}

// =========== VIRTUAL STRUCTURE TRANSPORT ===========

void formation_lock_transport_offset(double centroid_x, double centroid_y)
{
  pthread_mutex_lock(&g_formation.mutex);

  // Lấy vị trí và theta robot hiện tại
  pthread_mutex_lock(&g_ekf_mutex);
  double my_x = g_ekf.x[0];
  double my_y = g_ekf.x[1];
  double my_theta = g_ekf.x[4]; // Theta from EKF state
  pthread_mutex_unlock(&g_ekf_mutex);

  // Lock offset = robot_pos - centroid (for Virtual Structure mode)
  g_formation.transport_offset_x = my_x - centroid_x;
  g_formation.transport_offset_y = my_y - centroid_y;

  // Lock theta - robot must maintain this heading during transport
  // to prevent arm breakage when both robots are holding the object
  g_formation.transport_locked_theta = my_theta;

  g_formation.transport_active = true;

  // Store centroid position for later use
  g_formation.centroid_target_x = centroid_x;
  g_formation.centroid_target_y = centroid_y;

  // Also lock initial_offset for leader-follower mode
  // initial_offset = my_pos - neighbor_pos
#if SINGLE_ROBOT_MODE
  // Chế độ 1 robot: không cần dữ liệu neighbor, lock ngay lập tức
  g_formation.is_locked = true;
  g_formation.neighbor_offset_x = 0.0;
  g_formation.neighbor_offset_y = 0.0;
  g_formation.neighbor_offset_valid = true; // Đánh dấu valid để tắt deferred calculation loop
  printf("[FORMATION] SINGLE_ROBOT_MODE: transport locked (không cần Robot2)\n");
#else
  if (g_formation.has_neighbor_data)
  {
    // Check if neighbor data is fresh (within last 0.5 seconds)
    double current_time = get_current_epoch_time();
    bool neighbor_data_fresh = (current_time - g_formation.neighbor_ts) < 0.5;

    if (neighbor_data_fresh)
    {
      g_formation.initial_offset_x = my_x - g_formation.neighbor_x;
      g_formation.initial_offset_y = my_y - g_formation.neighbor_y;
      g_formation.is_locked = true;

      // Tính neighbor offset từ centroid
      g_formation.neighbor_offset_x = g_formation.neighbor_x - centroid_x;
      g_formation.neighbor_offset_y = g_formation.neighbor_y - centroid_y;
      g_formation.neighbor_offset_valid = true;

      printf("  Neighbor offset: (%.3f, %.3f) [VALID - data age: %.0fms]\n",
             g_formation.neighbor_offset_x, g_formation.neighbor_offset_y,
             (current_time - g_formation.neighbor_ts) * 1000.0);
      printf("  Initial offset (for L-F): (%.3f, %.3f)\n",
             g_formation.initial_offset_x, g_formation.initial_offset_y);
    }
    else
    {
      // Neighbor data exists but is stale - wait for fresh data
      printf("  WARNING: Neighbor data is stale (%.0fms old). Will recalculate on next sync.\n",
             (current_time - g_formation.neighbor_ts) * 1000.0);
      g_formation.neighbor_offset_valid = false;
    }
  }
  else
  {
    // WARNING: No neighbor data yet - will need to calculate offset later
    printf("  WARNING: No neighbor data available! neighbor_offset will be calculated when first sync received.\n");
    g_formation.neighbor_offset_x = 0.0;
    g_formation.neighbor_offset_y = 0.0;
    g_formation.neighbor_offset_valid = false;
  }
#endif // SINGLE_ROBOT_MODE

  printf("[FORMATION] TRANSPORT OFFSET LOCKED:\n");
  printf("  Robot pos: (%.3f, %.3f)\n", my_x, my_y);
  printf("  Robot theta: %.3f rad (%.1f deg) - LOCKED!\n",
         g_formation.transport_locked_theta,
         g_formation.transport_locked_theta * 180.0 / M_PI);
  printf("  Centroid:  (%.3f, %.3f)\n", centroid_x, centroid_y);
  printf("  Offset:    (%.3f, %.3f)\n", g_formation.transport_offset_x,
         g_formation.transport_offset_y);

  pthread_mutex_unlock(&g_formation.mutex);
}

void formation_set_centroid_target(double x, double y)
{
  pthread_mutex_lock(&g_formation.mutex);
  g_formation.centroid_target_x = x;
  g_formation.centroid_target_y = y;
  pthread_mutex_unlock(&g_formation.mutex);
}

bool formation_get_robot_target(double *x, double *y)
{
  pthread_mutex_lock(&g_formation.mutex);

  if (!g_formation.transport_active)
  {
    pthread_mutex_unlock(&g_formation.mutex);
    return false;
  }

  // Base target = centroid + my_offset
  double base_x =
      g_formation.centroid_target_x + g_formation.transport_offset_x;
  double base_y =
      g_formation.centroid_target_y + g_formation.transport_offset_y;

  // Robot 2 (Follower): Thêm correction từ Robot 1
#if ROBOT_ID == 2
  // Kiểm tra có neighbor data không
  double current_time = get_current_epoch_time();
  if (g_formation.has_neighbor_data &&
      (current_time - g_formation.neighbor_ts) < 1.0)
  {
    // Expected neighbor position = centroid + neighbor_offset
    double expected_neighbor_x =
        g_formation.centroid_target_x + g_formation.neighbor_offset_x;
    double expected_neighbor_y =
        g_formation.centroid_target_y + g_formation.neighbor_offset_y;

    // Actual neighbor position (từ sync_position)
    double actual_neighbor_x = g_formation.neighbor_x;
    double actual_neighbor_y = g_formation.neighbor_y;

    // Correction = actual - expected
    double correction_x = actual_neighbor_x - expected_neighbor_x;
    double correction_y = actual_neighbor_y - expected_neighbor_y;

    // Giới hạn correction (tránh outlier gây nhảy)
    const double max_corr = 0.1; // 10cm max correction
    double corr_mag =
        sqrt(correction_x * correction_x + correction_y * correction_y);
    if (corr_mag > max_corr)
    {
      double scale = max_corr / corr_mag;
      correction_x *= scale;
      correction_y *= scale;
    }

    // Áp dụng correction
    base_x += correction_x;
    base_y += correction_y;

    // Debug log (mỗi 1 giây)
    static int debug_cnt = 0;
    if (++debug_cnt >= 20)
    {
      printf("[FORMATION] VS Correction: expected_n(%.3f,%.3f) "
             "actual_n(%.3f,%.3f) corr(%.3f,%.3f)\n",
             expected_neighbor_x, expected_neighbor_y, actual_neighbor_x,
             actual_neighbor_y, correction_x, correction_y);
      debug_cnt = 0;
    }
  }
#endif

  *x = base_x;
  *y = base_y;

  pthread_mutex_unlock(&g_formation.mutex);
  return true;
}

bool formation_get_transport_offset(double *offset_x, double *offset_y)
{
  pthread_mutex_lock(&g_formation.mutex);
  if (!g_formation.transport_active)
  {
    pthread_mutex_unlock(&g_formation.mutex);
    *offset_x = 0.0;
    *offset_y = 0.0;
    return false;
  }
  *offset_x = g_formation.transport_offset_x;
  *offset_y = g_formation.transport_offset_y;
  pthread_mutex_unlock(&g_formation.mutex);
  return true;
}

bool formation_is_transport_active(void)
{
  pthread_mutex_lock(&g_formation.mutex);
  bool active = g_formation.transport_active;
  pthread_mutex_unlock(&g_formation.mutex);
  return active;
}

void formation_end_transport(void)
{
  pthread_mutex_lock(&g_formation.mutex);
  g_formation.transport_active = false;
  g_formation.neighbor_offset_valid = false;
  g_formation.is_locked = false;
  printf("[FORMATION] Transport mode ENDED\n");
  pthread_mutex_unlock(&g_formation.mutex);
}

bool formation_get_locked_theta(double *locked_theta)
{
  pthread_mutex_lock(&g_formation.mutex);

  if (!g_formation.transport_active)
  {
    pthread_mutex_unlock(&g_formation.mutex);
    *locked_theta = 0.0;
    return false;
  }

  *locked_theta = g_formation.transport_locked_theta;
  pthread_mutex_unlock(&g_formation.mutex);
  return true;
}

bool formation_get_follower_error(double *error_distance)
{
  // Virtual Structure mode: Calculate follower error based on centroid + offset
  // This is used by Robot1 (leader) to check if Robot2 (follower) is keeping up

  *error_distance = 0.0;

#if ROBOT_ID == 1
  pthread_mutex_lock(&g_formation.mutex);

  // Only valid in transport mode
  if (!g_formation.transport_active)
  {
    pthread_mutex_unlock(&g_formation.mutex);
    return false;
  }

  // Check if we have valid neighbor (follower) data
  if (!g_formation.has_neighbor_data)
  {
    pthread_mutex_unlock(&g_formation.mutex);
    return false;
  }

  // Check if neighbor data is too old (timeout)
  double current_time = get_current_epoch_time();
  if (current_time - g_formation.neighbor_ts > FOLLOWER_DATA_TIMEOUT)
  {
    pthread_mutex_unlock(&g_formation.mutex);
    return false;
  }

  // Calculate expected follower position based on ACTUAL robot1 position
  // NOT based on lookahead target (which is in the future)
  //
  // Logic:
  //   actual_centroid = my_position - my_offset (where object actually is)
  //   expected_follower = actual_centroid + follower_offset
  //
  // This ensures we're comparing follower's actual position to where it
  // SHOULD be based on current object position, not future trajectory point

  pthread_mutex_lock(&g_ekf_mutex);
  double my_x = g_ekf.x[0];
  double my_y = g_ekf.x[1];
  double my_vx = g_ekf.x[2];
  double my_vy = g_ekf.x[3];
  pthread_mutex_unlock(&g_ekf_mutex);

  // Calculate actual centroid (object) position from Robot1's current position
  double actual_centroid_x = my_x - g_formation.transport_offset_x;
  double actual_centroid_y = my_y - g_formation.transport_offset_y;

  // Expected follower = actual_centroid + follower_offset
  double expected_follower_x = actual_centroid_x + g_formation.neighbor_offset_x;
  double expected_follower_y = actual_centroid_y + g_formation.neighbor_offset_y;

  // Actual follower position and velocity (from sync_position)
  double actual_follower_x = g_formation.neighbor_x;
  double actual_follower_y = g_formation.neighbor_y;
  double follower_vx = g_formation.neighbor_vx;
  double follower_vy = g_formation.neighbor_vy;

  // Calculate CURRENT position error
  double dx = actual_follower_x - expected_follower_x;
  double dy = actual_follower_y - expected_follower_y;
  double current_error = sqrt(dx * dx + dy * dy);

  // === VELOCITY ANALYSIS ===
  // Check if follower is moving towards expected position (correction)
  double relative_vx = follower_vx - my_vx; // Relative velocity
  double relative_vy = follower_vy - my_vy;

  // Dot product: negative means follower approaching expected position
  double velocity_dot = dx * relative_vx + dy * relative_vy;
  bool follower_correcting = (velocity_dot < 0.0);

  // === PREDICTIVE ERROR ===
  // Predict error after 0.2s (typical reaction time)
  const double PREDICTION_TIME = 0.1;
  double predicted_dx = dx + relative_vx * PREDICTION_TIME;
  double predicted_dy = dy + relative_vy * PREDICTION_TIME;
  double predicted_error = sqrt(predicted_dx * predicted_dx + predicted_dy * predicted_dy);

  // === SMART ERROR CALCULATION ===
  // If follower is correcting actively, use predicted error (less conservative)
  // If follower is not correcting, use current error (more conservative)
  if (follower_correcting && predicted_error < current_error)
  {
    *error_distance = predicted_error;
  }
  else
  {
    *error_distance = current_error;
  }

  // Debug log every 1 second
  static int debug_cnt = 0;
  if (++debug_cnt >= 20) // 20Hz
  {
    printf("[FORMATION] Error: cur=%.1fcm pred=%.1fcm correcting=%s vel_dot=%.3f\n",
           current_error * 100.0, predicted_error * 100.0,
           follower_correcting ? "YES" : "NO", velocity_dot);
    debug_cnt = 0;
  }

  pthread_mutex_unlock(&g_formation.mutex);
  return true;

#else
  // Robot2 (follower) không cần kiểm tra follower error
  (void)error_distance;
  return false;
#endif
}

// =========== COMMON ===========

bool formation_is_locked(void)
{
  pthread_mutex_lock(&g_formation.mutex);
  bool is_locked = g_formation.is_locked;
  pthread_mutex_unlock(&g_formation.mutex);
  return is_locked;
}

void formation_reset(void)
{
  pthread_mutex_lock(&g_formation.mutex);
  g_formation.is_locked = false;
  g_formation.has_neighbor_data = false;
  g_formation.transport_active = false;
  g_formation.neighbor_offset_valid = false;
  printf("[FORMATION] Reset - will recalculate offset on next neighbor data\n");
  pthread_mutex_unlock(&g_formation.mutex);
}

void formation_set_follow_enabled(bool enable)
{
  pthread_mutex_lock(&g_formation.mutex);
  g_formation.follow_enabled = enable;
  printf("[FORMATION] Follow mode %s\n", enable ? "ENABLED" : "DISABLED");
  pthread_mutex_unlock(&g_formation.mutex);
}

bool formation_is_follow_enabled(void)
{
  pthread_mutex_lock(&g_formation.mutex);
  bool enabled = g_formation.follow_enabled;
  pthread_mutex_unlock(&g_formation.mutex);
  return enabled;
}

void formation_dispatch_user_control(void)
{
  pthread_mutex_lock(&g_formation.mutex);
  g_formation.last_control_ts = get_current_epoch_time();
  if (g_formation.follow_enabled)
  {
    g_formation.follow_enabled = false;
    printf("[FORMATION] User control detected -> Follow DISABLED\n");
  }
  pthread_mutex_unlock(&g_formation.mutex);
}

void formation_cleanup(void)
{
  // Stop thread first
  g_formation.thread_running = false;
  pthread_join(g_formation.follow_thread, NULL);

  pthread_mutex_lock(&g_formation.mutex);
  g_formation.is_locked = false;
  g_formation.follow_enabled = false;
  g_formation.has_neighbor_data = false;
  g_formation.transport_active = false;
  pthread_mutex_unlock(&g_formation.mutex);
  printf("[FORMATION] Cleanup done\n");
}

// =========== TRANSPORT LEADER-FOLLOWER MODE ===========

/**
 * Get follow velocity for Robot 2 (Follower) during transport phase.
 *
 * Robot 2 follows Robot 1's position + maintains transport offset.
 * This is used when TRANSPORT_MODE_LEADER_FOLLOWER = 1.
 *
 * Target position for Robot 2 = Robot 1 position + transport_offset
 * (transport_offset was locked when grip completed)
 *
 * @param vx: Output velocity X
 * @param vy: Output velocity Y
 * @return true if valid velocity computed
 */
bool formation_get_transport_follow_velocity(double *vx, double *vy)
{
  pthread_mutex_lock(&g_formation.mutex);

  // Check transport mode is active
  if (!g_formation.transport_active)
  {
    pthread_mutex_unlock(&g_formation.mutex);
    *vx = 0;
    *vy = 0;
    return false;
  }

  // Check neighbor data freshness (timeout 1 second)
  double current_time = get_current_epoch_time();
  if (!g_formation.has_neighbor_data ||
      (current_time - g_formation.neighbor_ts) > 1.0)
  {
    pthread_mutex_unlock(&g_formation.mutex);
    *vx = 0;
    *vy = 0;
    printf("[TRANSPORT-FOLLOW] Timeout - no neighbor data\n");
    return false;
  }

  // Check if neighbor_offset is valid (calculated from fresh data)
  if (!g_formation.neighbor_offset_valid)
  {
    pthread_mutex_unlock(&g_formation.mutex);
    *vx = 0;
    *vy = 0;
    printf("[TRANSPORT-FOLLOW] Waiting for valid neighbor_offset...\n");
    return false;
  }

  // Get my current position and velocity
  pthread_mutex_lock(&g_ekf_mutex);
  double my_x = g_ekf.x[0];
  double my_y = g_ekf.x[1];
  double my_vx = g_ekf.x[2];
  double my_vy = g_ekf.x[3];
  pthread_mutex_unlock(&g_ekf_mutex);

  // Calculate relative offset (Robot2 offset from Robot1)
  // relative_offset = my_transport_offset - neighbor_transport_offset
  //                 = (my_pos - centroid) - (neighbor_pos - centroid)
  //                 = my_pos - neighbor_pos
  // This ensures Robot2 maintains the same relative position to Robot1
  double relative_offset_x =
      g_formation.transport_offset_x - g_formation.neighbor_offset_x;
  double relative_offset_y =
      g_formation.transport_offset_y - g_formation.neighbor_offset_y;

  // Target = current neighbor position + relative offset
  // When Robot1 moves, Robot2's target moves by the same amount
  double target_x = g_formation.neighbor_x + relative_offset_x;
  double target_y = g_formation.neighbor_y + relative_offset_y;

  // Calculate position error
  double err_x = target_x - my_x;
  double err_y = target_y - my_y;
  double err_dist = sqrt(err_x * err_x + err_y * err_y);

  // ============ ADAPTIVE PD: Error Derivative Based ============
  // Ý tưởng: Nếu error đang GIẢM → robot đang sửa đúng → giảm Kp để không vọt lố
  //          Nếu error đang TĂNG → robot đang lạc → giữ Kp cao để phản ứng nhanh

  static double last_err_dist = 0.0;
  static double last_time = 0.0;

  double dt = current_time - last_time;
  if (dt < 0.01)
    dt = 0.05; // Default 20Hz nếu lần đầu

  // Đạo hàm error: d_err < 0 nghĩa là error đang giảm
  double d_err = (err_dist - last_err_dist) / dt;

  last_err_dist = err_dist;
  last_time = current_time;

  // Base gains
  const double BASE_KP = 1.8;
  const double BASE_KD = 0.5;

  // Adaptive factor dựa trên đạo hàm error
  // d_err < 0 (đang sửa) → giảm Kp (factor < 1)
  // d_err > 0 (đang lạc) → giữ nguyên Kp (factor = 1)
  double adaptive_factor = 1.0;
  if (d_err < 0)
  {
    // Error đang giảm → giảm Kp tỷ lệ với tốc độ giảm
    // d_err = -0.1 m/s → factor = 0.5 (giảm 50%)
    // d_err = -0.2 m/s → factor = 0.0 (gần như dừng correction)
    adaptive_factor = 1.0 + d_err * 5.0; // d_err âm nên factor < 1
    if (adaptive_factor < 0.1)
      adaptive_factor = 0.1; // Min 10%
  }

  double Kp = BASE_KP * adaptive_factor;
  double Kd = BASE_KD; // Kd giữ nguyên để damping ổn định

  // P term: proportional to position error
  double p_vx = Kp * err_x;
  double p_vy = Kp * err_y;

  // D term: velocity matching (feedforward damping)
  double velocity_err_x = g_formation.neighbor_vx - my_vx;
  double velocity_err_y = g_formation.neighbor_vy - my_vy;
  double d_vx = Kd * velocity_err_x;
  double d_vy = Kd * velocity_err_y;

  // Total correction = P + D
  double correction_vx = p_vx + d_vx;
  double correction_vy = p_vy + d_vy;

  // ============ RATE LIMITING (không phải LPF!) ============
  // Giới hạn tốc độ THAY ĐỔI của correction, không gây delay
  // Nếu correction thay đổi quá nhanh → clamp delta, không clamp value
  static double last_correction_vx = 0.0;
  static double last_correction_vy = 0.0;

  const double MAX_CORRECTION_RATE = 0.5; // m/s per second (acceleration limit)
  double max_delta = MAX_CORRECTION_RATE * dt;

  double delta_vx = correction_vx - last_correction_vx;
  double delta_vy = correction_vy - last_correction_vy;

  // Clamp delta (rate limit)
  if (delta_vx > max_delta)
    delta_vx = max_delta;
  if (delta_vx < -max_delta)
    delta_vx = -max_delta;
  if (delta_vy > max_delta)
    delta_vy = max_delta;
  if (delta_vy < -max_delta)
    delta_vy = -max_delta;

  correction_vx = last_correction_vx + delta_vx;
  correction_vy = last_correction_vy + delta_vy;

  last_correction_vx = correction_vx;
  last_correction_vy = correction_vy;

  // ============ RATE LIMIT FEEDFORWARD ============
  // Feedforward (neighbor velocity) cũng cần rate limit vì:
  // - Velocity của Robot1 có noise từ EKF
  // - Noise này truyền trực tiếp sang Robot2 gây giật
  // Rate limit KHÔNG gây delay, chỉ giới hạn acceleration
  static double smooth_ff_vx = 0.0;
  static double smooth_ff_vy = 0.0;

  const double MAX_FF_RATE = 0.8; // m/s² - feedforward có thể thay đổi nhanh hơn correction
  double max_ff_delta = MAX_FF_RATE * dt;

  double ff_delta_vx = g_formation.neighbor_vx - smooth_ff_vx;
  double ff_delta_vy = g_formation.neighbor_vy - smooth_ff_vy;

  if (ff_delta_vx > max_ff_delta)
    ff_delta_vx = max_ff_delta;
  if (ff_delta_vx < -max_ff_delta)
    ff_delta_vx = -max_ff_delta;
  if (ff_delta_vy > max_ff_delta)
    ff_delta_vy = max_ff_delta;
  if (ff_delta_vy < -max_ff_delta)
    ff_delta_vy = -max_ff_delta;

  smooth_ff_vx += ff_delta_vx;
  smooth_ff_vy += ff_delta_vy;

  // Final velocity = Rate-limited feedforward + Rate-limited correction
  *vx = smooth_ff_vx + correction_vx;
  *vy = smooth_ff_vy + correction_vy;

  // Limit total velocity (safety)
  double total_mag = sqrt((*vx) * (*vx) + (*vy) * (*vy));
  const double max_vel = TRANSPORT_VELOCITY * 1.5;
  if (total_mag > max_vel)
  {
    double scale = max_vel / total_mag;
    *vx *= scale;
    *vy *= scale;
  }

  // Debug log every 1 second
  static int debug_cnt = 0;
  if (++debug_cnt >= 20)
  {
    printf("[TRANSPORT-FOLLOW] Err=%.1fcm d_err=%.2f Kp=%.2f Corr(%.3f,%.3f) Vel(%.3f,%.3f)\n",
           err_dist * 100.0, d_err, Kp, correction_vx, correction_vy, *vx, *vy);
    debug_cnt = 0;
  }

  pthread_mutex_unlock(&g_formation.mutex);
  return true;
}

// =========== END OF FORMATION MANAGER ===========
