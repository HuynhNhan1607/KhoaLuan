#include "formation_manager.h"
#include "client_manager.h"
#include "ekf.h"
#include "sys_config.h"
#include "trajectory_executor.h"
#include <math.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

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

    // =========== VIRTUAL STRUCTURE MODE (NEW) ===========
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
                                        .transport_active = false,
                                        .transport_offset_x = 0.0,
                                        .transport_offset_y = 0.0,
                                        .centroid_target_x = 0.0,
                                        .centroid_target_y = 0.0,
                                        .neighbor_offset_x = 0.0,
                                        .neighbor_offset_y = 0.0,
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
#if TRANSPORT_MODE_LEADER_FOLLOWER && ROBOT_ID == 2
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

        // Chỉ gửi vận tốc follow khi KHÔNG có trajectory đang chạy
        // VÀ KHÔNG ở transport mode
        if (!trajectory_is_running() && !g_formation.transport_active)
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
                               double theta, double ts)
{
    double receive_time = get_current_epoch_time();
    double delay_s = receive_time - ts; // Delay in seconds
    double delay_ms = delay_s * 1000.0; // Delay in milliseconds

    // **DELAY COMPENSATION**: Extrapolate leader position to current time
    double predicted_x = x + vx * delay_s;
    double predicted_y = y + vy * delay_s;

    // Log delay (mỗi 1 giây để không spam)
    static int log_cnt = 0;
    if (++log_cnt >= 10)
    { // 10Hz -> log mỗi 1 giây
        printf("[FORMATION] Recv sync_position: delay=%.1fms vx=%.3f vy=%.3f | "
               "Compensated: (%.3f,%.3f)->(%.3f,%.3f)\n",
               delay_ms, vx, vy, x, y, predicted_x, predicted_y);
        log_cnt = 0;
    }

    pthread_mutex_lock(&g_formation.mutex);

    // Cập nhật dữ liệu neighbor với vị trí đã compensate
    g_formation.neighbor_x = predicted_x;
    g_formation.neighbor_y = predicted_y;
    g_formation.neighbor_vx = vx;
    g_formation.neighbor_vy = vy;
    g_formation.neighbor_theta = theta;
    g_formation.neighbor_ts = receive_time; // Dùng thời gian LOCAL khi nhận
    g_formation.has_neighbor_data = true;

    // KHÔNG TỰ ĐỘNG LOCK OFFSET ở đây nữa!
    // Offset sẽ được lock khi:
    // 1. execute_grip thành công (arm_execute_grip)
    // 2. lock_transport_offset command từ server
    // Điều này tránh robot tự động follow khi vừa bật lên

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

    // Debug log mỗi 1 giây
    static int debug_cnt = 0;
    if (++debug_cnt >= 20)
    {
        printf("[FORMATION] Target(%.3f,%.3f) Current(%.3f,%.3f) Err(%.3f,%.3f) "
               "Corr(%.3f,%.3f)\n",
               target_x, target_y, my_x, my_y, err_x, err_y, correction_vx,
               correction_vy);
        debug_cnt = 0;
    }

    pthread_mutex_unlock(&g_formation.mutex);
    return true;
}

// =========== VIRTUAL STRUCTURE MODE ===========

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

    // Also lock initial_offset for leader-follower mode
    // initial_offset = my_pos - neighbor_pos
    if (g_formation.has_neighbor_data)
    {
        g_formation.initial_offset_x = my_x - g_formation.neighbor_x;
        g_formation.initial_offset_y = my_y - g_formation.neighbor_y;
        g_formation.is_locked = true;

        // Tính neighbor offset từ centroid
        g_formation.neighbor_offset_x = g_formation.neighbor_x - centroid_x;
        g_formation.neighbor_offset_y = g_formation.neighbor_y - centroid_y;
        printf("  Neighbor offset: (%.3f, %.3f)\n", g_formation.neighbor_offset_x,
               g_formation.neighbor_offset_y);
        printf("  Initial offset (for L-F): (%.3f, %.3f)\n",
               g_formation.initial_offset_x, g_formation.initial_offset_y);
    }
    else
    {
        // WARNING: No neighbor data yet - will need to calculate offset later
        printf("  WARNING: No neighbor data available! neighbor_offset will be calculated when first sync received.\n");
        g_formation.neighbor_offset_x = 0.0;
        g_formation.neighbor_offset_y = 0.0;
    }

    // Store centroid position for later neighbor_offset calculation if needed
    g_formation.centroid_target_x = centroid_x;
    g_formation.centroid_target_y = centroid_y;

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

bool formation_is_transport_active(void)
{
    pthread_mutex_lock(&g_formation.mutex);
    bool active = g_formation.transport_active;
    pthread_mutex_unlock(&g_formation.mutex);
    return active;
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

void formation_end_transport(void)
{
    pthread_mutex_lock(&g_formation.mutex);
    g_formation.transport_active = false;
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
    // This function is meant to be called by Robot1 (leader) to check
    // how far Robot2 (follower) has deviated from its expected position

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
        // Rate-limit this log to avoid spam (every 2 seconds)
        static int no_data_log_cnt = 0;
        if (++no_data_log_cnt >= 40)
        { // 20Hz * 2s = 40
            printf("[FORMATION] No follower data available (check laptop forwarding)\n");
            no_data_log_cnt = 0;
        }
        return false;
    }

    // Check if neighbor data is too old (timeout)
    double current_time = get_current_epoch_time();
    if (current_time - g_formation.neighbor_ts > FOLLOWER_DATA_TIMEOUT)
    {
        pthread_mutex_unlock(&g_formation.mutex);
        // Rate-limit this log
        static int timeout_log_cnt = 0;
        if (++timeout_log_cnt >= 40)
        {
            printf("[FORMATION] Follower data timeout (%.1fs old)\n",
                   current_time - g_formation.neighbor_ts);
            timeout_log_cnt = 0;
        }
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
    pthread_mutex_unlock(&g_ekf_mutex);

    // Calculate actual centroid (object) position from Robot1's current position
    double actual_centroid_x = my_x - g_formation.transport_offset_x;
    double actual_centroid_y = my_y - g_formation.transport_offset_y;

    // Expected follower = actual_centroid + follower_offset
    double expected_follower_x = actual_centroid_x + g_formation.neighbor_offset_x;
    double expected_follower_y = actual_centroid_y + g_formation.neighbor_offset_y;

    // Actual follower position (from sync_position)
    double actual_follower_x = g_formation.neighbor_x;
    double actual_follower_y = g_formation.neighbor_y;

    // Calculate error distance
    double dx = actual_follower_x - expected_follower_x;
    double dy = actual_follower_y - expected_follower_y;
    *error_distance = sqrt(dx * dx + dy * dy);

    // Debug log (every 1 second)
    static int debug_cnt = 0;
    if (++debug_cnt >= 20)
    {
        printf("[FORMATION] Follower check: centroid(%.3f,%.3f) expected(%.3f,%.3f) actual(%.3f,%.3f) error=%.3fm\n",
               actual_centroid_x, actual_centroid_y,
               expected_follower_x, expected_follower_y,
               actual_follower_x, actual_follower_y, *error_distance);
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

    // Get my current position and velocity
    pthread_mutex_lock(&g_ekf_mutex);
    double my_x = g_ekf.x[0];
    double my_y = g_ekf.x[1];
    double my_vx = g_ekf.x[2];
    double my_vy = g_ekf.x[3];
    // Get position uncertainty (P[0][0] = var_x, P[1][1] = var_y)
    double pos_uncertainty = sqrt(g_ekf.P[0][0] + g_ekf.P[1][1]);
    pthread_mutex_unlock(&g_ekf_mutex);

    // Calculate relative offset (Robot2 offset from Robot1)
    double relative_offset_x =
        g_formation.transport_offset_x - g_formation.neighbor_offset_x;
    double relative_offset_y =
        g_formation.transport_offset_y - g_formation.neighbor_offset_y;

    // Target = current neighbor position + relative offset (NO LOOKAHEAD!)
    // Lookahead was causing overshoot when leader decelerates
    double target_x = g_formation.neighbor_x + relative_offset_x;
    double target_y = g_formation.neighbor_y + relative_offset_y;

    // Calculate position error
    double err_x = target_x - my_x;
    double err_y = target_y - my_y;
    double err_dist = sqrt(err_x * err_x + err_y * err_y);

    // **CONSERVATIVE PD Control with Uncertainty Scaling**
    // When uncertainty is high, reduce gain to avoid aggressive corrections
    const double BASE_KP = 1.8; // Reduced from 2.5 - more conservative
    const double BASE_KD = 0.5; // Increased damping - smoother

    // Scale down gains when position uncertainty is high
    // uncertainty > 0.2m -> reduce gains by 50%
    double uncertainty_factor = 1.0;
    if (pos_uncertainty > 0.2)
    {
        uncertainty_factor = 0.5;
    }
    else if (pos_uncertainty > 0.1)
    {
        uncertainty_factor = 1.0 - (pos_uncertainty - 0.1) * 5.0; // linear 1.0->0.5
    }

    double Kp = BASE_KP * uncertainty_factor;
    double Kd = BASE_KD * uncertainty_factor;

    // P term: proportional to position error
    double p_vx = Kp * err_x;
    double p_vy = Kp * err_y;

    // D term: velocity matching + damping
    double velocity_err_x = g_formation.neighbor_vx - my_vx;
    double velocity_err_y = g_formation.neighbor_vy - my_vy;
    double d_vx = Kd * velocity_err_x;
    double d_vy = Kd * velocity_err_y;

    // Total correction = P + D
    double correction_vx = p_vx + d_vx;
    double correction_vy = p_vy + d_vy;

    // **DEADBAND**: Ignore tiny errors to reduce jitter
    const double ERROR_DEADBAND = 0.02; // 2cm
    if (err_dist < ERROR_DEADBAND)
    {
        correction_vx *= 0.2; // Minimal correction
        correction_vy *= 0.2;
    }

    // Limit correction magnitude (conservative)
    const double max_correction = TRANSPORT_VELOCITY * 1.5; // Reduced from 3.0
    double correction_mag =
        sqrt(correction_vx * correction_vx + correction_vy * correction_vy);
    if (correction_mag > max_correction)
    {
        double scale = max_correction / correction_mag;
        correction_vx *= scale;
        correction_vy *= scale;
    }

    // Feedforward: match neighbor's velocity
    *vx = g_formation.neighbor_vx + correction_vx;
    *vy = g_formation.neighbor_vy + correction_vy;

    // Limit total velocity (safety)
    double total_mag = sqrt((*vx) * (*vx) + (*vy) * (*vy));
    const double max_vel = TRANSPORT_VELOCITY * 1.2; // Reduced from 2.0
    if (total_mag > max_vel)
    {
        double scale = max_vel / total_mag;
        *vx *= scale;
        *vy *= scale;
    }

    // **VELOCITY SMOOTHING**: Low-pass filter to reduce jerk
    static double smooth_vx = 0.0;
    static double smooth_vy = 0.0;
    const double ALPHA = 0.7; // 0=no smoothing, 1=instant
    smooth_vx = ALPHA * (*vx) + (1.0 - ALPHA) * smooth_vx;
    smooth_vy = ALPHA * (*vy) + (1.0 - ALPHA) * smooth_vy;
    *vx = smooth_vx;
    *vy = smooth_vy;

    // Debug log every 1 second
    static int debug_cnt = 0;
    if (++debug_cnt >= 20)
    {
        printf("[TRANSPORT-FOLLOW] Target(%.3f,%.3f) Current(%.3f,%.3f) "
               "Err=%.1fcm Unc=%.1fcm Kp=%.2f Vel(%.3f,%.3f)\n",
               target_x, target_y, my_x, my_y,
               err_dist * 100.0, pos_uncertainty * 100.0, Kp, *vx, *vy);
        debug_cnt = 0;
    }

    pthread_mutex_unlock(&g_formation.mutex);
    return true;
}
