#include "trajectory_executor.h"
#include "camera_docking.h"    // Shared docking constants (port, marker id, ...)
#include "cJSON.h"
#include "client_manager.h"
#include "ekf.h"               // Include EKF header
#include "formation_manager.h" // For cooperative transport
#include "localize.h"          // For current pose if needed directly, though we might inject it
#include "socket.h"            // For send_to_laptop_clients
#include "sys_config.h"        // For ENABLE_THETA_TRACKING
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
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

static long get_time_ms(void);

typedef enum
{
  DOCK_STATE_IDLE = 0,
  DOCK_STATE_SEARCH_RIGHT,
  DOCK_STATE_SEARCH_LEFT,
  DOCK_STATE_ALIGN_X,
  DOCK_STATE_APPROACH_Y
} DockState;

typedef struct
{
  bool active;
  bool test_mode;
  DockState state;
  long docking_start_ms;
  long state_start_ms;
  long last_vision_ms;
  bool vision_found;
  float vision_x;
  float vision_z;
  char vision_hint[8]; /* "LEFT", "RIGHT", "CENTER", "NONE" — từ color detection */
  int vision_sock;
  char vision_buf[1024];
  int vision_buf_len;
  long last_status_log_ms;
  long last_connect_try_ms;
} DockingCtx;

static DockingCtx g_dock = {
    .active = false,
    .test_mode = false,
    .state = DOCK_STATE_IDLE,
    .docking_start_ms = 0,
    .state_start_ms = 0,
    .last_vision_ms = 0,
    .vision_found = false,
    .vision_x = 0.0f,
    .vision_z = 0.0f,
    .vision_hint = "NONE",
    .vision_sock = -1,
    .vision_buf_len = 0,
    .last_status_log_ms = 0,
    .last_connect_try_ms = 0};
static pthread_mutex_t g_dock_mutex = PTHREAD_MUTEX_INITIALIZER;

static const char *dock_state_name(DockState s)
{
  switch (s)
  {
  case DOCK_STATE_IDLE:
    return "IDLE";
  case DOCK_STATE_SEARCH_RIGHT:
    return "SEARCH_RIGHT";
  case DOCK_STATE_SEARCH_LEFT:
    return "SEARCH_LEFT";
  case DOCK_STATE_ALIGN_X:
    return "ALIGN_X";
  case DOCK_STATE_APPROACH_Y:
    return "APPROACH_Y";
  default:
    return "UNKNOWN";
  }
}

static int set_nonblock_fd(int fd)
{
  int flags = fcntl(fd, F_GETFL, 0);
  if (flags < 0)
    return -1;
  return fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

static void send_motor_velocity(float vx, float vy, float vtheta)
{
  char cmd[128];
  snprintf(cmd, sizeof(cmd), "dot_x:%.4f dot_y:%.4f dot_theta:%.4f\n", vx, vy,
           vtheta);
  client_manager_broadcast_to_motor(cmd, strlen(cmd));
  g_last_cmd_vx = vx;
  g_last_cmd_vy = vy;
}

static void docking_close_socket(void)
{
  if (g_dock.vision_sock >= 0)
  {
    printf("[DOCK] Vision socket closed\n");
    close(g_dock.vision_sock);
    g_dock.vision_sock = -1;
  }
  g_dock.vision_buf_len = 0;
}

static void docking_reset(bool stop_motion)
{
  if (stop_motion)
  {
    send_motor_velocity(0.0f, 0.0f, 0.0f);
  }
  g_dock.active = false;
  g_dock.test_mode = false;
  g_dock.state = DOCK_STATE_IDLE;
  g_dock.docking_start_ms = 0;
  g_dock.state_start_ms = 0;
  g_dock.last_vision_ms = 0;
  g_dock.vision_found = false;
  g_dock.vision_x = 0.0f;
  g_dock.vision_z = 0.0f;
  strncpy(g_dock.vision_hint, "NONE", sizeof(g_dock.vision_hint));
  g_dock.last_status_log_ms = 0;
  g_dock.last_connect_try_ms = 0;
  docking_close_socket();
}

static void docking_set_state(DockState new_state)
{
  if (g_dock.state == new_state)
  {
    return;
  }
  long now = get_time_ms();
  printf("[DOCK] STATE %s -> %s (state_elapsed=%ldms total_elapsed=%ldms)\n",
         dock_state_name(g_dock.state), dock_state_name(new_state),
         now - g_dock.state_start_ms, now - g_dock.docking_start_ms);
  g_dock.state = new_state;
  g_dock.state_start_ms = now;
}

static bool docking_connect_vision_if_needed(void)
{
  if (g_dock.vision_sock >= 0)
  {
    return true;
  }

  long now = get_time_ms();
  if (g_dock.last_connect_try_ms != 0 &&
      (now - g_dock.last_connect_try_ms) < 1000)
  {
    return false;
  }
  g_dock.last_connect_try_ms = now;

  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0)
  {
    perror("[DOCK] socket vision");
    return false;
  }
  if (set_nonblock_fd(sock) < 0)
  {
    perror("[DOCK] set_nonblock vision");
    close(sock);
    return false;
  }

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(CAMERA_DOCKING_PORT);
  inet_pton(AF_INET, CAMERA_DOCKING_HOST, &addr.sin_addr);

  int rc = connect(sock, (struct sockaddr *)&addr, sizeof(addr));
  if (rc < 0 && errno != EINPROGRESS)
  {
    printf("[DOCK] Vision connect failed: %s\n", strerror(errno));
    close(sock);
    return false;
  }

  g_dock.vision_sock = sock;
  g_dock.vision_buf_len = 0;
  printf("[DOCK] Connected to camera service 127.0.0.1:9091\n");
  return true;
}

static bool parse_vision_line(const char *line, bool *found, float *x, float *y, float *z, char *hint)
{
  int found_i = 0;
  float x_f = 0.0f;
  float y_f = 0.0f;
  float z_f = 0.0f;
  char hint_s[8] = "NONE";
  /* Format mới: {"type":"docking_vision","found":N,"x":F,"y":F,"z":F,"hint":".."} */
  int matched = sscanf(line,
                       "{\"type\":\"docking_vision\",\"found\":%d,"
                       "\"x\":%f,\"y\":%f,\"z\":%f,"
                       "\"hint\":\"%7[^\"]\"}",
                       &found_i, &x_f, &y_f, &z_f, hint_s);
  if (matched >= 4)
  {
    *found = (found_i != 0);
    *x = x_f;
    *y = y_f;
    *z = z_f;
    if (matched == 5) strncpy(hint, hint_s, 8);
    else              strncpy(hint, "NONE", 8);
    hint[7] = '\0';
    return true;
  }
  /* Fallback: format cũ không có y, không có hint */
  matched = sscanf(line,
                   "{\"type\":\"docking_vision\",\"found\":%d,\"x\":%f,\"z\":%f}",
                   &found_i, &x_f, &z_f);
  if (matched == 3)
  {
    *found = (found_i != 0);
    *x = x_f;
    *y = 0.0f;
    *z = z_f;
    strncpy(hint, "NONE", 8);
    return true;
  }
  return false;
}

static void docking_poll_vision(void)
{
  static bool last_found = false;
  static bool last_found_valid = false;

  if (g_dock.vision_sock < 0)
  {
    g_dock.vision_found = false;
    strncpy(g_dock.vision_hint, "NONE", sizeof(g_dock.vision_hint));
    return;
  }

  char recv_buf[256];
  while (1)
  {
    ssize_t n = recv(g_dock.vision_sock, recv_buf, sizeof(recv_buf) - 1, 0);
    if (n < 0)
    {
      if (errno == EAGAIN || errno == EWOULDBLOCK)
      {
        break;
      }
      printf("[DOCK] Vision recv error: %s\n", strerror(errno));
      docking_close_socket();
      g_dock.vision_found = false;
      strncpy(g_dock.vision_hint, "NONE", sizeof(g_dock.vision_hint));
      break;
    }
    if (n == 0)
    {
      printf("[DOCK] Vision peer disconnected\n");
      docking_close_socket();
      g_dock.vision_found = false;
      strncpy(g_dock.vision_hint, "NONE", sizeof(g_dock.vision_hint));
      break;
    }
    recv_buf[n] = '\0';
    for (ssize_t i = 0; i < n; i++)
    {
      char c = recv_buf[i];
      if (c == '\n')
      {
        g_dock.vision_buf[g_dock.vision_buf_len] = '\0';
        bool found = false;
        float x = 0.0f, y = 0.0f, z = 0.0f;
        char hint[8] = "NONE";
        if (parse_vision_line(g_dock.vision_buf, &found, &x, &y, &z, hint))
        {
          g_dock.last_vision_ms = get_time_ms();
          g_dock.vision_found = found;
          g_dock.vision_x = x;
          g_dock.vision_z = z;
          strncpy(g_dock.vision_hint, hint, sizeof(g_dock.vision_hint));
          g_dock.vision_hint[7] = '\0';
          if (!last_found_valid || last_found != found)
          {
            /* x>0: marker bên PHẢI camera → robot cần strafe RIGHT
             * x<0: marker bên TRÁI camera → robot cần strafe LEFT
             * ALIGN_X state sẽ xử lý strafe theo vision_x */
            const char *dir = (x < -0.02f) ? "LEFT" : (x > 0.02f) ? "RIGHT" : "CENTER";
            printf("[DOCK] Vision found=%d  dir=%-6s x=%+.4f y=%+.4f z=%.4f  hint=%s\n",
                   found ? 1 : 0, found ? dir : "-", x, y, z, hint);
            last_found = found;
            last_found_valid = true;
          }
        }
        else if (g_dock.vision_buf[0] != '\0')
        {
          printf("[DOCK] Vision parse skipped: %s\n", g_dock.vision_buf);
        }
        g_dock.vision_buf_len = 0;
      }
      else if (g_dock.vision_buf_len < (int)sizeof(g_dock.vision_buf) - 1)
      {
        g_dock.vision_buf[g_dock.vision_buf_len++] = c;
      }
      else
      {
        g_dock.vision_buf_len = 0;
      }
    }
  }
}

static void docking_begin(bool test_mode)
{
  g_dock.active = true;
  g_dock.test_mode = test_mode;
  g_dock.docking_start_ms = get_time_ms();
  g_dock.state_start_ms = g_dock.docking_start_ms;
  g_dock.last_vision_ms = 0;
  g_dock.vision_found = false;
  g_dock.vision_x = 0.0f;
  g_dock.vision_z = 0.0f;
  strncpy(g_dock.vision_hint, "NONE", sizeof(g_dock.vision_hint));
  g_dock.state = DOCK_STATE_SEARCH_RIGHT;
  g_dock.last_status_log_ms = 0;
  (void)docking_connect_vision_if_needed();
  printf("[DOCK] Start docking mode=%s scan_vx=%.3f align_vx=%.3f approach_vy=%.3f x_tol=%.3f z_target=%.3f z_tol=%.3f\n",
         test_mode ? "test" : "trajectory", DOCK_SCAN_VX, DOCK_ALIGN_VX,
         DOCK_APPROACH_VY, DOCK_X_TOL, DOCK_TARGET_Z, DOCK_Y_TOL);
}

typedef enum
{
  DOCK_STEP_IN_PROGRESS = 0,
  DOCK_STEP_DONE,
  DOCK_STEP_NOT_FOUND
} DockStepResult;

static DockStepResult docking_step_once(void)
{
  (void)docking_connect_vision_if_needed();
  docking_poll_vision();

  long now = get_time_ms();

  /* Fix: Stale vision check
   * Nếu camera service dừng gửi hoặc crash, vision_found giữ giá trị cuối
   * mãi mãi → robot tiếp tục chạy theo dữ liệu cũ. Cần hủy rõ ràng.
   */
  if (g_dock.last_vision_ms > 0 &&
      (now - g_dock.last_vision_ms) > VISION_STALE_MS)
  {
    if (g_dock.vision_found)
    {
      printf("[DOCK] Vision stale age=%ldms > %dms -> clear found\n",
             now - g_dock.last_vision_ms, VISION_STALE_MS);
    }
    g_dock.vision_found = false;
    strncpy(g_dock.vision_hint, "NONE", sizeof(g_dock.vision_hint));
  }
  if (g_dock.last_status_log_ms == 0 ||
      (now - g_dock.last_status_log_ms) >= 1000)
  {
    long vision_age =
        (g_dock.last_vision_ms > 0) ? (now - g_dock.last_vision_ms) : -1;
    printf("[DOCK] Tick state=%s found=%d x=%.4f z=%.4f vision_age=%ldms elapsed=%ldms\n",
           dock_state_name(g_dock.state), g_dock.vision_found ? 1 : 0,
           g_dock.vision_x, g_dock.vision_z, vision_age,
           now - g_dock.docking_start_ms);
    g_dock.last_status_log_ms = now;
  }

  if (now - g_dock.docking_start_ms > DOCK_TOTAL_TIMEOUT_MS)
  {
    printf("[DOCK] Timeout total=%dms state=%s x=%.4f z=%.4f found=%d\n",
           DOCK_TOTAL_TIMEOUT_MS, dock_state_name(g_dock.state), g_dock.vision_x,
           g_dock.vision_z, g_dock.vision_found ? 1 : 0);
    send_motor_velocity(0.0f, 0.0f, 0.0f);
    return DOCK_STEP_NOT_FOUND;
  }

  if (g_dock.vision_found)
  {
    if (fabsf(g_dock.vision_x) > DOCK_X_TOL)
    {
      docking_set_state(DOCK_STATE_ALIGN_X);
    }
    else
    {
      docking_set_state(DOCK_STATE_APPROACH_Y);
    }
  }

  switch (g_dock.state)
  {
  case DOCK_STATE_SEARCH_RIGHT:
    /* Nếu color hint chỉ rõ hướng → chuyển ngay, không cần đợi timeout */
    if (strncmp(g_dock.vision_hint, "LEFT", 4) == 0)
    {
      printf("[DOCK] Color hint=LEFT → switch to SEARCH_LEFT immediately\n");
      docking_set_state(DOCK_STATE_SEARCH_LEFT);
      return DOCK_STEP_IN_PROGRESS;
    }
    send_motor_velocity(+DOCK_SCAN_VX, 0.0f, 0.0f);
    if (now - g_dock.state_start_ms > DOCK_SCAN_TIMEOUT_MS)
    {
      printf("[DOCK] Search right timeout=%dms (target not found)\n",
             DOCK_SCAN_TIMEOUT_MS);
      docking_set_state(DOCK_STATE_SEARCH_LEFT);
    }
    return DOCK_STEP_IN_PROGRESS;

  case DOCK_STATE_SEARCH_LEFT:
    /* Nếu color hint chỉ rõ hướng → chuyển ngay */
    if (strncmp(g_dock.vision_hint, "RIGHT", 5) == 0)
    {
      printf("[DOCK] Color hint=RIGHT → switch to SEARCH_RIGHT immediately\n");
      docking_set_state(DOCK_STATE_SEARCH_RIGHT);
      return DOCK_STEP_IN_PROGRESS;
    }
    send_motor_velocity(-DOCK_SCAN_VX, 0.0f, 0.0f);
    if (now - g_dock.state_start_ms > DOCK_SCAN_TIMEOUT_MS)
    {
      printf("[DOCK] Search left timeout=%dms -> NOT_FOUND\n",
             DOCK_SCAN_TIMEOUT_MS);
      send_motor_velocity(0.0f, 0.0f, 0.0f);
      return DOCK_STEP_NOT_FOUND;
    }
    return DOCK_STEP_IN_PROGRESS;

  case DOCK_STATE_ALIGN_X:
    if (!g_dock.vision_found)
    {
      send_motor_velocity(0.0f, 0.0f, 0.0f);
      return DOCK_STEP_IN_PROGRESS;
    }
    if (fabsf(g_dock.vision_x) <= DOCK_X_TOL)
    {
      printf("[DOCK] X aligned x=%.4f tol=%.4f\n", g_dock.vision_x, DOCK_X_TOL);
      docking_set_state(DOCK_STATE_APPROACH_Y);
      send_motor_velocity(0.0f, 0.0f, 0.0f);
      return DOCK_STEP_IN_PROGRESS;
    }
    send_motor_velocity((g_dock.vision_x > 0.0f) ? +DOCK_ALIGN_VX : -DOCK_ALIGN_VX,
                        0.0f, 0.0f);
    return DOCK_STEP_IN_PROGRESS;

  case DOCK_STATE_APPROACH_Y:
    if (!g_dock.vision_found)
    {
      send_motor_velocity(0.0f, 0.0f, 0.0f);
      return DOCK_STEP_IN_PROGRESS;
    }
    if (fabsf(g_dock.vision_x) > DOCK_X_TOL)
    {
      docking_set_state(DOCK_STATE_ALIGN_X);
      return DOCK_STEP_IN_PROGRESS;
    }

    if (g_dock.vision_z > (DOCK_TARGET_Z + DOCK_Y_TOL))
    {
      send_motor_velocity(0.0f, +DOCK_APPROACH_VY, 0.0f);
      return DOCK_STEP_IN_PROGRESS;
    }
    if (g_dock.vision_z < (DOCK_TARGET_Z - DOCK_Y_TOL))
    {
      send_motor_velocity(0.0f, -DOCK_APPROACH_VY, 0.0f);
      return DOCK_STEP_IN_PROGRESS;
    }

    send_motor_velocity(0.0f, 0.0f, 0.0f);
    printf("[DOCK] Done x=%.4f z=%.4f (target_z=%.4f)\n", g_dock.vision_x,
           g_dock.vision_z, DOCK_TARGET_Z);
    return DOCK_STEP_DONE;

  case DOCK_STATE_IDLE:
  default:
    return DOCK_STEP_IN_PROGRESS;
  }
}

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

  pthread_mutex_lock(&g_dock_mutex);
  docking_reset(false);
  pthread_mutex_unlock(&g_dock_mutex);
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

bool trajectory_start_docking_test(void)
{
  pthread_mutex_lock(&g_dock_mutex);
  if (g_dock.active)
  {
    pthread_mutex_unlock(&g_dock_mutex);
    return false;
  }
  docking_begin(true);
  pthread_mutex_unlock(&g_dock_mutex);

  if (!g_thread_running)
  {
    g_thread_running = true;
    if (pthread_create(&g_traj_thread, NULL, trajectory_thread_func, NULL) !=
        0)
    {
      printf("[DOCK] Failed to create execution thread for docking test\n");
      g_thread_running = false;
      return false;
    }
  }
  return true;
}

void trajectory_stop(void)
{
  pthread_mutex_lock(&g_dock_mutex);
  docking_reset(false);
  pthread_mutex_unlock(&g_dock_mutex);

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

    pthread_mutex_lock(&g_dock_mutex);
    bool docking_active = g_dock.active;
    pthread_mutex_unlock(&g_dock_mutex);

    if ((!active || count == 0) && !docking_active)
    {
      usleep(100000); // Sleep 100ms when idle
      continue;
    }

    if (docking_active)
    {
      pthread_mutex_lock(&g_dock_mutex);
      DockStepResult step_result = docking_step_once();
      bool test_mode = g_dock.test_mode;
      pthread_mutex_unlock(&g_dock_mutex);

      if (step_result == DOCK_STEP_DONE)
      {
        trajectory_stop();
        const char *arrived_msg = "{\"type\": \"control\", \"status\": \"arrived\"}\n";
        send_to_laptop_clients(arrived_msg, strlen(arrived_msg));
        printf("[DOCK] %s complete -> sent arrived\n",
               test_mode ? "Test docking" : "Trajectory docking");
        continue;
      }
      if (step_result == DOCK_STEP_NOT_FOUND)
      {
        trajectory_stop();
        const char *not_found_msg = "{\"type\": \"control\", \"status\": \"not_found\"}\n";
        send_to_laptop_clients(not_found_msg, strlen(not_found_msg));
        printf("[DOCK] %s failed -> sent not_found\n",
               test_mode ? "Test docking" : "Trajectory docking");
        continue;
      }

      usleep(CONTROL_LOOP_DELAY_US);
      continue;
    }

    // === ROBOT2: Skip trajectory execution during transport mode ===
    // Robot2 is controlled by formation_manager (follow Robot1)
    // NOT by trajectory_executor during transport phase
#if ROBOT_ID == 2
    if (formation_is_transport_active())
    {
      // Robot2 trong transport mode: formation_manager đang control
      // Trajectory executor không can thiệp
      usleep(100000); // Sleep 100ms
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
    // During transport mode, trajectory contains CENTROID positions
    // So we need to compare centroid positions, not robot positions
    float search_x = cur_x;
    float search_y = cur_y;
    double transport_offset_x = 0.0, transport_offset_y = 0.0;
    bool is_transport = formation_is_transport_active();

    if (is_transport)
    {
      // Get transport offset (robot_pos - centroid at lock time)
      if (formation_get_transport_offset(&transport_offset_x, &transport_offset_y))
      {
        // Convert robot position to centroid position for search
        // centroid = robot_pos - offset
        search_x = cur_x - (float)transport_offset_x;
        search_y = cur_y - (float)transport_offset_y;
      }
    }

    // --- PURE PURSUIT: Find Lookahead Point ---
    // Search forward from current_index to find the first point beyond
    // lookahead_dist. This allows the robot to "skip" dense points and maintain
    // smooth velocity.
    // NOTE: During transport, we search using centroid position (search_x, search_y)
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
      // If we reach here, the point is within lookahead_dist.
      // Update current_index to "consume" passed waypoints.
      lookahead_idx = i; // Keep advancing until we find one outside
    }

    // If all remaining points are within lookahead, target the final point.
    if (lookahead_idx >= count)
    {
      lookahead_idx = count - 1;
    }

    // Update current_index to the lookahead point (or just before it)
    // This prevents re-scanning already passed points.
    pthread_mutex_lock(&g_traj_mutex);
    if (lookahead_idx > g_trajectory.current_index)
    {
      g_trajectory.current_index = lookahead_idx;
    }
    pthread_mutex_unlock(&g_traj_mutex);

    // Get the lookahead target point (this is CENTROID position from trajectory)
    TrajectoryPoint target = g_trajectory.points[lookahead_idx];

    // === VIRTUAL STRUCTURE: Convert centroid target to robot target ===
    // During transport mode, trajectory contains centroid (object) positions
    // Robot target = centroid + transport_offset (locked at grip time)
    if (is_transport)
    {
      // Update centroid target in formation manager
      formation_set_centroid_target((double)target.x, (double)target.y);

      // Get robot's actual target position = centroid + offset
      double robot_target_x, robot_target_y;
      if (formation_get_robot_target(&robot_target_x, &robot_target_y))
      {
        target.x = (float)robot_target_x;
        target.y = (float)robot_target_y;
        // theta is ignored - will use locked theta
      }
    }

    // --- STOP CONDITION: Check POSITION (and THETA if enabled) ---
    TrajectoryPoint final_point = g_trajectory.points[count - 1];

    // For transport mode, also convert final point to robot position
    if (is_transport)
    {
      final_point.x += (float)transport_offset_x;
      final_point.y += (float)transport_offset_y;
    }

    float dx_final = final_point.x - cur_x;
    float dy_final = final_point.y - cur_y;
    float distance_to_final = sqrtf(dx_final * dx_final + dy_final * dy_final);

    bool goal_reached = false;
    if (distance_to_final < acceptance_radius)
    {
#if ENABLE_THETA_TRACKING
      // Theta tracking enabled: check both position AND theta
      if (final_point.has_theta)
      {
        float theta_err = cur_theta - final_point.theta;
        // Normalization removed as per user request
        if (fabs(theta_err) < ACCEPTANCE_ANGLE)
        {
          goal_reached = true;
        }
      }
      else
      {
        // No theta in trajectory, only check distance
        goal_reached = true;
      }
#else
      // Theta tracking disabled: only check position distance
      goal_reached = true;
#endif
    }

    if (goal_reached)
    {
      if (goal_reached_start_time == 0)
      {
        goal_reached_start_time = get_time_ms();
        printf("[TRAJ] Entered acceptance zone. Holding for %dms...\n",
               ACCEPTANCE_HOLD_TIME_MS);
      }

      long elapsed = get_time_ms() - goal_reached_start_time;
      if (elapsed >= ACCEPTANCE_HOLD_TIME_MS)
      {
        pthread_mutex_lock(&g_dock_mutex);
        if (!g_dock.active)
        {
          docking_begin(false);
          printf("[TRAJ] Acceptance reached. Switch to docking phase.\n");
        }
        pthread_mutex_unlock(&g_dock_mutex);
        goal_reached_start_time = 0;
        continue;
      }

      // HOLDING: Send zero velocity while waiting in acceptance zone
      send_motor_velocity(0.0f, 0.0f, 0.0f);
      usleep(CONTROL_LOOP_DELAY_US);
      continue; // Skip PID control while holding
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
      double locked_theta = 0.0;
      if (formation_get_locked_theta(&locked_theta))
      {
        // P-controller to hold locked theta
        float theta_err = cur_theta - (float)locked_theta;
        // Normalize to [-PI, PI]
        while (theta_err > M_PI)
          theta_err -= 2.0f * M_PI;
        while (theta_err < -M_PI)
          theta_err += 2.0f * M_PI;

        const float KP_THETA_HOLD = 1.0f;
        const float MAX_ANGULAR_VEL_HOLD = 0.5f;
        cmd_dot_theta = -KP_THETA_HOLD * theta_err;
        if (cmd_dot_theta > MAX_ANGULAR_VEL_HOLD)
          cmd_dot_theta = MAX_ANGULAR_VEL_HOLD;
        if (cmd_dot_theta < -MAX_ANGULAR_VEL_HOLD)
          cmd_dot_theta = -MAX_ANGULAR_VEL_HOLD;
      }
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
