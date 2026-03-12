#include "json_handler.h"
#include "arm_controller.h"
#include "arm_kinematic.h"
#include "cJSON.h"
#include "client_manager.h"
#include "db_manager.h"
#include "ekf.h"
#include "formation_manager.h"
#include "imu_processor.h"
#include "localize.h"
#include "math.h"
#include "optical_flow.h"
#include "socket.h"
#include "sys_config.h"
#include <errno.h>
#include <pthread.h>
#include <sqlite3.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include "docking.h"

#define QUEUE_SIZE 100
#define MAX_JSON_SIZE 512

typedef struct
{
  char *data;
  int length;
} json_message_t;

typedef struct
{
  float rpm[4];
  long timestamp;
} encoder_data_t;

typedef struct
{
  float heading, pitch, roll;
  float w, x, y, z;
  float accel_x, accel_y, accel_z;
  float gravity_x, gravity_y, gravity_z;
  float gyro_raw_x, gyro_raw_y, gyro_raw_z;
  char status[32];
  long timestamp;
} bno055_data_t;

typedef struct
{
  float pos_x, pos_y, vel_x, vel_y, theta;
  float odom_x, odom_y;
  float quality;         // Localization quality
  float optical_quality; // Optical flow quality
  char source[32];
  long timestamp;
} position_data_t;

typedef struct
{
  char message[256];
  long timestamp;
} log_data_t;

#define ENCODER_QUEUE_SIZE 100
#define BNO055_QUEUE_SIZE 100
#define POSITION_QUEUE_SIZE 100
#define LOG_QUEUE_SIZE 100

static encoder_data_t encoder_queue[ENCODER_QUEUE_SIZE];
static int encoder_head = 0, encoder_tail = 0, encoder_count = 0;

static bno055_data_t bno055_queue[BNO055_QUEUE_SIZE];
static int bno055_head = 0, bno055_tail = 0, bno055_count = 0;

static position_data_t position_queue[POSITION_QUEUE_SIZE];
static int position_head = 0, position_tail = 0, position_count = 0;

static log_data_t log_queue[LOG_QUEUE_SIZE];
static int log_head = 0, log_tail = 0, log_count = 0;

static pthread_mutex_t queue_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t queue_cond = PTHREAD_COND_INITIALIZER;
static pthread_t handler_thread;
static pthread_t ekf_publish_thread;
static pthread_t sync_position_thread;

static bool handler_running = false;
static bool ekf_publish_running = false;
static bool sync_position_running = false;

static char g_robot_id[32] = "unknown";

static sqlite3 *db = NULL;
static long current_run_id = -1;

ekf_t g_ekf;
pthread_mutex_t g_ekf_mutex = PTHREAD_MUTEX_INITIALIZER;

// External Optical Flow variables
extern optical_flow_t g_optical_flow;
extern pthread_mutex_t g_optical_mutex;

// Function declarations
static void *json_handler_thread(void *arg);
void parse_json_message(const char *json_str, int length);
static void flush_queue(void);
void json_handler_push_encoder(const encoder_data_t *enc);
void json_handler_push_bno055(const bno055_data_t *bno);
void json_handler_push_position(const position_data_t *pos);
void json_handler_push_log(const log_data_t *log);

static void *ekf_publish_thread_func(void *arg);
static void *sync_position_thread_func(void *arg);
// Initialize the JSON handler and start the processing thread

static long get_current_timestamp_ms()
{
  static struct timeval start_time = {0};
  static long last_timestamp = 0;
  struct timeval now;

  if (start_time.tv_sec == 0)
  {
    gettimeofday(&start_time, NULL);
    return 0;
  }

  gettimeofday(&now, NULL);
  long timestamp = (now.tv_sec - start_time.tv_sec) * 1000 +
                   (now.tv_usec - start_time.tv_usec) / 1000;
  if (timestamp <= last_timestamp)
    timestamp = last_timestamp + 1;
  last_timestamp = timestamp;
  return timestamp;
}

void json_handler_push_encoder(const encoder_data_t *enc)
{
  pthread_mutex_lock(&queue_mutex);
  if (encoder_count < ENCODER_QUEUE_SIZE)
  {
    encoder_queue[encoder_tail] = *enc;
    encoder_tail = (encoder_tail + 1) % ENCODER_QUEUE_SIZE;
    encoder_count++;
  }
  pthread_mutex_unlock(&queue_mutex);
}

void json_handler_push_bno055(const bno055_data_t *bno)
{
  pthread_mutex_lock(&queue_mutex);
  if (bno055_count < BNO055_QUEUE_SIZE)
  {
    bno055_queue[bno055_tail] = *bno;
    bno055_tail = (bno055_tail + 1) % BNO055_QUEUE_SIZE;
    bno055_count++;
  }
  pthread_mutex_unlock(&queue_mutex);
}

void json_handler_push_position(const position_data_t *pos)
{
  pthread_mutex_lock(&queue_mutex);
  if (position_count < POSITION_QUEUE_SIZE)
  {
    position_queue[position_tail] = *pos;
    position_tail = (position_tail + 1) % POSITION_QUEUE_SIZE;
    position_count++;
  }
  pthread_mutex_unlock(&queue_mutex);
}

void json_handler_push_log(const log_data_t *log)
{
  pthread_mutex_lock(&queue_mutex);
  if (log_count < LOG_QUEUE_SIZE)
  {
    log_queue[log_tail] = *log;
    log_tail = (log_tail + 1) % LOG_QUEUE_SIZE;
    log_count++;
  }
  pthread_mutex_unlock(&queue_mutex);
}

bool json_handler_init(void)
{
  // Lấy stable position từ UWB localize trước khi khởi tạo EKF
  extern bool localize_get_stable_position(Coordinates * stable_pos);
  extern void localize_force_recalculate_stable_position(void);

  // Set Robot ID from compile-time define (sys_config.h)
#if ROBOT_ID == 1
  strcpy(g_robot_id, "robot1");
#elif ROBOT_ID == 2
  strcpy(g_robot_id, "robot2");
#else
#error "ROBOT_ID must be 1 or 2"
#endif
  printf("[JSON_HANDLER] Robot ID: %s (compile-time: ROBOT_ID=%d)\n",
         g_robot_id, ROBOT_ID);

  // Initialize Formation Manager
  formation_init();

  // Auto-configure formation role based on robot ID
  if (strcmp(g_robot_id, "robot1") == 0)
  {
    // Robot1 is ALWAYS leader - disable follow mode
    formation_set_follow_enabled(false);
    printf("[FORMATION] Robot1 configured as LEADER (follow disabled)\n");
  }
  else if (strcmp(g_robot_id, "robot2") == 0)
  {
    // Robot2 is ALWAYS follower - enable follow mode
    formation_set_follow_enabled(true);
    printf("[FORMATION] Robot2 configured as FOLLOWER (follow enabled)\n");
  }

  Coordinates stable_pos = {0};

  // Yêu cầu localize tính stable position
  localize_force_recalculate_stable_position();

  // Chờ lấy stable position (timeout 10 giây)
  printf("[JSON_HANDLER] Waiting for UWB stable position...\n");
  if (localize_get_stable_position(&stable_pos))
  {
    // Có stable position -> khởi tạo EKF với vị trí này
    ekf_init_with_position(&g_ekf, stable_pos.x, stable_pos.y);
  }
  else
  {
    // Không có stable position -> dùng default (origin)
    printf(
        "[JSON_HANDLER] WARNING: No UWB stable position, using default init\n");
    ekf_init_default(&g_ekf);
  }

  // Open database connection
  if (sqlite3_open(DB_FILE, &db) != SQLITE_OK)
  {
    fprintf(stderr, "[JSON_HANDLER] Failed to open database: %s\n",
            sqlite3_errmsg(db));
    return false;
  }

  // Initialize database if needed
  initialize_database(db);

  // Start a new run
  current_run_id = start_new_run(db);

  // Start handler thread
  handler_running = true;
  if (pthread_create(&handler_thread, NULL, json_handler_thread, NULL) != 0)
  {
    fprintf(stderr, "[JSON_HANDLER] Failed to create handler thread\n");
    handler_running = false;
    sqlite3_close(db);
    return false;
  }

  ekf_publish_running = true;
  if (pthread_create(&ekf_publish_thread, NULL, ekf_publish_thread_func,
                     NULL) != 0)
  {
    fprintf(stderr, "[JSON_HANDLER] Failed to create EKF publish thread\n");
    ekf_publish_running = false;
    handler_running = false;
    pthread_join(handler_thread, NULL);
    sqlite3_close(db);
    return false;
  }
  if (!imu_processor_init())
  {
    fprintf(stderr, "[JSON_HANDLER] Failed to initialize IMU processor\n");
    return false;
  }

#if !SINGLE_ROBOT_MODE
  // Start sync position thread (for multi-robot coordination)
  sync_position_running = true;
  if (pthread_create(&sync_position_thread, NULL, sync_position_thread_func,
                     NULL) != 0)
  {
    fprintf(stderr, "[JSON_HANDLER] Failed to create sync position thread\n");
    sync_position_running = false;
  }
#else
  printf("[JSON_HANDLER] SINGLE_ROBOT_MODE: sync_position thread TẮT (không có peer)\n");
#endif // !SINGLE_ROBOT_MODE

  printf("[JSON_HANDLER] Initialized with run_id: %ld\n", current_run_id);
  return true;
}

// Add a JSON message to the queue
bool json_handler_add_message(const char *json_message, int length)
{
  if (!handler_running || length <= 0 || length > MAX_JSON_SIZE)
  {
    return false;
  }

#if Cal_Freq == 1
  metrics_bump(json_message, length);
#endif
  // Parse và xử lý message ngay khi nhận
  parse_json_message(json_message, length);

  return true;
}

static void *ekf_publish_thread_func(void *arg)
{
  (void)arg;
  printf("[EKF] EKF publish thread started\n");

  while (ekf_publish_running)
  {
    // Gọi ekf_publish_position mỗi 50ms (20Hz)
    ekf_publish_position(&g_ekf);

    usleep(50000);
  }

  printf("[EKF] EKF publish thread stopped\n");
  return NULL;
}

// Sync position thread for multi-robot coordination (10Hz)
static void *sync_position_thread_func(void *arg)
{
  (void)arg;
  printf("[SYNC] Sync position thread started (10Hz)\n");

  while (sync_position_running)
  {
    // Get current epoch time for synchronization
    struct timeval tv;
    gettimeofday(&tv, NULL);
    double ts = (double)tv.tv_sec + (double)tv.tv_usec / 1000000.0;

    // Get EKF state and uncertainty (covariance)
    pthread_mutex_lock(&g_ekf_mutex);
    double x = g_ekf.x[0];
    double y = g_ekf.x[1];
    double vx = g_ekf.x[2];
    double vy = g_ekf.x[3];
    double theta = g_ekf.x[4];

    // Position uncertainty: use trace of position covariance submatrix
    // P[0][0] = var(x), P[1][1] = var(y)
    // uncertainty = sqrt(P[0][0] + P[1][1]) - RMS position error
    double pos_uncertainty = sqrt(g_ekf.P[0][0] + g_ekf.P[1][1]);

    // Velocity uncertainty (optional, for advanced control)
    double vel_uncertainty = sqrt(g_ekf.P[2][2] + g_ekf.P[3][3]);
    pthread_mutex_unlock(&g_ekf_mutex);

    // Send sync_position message to upstream server with uncertainty
    char json[420];
    snprintf(json, sizeof(json),
             "{\"type\":\"sync_position\",\"id\":\"%s\","
             "\"x\":%.6f,\"y\":%.6f,\"vx\":%.6f,\"vy\":%.6f,\"theta\":%.6f,"
             "\"ts\":%.6f,\"pos_unc\":%.6f,\"vel_unc\":%.6f}\n",
             g_robot_id, x, y, vx, vy, theta, ts, pos_uncertainty, vel_uncertainty);
    send_to_upstream_server(json, strlen(json));

    // Use configured sync rate from sys_config.h
    usleep(1000000 / SYNC_POSITION_RATE_HZ); // Default 20Hz = 50ms
  }

  printf("[SYNC] Sync position thread stopped\n");
  return NULL;
}
// Clean up resources
void json_handler_cleanup(void)
{
  if (!handler_running)
    return;

  // Yêu cầu thread dừng lại
  pthread_mutex_lock(&queue_mutex);
  handler_running = false;
  pthread_cond_signal(&queue_cond);
  pthread_mutex_unlock(&queue_mutex);

  // Chờ thread kết thúc
  pthread_join(handler_thread, NULL);

  // Dọn sạch các queue sensor
  pthread_mutex_lock(&queue_mutex);
  encoder_head = encoder_tail = encoder_count = 0;
  bno055_head = bno055_tail = bno055_count = 0;
  position_head = position_tail = position_count = 0;
  log_head = log_tail = log_count = 0;
  pthread_mutex_unlock(&queue_mutex);

  // Đóng database nếu còn mở
  if (db)
  {
    sqlite3_close(db);
    db = NULL;
  }

  printf("[JSON_HANDLER] Cleaned up resources\n");
}

// Main handler thread function
static void *json_handler_thread(void *arg)
{
  (void)arg;
  printf("[JSON_HANDLER] Handler thread started\n");
  while (handler_running)
  {
    usleep(500000);
    pthread_mutex_lock(&queue_mutex);
    flush_queue();
    pthread_mutex_unlock(&queue_mutex);
  }

  return NULL;
}
// Process and flush all messages in the queue
static void flush_queue(void)
{
  begin_transaction(db);
  // Encoder
  while (encoder_count > 0)
  {
    encoder_data_t *enc = &encoder_queue[encoder_head];
    log_encoder(db, current_run_id, enc->rpm[0], enc->rpm[1], enc->rpm[2],
                enc->rpm[3], enc->timestamp);
    encoder_head = (encoder_head + 1) % ENCODER_QUEUE_SIZE;
    encoder_count--;
  }
  // BNO055
  while (bno055_count > 0)
  {
    bno055_data_t *bno = &bno055_queue[bno055_head];
    log_bno055(db, current_run_id, bno->heading, bno->pitch, bno->roll, bno->w,
               bno->x, bno->y, bno->z, bno->accel_x, bno->accel_y, bno->accel_z,
               bno->gravity_x, bno->gravity_y, bno->gravity_z, bno->gyro_raw_x,
               bno->gyro_raw_y, bno->gyro_raw_z, bno->status, bno->timestamp);
    bno055_head = (bno055_head + 1) % BNO055_QUEUE_SIZE;
    bno055_count--;
  }
  // Position
  while (position_count > 0)
  {
    position_data_t *pos = &position_queue[position_head];
    if (strcmp(pos->source, "optical_flow") == 0)
    {
      log_position(db, current_run_id, 0.0, 0.0, 0.0, 0.0,
                   0.0, // ekf_x, ekf_y, ekf_vx, ekf_vy, ekf_theta
                   pos->pos_x, pos->pos_y, pos->vel_x,
                   pos->vel_y,         // optical_flow_x, optical_flow_y,
                                       // optical_flow_vx, optical_flow_vy
                   0.0, 0.0, 0.0, 0.0, // odom_x, odom_y, odom_vx, odom_vy
                   0.0, 0.0, 0.0, 0.0, // loc_x, loc_y, loc_vx, loc_vy
                   0.0, pos->quality,  // loc_quality, optical_quality
                   pos->timestamp);
    }
    else if (strcmp(pos->source, "odometry") == 0)
    {
      log_position(db, current_run_id, 0.0, 0.0, 0.0, 0.0,
                   0.0, // ekf_x, ekf_y, ekf_vx, ekf_vy, ekf_theta
                   0.0, 0.0, 0.0,
                   0.0, // bno055_x, bno055_y, bno055_vx, bno055_vy
                   pos->pos_x, pos->pos_y, pos->vel_x,
                   pos->vel_y,         // odom_x, odom_y, odom_vx, odom_vy
                   0.0, 0.0, 0.0, 0.0, // loc_x, loc_y, loc_vx, loc_vy
                   0.0, 0.0,           // loc_quality, optical_quality
                   pos->timestamp);
    }
    else if (strcmp(pos->source, "localization") == 0)
    {
      log_position(db, current_run_id, 0.0, 0.0, 0.0, 0.0,
                   pos->theta, // ekf_x, ekf_y, ekf_vx, ekf_vy, ekf_theta
                   0.0, 0.0, 0.0,
                   0.0, // bno055_x, bno055_y, bno055_vx, bno055_vy
                   pos->odom_x, pos->odom_y, 0.0,
                   0.0, // odom_x, odom_y, odom_vx, odom_vy
                   pos->pos_x, pos->pos_y, pos->vel_x,
                   pos->vel_y,        // loc_x, loc_y, loc_vx, loc_vy
                   pos->quality, 0.0, // quality, optical_quality
                   pos->timestamp);
    }
    else if (strcmp(pos->source, "ekf") == 0)
    {
      log_position(
          db, current_run_id, pos->pos_x, pos->pos_y, pos->vel_x, pos->vel_y,
          pos->theta,         // ekf_x, ekf_y, ekf_vx, ekf_vy, ekf_theta
          0.0, 0.0, 0.0, 0.0, // bno055_x, bno055_y, bno055_vx, bno055_vy
          0.0, 0.0, 0.0, 0.0, // odom_x, odom_y, odom_vx, odom_vy
          0.0, 0.0, 0.0, 0.0, // loc_x, loc_y, loc_vx, loc_vy
          0.0, 0.0,           // loc_quality, optical_quality
          pos->timestamp);
    }
    position_head = (position_head + 1) % POSITION_QUEUE_SIZE;
    position_count--;
  }
  // Log
  while (log_count > 0)
  {
    log_data_t *log = &log_queue[log_head];
    log_system(db, current_run_id, log->message, log->timestamp);
    log_head = (log_head + 1) % LOG_QUEUE_SIZE;
    log_count--;
  }

  commit_transaction(db);
}

static inline void rotate_accel_by_heading_deg(float heading_deg, float *ax_b,
                                               float *ay_b, float *az_b)
{
  const float d2r = (float)M_PI / 180.0f;
  float c = cosf(heading_deg * d2r);
  float s = sinf(heading_deg * d2r);

  // Lấy giá trị hiện tại (body frame)
  float ax = *ax_b;
  float ay = *ay_b;
  float az = *az_b;

  // Global = Rz(heading) * Body
  float ax_g = c * ax - s * ay;
  float ay_g = s * ax + c * ay;
  float az_g = az; // robot 2D có thể đặt 0.0f nếu muốn

  // Ghi trả kết quả qua con trỏ
  *ax_b = ax_g;
  *ay_b = ay_g;
  *az_b = az_g;
}

// Helper function: Rotate velocity from global to body frame using theta
static inline void rotate_global_to_body(double theta_rad, double vxg,
                                         double vyg, double *vxb, double *vyb)
{
  double c = cos(theta_rad);
  double s = sin(theta_rad);

  // Body = Rz(-theta) * Global = Rz(theta)^T * Global
  // [vxb]   [cos(th)   sin(th)] [vxg]
  // [vyb] = [-sin(th)  cos(th)] [vyg]
  *vxb = c * vxg + s * vyg;
  *vyb = -s * vxg + c * vyg;
}

// Parse a JSON message and call appropriate handlers
static double last_vxg = 0.0, last_vyg = 0.0;

// Odometry position (exported for trajectory_executor when USE_ODOMETRY_FOR_TRAJECTORY=1)
float g_odom_x = 0.0f, g_odom_y = 0.0f, g_odom_theta = 0.0f;
pthread_mutex_t g_odom_mutex = PTHREAD_MUTEX_INITIALIZER;

static float last_loc_x = 0.0f, last_loc_y = 0.0f;
static long last_loc_timestamp = 0;
static bool first_loc_data = true;

void parse_json_message(const char *json_str, int length)
{
  long timestamp = get_current_timestamp_ms();
  cJSON *json = cJSON_ParseWithLength(json_str, length);
  if (!json)
    return;

  cJSON *type_json = cJSON_GetObjectItemCaseSensitive(json, "type");
  if (!cJSON_IsString(type_json))
  {
    cJSON_Delete(json);
    return;
  }
  const char *type = type_json->valuestring;
  cJSON *data_json = cJSON_GetObjectItemCaseSensitive(json, "data");

  if (strcmp(type, "encoder") == 0)
  {
    encoder_data_t enc = {0};
    enc.timestamp = timestamp;
    if (data_json && cJSON_IsArray(data_json))
    {
      for (int i = 0; i < 4; ++i)
      {
        cJSON *item = cJSON_GetArrayItem(data_json, i);
        if (item)
          enc.rpm[i] = item->valuedouble;
      }
    }
    json_handler_push_encoder(&enc);
  }
  else if (strcmp(type, "bno055") == 0)
  {
    bno055_data_t bno = {0};
    bno.timestamp = timestamp;
    strcpy(bno.status, "unknown");
    if (data_json && cJSON_IsObject(data_json))
    {
      cJSON *euler = cJSON_GetObjectItemCaseSensitive(data_json, "euler");
      if (euler && cJSON_IsArray(euler))
      {
        bno.heading = cJSON_GetArrayItem(euler, 0)->valuedouble;
        bno.pitch = cJSON_GetArrayItem(euler, 1)->valuedouble;
        bno.roll = cJSON_GetArrayItem(euler, 2)->valuedouble;
      }
      cJSON *quaternion =
          cJSON_GetObjectItemCaseSensitive(data_json, "quaternion");
      if (quaternion && cJSON_IsArray(quaternion))
      {
        bno.w = cJSON_GetArrayItem(quaternion, 0)->valuedouble;
        bno.x = cJSON_GetArrayItem(quaternion, 1)->valuedouble;
        bno.y = cJSON_GetArrayItem(quaternion, 2)->valuedouble;
        bno.z = cJSON_GetArrayItem(quaternion, 3)->valuedouble;
      }
      cJSON *lin_accel =
          cJSON_GetObjectItemCaseSensitive(data_json, "lin_accel");
      if (lin_accel && cJSON_IsArray(lin_accel))
      {
        bno.accel_x = cJSON_GetArrayItem(lin_accel, 0)->valuedouble;
        bno.accel_y = cJSON_GetArrayItem(lin_accel, 1)->valuedouble;
        bno.accel_z = cJSON_GetArrayItem(lin_accel, 2)->valuedouble;
      }
      cJSON *gravity = cJSON_GetObjectItemCaseSensitive(data_json, "gravity");
      if (gravity && cJSON_IsArray(gravity))
      {
        bno.gravity_x = cJSON_GetArrayItem(gravity, 0)->valuedouble;
        bno.gravity_y = cJSON_GetArrayItem(gravity, 1)->valuedouble;
        bno.gravity_z = cJSON_GetArrayItem(gravity, 2)->valuedouble;
      }
      cJSON *gyro_raw = cJSON_GetObjectItemCaseSensitive(data_json, "gyro_raw");
      if (gyro_raw && cJSON_IsArray(gyro_raw))
      {
        bno.gyro_raw_x = cJSON_GetArrayItem(gyro_raw, 0)->valuedouble;
        bno.gyro_raw_y = cJSON_GetArrayItem(gyro_raw, 1)->valuedouble;
        bno.gyro_raw_z = cJSON_GetArrayItem(gyro_raw, 2)->valuedouble;
      }
      cJSON *status = cJSON_GetObjectItemCaseSensitive(data_json, "status");
      if (status && cJSON_IsString(status))
      {
        strncpy(bno.status, status->valuestring, sizeof(bno.status) - 1);
      }
    }
    //
    float filtered_accel_x, filtered_accel_y, filtered_accel_z;
    imu_process_accel(bno.accel_x, bno.accel_y, bno.accel_z, &filtered_accel_x,
                      &filtered_accel_y, &filtered_accel_z);

    bno.accel_x = filtered_accel_x;
    bno.accel_y = filtered_accel_y;
    bno.accel_z = filtered_accel_z;

    rotate_accel_by_heading_deg(bno.heading, &bno.accel_x, &bno.accel_y,
                                &bno.accel_z);

    // Update EKF với IMU heading
    // Dùng global velocities trực tiếp (Linear KF)
    double z[1] = {bno.heading};
    double dt = dt_from_last_predict();
    ekf_predict_input_global(&g_ekf, last_vxg, last_vyg, dt);
    ekf_update_sensor(&g_ekf, EKF_SENSOR_IMU, z);
    json_handler_push_bno055(&bno);
  }
  else if (strcmp(type, "position") == 0)
  {
    position_data_t pos = {0};
    pos.timestamp = timestamp;
    strcpy(pos.source, "unknown");
    cJSON *source_json = cJSON_GetObjectItemCaseSensitive(json, "source");
    if (source_json && cJSON_IsString(source_json))
    {
      strncpy(pos.source, source_json->valuestring, sizeof(pos.source) - 1);
    }
    if (data_json && cJSON_IsObject(data_json))
    {
      cJSON *position = cJSON_GetObjectItemCaseSensitive(data_json, "position");
      if (position && cJSON_IsArray(position))
      {
        if (strcmp(pos.source, "ekf") == 0)
        {
          pos.pos_x = cJSON_GetArrayItem(position, 0)->valuedouble;
          pos.pos_y = cJSON_GetArrayItem(position, 1)->valuedouble;
          pos.odom_x = cJSON_GetArrayItem(position, 2)->valuedouble;
          pos.odom_y = cJSON_GetArrayItem(position, 3)->valuedouble;
          pos.theta = cJSON_GetArrayItem(position, 4)->valuedouble;

          json_handler_push_position(&pos);
          return;
        }
        else if (strcmp(pos.source, "localization") == 0)
        {
          float original_loc_x = cJSON_GetArrayItem(position, 0)->valuedouble;
          float original_loc_y = cJSON_GetArrayItem(position, 1)->valuedouble;
          float quality = 0.0f;
          // Phần tử thứ 3 là quality
          cJSON *quality_item = cJSON_GetArrayItem(position, 2);
          if (quality_item)
            quality = quality_item->valuedouble;

          float loc_vel_x = 0.0f, loc_vel_y = 0.0f;
          if (!first_loc_data && timestamp > last_loc_timestamp)
          {
            // Time difference in seconds
            float dt = (timestamp - last_loc_timestamp) / 1000.0f;
            // Avoid division by very small numbers
            if (dt > 0.001f)
            {
              loc_vel_x = (original_loc_x - last_loc_x) / dt;
              loc_vel_y = (original_loc_y - last_loc_y) / dt;
            }
          }
          last_loc_x = original_loc_x;
          last_loc_y = original_loc_y;
          last_loc_timestamp = timestamp;
          first_loc_data = false;

          pos.vel_x = loc_vel_x;
          pos.vel_y = loc_vel_y;

          pos.pos_x = original_loc_x;
          pos.pos_y = original_loc_y;
          pos.quality = quality;
        }
        else
        {
          pos.pos_x = cJSON_GetArrayItem(position, 0)->valuedouble;
          pos.pos_y = cJSON_GetArrayItem(position, 1)->valuedouble;
        }
      }
      cJSON *velocity = cJSON_GetObjectItemCaseSensitive(data_json, "velocity");
      if (velocity && cJSON_IsArray(velocity))
      {
        pos.vel_x = cJSON_GetArrayItem(velocity, 0)->valuedouble;
        pos.vel_y = cJSON_GetArrayItem(velocity, 1)->valuedouble;
      }
      cJSON *quality_json =
          cJSON_GetObjectItemCaseSensitive(data_json, "quality");
      if (quality_json && cJSON_IsNumber(quality_json))
      {
        pos.quality = (float)quality_json->valueint;
      }
    }
    if (strcmp(pos.source, "odometry") == 0)
    {
      // Odometry từ ESP32 đã được xoay sang global frame
      // Cần xoay lại về body frame để sử dụng theta trong EKF (TRUE EKF)
      pthread_mutex_lock(&g_odom_mutex);
      g_odom_x = pos.pos_x;
      g_odom_y = pos.pos_y;
      pthread_mutex_unlock(&g_odom_mutex);

      // Lưu global velocities cho optical flow prediction
      last_vxg = pos.vel_x;
      last_vyg = pos.vel_y;

      // Xoay vận tốc từ global về body frame sử dụng theta hiện tại từ EKF
      pthread_mutex_lock(&g_ekf_mutex);
      double theta = g_ekf.x[4];
      pthread_mutex_unlock(&g_ekf_mutex);

      double vxb, vyb;
      rotate_global_to_body(theta, pos.vel_x, pos.vel_y, &vxb, &vyb);

      // Predict EKF với body frame velocities (TRUE EKF)
      double dt = dt_from_last_predict();
      ekf_predict_input_body(&g_ekf, vxb, vyb, dt);
    }
#if ENABLE_OPTICAL_FLOW == 1
    else if (strcmp(pos.source, "optical_flow") == 0)
    {
      // Predict trước khi update EKF với optical flow velocity
      // Dùng global velocities trực tiếp (Linear KF)
      double dt = dt_from_last_predict();
      ekf_predict_input_global(&g_ekf, last_vxg, last_vyg, dt);
#if TEST_MODE_WHEEL_UP == 0
      // EKF expects z[4] = {x, y, vx, vy} - consistent với tên hàm
      // ekf_update_pos_vel_xyvxvy
      double z[4] = {pos.pos_x, pos.pos_y, pos.vel_x, pos.vel_y};
      ekf_update_sensor(&g_ekf, EKF_SENSOR_OPTICAL_FLOW, z);
#endif
    }
#endif
    else if (strcmp(pos.source, "localization") == 0)
    {
#if TEST_MODE_WHEEL_UP == 0
      double z[2] = {pos.pos_x, pos.pos_y};
#endif
      // Dùng global velocities trực tiếp (Linear KF)
      double dt = dt_from_last_predict();
      ekf_predict_input_global(&g_ekf, last_vxg, last_vyg, dt);
#if TEST_MODE_WHEEL_UP == 0
      ekf_update_sensor(&g_ekf, EKF_SENSOR_LOCALIZATION, z);
#endif
    }
    json_handler_push_position(&pos);
  }
  else if (strcmp(type, "log") == 0)
  {
    log_data_t log = {0};
    log.timestamp = timestamp;
    cJSON *message_json = cJSON_GetObjectItemCaseSensitive(json, "message");
    if (message_json && cJSON_IsString(message_json))
    {
      strncpy(log.message, message_json->valuestring, sizeof(log.message) - 1);
    }
    json_handler_push_log(&log);
  }
  // ========== ARM CONTROL HANDLING ==========
  else if (strcmp(type, "arm_ik_request") == 0)
  {
    if (data_json && cJSON_IsObject(data_json))
    {
      // 1. Parse coordinates from JSON
      cJSON *x_json = cJSON_GetObjectItemCaseSensitive(data_json, "x");
      cJSON *y_json = cJSON_GetObjectItemCaseSensitive(data_json, "y");
      cJSON *z_json = cJSON_GetObjectItemCaseSensitive(data_json, "z");

      if (x_json && y_json && z_json)
      {
        double x = x_json->valuedouble;
        double y = y_json->valuedouble;
        double z = z_json->valuedouble;
        double pitch = -90.0; // Always use -90° (gripper points straight down)

        printf("[ARM] IK Request: X=%.1f, Y=%.1f, Z=%.1f, Pitch=%.1f\n", x, y,
               z, pitch);

        // 2. Calculate IK
        ArmAngles angles;
        if (arm_ik_solve_simple(x, y, z, pitch, &angles))
        {
          printf(
              "[ARM] IK Solved: J0=%.1f, J1=%.1f, J2=%.1f, J3=%.1f, J4=%.1f\n",
              angles.j0, angles.j1, angles.j2, angles.j3, angles.j4);

          // 3. Send servo commands to ESP32
          double servo_angles[6] = {angles.j0, angles.j1, angles.j2,
                                    angles.j3, angles.j4, 90.0};
          for (int i = 0; i < 6; i++)
          {
            char cmd[128];
            snprintf(cmd, sizeof(cmd),
                     "{\"cmd\":\"servo\",\"ch\":%d,\"deg\":%.1f}\n", i,
                     servo_angles[i]);
            client_manager_broadcast(cmd, strlen(cmd));
          }

          // 4. Send IK result back to GUI
          char result[256];
          snprintf(result, sizeof(result),
                   "{\"type\":\"arm_ik_result\",\"data\":{\"j0\":%.2f,\"j1\":%."
                   "2f,\"j2\":%.2f,\"j3\":%.2f,\"j4\":%.2f,\"j5\":%.2f}}\n",
                   angles.j0, angles.j1, angles.j2, angles.j3, angles.j4, 90.0);
          send_to_upstream_server(result, strlen(result));
        }
        else
        {
          printf(
              "[ARM] IK Failed: No solution found for X=%.1f, Y=%.1f, Z=%.1f\n",
              x, y, z);
          // Send error message back to GUI
          const char *err = "{\"type\":\"arm_ik_result\",\"error\":\"No IK "
                            "solution found\"}\n";
          send_to_upstream_server(err, strlen(err));
        }
      }
      else
      {
        printf("[ARM] IK Request missing required fields (x, y, z, pitch)\n");
      }
    }
  }
  else if (strcmp(type, "arm_servo_cmd") == 0)
  {
    if (data_json && cJSON_IsObject(data_json))
    {
      printf("[ARM] Direct servo command received\n");

      // Joint names mapping
      const char *joints[] = {"j0", "j1", "j2", "j3", "j4", "j5"};

      for (int i = 0; i < 6; i++)
      {
        cJSON *joint = cJSON_GetObjectItemCaseSensitive(data_json, joints[i]);
        if (joint && cJSON_IsNumber(joint))
        {
          double deg = joint->valuedouble;
          char cmd[128];
          snprintf(cmd, sizeof(cmd),
                   "{\"cmd\":\"servo\",\"ch\":%d,\"deg\":%.1f}\n", i, deg);
          client_manager_broadcast(cmd, strlen(cmd));
          printf("[ARM] Sent servo ch=%d deg=%.1f\n", i, deg);
        }
      }
    }
  }
  // ========== NEW ARM CONTROL COMMANDS ==========
  else if (strcmp(type, "arm_pick") == 0)
  {
    // Pick object at position (x, y, z)
    if (data_json && cJSON_IsObject(data_json))
    {
      cJSON *x_json = cJSON_GetObjectItemCaseSensitive(data_json, "x");
      cJSON *y_json = cJSON_GetObjectItemCaseSensitive(data_json, "y");
      cJSON *z_json = cJSON_GetObjectItemCaseSensitive(data_json, "z");

      if (x_json && y_json && z_json)
      {
        double x = x_json->valuedouble;
        double y = y_json->valuedouble;
        double z = z_json->valuedouble;

        bool success = arm_pick(x, y, z);

        // Send result back
        char result[128];
        snprintf(result, sizeof(result),
                 "{\"type\":\"arm_pick_result\",\"ok\":%s}\n",
                 success ? "true" : "false");
        send_to_upstream_server(result, strlen(result));
      }
      else
      {
        printf("[ARM] Pick missing required fields (x, y, z)\n");
        const char *err =
            "{\"type\":\"arm_pick_result\",\"ok\":false,\"error\":\"missing "
            "fields\"}\n";
        send_to_upstream_server(err, strlen(err));
      }
    }
  }
  else if (strcmp(type, "arm_place") == 0)
  {
    // Place object at position (x, y, z)
    if (data_json && cJSON_IsObject(data_json))
    {
      cJSON *x_json = cJSON_GetObjectItemCaseSensitive(data_json, "x");
      cJSON *y_json = cJSON_GetObjectItemCaseSensitive(data_json, "y");
      cJSON *z_json = cJSON_GetObjectItemCaseSensitive(data_json, "z");

      if (x_json && y_json && z_json)
      {
        double x = x_json->valuedouble;
        double y = y_json->valuedouble;
        double z = z_json->valuedouble;

        bool success = arm_place(x, y, z);

        char result[128];
        snprintf(result, sizeof(result),
                 "{\"type\":\"arm_place_result\",\"ok\":%s}\n",
                 success ? "true" : "false");
        send_to_upstream_server(result, strlen(result));
      }
      else
      {
        printf("[ARM] Place missing required fields (x, y, z)\n");
        const char *err =
            "{\"type\":\"arm_place_result\",\"ok\":false,\"error\":\"missing "
            "fields\"}\n";
        send_to_upstream_server(err, strlen(err));
      }
    }
  }
  else if (strcmp(type, "arm_gripper") == 0)
  {
    // Control gripper: {"action": "open"} or {"action": "close"}
    if (data_json && cJSON_IsObject(data_json))
    {
      cJSON *action_json =
          cJSON_GetObjectItemCaseSensitive(data_json, "action");
      if (action_json && cJSON_IsString(action_json))
      {
        bool success = arm_gripper(action_json->valuestring);

        char result[128];
        snprintf(result, sizeof(result),
                 "{\"type\":\"arm_gripper_result\",\"ok\":%s,\"action\":\"%s\"}"
                 "\n",
                 success ? "true" : "false", action_json->valuestring);
        send_to_upstream_server(result, strlen(result));
      }
      else
      {
        printf("[ARM] Gripper missing action field\n");
        const char *err =
            "{\"type\":\"arm_gripper_result\",\"ok\":false,\"error\":\"missing "
            "action\"}\n";
        send_to_upstream_server(err, strlen(err));
      }
    }
  }
  else if (strcmp(type, "arm_rest") == 0)
  {
    // Move to rest position
    printf("[ARM] Received arm_rest command, calling arm_rest()...\n");
    bool success = arm_rest();

    char result[128];
    snprintf(result, sizeof(result),
             "{\"type\":\"arm_rest_result\",\"ok\":%s}\n",
             success ? "true" : "false");
    send_to_upstream_server(result, strlen(result));
  }
  // ========== SYNC POSITION HANDLING (Multi-Robot) ==========
  else if (strcmp(type, "sync_position") == 0)
  {
    // Ignore our own messages (loopback)
    cJSON *id_json = cJSON_GetObjectItemCaseSensitive(json, "id");
    const char *sender_id = (id_json && cJSON_IsString(id_json)) ? id_json->valuestring : "unknown";

    if (strcmp(sender_id, g_robot_id) == 0)
    {
      // It's me, ignore
      cJSON_Delete(json);
      return;
    }

    // Determine if we should process this sync_position:
    // - Follower (Robot2): Always process to follow leader
    // - Leader (Robot1): Only process in TRANSPORT mode to track follower
    bool should_process = false;
    bool is_follower = formation_is_follow_enabled();
    bool is_transport = formation_is_transport_active();

    if (is_follower)
    {
      // I'm a follower - need leader's position
      should_process = true;
    }
    else if (is_transport)
    {
      // I'm the leader but in transport mode - need follower's position
      // to calculate follower error and adjust speed
      should_process = true;
    }

    // Debug log (rate-limited)

    if (!should_process)
    {
      // Leader not in transport mode, ignore
      cJSON_Delete(json);
      return;
    }

    double neighbor_x = 0, neighbor_y = 0, neighbor_vx = 0, neighbor_vy = 0,
           neighbor_theta = 0, neighbor_ts = 0;
    double neighbor_pos_unc = 0.0; // Position uncertainty (optional)

    cJSON *ts_json = cJSON_GetObjectItemCaseSensitive(json, "ts");
    if (ts_json && cJSON_IsNumber(ts_json))
      neighbor_ts = ts_json->valuedouble;

    cJSON *x_json = cJSON_GetObjectItemCaseSensitive(json, "x");
    if (x_json && cJSON_IsNumber(x_json))
      neighbor_x = x_json->valuedouble;

    cJSON *y_json = cJSON_GetObjectItemCaseSensitive(json, "y");
    if (y_json && cJSON_IsNumber(y_json))
      neighbor_y = y_json->valuedouble;

    cJSON *vx_json = cJSON_GetObjectItemCaseSensitive(json, "vx");
    if (vx_json && cJSON_IsNumber(vx_json))
      neighbor_vx = vx_json->valuedouble;

    cJSON *vy_json = cJSON_GetObjectItemCaseSensitive(json, "vy");
    if (vy_json && cJSON_IsNumber(vy_json))
      neighbor_vy = vy_json->valuedouble;

    cJSON *theta_json = cJSON_GetObjectItemCaseSensitive(json, "theta");
    if (theta_json && cJSON_IsNumber(theta_json))
      neighbor_theta = theta_json->valuedouble;

    // Parse position uncertainty (optional field)
    cJSON *pos_unc_json = cJSON_GetObjectItemCaseSensitive(json, "pos_unc");
    if (pos_unc_json && cJSON_IsNumber(pos_unc_json))
      neighbor_pos_unc = pos_unc_json->valuedouble;

    // Update via Formation Manager with uncertainty
    formation_update_neighbor(neighbor_x, neighbor_y, neighbor_vx, neighbor_vy,
                              neighbor_theta, neighbor_ts, neighbor_pos_unc);
  }
  // ========== LOCK TRANSPORT OFFSET (without grip) ==========
  // Use this when testing without physical gripper
  else if (strcmp(type, "control") == 0)
  {
    cJSON *cmd_json = cJSON_GetObjectItemCaseSensitive(json, "cmd");

    // Check for lock_transport_offset command (for testing without grip)
    if (cmd_json && cJSON_IsString(cmd_json) &&
        strcmp(cmd_json->valuestring, "lock_transport_offset") == 0)
    {
      printf("[TRANSPORT] Received lock_transport_offset command\n");

      cJSON *object_pos = cJSON_GetObjectItemCaseSensitive(json, "object_pos");

      if (object_pos && cJSON_IsArray(object_pos) &&
          cJSON_GetArraySize(object_pos) >= 2)
      {
        double obj_x = cJSON_GetArrayItem(object_pos, 0)->valuedouble;
        double obj_y = cJSON_GetArrayItem(object_pos, 1)->valuedouble;

        printf("[TRANSPORT] Locking offset with centroid: (%.3f, %.3f)\n",
               obj_x, obj_y);

        // Lock transport offset (same as what execute_grip does)
        formation_lock_transport_offset(obj_x, obj_y);

        // Send result back
        char result[256];
        snprintf(result, sizeof(result),
                 "{\"type\":\"control\",\"status\":\"offset_locked\",\"cmd\":"
                 "\"lock_transport_offset_result\"}\n");
        send_to_upstream_server(result, strlen(result));
      }
      else
      {
        printf("[TRANSPORT] ERROR: Missing object_pos\n");
        const char *err = "{\"type\":\"control\",\"status\":\"error\",\"cmd\":"
                          "\"lock_transport_offset_result\"}\n";
        send_to_upstream_server(err, strlen(err));
      }

      cJSON_Delete(json);
      return; // Exit early, don't process execute_grip below
    }

    // Check for execute_grip command
    if (cmd_json && cJSON_IsString(cmd_json) &&
        strcmp(cmd_json->valuestring, "execute_grip") == 0)
    {
      printf("[GRIP] Received execute_grip command from laptop\n");

      // Parse object_pos [x, y]
      cJSON *object_pos = cJSON_GetObjectItemCaseSensitive(json, "object_pos");
      // Parse object_size [length, width]
      cJSON *object_size =
          cJSON_GetObjectItemCaseSensitive(json, "object_size");
      // Parse grip_side
      cJSON *grip_side = cJSON_GetObjectItemCaseSensitive(json, "grip_side");

      // Parse time field (optional - for synchronized execution)
      cJSON *time_json = cJSON_GetObjectItemCaseSensitive(json, "time");

      if (object_pos && cJSON_IsArray(object_pos) &&
          cJSON_GetArraySize(object_pos) >= 2 && object_size &&
          cJSON_IsArray(object_size) && cJSON_GetArraySize(object_size) >= 2 &&
          grip_side && cJSON_IsString(grip_side))
      {
        double obj_x = cJSON_GetArrayItem(object_pos, 0)->valuedouble;
        double obj_y = cJSON_GetArrayItem(object_pos, 1)->valuedouble;
        double obj_length = cJSON_GetArrayItem(object_size, 0)->valuedouble;
        double obj_width = cJSON_GetArrayItem(object_size, 1)->valuedouble;
        const char *side = grip_side->valuestring;

        printf("[GRIP] Object: pos(%.3f, %.3f) size(%.2f x %.2f) side=%s\n",
               obj_x, obj_y, obj_length, obj_width, side);

        // Handle time synchronization if specified
        if (time_json && cJSON_IsNumber(time_json))
        {
          double scheduled_time = time_json->valuedouble;
          printf("[GRIP] Scheduled time: %.3f\n", scheduled_time);
        }

        // Get current robot position from EKF
        extern ekf_t g_ekf;
        extern pthread_mutex_t g_ekf_mutex;

        pthread_mutex_lock(&g_ekf_mutex);
        double robot_x = g_ekf.x[0];
        double robot_y = g_ekf.x[1];
        double robot_theta = g_ekf.x[4]; // theta in radians
        pthread_mutex_unlock(&g_ekf_mutex);

        printf("[GRIP] Robot EKF: pos(%.3f, %.3f) theta=%.2f rad\n", robot_x,
               robot_y, robot_theta);

        bool success = false;

#if ENABLE_DOCKING
        if (docking_is_complete())
        {
          // === DOCKING MODE: Grip at fixed distance ===
          // VL53L0X đã docking chính xác, dùng khoảng cách cố định
          // Body frame: X=0 (ngay tâm), Y=dock_distance (trước mặt)
          double grip_dist_m = (double)DOCK_FIXED_GRIP_DISTANCE_MM / 1000.0;

          // Convert body offset to global position:
          // obj_global = robot_pos + R(theta) * [grip_dist; 0]
          // (robot nhìn thẳng về vật, offset theo trục X body = hướng trước)
          double dock_obj_x = robot_x + cos(robot_theta) * grip_dist_m;
          double dock_obj_y = robot_y + sin(robot_theta) * grip_dist_m;

          printf("[GRIP] DOCKING MODE: Fixed grip at distance %d mm\n",
                 DOCK_FIXED_GRIP_DISTANCE_MM);
          printf("[GRIP] Fixed obj pos: (%.3f, %.3f) [body offset: X=%.3f, Y=0]\n",
                 dock_obj_x, dock_obj_y, grip_dist_m);

          success = arm_execute_grip(robot_x, robot_y, robot_theta,
                                    dock_obj_x, dock_obj_y,
                                    obj_length, obj_width, side);
        }
        else
#endif // ENABLE_DOCKING
        {
          // === ORIGINAL MODE: Grip using server object_pos ===
          printf("[GRIP] NORMAL MODE: Grip using server position\n");
          success = arm_execute_grip(robot_x, robot_y, robot_theta, obj_x,
                                    obj_y, obj_length, obj_width, side);
        }

        // Lock transport offset for Virtual Structure mode (Phase 2)
        // centroid = object center position
        if (success)
        {
          formation_lock_transport_offset(obj_x, obj_y);
          printf("[GRIP] Transport offset locked for Phase 2\n");
        }

        // Send result back to laptop
        char result[256];
        snprintf(result, sizeof(result),
                 "{\"type\":\"control\",\"status\":\"%s\",\"cmd\":\"execute_"
                 "grip_result\"}\n",
                 success ? "grip_complete" : "grip_failed");
        send_to_upstream_server(result, strlen(result));
      }
      else
      {
        printf("[GRIP] ERROR: Missing required fields (object_pos, "
               "object_size, grip_side)\n");
        const char *err = "{\"type\":\"control\",\"status\":\"error\",\"cmd\":"
                          "\"execute_grip_result\","
                          "\"error\":\"missing required fields\"}\n";
        send_to_upstream_server(err, strlen(err));
      }
    }

    // Check for execute_place command
    if (cmd_json && cJSON_IsString(cmd_json) &&
        strcmp(cmd_json->valuestring, "execute_place") == 0)
    {
      printf("[PLACE] Received execute_place command from laptop\n");

      // Parse object_pos [x, y]
      cJSON *object_pos = cJSON_GetObjectItemCaseSensitive(json, "object_pos");
      // Parse object_size [length, width]
      cJSON *object_size =
          cJSON_GetObjectItemCaseSensitive(json, "object_size");
      // Parse grip_side (required - must match the side used during grip)
      cJSON *grip_side_json =
          cJSON_GetObjectItemCaseSensitive(json, "grip_side");

      if (object_pos && cJSON_IsArray(object_pos) &&
          cJSON_GetArraySize(object_pos) >= 2 && object_size &&
          cJSON_IsArray(object_size) && cJSON_GetArraySize(object_size) >= 2 &&
          grip_side_json && cJSON_IsString(grip_side_json))
      {
        double obj_x = cJSON_GetArrayItem(object_pos, 0)->valuedouble;
        double obj_y = cJSON_GetArrayItem(object_pos, 1)->valuedouble;
        double obj_length = cJSON_GetArrayItem(object_size, 0)->valuedouble;
        double obj_width = cJSON_GetArrayItem(object_size, 1)->valuedouble;
        const char *side = grip_side_json->valuestring;

        printf("[PLACE] Object: pos(%.3f, %.3f) size(%.2f x %.2f) side=%s\n",
               obj_x, obj_y, obj_length, obj_width, side);

        // Get current robot position from EKF
        extern ekf_t g_ekf;
        extern pthread_mutex_t g_ekf_mutex;

        pthread_mutex_lock(&g_ekf_mutex);
        double robot_x = g_ekf.x[0];
        double robot_y = g_ekf.x[1];
        double robot_theta = g_ekf.x[4]; // theta in radians
        pthread_mutex_unlock(&g_ekf_mutex);

        printf("[PLACE] Robot EKF: pos(%.3f, %.3f) theta=%.2f rad\n", robot_x,
               robot_y, robot_theta);

        // Execute place
        bool success = arm_execute_place(robot_x, robot_y, robot_theta, obj_x,
                                         obj_y, obj_length, obj_width, side);

        // End transport mode after placing
        if (success)
        {
          formation_end_transport();
          printf("[PLACE] Transport mode ended after successful place\n");
        }

        // Send result back to laptop
        char result[256];
        snprintf(result, sizeof(result),
                 "{\"type\":\"control\",\"status\":\"%s\",\"cmd\":\"execute_"
                 "place_result\"}\n",
                 success ? "place_complete" : "place_failed");
        send_to_upstream_server(result, strlen(result));
      }
      else
      {
        printf("[PLACE] ERROR: Missing required fields (object_pos, "
               "object_size, grip_side)\n");
        const char *err = "{\"type\":\"control\",\"status\":\"error\",\"cmd\":"
                          "\"execute_place_result\","
                          "\"error\":\"missing required fields (object_pos, object_size, grip_side)\"}\n";
        send_to_upstream_server(err, strlen(err));
      }
    }

    // ========== START DOCKING (for testing VL53L0X docking independently) ==========
#if ENABLE_DOCKING
    if (cmd_json && cJSON_IsString(cmd_json) &&
        strcmp(cmd_json->valuestring, "start_docking") == 0)
    {
      printf("[DOCKING] Received start_docking test command from laptop\n");

      if (docking_is_active())
      {
        printf("[DOCKING] Docking already active — resetting\n");
        docking_stop();
      }

      docking_start();

      const char *result =
          "{\"type\":\"control\",\"status\":\"docking_started\","
          "\"cmd\":\"start_docking_result\"}\n";
      send_to_upstream_server(result, strlen(result));
    }

    // Stop docking test
    if (cmd_json && cJSON_IsString(cmd_json) &&
        strcmp(cmd_json->valuestring, "stop_docking") == 0)
    {
      printf("[DOCKING] Received stop_docking command from laptop\n");
      docking_stop();

      const char *result =
          "{\"type\":\"control\",\"status\":\"docking_stopped\","
          "\"cmd\":\"stop_docking_result\"}\n";
      send_to_upstream_server(result, strlen(result));
    }
#endif // ENABLE_DOCKING
  }
  cJSON_Delete(json);
}