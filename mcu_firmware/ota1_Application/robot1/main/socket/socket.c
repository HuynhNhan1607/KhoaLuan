#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"
#include "lwip/sockets.h"
#include "wifi_handler.h"

#include "esp_ota_ops.h"
#include "esp_partition.h"

#include "bno055_handler.h"
#include "esp_timer.h"
#include "kinematic.h"
#include "log_handler.h"
#include "motion_control.h"
#include "motor_driver.h"
#include "nvs_handler.h"
#include "socket.h"
#include "sys_config.h"

static const char *TAG_Socket = "Socket";

static int socket_server = -1;

static inline float rpm_to_rads(float rpm) { return rpm * 0.10472f; }

static inline float rads_to_rpm(float rads) { return rads * 9.5493f; }

int setup_socket(const char *server_ip, int server_port) {
  struct sockaddr_in dest_addr;
  dest_addr.sin_addr.s_addr = inet_addr(server_ip);
  dest_addr.sin_family = AF_INET;
  dest_addr.sin_port = htons(server_port);

  int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
  if (sock < 0) {
    ESP_LOGE(TAG_Socket, "Unable to create socket %d", server_port);
  }

  if (connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) != 0) {
    ESP_LOGE(TAG_Socket, "Socket connection failed %d", server_port);
    sock = -1;
    close(sock);
  }
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 50000;

  setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
  ESP_LOGI(TAG_Socket, "Connected to server %d", server_port);
  return sock;
}
odometry_t odom = {0};
void task_socket(void *pvParameters) {
  int socket = *(int *)pvParameters;

  float dot_x, dot_y, dot_theta;
  dot_x = dot_y = dot_theta = 0;

  int motor_id, motor_speed = 0;

  char rx_buffer[128];
  while (1) {
    int len = recv(socket, rx_buffer, sizeof(rx_buffer) - 1, 0);
    if (len > 0) {
      rx_buffer[len] = '\0';
      // ESP_LOGI(TAG_Socket, "Received: %s", rx_buffer);
      printf("Received: %s\n", rx_buffer);
      // Switch to Upgrade Mode
      if (strcmp(rx_buffer, "Upgrade") == 0) {
        static const esp_partition_t *update_partition = NULL;
        update_partition = esp_partition_find_first(
            ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
        ESP_LOGW(TAG_Socket, "----Switch to Upgrade----");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        esp_ota_set_boot_partition(update_partition);
        esp_restart();
      } else if (sscanf(rx_buffer, "dot_x:%f dot_y:%f dot_theta:%f", &dot_x,
                        &dot_y, &dot_theta) == 3) {
        ESP_LOGW(TAG_Socket, "Received dot_x: %f, dot_y: %f, dot_theta: %f",
                 dot_x, dot_y, dot_theta);
        // MecanumSpeedControl(theta, dot_x, dot_y, dot_theta);
        set_robot_status(dot_x, dot_y, dot_theta);
      }

      else if (sscanf(rx_buffer, "Localize: X=%f Y=%f", &odom.distance[X_AXIS],
                      &odom.distance[Y_AXIS]) == 2) {
        ESP_LOGW(TAG_Socket, "Setting Position X: %.2f, Y: %.2f",
                 odom.distance[X_AXIS], odom.distance[Y_AXIS]);
      }
      // Motor Set Speed
      else if (sscanf(rx_buffer, "MOTOR:%d SPEED:%d", &motor_id,
                      &motor_speed) == 2) {
        ESP_LOGW(TAG_Socket, "Setting Motor %d Speed to %d RPM", motor_id,
                 motor_speed);
        SetSingleWheelSpeed(motor_id + 1, rpm_to_rads((float)motor_speed));
      } else if (strcmp(rx_buffer, "clear nvs") == 0) {
        ESP_LOGW(TAG_Socket, "Clearing NVS");
        nvs_clear_bno055_calibration();
      } else if (strcmp(rx_buffer, "pid_auto\n") == 0) {
        ESP_LOGW(TAG_Socket, "Starting PID Autotune with default parameters");
        motor_start_autotune();
      }
      // RPM change to Rad/s!!!
      else if (strncmp(rx_buffer, "RPM:", 4) == 0) {
        float target_rpm = 4.0f; // Default
        float relay_pwm = 60.0f; // Default

        if (sscanf(rx_buffer, "RPM:%f PWM:%f", &target_rpm, &relay_pwm) >= 1) {
          // If only RPM is provided, use default PWM
          if (sscanf(rx_buffer, "RPM:%f PWM:%f", &target_rpm, &relay_pwm) ==
              1) {
            relay_pwm = 100.0f;
          }

          ESP_LOGW(TAG_Socket,
                   "Starting PID Autotune with custom parameters: RPM=%.1f, "
                   "PWM=%.1f",
                   target_rpm, relay_pwm);
          motor_start_autotune_custom(target_rpm, relay_pwm);
        } else {
          ESP_LOGW(TAG_Socket,
                   "Invalid RPM/PWM format. Use: RPM:<value> PWM:<value>");
        }
      } else {
        // ESP_LOGW(TAG_Socket, "Unknown command: %s", rx_buffer);
      }
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
  close(socket);
  vTaskDelete(NULL);
}

#define SEND_RPM 1

int64_t start_time;
wheel_infor_t wheel_infor[4] = {0};
void send_rpm_data(int socket) {
  GetWheelInfor(wheel_infor);
  float yaw_angle = get_heading();
  convert_euler2radian(&yaw_angle);

  int64_t end_time = esp_timer_get_time();

  EstimateRobotPosition(&odom, yaw_angle, wheel_infor,
                        (end_time - start_time) / 1000000.0f);

  // ESP_LOGW(TAG_Socket, "x: %.2fm y: %.2fm", odom.distance[X_AXIS],
  // odom.distance[Y_AXIS]); Format the JSON string with robot ID and 4 motor
  // values
  char message[256];
#if SEND_RPM == 1
  int message_len = snprintf(
      message, sizeof(message),
      "{\"id\":\"%s\",\"type\":\"encoder\",\"data\":[%.2f,%.2f,%.2f,%.2f]}\n",
      ID_ROBOT, rads_to_rpm(wheel_infor[0].WheelSpeed),
      rads_to_rpm(wheel_infor[1].WheelSpeed),
      rads_to_rpm(wheel_infor[2].WheelSpeed),
      rads_to_rpm(wheel_infor[3].WheelSpeed)); // RPM

  // Send data to server
  if (socket_send(message, message_len) != ESP_OK) {
    ESP_LOGE(TAG_Socket, "%s, Send RPM Fail", __func__);
  }
#else
  int message_len;
#endif

  memset(message, 0, sizeof(message));
  message_len = snprintf(
      message, sizeof(message),
      "{\"id\":\"%s\",\"type\":\"position\",\"source\":\"odometry\",\"data\":{"
      "\"position\":[%.4f,%.4f],\"velocity\":[%.4f,%.4f]}}\n",
      ID_ROBOT, odom.distance[X_AXIS], odom.distance[Y_AXIS],
      odom.velocity[X_AXIS], odom.velocity[Y_AXIS]);

  if (socket_send(message, message_len) != ESP_OK) {
    ESP_LOGE(TAG_Socket, "%s, Send Odom Fail", __func__);
  }
}

void task_send_encoder(void *pvParameters) {
  int socket = *(int *)pvParameters;
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(20);

  xLastWakeTime = xTaskGetTickCount();
  start_time = esp_timer_get_time();

  while (1) {
    send_rpm_data(socket);
    start_time = esp_timer_get_time();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  close(socket);
  vTaskDelete(NULL);
}

void send_pid_data(void) {
  char message[512];
  float kp[4], ki[4], kd[4];

  // Get PID gains for all 4 motors
  for (int i = 0; i < 4; i++) {
    motor_get_pid_gains(i, &kp[i], &ki[i], &kd[i]);
  }

  // Format JSON message
  int message_len = snprintf(message, sizeof(message),
                             "{\"id\":\"%s\",\"type\":\"pid_data\",\"data\":{"
                             "\"motor1\":[%.3f,%.3f,%.3f],"
                             "\"motor2\":[%.3f,%.3f,%.3f],"
                             "\"motor3\":[%.3f,%.3f,%.3f],"
                             "\"motor4\":[%.3f,%.3f,%.3f]}}\n",
                             ID_ROBOT, kp[0], ki[0], kd[0], kp[1], ki[1], kd[1],
                             kp[2], ki[2], kd[2], kp[3], ki[3], kd[3]);

  // Send to server
  if (socket_send(message, message_len) != ESP_OK) {
    ESP_LOGE(TAG_Socket, "Failed to send PID data");
  } else {
    ESP_LOGI(TAG_Socket, "PID data sent successfully");
  }
}

void send_reset_reason(void) {
  const char *reset_reason_str;
  esp_reset_reason_t reset_reason = esp_reset_reason();

  switch (reset_reason) {
  case ESP_RST_UNKNOWN:
    reset_reason_str = "Unknown reset";
    break;
  case ESP_RST_POWERON:
    reset_reason_str = "Power-on reset";
    break;
  case ESP_RST_EXT:
    reset_reason_str = "External pin reset";
    break;
  case ESP_RST_SW:
    reset_reason_str = "Software reset";
    break;
  case ESP_RST_PANIC:
    reset_reason_str = "Exception/panic reset";
    break;
  case ESP_RST_INT_WDT:
    reset_reason_str = "Interrupt watchdog";
    break;
  case ESP_RST_TASK_WDT:
    reset_reason_str = "Task watchdog";
    break;
  case ESP_RST_WDT:
    reset_reason_str = "Other watchdog";
    break;
  case ESP_RST_DEEPSLEEP:
    reset_reason_str = "Deep sleep reset";
    break;
  case ESP_RST_BROWNOUT:
    reset_reason_str = "Brownout reset";
    break;
  case ESP_RST_SDIO:
    reset_reason_str = "SDIO reset";
    break;
  case ESP_RST_USB:
    reset_reason_str = "USB reset";
    break;
  case ESP_RST_JTAG:
    reset_reason_str = "JTAG reset";
    break;
  default:
    reset_reason_str = "Unknown";
    break;
  }

  ESP_LOGW(TAG_Socket, "Last reset reason: %s", reset_reason_str);
  printf("Last reset reason: %s\n", reset_reason_str);

  char message[256];
  int message_len =
      snprintf(message, sizeof(message),
               "{\"id\":\"%s\",\"type\":\"reset_info\",\"reason\":\"%s\"}\n",
               ID_ROBOT, reset_reason_str);

  if (socket_send(message, message_len) != ESP_OK) {
    ESP_LOGE(TAG_Socket, "Failed to send reset reason");
  } else {
    ESP_LOGI(TAG_Socket, "Reset reason sent successfully");
  }
}

esp_err_t socket_send(char *msg, int msg_len) {
  if (msg == NULL || msg_len <= 0) {
    return ESP_FAIL;
  }
  if (socket_server < 0) {
    return ESP_FAIL;
  }
  if (send(socket_server, msg, msg_len, 0) < 0) {
    return ESP_FAIL;
  }
  return ESP_OK;
}

esp_err_t server_init(void) {
  connect_to_wifi();
  socket_server = setup_socket(SERVER_IP, SERVER_PORT);

#if LOG_SERVER == 1
  log_init(socket_server);
#endif
  ESP_LOGI(TAG_Socket, "Server initialized");

  // Wait for connection to stabilize
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  // Send reset reason to server
  send_reset_reason();

  // Send PID data to server on startup

  return ESP_OK;
}

void server_tasks_start(void) {
  xTaskCreatePinnedToCore(task_socket, "task_socket", 4096, &socket_server, 5,
                          NULL, 1);
  xTaskCreatePinnedToCore(task_send_encoder, "task_send_encoder", 4096,
                          &socket_server, 5, NULL, 1);
}