#include "cJSON.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "nvs_flash.h"
#include <arpa/inet.h>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <new>

// #include "6dof-kinematic.h"

// Kinematics removed - server handles all math

static const char *TAG = "ROBOT_TCP";

// ===================== WiFi Configuration =====================
#define WIFI_SSID "A10.14"       // Thay đổi SSID của bạn
#define WIFI_PASS "MMNT2004"     // Thay đổi password của bạn
#define SERVER_IP "192.168.1.17" // IP của laptop chạy server.py
#define SERVER_PORT 8888         // Port của server

static int g_server_sock = -1;        // Socket kết nối đến server
static bool g_wifi_connected = false; // WiFi connection status
static bool g_got_ip = false;         // Got IP address flag

static void send_json(const cJSON *obj) {
  char *str = cJSON_PrintUnformatted(obj);
  if (str) {
    if (g_server_sock != -1) {
      // Send over TCP to server
      char buffer[2048];
      snprintf(buffer, sizeof(buffer), "%s\n", str);
      send(g_server_sock, buffer, strlen(buffer), 0);
    }

    // Always log to Serial for debugging
    ESP_LOGI(TAG, "[TX] %s", str);
    fflush(stdout);
    cJSON_free(str);
  }
}

static void reply_error(const char *type, const char *msg) {
  cJSON *root = cJSON_CreateObject();
  cJSON_AddBoolToObject(root, "ok", false);
  if (type)
    cJSON_AddStringToObject(root, "type", type);
  if (msg)
    cJSON_AddStringToObject(root, "error", msg);
  send_json(root);
  cJSON_Delete(root);
}

// ===================== PCA9685 Servo Control =====================
static bool g_i2c_inited = false;
static i2c_master_bus_handle_t g_i2c_bus = nullptr;
static i2c_master_dev_handle_t g_pca_dev = nullptr;
static bool g_pca_ready = false;
static uint16_t g_servo_freq = 50; // Hz

// Giới hạn an toàn cho từng khớp (degrees)
struct JointLimit {
  float min;
  float max;
  float home;
  uint16_t min_us;
  uint16_t max_us;
};

static JointLimit g_joint_limits[6] = {
    {0, 180, 90, 400, 2700},   // J0
    {0, 180, 90, 400, 2700},   // J1
    {45, 180, 135, 400, 2700}, // J2
    {20, 130, 90, 400, 2700},  // J3
    {0, 180, 90, 400, 2700},   // J4
    {0, 90, 90, 400, 2700}     // J5
};

// is_safe_angle removed - server handles all validation via joint_limits.json

static esp_err_t i2c_init(void) {
  if (g_i2c_inited)
    return ESP_OK;
  // Create I2C master bus and attach PCA9685 device using new API (IDF v5)
  i2c_master_bus_config_t bus_cfg = {};
  bus_cfg.i2c_port = I2C_NUM_0;
  bus_cfg.sda_io_num = (gpio_num_t)21;
  bus_cfg.scl_io_num = (gpio_num_t)22;
  bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
  bus_cfg.glitch_ignore_cnt = 7; // optional debounce for noisy lines

  esp_err_t ret = i2c_new_master_bus(&bus_cfg, &g_i2c_bus);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "I2C bus init failed: 0x%x", ret);
    return ret;
  }

  i2c_device_config_t dev_cfg = {};
  dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  dev_cfg.device_address = 0x40;
  dev_cfg.scl_speed_hz = 400000;
  ret = i2c_master_bus_add_device(g_i2c_bus, &dev_cfg, &g_pca_dev);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "PCA9685 device add failed: 0x%x", ret);
    return ret;
  }
  g_i2c_inited = true;
  return ESP_OK;
}

static inline esp_err_t pca_write_reg(uint8_t reg, uint8_t val) {
  uint8_t data[2] = {reg, val};
  return i2c_master_transmit(g_pca_dev, data, sizeof(data), pdMS_TO_TICKS(100));
}
static inline esp_err_t pca_read_reg(uint8_t reg, uint8_t *out) {
  return i2c_master_transmit_receive(g_pca_dev, &reg, 1, out, 1,
                                     pdMS_TO_TICKS(100));
}

static esp_err_t pca_reset(void) {
  esp_err_t ret;
  ret = pca_write_reg(0x00, 0x20); // MODE1 AI
  if (ret != ESP_OK)
    return ret;
  ret = pca_write_reg(0x01, 0x04); // MODE2 OUTDRV
  if (ret != ESP_OK)
    return ret;
  vTaskDelay(pdMS_TO_TICKS(10));
  g_pca_ready = true;
  return ESP_OK;
}

static esp_err_t pca_set_pwm_freq(uint16_t freq_hz) {
  esp_err_t ret;
  if (!g_pca_ready) {
    ret = pca_reset();
    if (ret != ESP_OK)
      return ret;
  }
  float prescale_f = 25000000.0f / (4096.0f * (float)freq_hz) - 1.0f;
  uint8_t prescale = (uint8_t)(prescale_f + 0.5f);
  uint8_t oldmode = 0;
  ret = pca_read_reg(0x00, &oldmode);
  if (ret != ESP_OK)
    return ret;
  uint8_t sleepmode = (uint8_t)((oldmode & ~0x80) | 0x10);
  ret = pca_write_reg(0x00, sleepmode);
  if (ret != ESP_OK)
    return ret;
  ret = pca_write_reg(0xFE, prescale);
  if (ret != ESP_OK)
    return ret;
  ret = pca_write_reg(0x00, oldmode);
  if (ret != ESP_OK)
    return ret;
  vTaskDelay(pdMS_TO_TICKS(5));
  ret = pca_write_reg(0x00, (uint8_t)(oldmode | 0x80 | 0x20));
  return ret;
}

static inline uint16_t micros_to_ticks(uint16_t us, uint16_t freq_hz) {
  float period_us = 1000000.0f / (float)freq_hz;
  float ticks = (us / period_us) * 4096.0f;
  if (ticks < 0) {
    ticks = 0;
  }
  if (ticks > 4095) {
    ticks = 4095;
  }
  return (uint16_t)(ticks + 0.5f);
}

static esp_err_t pca_set_channel_pwm(uint8_t ch, uint16_t on_tick,
                                     uint16_t off_tick) {
  uint8_t reg = (uint8_t)(0x06 + 4 * ch);
  uint8_t buf[5];
  buf[0] = reg;
  buf[1] = (uint8_t)(on_tick & 0xFF);
  buf[2] = (uint8_t)((on_tick >> 8) & 0x0F);
  buf[3] = (uint8_t)(off_tick & 0xFF);
  buf[4] = (uint8_t)((off_tick >> 8) & 0x0F);
  return i2c_master_transmit(g_pca_dev, buf, sizeof(buf), pdMS_TO_TICKS(100));
}

static esp_err_t servo_write_us(uint8_t ch, uint16_t pulse_us,
                                uint16_t freq_hz) {
  uint16_t ticks = micros_to_ticks(pulse_us, freq_hz);
  uint16_t on = (uint16_t)((ch * (4096 / 16)) & 0x0FFF);
  uint16_t off = (uint16_t)((on + ticks) & 0x0FFF);
  return pca_set_channel_pwm(ch, on, off);
}

static esp_err_t servo_write_angle(uint8_t ch, float deg, uint16_t freq_hz,
                                   uint16_t us_min = 400,
                                   uint16_t us_max = 2700) {
  // No clamping - server handles all validation via joint_limits.json
  // Just convert angle to pulse width and send to servo
  float us = (float)us_min + ((float)us_max - (float)us_min) * (deg / 180.0f);
  return servo_write_us(ch, (uint16_t)(us + 0.5f), freq_hz);
}

// ===================== Smooth Servo Control (FreeRTOS Task)
// =====================
struct ServoTarget {
  uint8_t ch;
  float target_deg;
};

static QueueHandle_t g_servo_queue = nullptr;
static float g_current_pos[6] = {90, 90, 90,
                                 90, 90, 90}; // Track current positions
static bool g_smooth_enabled = true;
static float g_step_size = 2.0f;       // Degrees per step
static uint16_t g_step_delay_ms = 200; // Delay between steps

static void servo_control_task(void *arg) {
  ServoTarget target;
  ESP_LOGI(TAG, "Servo control task started");

  while (true) {
    // Wait for new target from queue
    if (xQueueReceive(g_servo_queue, &target, portMAX_DELAY) == pdTRUE) {
      uint8_t ch = target.ch;
      float target_deg = target.target_deg;

      if (ch > 5) {
        ESP_LOGW(TAG, "Invalid channel %d", ch);
        continue;
      }

      // Get pulse width limits
      uint16_t us_min = g_joint_limits[ch].min_us;
      uint16_t us_max = g_joint_limits[ch].max_us;

      if (g_smooth_enabled) {
        // Smooth interpolation
        float current = g_current_pos[ch];
        float diff = target_deg - current;

        if (fabs(diff) < 0.5f) {
          // Close enough, just set directly
          servo_write_angle(ch, target_deg, g_servo_freq, us_min, us_max);
          g_current_pos[ch] = target_deg;
        } else {
          // Interpolate step by step
          int steps = (int)(fabs(diff) / g_step_size);
          if (steps < 1)
            steps = 1;

          ESP_LOGI(TAG, "CH%d: %.1f° -> %.1f° (smooth %d steps)", ch, current,
                   target_deg, steps);

          for (int i = 0; i <= steps; i++) {
            float progress = (float)i / (float)steps;
            float angle = current + (diff * progress);

            servo_write_angle(ch, angle, g_servo_freq, us_min, us_max);
            g_current_pos[ch] = angle;

            if (i < steps) {
              vTaskDelay(pdMS_TO_TICKS(g_step_delay_ms));
            }
          }

          // Final position
          servo_write_angle(ch, target_deg, g_servo_freq, us_min, us_max);
          g_current_pos[ch] = target_deg;
        }
      } else {
        // Direct movement
        servo_write_angle(ch, target_deg, g_servo_freq, us_min, us_max);
        g_current_pos[ch] = target_deg;
      }
    }
  }
}

// ===================== WiFi & TCP Server =====================
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    ESP_LOGI(TAG, "WiFi STA started, connecting...");
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    g_wifi_connected = false;
    g_got_ip = false;
    ESP_LOGW(TAG, "WiFi disconnected, reconnecting...");
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
    g_wifi_connected = true;
    ESP_LOGI(TAG, "WiFi connected to AP, waiting for IP...");
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    g_got_ip = true;
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    ESP_LOGI(TAG, "Gateway: " IPSTR, IP2STR(&event->ip_info.gw));
    ESP_LOGI(TAG, "Netmask: " IPSTR, IP2STR(&event->ip_info.netmask));
    ESP_LOGI(TAG, "========================================");
  }
}

static void wifi_init_sta(void) {
  esp_event_handler_instance_t instance_got_ip;
  nvs_flash_init();
  esp_netif_init();
  esp_event_loop_create_default();
  esp_netif_create_default_wifi_sta();

  // Use DHCP (dynamic IP)
  // No static IP configuration needed

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);

  esp_event_handler_instance_t instance_any_id;
  esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                      &wifi_event_handler, NULL,
                                      &instance_any_id);
  esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                      &wifi_event_handler, NULL,
                                      &instance_got_ip);

  wifi_config_t wifi_config = {};
  strcpy((char *)wifi_config.sta.ssid, WIFI_SSID);
  strcpy((char *)wifi_config.sta.password, WIFI_PASS);
  wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
  esp_wifi_start();

  ESP_LOGI(TAG, "WiFi init finished. Connecting to %s...", WIFI_SSID);
}

// ===================== Socket Task (Forward Declaration) =====================
static SemaphoreHandle_t g_sock_mutex = nullptr;
static void socket_task(void *arg);

// ===================== Command Handlers =====================
static void handle_servo_init(const cJSON *root) {
  const cJSON *f = cJSON_GetObjectItemCaseSensitive(root, "freq");
  uint16_t freq =
      (uint16_t)((f && cJSON_IsNumber(f)) ? (int)f->valuedouble : 50);
  if (freq < 10) {
    freq = 10;
  }
  if (freq > 400) {
    freq = 400;
  }
  if (i2c_init() != ESP_OK) {
    ESP_LOGE(TAG, "I2C Init Failed");
    reply_error("servo_init", "i2c init failed");
    return;
  }
  ESP_LOGI(TAG, "I2C Init OK");

  if (pca_reset() != ESP_OK) {
    ESP_LOGE(TAG, "PCA Reset Failed");
    reply_error("servo_init", "pca reset failed");
    return;
  }
  ESP_LOGI(TAG, "PCA Reset OK");

  if (pca_set_pwm_freq(freq) != ESP_OK) {
    ESP_LOGE(TAG, "Set Freq Failed");
    reply_error("servo_init", "set freq failed");
    return;
  }
  g_servo_freq = freq;
  ESP_LOGI(TAG, "Servo Init Complete. Freq: %d", freq);
  cJSON *resp = cJSON_CreateObject();
  cJSON_AddBoolToObject(resp, "ok", true);
  cJSON_AddStringToObject(resp, "type", "servo_init");
  cJSON_AddNumberToObject(resp, "freq", g_servo_freq);
  send_json(resp);
  cJSON_Delete(resp);
}

static void handle_servo(const cJSON *root) {
  const cJSON *chJ = cJSON_GetObjectItemCaseSensitive(root, "ch");
  const cJSON *degJ = cJSON_GetObjectItemCaseSensitive(root, "deg");
  if (!cJSON_IsNumber(chJ) || !cJSON_IsNumber(degJ)) {
    reply_error("servo", "ch/deg required");
    return;
  }
  uint8_t ch = (uint8_t)chJ->valuedouble;
  float deg = (float)degJ->valuedouble;
  if (ch > 15) {
    reply_error("servo", "ch must be 0-15");
    return;
  }

  // No angle validation - server handles all limits via joint_limits.json
  if (!g_pca_ready) {
    reply_error("servo", "not inited");
    return;
  }

  // Send to servo control task via queue
  if (ch < 6 && g_servo_queue) {
    ServoTarget target;
    target.ch = ch;
    target.target_deg = deg;

    if (xQueueSend(g_servo_queue, &target, pdMS_TO_TICKS(100)) != pdTRUE) {
      ESP_LOGW(TAG, "Servo queue full, dropping command");
      reply_error("servo", "queue full");
      return;
    }

    // Send immediate reply
    cJSON *resp = cJSON_CreateObject();
    cJSON_AddBoolToObject(resp, "ok", true);
    cJSON_AddStringToObject(resp, "type", "servo");
    cJSON_AddNumberToObject(resp, "ch", ch);
    cJSON_AddNumberToObject(resp, "deg", deg);
    send_json(resp);
    cJSON_Delete(resp);
  } else {
    // Channels 6-15: direct control (no smooth)
    uint16_t us_min = 400;
    uint16_t us_max = 2700;

    esp_err_t ret = servo_write_angle(ch, deg, g_servo_freq, us_min, us_max);
    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "Servo write failed (ch=%d, deg=%.1f)", ch, deg);
      reply_error("servo", "write failed");
      return;
    }

    cJSON *resp = cJSON_CreateObject();
    cJSON_AddBoolToObject(resp, "ok", true);
    cJSON_AddStringToObject(resp, "type", "servo");
    cJSON_AddNumberToObject(resp, "ch", ch);
    cJSON_AddNumberToObject(resp, "deg", deg);
    send_json(resp);
    cJSON_Delete(resp);
  }
}

static void handle_servo_us(const cJSON *root) {
  const cJSON *chJ = cJSON_GetObjectItemCaseSensitive(root, "ch");
  const cJSON *usJ = cJSON_GetObjectItemCaseSensitive(root, "us");
  if (!cJSON_IsNumber(chJ) || !cJSON_IsNumber(usJ)) {
    reply_error("servo_us", "ch/us required");
    return;
  }
  uint8_t ch = (uint8_t)chJ->valuedouble;
  uint16_t us = (uint16_t)usJ->valuedouble;
  if (ch > 15) {
    reply_error("servo_us", "ch must be 0-15");
    return;
  } // PCA9685 có 16 channel (0-15)
  if (ch > 15) {
    reply_error("servo_us", "ch must be 0-15");
    return;
  } // PCA9685 có 16 channel (0-15)

  // Use configured limits for J0-J5, default 400-2700 for others
  uint16_t limit_min = (ch < 6) ? g_joint_limits[ch].min_us : 400;
  uint16_t limit_max = (ch < 6) ? g_joint_limits[ch].max_us : 2700;

  if (us < limit_min || us > limit_max) {
    char err[64];
    snprintf(err, sizeof(err), "us must be %d-%d", limit_min, limit_max);
    reply_error("servo_us", err);
    return;
  }
  if (!g_pca_ready) {
    reply_error("servo_us", "not inited");
    return;
  }

  // Retry logic for I2C reliability (max 3 attempts)
  esp_err_t ret = ESP_FAIL;
  for (int retry = 0; retry < 3; retry++) {
    ret = servo_write_us(ch, us, g_servo_freq);
    if (ret == ESP_OK)
      break;
    vTaskDelay(pdMS_TO_TICKS(5)); // Wait 5ms before retry
  }
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Servo_us write failed after 3 retries (ch=%d, us=%d)", ch,
             us);
    reply_error("servo_us", "write failed");
    return;
  }
  cJSON *resp = cJSON_CreateObject();
  cJSON_AddBoolToObject(resp, "ok", true);
  cJSON_AddStringToObject(resp, "type", "servo_us");
  cJSON_AddNumberToObject(resp, "ch", ch);
  cJSON_AddNumberToObject(resp, "us", us);
  send_json(resp);
  cJSON_Delete(resp);
}

static void handle_home(const cJSON *root) {
  if (!g_pca_ready) {
    reply_error("home", "not inited");
    return;
  }

  ESP_LOGI(TAG, "Moving to home position...");

  // Di chuyển tất cả servo về vị trí home
  for (uint8_t ch = 0; ch < 6; ch++) {
    float home_angle = g_joint_limits[ch].home;

    esp_err_t ret = ESP_FAIL;
    for (int retry = 0; retry < 3; retry++) {
      ret = servo_write_angle(ch, home_angle, g_servo_freq);
      if (ret == ESP_OK)
        break;
      vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "Failed to move J%d to home position", ch);
      reply_error("home", "move failed");
      return;
    }

    vTaskDelay(pdMS_TO_TICKS(200)); // Delay 200ms giữa các servo
  }

  cJSON *resp = cJSON_CreateObject();
  cJSON_AddBoolToObject(resp, "ok", true);
  cJSON_AddStringToObject(resp, "type", "home");
  cJSON *positions = cJSON_CreateArray();
  for (int i = 0; i < 6; i++) {
    cJSON_AddItemToArray(positions, cJSON_CreateNumber(g_joint_limits[i].home));
  }
  cJSON_AddItemToObject(resp, "positions", positions);
  send_json(resp);
  cJSON_Delete(resp);

  ESP_LOGI(TAG, "Home position reached");
}

// Helper functions parse_float and parse_joints removed
// Handler handle_set_config removed

static void handle_set_limits(const cJSON *root) {
  const cJSON *limits = cJSON_GetObjectItemCaseSensitive(root, "limits");
  if (!cJSON_IsArray(limits)) {
    reply_error("set_limits", "limits must be array");
    return;
  }

  int count = cJSON_GetArraySize(limits);
  for (int i = 0; i < count; i++) {
    cJSON *item = cJSON_GetArrayItem(limits, i);
    cJSON *chJ = cJSON_GetObjectItemCaseSensitive(item, "ch");
    cJSON *minUsJ = cJSON_GetObjectItemCaseSensitive(item, "min_us");
    cJSON *maxUsJ = cJSON_GetObjectItemCaseSensitive(item, "max_us");

    if (chJ && cJSON_IsNumber(chJ)) {
      int ch = (int)chJ->valuedouble;
      if (ch >= 0 && ch < 6) {
        if (minUsJ && cJSON_IsNumber(minUsJ))
          g_joint_limits[ch].min_us = (uint16_t)minUsJ->valuedouble;
        if (maxUsJ && cJSON_IsNumber(maxUsJ))
          g_joint_limits[ch].max_us = (uint16_t)maxUsJ->valuedouble;
      }
    }
  }

  cJSON *resp = cJSON_CreateObject();
  cJSON_AddBoolToObject(resp, "ok", true);
  cJSON_AddStringToObject(resp, "type", "set_limits");
  send_json(resp);
  cJSON_Delete(resp);
}

// Data handlers handle_fk and handle_ik removed

// ===================== Socket Task Implementation =====================
static void socket_task(void *arg) {
  ESP_LOGI(TAG, "[SOCKET-TASK] Started");

  while (true) {
    ESP_LOGI(TAG, "[SOCKET] Connecting to %s:%d...", SERVER_IP, SERVER_PORT);

    // Create TCP Client Socket
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (sock < 0) {
      ESP_LOGE(TAG, "[SOCKET] Failed to create socket: errno %d", errno);
      vTaskDelay(pdMS_TO_TICKS(5000));
      continue;
    }

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr);

    int err =
        connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr));
    if (err != 0) {
      ESP_LOGE(TAG, "[SOCKET] Connect failed: errno %d", errno);
      shutdown(sock, 0);
      close(sock);
      vTaskDelay(pdMS_TO_TICKS(5000));
      continue;
    }

    ESP_LOGI(TAG, "[SOCKET] Connected successfully!");

    // Update global socket (with mutex)
    if (xSemaphoreTake(g_sock_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
      g_server_sock = sock;
      xSemaphoreGive(g_sock_mutex);
    }

    // Process commands from server
    char line[1024];
    int pos = 0;
    uint32_t last_cmd_time = 0;

    while (true) {
      char ch;
      int len = recv(sock, &ch, 1, 0);

      if (len < 0) {
        ESP_LOGE(TAG, "[SOCKET] recv failed: errno %d", errno);
        break;
      } else if (len == 0) {
        ESP_LOGI(TAG, "[SOCKET] Server disconnected");
        break;
      }

      if (ch == '\n' || ch == '\r') {
        if (pos <= 0)
          continue;
        line[pos] = 0;
        pos = 0;

        // Rate limiting
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        if (now - last_cmd_time < 10) {
          reply_error(nullptr, "rate limit");
          continue;
        }
        last_cmd_time = now;

        // Check heap
        size_t free_heap = esp_get_free_heap_size();
        if (free_heap < 8192) {
          reply_error(nullptr, "low memory");
          ESP_LOGW(TAG, "[SOCKET] Low heap: %u bytes", (unsigned)free_heap);
          continue;
        }

        cJSON *root = cJSON_Parse(line);
        if (!root) {
          ESP_LOGW(TAG, "[SOCKET] Invalid JSON: %s", line);
          reply_error(nullptr, "invalid json");
          continue;
        }

        const cJSON *cmd = cJSON_GetObjectItemCaseSensitive(root, "cmd");
        if (!cJSON_IsString(cmd) || !cmd->valuestring) {
          ESP_LOGW(TAG, "[SOCKET] Missing cmd field: %s", line);
          cJSON_Delete(root);
          reply_error(nullptr, "cmd missing");
          continue;
        }

        ESP_LOGI(TAG, "[RX] Command: %s", cmd->valuestring);
        ESP_LOGI(TAG, "[RX] Full JSON: %s", line);

        if (strcmp(cmd->valuestring, "servo_init") == 0) {
          handle_servo_init(root);
        } else if (strcmp(cmd->valuestring, "servo") == 0) {
          handle_servo(root);
        } else if (strcmp(cmd->valuestring, "servo_us") == 0) {
          handle_servo_us(root);
        } else if (strcmp(cmd->valuestring, "home") == 0) {
          handle_home(root);
        } else if (strcmp(cmd->valuestring, "set_limits") == 0) {
          handle_set_limits(root);
        } else {
          reply_error(cmd->valuestring, "unknown cmd");
        }
        cJSON_Delete(root);
      } else if (pos < (int)sizeof(line) - 1) {
        line[pos++] = ch;
      } else {
        pos = 0;
        reply_error(nullptr, "line too long");
      }
    }

    // Cleanup socket
    if (xSemaphoreTake(g_sock_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
      g_server_sock = -1;
      xSemaphoreGive(g_sock_mutex);
    }

    shutdown(sock, 0);
    close(sock);
    ESP_LOGI(TAG, "[SOCKET] Connection closed. Reconnecting in 5s...");
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "=== APP START (TCP Client Mode) ===");

  // Step 1: Init WiFi
  ESP_LOGI(TAG, "Step 1: Init WiFi (DHCP)");
  wifi_init_sta();

  // Wait for IP address with timeout
  ESP_LOGI(TAG, "Waiting for IP address...");
  int wait_count = 0;
  while (!g_got_ip && wait_count < 100) { // Max 10 seconds
    vTaskDelay(pdMS_TO_TICKS(100));
    wait_count++;
    if (wait_count % 10 == 0) {
      ESP_LOGI(TAG, "Still waiting for IP... (%d/10s)", wait_count / 10);
    }
  }

  if (!g_got_ip) {
    ESP_LOGE(TAG, "Failed to get IP address after 10s, restarting...");
    esp_restart();
  }

  ESP_LOGI(TAG, "=== Initialization Complete ===");
  ESP_LOGI(TAG, "Free heap: %lu bytes",
           (unsigned long)esp_get_free_heap_size());

  // Create socket mutex
  g_sock_mutex = xSemaphoreCreateMutex();
  if (!g_sock_mutex) {
    ESP_LOGE(TAG, "Failed to create socket mutex!");
    esp_restart();
  }
  ESP_LOGI(TAG, "Socket mutex created");

  // Create servo control queue and task
  g_servo_queue = xQueueCreate(10, sizeof(ServoTarget));
  if (!g_servo_queue) {
    ESP_LOGE(TAG, "Failed to create servo queue!");
    esp_restart();
  }

  if (xTaskCreate(servo_control_task, "servo_ctrl", 4096, nullptr, 5,
                  nullptr) != pdPASS) {
    ESP_LOGE(TAG, "Failed to create servo task!");
    esp_restart();
  }
  ESP_LOGI(TAG, "Servo control task created");

  // Create socket task (handles all TCP operations)
  if (xTaskCreate(socket_task, "socket", 8192, nullptr, 4, nullptr) != pdPASS) {
    ESP_LOGE(TAG, "Failed to create socket task!");
    esp_restart();
  }
  ESP_LOGI(TAG, "Socket task created");

  ESP_LOGI(TAG, "=== All tasks running. Main task idling ===");

  // Main task done - just idle forever
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(60000)); // Sleep 1 minute
  }
}
