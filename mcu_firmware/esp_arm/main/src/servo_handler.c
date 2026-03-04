#include "servo_handler.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "socket.h"
#include <math.h>
#include <string.h>
#include "sys_config.h"

/* ===================== Gravity Compensation Configuration
 * =====================
 *
 * HARDWARE CONFIGURATION (YOUR ROBOT):
 *   - Servo Angle 90°  = Arm VERTICAL (Minimum Gravity Torque)
 *   - Servo Angle 0°   = Arm Horizontal (Maximum Gravity Torque)
 *   - Servo Angle 180° = Arm Horizontal opposite (Maximum Gravity Torque)
 *
 * FORMULA:
 *   Compensated_Angle = Current_Angle + (Gain * cos(Current_Angle))
 *
 *   Because: cos(90°) = 0 (No compensation at vertical)
 *            cos(0°) = 1, cos(180°) = -1 (Max compensation at horizontal)
 *
 * TUNING:
 *   - Start with Gain = 0.0 (disabled)
 *   - Increase Gain gradually (e.g., 2.0 to 10.0) until arm holds position
 *   - If compensation pushes arm wrong direction, negate the Gain value
 */
#if ROBOT_ID == 1
/* Shoulder joint - typically needs most compensation */
#define GRAVITY_GAIN_J1 8.0f
/* Elbow joint - may need less than J1 */
#define GRAVITY_GAIN_J2 3.0f
/* Wrist pitch joint - lightest load, least compensation */
#define GRAVITY_GAIN_J3 0.0f
#elif ROBOT_ID == 2
/* Shoulder joint - typically needs most compensation */
#define GRAVITY_GAIN_J1 8.0f
/* Elbow joint - may need less than J1 */
#define GRAVITY_GAIN_J2 3.0f
/* Wrist pitch joint - lightest load, least compensation */
#define GRAVITY_GAIN_J3 0.0f
#else
/* Shoulder joint - typically needs most compensation */
#define GRAVITY_GAIN_J1 8.0f
/* Elbow joint - may need less than J1 */
#define GRAVITY_GAIN_J2 3.0f
/* Wrist pitch joint - lightest load, least compensation */
#define GRAVITY_GAIN_J3 0.0f
#endif

/* Degree to Radian conversion for cos() function */
#define DEG_TO_RAD_SERVO(deg) ((deg) * 3.14159265358979323846f / 180.0f)

static const char *TAG = "SERVO";

// I2C and PCA9685 state
static bool g_i2c_inited = false;
static i2c_master_bus_handle_t g_i2c_bus = NULL;
static i2c_master_dev_handle_t g_pca_dev = NULL;

// Global variables (definitions)
bool g_pca_ready = false;
uint16_t g_servo_freq = 50;
QueueHandle_t g_servo_queue = NULL;
QueueHandle_t g_multi_servo_queue = NULL;
float g_current_pos[6] = {90, 180, 180, 180, 90, 90};
bool g_smooth_enabled = true;
float g_step_size = 2.0f;
uint16_t g_step_delay_ms = 50; // Faster for smoother S-Curve

// First move flag: skip smooth on first command (unknown initial position)
static bool g_first_move[6] = {true, true, true, true, true, true};

// Global angles from Xavier (angle of each link relative to horizontal plane)
// Used for gravity compensation calculation
float g_global_angles[6] = {0, 10, 90, 10, 90, 90};

#if ROBOT_ID == 1
JointLimit g_joint_limits[6] = {
    {0, 180, 90, 530, 2550}, // J0 - Base Rotation
    {0, 180, 90, 470, 2350}, // J1 - Shoulder Pitch
    {0, 180, 90, 570, 2450}, // J2 - Elbow
    {0, 180, 90, 450, 2400}, // J3 - Wrist Pitch
    {0, 180, 90, 400, 2700}, // J4 - Wrist Roll
    {0, 90, 90, 400, 2700}   // J5 - Gripper
};
#elif ROBOT_ID == 2
JointLimit g_joint_limits[6] = {
    {0, 180, 90, 500, 2450}, // J0 - Base Rotation
    {0, 180, 90, 350, 2350}, // J1 - Shoulder Pitch
    {0, 180, 90, 475, 2400}, // J2 - Elbow
    {0, 180, 90, 450, 2350}, // J3 - Wrist Pitch
    {0, 180, 90, 500, 2500}, // J4 - Wrist Roll
    {0, 90, 90, 500, 2500}   // J5 - Gripper
};
#else
JointLimit g_joint_limits[6] = {
    {0, 180, 90, 500, 2450}, // J0 - Base Rotation
    {0, 180, 90, 350, 2350}, // J1 - Shoulder Pitch
    {0, 180, 90, 475, 2400}, // J2 - Elbow
    {0, 180, 90, 450, 2350}, // J3 - Wrist Pitch
    {0, 180, 90, 500, 2500}, // J4 - Wrist Roll
    {0, 90, 90, 500, 2500}   // J5 - Gripper
};
#endif
// ===================== S-Curve Interpolation =====================
/**
 * @brief Smoothstep function for S-Curve interpolation
 * @param t Progress value from 0.0 to 1.0
 * @return S-Curve interpolated value (starts slow, speeds up, slows down)
 */
static inline float smoothstep(float t)
{
  // Clamp t to [0, 1]
  if (t < 0.0f)
    t = 0.0f;
  if (t > 1.0f)
    t = 1.0f;
  // Smoothstep: 3t² - 2t³
  return t * t * (3.0f - 2.0f * t);
}

// ===================== PCA9685 Low-Level Functions =====================
static inline esp_err_t pca_write_reg(uint8_t reg, uint8_t val)
{
  uint8_t data[2] = {reg, val};
  return i2c_master_transmit(g_pca_dev, data, sizeof(data), pdMS_TO_TICKS(100));
}

static inline esp_err_t pca_read_reg(uint8_t reg, uint8_t *out)
{
  return i2c_master_transmit_receive(g_pca_dev, &reg, 1, out, 1,
                                     pdMS_TO_TICKS(100));
}

static inline uint16_t micros_to_ticks(uint16_t us, uint16_t freq_hz)
{
  float period_us = 1000000.0f / (float)freq_hz;
  float ticks = (us / period_us) * 4096.0f;
  if (ticks < 0)
  {
    ticks = 0;
  }
  if (ticks > 4095)
  {
    ticks = 4095;
  }
  return (uint16_t)(ticks + 0.5f);
}

static esp_err_t pca_set_channel_pwm(uint8_t ch, uint16_t on_tick,
                                     uint16_t off_tick)
{
  uint8_t reg = (uint8_t)(0x06 + 4 * ch);
  uint8_t buf[5];
  buf[0] = reg;
  buf[1] = (uint8_t)(on_tick & 0xFF);
  buf[2] = (uint8_t)((on_tick >> 8) & 0x0F);
  buf[3] = (uint8_t)(off_tick & 0xFF);
  buf[4] = (uint8_t)((off_tick >> 8) & 0x0F);
  return i2c_master_transmit(g_pca_dev, buf, sizeof(buf), pdMS_TO_TICKS(100));
}

// ===================== Public I2C/PCA Functions =====================
esp_err_t i2c_init(void)
{
  if (g_i2c_inited)
    return ESP_OK;

  i2c_master_bus_config_t bus_cfg = {0};
  bus_cfg.i2c_port = I2C_NUM_0;
  bus_cfg.sda_io_num = (gpio_num_t)4;
  bus_cfg.scl_io_num = (gpio_num_t)5;
  bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
  bus_cfg.glitch_ignore_cnt = 7;

  esp_err_t ret = i2c_new_master_bus(&bus_cfg, &g_i2c_bus);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "I2C bus init failed: 0x%x", ret);
    return ret;
  }

  i2c_device_config_t dev_cfg = {0};
  dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  dev_cfg.device_address = 0x40;
  dev_cfg.scl_speed_hz = 400000;
  ret = i2c_master_bus_add_device(g_i2c_bus, &dev_cfg, &g_pca_dev);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "PCA9685 device add failed: 0x%x", ret);
    return ret;
  }
  g_i2c_inited = true;
  return ESP_OK;
}

esp_err_t pca_reset(void)
{
  esp_err_t ret;
  ret = pca_write_reg(0x00, 0x20);
  if (ret != ESP_OK)
    return ret;
  ret = pca_write_reg(0x01, 0x04);
  if (ret != ESP_OK)
    return ret;
  vTaskDelay(pdMS_TO_TICKS(10));
  g_pca_ready = true;
  return ESP_OK;
}

esp_err_t pca_set_pwm_freq(uint16_t freq_hz)
{
  esp_err_t ret;
  if (!g_pca_ready)
  {
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

esp_err_t servo_write_us(uint8_t ch, uint16_t pulse_us, uint16_t freq_hz)
{
  uint16_t ticks = micros_to_ticks(pulse_us, freq_hz);
  uint16_t on = (uint16_t)((ch * (4096 / 16)) & 0x0FFF);
  uint16_t off = (uint16_t)((on + ticks) & 0x0FFF);
  return pca_set_channel_pwm(ch, on, off);
}

esp_err_t servo_write_angle(uint8_t ch, float deg, uint16_t freq_hz,
                            uint16_t us_min, uint16_t us_max)
{
  float us = (float)us_min + ((float)us_max - (float)us_min) * (deg / 180.0f);
  return servo_write_us(ch, (uint16_t)(us + 0.5f), freq_hz);
}

// ===================== Servo Auto Init =====================
esp_err_t servo_auto_init(void)
{
  ESP_LOGI(TAG, "Auto-initializing servo with 50Hz...");

  esp_err_t ret = i2c_init();
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "I2C Init Failed: 0x%x", ret);
    return ret;
  }
  ESP_LOGI(TAG, "I2C Init OK");

  ret = pca_reset();
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "PCA Reset Failed: 0x%x", ret);
    return ret;
  }
  ESP_LOGI(TAG, "PCA Reset OK");

  ret = pca_set_pwm_freq(50);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Set Freq Failed: 0x%x", ret);
    return ret;
  }
  g_servo_freq = 50;
  ESP_LOGI(TAG, "Servo Auto Init Complete. Freq: 50Hz");

  return ESP_OK;
}

// ===================== Gravity Compensation Helper =====================
/**
 * @brief Apply gravity compensation using global angles from Xavier.
 *
 * Xavier sends global angles (relative to horizontal plane) via set_gravity
 * command. These are stored in g_global_angles[] and used for compensation.
 *
 * CONVENTION (from Xavier):
 *   - 0° = horizontal (link parallel to ground) → MAX gravity torque
 *   - 90° = vertical up → NO gravity torque
 *   - -90° = vertical down → NO gravity torque
 *
 * FORMULA: Compensated = servo_angle + (Gain * sin(global_angle))
 *   - sin(0°) = 0 → horizontal, but we want MAX compensation here
 *   - We use cos() since cos(0°) = 1 (horizontal = max) and cos(±90°) = 0
 *
 * @param ch    Channel number (0-5)
 * @param angle Servo angle in degrees (0-180)
 * @return      Compensated servo angle in degrees
 */
static inline float apply_gravity_compensation(uint8_t ch, float angle)
{
  float gain = 0.0f;

  switch (ch)
  {
  case 1: /* J1 - Shoulder */
    gain = GRAVITY_GAIN_J1;
    break;
  case 2: /* J2 - Elbow */
    gain = GRAVITY_GAIN_J2;
    break;
  case 3: /* J3 - Wrist Pitch */
    gain = GRAVITY_GAIN_J3;
    break;
  default:
    return angle; /* No compensation for J0, J4, J5 */
  }

  if (gain == 0.0f)
  {
    return angle; /* Compensation disabled for this joint */
  }

  /* Use global angle from Xavier for compensation */
  /* cos(0°) = 1 (horizontal = max compensation) */
  /* cos(±90°) = 0 (vertical = no compensation) */
  float global_angle = g_global_angles[ch];
  float angle_rad = DEG_TO_RAD_SERVO(global_angle);
  float compensation = gain * cosf(angle_rad);
  float compensated_angle = angle + compensation;

  /* Clamp to valid servo range */
  if (compensated_angle < 0.0f)
    compensated_angle = 0.0f;
  if (compensated_angle > 180.0f)
    compensated_angle = 180.0f;

  return compensated_angle;
}

// ===================== Servo Control Task =====================
void servo_control_task(void *arg)
{
  ServoTarget target;
  ESP_LOGI(TAG, "Servo control task started (S-Curve enabled, delay=%dms)",
           g_step_delay_ms);

  while (true)
  {
    if (xQueueReceive(g_servo_queue, &target, portMAX_DELAY) == pdTRUE)
    {
      uint8_t ch = target.ch;
      float target_deg = target.target_deg;

      if (ch > 5)
      {
        ESP_LOGW(TAG, "Invalid channel %d", ch);
        continue;
      }

      uint16_t us_min = g_joint_limits[ch].min_us;
      uint16_t us_max = g_joint_limits[ch].max_us;

      /* Log target angle from server */
      // float final_compensated = apply_gravity_compensation(ch, target_deg);
      // if (ch >= 1 && ch <= 3)
      // {
      //   ESP_LOGI(TAG, "[GRAVITY] J%d: Server=%.1f° -> Compensated=%.1f°", ch,
      //            target_deg, final_compensated);
      // }

      /* FIRST MOVE: Skip smooth, go directly to target (unknown initial pos) */
      if (g_first_move[ch])
      {
        ESP_LOGI(TAG, "CH%d: First move -> %.1f° (direct, no smooth)", ch,
                 target_deg);
        float compensated = apply_gravity_compensation(ch, target_deg);
        servo_write_angle(ch, compensated, g_servo_freq, us_min, us_max);
        g_current_pos[ch] = target_deg;
        g_first_move[ch] = false;
        continue;
      }

      if (g_smooth_enabled)
      {
        float current = g_current_pos[ch];
        float diff = target_deg - current;

        if (fabs(diff) < 0.5f)
        {
          /* Already at target */
          float compensated = apply_gravity_compensation(ch, target_deg);
          servo_write_angle(ch, compensated, g_servo_freq, us_min, us_max);
          g_current_pos[ch] = target_deg;
        }
        else
        {
          /* S-CURVE SMOOTH MOVEMENT */
          int steps = (int)(fabs(diff) / g_step_size);
          if (steps < 1)
            steps = 1;

          ESP_LOGI(TAG, "CH%d: %.1f° -> %.1f° (S-Curve %d steps)", ch, current,
                   target_deg, steps);

          for (int i = 0; i <= steps; i++)
          {
            float t = (float)i / (float)steps;
            float s = smoothstep(t); // S-Curve interpolation
            float angle = current + (diff * s);

            /* Apply gravity compensation */
            float compensated = apply_gravity_compensation(ch, angle);
            servo_write_angle(ch, compensated, g_servo_freq, us_min, us_max);
            g_current_pos[ch] = angle;

            if (i < steps)
            {
              vTaskDelay(pdMS_TO_TICKS(g_step_delay_ms));
            }
          }

          /* Ensure final position is exact */
          float final_comp = apply_gravity_compensation(ch, target_deg);
          servo_write_angle(ch, final_comp, g_servo_freq, us_min, us_max);
          g_current_pos[ch] = target_deg;
        }
      }
      else
      {
        /* Non-smooth mode */
        float compensated = apply_gravity_compensation(ch, target_deg);
        servo_write_angle(ch, compensated, g_servo_freq, us_min, us_max);
        g_current_pos[ch] = target_deg;
      }

      // Log current pose summary
      ESP_LOGI(TAG, "[POSE] J0:%.1f J1:%.1f J2:%.1f J3:%.1f J4:%.1f J5:%.1f",
               g_current_pos[0], g_current_pos[1], g_current_pos[2],
               g_current_pos[3], g_current_pos[4], g_current_pos[5]);
    }
  }
}

// ===================== Multi-Servo Control Task =====================
// Moves multiple servos simultaneously with synchronized S-Curve interpolation
void multi_servo_control_task(void *arg)
{
  MultiServoTarget multi;
  ESP_LOGI(TAG, "Multi-servo control task started (synchronized S-Curve)");

  while (true)
  {
    if (xQueueReceive(g_multi_servo_queue, &multi, portMAX_DELAY) == pdTRUE)
    {
      if (multi.count == 0 || multi.count > MAX_MULTI_SERVO)
      {
        ESP_LOGW(TAG, "Invalid multi-servo count: %d", multi.count);
        continue;
      }

      ESP_LOGI(TAG, "[MULTI] Moving %d servos simultaneously", multi.count);

      // Log target angles and gravity compensation for joints 1-3
      for (int i = 0; i < multi.count; i++)
      {
        uint8_t ch = multi.ch[i];
        if (ch >= 1 && ch <= 3)
        {
          float target_deg = multi.target_deg[i];
          float compensated = apply_gravity_compensation(ch, target_deg);
          ESP_LOGI(TAG,
                   "[MULTI-GRAVITY] J%d: Server=%.1f° -> Compensated=%.1f°", ch,
                   target_deg, compensated);
        }
      }

      // Calculate max steps needed (based on largest movement)
      int max_steps = 1;
      float diffs[MAX_MULTI_SERVO] = {0};
      float starts[MAX_MULTI_SERVO] = {0};

      for (int i = 0; i < multi.count; i++)
      {
        uint8_t ch = multi.ch[i];
        if (ch > 5)
          continue;

        starts[i] = g_current_pos[ch];
        diffs[i] = multi.target_deg[i] - starts[i];

        // Handle first move - use target as start (skip smooth for unknown pos)
        if (g_first_move[ch])
        {
          starts[i] = multi.target_deg[i];
          diffs[i] = 0;
          g_first_move[ch] = false;
        }

        int steps = (int)(fabs(diffs[i]) / g_step_size);
        if (steps > max_steps)
          max_steps = steps;
      }

      if (g_smooth_enabled && max_steps > 0)
      {
        // S-CURVE SYNCHRONIZED MOVEMENT
        for (int step = 0; step <= max_steps; step++)
        {
          float t = (float)step / (float)max_steps;
          float s = smoothstep(t);

          // Move all servos to interpolated position
          for (int i = 0; i < multi.count; i++)
          {
            uint8_t ch = multi.ch[i];
            if (ch > 5)
              continue;

            float angle = starts[i] + (diffs[i] * s);
            float compensated = apply_gravity_compensation(ch, angle);

            uint16_t us_min = g_joint_limits[ch].min_us;
            uint16_t us_max = g_joint_limits[ch].max_us;
            servo_write_angle(ch, compensated, g_servo_freq, us_min, us_max);
            g_current_pos[ch] = angle;
          }

          if (step < max_steps)
          {
            vTaskDelay(pdMS_TO_TICKS(g_step_delay_ms));
          }
        }

        // Ensure final positions are exact
        for (int i = 0; i < multi.count; i++)
        {
          uint8_t ch = multi.ch[i];
          if (ch > 5)
            continue;

          float final_comp =
              apply_gravity_compensation(ch, multi.target_deg[i]);
          uint16_t us_min = g_joint_limits[ch].min_us;
          uint16_t us_max = g_joint_limits[ch].max_us;
          servo_write_angle(ch, final_comp, g_servo_freq, us_min, us_max);
          g_current_pos[ch] = multi.target_deg[i];
        }
      }
      else
      {
        // Non-smooth mode - move all immediately
        for (int i = 0; i < multi.count; i++)
        {
          uint8_t ch = multi.ch[i];
          if (ch > 5)
            continue;

          float compensated =
              apply_gravity_compensation(ch, multi.target_deg[i]);
          uint16_t us_min = g_joint_limits[ch].min_us;
          uint16_t us_max = g_joint_limits[ch].max_us;
          servo_write_angle(ch, compensated, g_servo_freq, us_min, us_max);
          g_current_pos[ch] = multi.target_deg[i];
        }
      }

      // Log current pose summary
      ESP_LOGI(TAG, "[POSE] J0:%.1f J1:%.1f J2:%.1f J3:%.1f J4:%.1f J5:%.1f",
               g_current_pos[0], g_current_pos[1], g_current_pos[2],
               g_current_pos[3], g_current_pos[4], g_current_pos[5]);
    }
  }
}

// ===================== Command Handlers =====================
void handle_servo_init(const cJSON *root)
{
  const cJSON *f = cJSON_GetObjectItemCaseSensitive(root, "freq");
  uint16_t freq =
      (uint16_t)((f && cJSON_IsNumber(f)) ? (int)f->valuedouble : 50);
  if (freq < 10)
  {
    freq = 10;
  }
  if (freq > 400)
  {
    freq = 400;
  }
  if (i2c_init() != ESP_OK)
  {
    ESP_LOGE(TAG, "I2C Init Failed");
    reply_error("servo_init", "i2c init failed");
    return;
  }
  ESP_LOGI(TAG, "I2C Init OK");

  if (pca_reset() != ESP_OK)
  {
    ESP_LOGE(TAG, "PCA Reset Failed");
    reply_error("servo_init", "pca reset failed");
    return;
  }
  ESP_LOGI(TAG, "PCA Reset OK");

  if (pca_set_pwm_freq(freq) != ESP_OK)
  {
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

void handle_servo(const cJSON *root)
{
  const cJSON *chJ = cJSON_GetObjectItemCaseSensitive(root, "ch");
  const cJSON *degJ = cJSON_GetObjectItemCaseSensitive(root, "deg");

  if (!chJ || !degJ)
  {
    reply_error("servo", "ch/deg required");
    return;
  }

  if (!g_pca_ready)
  {
    reply_error("servo", "not inited");
    return;
  }

  // Check if it's array format (multi-servo) or single value
  bool is_array = cJSON_IsArray(chJ) && cJSON_IsArray(degJ);

  if (is_array)
  {
    // ===== MULTI-SERVO MODE =====
    int ch_count = cJSON_GetArraySize(chJ);
    int deg_count = cJSON_GetArraySize(degJ);

    if (ch_count != deg_count)
    {
      reply_error("servo", "ch and deg array size mismatch");
      return;
    }

    if (ch_count == 0 || ch_count > MAX_MULTI_SERVO)
    {
      reply_error("servo", "invalid array size (1-6)");
      return;
    }

    MultiServoTarget multi;
    multi.count = (uint8_t)ch_count;

    // Parse arrays
    for (int i = 0; i < ch_count; i++)
    {
      cJSON *ch_item = cJSON_GetArrayItem(chJ, i);
      cJSON *deg_item = cJSON_GetArrayItem(degJ, i);

      if (!cJSON_IsNumber(ch_item) || !cJSON_IsNumber(deg_item))
      {
        reply_error("servo", "ch/deg array items must be numbers");
        return;
      }

      uint8_t ch = (uint8_t)ch_item->valuedouble;
      float deg = (float)deg_item->valuedouble;

      if (ch > 5)
      {
        reply_error("servo", "ch must be 0-5 for multi-servo");
        return;
      }

      multi.ch[i] = ch;
      multi.target_deg[i] = deg;
    }

    // Queue multi-servo command
    if (g_multi_servo_queue)
    {
      if (xQueueSend(g_multi_servo_queue, &multi, pdMS_TO_TICKS(100)) !=
          pdTRUE)
      {
        ESP_LOGW(TAG, "Multi-servo queue full, dropping command");
        reply_error("servo", "queue full");
        return;
      }

      // Build response with arrays
      cJSON *resp = cJSON_CreateObject();
      cJSON_AddBoolToObject(resp, "ok", true);
      cJSON_AddStringToObject(resp, "type", "servo");
      cJSON *ch_arr = cJSON_CreateArray();
      cJSON *deg_arr = cJSON_CreateArray();
      for (int i = 0; i < ch_count; i++)
      {
        cJSON_AddItemToArray(ch_arr, cJSON_CreateNumber(multi.ch[i]));
        cJSON_AddItemToArray(deg_arr, cJSON_CreateNumber(multi.target_deg[i]));
      }
      cJSON_AddItemToObject(resp, "ch", ch_arr);
      cJSON_AddItemToObject(resp, "deg", deg_arr);
      send_json(resp);
      cJSON_Delete(resp);
    }
    else
    {
      reply_error("servo", "multi-servo queue not ready");
    }
  }
  else if (cJSON_IsNumber(chJ) && cJSON_IsNumber(degJ))
  {
    // ===== SINGLE SERVO MODE (backward compatible) =====
    uint8_t ch = (uint8_t)chJ->valuedouble;
    float deg = (float)degJ->valuedouble;

    if (ch > 15)
    {
      reply_error("servo", "ch must be 0-15");
      return;
    }

    if (ch < 6 && g_servo_queue)
    {
      ServoTarget target;
      target.ch = ch;
      target.target_deg = deg;

      if (xQueueSend(g_servo_queue, &target, pdMS_TO_TICKS(100)) != pdTRUE)
      {
        ESP_LOGW(TAG, "Servo queue full, dropping command");
        reply_error("servo", "queue full");
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
    else
    {
      uint16_t us_min = 400;
      uint16_t us_max = 2700;

      esp_err_t ret = servo_write_angle(ch, deg, g_servo_freq, us_min, us_max);
      if (ret != ESP_OK)
      {
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
  else
  {
    reply_error("servo", "ch/deg must be numbers or arrays");
  }
}

void handle_servo_us(const cJSON *root)
{
  const cJSON *chJ = cJSON_GetObjectItemCaseSensitive(root, "ch");
  const cJSON *usJ = cJSON_GetObjectItemCaseSensitive(root, "us");
  if (!cJSON_IsNumber(chJ) || !cJSON_IsNumber(usJ))
  {
    reply_error("servo_us", "ch/us required");
    return;
  }
  uint8_t ch = (uint8_t)chJ->valuedouble;
  uint16_t us = (uint16_t)usJ->valuedouble;
  if (ch > 15)
  {
    reply_error("servo_us", "ch must be 0-15");
    return;
  }

  uint16_t limit_min = (ch < 6) ? g_joint_limits[ch].min_us : 400;
  uint16_t limit_max = (ch < 6) ? g_joint_limits[ch].max_us : 2700;

  if (us < limit_min || us > limit_max)
  {
    char err[64];
    snprintf(err, sizeof(err), "us must be %d-%d", limit_min, limit_max);
    reply_error("servo_us", err);
    return;
  }
  if (!g_pca_ready)
  {
    reply_error("servo_us", "not inited");
    return;
  }

  esp_err_t ret = ESP_FAIL;
  for (int retry = 0; retry < 3; retry++)
  {
    ret = servo_write_us(ch, us, g_servo_freq);
    if (ret == ESP_OK)
      break;
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  if (ret != ESP_OK)
  {
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

void handle_home(const cJSON *root)
{
  if (!g_pca_ready)
  {
    reply_error("home", "not inited");
    return;
  }

  ESP_LOGI(TAG, "Moving to home position...");

  for (uint8_t ch = 0; ch < 6; ch++)
  {
    float home_angle = g_joint_limits[ch].home;

    esp_err_t ret = ESP_FAIL;
    for (int retry = 0; retry < 3; retry++)
    {
      ret = servo_write_angle(ch, home_angle, g_servo_freq,
                              g_joint_limits[ch].min_us,
                              g_joint_limits[ch].max_us);
      if (ret == ESP_OK)
        break;
      vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (ret != ESP_OK)
    {
      ESP_LOGW(TAG, "Failed to move J%d to home position", ch);
      reply_error("home", "move failed");
      return;
    }

    vTaskDelay(pdMS_TO_TICKS(200));
  }

  cJSON *resp = cJSON_CreateObject();
  cJSON_AddBoolToObject(resp, "ok", true);
  cJSON_AddStringToObject(resp, "type", "home");
  cJSON *positions = cJSON_CreateArray();
  for (int i = 0; i < 6; i++)
  {
    cJSON_AddItemToArray(positions, cJSON_CreateNumber(g_joint_limits[i].home));
  }
  cJSON_AddItemToObject(resp, "positions", positions);
  send_json(resp);
  cJSON_Delete(resp);

  ESP_LOGI(TAG, "Home position reached");
}

void handle_set_limits(const cJSON *root)
{
  const cJSON *limits = cJSON_GetObjectItemCaseSensitive(root, "limits");
  if (!cJSON_IsArray(limits))
  {
    reply_error("set_limits", "limits must be array");
    return;
  }

  int count = cJSON_GetArraySize(limits);
  for (int i = 0; i < count; i++)
  {
    cJSON *item = cJSON_GetArrayItem(limits, i);
    cJSON *chJ = cJSON_GetObjectItemCaseSensitive(item, "ch");
    cJSON *minUsJ = cJSON_GetObjectItemCaseSensitive(item, "min_us");
    cJSON *maxUsJ = cJSON_GetObjectItemCaseSensitive(item, "max_us");

    if (chJ && cJSON_IsNumber(chJ))
    {
      int ch = (int)chJ->valuedouble;
      if (ch >= 0 && ch < 6)
      {
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

void handle_servo_off(const cJSON *root)
{
  if (!g_pca_ready)
  {
    reply_error("servo_off", "not inited");
    return;
  }

  const cJSON *allJ = cJSON_GetObjectItemCaseSensitive(root, "all");
  if (!cJSON_IsBool(allJ) || !cJSON_IsTrue(allJ))
  {
    reply_error("servo_off", "all must be true");
    return;
  }

  ESP_LOGI(TAG, "Turning off all servo PWM outputs...");

  // Turn off all 16 channels by setting ON=0, OFF=0 (no pulse)
  for (uint8_t ch = 0; ch < 16; ch++)
  {
    uint8_t reg = (uint8_t)(0x06 + 4 * ch);
    uint8_t buf[5] = {reg, 0, 0, 0, 0}; // ON_L, ON_H, OFF_L, OFF_H all = 0
    esp_err_t ret =
        i2c_master_transmit(g_pca_dev, buf, sizeof(buf), pdMS_TO_TICKS(100));
    if (ret != ESP_OK)
    {
      ESP_LOGW(TAG, "Failed to turn off channel %d", ch);
    }
  }

  cJSON *resp = cJSON_CreateObject();
  cJSON_AddBoolToObject(resp, "ok", true);
  cJSON_AddStringToObject(resp, "type", "servo_off");
  send_json(resp);
  cJSON_Delete(resp);

  ESP_LOGI(TAG, "All servo PWM outputs disabled");
}

void handle_set_gravity(const cJSON *root)
{
  const cJSON *angles = cJSON_GetObjectItemCaseSensitive(root, "angles");
  if (!cJSON_IsArray(angles))
  {
    reply_error("set_gravity", "angles must be array");
    return;
  }

  int count = cJSON_GetArraySize(angles);
  if (count > 6)
    count = 6;

  for (int i = 0; i < count; i++)
  {
    cJSON *item = cJSON_GetArrayItem(angles, i);
    if (cJSON_IsNumber(item))
    {
      g_global_angles[i] = (float)item->valuedouble;
    }
  }

  ESP_LOGI(TAG, "Global angles updated: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
           g_global_angles[0], g_global_angles[1], g_global_angles[2],
           g_global_angles[3], g_global_angles[4], g_global_angles[5]);

  cJSON *resp = cJSON_CreateObject();
  cJSON_AddBoolToObject(resp, "ok", true);
  cJSON_AddStringToObject(resp, "type", "set_gravity");
  send_json(resp);
  cJSON_Delete(resp);
}
