#include "motor_driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "kinematic.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "MotorDriver";
static const char *NVS_NAMESPACE = "motor_pid";

// Motor control structure
typedef struct {
  mcpwm_timer_handle_t timer;
  mcpwm_oper_handle_t operator;
  mcpwm_cmpr_handle_t comparator;
  mcpwm_gen_handle_t generator_l;
  mcpwm_gen_handle_t generator_r;
  int l_pwm_gpio;
  int r_pwm_gpio;
  float target_speed;      // Target speed in rad/s
  float prev_target_speed; // Previous target for feedforward detection
  float current_speed;     // Current speed in rad/s
  PID_Context_t pid;       // PID controller for this motor (new API)
} motor_t;

// Array of motors
static motor_t motors[NUM_MOTORS];

// Share Timer
static mcpwm_timer_handle_t shared_timers[2];

// Task handle for motor control update
static TaskHandle_t motor_control_task_handle = NULL;

// Autotune state
static bool autotune_active = false;
static uint8_t autotune_completed_count = 0;

// GPIO pin arrays for configuration
static const int l_pwm_pins[NUM_MOTORS] = {
    MOTOR_1_L_PWM_GPIO, MOTOR_2_L_PWM_GPIO, MOTOR_3_L_PWM_GPIO,
    MOTOR_4_L_PWM_GPIO};

static const int r_pwm_pins[NUM_MOTORS] = {
    MOTOR_1_R_PWM_GPIO, MOTOR_2_R_PWM_GPIO, MOTOR_3_R_PWM_GPIO,
    MOTOR_4_R_PWM_GPIO};

// Forward declarations
static void motor_control_task(void *pvParameters);

/**
 * @brief Set motor PWM duty cycle and direction for BTS7960
 *
 * @param motor_index Motor index (0-3)
 * @param duty_cycle Duty cycle (-100 to 100, negative = reverse)
 */
static void set_motor_pwm(uint8_t motor_index, float duty_cycle) {
  if (motor_index >= NUM_MOTORS) {
    return;
  }

  // Determine direction and absolute duty cycle
  float abs_duty = (duty_cycle >= 0) ? duty_cycle : -duty_cycle;

  // Clamp duty cycle to 0-100%
  if (abs_duty > 100.0f) {
    abs_duty = 100.0f;
  }

  uint32_t period = MOTOR_PWM_RESOLUTION_HZ / MOTOR_PWM_FREQ_HZ;

  // Convert percentage to compare value (0-100% maps to 0-period)
  uint32_t compare_value = (uint32_t)(abs_duty * (period / 100.0f));

  // Set PWM duty cycle
  mcpwm_comparator_set_compare_value(motors[motor_index].comparator,
                                     compare_value);

  if (duty_cycle > 0) {
    // Forward: L_PWM active, R_PWM low
    mcpwm_generator_set_force_level(motors[motor_index].generator_r,
                                    MCPWM_FORCE_LOCK_LOW, true); // Force R low
    mcpwm_generator_set_force_level(motors[motor_index].generator_l,
                                    MCPWM_FORCE_ENABLE_PWM,
                                    true); // Enable L PWM
  } else if (duty_cycle < 0) {
    // Reverse: L_PWM low, R_PWM active
    mcpwm_generator_set_force_level(motors[motor_index].generator_l,
                                    MCPWM_FORCE_LOCK_LOW, true); // Force L low
    mcpwm_generator_set_force_level(motors[motor_index].generator_r,
                                    MCPWM_FORCE_ENABLE_PWM,
                                    true); // Enable R PWM
  } else {
    // Stop: Both low
    mcpwm_generator_set_force_level(motors[motor_index].generator_l,
                                    MCPWM_FORCE_LOCK_LOW, true);
    mcpwm_generator_set_force_level(motors[motor_index].generator_r,
                                    MCPWM_FORCE_LOCK_LOW, true);
  }
}

void motor_driver_init(void) {
  ESP_LOGI(TAG, "Initializing motor driver (Hybrid Mode - Corrected)...");

  esp_err_t ret;

  // 1. Khởi tạo Encoders trước
  ret = encoder_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize encoders: %s", esp_err_to_name(ret));
    return;
  }

  // 2. TẠO SHARED TIMERS (Nhưng CHƯA Start vội)
  mcpwm_timer_config_t timer_config = {
      .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
      .resolution_hz = MOTOR_PWM_RESOLUTION_HZ,
      .period_ticks = MOTOR_PWM_RESOLUTION_HZ / MOTOR_PWM_FREQ_HZ,
      .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
  };

  for (int i = 0; i < 2; i++) {
    timer_config.group_id = i;
    ret = mcpwm_new_timer(&timer_config, &shared_timers[i]);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to create Shared Timer %d: %s", i,
               esp_err_to_name(ret));
      return;
    }
  }

  // 3. CẤU HÌNH CHI TIẾT 4 MOTOR (Operator, Comparator, Generator)
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].l_pwm_gpio = l_pwm_pins[i];
    motors[i].r_pwm_gpio = r_pwm_pins[i];
    motors[i].target_speed = 0.0f;
    motors[i].prev_target_speed = 0.0f;
    motors[i].current_speed = 0.0f;

    int group_id = i / 2;
    motors[i].timer = shared_timers[group_id]; // Gán timer để quản lý

    // 3.1. Tạo Operator
    mcpwm_operator_config_t operator_config = {.group_id = group_id};
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &motors[i].operator));

    // 3.2. Kết nối Operator vào Shared Timer (Timer lúc này vẫn đang ĐỨNG IM)
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(motors[i].operator,
                                                 shared_timers[group_id]));

    // 3.3. Tạo Comparator
    mcpwm_comparator_config_t comparator_config = {.flags.update_cmp_on_tez =
                                                       true};
    ESP_ERROR_CHECK(mcpwm_new_comparator(motors[i].operator, &comparator_config,
                                         &motors[i].comparator));

    // 3.4. Tạo Generators
    mcpwm_generator_config_t generator_l_config = {.gen_gpio_num =
                                                       motors[i].l_pwm_gpio};
    mcpwm_generator_config_t generator_r_config = {.gen_gpio_num =
                                                       motors[i].r_pwm_gpio};

    ESP_ERROR_CHECK(mcpwm_new_generator(motors[i].operator, &generator_l_config,
                                        &motors[i].generator_l));
    ESP_ERROR_CHECK(mcpwm_new_generator(motors[i].operator, &generator_r_config,
                                        &motors[i].generator_r));

    // 3.5. Cấu hình Action
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        motors[i].generator_l,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        motors[i].generator_l,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       motors[i].comparator,
                                       MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        motors[i].generator_r,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        motors[i].generator_r,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       motors[i].comparator,
                                       MCPWM_GEN_ACTION_LOW)));

    // 3.6. Set trạng thái ban đầu an toàn (STOP)
    mcpwm_generator_set_force_level(motors[i].generator_l, MCPWM_FORCE_LOCK_LOW,
                                    true);
    mcpwm_generator_set_force_level(motors[i].generator_r, MCPWM_FORCE_LOCK_LOW,
                                    true);
    mcpwm_comparator_set_compare_value(motors[i].comparator, 0);

    // Init PID
    PID_Init(&motors[i].pid, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, 100.0f);

    ESP_LOGI(TAG, "Motor %d configured (Group %d)", i, group_id);
  }

  // 4. KÍCH HOẠT TIMERS (Bây giờ mọi thứ đã sẵn sàng mới Start)
  ESP_LOGI(TAG, "Starting all PWM timers...");
  for (int i = 0; i < 2; i++) {
    ESP_ERROR_CHECK(mcpwm_timer_enable(shared_timers[i]));
    ESP_ERROR_CHECK(
        mcpwm_timer_start_stop(shared_timers[i], MCPWM_TIMER_START_NO_STOP));
  }

  // Load PID từ NVS
  motor_load_pid_from_nvs();

  ESP_LOGI(TAG, "Motor driver initialization complete");
}

void motor_control_start(void) {
  BaseType_t task_created =
      xTaskCreatePinnedToCore(motor_control_task, "motor_ctrl", 4096, NULL,
                              24, // Highest priority for control loop (PID)
                              &motor_control_task_handle, 1);

  if (task_created != pdPASS) {
    ESP_LOGE(TAG, "Failed to create motor control task");
    return;
  }
}

void motor_control_update(void) {
  // Update encoder readings
  encoder_update();

  // Check if autotune mode is active
  if (autotune_active) {
    // Get current time in milliseconds
    uint32_t current_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Run autotune for all motors
    for (int i = 0; i < NUM_MOTORS; i++) {
      // Get current speed from encoder
      motors[i].current_speed = get_encoder_velocity(i);

      // Run autotune and get relay PWM output
      float relay_pwm =
          PID_RunAutotune(&motors[i].pid, motors[i].target_speed,
                          motors[i].current_speed, current_time_ms);

      // Set motor PWM to relay output
      set_motor_pwm(i, relay_pwm);
    }

    // Check if all motors completed autotune
    uint8_t completed = 0;
    for (int i = 0; i < NUM_MOTORS; i++) {
      if (PID_CheckAutotuneComplete(&motors[i].pid)) {
        completed++;
      }
    }

    // If all motors completed, apply results and save to NVS
    if (completed == NUM_MOTORS && autotune_completed_count != NUM_MOTORS) {
      ESP_LOGI(TAG, "Autotune completed for all motors!");

      for (int i = 0; i < NUM_MOTORS; i++) {
        PID_ApplyAutotuneResults(&motors[i].pid);
        ESP_LOGI(TAG,
                 "Motor %d - Kp: %.3f, Ki: %.3f, Kd: %.3f, Tu: %.3f, Ku: %.3f",
                 i, motors[i].pid.kp, motors[i].pid.ki, motors[i].pid.kd,
                 motors[i].pid.Tu, motors[i].pid.Ku);
      }

      // Save to NVS
      motor_save_pid_to_nvs();

      // Stop all motors
      for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].target_speed = 0.0f;
        set_motor_pwm(i, 0);
      }

      autotune_active = false;
      autotune_completed_count = NUM_MOTORS;
      ESP_LOGI(TAG, "Autotune finished and PID saved to NVS");
    }
  } else {
    // Normal PID control mode
    for (int i = 0; i < NUM_MOTORS; i++) {
      // Get current speed from encoder
      motors[i].current_speed = get_encoder_velocity(i);

      // Initialize feedforward if target changed significantly
      float speed_change =
          fabs(motors[i].target_speed - motors[i].prev_target_speed);
      if (speed_change > 0.5f) // More than 0.5 rad/s (~5 RPM) change
      {
        // Special case: Target = 0 (STOP command)
        if (fabs(motors[i].target_speed) < 0.1f) {
          // Force accumulated_output to 0 immediately
          motors[i].pid.accumulated_output = 0.0f;
        } else {
          // Estimate feedforward: Assume linear relationship RPM -> PWM
          // For 100 RPM max motor: 10.47 rad/s requires ~100% PWM
          // So 1 rad/s ≈ 9.55% PWM
          float feedforward = motors[i].target_speed * 9.55f;

          // Initialize accumulated_output to feedforward value
          motors[i].pid.accumulated_output = feedforward;
        }

        // Reset PID state to avoid sudden jumps
        motors[i].pid.prev_error = 0.0f;
        motors[i].pid.filtered_derivative = 0.0f;
        motors[i].pid.prev_d_term = 0.0f;

        motors[i].prev_target_speed = motors[i].target_speed;
      } // Compute PID output with fixed 10ms sample time (100 Hz loop)
      float raw_pid_output = PID_Compute(&motors[i].pid, motors[i].target_speed,
                                         motors[i].current_speed, 0.01f);
      int compensated_pwm_int =
          PID_ApplyMotorCompensation(&motors[i].pid, (int)raw_pid_output);

      float final_pwm = (float)compensated_pwm_int;
      // Set motor PWM
      set_motor_pwm(i, final_pwm);
    }
  }
}

/**
 * @brief Motor control task - runs PID control loop
 */
static void motor_control_task(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100 Hz control loop

  xLastWakeTime = xTaskGetTickCount();

  ESP_LOGW(TAG, "Motor control task started");

  while (1) {
    motor_control_update();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void SetWheelSpeed(float *WheelSpeed) {
  if (WheelSpeed == NULL) {
    ESP_LOGE(TAG, "SetWheelSpeed: NULL pointer");
    return;
  }

  // Set target speeds for all motors
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].target_speed = WheelSpeed[i];
  }
}

void SetSingleWheelSpeed(wheel_addr_t wheel_addr, float speed) {
  if (wheel_addr < WHEEL_1_ADDR || wheel_addr > WHEEL_4_ADDR) {
    ESP_LOGE(TAG, "SetSingleWheelSpeed: Invalid wheel address %d", wheel_addr);
    return;
  }

  // Convert wheel address to motor index (WHEEL_1_ADDR = 1, motor index = 0)
  int motor_index = wheel_addr - 1;
  motors[motor_index].target_speed = (float)speed;
}

void MecanumSpeedControl(float theta, float vx, float vy, float omega_theta) {
  float wheel_speed[4];
  CalculateWheelSpeed(wheel_speed, theta, vx, vy, omega_theta);
  SetWheelSpeed(wheel_speed);
}

void GetWheelInfor(wheel_infor_t *wheel_infor) {
  if (wheel_infor == NULL) {
    ESP_LOGE(TAG, "GetWheelInfor: NULL pointer");
    return;
  }

  // Update encoders first
  encoder_update();

  // Populate wheel information from encoders
  for (int i = 0; i < NUM_MOTORS; i++) {
    wheel_infor[i].WheelSpeed = get_encoder_velocity(i);
    wheel_infor[i].WheelPos = get_encoder_position(i);
  }
}

// ============================================================
// PID AUTOTUNE & NVS FUNCTIONS
// ============================================================

void motor_start_autotune(void) {
  // Default values: ~4.0 rad/s (approx 0.2 m/s), 60 PWM
  motor_start_autotune_custom(PID_AUTOTUNE_TARGET_RADS, 60.0f);
}

void motor_start_autotune_custom(float target_rads, float relay_pwm) {
  ESP_LOGI(TAG, "Starting PID autotune for all 4 motors...");

  // Safety limits: Max ~11.5 rad/s (110 RPM), PWM max 100
  if (target_rads > 11.5f) {
    ESP_LOGW(TAG, "Target %.1f rad/s exceeds limit, clamping to 11.5",
             target_rads);
    target_rads = 11.5f;
  }

  if (relay_pwm > 100.0f) {
    ESP_LOGW(TAG, "PWM %.1f exceeds limit, clamping to 100 PWM", relay_pwm);
    relay_pwm = 100.0f;
  }

  // Start autotune for all 4 motors simultaneously
  for (int i = 0; i < NUM_MOTORS; i++) {
    PID_StartAutotune(&motors[i].pid, relay_pwm);
    motors[i].target_speed = target_rads;
  }

  autotune_active = true;
  autotune_completed_count = 0;

  ESP_LOGI(TAG, "Autotune started (Target: %.1f rad/s, Relay: %.1f PWM)",
           target_rads, relay_pwm);
}

bool motor_is_autotune_running(void) { return autotune_active; }

void motor_get_pid_gains(uint8_t motor_index, float *kp, float *ki, float *kd) {
  if (motor_index >= NUM_MOTORS || kp == NULL || ki == NULL || kd == NULL) {
    return;
  }

  *kp = motors[motor_index].pid.kp;
  *ki = motors[motor_index].pid.ki;
  *kd = motors[motor_index].pid.kd;
}

esp_err_t motor_save_pid_to_nvs(void) {
  nvs_handle_t nvs_handle;
  esp_err_t err;

  err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
    return err;
  }

  // Save PID gains for each motor
  for (int i = 0; i < NUM_MOTORS; i++) {
    char key_kp[16], key_ki[16], key_kd[16];
    snprintf(key_kp, sizeof(key_kp), "m%d_kp", i);
    snprintf(key_ki, sizeof(key_ki), "m%d_ki", i);
    snprintf(key_kd, sizeof(key_kd), "m%d_kd", i);

    // Save as uint32_t (reinterpret float bits)
    uint32_t kp_bits, ki_bits, kd_bits;
    memcpy(&kp_bits, &motors[i].pid.kp, sizeof(float));
    memcpy(&ki_bits, &motors[i].pid.ki, sizeof(float));
    memcpy(&kd_bits, &motors[i].pid.kd, sizeof(float));

    nvs_set_u32(nvs_handle, key_kp, kp_bits);
    nvs_set_u32(nvs_handle, key_ki, ki_bits);
    nvs_set_u32(nvs_handle, key_kd, kd_bits);

    ESP_LOGI(TAG, "Saved Motor %d PID: Kp=%.3f, Ki=%.3f, Kd=%.3f", i,
             motors[i].pid.kp, motors[i].pid.ki, motors[i].pid.kd);
  }

  err = nvs_commit(nvs_handle);
  nvs_close(nvs_handle);

  if (err == ESP_OK) {
    ESP_LOGI(TAG, "PID gains saved to NVS successfully");
  } else {
    ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(err));
  }

  return err;
}

esp_err_t motor_load_pid_from_nvs(void) {
  nvs_handle_t nvs_handle;
  esp_err_t err;

  err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "No saved PID data in NVS (using defaults)");
    return err;
  }

  // Load PID gains for each motor
  bool all_loaded = true;
  for (int i = 0; i < NUM_MOTORS; i++) {
    char key_kp[16], key_ki[16], key_kd[16];
    snprintf(key_kp, sizeof(key_kp), "m%d_kp", i);
    snprintf(key_ki, sizeof(key_ki), "m%d_ki", i);
    snprintf(key_kd, sizeof(key_kd), "m%d_kd", i);

    uint32_t kp_bits, ki_bits, kd_bits;

    if (nvs_get_u32(nvs_handle, key_kp, &kp_bits) == ESP_OK &&
        nvs_get_u32(nvs_handle, key_ki, &ki_bits) == ESP_OK &&
        nvs_get_u32(nvs_handle, key_kd, &kd_bits) == ESP_OK) {
      // Restore float values
      float kp, ki, kd;
      memcpy(&kp, &kp_bits, sizeof(float));
      memcpy(&ki, &ki_bits, sizeof(float));
      memcpy(&kd, &kd_bits, sizeof(float));

      PID_SetGains(&motors[i].pid, kp, ki, kd);
      // ESP_LOGW(TAG, "Motor %d PID: Kp=%.3f, Ki=%.3f, Kd=%.3f", i, kp, ki,
      // kd);
      ESP_LOGW(TAG, "Loaded Motor %d PID: Kp=%.3f, Ki=%.3f, Kd=%.3f", i, kp, ki,
               kd);
    } else {
      ESP_LOGW(TAG, "Motor %d PID not found in NVS", i);
      all_loaded = false;
    }
  }

  nvs_close(nvs_handle);

  return all_loaded ? ESP_OK : ESP_ERR_NOT_FOUND;
}