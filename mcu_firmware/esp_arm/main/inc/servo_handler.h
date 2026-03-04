#ifndef SERVO_HANDLER_H
#define SERVO_HANDLER_H

#include "cJSON.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

  // Joint limit structure
  typedef struct
  {
    float min;
    float max;
    float home;
    uint16_t min_us;
    uint16_t max_us;
  } JointLimit;

  // Servo target structure for queue (single servo)
  typedef struct
  {
    uint8_t ch;
    float target_deg;
  } ServoTarget;

// Multi-servo target structure for synchronized movement
#define MAX_MULTI_SERVO 6
  typedef struct
  {
    uint8_t count;                     // Number of servos to move
    uint8_t ch[MAX_MULTI_SERVO];       // Channel array
    float target_deg[MAX_MULTI_SERVO]; // Target degrees array
  } MultiServoTarget;

  // Global variables (extern)
  extern JointLimit g_joint_limits[6];
  extern float g_current_pos[6];
  extern bool g_smooth_enabled;
  extern float g_step_size;
  extern uint16_t g_step_delay_ms;
  extern uint16_t g_servo_freq;
  extern bool g_pca_ready;
  extern QueueHandle_t g_servo_queue;
  extern QueueHandle_t g_multi_servo_queue;

  // I2C and PCA9685 functions
  esp_err_t i2c_init(void);
  esp_err_t pca_reset(void);
  esp_err_t pca_set_pwm_freq(uint16_t freq_hz);
  esp_err_t servo_write_us(uint8_t ch, uint16_t pulse_us, uint16_t freq_hz);
  esp_err_t servo_write_angle(uint8_t ch, float deg, uint16_t freq_hz,
                              uint16_t us_min, uint16_t us_max);

  // Servo auto init (50Hz)
  esp_err_t servo_auto_init(void);

  // Servo control task
  void servo_control_task(void *arg);
  void multi_servo_control_task(void *arg);

  // Command handlers
  void handle_servo_init(const cJSON *root);
  void handle_servo(const cJSON *root);
  void handle_servo_us(const cJSON *root);
  void handle_home(const cJSON *root);
  void handle_set_limits(const cJSON *root);
  void handle_servo_off(const cJSON *root);
  void handle_set_gravity(const cJSON *root);

  // Global angles from Xavier for gravity compensation
  extern float g_global_angles[6];

#ifdef __cplusplus
}
#endif

#endif // SERVO_HANDLER_H
