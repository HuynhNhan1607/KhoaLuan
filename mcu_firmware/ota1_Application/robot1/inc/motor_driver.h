#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "encoder.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "pid.h"
#include <string.h>

#include "gpio_def.h"

// Number of motors
#define NUM_MOTORS 4

// MCPWM frequency and resolution
#define MOTOR_PWM_FREQ_HZ 5000           // 5 kHz PWM frequency
#define MOTOR_PWM_RESOLUTION_HZ 10000000 // 10 MHz resolution (10,000,000 Hz)

// Default PID gains (tune these for your motors)
#define DEFAULT_KP 0.5f // Reduced from 1.0 to reduce oscillation
#define DEFAULT_KI 0.5f
#define DEFAULT_KD 0.0f // Disabled to avoid noise amplification

#define MCPWM_FORCE_ENABLE_PWM -1
#define MCPWM_FORCE_LOCK_LOW 0
#define MCPWM_FORCE_LOCK_HIGH 1

typedef enum {
  MULTICAST_ADDR = 0,
  WHEEL_1_ADDR,
  WHEEL_2_ADDR,
  WHEEL_3_ADDR,
  WHEEL_4_ADDR
} wheel_addr_t;

typedef struct {
  float WheelSpeed; // Velocity in rad/s
  float WheelPos;   // Position in radians
} wheel_infor_t;

/**
 * @brief Initialize motor driver (MCPWM, encoders, and PID controllers)
 */
void motor_driver_init(void);

/**
 * @brief Set speed for all wheels (rad/s)
 *
 * @param WheelSpeed Array of 4 wheel speeds in rad/s
 */

void motor_control_start(void);

void SetWheelSpeed(float *WheelSpeed);

/**
 * @brief Set speed for a single wheel (rad/s)
 *
 * @param wheel_addr Wheel address (WHEEL_1_ADDR to WHEEL_4_ADDR)
 * @param speed Speed in rad/s
 */
void SetSingleWheelSpeed(wheel_addr_t wheel_addr, float speed);

/**
 * @brief Control mecanum wheel robot
 *
 * @param theta Robot yaw angle
 * @param vx Velocity in x direction
 * @param vy Velocity in y direction
 * @param omega_theta Angular velocity
 */
void MecanumSpeedControl(float theta, float vx, float vy, float omega_theta);

/**
 * @brief Get wheel information (speed and position)
 *
 * @param wheel_infor Array of 4 wheel_infor_t structures to populate
 */
void GetWheelInfor(wheel_infor_t *wheel_infor);

/**
 * @brief Update motor control loop (call this periodically from a task)
 *        This function reads encoders, runs PID, and updates PWM outputs
 */
void motor_control_update(void);

/**
 * @brief Start PID autotune for all 4 motors simultaneously
 *        Target: 70 RPM (fixed), Relay step: 300 PWM
 */
void motor_start_autotune(void);

/**
 * @brief Start PID autotune with custom parameters
 *
 * @param target_rads Target speed in rad/s for autotune
 * @param relay_pwm Relay PWM step value
 */
void motor_start_autotune_custom(float target_rads, float relay_pwm);

/**
 * @brief Check if autotune is running
 *
 * @return true if autotune is active, false otherwise
 */
bool motor_is_autotune_running(void);

/**
 * @brief Get PID gains for a specific motor
 *
 * @param motor_index Motor index (0-3)
 * @param kp Pointer to store Kp
 * @param ki Pointer to store Ki
 * @param kd Pointer to store Kd
 */
void motor_get_pid_gains(uint8_t motor_index, float *kp, float *ki, float *kd);

/**
 * @brief Save PID gains of all 4 motors to NVS
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t motor_save_pid_to_nvs(void);

/**
 * @brief Load PID gains of all 4 motors from NVS
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t motor_load_pid_from_nvs(void);

#endif