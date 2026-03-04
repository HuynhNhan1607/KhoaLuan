#ifndef ENCODER_H_
#define ENCODER_H_

#include "driver/pulse_cnt.h"
#include "esp_err.h"
#include <stdint.h>
#include "gpio_def.h"

// Encoder resolution: 56 (gear ratio) * 16 (motor ppr) * 4 (quadrature) = 3584 pulses per revolution
#define ENCODER_PPR 3584
#define PULSES_PER_RADIAN (ENCODER_PPR / (2.0f * M_PI))

// Number of motors/encoders
#define NUM_ENCODERS 4

/**
 * @brief Encoder instance structure
 */
typedef struct
{
    pcnt_unit_handle_t pcnt_unit; // PCNT unit handle
    int phase_a_gpio;             // Phase A GPIO pin
    int phase_b_gpio;             // Phase B GPIO pin
    int32_t pulse_count;          // Current pulse count
    int32_t prev_pulse_count;     // Previous pulse count
    float position;               // Position in radians
    float velocity;               // Velocity in rad/s
    int64_t last_update_time;     // Last update timestamp in microseconds
} encoder_t;

/**
 * @brief Initialize all encoders
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t encoder_init(void);

/**
 * @brief Get encoder velocity
 *
 * @param index Encoder index (0-3 for motors 1-4)
 * @return float Velocity in rad/s
 */
float get_encoder_velocity(uint8_t index);

/**
 * @brief Get encoder position
 *
 * @param index Encoder index (0-3 for motors 1-4)
 * @return float Position in radians
 */
float get_encoder_position(uint8_t index);

/**
 * @brief Update all encoder readings (pulse count, velocity, position)
 *        Call this periodically from a task
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t encoder_update(void);

/**
 * @brief Reset encoder position to zero
 *
 * @param index Encoder index (0-3 for motors 1-4)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t encoder_reset(uint8_t index);

#endif // ENCODER_H_
