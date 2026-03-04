#ifndef PID_H_
#define PID_H_

#include <stdbool.h>
#include <stdint.h>

// Configuration Constants
#define PID_ERROR_FILTER_ALPHA 0.8f // Low-pass filter coefficient for error
#define PID_D_FILTER_TAU_SEC 0.05f  // D-term filter time constant
#define PID_AUTOTUNE_TARGET_RADS                                               \
  4.0f // Autotune target in rad/s (approx 0.2 m/s)
#define PID_AUTOTUNE_HYSTERESIS 0.5f  // Hysteresis in rad/s for relay
#define PID_AUTOTUNE_MAX_CYCLES 8     // Number of cycles to complete
#define PID_AUTOTUNE_MAX_CROSSINGS 20 // Max crossing times to store

/**
 * @brief PID Controller Context (Incremental/Delta Form with Autotune)
 *        Supports per-motor instance for multi-motor systems
 */
typedef struct {
  // === PID Gains ===
  float kp; // Proportional gain
  float ki; // Integral gain
  float kd; // Derivative gain

  // === PID State (Incremental Form) ===
  float prev_error;          // e(k-1) - Previous filtered error
  float accumulated_output;  // u(k) - Accumulated output
  float prev_measurement;    // For derivative on measurement
  float filtered_derivative; // Low-pass filtered derivative
  float prev_d_term;         // D-term from previous iteration
  bool is_first_run;         // Flag for first PID computation

  // === Output Configuration ===
  float output_limit;    // Maximum absolute output (e.g., 1000 for PWM)
  float motor_deadband;  // Deadband compensation
  float motor_static_ff; // Static friction feedforward

  // === Autotune State ===
  bool autotune_active;           // Autotune mode flag
  float autotune_relay_pwm;       // Relay step amplitude
  int autotune_cycles_completed;  // Number of completed cycles
  int last_error_sign;            // Sign of error from previous iteration
  uint32_t last_crossing_time_ms; // Last zero-crossing time
  uint32_t crossing_times_ms[PID_AUTOTUNE_MAX_CROSSINGS]; // Crossing timestamps
  int crossing_count; // Number of crossings recorded
  float peak_max;     // Maximum value during autotune (rad/s)
  float peak_min;     // Minimum value during autotune (rad/s)
  bool first_cycle;   // Flag for first autotune cycle

  float last_relay_output;
  // === Autotune Results ===
  float Tu; // Ultimate period (seconds)
  float Ku; // Ultimate gain

} PID_Context_t;

/**
 * @brief Initialize PID controller context
 *
 * @param ctx Pointer to PID context structure
 * @param kp Proportional gain (initial, can be 0 if using autotune)
 * @param ki Integral gain (initial, can be 0 if using autotune)
 * @param kd Derivative gain (initial, can be 0 if using autotune)
 * @param output_limit Maximum absolute output value (e.g., 1000 for PWM)
 */
void PID_Init(PID_Context_t *ctx, float kp, float ki, float kd,
              float output_limit);

/**
 * @brief Reset PID controller state (clear integrator, errors, etc.)
 *
 * @param ctx Pointer to PID context structure
 */
void PID_Reset(PID_Context_t *ctx);

/**
 * @brief Compute PID output using Incremental (Delta) form
 *        Implements Low-Pass Filter on error and derivative on measurement
 *
 * @param ctx Pointer to PID context structure
 * @param setpoint Target value (e.g., desired RPM)
 * @param measurement Current measured value (e.g., actual RPM)
 * @param dt Sample time in seconds
 * @return float Raw PID output (before motor compensation)
 */
float PID_Compute(PID_Context_t *ctx, float setpoint, float measurement,
                  float dt);

/**
 * @brief Apply motor compensation (deadband + static friction feedforward)
 *
 * @param ctx Pointer to PID context structure
 * @param raw_pwm Raw PWM value from PID
 * @return int Compensated PWM value
 */
int PID_ApplyMotorCompensation(PID_Context_t *ctx, int raw_pwm);

/**
 * @brief Start autotune process (Relay Method)
 *        Target is fixed at 70 RPM
 *
 * @param ctx Pointer to PID context structure
 * @param relay_step PWM step size for relay (e.g., 300)
 */
void PID_StartAutotune(PID_Context_t *ctx, float relay_step);

/**
 * @brief Run one iteration of autotune (Relay Method)
 *        Must be called periodically at the control loop rate
 *
 * @param ctx Pointer to PID context structure
 * @param measurement Current measured RPM
 * @param current_time_ms Current timestamp in milliseconds
 * @return float Relay output PWM (high or low)
 */
float PID_RunAutotune(PID_Context_t *ctx, float target, float measurement,
                      uint32_t current_time_ms);
/**
 * @brief Check if autotune has completed
 *
 * @param ctx Pointer to PID context structure
 * @return true if autotune is complete, false otherwise
 */
bool PID_CheckAutotuneComplete(PID_Context_t *ctx);

/**
 * @brief Calculate and apply autotune results (Ziegler-Nichols Conservative)
 *        Automatically sets Kp, Ki, Kd in the context
 *
 * @param ctx Pointer to PID context structure
 */
void PID_ApplyAutotuneResults(PID_Context_t *ctx);

/**
 * @brief Manually set PID gains
 *
 * @param ctx Pointer to PID context structure
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 */
void PID_SetGains(PID_Context_t *ctx, float kp, float ki, float kd);

/**
 * @brief Set motor compensation parameters
 *
 * @param ctx Pointer to PID context structure
 * @param deadband Deadband PWM value
 * @param static_ff Static friction feedforward PWM value
 */
void PID_SetMotorCompensation(PID_Context_t *ctx, float deadband,
                              float static_ff);

#endif // PID_H_
