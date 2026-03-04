#include "pid.h"
#include <math.h>
#include <string.h>

// Helper macros
#define ABS(x) ((x) > 0 ? (x) : -(x))
#define SIGN(x) ((x) > 0 ? 1 : ((x) < 0 ? -1 : 0))

void PID_Init(PID_Context_t *ctx, float kp, float ki, float kd,
              float output_limit) {
  // Clear entire structure
  memset(ctx, 0, sizeof(PID_Context_t));

  // Set PID gains
  ctx->kp = kp;
  ctx->ki = ki;
  ctx->kd = kd;

  // Set output configuration
  ctx->output_limit = output_limit;
  ctx->motor_deadband = 5.0f;  // Default from main.c
  ctx->motor_static_ff = 8.0f; // Default from main.c

  // Initialize flags
  ctx->is_first_run = true;
  ctx->autotune_active = false;
}

void PID_Reset(PID_Context_t *ctx) {
  ctx->prev_error = 0.0f;
  ctx->accumulated_output = 0.0f;
  ctx->prev_measurement = 0.0f;
  ctx->filtered_derivative = 0.0f;
  ctx->prev_d_term = 0.0f;
  ctx->is_first_run = true;
}

float PID_Compute(PID_Context_t *ctx, float setpoint, float measurement,
                  float dt) {
  if (dt <= 0.0f || dt > 1.0f) {
    dt = 0.02f; // Default 20ms from main.c
  }

  // 1. Calculate raw error
  float raw_error = setpoint - measurement;

  // 2. Apply Low-Pass Filter on error
  float filtered_error = (PID_ERROR_FILTER_ALPHA * raw_error) +
                         ((1.0f - PID_ERROR_FILTER_ALPHA) * ctx->prev_error);

  // On first run, use raw error to avoid startup lag
  if (ctx->is_first_run) {
    filtered_error = raw_error;
    ctx->prev_error = raw_error;
    ctx->prev_measurement = measurement;
    ctx->is_first_run = false;
  }

  // 3. Calculate Delta terms (Incremental PID)

  // Delta P = Kp * (Error_Now - Error_Prev)
  float delta_p = ctx->kp * (filtered_error - ctx->prev_error);

  // Delta I = Ki * Error_Now * dt
  float delta_i = ctx->ki * filtered_error * dt;

  // Delta D (Derivative on Measurement to reduce noise)
  float derivative_raw = 0.0f;
  if (dt > 0.0f) {
    derivative_raw = (measurement - ctx->prev_measurement) / dt;
  }

  // Apply Low-Pass Filter on derivative
  float alpha_d = dt / (PID_D_FILTER_TAU_SEC + dt);
  ctx->filtered_derivative +=
      alpha_d * (derivative_raw - ctx->filtered_derivative);

  float current_d_term = -ctx->kd * ctx->filtered_derivative;
  float delta_d = current_d_term - ctx->prev_d_term;

  // 4. Update accumulated output
  float delta_output = delta_p + delta_i + delta_d;
  ctx->accumulated_output += delta_output;

  // 5. Anti-windup: Clamp accumulated output
  if (ctx->accumulated_output > ctx->output_limit) {
    ctx->accumulated_output = ctx->output_limit;
  } else if (ctx->accumulated_output < -ctx->output_limit) {
    ctx->accumulated_output = -ctx->output_limit;
  }

  // 6. Save state for next iteration
  ctx->prev_error = filtered_error;
  ctx->prev_measurement = measurement;
  ctx->prev_d_term = current_d_term;

  return ctx->accumulated_output;
}

int PID_ApplyMotorCompensation(PID_Context_t *ctx, int raw_pwm) {
  if (raw_pwm == 0)
    return 0;

  int sign = SIGN(raw_pwm);
  int magnitude = ABS(raw_pwm);

  // Apply deadband
  if (magnitude <= (int)ctx->motor_deadband)
    return 0;

  // Compensate: remove deadband and add static friction
  magnitude = magnitude - (int)ctx->motor_deadband + (int)ctx->motor_static_ff;

  // Clamp to output limit
  if (magnitude > (int)ctx->output_limit)
    magnitude = (int)ctx->output_limit;

  return sign * magnitude;
}

void PID_StartAutotune(PID_Context_t *ctx, float relay_step) {
  // Reset autotune state
  ctx->autotune_active = true;
  ctx->autotune_relay_pwm = relay_step;
  ctx->autotune_cycles_completed = 0;
  ctx->crossing_count = 0;
  ctx->last_error_sign = 0;
  ctx->last_crossing_time_ms = 0;
  ctx->peak_max = 0.0f;
  ctx->peak_min = 0.0f;
  ctx->first_cycle = true;
  memset(ctx->crossing_times_ms, 0, sizeof(ctx->crossing_times_ms));
}

float PID_RunAutotune(PID_Context_t *ctx, float target, float measurement,
                      uint32_t current_time_ms) {
  if (!ctx->autotune_active)
    return 0.0f;

  // 1. Dùng target truyền vào thay vì define cứng
  float error = target - measurement;
  int error_sign = SIGN(error);

  // Track peaks
  if (measurement > ctx->peak_max)
    ctx->peak_max = measurement;
  if (measurement < ctx->peak_min || ctx->first_cycle)
    ctx->peak_min = measurement;

  // Detect zero crossings
  if (ctx->last_error_sign != 0 && error_sign != 0 &&
      ctx->last_error_sign != error_sign) {
    if (ctx->crossing_count < PID_AUTOTUNE_MAX_CROSSINGS) {
      ctx->crossing_times_ms[ctx->crossing_count++] = current_time_ms;
    }
    if (ctx->crossing_count % 2 == 0) {
      ctx->autotune_cycles_completed = ctx->crossing_count / 2;
      if (ctx->first_cycle)
        ctx->first_cycle = false;
    }
    ctx->last_crossing_time_ms = current_time_ms;
  }
  ctx->last_error_sign = error_sign;

  // 2. Logic Relay chuẩn có Hysteresis (như file main.c cũ)
  float relay_output = 0.0f;

  // QUAN TRỌNG: Điều chỉnh Hysteresis cho phù hợp với đơn vị Rad/s
  // Nếu đơn vị là Rad/s, 0.5f có thể hơi lớn, nhưng tạm giữ nguyên hoặc giảm
  // xuống 0.1f nếu cần
  float hysteresis_val = PID_AUTOTUNE_HYSTERESIS;

  if (error > hysteresis_val) {
    relay_output = ctx->autotune_relay_pwm;
  } else if (error < -hysteresis_val) {
    relay_output = -ctx->autotune_relay_pwm;
  } else {
    // Vùng chết: GIỮ NGUYÊN trạng thái cũ (Latch state)
    relay_output = ctx->last_relay_output;
  }

  // Clamp output
  if (relay_output > ctx->output_limit)
    relay_output = ctx->output_limit;
  if (relay_output < -ctx->output_limit)
    relay_output = -ctx->output_limit;

  // Lưu lại trạng thái
  ctx->last_relay_output = relay_output;

  return relay_output;
}

bool PID_CheckAutotuneComplete(PID_Context_t *ctx) {
  return (ctx->autotune_cycles_completed >= PID_AUTOTUNE_MAX_CYCLES);
}

// void PID_ApplyAutotuneResults(PID_Context_t *ctx)
// {
//     if (ctx->crossing_count < 4)
//     {
//         // Insufficient data, can't calculate
//         ctx->Tu = 0.0f;
//         ctx->Ku = 0.0f;
//         return;
//     }

//     // Calculate average period (Tu)
//     float total_period_sec = 0.0f;
//     int period_count = 0;

//     for (int i = 2; i < ctx->crossing_count && i <
//     PID_AUTOTUNE_MAX_CROSSINGS; i++)
//     {
//         uint32_t period_ms = ctx->crossing_times_ms[i] -
//         ctx->crossing_times_ms[i - 2]; total_period_sec += (float)period_ms /
//         1000.0f; period_count++;
//     }

//     ctx->Tu = (period_count > 0) ? (total_period_sec / period_count) : 0.0f;

//     // Calculate ultimate gain (Ku)
//     float rpm_amplitude = (ctx->rpm_peak_max - ctx->rpm_peak_min) / 2.0f;
//     float d = ctx->autotune_relay_pwm;
//     ctx->Ku = (rpm_amplitude > 0.1f) ? ((4.0f * d) / (3.14159f *
//     rpm_amplitude)) : 0.0f;

//     // Apply Ziegler-Nichols Conservative Tuning
//     if (ctx->Tu > 0.0f && ctx->Ku > 0.0f)
//     {
//         float Kp_cons = 0.33f * ctx->Ku;
//         float Ki_cons = 2.0f * Kp_cons / ctx->Tu;
//         float Kd_cons = Kp_cons * ctx->Tu / 3.0f;

//         ctx->kp = Kp_cons;
//         ctx->ki = Ki_cons;
//         ctx->kd = Kd_cons;
//     }

//     // Disable autotune and reset PID state
//     ctx->autotune_active = false;
//     PID_Reset(ctx);
// }

void PID_ApplyAutotuneResults(PID_Context_t *ctx) {
  if (ctx->crossing_count < 4) {
    // Insufficient data, can't calculate
    ctx->Tu = 0.0f;
    ctx->Ku = 0.0f;
    return;
  }

  // Calculate average period (Tu)
  float total_period_sec = 0.0f;
  int period_count = 0;

  for (int i = 2; i < ctx->crossing_count && i < PID_AUTOTUNE_MAX_CROSSINGS;
       i++) {
    uint32_t period_ms =
        ctx->crossing_times_ms[i] - ctx->crossing_times_ms[i - 2];
    total_period_sec += (float)period_ms / 1000.0f;
    period_count++;
  }

  ctx->Tu = (period_count > 0) ? (total_period_sec / period_count) : 0.0f;

  // Calculate ultimate gain (Ku)
  // Ku = 4 * d / (pi * a)
  float amplitude = (ctx->peak_max - ctx->peak_min) / 2.0f;
  float d = ctx->autotune_relay_pwm;
  ctx->Ku = (amplitude > 0.1f) ? ((4.0f * d) / (3.14159f * amplitude)) : 0.0f;

  // --- Công thức bảo thủ cho sàn trơn (Over-damped) ---
  if (ctx->Tu > 0.0f && ctx->Ku > 0.0f) {
    // 1. Kp: Giảm mạnh gain để tránh trượt khi khởi động (Kp = Ku / 6.0)
    float Kp_safe = ctx->Ku / 6.0f;

    // 2. Ki: Đặt Ti lớn để tích phân chậm, tránh vọt lố (Ti = 3.0 * Tu)
    float Ki_safe = Kp_safe / (3.0f * ctx->Tu);

    // 3. Kd: Tăng thời gian vi phân để giảm dao động (Td = Tu / 3.0)
    float Kd_safe = Kp_safe * (ctx->Tu / 3.0f);

    ctx->kp = Kp_safe;
    ctx->ki = Ki_safe;
    ctx->kd = Kd_safe;
  }

  // Disable autotune and reset PID state
  ctx->autotune_active = false;
  PID_Reset(ctx);
}

void PID_SetGains(PID_Context_t *ctx, float kp, float ki, float kd) {
  ctx->kp = kp;
  ctx->ki = ki;
  ctx->kd = kd;
}

void PID_SetMotorCompensation(PID_Context_t *ctx, float deadband,
                              float static_ff) {
  ctx->motor_deadband = deadband;
  ctx->motor_static_ff = static_ff;
}
