#include "optical_flow.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

// Hàm khởi tạo LPF state
static void lpf_init(lpf_state_t *lpf)
{
  memset(lpf, 0, sizeof(lpf_state_t));
  lpf->initialized = false;
}

// Hàm LPF Order 2: y[n] = a1*y[n-1] + a2*y[n-2] + b0*x[n] + b1*x[n-1] +
// b2*x[n-2]
static double lpf_update(lpf_state_t *lpf, double input)
{
  double output;

  if (!lpf->initialized)
  {
    // Khởi tạo với giá trị input để tránh transient lớn
    lpf->y[0] = input;
    lpf->y[1] = input;
    lpf->x[0] = input;
    lpf->x[1] = input;
    lpf->initialized = true;
    output = input;
  }
  else
  {
    // Tính output
    output = LPF_A1 * lpf->y[0] + LPF_A2 * lpf->y[1] + LPF_B0 * input +
             LPF_B1 * lpf->x[0] + LPF_B2 * lpf->x[1];

    // Shift states
    lpf->y[1] = lpf->y[0];
    lpf->y[0] = output;
    lpf->x[1] = lpf->x[0];
    lpf->x[0] = input;
  }

  return output;
}

// Khởi tạo optical flow processor
void optical_flow_init(optical_flow_t *of)
{
  memset(of, 0, sizeof(optical_flow_t));
  lpf_init(&of->lpf_vx);
  lpf_init(&of->lpf_vy);
  of->valid = false;
}

// Xử lý dữ liệu raw
bool optical_flow_process(optical_flow_t *of, int16_t vx_raw, int16_t vy_raw,
                          double theta, double omega_z, uint8_t quality)
{
  of->frame_count++;
  of->quality = quality; // Lưu quality

  // Check quality threshold
  if (quality < OPTICAL_FLOW_MIN_QUALITY)
  {
    of->rejected_count++;
    // DEBUG: Don't invalidate, just warn
    // of->valid = false;
    // return false;
    printf(
        "[OF DEBUG] Low Quality (%d) but forcing VALID to check Y-axis data\n",
        quality);
  }

  const double SCALE = 0.143 / 100.0; // cm/s to m/s
  double vx_sensor = vx_raw * SCALE;
  double vy_sensor = vy_raw * SCALE;

  // ===== BƯỚC 1: Chuyển từ hệ Optical Flow sang hệ Body =====
  // Optical Flow: x cùng chiều forward, y NGƯỢC chiều left
  // Body frame: x forward, y left
  // => vx_body = vx_sensor, vy_body = -vy_sensor
  double vx_body = vx_sensor;
  double vy_body = -vy_sensor; // ĐỔI DẤU Y

  // ===== BƯỚC 2: Hiệu chỉnh offset do sensor không ở tâm =====
  // Sensor ở vị trí (0, +0.15m, 0) trong body frame
  // v_center = v_sensor - omega × r
  // v_x_center = v_x_sensor + omega_z * r_y
  // v_y_center = v_y_sensor (vì r_x = 0)
  double vx_body_corrected = vx_body + omega_z * OPTICAL_FLOW_OFFSET_Y;
  double vy_body_corrected = vy_body; // Không thay đổi

  // ===== BƯỚC 3: Apply LPF Order 2 (trên velocity đã hiệu chỉnh) =====
  double vx_body_filtered = lpf_update(&of->lpf_vx, vx_body_corrected);
  double vy_body_filtered = lpf_update(&of->lpf_vy, vy_body_corrected);

  // ===== BƯỚC 4: Xoay từ Body frame sang Global frame =====
  // [vx_global]   [cos(theta)  -sin(theta)] [vx_body]
  // [vy_global] = [sin(theta)   cos(theta)] [vy_body]
  double cos_th = cos(theta);
  double sin_th = sin(theta);

  of->vx_global = cos_th * vx_body_filtered - sin_th * vy_body_filtered;
  of->vy_global = sin_th * vx_body_filtered + cos_th * vy_body_filtered;

  of->valid = true;
  return true;
}

// Lấy velocity đã filter
void optical_flow_get_velocity(const optical_flow_t *of, double *vx_global,
                               double *vy_global)
{
  if (of->valid)
  {
    *vx_global = of->vx_global;
    *vy_global = of->vy_global;
  }
  else
  {
    *vx_global = 0.0;
    *vy_global = 0.0;
  }
}
