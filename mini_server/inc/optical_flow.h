#ifndef OPTICAL_FLOW_H_
#define OPTICAL_FLOW_H_

#include <stdbool.h>
#include <stdint.h>

// Hệ số LPF Order 2, 2Hz cutoff @ 50Hz sampling
#define LPF_A1 1.647460
#define LPF_A2 -0.700897
#define LPF_B0 0.013359
#define LPF_B1 0.026718
#define LPF_B2 0.013359

// Quality threshold
#define OPTICAL_FLOW_MIN_QUALITY 10

// Sensor mounting offset from robot center (body frame)
#define OPTICAL_FLOW_OFFSET_Y 0.15 // 15cm along positive Y-axis

// Cấu trúc LPF Order 2 state
typedef struct {
  double y[2]; // y[n-1], y[n-2]
  double x[2]; // x[n-1], x[n-2]
  bool initialized;
} lpf_state_t;

// Cấu trúc Optical Flow Processor
typedef struct {
  lpf_state_t lpf_vx;
  lpf_state_t lpf_vy;

  // Velocity sau khi filter và transform (global frame)
  double vx_global;
  double vy_global;
  bool valid;
  uint8_t quality; // Signal quality (0-255)

  // Stats
  uint32_t frame_count;
  uint32_t rejected_count;
} optical_flow_t;

// Khởi tạo
void optical_flow_init(optical_flow_t *of);

// Xử lý dữ liệu raw từ sensor MTF-02
// vx_raw, vy_raw: giá trị raw từ sensor (int16_t)
// theta: góc hướng hiện tại của robot (radian)
// omega_z: vận tốc góc hiện tại (rad/s) - for offset correction
// quality: chất lượng tín hiệu
// Returns: true nếu data valid và đã filter
bool optical_flow_process(optical_flow_t *of, int16_t vx_raw, int16_t vy_raw,
                          double theta, double omega_z, uint8_t quality);

// Lấy velocity đã filter (global frame)
void optical_flow_get_velocity(const optical_flow_t *of, double *vx_global,
                               double *vy_global);

// === Integration functions (optical_flow_integration.c) ===

// Thread function
void *optical_flow_uart_thread(void *arg);

// Khởi tạo position ban đầu từ localization
void optical_flow_set_initial_position(double x, double y);

#endif // OPTICAL_FLOW_H_
