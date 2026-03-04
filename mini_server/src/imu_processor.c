#include <float.h>
#include <math.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "imu_processor.h"

#define WINDOW_SIZE 5
#define CALIBRATION_SAMPLES 50   // Number of samples to collect for calibration
#define DEADBAND_THRESHOLD 0.02f // Threshold for soft deadband
#define DEADBAND_RANGE 0.03f     // Range for soft deadband scaling

// ===== Add to your includes / config =====
#define IMU_FS_HZ 100.0f // tần số mẫu (điền đúng theo thực tế)
#define IMU_DT (1.0f / IMU_FS_HZ)

// Bias tracker: LPF cực chậm (fc rất thấp) 0.02–0.05 Hz
#define BIAS_FC_HZ 0.03f
#define BIAS_ALPHA (1.0f - expf(-2.0f * (float)M_PI * BIAS_FC_HZ / IMU_FS_HZ))

// Jerk-gating: chỉ học bias nếu jerk nhỏ (tránh ăn motion thật)
#define JERK_THRESHOLD 4.0f      // m/s^3 (điều chỉnh theo data)
#define BIAS_CLAMP_ABS_MAX 0.15f // kẹp biên offset (m/s^2)

typedef struct {
  float values[WINDOW_SIZE];
  int index;
  bool filled;
} MedianWindow;

typedef struct {
  int w;      // window size (odd number, e.g., 9 or 11)
  int idx;    // circular index
  int count;  // filled count (<= w)
  float *buf; // ring buffer [w]
} HampelWindow;

// Structure to hold all IMU processing state
typedef struct {
  HampelWindow hx, hy, hz;
  float hx_buf[9]; // w=9
  float hy_buf[9];
  float hz_buf[9];

  // Median windows for accelerometer
  MedianWindow accel_x_window;
  MedianWindow accel_y_window;
  MedianWindow accel_z_window;

  // Calibration offsets
  float accel_offset_x;
  float accel_offset_y;
  float accel_offset_z;

  // Calibration state
  bool calibrating;
  int calibration_samples;
  float calibration_sum_accel_x;
  float calibration_sum_accel_y;
  float calibration_sum_accel_z;

  // State cho jerk-gating
  bool prev_accel_valid;
  float prev_ax, prev_ay, prev_az;

  // Nếu muốn đếm % thời gian học bias để giám sát
  unsigned int bias_update_count;
  unsigned int total_samples;

  pthread_mutex_t mutex;
} IMUProcessor;

static IMUProcessor g_imu_processor;

// ===================== H A M P E L   F I L T E R =====================

#ifndef HAMPel_MAD_TO_SIGMA
#define HAMPel_MAD_TO_SIGMA 1.4826f // ~ sqrt(π/2) for Gaussian
#endif

static int cmp_float_qsort(const void *a, const void *b) {
  float fa = *(const float *)a, fb = *(const float *)b;
  return (fa > fb) - (fa < fb);
}

// median of array[0..n-1] (n>=1), will qsort a local copy
static float median_of(const float *arr, int n) {
  float tmp[32]; // enough for w <= 32; adjust if you use larger w
  // If you want arbitrary w, malloc/free or static alloc
  for (int i = 0; i < n; ++i)
    tmp[i] = arr[i];
  qsort(tmp, n, sizeof(float), cmp_float_qsort);
  if (n & 1)
    return tmp[n / 2];
  return 0.5f * (tmp[n / 2 - 1] + tmp[n / 2]);
}

static inline void hampel_init(HampelWindow *hw, int window_size,
                               float *backing_buffer) {
  // window_size must be odd and <= size of backing_buffer
  hw->w = window_size;
  hw->idx = 0;
  hw->count = 0;
  hw->buf = backing_buffer;
  for (int i = 0; i < window_size; ++i)
    hw->buf[i] = 0.0f;
}

// Return filtered value; replaces outliers with window median
// k: sensitivity (typical 3.0). If MAD==0, no replace unless |x - med| > eps.
static inline float hampel_update(HampelWindow *hw, float x, float k) {
  // write new sample into ring
  hw->buf[hw->idx] = x;
  if (hw->count < hw->w)
    hw->count++;
  hw->idx = (hw->idx + 1) % hw->w;

  // compute median on the "valid" prefix
  int n = hw->count;
  float med = median_of(hw->buf, n);

  // build absolute deviation array
  float dev[32]; // enough for w<=32
  for (int i = 0; i < n; ++i) {
    float d = hw->buf[i] - med;
    dev[i] = d >= 0 ? d : -d;
  }
  float mad = median_of(dev, n); // median absolute deviation
  float sigma_hat = (mad > 0.0f) ? (HAMPel_MAD_TO_SIGMA * mad) : 0.0f;

  // threshold
  float thr = k * sigma_hat;

  // decide replacement
  float diff = x - med;
  if (diff < 0)
    diff = -diff;

  // If sigma_hat==0 (all equal), only replace if diff > tiny epsilon
  const float eps = 1e-6f;
  if ((sigma_hat > 0.0f && diff > thr) || (sigma_hat == 0.0f && diff > eps)) {
    return med; // outlier -> replace by median
  }
  return x; // keep original
}
// ================== E N D   H A M P E L   F I L T E R =================

// Helper function to initialize a median window
static void init_median_window(MedianWindow *window) {
  memset(window->values, 0, sizeof(window->values));
  window->index = 0;
  window->filled = false;
}

// Compare function for qsort
static int compare_floats(const void *a, const void *b) {
  float fa = *(const float *)a;
  float fb = *(const float *)b;
  return (fa > fb) - (fa < fb);
}

// Get median value from window
static float get_median(MedianWindow *window) {
  float sorted[WINDOW_SIZE];
  int count = window->filled ? WINDOW_SIZE : window->index;

  memcpy(sorted, window->values, sizeof(float) * count);
  qsort(sorted, count, sizeof(float), compare_floats);

  if (count % 2 == 0 && count > 0) {
    return (sorted[count / 2 - 1] + sorted[count / 2]) / 2.0f;
  } else if (count > 0) {
    return sorted[count / 2];
  }
  return 0.0f;
}

// Add a value to the median window
static float update_median_window(MedianWindow *window, float value) {
  window->values[window->index] = value;
  window->index = (window->index + 1) % WINDOW_SIZE;
  if (window->index == 0) {
    window->filled = true;
  }
  return get_median(window);
}

// Apply soft deadband filtering
static float apply_soft_deadband(float value, float threshold, float range) {
  float abs_value = fabsf(value);

  if (abs_value <= threshold) {
    return 0.0f; // Complete filtering for very small values
  } else if (abs_value <= threshold + range) {
    // Smoothly scale up as we move away from the threshold
    float scale = (abs_value - threshold) / range;
    return value * scale;
  }

  return value;
}

static inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

// Cập nhật bias (offset) theo EMA siêu chậm, chỉ khi "gating == true"
static inline void bias_ema_update_gated(float sample, float *bias,
                                         bool gating) {
  if (gating) {
    // EMA rất chậm
    *bias = (1.0f - BIAS_ALPHA) * (*bias) + BIAS_ALPHA * sample;
    // Kẹp biên để tránh drift dài hạn
    *bias = clampf(*bias, -BIAS_CLAMP_ABS_MAX, BIAS_CLAMP_ABS_MAX);
  }
}
// Initialize the IMU processor
bool imu_processor_init(void) {
  // Initialize mutex
  if (pthread_mutex_init(&g_imu_processor.mutex, NULL) != 0) {
    printf("[IMU_PROCESSOR] Failed to initialize mutex\n");
    return false;
  }

  // Initialize median windows
  init_median_window(&g_imu_processor.accel_x_window);
  init_median_window(&g_imu_processor.accel_y_window);
  init_median_window(&g_imu_processor.accel_z_window);

  hampel_init(&g_imu_processor.hx, 9, g_imu_processor.hx_buf);
  hampel_init(&g_imu_processor.hy, 9, g_imu_processor.hy_buf);
  hampel_init(&g_imu_processor.hz, 9, g_imu_processor.hz_buf);

  // Initialize calibration offsets to zero
  g_imu_processor.accel_offset_x = 0.0f;
  g_imu_processor.accel_offset_y = 0.0f;
  g_imu_processor.accel_offset_z = 0.0f;

  // Initialize calibration state
  g_imu_processor.calibrating = false;
  g_imu_processor.calibration_samples = 0;
  g_imu_processor.calibration_sum_accel_x = 0.0f;
  g_imu_processor.calibration_sum_accel_y = 0.0f;
  g_imu_processor.calibration_sum_accel_z = 0.0f;

  g_imu_processor.prev_accel_valid = false;
  g_imu_processor.prev_ax = g_imu_processor.prev_ay = g_imu_processor.prev_az =
      0.0f;
  g_imu_processor.bias_update_count = 0;
  g_imu_processor.total_samples = 0;
  printf("[IMU_PROCESSOR] Initialization complete\n");

  return true;
}
// Modified function that applies offsets and median window filtering
void imu_process_accel(float accel_x, float accel_y, float accel_z,
                       float *filtered_accel_x, float *filtered_accel_y,
                       float *filtered_accel_z) {
  pthread_mutex_lock(&g_imu_processor.mutex);

  bool gating = false;
  if (g_imu_processor.prev_accel_valid) {
    float dax = (accel_x - g_imu_processor.prev_ax) / IMU_DT;
    float day = (accel_y - g_imu_processor.prev_ay) / IMU_DT;
    float daz = (accel_z - g_imu_processor.prev_az) / IMU_DT;
    float jerk_norm = sqrtf(dax * dax + day * day + daz * daz);

    // Gating: jerk nhỏ ⇒ có thể coi a_true ~ 0 ⇒ học bias
    gating = (jerk_norm < JERK_THRESHOLD);
    // Cập nhật bias theo EMA siêu chậm
    bias_ema_update_gated(accel_x, &g_imu_processor.accel_offset_x, gating);
    bias_ema_update_gated(accel_y, &g_imu_processor.accel_offset_y, gating);
    bias_ema_update_gated(accel_z, &g_imu_processor.accel_offset_z, gating);

    // Thống kê (tùy chọn)
    g_imu_processor.total_samples++;
    if (gating)
      g_imu_processor.bias_update_count++;
  }

  g_imu_processor.prev_ax = accel_x; // giá trị TRƯỚC KHI trừ offset
  g_imu_processor.prev_ay = accel_y;
  g_imu_processor.prev_az = accel_z;
  g_imu_processor.prev_accel_valid = true;
  // Apply offsets
  accel_x -= g_imu_processor.accel_offset_x;
  accel_y -= g_imu_processor.accel_offset_y;
  accel_z -= g_imu_processor.accel_offset_z;

  accel_x = hampel_update(&g_imu_processor.hx, accel_x, 3.0f);
  accel_y = hampel_update(&g_imu_processor.hy, accel_y, 3.0f);
  accel_z = hampel_update(&g_imu_processor.hz, accel_z, 3.0f);

  // Update median windows and get median values
  *filtered_accel_x =
      update_median_window(&g_imu_processor.accel_x_window, accel_x);
  *filtered_accel_y =
      update_median_window(&g_imu_processor.accel_y_window, accel_y);
  *filtered_accel_z =
      update_median_window(&g_imu_processor.accel_z_window, accel_z);

  // // Apply soft deadband to filtered values
  *filtered_accel_x = apply_soft_deadband(*filtered_accel_x, DEADBAND_THRESHOLD,
                                          DEADBAND_RANGE);
  *filtered_accel_y = apply_soft_deadband(*filtered_accel_y, DEADBAND_THRESHOLD,
                                          DEADBAND_RANGE);
  *filtered_accel_z = apply_soft_deadband(*filtered_accel_z, DEADBAND_THRESHOLD,
                                          DEADBAND_RANGE);
  pthread_mutex_unlock(&g_imu_processor.mutex);
}
