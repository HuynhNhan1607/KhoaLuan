/**
 * @file vl53l0x_manager.c
 * @brief High-level dual VL53L0X manager — implementation
 *
 * Encapsulates:
 *   - GPIO XSHUT control via libgpiod (replaces stand-alone gpio_helper usage)
 *   - Dual-sensor I2C address assignment sequence
 *   - 5-case docking state machine with velocity command output
 */

#include "vl53l0x_manager.h"
#include "gpio_helper.h"
#include "vl53l0x_c.h"

#include <math.h> /* fabsf  */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h> /* usleep */

/* ═══════════════════════════════════════════════════════════════════════════
 * INTERNAL STRUCT (opaque to callers)
 * ═══════════════════════════════════════════════════════════════════════════
 */

struct VL53L0XManager
{
  /* Sensors */
  VL53L0X sensor_left;
  VL53L0X sensor_right;

  /* GPIO handles for XSHUT lines */
  GpioHandle gpio_left;
  GpioHandle gpio_right;

  /* Saved configuration */
  DockConfig config;

  /* State machine */
  DockState state;
  bool initialized;

  /* Search state */
  int search_dir;         /* +1 = right, -1 = left */
  int search_counter;     /* cycles in current direction */
  bool search_tried_both; /* true = already tried both directions */
};

/* ═══════════════════════════════════════════════════════════════════════════
 * HELPERS
 * ═══════════════════════════════════════════════════════════════════════════
 */

static inline float clampf(float v, float lo, float hi)
{
  if (v < lo)
    return lo;
  if (v > hi)
    return hi;
  return v;
}

static inline int16_t abs16(int16_t v) { return (v < 0) ? -v : v; }

/* ═══════════════════════════════════════════════════════════════════════════
 * DEFAULT CONFIG
 * ═══════════════════════════════════════════════════════════════════════════
 */

DockConfig vl53l0x_manager_default_config(void)
{
  DockConfig c;
  memset(&c, 0, sizeof(c));

  /* I2C */
  c.i2c_bus = 1;      /* I2C bus 1 */
  c.addr_left = 0x30; /* Đổi LEFT từ 0x29 thành 0x30   */

  /* GPIO (libgpiod) — gpiochip1 trên Xavier NX */
  c.gpio_chip = "gpiochip1";
  c.gpio_line_left = 108;  /* Board Pin 31 = GPIO11 */
  c.gpio_line_right = 118; /* Board Pin 33 = GPIO13 ← kiểm tra lại! */

  /* Detection thresholds (mm) */
  c.max_range_mm = 800;     /* VL53L0X max ~2m, nhưng đáng tin ≤1.2m */
  c.yaw_tolerance_mm = 10;  /* |delta| ≤ 5 mm = coi như thẳng hàng   */
  c.yaw_realign_mm = 15;    /* Trong lúc tiến, re-align nếu |delta|>15 */
  c.dock_distance_mm = 130; /* Khoảng cách mục tiêu gắp               */
  c.dock_tolerance_mm = 10; /* ± 3 mm quanh dock_distance              */

  /* Speeds (m/s for linear, rad/s for angular) */
  c.search_vy = 0.05f;        /* Tốc độ trượt ngang khi tìm (m/s)     */
  c.center_vy = 0.05f;        /* Tốc độ trượt ngang khi căn tâm (m/s) */
  c.align_omega = 0.1f;       /* Tốc độ xoay khi căn yaw (rad/s)      */
  c.approach_vx = 0.05f;      /* Tốc độ tiến tới mục tiêu (m/s)       */
  c.approach_yaw_gain = 0.5f; /* Hệ số chỉnh yaw khi đang tiến       */

  /* Search */
  c.search_direction = -1; /* +1 = trượt sang phải trước      */

  /* Calibration offsets — đo thực nghiệm:
   * Đặt 2 cảm biến cùng khoảng cách, ghi nhận chênh lệch.
   * RIGHT cho ra +20 mm so với LEFT → bù -20 mm cho RIGHT. */
  c.calib_offset_left_mm = 0;    /* LEFT chuẩn, không cần bù    */
  c.calib_offset_right_mm = -15; /* RIGHT nhiễu +20 mm → trừ 20 */

  /* Sensor IO timeout — must be > VL53L0X minimum measurement time (33ms). */
  /* With TCP sends in the same thread, CPU contention can delay I2C polling */
  /* → need extra margin.  100ms is NOT enough when the server has heavy     */
  /* multi-thread load (TCP, EKF, motor control).  Use 500ms like TestVL53.  */
  c.sensor_timeout_ms = 500;

  return c;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * INIT  — GPIO XSHUT sequence + dual sensor address assignment
 * ═══════════════════════════════════════════════════════════════════════════
 */

VL53L0XManager *vl53l0x_manager_init(const DockConfig *config)
{
  VL53L0XManager *mgr = (VL53L0XManager *)calloc(1, sizeof(VL53L0XManager));
  if (!mgr)
  {
    fprintf(stderr, "[manager] Loi: khong du bo nho!\n");
    return NULL;
  }
  mgr->config = *config;
  mgr->state = DOCK_STATE_SEARCHING;
  mgr->initialized = false;

  /* Dia chi da duoc set san boi vl53l0x_addr_init truoc khi chay server.
   * Manager chi can mo I2C tai dung dia chi, khong can dung GPIO XSHUT. */
  printf("[manager] Ket noi cam bien: LEFT=0x%02X  RIGHT=0x29  bus=%d\n",
         config->addr_left, config->i2c_bus);

  /* ─── Mở LEFT tại addr_left đã set sẵn ─── */
  vl53l0x_create(&mgr->sensor_left);
  mgr->sensor_left.i2c_bus = config->i2c_bus;
  mgr->sensor_left.address = config->addr_left;
  if (!vl53l0x_open(&mgr->sensor_left))
  {
    fprintf(stderr, "[manager] Loi: Khong mo I2C LEFT 0x%02X (errno=%d)\n",
            config->addr_left, mgr->sensor_left.error);
    goto fail;
  }
  vl53l0x_set_timeout(&mgr->sensor_left, config->sensor_timeout_ms);
  if (!vl53l0x_init(&mgr->sensor_left, true))
  {
    fprintf(stderr, "[manager] Loi: Khong init duoc LEFT!\n");
    goto fail;
  }
  printf("[manager] LEFT  OK (addr=0x%02X)\n", vl53l0x_get_address(&mgr->sensor_left));

  /* ─── Mở RIGHT tại 0x29 (mặc định) ─── */
  vl53l0x_create(&mgr->sensor_right);
  mgr->sensor_right.i2c_bus = config->i2c_bus;
  if (!vl53l0x_open(&mgr->sensor_right))
  {
    fprintf(stderr, "[manager] Loi: Khong mo I2C RIGHT 0x29 (errno=%d)\n",
            mgr->sensor_right.error);
    goto fail;
  }
  vl53l0x_set_timeout(&mgr->sensor_right, config->sensor_timeout_ms);
  if (!vl53l0x_init(&mgr->sensor_right, true))
  {
    fprintf(stderr, "[manager] Loi: Khong init duoc RIGHT!\n");
    goto fail;
  }
  printf("[manager] RIGHT OK (addr=0x29)\n");

  printf("[manager] === HOAN TAT ===\n\n");
  mgr->initialized = true;
  return mgr;

fail:
  vl53l0x_close(&mgr->sensor_left);
  vl53l0x_close(&mgr->sensor_right);
  free(mgr);
  return NULL;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * SHUTDOWN
 * ═══════════════════════════════════════════════════════════════════════════
 */

void vl53l0x_manager_shutdown(VL53L0XManager *mgr)
{
  if (!mgr)
    return;

  vl53l0x_close(&mgr->sensor_left);
  vl53l0x_close(&mgr->sensor_right);

  printf("[manager] Da dong tat ca ket noi.\n");
  free(mgr);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * RESET
 * ═══════════════════════════════════════════════════════════════════════════
 */

void vl53l0x_manager_reset(VL53L0XManager *mgr)
{
  if (mgr)
  {
    mgr->state = DOCK_STATE_SEARCHING;
    mgr->search_dir = mgr->config.search_direction;
    mgr->search_counter = 0;
    mgr->search_tried_both = false;
  }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * UPDATE  — Máy trạng thái docking chính
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * Ưu tiên trạng thái (từ cao → thấp):
 *   1. Cả hai mất    → SEARCHING         (Case 1)
 *   2. Một bên mất   → CENTERING_X       (Case 2)
 *   3. Quá sát       → BACKING_Y         (Case 4b) ← trước ALIGNING để tránh cọ vật
 *   4. Lệch yaw lớn  → ALIGNING_YAW      (Case 3)
 *   5. Còn xa        → APPROACHING_Y     (Case 4)
 *   6. Đúng vị trí   → DOCKED            (Case 5)
 *
 * Nhờ thứ tự ưu tiên này, các chuyển trạng thái ngược tự
 * động xảy ra, ví dụ: đang xoay mà 1 bên trượt ra → ngay
 * lập tức quay về CENTERING_X (Case 3 → Case 2).
 *
 * BACKING_Y được ưu tiên trước ALIGNING_YAW vì khi quá sát
 * mà xoay tại chỗ thì dễ cọ vào vật bằng góc của robot.
 */

DockStatus vl53l0x_manager_update(VL53L0XManager *mgr)
{
  DockStatus st;
  memset(&st, 0, sizeof(st));

  if (!mgr || !mgr->initialized)
  {
    st.state = DOCK_STATE_ERROR;
    return st;
  }

  const DockConfig *cfg = &mgr->config;

  /* ── 1. Đọc cả hai cảm biến (+ áp calibration offset) ──────────── */

  {
    uint16_t raw_l = vl53l0x_read_range_single_mm(&mgr->sensor_left);
    uint16_t raw_r = vl53l0x_read_range_single_mm(&mgr->sensor_right);

    /* Áp offset: cộng giá trị có dấu, kẹp về 0 để tránh underflow uint16 */
    int32_t cal_l = (int32_t)raw_l + cfg->calib_offset_left_mm;
    int32_t cal_r = (int32_t)raw_r + cfg->calib_offset_right_mm;
    st.dist_left_mm = (uint16_t)(cal_l < 0 ? 0 : cal_l);
    st.dist_right_mm = (uint16_t)(cal_r < 0 ? 0 : cal_r);
  }

  bool timeout_l = vl53l0x_timeout_occurred(&mgr->sensor_left);
  bool timeout_r = vl53l0x_timeout_occurred(&mgr->sensor_right);

  st.left_valid = !timeout_l && (st.dist_left_mm < cfg->max_range_mm);
  st.right_valid = !timeout_r && (st.dist_right_mm < cfg->max_range_mm);

  /* ── 2. Tính delta và trung bình ────────────────────────────────── */

  bool both_valid = st.left_valid && st.right_valid;
  bool none_valid = !st.left_valid && !st.right_valid;

  if (both_valid)
  {
    st.delta_mm = (int16_t)st.dist_left_mm - (int16_t)st.dist_right_mm;
    st.avg_distance_mm = (st.dist_left_mm + st.dist_right_mm) / 2;
  }

  /* ── 3. Xác định trạng thái mới theo thứ tự ưu tiên ────────────── */

  bool yaw_ok;
  bool at_dock;

  if (none_valid)
  {
    /* ╔═══════════════════════════════════════════════════════╗
     * ║  Case 1: MẤT DẤU MỤC TIÊU HOÀN TOÀN                ║
     * ║  Cả hai cảm biến không thấy vật.                     ║
     * ║  → Trượt ngang chậm theo trục X để rà quét.          ║
     * ╚═══════════════════════════════════════════════════════╝ */
    mgr->state = DOCK_STATE_SEARCHING;
  }
  else if (!both_valid)
  {
    /* ╔═══════════════════════════════════════════════════════╗
     * ║  Case 2: LỆCH TÂM NGANG TRỤC X                     ║
     * ║  Một cảm biến thấy, một trượt ra ngoài.             ║
     * ║  → Trượt ngang về phía cảm biến bị trượt,           ║
     * ║    cho đến khi cả hai tia laser chạm bề mặt vật.    ║
     * ╚═══════════════════════════════════════════════════════╝ */
    mgr->state = DOCK_STATE_CENTERING_X;
  }
  else
  {
    /* Cả hai đều thấy — kiểm tra yaw và khoảng cách */

    /* Trong lúc APPROACHING/BACKING nếu |delta| < yaw_realign thì vẫn giữ
     * trạng thái đó, chỉ chuyển ALIGNING_YAW khi delta vượt ngưỡng lớn hơn.
     * Ngoài hai trạng thái đó, dùng yaw_tolerance bình thường. */
    uint16_t yaw_thresh =
        (mgr->state == DOCK_STATE_APPROACHING_Y ||
         mgr->state == DOCK_STATE_BACKING_Y)
            ? cfg->yaw_realign_mm
            : cfg->yaw_tolerance_mm;

    yaw_ok = (abs16(st.delta_mm) <= (int16_t)yaw_thresh);
    at_dock =
        (abs16(st.delta_mm) <= (int16_t)cfg->yaw_tolerance_mm) &&
        (st.avg_distance_mm >=
         cfg->dock_distance_mm - cfg->dock_tolerance_mm) &&
        (st.avg_distance_mm <= cfg->dock_distance_mm + cfg->dock_tolerance_mm);

    /* Robot đang quá sát vật (chưa tính dung sai) */
    bool too_close =
        (st.avg_distance_mm < cfg->dock_distance_mm - cfg->dock_tolerance_mm);

    if (at_dock)
    {
      /* ╔═══════════════════════════════════════════════════════╗
       * ║  Case 5: HOÀN THÀNH ĐỊNH VỊ                         ║
       * ║  Thẳng hàng, ngay tâm, đúng khoảng cách.            ║
       * ║  → Dừng hẳn, gửi tín hiệu kích hoạt tay gắp.       ║
       * ╚═══════════════════════════════════════════════════════╝ */
      mgr->state = DOCK_STATE_DOCKED;
    }
    else if (too_close)
    {
      /* ╔═══════════════════════════════════════════════════════╗
       * ║  Case 4b: QUÁ SÁT MỤC TIÊU                         ║
       * ║  avg_distance < dock_distance - tolerance.           ║
       * ║  → Lùi thẳng ra theo trục Y cho đến khi đủ xa.      ║
       * ║  Ưu tiên trước ALIGNING_YAW để tránh cọ vật khi     ║
       * ║  xoay tại chỗ lúc còn quá gần.                      ║
       * ╚═══════════════════════════════════════════════════════╝ */
      mgr->state = DOCK_STATE_BACKING_Y;
    }
    else if (!yaw_ok)
    {
      /* ╔═══════════════════════════════════════════════════════╗
       * ║  Case 3: LỆCH GÓC XOAY YAW                         ║
       * ║  Chênh lệch khoảng cách quá lớn → robot đang xéo.   ║
       * ║  → Xoay tại chỗ để làm phẳng.                       ║
       * ║  Nếu 1 bên trượt ra → quay về Case 2 (tự động).     ║
       * ╚═══════════════════════════════════════════════════════╝ */
      mgr->state = DOCK_STATE_ALIGNING_YAW;
    }
    else
    {
      /* ╔═══════════════════════════════════════════════════════╗
       * ║  Case 4: TIẾN VÀO MỤC TIÊU TRỤC Y                  ║
       * ║  Đã thẳng hàng nhưng còn xa tầm gắp.                ║
       * ║  → Tiến thẳng, liên tục tinh chỉnh yaw.             ║
       * ╚═══════════════════════════════════════════════════════╝ */
      mgr->state = DOCK_STATE_APPROACHING_Y;
    }
  }

  /* ── 4. Sinh lệnh vận tốc ───────────────────────────────────────── */

  st.state = mgr->state;
  st.cmd.vx = 0.0f;
  st.cmd.vy = 0.0f;
  st.cmd.omega = 0.0f;
  st.docking_complete = false;

  switch (mgr->state)
  {

  case DOCK_STATE_SEARCHING:
  {
/*
 * Tìm từ vị trí gốc (0):
 *   Bước 1: phải N cycles    →  0 → +N
 *   Bước 2: trái 2N cycles   → +N → 0 → -N
 *   Không thấy → NOT_FOUND, dừng.
 *
 * vl53l0x_manager_update() được gọi ở 10Hz (rate-limited trong docking_update).
 * SEARCH_N_CYCLES = 20 → Bước 1 ~2s, Bước 2 ~4s, tổng ~6s tìm kiếm.
 */
#define SEARCH_N_CYCLES 20 /* ~2s ở 10Hz */
    mgr->search_counter++;

    if (!mgr->search_tried_both)
    {
      /* Bước 1: đi phải N cycles */
      if (mgr->search_counter >= SEARCH_N_CYCLES)
      {
        /* Hết bước 1 → đổi trái, chạy 2N */
        mgr->search_dir = -mgr->search_dir;
        mgr->search_counter = 0;
        mgr->search_tried_both = true;
      }
    }
    else
    {
      /* Bước 2: đi trái 2N cycles (+N → 0 → -N) */
      if (mgr->search_counter >= SEARCH_N_CYCLES * 2)
      {
        /* Hết cả 2 bước → không tìm thấy */
        mgr->state = DOCK_STATE_NOT_FOUND;
        st.state = DOCK_STATE_NOT_FOUND;
        st.cmd.vx = 0.0f;
        st.cmd.vy = 0.0f;
        st.cmd.omega = 0.0f;
        break;
      }
    }
    st.cmd.vx = (float)mgr->search_dir * cfg->search_vy;
    break;
  }

  case DOCK_STATE_CENTERING_X:
    /*
     * Nếu LEFT thấy nhưng RIGHT không → robot lệch sang TRÁI
     *   → cần trượt sang PHẢI (vy > 0)
     * Nếu RIGHT thấy nhưng LEFT không → robot lệch sang PHẢI
     *   → cần trượt sang TRÁI  (vy < 0)
     */
    if (st.left_valid && !st.right_valid)
    {
      st.cmd.vx = -cfg->center_vy;
    }
    else
    {
      st.cmd.vx = cfg->center_vy;
    }
    break;

  case DOCK_STATE_ALIGNING_YAW:
    /*
     * delta > 0  →  left xa hơn right  →  robot xéo (trái xa)
     *   → xoay CW (omega < 0) để kéo trái lại gần
     * delta < 0  →  right xa hơn left
     *   → xoay CCW (omega > 0)
     */
    if (st.delta_mm > 0)
    {
      st.cmd.omega = -cfg->align_omega;
    }
    else
    {
      st.cmd.omega = cfg->align_omega;
    }
    break;

  case DOCK_STATE_APPROACHING_Y:
    /* Tiến thẳng (heading = Y) + tinh chỉnh yaw theo tỷ lệ */
    st.cmd.vy = cfg->approach_vx;

    // if (abs16(st.delta_mm) > 1)
    // {
    //   float ratio = (float)st.delta_mm / (float)cfg->yaw_realign_mm;
    //   ratio = clampf(ratio, -1.0f, 1.0f);
    //   st.cmd.omega = -ratio * cfg->align_omega * cfg->approach_yaw_gain;
    // }
    break;

  case DOCK_STATE_BACKING_Y:
    /* Lùi ra (heading = −Y) khi robot vô tình vào quá gần mục tiêu.
     * Dùng cùng tốc độ với approach_vx để đối xứng.
     * Máy trạng thái sẽ tự chuyển sang APPROACHING_Y hoặc DOCKED
     * khi avg_distance quay về vùng hợp lệ. */
    st.cmd.vy = -cfg->approach_vx;
    break;

  case DOCK_STATE_DOCKED:
    /* Vận tốc = 0, kích hoạt tín hiệu gắp */
    st.docking_complete = true;
    break;

  case DOCK_STATE_ERROR:
  default:
    break;
  }

  return st;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * UTILITIES
 * ═══════════════════════════════════════════════════════════════════════════
 */

const char *dock_state_name(DockState state)
{
  switch (state)
  {
  case DOCK_STATE_SEARCHING:
    return "SEARCHING      (Case 1)";
  case DOCK_STATE_CENTERING_X:
    return "CENTERING_X    (Case 2)";
  case DOCK_STATE_ALIGNING_YAW:
    return "ALIGNING_YAW   (Case 3)";
  case DOCK_STATE_APPROACHING_Y:
    return "APPROACHING_Y  (Case 4)";
  case DOCK_STATE_BACKING_Y:
    return "BACKING_Y      (Case 4b)";
  case DOCK_STATE_DOCKED:
    return "DOCKED         (Case 5)";
  case DOCK_STATE_NOT_FOUND:
    return "NOT_FOUND";
  case DOCK_STATE_ERROR:
    return "ERROR";
  default:
    return "UNKNOWN";
  }
}
