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

  /* Sensor debounce — only declare invalid after N consecutive bad reads */
  int invalid_cnt_left;         /* consecutive out-of-range / error reads, LEFT  */
  int invalid_cnt_right;        /* consecutive out-of-range / error reads, RIGHT */
  uint16_t last_valid_left_mm;  /* last confirmed valid reading, LEFT  */
  uint16_t last_valid_right_mm; /* last confirmed valid reading, RIGHT */

  /* I2C timeout recovery — track consecutive TIMEOUT/I2C_ERROR specifically */
  int timeout_cnt_left;  /* consecutive timeout reads, LEFT  */
  int timeout_cnt_right; /* consecutive timeout reads, RIGHT */

  /* Continuous mode flag */
  bool continuous_started; /* true after vl53l0x_start_continuous() called */
};

/* ═══════════════════════════════════════════════════════════════════════════
 * HELPERS
 * ═══════════════════════════════════════════════════════════════════════════
 */

/* 0x1FFE (8190) = VL53L0X phase-failure sentinel; 0xFFFF = I2C error */
#define VL53L0X_PHASE_FAIL 8190u
#define VL53L0X_I2C_ERROR 65535u
/* Minimum consecutive bad reads before the sensor is declared invalid */
#define SENSOR_INVALID_DEBOUNCE 3
/* VL53L0X minimum physical range ~30mm; 0 = I2C bus returned all-zeros */
#define VL53L0X_MIN_RANGE_MM 10u
/* Consecutive invalid reads before triggering XSHUT power-cycle.
 * Lowered to 3 because Jetson I2C is more sensitive to wire glitches
 * than ESP32, and we want fast recovery before robot drifts too far. */
#define TIMEOUT_RECOVERY_THRESHOLD 3

/* ── Continuous-mode measurement period (ms) ──────────────────────────
 * CRITICAL FIX for dual-sensor I2C bus contention.
 *
 * Back-to-back mode (period=0): both sensors measure NON-STOP and
 * continuously stress the shared I2C bus.  Each sensor's internal
 * state machine clock-stretches SCL during measurement (~33ms per
 * cycle), so with TWO sensors the bus is ALWAYS under contention.
 * This is why 1 sensor works but 2 sensors fail.
 *
 * Timed mode (period=50ms): each sensor measures every 50ms with a
 * ~17ms idle window between measurements.  The bus gets breathing
 * room, and clock-stretching conflicts are minimised.
 *
 * Must be < docking loop period (100ms) to ensure fresh data each cycle.
 * ──────────────────────────────────────────────────────────────────── */
#define CONTINUOUS_PERIOD_MS 50

static inline float clampf(float v, float lo, float hi)
{
  if (v < lo)
    return lo;
  if (v > hi)
    return hi;
  return v;
}

static inline int16_t abs16(int16_t v) { return (v < 0) ? -v : v; }

/* ── Helper: attempt to re-initialise a single sensor ─────────────────── */
/* Returns true if the sensor is alive and re-init succeeded.              */
static bool reinit_sensor(VL53L0X *sensor, const char *name, uint16_t timeout_ms)
{
  /* Step 1: Check if the sensor responds at all by reading model ID.
   * VL53L0X always returns 0xEE for register 0xC0.                       */
  uint8_t model_id = vl53l0x_read_reg(sensor, VL53L0X_REG_IDENTIFICATION_MODEL_ID);
  if (model_id != 0xEE)
  {
    printf("[manager] %s: I2C check FAILED (model_id=0x%02X, expected 0xEE) "
           "— sensor not responding at addr 0x%02X\n",
           name, model_id, sensor->address);
    return false;
  }

  printf("[manager] %s: I2C OK (model_id=0xEE) — re-initialising...\n", name);

  /* Step 2: Full re-init (resets all internal registers + calibration) */
  vl53l0x_set_timeout(sensor, timeout_ms);
  if (!vl53l0x_init(sensor, true))
  {
    printf("[manager] %s: re-init FAILED\n", name);
    return false;
  }

  printf("[manager] %s: re-init OK\n", name);
  return true;
}

/* ── Helper: full XSHUT power-cycle recovery for both sensors ─────────── */
/* When motor EMI causes I2C bus lockup (SDA stuck LOW), a simple re-init  */
/* won't work because I2C transactions themselves fail. The only reliable  */
/* recovery is to power-cycle both sensors via XSHUT toggling, which       */
/* resets the I2C slave state machines inside the VL53L0X chips.           */
static bool power_cycle_recovery(VL53L0XManager *mgr)
{
  printf("[manager] *** I2C BUS RECOVERY: power-cycling both sensors via XSHUT ***\n");

  const DockConfig *cfg = &mgr->config;

  /* Step 1: Close existing I2C file descriptors */
  vl53l0x_close(&mgr->sensor_left);
  vl53l0x_close(&mgr->sensor_right);

  /* Step 2: Pull XSHUT LOW on both sensors → power off */
  gpio_write(&mgr->gpio_left, 0);
  gpio_write(&mgr->gpio_right, 0);
  usleep(50000); /* 50ms — ensure full power-down + I2C bus release */

  /* Step 3: Wake LEFT sensor first (same sequence as init) */
  gpio_write(&mgr->gpio_left, 1);
  usleep(10000); /* 10ms boot time */

  vl53l0x_create(&mgr->sensor_left);
  mgr->sensor_left.i2c_bus = cfg->i2c_bus;
  if (!vl53l0x_open(&mgr->sensor_left))
  {
    printf("[manager] RECOVERY FAILED: cannot open LEFT I2C\n");
    return false;
  }

  /* Change LEFT address (it boots at default 0x29) */
  vl53l0x_set_address(&mgr->sensor_left, cfg->addr_left);
  usleep(2000);

  vl53l0x_set_timeout(&mgr->sensor_left, cfg->sensor_timeout_ms);
  if (!vl53l0x_init(&mgr->sensor_left, true))
  {
    printf("[manager] RECOVERY FAILED: LEFT init failed\n");
    return false;
  }
  printf("[manager] RECOVERY: LEFT OK (addr=0x%02X)\n", cfg->addr_left);

  /* Step 4: Wake RIGHT sensor */
  gpio_write(&mgr->gpio_right, 1);
  usleep(10000);

  vl53l0x_create(&mgr->sensor_right);
  mgr->sensor_right.i2c_bus = cfg->i2c_bus;
  if (!vl53l0x_open(&mgr->sensor_right))
  {
    printf("[manager] RECOVERY FAILED: cannot open RIGHT I2C\n");
    return false;
  }

  vl53l0x_set_timeout(&mgr->sensor_right, cfg->sensor_timeout_ms);
  if (!vl53l0x_init(&mgr->sensor_right, true))
  {
    printf("[manager] RECOVERY FAILED: RIGHT init failed\n");
    return false;
  }
  printf("[manager] RECOVERY: RIGHT OK (addr=0x29)\n");

  /* Step 5: Warm-up reads (discard first 2 readings) */
  for (int i = 0; i < 2; i++)
  {
    vl53l0x_read_range_single_mm(&mgr->sensor_left);
    vl53l0x_timeout_occurred(&mgr->sensor_left);
    vl53l0x_read_range_single_mm(&mgr->sensor_right);
    vl53l0x_timeout_occurred(&mgr->sensor_right);
    usleep(50000);
  }

  /* Start continuous timed mode on recovered sensors */
  vl53l0x_start_continuous(&mgr->sensor_left, CONTINUOUS_PERIOD_MS);
  usleep(25000); /* 25ms offset — stagger measurement windows */
  vl53l0x_start_continuous(&mgr->sensor_right, CONTINUOUS_PERIOD_MS);
  mgr->continuous_started = true;

  /* Reset counters */
  mgr->timeout_cnt_left = 0;
  mgr->timeout_cnt_right = 0;
  mgr->invalid_cnt_left = 0;
  mgr->invalid_cnt_right = 0;
  mgr->last_valid_left_mm = 0;
  mgr->last_valid_right_mm = 0;

  printf("[manager] *** RECOVERY COMPLETE (continuous mode) ***\n");
  return true;
}

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
  c.max_range_mm = 1200;    /* VL53L0X max ~2m, nhưng đáng tin ≤1.2m */
  c.yaw_tolerance_mm = 10;  /* |delta| ≤ 5 mm = coi như thẳng hàng   */
  c.yaw_realign_mm = 15;    /* Trong lúc tiến, re-align nếu |delta|>15 */
  c.dock_distance_mm = 130; /* Khoảng cách mục tiêu gắp               */
  c.dock_tolerance_mm = 5;  /* ± 3 mm quanh dock_distance              */

  /* Speeds (m/s for linear, rad/s for angular) */
  c.search_vy = 0.05f;        /* Tốc độ trượt ngang khi tìm (m/s)     */
  c.center_vy = 0.05f;        /* Tốc độ trượt ngang khi căn tâm (m/s) */
  c.align_omega = 0.1f;       /* Tốc độ xoay khi căn yaw (rad/s)      */
  c.approach_vx = 0.05f;      /* Tốc độ tiến tới mục tiêu (m/s)       */
  c.approach_yaw_gain = 0.5f; /* Hệ số chỉnh yaw khi đang tiến       */

  /* Search */
  c.search_direction = 1; /* +1 = trượt sang phải trước      */

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

  printf("[manager] === KHOI TAO DUAL VL53L0X ===\n");
  printf("[manager] I2C bus       : %d\n", config->i2c_bus);
  printf("[manager] GPIO chip     : %s\n", config->gpio_chip);
  printf("[manager] XSHUT LEFT    : line %u\n", config->gpio_line_left);
  printf("[manager] XSHUT RIGHT   : line %u\n", config->gpio_line_right);
  printf("[manager] Addr LEFT     : 0x%02X\n", config->addr_left);
  printf("[manager] Addr RIGHT    : 0x29 (default)\n\n");

  /* ─── Mở GPIO cho XSHUT (ban đầu = LOW → tắt cả hai) ─── */
  if (!gpio_open_output(&mgr->gpio_left, config->gpio_chip,
                        config->gpio_line_left, 0, "vl53l0x-xshut-left"))
  {
    fprintf(stderr, "[manager] Loi: Khong mo duoc GPIO XSHUT LEFT\n");
    free(mgr);
    return NULL;
  }
  if (!gpio_open_output(&mgr->gpio_right, config->gpio_chip,
                        config->gpio_line_right, 0, "vl53l0x-xshut-right"))
  {
    fprintf(stderr, "[manager] Loi: Khong mo duoc GPIO XSHUT RIGHT\n");
    gpio_close(&mgr->gpio_left);
    free(mgr);
    return NULL;
  }

  /* ─── Bước 1: Tắt cả hai (Reset) ─── */
  printf("[manager] [B1] Tat ca hai cam bien (XSHUT = LOW)...\n");
  gpio_write(&mgr->gpio_left, 0);
  gpio_write(&mgr->gpio_right, 0);
  usleep(50000); /* 10 ms */

  /* ─── Bước 2: Đánh thức con TRÁI ─── */
  printf("[manager] [B2] Danh thuc cam bien TRAI...\n");
  gpio_write(&mgr->gpio_left, 1);
  usleep(50000); /* 5 ms chờ khởi động */

  /* Mở I2C – lúc này TRÁI đang ở 0x29 */
  vl53l0x_create(&mgr->sensor_left);
  mgr->sensor_left.i2c_bus = config->i2c_bus;
  if (!vl53l0x_open(&mgr->sensor_left))
  {
    fprintf(stderr,
            "[manager] Loi: Khong mo duoc I2C cho cam bien TRAI (errno=%d)\n",
            mgr->sensor_left.error);
    goto fail;
  }

  /* ─── Bước 3: Đổi địa chỉ con TRÁI ─── */
  printf("[manager] [B3] Doi dia chi TRAI: 0x29 -> 0x%02X...\n",
         config->addr_left);
  vl53l0x_set_address(&mgr->sensor_left, config->addr_left);
  usleep(2000);

  /* Init TRÁI */
  printf("[manager]      Khoi tao cam bien TRAI...\n");
  vl53l0x_set_timeout(&mgr->sensor_left, config->sensor_timeout_ms);
  if (!vl53l0x_init(&mgr->sensor_left, true))
  {
    fprintf(stderr, "[manager] Loi: Khong init duoc cam bien TRAI!\n");
    goto fail;
  }
  printf("[manager]      TRAI OK (addr=0x%02X)\n\n",
         vl53l0x_get_address(&mgr->sensor_left));

  /* ─── Bước 4: Đánh thức con PHẢI ─── */
  printf("[manager] [B4] Danh thuc cam bien PHAI...\n");
  gpio_write(&mgr->gpio_right, 1);
  usleep(5000);

  /* Mở I2C – PHẢI thức dậy với 0x29 */
  vl53l0x_create(&mgr->sensor_right);
  mgr->sensor_right.i2c_bus = config->i2c_bus;
  if (!vl53l0x_open(&mgr->sensor_right))
  {
    fprintf(stderr,
            "[manager] Loi: Khong mo duoc I2C cho cam bien PHAI (errno=%d)\n",
            mgr->sensor_right.error);
    goto fail;
  }

  /* Init PHẢI */
  printf("[manager]      Khoi tao cam bien PHAI...\n");
  vl53l0x_set_timeout(&mgr->sensor_right, config->sensor_timeout_ms);
  if (!vl53l0x_init(&mgr->sensor_right, true))
  {
    fprintf(stderr, "[manager] Loi: Khong init duoc cam bien PHAI!\n");
    goto fail;
  }
  printf("[manager]      PHAI OK (addr=0x29)\n\n");

  /* ─── Hoàn tất ─── */
  printf("[manager] === HOAN TAT ===\n");
  printf("[manager]   TRAI : 0x%02X\n", vl53l0x_get_address(&mgr->sensor_left));
  printf("[manager]   PHAI : 0x29\n\n");

  /* ─── Start continuous TIMED ranging mode ─── */
  /* Timed mode (period=50ms) instead of back-to-back (0) is critical
   * for dual-sensor I2C stability.  Back-to-back causes both sensors
   * to clock-stretch the shared bus simultaneously → contention.
   * Stagger start by 25ms so their measurement windows don't overlap. */
  vl53l0x_start_continuous(&mgr->sensor_left, CONTINUOUS_PERIOD_MS);
  usleep(25000); /* 25ms offset — stagger LEFT/RIGHT measurement windows */
  vl53l0x_start_continuous(&mgr->sensor_right, CONTINUOUS_PERIOD_MS);
  mgr->continuous_started = true;
  printf("[manager] Continuous timed mode started (period=%dms, staggered)\n",
         CONTINUOUS_PERIOD_MS);

  mgr->initialized = true;
  return mgr;

fail:
  vl53l0x_close(&mgr->sensor_left);
  vl53l0x_close(&mgr->sensor_right);
  gpio_close(&mgr->gpio_left);
  gpio_close(&mgr->gpio_right);
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

  /* Stop continuous mode before closing */
  if (mgr->continuous_started)
  {
    vl53l0x_stop_continuous(&mgr->sensor_left);
    vl53l0x_stop_continuous(&mgr->sensor_right);
    mgr->continuous_started = false;
  }

  vl53l0x_close(&mgr->sensor_left);
  vl53l0x_close(&mgr->sensor_right);
  gpio_close(&mgr->gpio_left);
  gpio_close(&mgr->gpio_right);

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
    mgr->invalid_cnt_left = 0;
    mgr->invalid_cnt_right = 0;
    mgr->last_valid_left_mm = 0;
    mgr->last_valid_right_mm = 0;
    mgr->timeout_cnt_left = 0;
    mgr->timeout_cnt_right = 0;

    /* ── Health-check & warm-up ──────────────────────────────────────────
     * After a long idle period between init and first use, sensors may
     * be in a stale state.  Verify they respond on I2C, re-init if needed,
     * then do warm-up reads.
     * ────────────────────────────────────────────────────────────────── */

    /* Check LEFT sensor health */
    uint8_t id_l = vl53l0x_read_reg(&mgr->sensor_left,
                                    VL53L0X_REG_IDENTIFICATION_MODEL_ID);
    if (id_l != 0xEE)
    {
      printf("[manager] LEFT sensor not responding (id=0x%02X) — re-init...\n", id_l);
      reinit_sensor(&mgr->sensor_left, "LEFT", mgr->config.sensor_timeout_ms);
    }

    /* Check RIGHT sensor health */
    uint8_t id_r = vl53l0x_read_reg(&mgr->sensor_right,
                                    VL53L0X_REG_IDENTIFICATION_MODEL_ID);
    if (id_r != 0xEE)
    {
      printf("[manager] RIGHT sensor not responding (id=0x%02X) — re-init...\n", id_r);
      reinit_sensor(&mgr->sensor_right, "RIGHT", mgr->config.sensor_timeout_ms);
    }

    /* Warm-up: discard first readings and log results */
    printf("[manager] Warming up sensors (3 dummy reads)...\n");
    int warmup_ok_l = 0, warmup_ok_r = 0;
    for (int i = 0; i < 3; i++)
    {
      uint16_t wl = vl53l0x_read_range_single_mm(&mgr->sensor_left);
      bool wl_timeout = vl53l0x_timeout_occurred(&mgr->sensor_left);
      uint16_t wr = vl53l0x_read_range_single_mm(&mgr->sensor_right);
      bool wr_timeout = vl53l0x_timeout_occurred(&mgr->sensor_right);

      if (!wl_timeout && wl < mgr->config.max_range_mm && wl > VL53L0X_MIN_RANGE_MM)
        warmup_ok_l++;
      if (!wr_timeout && wr < mgr->config.max_range_mm && wr > VL53L0X_MIN_RANGE_MM)
        warmup_ok_r++;

      printf("[manager]   warmup #%d: L=%u%s  R=%u%s\n",
             i + 1, wl, wl_timeout ? " TIMEOUT" : "",
             wr, wr_timeout ? " TIMEOUT" : "");
      usleep(50000); /* 50ms between warm-up reads */
    }

    /* If warm-up found all timeouts, try one more re-init */
    if (warmup_ok_l == 0)
    {
      printf("[manager] LEFT: all warm-up reads failed — attempting re-init\n");
      if (reinit_sensor(&mgr->sensor_left, "LEFT", mgr->config.sensor_timeout_ms))
      {
        /* One more test read */
        uint16_t t = vl53l0x_read_range_single_mm(&mgr->sensor_left);
        vl53l0x_timeout_occurred(&mgr->sensor_left);
        printf("[manager] LEFT after re-init: %u mm\n", t);
      }
    }
    if (warmup_ok_r == 0)
    {
      printf("[manager] RIGHT: all warm-up reads failed — attempting re-init\n");
      if (reinit_sensor(&mgr->sensor_right, "RIGHT", mgr->config.sensor_timeout_ms))
      {
        /* One more test read */
        uint16_t t = vl53l0x_read_range_single_mm(&mgr->sensor_right);
        vl53l0x_timeout_occurred(&mgr->sensor_right);
        printf("[manager] RIGHT after re-init: %u mm\n", t);
      }
    }

    printf("[manager] Warm-up complete (L: %d/3 ok, R: %d/3 ok)\n",
           warmup_ok_l, warmup_ok_r);

    /* (Re-)start continuous mode after warm-up */
    if (mgr->continuous_started)
    {
      vl53l0x_stop_continuous(&mgr->sensor_left);
      vl53l0x_stop_continuous(&mgr->sensor_right);
      usleep(5000);
    }
    vl53l0x_start_continuous(&mgr->sensor_left, CONTINUOUS_PERIOD_MS);
    usleep(25000); /* 25ms offset — stagger measurement windows */
    vl53l0x_start_continuous(&mgr->sensor_right, CONTINUOUS_PERIOD_MS);
    mgr->continuous_started = true;
    printf("[manager] Continuous timed mode (re)started, period=%dms\n",
           CONTINUOUS_PERIOD_MS);
  }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * UPDATE  — Máy trạng thái docking chính
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * Ưu tiên trạng thái (từ cao → thấp):
 *   1. Cả hai mất   → SEARCHING         (Case 1)
 *   2. Một bên mất   → CENTERING_X       (Case 2)
 *   3. Lệch yaw lớn  → ALIGNING_YAW      (Case 3)
 *   4. Còn xa         → APPROACHING_Y     (Case 4)
 *   5. Đúng vị trí    → DOCKED            (Case 5)
 *
 * Nhờ thứ tự ưu tiên này, các chuyển trạng thái ngược tự
 * động xảy ra, ví dụ: đang xoay mà 1 bên trượt ra → ngay
 * lập tức quay về CENTERING_X (Case 3 → Case 2).
 */

/* ── Helper: classify a raw uint16 read into a human-readable error tag ── */
/* Returns NULL when the value is physically valid */
static const char *sensor_err_reason(uint16_t v, bool timeout, uint16_t max_range)
{
  if (timeout)
    return "TIMEOUT (no response within limit)";
  if (v == VL53L0X_I2C_ERROR)
    return "I2C_ERROR 0xFFFF (bus read failed — check wiring/address)";
  if (v == VL53L0X_PHASE_FAIL)
    return "PHASE_FAIL 0x1FFE (target too close/far or ambient light)";
  if (v <= VL53L0X_MIN_RANGE_MM)
    return "ZERO/TOO_SMALL (I2C returned all-zeros — bus glitch)";
  if (v >= max_range)
    return "OUT_OF_RANGE (target beyond max_range_mm)";
  return NULL; /* valid */
}

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

  /* ── 1. Đọc cả hai cảm biến (continuous mode + retry) ───────────── */
  /* Continuous mode: sensor auto-measures, we only poll interrupt +     */
  /* read result → 2-3 I2C transactions instead of 12+ in single-shot.  */
  /* If a read fails (vibration glitch), retry once after 2ms.           */

#define SENSOR_READ_MAX_RETRIES 2

  st.dist_left_mm = vl53l0x_read_range_continuous_mm(&mgr->sensor_left);
  bool timeout_l = vl53l0x_timeout_occurred(&mgr->sensor_left);
  if (timeout_l)
  {
    for (int r = 1; r < SENSOR_READ_MAX_RETRIES; r++)
    {
      usleep(2000); /* 2ms — wait for transient to pass */
      st.dist_left_mm = vl53l0x_read_range_continuous_mm(&mgr->sensor_left);
      timeout_l = vl53l0x_timeout_occurred(&mgr->sensor_left);
      if (!timeout_l)
        break;
    }
  }

  /* ── Bus settle delay between dual-sensor reads ──────────────────
   * Give the I2C bus 5ms to stabilise after LEFT's transactions
   * before switching to RIGHT's address.  This prevents the Tegra
   * I2C controller from seeing stale SCL/SDA states from LEFT's
   * clock-stretching when it starts talking to RIGHT.  */
  usleep(5000);

  st.dist_right_mm = vl53l0x_read_range_continuous_mm(&mgr->sensor_right);
  bool timeout_r = vl53l0x_timeout_occurred(&mgr->sensor_right);
  if (timeout_r)
  {
    for (int r = 1; r < SENSOR_READ_MAX_RETRIES; r++)
    {
      usleep(2000);
      st.dist_right_mm = vl53l0x_read_range_continuous_mm(&mgr->sensor_right);
      timeout_r = vl53l0x_timeout_occurred(&mgr->sensor_right);
      if (!timeout_r)
        break;
    }
  }

  bool raw_left_valid = !timeout_l && (st.dist_left_mm > VL53L0X_MIN_RANGE_MM) && (st.dist_left_mm != VL53L0X_PHASE_FAIL) && (st.dist_left_mm != VL53L0X_I2C_ERROR) && (st.dist_left_mm < cfg->max_range_mm);
  bool raw_right_valid = !timeout_r && (st.dist_right_mm > VL53L0X_MIN_RANGE_MM) && (st.dist_right_mm != VL53L0X_PHASE_FAIL) && (st.dist_right_mm != VL53L0X_I2C_ERROR) && (st.dist_right_mm < cfg->max_range_mm);

  /* ── Log mỗi lần đọc lỗi (chỉ in lần đầu và lần xác nhận confirmed) ── */
  if (!raw_left_valid)
  {
    const char *reason = sensor_err_reason(st.dist_left_mm, timeout_l, cfg->max_range_mm);
    int cnt_after = mgr->invalid_cnt_left + 1; /* +1 vì chưa increment */
    if (cnt_after == 1)
      printf("[VL53L0X] LEFT  bad read #%d : raw=%u → %s\n", cnt_after, st.dist_left_mm, reason);
    else if (cnt_after == SENSOR_INVALID_DEBOUNCE)
      printf("[VL53L0X] LEFT  CONFIRMED INVALID after %d consecutive errors : raw=%u → %s\n",
             cnt_after, st.dist_left_mm, reason);
  }
  if (!raw_right_valid)
  {
    const char *reason = sensor_err_reason(st.dist_right_mm, timeout_r, cfg->max_range_mm);
    int cnt_after = mgr->invalid_cnt_right + 1;
    if (cnt_after == 1)
      printf("[VL53L0X] RIGHT bad read #%d : raw=%u → %s\n", cnt_after, st.dist_right_mm, reason);
    else if (cnt_after == SENSOR_INVALID_DEBOUNCE)
      printf("[VL53L0X] RIGHT CONFIRMED INVALID after %d consecutive errors : raw=%u → %s\n",
             cnt_after, st.dist_right_mm, reason);
  }

  /* ── I2C bus recovery: track ALL consecutive invalid reads ────────── */
  /* When vibration corrupts I2C, we can get various garbage values:      */
  /*   - TIMEOUT (65535): bus completely stuck                            */
  /*   - I2C_ERROR (65535): read returned -1                             */
  /*   - ZERO/small (<=10): SDA stuck low, all-zeros                     */
  /*   - Garbage (e.g. 34695): partial byte corruption from wire bounce  */
  /*   - OUT_OF_RANGE/PHASE_FAIL: could be real or corrupted             */
  /* When BOTH sensors produce ANY invalid data simultaneously for N     */
  /* consecutive cycles, the bus is clearly broken → power-cycle.        */

  if (!raw_left_valid)
    mgr->timeout_cnt_left++;
  else
    mgr->timeout_cnt_left = 0;

  if (!raw_right_valid)
    mgr->timeout_cnt_right++;
  else
    mgr->timeout_cnt_right = 0;

  /* When EITHER sensor has consecutive failures → I2C bus is unstable.
   * Both sensors share the same I2C bus, so if one fails the bus itself
   * is likely in a bad state. Power-cycle via XSHUT to recover.          */
  if (mgr->timeout_cnt_left >= TIMEOUT_RECOVERY_THRESHOLD ||
      mgr->timeout_cnt_right >= TIMEOUT_RECOVERY_THRESHOLD)
  {
    printf("[VL53L0X] *** Sensor(s) failed %d+ times (L:%d R:%d) — triggering I2C recovery ***\n",
           TIMEOUT_RECOVERY_THRESHOLD, mgr->timeout_cnt_left, mgr->timeout_cnt_right);

    if (power_cycle_recovery(mgr))
    {
      /* Recovery succeeded — return a zeroed status this cycle,
       * next cycle will get fresh readings */
      st.state = mgr->state; /* preserve current docking state */
      st.cmd.vx = 0.0f;
      st.cmd.vy = 0.0f;
      st.cmd.omega = 0.0f;
      return st;
    }
    else
    {
      printf("[VL53L0X] *** RECOVERY FAILED — sensors may need manual reset ***\n");
      mgr->timeout_cnt_left = 0; /* reset to avoid infinite recovery loop */
      mgr->timeout_cnt_right = 0;
    }
  }

  /* Debounce LEFT */
  if (raw_left_valid)
  {
    if (mgr->invalid_cnt_left >= SENSOR_INVALID_DEBOUNCE)
      printf("[VL53L0X] LEFT  recovered — reading valid again: %u mm\n", st.dist_left_mm);
    mgr->invalid_cnt_left = 0;
    mgr->last_valid_left_mm = st.dist_left_mm;
    st.left_valid = true;
  }
  else
  {
    mgr->invalid_cnt_left++;
    if (mgr->invalid_cnt_left < SENSOR_INVALID_DEBOUNCE && mgr->last_valid_left_mm > 0)
    {
      /* Still within grace period AND have a previous valid reading — use it */
      st.dist_left_mm = mgr->last_valid_left_mm;
      st.left_valid = true;
    }
    else
    {
      /* No previous valid reading OR debounce exhausted → invalid immediately */
      st.left_valid = false;
    }
  }

  /* Debounce RIGHT */
  if (raw_right_valid)
  {
    if (mgr->invalid_cnt_right >= SENSOR_INVALID_DEBOUNCE)
      printf("[VL53L0X] RIGHT recovered — reading valid again: %u mm\n", st.dist_right_mm);
    mgr->invalid_cnt_right = 0;
    mgr->last_valid_right_mm = st.dist_right_mm;
    st.right_valid = true;
  }
  else
  {
    mgr->invalid_cnt_right++;
    if (mgr->invalid_cnt_right < SENSOR_INVALID_DEBOUNCE && mgr->last_valid_right_mm > 0)
    {
      /* Still within grace period AND have a previous valid reading — use it */
      st.dist_right_mm = mgr->last_valid_right_mm;
      st.right_valid = true;
    }
    else
    {
      /* No previous valid reading OR debounce exhausted → invalid immediately */
      st.right_valid = false;
    }
  }

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

    /* Trong lúc APPROACHING nếu |delta| < yaw_realign thì vẫn giữ APPROACHING,
     * chỉ chuyển ALIGNING_YAW khi delta vượt ngưỡng realign lớn hơn.
     * Ngoài trạng thái APPROACHING, dùng yaw_tolerance bình thường. */
    uint16_t yaw_thresh = (mgr->state == DOCK_STATE_APPROACHING_Y)
                              ? cfg->yaw_realign_mm
                              : cfg->yaw_tolerance_mm;

    yaw_ok = (abs16(st.delta_mm) <= (int16_t)yaw_thresh);
    at_dock =
        (abs16(st.delta_mm) <= (int16_t)cfg->yaw_tolerance_mm) &&
        (st.avg_distance_mm >=
         cfg->dock_distance_mm - cfg->dock_tolerance_mm) &&
        (st.avg_distance_mm <= cfg->dock_distance_mm + cfg->dock_tolerance_mm);

    if (at_dock)
    {
      /* ╔═══════════════════════════════════════════════════════╗
       * ║  Case 5: HOÀN THÀNH ĐỊNH VỊ                         ║
       * ║  Thẳng hàng, ngay tâm, đúng khoảng cách.            ║
       * ║  → Dừng hẳn, gửi tín hiệu kích hoạt tay gắp.       ║
       * ╚═══════════════════════════════════════════════════════╝ */
      mgr->state = DOCK_STATE_DOCKED;
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
      st.cmd.vx = -cfg->center_vy; /* trượt phải */
    }
    else
    {
      st.cmd.vx = cfg->center_vy; /* trượt trái  */
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
      st.cmd.omega = cfg->align_omega;
    }
    else
    {
      st.cmd.omega = -cfg->align_omega;
    }
    break;

  case DOCK_STATE_APPROACHING_Y:
    /* Tiến thẳng (heading = Y) + tinh chỉnh yaw theo tỷ lệ */
    st.cmd.vy = cfg->approach_vx;

    if (abs16(st.delta_mm) > 1)
    {
      float ratio = (float)st.delta_mm / (float)cfg->yaw_realign_mm;
      ratio = clampf(ratio, -1.0f, 1.0f);
      st.cmd.omega = -ratio * cfg->align_omega * cfg->approach_yaw_gain;
    }
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
