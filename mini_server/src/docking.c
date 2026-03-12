/**
 * @file docking.c
 * @brief VL53L0X docking integration — implementation
 *
 * Wraps vl53l0x_manager to:
 *   1. Initialize dual sensors at startup
 *   2. Convert body-frame docking commands to global frame
 *   3. Send motor commands during docking
 *   4. Signal completion to trajectory_executor / json_handler
 */

#include "docking.h"
#include "client_manager.h"
#include "socket.h"
#include "sys_config.h"
#include "vl53l0x_manager.h"

#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>

#if ENABLE_DOCKING == 1

/* ═══════════════════════════════════════════════════════════════════════════
 * STATE
 * ═══════════════════════════════════════════════════════════════════════════
 */

static VL53L0XManager *g_dock_mgr = NULL;
static bool g_dock_active = false;
static bool g_dock_complete = false;
static bool g_dock_initialized = false;
static pthread_mutex_t g_dock_mutex = PTHREAD_MUTEX_INITIALIZER;

/* ═══════════════════════════════════════════════════════════════════════════
 * INIT
 * ═══════════════════════════════════════════════════════════════════════════
 */

bool docking_init(void)
{
  DockConfig cfg = vl53l0x_manager_default_config();

  /* Override config from sys_config.h */
  cfg.i2c_bus = DOCK_I2C_BUS;
  cfg.gpio_chip = "gpiochip1";
  cfg.gpio_line_left = DOCK_GPIO_XSHUT_LEFT;
  cfg.gpio_line_right = DOCK_GPIO_XSHUT_RIGHT;
  cfg.dock_distance_mm = DOCK_FIXED_GRIP_DISTANCE_MM;

  g_dock_mgr = vl53l0x_manager_init(&cfg);
  if (!g_dock_mgr)
  {
    printf("[DOCKING] WARNING: VL53L0X init failed — docking disabled, "
           "falling back to acceptance-zone behavior\n");
    g_dock_initialized = false;
    return false;
  }

  g_dock_initialized = true;
  g_dock_active = false;
  g_dock_complete = false;
  printf("[DOCKING] VL53L0X dual sensors initialized successfully\n");
  return true;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * START / STOP
 * ═══════════════════════════════════════════════════════════════════════════
 */

void docking_start(void)
{
  if (!g_dock_initialized || !g_dock_mgr)
  {
    printf("[DOCKING] Cannot start — not initialized\n");
    return;
  }

  pthread_mutex_lock(&g_dock_mutex);
  vl53l0x_manager_reset(g_dock_mgr);
  g_dock_active = true;
  g_dock_complete = false;
  pthread_mutex_unlock(&g_dock_mutex);

  printf("[DOCKING] === DOCKING STARTED ===\n");
}

void docking_stop(void)
{
  pthread_mutex_lock(&g_dock_mutex);
  g_dock_active = false;
  pthread_mutex_unlock(&g_dock_mutex);

  /* Send zero velocity */
  const char *stop_cmd = "dot_x:0.0000 dot_y:0.0000 dot_theta:0.0000\n";
  client_manager_broadcast_to_motor(stop_cmd, strlen(stop_cmd));

  printf("[DOCKING] Stopped\n");
}

/* ═══════════════════════════════════════════════════════════════════════════
 * STATUS
 * ═══════════════════════════════════════════════════════════════════════════
 */

bool docking_is_active(void)
{
  bool active;
  pthread_mutex_lock(&g_dock_mutex);
  active = g_dock_active;
  pthread_mutex_unlock(&g_dock_mutex);
  return active;
}

bool docking_is_complete(void)
{
  bool complete;
  pthread_mutex_lock(&g_dock_mutex);
  complete = g_dock_complete;
  pthread_mutex_unlock(&g_dock_mutex);
  return complete;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * UPDATE — Main docking cycle
 * ═══════════════════════════════════════════════════════════════════════════
 */

void docking_update(float cur_theta)
{
  if (!g_dock_initialized || !g_dock_mgr || !g_dock_active)
    return;

  /* ── Caller runs at DOCKING_LOOP_RATE_HZ (10 Hz / 100 ms per cycle) ────
   * Each VL53L0X sensor needs ~33 ms minimum per reading.
   * At 100 ms per cycle, both sensors (~66 ms total) fit comfortably.
   * ────────────────────────────────────────────────────────────────────── */

  /* 1. Run one cycle of the docking state machine */
  DockStatus st = vl53l0x_manager_update(g_dock_mgr);

  /* 2. Check if docking is complete */
  if (st.docking_complete)
  {
    pthread_mutex_lock(&g_dock_mutex);
    g_dock_active = false;
    g_dock_complete = true;
    pthread_mutex_unlock(&g_dock_mutex);

    /* Stop motors */
    const char *stop_cmd = "dot_x:0.0000 dot_y:0.0000 dot_theta:0.0000\n";
    client_manager_broadcast_to_motor(stop_cmd, strlen(stop_cmd));

    printf("[DOCKING] === DOCKING COMPLETE ===\n");
    printf("[DOCKING] Distance: %u mm — ready for fixed-distance grip\n",
           st.avg_distance_mm);
    return;
  }

  /* 2b. Check if search failed — object not found */
  if (st.state == DOCK_STATE_NOT_FOUND)
  {
    pthread_mutex_lock(&g_dock_mutex);
    g_dock_active = false;
    g_dock_complete = false;
    pthread_mutex_unlock(&g_dock_mutex);

    /* Stop motors */
    const char *stop_cmd = "dot_x:0.0000 dot_y:0.0000 dot_theta:0.0000\n";
    client_manager_broadcast_to_motor(stop_cmd, strlen(stop_cmd));

    /* Notify server */
    const char *err_msg =
        "{\"type\": \"control\", \"status\": \"docking_not_found\"}\n";
    send_to_upstream_server(err_msg, strlen(err_msg));

    printf("[DOCKING] === OBJECT NOT FOUND — docking aborted ===\n");
    return;
  }

  float dot_x = st.cmd.vx;
  float dot_y = st.cmd.vy;
  float dot_theta = st.cmd.omega;

  /* 4. Send motor command */
  char cmd[128];
  snprintf(cmd, sizeof(cmd), "dot_x:%.4f dot_y:%.4f dot_theta:%.4f\n", dot_x, dot_y, dot_theta);
  client_manager_broadcast_to_motor(cmd, strlen(cmd));

  /* 5. Debug log (every ~1 second at 10Hz) */
  static int dock_debug_cnt = 0;
  if (++dock_debug_cnt >= 1)
  {
    printf("[DOCKING] State=%s L=%s%u R=%s%u Δ=%d Avg=%u "
           "cmd(%.3f,%.3f,%.3f)\n",
           dock_state_name(st.state), st.left_valid ? "" : "!", st.dist_left_mm,
           st.right_valid ? "" : "!", st.dist_right_mm, st.delta_mm,
           st.avg_distance_mm, dot_x, dot_y, dot_theta);
    dock_debug_cnt = 0;
  }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * SHUTDOWN
 * ═══════════════════════════════════════════════════════════════════════════
 */

void docking_shutdown(void)
{
  docking_stop();
  if (g_dock_mgr)
  {
    vl53l0x_manager_shutdown(g_dock_mgr);
    g_dock_mgr = NULL;
  }
  g_dock_initialized = false;
  printf("[DOCKING] Shutdown complete\n");
}

#else /* ENABLE_DOCKING == 0 */

/* Stub implementations when docking is disabled */
bool docking_init(void) { return false; }
void docking_start(void) {}
void docking_update(float t) { (void)t; }
bool docking_is_active(void) { return false; }
bool docking_is_complete(void) { return false; }
void docking_stop(void) {}
void docking_shutdown(void) {}

#endif /* ENABLE_DOCKING */
