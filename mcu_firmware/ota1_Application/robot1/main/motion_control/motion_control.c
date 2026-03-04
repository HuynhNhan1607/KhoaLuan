#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "bno055_handler.h"
#include "motion_control.h"
#include "motor_driver.h"

// Set to 1 to disable heading hold (for trajectory following mode)
#define DISABLE_HEADING_HOLD 1

const static char *Tag_Motion = "MotionControl";
static TaskHandle_t handle_maintain_motion_state = NULL;

typedef struct robot_status_t {
  float vx;
  float vy;
  float theta;
  float omega_theta;
} robot_status_t;

static robot_status_t mecanum = {0};

void convert_euler2radian(float *euler) {
  *euler *= 0.017453292519943295f; // Pi/180
}
void task_maintain_motion_state() {
  TickType_t last_tick = xTaskGetTickCount();
  while (1) {
    mecanum.theta = get_heading();
    convert_euler2radian(&mecanum.theta);
#if DISABLE_HEADING_HOLD
    // Heading hold disabled - always use omega_theta from Xavier
    MecanumSpeedControl(mecanum.theta, mecanum.vx, mecanum.vy,
                        mecanum.omega_theta);
#else
    static float previous_theta = 0.0f;
    static bool is_rotate = false;
    if (!is_rotate && mecanum.omega_theta == 0.0f) {
      MecanumSpeedControl(mecanum.theta, mecanum.vx, mecanum.vy,
                          (int)((previous_theta - mecanum.theta) * 100.0f) *
                              10 / (20.0f));
    } else {
      MecanumSpeedControl(mecanum.theta, mecanum.vx, mecanum.vy,
                          mecanum.omega_theta);
    }
    if (is_rotate ^ (mecanum.omega_theta != 0.0f)) {
      is_rotate = !is_rotate;
      previous_theta = mecanum.theta;
    }
#endif
    vTaskDelayUntil(&last_tick,
                    pdMS_TO_TICKS(20)); // 50Hz - matches Xavier's control rate
  }
}

void set_robot_status(float vx, float vy, float omega_theta) {
  mecanum.vx = vx;
  mecanum.vy = vy;
  mecanum.omega_theta = omega_theta;
  if (!handle_maintain_motion_state) {
    xTaskCreatePinnedToCore(task_maintain_motion_state, "motion_state", 4096,
                            NULL, 1, &handle_maintain_motion_state, 1);
  }
}
