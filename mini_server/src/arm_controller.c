/**
 * @file arm_controller.c
 * @brief ARM Controller Module - Pick, Place, Gripper, Rest operations
 *
 * Ported from Python server.py for Xavier/C environment.
 */

#include "arm_controller.h"
#include "arm_kinematic.h"
#include "client_manager.h"
#include "socket.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ===== Helper Functions ===== */

static void delay_ms(int ms) { usleep(ms * 1000); }

static bool arm_state_initialized = false;
static double arm_commanded_servo[6] = {0.0};
static const char *arm_motion_op = "idle";
static const char *arm_motion_phase = "idle";
static unsigned long arm_motion_seq = 0;

static void arm_init_commanded_state(void)
{
  if (arm_state_initialized)
    return;

  arm_commanded_servo[0] = ARM_J0_REST;
  arm_commanded_servo[1] = ARM_J1_REST;
  arm_commanded_servo[2] = ARM_J2_REST;
  arm_commanded_servo[3] = ARM_J3_REST;
  arm_commanded_servo[4] =
      arm_calculate_j4_perpendicular_to_x(arm_unmap_angle(0, ARM_J0_REST));
  arm_commanded_servo[5] = ARM_GRIPPER_OPEN;
  arm_state_initialized = true;
}

static void arm_set_motion_context(const char *op, const char *phase)
{
  arm_motion_op = op ? op : "idle";
  arm_motion_phase = phase ? phase : "idle";
}

static void arm_compute_planar_points(double sv1, double sv2, double sv3,
                                      double *elbow_r, double *elbow_z,
                                      double *wrist_r, double *wrist_z,
                                      double *tcp_r, double *tcp_z)
{
  double m1 = arm_unmap_angle(1, sv1);
  double m2 = arm_unmap_angle(2, sv2);
  double m3 = arm_unmap_angle(3, sv3);

  double t1 = m1 * M_PI / 180.0;
  double t2 = m2 * M_PI / 180.0;
  double t3 = m3 * M_PI / 180.0;

  double angle_1_plot = t1 + M_PI / 2.0;
  double p1_r = ARM_A2 * cos(angle_1_plot);
  double p1_z = ARM_D1 + ARM_A2 * sin(angle_1_plot);

  double angle_2_plot = angle_1_plot - t2;
  double p2_r = p1_r + ARM_A3 * cos(angle_2_plot);
  double p2_z = p1_z + ARM_A3 * sin(angle_2_plot);

  double angle_3_plot = angle_2_plot - t3;
  double p3_r = p2_r + ARM_D5 * cos(angle_3_plot);
  double p3_z = p2_z + ARM_D5 * sin(angle_3_plot);

  if (elbow_r)
    *elbow_r = p1_r;
  if (elbow_z)
    *elbow_z = p1_z;
  if (wrist_r)
    *wrist_r = p2_r;
  if (wrist_z)
    *wrist_z = p2_z;
  if (tcp_r)
    *tcp_r = p3_r;
  if (tcp_z)
    *tcp_z = p3_z;
}

static void arm_emit_motion_snapshot(const char *event)
{
  ArmPosition tcp = {0};
  double elbow_r, elbow_z, wrist_r, wrist_z, tcp_r, tcp_z;
  char json[1024];

  arm_init_commanded_state();
  arm_compute_planar_points(arm_commanded_servo[1], arm_commanded_servo[2],
                            arm_commanded_servo[3], &elbow_r, &elbow_z,
                            &wrist_r, &wrist_z, &tcp_r, &tcp_z);
  arm_forward_kinematics(arm_commanded_servo[0], arm_commanded_servo[1],
                         arm_commanded_servo[2], arm_commanded_servo[3], &tcp);

  snprintf(
      json, sizeof(json),
      "{\"type\":\"arm_motion\",\"op\":\"%s\",\"phase\":\"%s\","
      "\"event\":\"%s\",\"seq\":%lu,"
      "\"servo\":{\"j0\":%.1f,\"j1\":%.1f,\"j2\":%.1f,\"j3\":%.1f,\"j4\":%.1f,\"j5\":%.1f},"
      "\"planar\":{\"base\":[0.0,0.0],\"shoulder\":[0.0,%.1f],\"elbow\":[%.1f,%.1f],\"wrist\":[%.1f,%.1f],\"tcp\":[%.1f,%.1f]},"
      "\"tcp\":{\"x\":%.1f,\"y\":%.1f,\"z\":%.1f,\"phi\":%.1f}}\n",
      arm_motion_op, arm_motion_phase, event ? event : "snapshot",
      ++arm_motion_seq, arm_commanded_servo[0], arm_commanded_servo[1],
      arm_commanded_servo[2], arm_commanded_servo[3], arm_commanded_servo[4],
      arm_commanded_servo[5], ARM_D1, elbow_r, elbow_z, wrist_r, wrist_z,
      tcp_r, tcp_z, tcp.x, tcp.y, tcp.z, tcp.phi);
  send_to_upstream_server(json, (int)strlen(json));
}

static void arm_emit_ik_plan(const char *op, const char *phase, double x,
                             double y, double z, double phi_deg,
                             const ArmAngles *angles)
{
  if (!angles)
    return;

  char json[768];
  snprintf(json, sizeof(json),
           "{\"type\":\"arm_plan\",\"op\":\"%s\",\"phase\":\"%s\","
           "\"event\":\"ik_solution\","
           "\"target\":{\"x\":%.1f,\"y\":%.1f,\"z\":%.1f,\"phi\":%.1f},"
           "\"servo\":{\"j0\":%.1f,\"j1\":%.1f,\"j2\":%.1f,\"j3\":%.1f,\"j4\":%.1f,\"j5\":%.1f}}\n",
           op ? op : "unknown", phase ? phase : "ik", x, y, z, phi_deg,
           angles->j0, angles->j1, angles->j2, angles->j3, angles->j4,
           arm_commanded_servo[5]);
  send_to_upstream_server(json, (int)strlen(json));
}

static void arm_emit_execute_target(const char *op, const char *phase,
                                    double robot_x, double robot_y,
                                    double robot_theta, double object_x,
                                    double object_y, double object_length,
                                    double object_width,
                                    const char *grip_side, double arm_x,
                                    double arm_y, double arm_z)
{
  char json[896];
  snprintf(json, sizeof(json),
           "{\"type\":\"arm_plan\",\"op\":\"%s\",\"phase\":\"%s\","
           "\"event\":\"arm_target\","
           "\"robot\":{\"x\":%.3f,\"y\":%.3f,\"theta\":%.3f},"
           "\"object\":{\"x\":%.3f,\"y\":%.3f,\"length\":%.3f,\"width\":%.3f,\"grip_side\":\"%s\"},"
           "\"arm_target\":{\"x\":%.1f,\"y\":%.1f,\"z\":%.1f}}\n",
           op ? op : "unknown", phase ? phase : "target", robot_x, robot_y,
           robot_theta, object_x, object_y, object_length, object_width,
           grip_side ? grip_side : "unknown", arm_x, arm_y, arm_z);
  send_to_upstream_server(json, (int)strlen(json));
}

static void arm_emit_operation_status(const char *op, const char *phase,
                                      const char *status,
                                      const char *message)
{
  char json[512];
  snprintf(json, sizeof(json),
           "{\"type\":\"arm_status\",\"op\":\"%s\",\"phase\":\"%s\","
           "\"status\":\"%s\",\"message\":\"%s\"}\n",
           op ? op : "unknown", phase ? phase : "unknown",
           status ? status : "info", message ? message : "");
  send_to_upstream_server(json, (int)strlen(json));
}

static void send_servo_cmd(int channel, double deg)
{
  arm_init_commanded_state();
  char cmd[128];
  snprintf(cmd, sizeof(cmd), "{\"cmd\":\"servo\",\"ch\":%d,\"deg\":%.1f}\n",
           channel, deg);
  if (channel >= 0 && channel < 6)
  {
    arm_commanded_servo[channel] = deg;
  }
  client_manager_broadcast_to_arm(cmd, strlen(cmd));
  arm_emit_motion_snapshot("servo_cmd");
}

static void send_servo_off_all(void)
{
  const char *cmd = "{\"cmd\":\"servo_off\",\"all\":true}\n";
  client_manager_broadcast_to_arm(cmd, strlen(cmd));
  arm_emit_motion_snapshot("servo_off_all");
}

/**
 * @brief Send servo command for multiple channels at once
 * Format: {"cmd":"servo","ch":[1,2,3],"deg":[sv1,sv2,sv3]}
 */
static void send_servo_multi(const int *channels, const double *angles,
                             int count)
{
  arm_init_commanded_state();
  char cmd[256];
  int pos = 0;

  pos += snprintf(cmd + pos, sizeof(cmd) - pos, "{\"cmd\":\"servo\",\"ch\":[");

  for (int i = 0; i < count; i++)
  {
    pos += snprintf(cmd + pos, sizeof(cmd) - pos, "%s%d", (i > 0 ? "," : ""),
                    channels[i]);
  }

  pos += snprintf(cmd + pos, sizeof(cmd) - pos, "],\"deg\":[");

  for (int i = 0; i < count; i++)
  {
    pos += snprintf(cmd + pos, sizeof(cmd) - pos, "%s%.1f", (i > 0 ? "," : ""),
                    angles[i]);
    if (channels[i] >= 0 && channels[i] < 6)
    {
      arm_commanded_servo[channels[i]] = angles[i];
    }
  }

  pos += snprintf(cmd + pos, sizeof(cmd) - pos, "]}\n");

  client_manager_broadcast_to_arm(cmd, strlen(cmd));
  arm_emit_motion_snapshot("servo_multi_cmd");
}

/**
 * @brief Send servo command for J1, J2, J3 together
 */
static void send_servo_j123(double j1, double j2, double j3)
{
  int channels[] = {1, 2, 3};
  double angles[] = {j1, j2, j3};
  send_servo_multi(channels, angles, 3);
}

/* ===== Emergency Stop Flag ===== */
static volatile bool arm_stop_requested = false;

void arm_request_stop(void) { arm_stop_requested = true; }

void arm_clear_stop(void) { arm_stop_requested = false; }

bool arm_is_stop_requested(void) { return arm_stop_requested; }

/* ===== Link Angle Calculation (Gravity Compensation) ===== */

void arm_calculate_link_angles(double sv0, double sv1, double sv2, double sv3,
                               ArmLinkAngles *angles)
{
  if (!angles)
    return;

  /* Convert servo angles to math angles */
  double m1 = arm_unmap_angle(1, sv1);
  double m2 = arm_unmap_angle(2, sv2);
  double m3 = arm_unmap_angle(3, sv3);

  double t1 = m1 * M_PI / 180.0;
  double t2 = m2 * M_PI / 180.0;
  double t3 = m3 * M_PI / 180.0;

  /* Calculate geometric angles (same logic as Python workspace_visualizer) */
  /* angle_1_plot = math angle + 90° (link1 angle from horizontal) */
  double link1_geo_rad = t1 + M_PI / 2.0;

  /* link2 = link1 - math_q2 */
  double link2_geo_rad = link1_geo_rad - t2;

  /* link3 = link2 - math_q3 */
  double link3_geo_rad = link2_geo_rad - t3;

  /* Convert to degrees */
  angles->link0 = 0.0;
  angles->link1 = link1_geo_rad * 180.0 / M_PI;
  angles->link2 = link2_geo_rad * 180.0 / M_PI;
  angles->link3 = link3_geo_rad * 180.0 / M_PI;
  angles->link4 = 0.0;
  angles->link5 = 0.0;
}

void arm_send_gravity_angles(double sv0, double sv1, double sv2, double sv3)
{
  ArmLinkAngles link_angles;
  arm_calculate_link_angles(sv0, sv1, sv2, sv3, &link_angles);

  char cmd[256];
  snprintf(
      cmd, sizeof(cmd),
      "{\"cmd\":\"set_gravity\",\"angles\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]}"
      "\n",
      link_angles.link0, link_angles.link1, link_angles.link2,
      link_angles.link3, link_angles.link4, link_angles.link5);
  client_manager_broadcast(cmd, strlen(cmd));

  printf("[ARM-GRAVITY] Sent link angles: J1=%.1f°, J2=%.1f°, J3=%.1f°\n",
         link_angles.link1, link_angles.link2, link_angles.link3);
}

/* ===== Gripper Control ===== */

bool arm_gripper(const char *action)
{
  double angle;

  if (strcmp(action, "open") == 0)
  {
    angle = ARM_GRIPPER_OPEN;
  }
  else if (strcmp(action, "close") == 0)
  {
    angle = ARM_GRIPPER_CLOSED;
  }
  else
  {
    printf("[ARM] Unknown gripper action: %s\n", action);
    return false;
  }

  send_servo_cmd(5, angle);

  int wait_ms = (strcmp(action, "close") == 0) ? ARM_GRIPPER_WAIT_MS : 300;
  delay_ms(wait_ms);

  printf("[ARM] Gripper %s (angle=%.1f)\n", action, angle);
  return true;
}

/* ===== Pick Operation ===== */

static bool arm_pick_internal(double x, double y, double z);

bool arm_pick(double x, double y, double z)
{
  struct timespec start, end;
  clock_gettime(CLOCK_MONOTONIC, &start);

  bool success = arm_pick_internal(x, y, z);

  clock_gettime(CLOCK_MONOTONIC, &end);
  double elapsed_ms = (double)(end.tv_sec - start.tv_sec) * 1e3 + 
                      (double)(end.tv_nsec - start.tv_nsec) / 1e6;
  printf("[ARM-PERF] Pick sequence took %.3f ms (%.2f seconds) | Result: %s\n",
         elapsed_ms, elapsed_ms / 1000.0, success ? "SUCCESS" : "FAILED");

  return success;
}

static bool arm_pick_internal(double x, double y, double z)
{
  arm_set_motion_context("pick", "start");
  arm_emit_operation_status("pick", "start", "started",
                            "Starting pick sequence");
  printf("[PICK] Starting pick at (%.1f, %.1f, %.1f)\n", x, y, z);

  /* Clear stop flag at start */
  arm_clear_stop();

  /* Validate position */
  if (x == 0.0 && y == 0.0)
  {
    printf("[PICK] Invalid position: x=y=0\n");
    return false;
  }

  /* Calculate IK for target position */
  ArmAngles target_angles;
  if (!arm_ik_solve(x, y, z, ARM_PITCH, 10.0, 90.0, &target_angles))
  {
    arm_emit_operation_status("pick", "ik_target", "failed",
                              "Target IK failed");
    printf("[PICK] Position unreachable (IK failed)\n");
    return false;
  }

  arm_emit_ik_plan("pick", "target_ik", x, y, z, ARM_PITCH, &target_angles);

  double sv0 = target_angles.j0;
  double sv1 = target_angles.j1;
  double sv2 = target_angles.j2;
  double sv3 = target_angles.j3;

  printf("[PICK] Target IK: J0=%.1f, J1=%.1f, J2=%.1f, J3=%.1f\n", sv0, sv1,
         sv2, sv3);

  /* === Find valid approach position (hover above target) === */
  ArmAngles approach_angles;
  double approach_z = z + ARM_APPROACH_HEIGHT;
  bool approach_found = false;

  /* Iterate from APPROACH_HEIGHT down to find valid hover height */
  for (double dz = ARM_APPROACH_HEIGHT; dz >= 0; dz -= 5.0)
  {
    double test_z = z + dz;
    if (arm_ik_solve(x, y, test_z, ARM_PITCH, 10.0, 90.0, &approach_angles))
    {
      approach_z = test_z;
      approach_found = true;
      arm_emit_ik_plan("pick", "approach_ik", x, y, test_z, ARM_PITCH,
                       &approach_angles);
      printf("[PICK] Found approach height: z=%.1f (%.1fmm above target)\n",
             approach_z, dz);
      break;
    }
  }

  if (!approach_found)
  {
    printf("[PICK] Warning: No valid approach position, going directly\n");
    approach_angles = target_angles;
    approach_z = z;
  }

  /* === Step 1: Open gripper === */
  if (arm_is_stop_requested())
    return false;
  printf("[PICK] Step 1: Opening gripper\n");
  arm_set_motion_context("pick", "step_1_open_gripper");
  send_servo_cmd(5, ARM_GRIPPER_OPEN);
  delay_ms(300);

  /* === Step 2: Rotate J0 to approach direction === */
  if (arm_is_stop_requested())
    return false;
  printf("[PICK] Step 2: Rotating J0 to %.1f°\n", approach_angles.j0);
  arm_set_motion_context("pick", "step_2_rotate_j0");
  send_servo_cmd(0, approach_angles.j0);
  delay_ms(ARM_MOVE_DELAY_MS);

  /* === Step 2b: Set J4 to maintain perpendicular orientation to X-axis === */
  if (arm_is_stop_requested())
    return false;
  double j0_math = arm_unmap_angle(0, approach_angles.j0);
  double j4_angle = arm_calculate_j4_perpendicular_to_x(j0_math);
  printf("[PICK] Step 2b: Setting J4 to %.1f° (perpendicular to X-axis)\n", j4_angle);
  arm_set_motion_context("pick", "step_2b_set_j4");
  send_servo_cmd(4, j4_angle);
  delay_ms(300);

  /* Send gravity angles for approach */
  arm_send_gravity_angles(approach_angles.j0, approach_angles.j1,
                          approach_angles.j2, approach_angles.j3);

  /* === Step 3: Move to approach position (J1+J2+J3 together) === */
  if (arm_is_stop_requested())
    return false;
  printf("[PICK] Step 3: Moving to approach position (z=%.1f)\n", approach_z);
  arm_set_motion_context("pick", "step_3_move_approach");
  send_servo_j123(approach_angles.j1, approach_angles.j2, approach_angles.j3);
  delay_ms(ARM_JOINT_DELAY_MS);

  /* === Step 4: Descend to target (J1+J2+J3 together) === */
  if (approach_z != z)
  {
    if (arm_is_stop_requested())
      return false;
    printf("[PICK] Step 4: Descending to target (z=%.1f)\n", z);

    /* Send gravity angles for target position */
    arm_send_gravity_angles(sv0, sv1, sv2, sv3);

    /* Move J1+J2+J3 to target */
    arm_set_motion_context("pick", "step_4_descend");
    send_servo_j123(sv1, sv2, sv3);

    /* Wait for descent (safety wait) */
    delay_ms(ARM_DESCENT_WAIT_MS);
  }

  /* === Step 5: Close gripper === */
  if (arm_is_stop_requested())
    return false;
  printf("[PICK] Step 5: Closing gripper\n");
  arm_set_motion_context("pick", "step_5_close_gripper");
  send_servo_cmd(5, ARM_GRIPPER_CLOSED);
  delay_ms(ARM_GRIPPER_WAIT_MS + ARM_GRIP_EXTRA_WAIT_MS);

  /* === Step 6: Lift up === */
  if (arm_is_stop_requested())
    return false;

  /* Lift to approach position */
  printf("[PICK] Step 6: Lifting to z=%.1f\n", approach_z);

  /* Send gravity angles for lift position */
  arm_send_gravity_angles(approach_angles.j0, approach_angles.j1,
                          approach_angles.j2, approach_angles.j3);

  /* Move J1+J2+J3 to approach/lift position */
  arm_set_motion_context("pick", "step_6_lift");
  send_servo_j123(approach_angles.j1, approach_angles.j2, approach_angles.j3);
  delay_ms(500);

  arm_set_motion_context("pick", "complete");
  arm_emit_operation_status("pick", "complete", "completed",
                            "Pick sequence completed");
  printf("[PICK] Pick complete!\n");
  return true;
}

/* ===== Place Operation ===== */

static bool arm_place_internal(double x, double y, double z);

bool arm_place(double x, double y, double z)
{
  struct timespec start, end;
  clock_gettime(CLOCK_MONOTONIC, &start);

  bool success = arm_place_internal(x, y, z);

  clock_gettime(CLOCK_MONOTONIC, &end);
  double elapsed_ms = (double)(end.tv_sec - start.tv_sec) * 1e3 + 
                      (double)(end.tv_nsec - start.tv_nsec) / 1e6;
  printf("[ARM-PERF] Place sequence took %.3f ms (%.2f seconds) | Result: %s\n",
         elapsed_ms, elapsed_ms / 1000.0, success ? "SUCCESS" : "FAILED");

  return success;
}

static bool arm_place_internal(double x, double y, double z)
{
  arm_set_motion_context("place", "start");
  arm_emit_operation_status("place", "start", "started",
                            "Starting place sequence");
  printf("[PLACE] Starting place at (%.1f, %.1f, %.1f)\n", x, y, z);

  /* Clear stop flag at start */
  arm_clear_stop();

  /* Validate position */
  if (x == 0.0 && y == 0.0)
  {
    printf("[PLACE] Invalid position: x=y=0\n");
    return false;
  }

  /* Calculate IK for target position */
  ArmAngles target_angles;
  if (!arm_ik_solve(x, y, z, ARM_PITCH, 10.0, 90.0, &target_angles))
  {
    arm_emit_operation_status("place", "ik_target", "failed",
                              "Target IK failed");
    printf("[PLACE] Position unreachable (IK failed)\n");
    return false;
  }

  arm_emit_ik_plan("place", "target_ik", x, y, z, ARM_PITCH, &target_angles);

  double sv0 = target_angles.j0;
  double sv1 = target_angles.j1;
  double sv2 = target_angles.j2;
  double sv3 = target_angles.j3;

  printf("[PLACE] Target IK: J0=%.1f, J1=%.1f, J2=%.1f, J3=%.1f\n", sv0, sv1,
         sv2, sv3);

  /* === Find valid approach position (hover above target) === */
  ArmAngles approach_angles;
  double approach_z = z + ARM_APPROACH_HEIGHT;
  bool approach_found = false;

  /* Iterate from APPROACH_HEIGHT down to find valid hover height */
  for (double dz = ARM_APPROACH_HEIGHT; dz >= 0; dz -= 5.0)
  {
    double test_z = z + dz;
    if (arm_ik_solve(x, y, test_z, ARM_PITCH, 10.0, 90.0, &approach_angles))
    {
      approach_z = test_z;
      approach_found = true;
      arm_emit_ik_plan("place", "approach_ik", x, y, test_z, ARM_PITCH,
                       &approach_angles);
      printf("[PLACE] Found approach height: z=%.1f (%.1fmm above target)\n",
             approach_z, dz);
      break;
    }
  }

  if (!approach_found)
  {
    printf("[PLACE] Warning: No valid approach position, going directly\n");
    approach_angles = target_angles;
    approach_z = z;
  }

  /* === Step 1: Rotate J0 to place direction === */
  if (arm_is_stop_requested())
    return false;
  printf("[PLACE] Step 1: Rotating J0 to %.1f°\n", approach_angles.j0);
  arm_set_motion_context("place", "step_1_rotate_j0");
  send_servo_cmd(0, approach_angles.j0);
  delay_ms(ARM_MOVE_DELAY_MS);

  /* Send gravity angles for approach */
  arm_send_gravity_angles(approach_angles.j0, approach_angles.j1,
                          approach_angles.j2, approach_angles.j3);

  /* === Step 2: Move to approach position (J1+J2+J3 together) === */
  if (arm_is_stop_requested())
    return false;
  printf("[PLACE] Step 2: Moving to approach position (z=%.1f)\n", approach_z);
  arm_set_motion_context("place", "step_2_move_approach");
  send_servo_j123(approach_angles.j1, approach_angles.j2, approach_angles.j3);
  delay_ms(ARM_JOINT_DELAY_MS);

  /* === Step 3: Descend to target (J1+J2+J3 together) === */
  if (approach_z != z)
  {
    if (arm_is_stop_requested())
      return false;
    printf("[PLACE] Step 3: Descending to target (z=%.1f)\n", z);

    /* Send gravity angles for target position */
    arm_send_gravity_angles(sv0, sv1, sv2, sv3);

    /* Move J1+J2+J3 to target */
    arm_set_motion_context("place", "step_3_descend");
    send_servo_j123(sv1, sv2, sv3);

    /* Wait for descent (safety wait) */
    delay_ms(ARM_DESCENT_WAIT_MS);
  }

  /* === Step 4: Open gripper (release object) === */
  if (arm_is_stop_requested())
    return false;
  printf("[PLACE] Step 4: Opening gripper\n");
  arm_set_motion_context("place", "step_4_open_gripper");
  send_servo_cmd(5, ARM_GRIPPER_OPEN);
  delay_ms(ARM_GRIPPER_WAIT_MS); /* Wait for gripper to fully open (ESP32 secure servo) */

  /* === Step 5: Lift up to approach position === */
  if (arm_is_stop_requested())
    return false;

  printf("[PLACE] Step 5: Lifting to z=%.1f\n", approach_z);

  /* Send gravity angles for lift position */
  arm_send_gravity_angles(approach_angles.j0, approach_angles.j1,
                          approach_angles.j2, approach_angles.j3);

  /* Move J1+J2+J3 to approach/lift position */
  arm_set_motion_context("place", "step_5_lift");
  send_servo_j123(approach_angles.j1, approach_angles.j2, approach_angles.j3);
  delay_ms(500);

  /* === Step 6: Return to REST position === */
  printf("[PLACE] Step 6: Returning to REST position...\n");

  /* Step 6a: Rotate J0 to Y-axis (90°) */
  if (arm_is_stop_requested())
    return false;
  printf("[PLACE] Step 6a: Rotating J0 to %.1f°\n", ARM_J0_REST);
  arm_set_motion_context("place", "step_6a_rest_j0");
  send_servo_cmd(0, ARM_J0_REST);
  delay_ms(ARM_MOVE_DELAY_MS);

  /* Step 6b: Move J1 to 170° */
  if (arm_is_stop_requested())
    return false;
  printf("[PLACE] Step 6b: Moving J1 to %.1f°\n", ARM_J1_REST);
  arm_set_motion_context("place", "step_6b_rest_j1");
  send_servo_cmd(1, ARM_J1_REST);
  delay_ms(ARM_JOINT_DELAY_MS);

  /* Step 6c: Move J2 to 170° */
  if (arm_is_stop_requested())
    return false;
  printf("[PLACE] Step 6c: Moving J2 to %.1f°\n", ARM_J2_REST);
  arm_set_motion_context("place", "step_6c_rest_j2");
  send_servo_cmd(2, ARM_J2_REST);
  delay_ms(ARM_JOINT_DELAY_MS);

  /* Step 6d: Move J3 to 170° */
  if (arm_is_stop_requested())
    return false;
  printf("[PLACE] Step 6d: Moving J3 to %.1f°\n", ARM_J3_REST);
  arm_set_motion_context("place", "step_6d_rest_j3");
  send_servo_cmd(3, ARM_J3_REST);
  delay_ms(ARM_JOINT_DELAY_MS);

  arm_set_motion_context("place", "complete");
  arm_emit_operation_status("place", "complete", "completed",
                            "Place sequence completed");
  printf("[PLACE] Place complete!\n");
  return true;
}

/* ===== Rest Position ===== */

bool arm_rest(void)
{
  printf("[REST] Starting path to rest position...\n");

  /* === Step 1: Rotate J0 to Y-axis (90°) === */
  printf("[REST] Step 1: Rotating J0 to %.1f°\n", ARM_J0_REST);
  send_servo_cmd(0, ARM_J0_REST);
  delay_ms(ARM_MOVE_DELAY_MS);

  /* === Step 2: Move J1 to 170° === */
  printf("[REST] Step 2: Moving J1 to %.1f°\n", ARM_J1_REST);
  send_servo_cmd(1, ARM_J1_REST);
  delay_ms(ARM_MOVE_DELAY_MS);

  /* === Step 3: Move J2 to 170° === */
  printf("[REST] Step 3: Moving J2 to %.1f°\n", ARM_J2_REST);
  send_servo_cmd(2, ARM_J2_REST);
  delay_ms(ARM_MOVE_DELAY_MS);

  /* === Step 4: Move J3 to 170° === */
  printf("[REST] Step 4: Moving J3 to %.1f°\n", ARM_J3_REST);
  send_servo_cmd(3, ARM_J3_REST);
  delay_ms(ARM_MOVE_DELAY_MS);

  /* === Step 4b: Set J4 perpendicular to X-axis === */
  double j4_rest = arm_calculate_j4_perpendicular_to_x(arm_unmap_angle(0, ARM_J0_REST));
  printf("[REST] Step 4b: Setting J4 to %.1f° (perpendicular to X-axis)\n", j4_rest);
  send_servo_cmd(4, j4_rest);
  delay_ms(ARM_MOVE_DELAY_MS);

  /* === Step 5: Open gripper === */
  printf("[REST] Step 5: Opening gripper\n");
  send_servo_cmd(5, ARM_GRIPPER_OPEN);
  delay_ms(ARM_MOVE_DELAY_MS);

  /* === Wait for servos to settle === */
  printf("[REST] Waiting for servos to settle...\n");
  delay_ms(1000);

  /* === Step 6: Disable all servos === */
  printf("[REST] Step 6: Disabling all servos\n");
  send_servo_off_all();
  delay_ms(200);

  printf("[REST] Rest position reached - Servos OFF!\n");
  return true;
}

/* ===== Execute Grip Operation ===== */

bool arm_execute_grip(double robot_x, double robot_y, double robot_theta,
                      double object_x, double object_y, double object_length,
                      double object_width, const char *grip_side)
{
  arm_set_motion_context("execute_grip", "start");
  arm_emit_operation_status("execute_grip", "start", "started",
                            "Starting execute_grip planning");
  printf("[GRIP] ======== EXECUTE GRIP ========\n");
  printf("[GRIP] Robot pos: (%.3f, %.3f) theta_ekf: %.2f rad (%.1f deg)\n",
         robot_x, robot_y, robot_theta, robot_theta * 180.0 / M_PI);
  printf("[GRIP] Object pos: (%.3f, %.3f) size: %.2fm x %.2fm\n", object_x,
         object_y, object_length, object_width);
  printf("[GRIP] Grip side: %s\n", grip_side);

  /* Clear stop flag at start */
  arm_clear_stop();

  /* Validate grip_side */
  if (strcmp(grip_side, "top") != 0 && strcmp(grip_side, "bottom") != 0 &&
      strcmp(grip_side, "left") != 0 && strcmp(grip_side, "right") != 0)
  {
    arm_emit_operation_status("execute_grip", "validate", "failed",
                              "Invalid grip_side");
    printf(
        "[GRIP] ERROR: Invalid grip_side '%s'. Must be top/bottom/left/right\n",
        grip_side);
    return false;
  }

  /* Step 1: Calculate target grip Y position and validate X range */
  /* For rectangular objects: only need to match Y coordinate (edge position)
   * X position can be anywhere along object's length - validate robot is within range */
  double edge_y;
  double object_x_min, object_x_max; /* Valid X range for gripping */

  if (strcmp(grip_side, "top") == 0)
  {
    edge_y = object_y + (object_length / 2.0);
    object_x_min = object_x - (object_width / 2.0);
    object_x_max = object_x + (object_width / 2.0);
    printf("[GRIP] Target edge (top): Y=%.3f, valid X range: [%.3f, %.3f]\n",
           edge_y, object_x_min, object_x_max);
  }
  else if (strcmp(grip_side, "bottom") == 0)
  {
    edge_y = object_y - (object_length / 2.0);
    object_x_min = object_x - (object_width / 2.0);
    object_x_max = object_x + (object_width / 2.0);
    printf("[GRIP] Target edge (bottom): Y=%.3f, valid X range: [%.3f, %.3f]\n",
           edge_y, object_x_min, object_x_max);
  }
  else if (strcmp(grip_side, "left") == 0)
  {
    edge_y = object_y - (object_width / 2.0);
    object_x_min = object_x - (object_length / 2.0);
    object_x_max = object_x + (object_length / 2.0);
    printf("[GRIP] Target edge (left): Y=%.3f, valid X range: [%.3f, %.3f]\n",
           edge_y, object_x_min, object_x_max);
  }
  else /* right */
  {
    edge_y = object_y + (object_width / 2.0);
    object_x_min = object_x - (object_length / 2.0);
    object_x_max = object_x + (object_length / 2.0);
    printf("[GRIP] Target edge (right): Y=%.3f, valid X range: [%.3f, %.3f]\n",
           edge_y, object_x_min, object_x_max);
  }

  /* Validate robot X position is within object's grippable range */
  if (robot_x < object_x_min || robot_x > object_x_max)
  {
    arm_emit_operation_status("execute_grip", "validate", "failed",
                              "Robot X outside grip range");
    printf("[GRIP] ERROR: Robot X position (%.3f) outside valid grip range [%.3f, %.3f]\n",
           robot_x, object_x_min, object_x_max);
    printf("[GRIP]   Robot needs to move along X-axis to align with object!\n");
    return false;
  }

  printf("[GRIP] Robot X position (%.3f) is within valid range - OK\n", robot_x);

  /* Step 2: Determine robot theta for arm calculations */
  double theta;

#if TEST_MODE_WHEEL_UP == 1
  /* Test mode: wheels are lifted, EKF theta doesn't update correctly
   * Use fixed theta based on grip_side:
   *   top: robot approaches from +Y, faces -Y → theta = π
   *   bottom: robot approaches from -Y, faces +Y → theta = 0
   *   left: robot approaches from -X, faces +X → theta = -π/2
   *   right: robot approaches from +X, faces -X → theta = π/2
   */
  if (strcmp(grip_side, "top") == 0)
  {
    theta = M_PI; /* 180° - facing -Y */
  }
  else if (strcmp(grip_side, "bottom") == 0)
  {
    theta = 0.0; /* 0° - facing +Y */
  }
  else if (strcmp(grip_side, "left") == 0)
  {
    theta = -M_PI / 2.0; /* -90° - facing +X */
  }
  else
  {                     /* right */
    theta = M_PI / 2.0; /* 90° - facing -X */
  }
  printf("[GRIP] TEST_MODE: Using fixed theta for '%s': %.2f rad (%.1f deg)\n",
         grip_side, theta, theta * 180.0 / M_PI);
#else
  /* Normal mode: Use robot theta from server (already oriented in global frame) */
  theta = robot_theta;
  printf("[GRIP] Using robot theta: %.2f rad (%.1f deg)\n", theta,
         theta * 180.0 / M_PI);
#endif

  /* Step 3: Calculate arm base position in global frame (meters) */
  /* Arm base is 7cm IN FRONT of robot center (in robot's +Y local direction) */
  double gripper_offset_m = GRIPPER_Y_OFFSET_MM / 1000.0; /* 0.07m */

  /* Robot's forward direction when facing object:
   * forward_x = cos(theta + pi/2) = -sin(theta)
   * forward_y = sin(theta + pi/2) = cos(theta)
   * Wait, this depends on convention. Let's use: robot faces angle theta.
   * forward = (cos(theta), sin(theta))  */

  /* Arm base = robot + forward * offset
   * But our convention: theta=0 means facing +Y, so forward = (-sin(theta),
   * cos(theta)) */
  double arm_base_x = robot_x + gripper_offset_m * (-sin(theta));
  double arm_base_y = robot_y + gripper_offset_m * cos(theta);

  printf("[GRIP] Arm base global pos: (%.3f, %.3f)\n", arm_base_x, arm_base_y);

  /* Step 4: Calculate delta from arm_base to target grip point (in global frame) */
  /* For rectangular objects: grip at robot's current X, only adjust Y to edge */
  double grip_x = robot_x; /* Grip at robot's current X position */
  double grip_y = edge_y;  /* Grip at object's edge Y position */

  double dx_global = grip_x - arm_base_x; /* target - arm_base */
  double dy_global = grip_y - arm_base_y;

  printf("[GRIP] Target grip point (global): (%.3f, %.3f)\n", grip_x, grip_y);
  printf("[GRIP] Delta global (m): dx=%.3f dy=%.3f\n", dx_global, dy_global);

  /* Step 5: Rotate to arm-local coordinates */
  /* Arm coordinate system:
   *   arm +Y (forward): along robot's forward direction
   *   arm +X (right): perpendicular to forward, to the right
   *
   * Global to arm local rotation by angle (-theta):  */

  /* For IK: (arm_x, arm_y) are coordinates in arm base frame where:
   *   arm origin at arm_base,
   *   arm +X to the right,
   *   arm +Y forward
   *
   * Rotation from global to arm frame by angle (-theta):  */
  double cos_theta = cos(-theta);
  double sin_theta = sin(-theta);

  double arm_x =
      (dx_global * cos_theta - dy_global * sin_theta) * 1000.0; /* mm */
  double arm_y =
      (dx_global * sin_theta + dy_global * cos_theta) * 1000.0; /* mm */
  double arm_z = DEFAULT_GRIP_HEIGHT;                           /* Default grip height */

  printf("[GRIP] Arm local coords (mm): X=%.1f Y=%.1f Z=%.1f\n", arm_x, arm_y,
         arm_z);
  arm_emit_execute_target("execute_grip", "arm_target", robot_x, robot_y,
                          robot_theta, object_x, object_y, object_length,
                          object_width, grip_side, arm_x, arm_y, arm_z);

  /* Step 6: Validate arm reach */
  double arm_reach = sqrt(arm_x * arm_x + arm_y * arm_y);
  printf("[GRIP] Arm reach distance: %.1f mm\n", arm_reach);

  if (arm_reach < 50.0)
  {
    printf("[GRIP] WARNING: Target too close (%.1f mm). Adjusting...\n",
           arm_reach);
    /* Scale to minimum reach */
    double scale = 60.0 / arm_reach;
    arm_x *= scale;
    arm_y *= scale;
    printf("[GRIP] Adjusted coords: X=%.1f Y=%.1f\n", arm_x, arm_y);
  }

  /* Step 7: Execute pick operation */
  printf("[GRIP] Calling arm_pick(%.1f, %.1f, %.1f)\n", arm_x, arm_y, arm_z);
  bool success = arm_pick(arm_x, arm_y, arm_z);

  if (success)
  {
    arm_emit_operation_status("execute_grip", "complete", "completed",
                              "execute_grip completed");
    printf("[GRIP] ======== GRIP SUCCESS ========\n");
  }
  else
  {
    arm_emit_operation_status("execute_grip", "complete", "failed",
                              "execute_grip failed");
    printf("[GRIP] ======== GRIP FAILED ========\n");
  }

  return success;
}

/* ===== Execute Place Operation ===== */

bool arm_execute_place(double robot_x, double robot_y, double robot_theta,
                       double object_x, double object_y, double object_length,
                       double object_width, const char *grip_side)
{
  arm_set_motion_context("execute_place", "start");
  arm_emit_operation_status("execute_place", "start", "started",
                            "Starting execute_place planning");
  printf("[PLACE] ======== EXECUTE PLACE ========\n");
  printf("[PLACE] Robot pos: (%.3f, %.3f) theta_ekf: %.2f rad (%.1f deg)\n",
         robot_x, robot_y, robot_theta, robot_theta * 180.0 / M_PI);
  printf("[PLACE] Object pos: (%.3f, %.3f) size: %.2fm x %.2fm\n", object_x,
         object_y, object_length, object_width);
  printf("[PLACE] Grip side: %s\n", grip_side);

  /* Clear stop flag at start */
  arm_clear_stop();

  /* Validate grip_side */
  if (strcmp(grip_side, "top") != 0 && strcmp(grip_side, "bottom") != 0 &&
      strcmp(grip_side, "left") != 0 && strcmp(grip_side, "right") != 0)
  {
    arm_emit_operation_status("execute_place", "validate", "failed",
                              "Invalid grip_side");
    printf("[PLACE] ERROR: Invalid grip_side '%s'. Must be top/bottom/left/right\n",
           grip_side);
    return false;
  }

  /* Step 1: Calculate target place edge using grip_side (mirrors arm_execute_grip) */
  double edge_y;

  if (strcmp(grip_side, "top") == 0)
  {
    edge_y = object_y + (object_length / 2.0);
    printf("[PLACE] Target edge (top): Y=%.3f\n", edge_y);
  }
  else if (strcmp(grip_side, "bottom") == 0)
  {
    edge_y = object_y - (object_length / 2.0);
    printf("[PLACE] Target edge (bottom): Y=%.3f\n", edge_y);
  }
  else if (strcmp(grip_side, "left") == 0)
  {
    edge_y = object_y - (object_width / 2.0);
    printf("[PLACE] Target edge (left): Y=%.3f\n", edge_y);
  }
  else /* right */
  {
    edge_y = object_y + (object_width / 2.0);
    printf("[PLACE] Target edge (right): Y=%.3f\n", edge_y);
  }

  double place_x = robot_x; /* Place at robot's current X (same as grip) */
  double place_y = edge_y;
  printf("[PLACE] Target place point (global): (%.3f, %.3f)\n", place_x, place_y);

  /* Step 2: Determine robot theta for arm calculations */
  double theta;

#if TEST_MODE_WHEEL_UP == 1
  /* Test mode: use fixed theta based on grip_side (mirrors arm_execute_grip) */
  if (strcmp(grip_side, "top") == 0)
    theta = M_PI;
  else if (strcmp(grip_side, "bottom") == 0)
    theta = 0.0;
  else if (strcmp(grip_side, "left") == 0)
    theta = -M_PI / 2.0;
  else
    theta = M_PI / 2.0;
  printf("[PLACE] TEST_MODE: Using fixed theta for '%s': %.2f rad (%.1f deg)\n",
         grip_side, theta, theta * 180.0 / M_PI);
#else
  theta = robot_theta;
  printf("[PLACE] Using robot theta: %.2f rad (%.1f deg)\n", theta,
         theta * 180.0 / M_PI);
#endif

  /* Step 3: Calculate arm base position in global frame (meters) */
  /* Arm base is 7cm IN FRONT of robot center (in robot's +Y local direction) */
  double gripper_offset_m = GRIPPER_Y_OFFSET_MM / 1000.0; /* 0.07m */

  /* Arm base = robot + forward * offset
   * Convention: theta=0 means facing +Y, so forward = (-sin(theta), cos(theta)) */
  double arm_base_x = robot_x + gripper_offset_m * (-sin(theta));
  double arm_base_y = robot_y + gripper_offset_m * cos(theta);

  printf("[PLACE] Arm base global pos: (%.3f, %.3f)\n", arm_base_x, arm_base_y);

  /* Step 4: Calculate delta from arm_base to target place point (in global frame) */
  double dx_global = place_x - arm_base_x;
  double dy_global = place_y - arm_base_y;

  printf("[PLACE] Delta global (m): dx=%.3f dy=%.3f\n", dx_global, dy_global);

  /* Step 5: Rotate to arm-local coordinates */
  /* Rotation from global to arm frame by angle (-theta): */
  double cos_theta = cos(-theta);
  double sin_theta = sin(-theta);

  double arm_x =
      (dx_global * cos_theta - dy_global * sin_theta) * 1000.0; /* mm */
  double arm_y =
      (dx_global * sin_theta + dy_global * cos_theta) * 1000.0; /* mm */
  double arm_z = DEFAULT_GRIP_HEIGHT;                           /* Default place height */

  printf("[PLACE] Arm local coords (mm): X=%.1f Y=%.1f Z=%.1f\n", arm_x, arm_y,
         arm_z);
  arm_emit_execute_target("execute_place", "arm_target", robot_x, robot_y,
                          robot_theta, object_x, object_y, object_length,
                          object_width, grip_side, arm_x, arm_y, arm_z);

  /* Step 6: Validate arm reach */
  double arm_reach = sqrt(arm_x * arm_x + arm_y * arm_y);
  printf("[PLACE] Arm reach distance: %.1f mm\n", arm_reach);

  if (arm_reach < 50.0)
  {
    printf("[PLACE] WARNING: Target too close (%.1f mm). Adjusting...\n",
           arm_reach);
    /* Scale to minimum reach */
    double scale = 60.0 / arm_reach;
    arm_x *= scale;
    arm_y *= scale;
    printf("[PLACE] Adjusted coords: X=%.1f Y=%.1f\n", arm_x, arm_y);
  }

  /* Step 7: Execute place operation */
  printf("[PLACE] Calling arm_place(%.1f, %.1f, %.1f)\n", arm_x, arm_y, arm_z);
  bool success = arm_place(arm_x, arm_y, arm_z);

  if (success)
  {
    arm_emit_operation_status("execute_place", "complete", "completed",
                              "execute_place completed");
    printf("[PLACE] ======== PLACE SUCCESS ========\n");
  }
  else
  {
    arm_emit_operation_status("execute_place", "complete", "failed",
                              "execute_place failed");
    printf("[PLACE] ======== PLACE FAILED ========\n");
  }

  return success;
}
