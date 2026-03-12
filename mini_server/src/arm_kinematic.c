/**
 * @file arm_kinematic.c
 * @brief Robot Arm Kinematics Module - Forward and Inverse Kinematics
 * Implementation
 *
 * Ported from Python kinematics.py for Xavier/C environment.
 */

#include "arm_kinematic.h"
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

/* ===== Constants ===== */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)
#define RAD_TO_DEG(x) ((x) * 180.0 / M_PI)

/* ===== Servo Mapping Configuration ===== */
/* Joint index: 0=j0, 1=j1, 2=j2, 3=j3, 4=j4, 5=j5 */
#if ROBOT_ID == 1
static const ServoMapConfig servo_config[6] = {
    {0.0, 1},   /* j0: Base - offset 0, dir 1 */
    {90.0, 1},  /* j1: Shoulder - offset 90, dir 1 */
    {90.0, 1},  /* j2: Elbow - offset 90, dir 1 */
    {90.0, -1}, /* j3: Wrist Pitch - offset 90, dir -1 */
    {90.0, -1}, /* j4: Wrist Roll - offset 0, dir 1 */
    {0.0, 1}    /* j5: Gripper - offset 0, dir 1 */
};
#elif ROBOT_ID == 2
static const ServoMapConfig servo_config[6] = {
    {0.0, 1},   /* j0: Base - offset 0, dir 1 */
    {90.0, 1},  /* j1: Shoulder - offset 90, dir 1 */
    {90.0, 1},  /* j2: Elbow - offset 90, dir 1 */
    {90.0, -1}, /* j3: Wrist Pitch - offset 90, dir -1 */
    {0.0, 1},   /* j4: Wrist Roll - offset 0, dir 1 */
    {0.0, 1}    /* j5: Gripper - offset 0, dir 1 */
};
#endif

/* ===== Helper Functions ===== */

static double clamp_d(double val, double min_val, double max_val) {
  if (val < min_val)
    return min_val;
  if (val > max_val)
    return max_val;
  return val;
}

/* ===== Angle Mapping Functions ===== */

double arm_map_angle(int joint_idx, double math_angle) {
  if (joint_idx < 0 || joint_idx > 5) {
    return 90.0; /* Default safe value */
  }

  double servo_angle = servo_config[joint_idx].offset +
                       (math_angle * servo_config[joint_idx].dir);

  /* Safety Clamp */
  /* Safety Clamp */
  double max_angle = (joint_idx == 3) ? 360.0 : 180.0;

  if (servo_angle < 0.0 || servo_angle > max_angle) {
    printf("[ARM-WARN] Clamping j%d: %.1f -> %.1f\n", joint_idx, servo_angle,
           clamp_d(servo_angle, 0.0, max_angle));
    servo_angle = clamp_d(servo_angle, 0.0, max_angle);
  }

  return servo_angle;
}

double arm_unmap_angle(int joint_idx, double servo_angle) {
  if (joint_idx < 0 || joint_idx > 5) {
    return 0.0;
  }

  /* Servo = Offset + (Math * Dir) => Math = (Servo - Offset) / Dir */
  return (servo_angle - servo_config[joint_idx].offset) /
         servo_config[joint_idx].dir;
}

/* ===== Forward Kinematics ===== */

bool arm_forward_kinematics(double j0_deg, double j1_servo, double j2_servo,
                            double j3_servo, ArmPosition *pos) {
  if (!pos)
    return false;

  /* 1. Base Rotation (J0) */
  double theta0_deg = arm_unmap_angle(0, j0_deg);
  double theta0_rad = DEG_TO_RAD(theta0_deg);

  /* 2. Planar Arm (R-Z plane) */
  double m1 = arm_unmap_angle(1, j1_servo);
  double m2 = arm_unmap_angle(2, j2_servo);
  double m3 = arm_unmap_angle(3, j3_servo);

  double t1 = DEG_TO_RAD(m1);
  double t2 = DEG_TO_RAD(m2);
  double t3 = DEG_TO_RAD(m3);

  /* Coordinates in R-Z plane (relative to J1 axis) */
  /* J1 (Dir 1) -> theta1 + 90 */
  double angle_1_plot = t1 + M_PI / 2.0;
  double p1_r = 0.0 + ARM_A2 * cos(angle_1_plot);
  double p1_z = ARM_D1 + ARM_A2 * sin(angle_1_plot);

  /* J2 */
  double angle_2_plot = angle_1_plot - t2;
  double p2_r = p1_r + ARM_A3 * cos(angle_2_plot);
  double p2_z = p1_z + ARM_A3 * sin(angle_2_plot);

  /* J3 */
  double angle_3_plot = angle_2_plot - t3;
  double p3_r = p2_r + ARM_D5 * cos(angle_3_plot);
  double p3_z = p2_z + ARM_D5 * sin(angle_3_plot);

  /* 3. Rotate 2D (R, Z) into 3D (X, Y, Z) */
  double r = p3_r;
  pos->z = p3_z;
  pos->x = r * cos(theta0_rad);
  pos->y = r * sin(theta0_rad);
  pos->phi = RAD_TO_DEG(angle_3_plot);

  return true;
}

/* ===== Inverse Kinematics (Exact) ===== */

bool arm_ik_exact(double target_r, double target_z, double target_phi,
                  double *j1, double *j2, double *j3) {
  if (!j1 || !j2 || !j3)
    return false;

  double phi_rad = DEG_TO_RAD(target_phi);

  /* Calculate Wrist Center */
  double wrist_r = target_r - ARM_D5 * cos(phi_rad);
  double wrist_z = target_z - ARM_D5 * sin(phi_rad);

  double r_rel = wrist_r;
  double z_rel = wrist_z - ARM_D1;

  double dist_sq = r_rel * r_rel + z_rel * z_rel;
  double dist = sqrt(dist_sq);

  double max_reach = ARM_A2 + ARM_A3;
  double min_reach = fabs(ARM_A2 - ARM_A3);

  printf("[IK-DEBUG] r=%.1f, z=%.1f, phi=%.1f\n", target_r, target_z,
         target_phi);
  printf(
      "[IK-DEBUG] wrist_r=%.1f, wrist_z=%.1f, dist=%.1f (range: %.1f-%.1f)\n",
      wrist_r, wrist_z, dist, min_reach, max_reach);

  /* Check reach constraints */
  if (dist > max_reach) {
    printf("[IK-FAIL] dist=%.1f > max_reach=%.1f\n", dist, max_reach);
    return false;
  }
  if (dist < min_reach) {
    printf("[IK-FAIL] dist=%.1f < min_reach=%.1f\n", dist, min_reach);
    return false;
  }

  /* Law of Cosines for J2 (Elbow) */
  double cos_q2 =
      (dist_sq - ARM_A2 * ARM_A2 - ARM_A3 * ARM_A3) / (2.0 * ARM_A2 * ARM_A3);
  cos_q2 = clamp_d(cos_q2, -1.0, 1.0);
  double q2_rad = acos(cos_q2);

  /* Calculate J1 (Shoulder) */
  double alpha = atan2(z_rel, r_rel);
  double beta = atan2(ARM_A3 * sin(q2_rad), ARM_A2 + ARM_A3 * cos(q2_rad));
  double q1_polar = alpha + beta;

  /* Convert to Math Angle Definition */
  double math_q1 = RAD_TO_DEG(q1_polar) - 90.0;
  double math_q2 = RAD_TO_DEG(q2_rad);

  /* Calculate J3 (Wrist) */
  double math_q3 = (math_q1 + 90.0) - math_q2 - target_phi;

  /* Map to Servo Values */
  double sv1 = arm_map_angle(1, math_q1);
  double sv2 = arm_map_angle(2, math_q2);
  double sv3 = arm_map_angle(3, math_q3);

  printf("[IK-DEBUG] math: q1=%.1f, q2=%.1f, q3=%.1f\n", math_q1, math_q2,
         math_q3);
  printf("[IK-DEBUG] servo: sv1=%.1f, sv2=%.1f, sv3=%.1f\n", sv1, sv2, sv3);

  /* Check Limits (Hardware 0-180) */
  /* Check Limits (Hardware 0-180 for most, 0-360 for J3) */
  if (sv1 < 0.0 || sv1 > 180.0 || sv2 < 0.0 || sv2 > 180.0 || sv3 < 0.0 ||
      sv3 > 360.0) {
    printf("[IK-FAIL] Servo out of range: sv1=%.1f, sv2=%.1f, sv3=%.1f\n", sv1,
           sv2, sv3);
    return false;
  }

  *j1 = sv1;
  *j2 = sv2;
  *j3 = sv3;

  return true;
}

/* ===== Inverse Kinematics (Fuzzy Search) ===== */

bool arm_ik_fuzzy(double target_r, double target_z, double target_phi,
                  double tol_pos, double tol_phi, double *j1, double *j2,
                  double *j3, FuzzyIKInfo *info) {
  if (!j1 || !j2 || !j3)
    return false;

  double best_j1 = 0.0, best_j2 = 0.0, best_j3 = 0.0;
  double min_score = DBL_MAX;
  bool found = false;
  FuzzyIKInfo best_info = {0};

  /* Position search grid (5mm steps) */
  double pos_step = 5.0;

  /* Angle search grid (3 degree steps) */
  double phi_step = 3.0;

  for (double d_phi = -tol_phi; d_phi <= tol_phi; d_phi += phi_step) {
    double curr_phi = target_phi + d_phi;

    for (double dr = -tol_pos; dr <= tol_pos; dr += pos_step) {
      for (double dz = -tol_pos; dz <= tol_pos; dz += pos_step) {
        double curr_r = target_r + dr;
        double curr_z = target_z + dz;

        double s1, s2, s3;
        if (arm_ik_exact(curr_r, curr_z, curr_phi, &s1, &s2, &s3)) {
          /* Cost Function */
          double dist_error = sqrt(dr * dr + dz * dz);
          double phi_error = fabs(d_phi);

          /* Servo Comfort: prefer 90 deg (center) */
          double servo_cost =
              (fabs(s1 - 90.0) + fabs(s2 - 90.0) + fabs(s3 - 90.0)) / 180.0;

          /* Weighting: Pos >> Angle >> ServoComfort */
          double total_score =
              (dist_error * 1.0) + (phi_error * 0.5) + (servo_cost * 1.0);

          if (total_score < min_score) {
            min_score = total_score;
            best_j1 = s1;
            best_j2 = s2;
            best_j3 = s3;
            best_info.r = curr_r;
            best_info.z = curr_z;
            best_info.phi = curr_phi;
            best_info.score = min_score;
            found = true;
          }
        }
      }
    }
  }

  if (found) {
    *j1 = best_j1;
    *j2 = best_j2;
    *j3 = best_j3;
    if (info) {
      *info = best_info;
    }
  }

  return found;
}

/* ===== Main IK Entry Point ===== */

bool arm_ik_solve(double x, double y, double z, double phi_deg, double tol_pos,
                  double tol_phi, ArmAngles *angles) {
  if (!angles)
    return false;

  /* 1. Calculate Base Rotation (J0) */
  double theta0_rad = atan2(y, x);
  double theta0_deg = RAD_TO_DEG(theta0_rad);

  /* Check if angle is within physical servo range [0°, 180°] */
  if (theta0_deg < 0.0 || theta0_deg > 180.0) {
    printf("[IK-FAIL] Target (x=%.1f, y=%.1f) requires J0=%.1f deg "
           "(out of range [0, 180])\n",
           x, y, theta0_deg);
    printf("[IK-FAIL] Workspace limited to Y>=0 (upper half-plane only)\n");
    return false;
  }

  /* Map to Servo J0 */
  double sv0 = arm_map_angle(0, theta0_deg);

  if (sv0 < 0.0 || sv0 > 180.0) {
    printf("[IK-FAIL] Base J0 servo out of bounds: %.1f deg\n", sv0);
    return false;
  }

  /* 2. Calculate Cylindrical Coordinates (R, Z) */
  double r = sqrt(x * x + y * y);

  /* 3. Try Exact IK only (no fuzzy fallback) */
  double j1, j2, j3;
  if (arm_ik_exact(r, z, phi_deg, &j1, &j2, &j3)) {
    angles->j0 = sv0;
    angles->j1 = j1;
    angles->j2 = j2;
    angles->j3 = j3;
    /* Calculate J4 to keep end-effector perpendicular to X-axis */
    angles->j4 = arm_calculate_j4_perpendicular_to_x(theta0_deg);
    return true;
  }

  /* Exact IK failed - no fuzzy fallback */
  printf("[IK-FAIL] Exact IK failed for target (x=%.1f, y=%.1f, z=%.1f, "
         "phi=%.1f)\n",
         x, y, z, phi_deg);
  return false;
}

bool arm_ik_solve_simple(double x, double y, double z, double phi_deg,
                         ArmAngles *angles) {
  /* Default tolerances: 10mm position, 90deg angle */
  return arm_ik_solve(x, y, z, phi_deg, 10.0, 90.0, angles);
}

/* ===== J4 Orientation Control ===== */

double arm_calculate_j4_perpendicular_to_x(double j0_deg) {
  /**
   * Purpose: Calculate J4 (wrist roll) to keep end-effector perpendicular to
   * X-axis
   *
   * Logic: As base J0 rotates, J4 must compensate to maintain perpendicularity
   * - When J0=0°:   arm points along +X, J4=90° for perpendicular
   * - When J0=90°:  arm points along +Y, J4=0° for perpendicular
   * - When J0=180°: arm points along -X, J4=90° for perpendicular
   */

  double j4_math;

#if ROBOT_ID == 1
  /* Robot 1: J0 and J4 rotate in opposite directions
   * When J0 increases, J4 should decrease to maintain perpendicularity
   * Formula: J4 = 180° - J0 */
  j4_math = 180.0 - j0_deg;
  printf("[ARM-DEBUG] Robot 1: J0=%.1f° -> J4_math=%.1f° (180-J0)\n", j0_deg,
         j4_math);
#else
  /* Robot 2: Standard compensation */
  j4_math = 180.0 - j0_deg;
  printf("[ARM-DEBUG] Robot 2: J0=%.1f° -> J4_math=%.1f° (90-J0)\n", j0_deg,
         j4_math);
#endif

  /* Map to servo angle */
  double j4_servo = arm_map_angle(4, j4_math);

  /* Safety clamp to servo range [0, 180] */
  if (j4_servo < 0.0) {
    printf("[ARM-WARN] J4 clamped: %.1f -> 0.0\n", j4_servo);
    j4_servo = 0.0;
  } else if (j4_servo > 180.0) {
    printf("[ARM-WARN] J4 clamped: %.1f -> 180.0\n", j4_servo);
    j4_servo = 180.0;
  }

  return j4_servo;
}
