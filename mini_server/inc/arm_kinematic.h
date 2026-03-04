/**
 * @file arm_kinematic.h
 * @brief Robot Arm Kinematics Module - Forward and Inverse Kinematics
 *
 * This module provides:
 *   - Forward Kinematics (FK): Servo angles -> TCP position
 *   - Inverse Kinematics (IK): TCP position -> Servo angles
 *   - Angle mapping between DH math angles and physical servo angles
 *
 * Designed for Xavier to compute kinematics and send commands to ESP32.
 */

#ifndef ARM_KINEMATIC_H_
#define ARM_KINEMATIC_H_

#include "sys_config.h" /* Required for USE_GRIPPER macro */
#include <stdbool.h>

/* ===== Robot Kinematics Parameters (mm) ===== */
#define ARM_D1 32.0  /* Vertical offset (base height) */
#define ARM_A2 105.0 /* Link 1 length (shoulder to elbow) */
#define ARM_A3 130.0 /* Link 2 length (elbow to wrist) */
#if USE_GRIPPER == 1
#define ARM_D5 140.0 /* Tool length (wrist to TCP) */
#else
#define ARM_D5 0.0 /* Tool length (wrist to TCP) */
#endif

/* ===== Servo Mapping Configuration ===== */
typedef struct
{
  double offset; /* Servo offset angle */
  int dir;       /* Direction multiplier (1 or -1) */
} ServoMapConfig;

/* ===== IK Result Structure ===== */
typedef struct
{
  double j0; /* Base rotation servo angle (0-180) */
  double j1; /* Shoulder pitch servo angle (0-180) */
  double j2; /* Elbow servo angle (0-180) */
  double j3; /* Wrist pitch servo angle (0-180) */
  double j4; /* Wrist roll servo angle (default 90) */
} ArmAngles;

/* ===== FK Result Structure ===== */
typedef struct
{
  double x;   /* TCP X position (mm) */
  double y;   /* TCP Y position (mm) */
  double z;   /* TCP Z position (mm) */
  double phi; /* End effector pitch angle (degrees) */
} ArmPosition;

/* ===== Fuzzy IK Info Structure ===== */
typedef struct
{
  double r;     /* Actual R coordinate */
  double z;     /* Actual Z coordinate */
  double phi;   /* Actual phi angle */
  double score; /* Solution cost score */
} FuzzyIKInfo;

/**
 * @brief Map math/DH angle to servo angle
 * @param joint_idx Joint index (0-5 for j0-j5)
 * @param math_angle Math/DH angle in degrees
 * @return Servo angle (0-180 degrees)
 */
double arm_map_angle(int joint_idx, double math_angle);

/**
 * @brief Unmap servo angle to math/DH angle
 * @param joint_idx Joint index (0-5 for j0-j5)
 * @param servo_angle Servo angle (0-180 degrees)
 * @return Math/DH angle in degrees
 */
double arm_unmap_angle(int joint_idx, double servo_angle);

/**
 * @brief Forward Kinematics (Geometric)
 * @param j0_deg Base rotation (servo angle)
 * @param j1_servo Shoulder angle (servo angle)
 * @param j2_servo Elbow angle (servo angle)
 * @param j3_servo Wrist pitch angle (servo angle)
 * @param[out] pos Output TCP position and phi
 * @return true on success
 */
bool arm_forward_kinematics(double j0_deg, double j1_servo, double j2_servo,
                            double j3_servo, ArmPosition *pos);

/**
 * @brief Exact Inverse Kinematics for 3-link planar arm
 * @param target_r Distance from Z-axis to TCP (R-Z plane)
 * @param target_z TCP height (mm)
 * @param target_phi Pitch angle in degrees from horizontal
 * @param[out] j1 Output shoulder servo angle
 * @param[out] j2 Output elbow servo angle
 * @param[out] j3 Output wrist pitch servo angle
 * @return true if solution found, false if unreachable
 */
bool arm_ik_exact(double target_r, double target_z, double target_phi,
                  double *j1, double *j2, double *j3);

/**
 * @brief Fuzzy IK Scanner - Find alternative solution near target
 * @param target_r Target radial distance (mm)
 * @param target_z Target height (mm)
 * @param target_phi Target pitch angle (degrees)
 * @param tol_pos Position tolerance (mm)
 * @param tol_phi Angle tolerance (degrees)
 * @param[out] j1 Output shoulder servo angle
 * @param[out] j2 Output elbow servo angle
 * @param[out] j3 Output wrist pitch servo angle
 * @param[out] info Optional fuzzy solution info (can be NULL)
 * @return true if solution found
 */
bool arm_ik_fuzzy(double target_r, double target_z, double target_phi,
                  double tol_pos, double tol_phi, double *j1, double *j2,
                  double *j3, FuzzyIKInfo *info);

/**
 * @brief Main IK Entry Point - 5-DOF Inverse Kinematics
 *        Converts 3D (x,y,z) -> 2D (r,z) + Base Rotation
 *
 * Physical Constraints:
 *   - J0 servo range: 0° to 180°
 *   - J0=0° → arm points to +X axis (y=0)
 *   - J0=180° → arm points to -X axis (y=0)
 *   - Workspace: only Y≥0 (upper half-plane)
 *
 * @param x Target X position (mm)
 * @param y Target Y position (mm)
 * @param z Target Z position (mm)
 * @param phi_deg Target pitch angle (degrees)
 * @param tol_pos Position tolerance for fuzzy search (mm)
 * @param tol_phi Angle tolerance for fuzzy search (degrees)
 * @param[out] angles Output servo angles
 * @return true if solution found, false if unreachable
 */
bool arm_ik_solve(double x, double y, double z, double phi_deg, double tol_pos,
                  double tol_phi, ArmAngles *angles);

/**
 * @brief Simplified IK with default tolerances
 * @param x Target X position (mm)
 * @param y Target Y position (mm)
 * @param z Target Z position (mm)
 * @param phi_deg Target pitch angle (degrees)
 * @param[out] angles Output servo angles
 * @return true if solution found
 */
bool arm_ik_solve_simple(double x, double y, double z, double phi_deg,
                         ArmAngles *angles);

/**
 * @brief Calculate J4 angle to keep end-effector perpendicular to X-axis
 * @param j0_deg Base rotation angle (math angle in degrees)
 * @return J4 servo angle (0-180 degrees)
 *
 * When J0 rotates, this function calculates J4 (wrist roll) so that
 * the end-effector remains perpendicular to the X-axis at all times.
 * This is useful for maintaining consistent gripper orientation.
 */
double arm_calculate_j4_perpendicular_to_x(double j0_deg);

#endif /* ARM_KINEMATIC_H_ */
