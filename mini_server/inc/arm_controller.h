/**
 * @file arm_controller.h
 * @brief ARM Controller Module - Pick, Place, Gripper, Rest operations
 *
 * High-level arm control functions that use IK solver and send commands to
 * ESP32. Ported from Python server.py for Xavier/C environment.
 */

#ifndef ARM_CONTROLLER_H_
#define ARM_CONTROLLER_H_

#include <stdbool.h>

/* ===== Configuration Constants ===== */
#define ARM_GRIPPER_OPEN 80     /* J5 angle for fully open */
#define ARM_GRIPPER_CLOSED 145  /* J5 angle for fully closed */
#define ARM_GRIPPER_WAIT_MS 500 /* ms to wait after closing gripper */

// #define ARM_LIFT_HEIGHT 50.0      /* mm - lift height after picking */
#define ARM_APPROACH_HEIGHT                                                    \
  170 /* mm - height to hover ABOVE object before descending */
#define ARM_PITCH -90.0 /* Gripper always points straight down */

#define ARM_MOVE_DELAY_MS 300       /* Delay between major steps */
#define ARM_JOINT_DELAY_MS 200      /* Delay between individual joint moves */
#define ARM_DESCENT_WAIT_MS 5000    /* Wait time during descent (safety) */
#define ARM_GRIP_EXTRA_WAIT_MS 2000 /* Extra wait after gripper close */

/* Rest position (SERVO angles) */

#define ARM_J1_REST 170.0 /* Shoulder rest angle */
#define ARM_J2_REST 90    /* Elbow rest angle */
#if ROBOT_ID == 1
#define ARM_J0_REST 90.0 /* Base rotation to face Y-axis */
#define ARM_J3_REST 10.0 /* Wrist rest angle */
#elif ROBOT_ID == 2
#define ARM_J0_REST 90.0 /* Base rotation to face Y-axis */
#define ARM_J3_REST 10.0 /* Wrist rest angle - robot 2 */
#endif

/* ===== Link Angles for Gravity Compensation ===== */
typedef struct {
  double link0; /* J0 geometric angle (always 0) */
  double link1; /* J1 geometric angle from horizontal */
  double link2; /* J2 geometric angle from horizontal */
  double link3; /* J3 geometric angle from horizontal */
  double link4; /* J4 geometric angle (always 0) */
  double link5; /* J5 geometric angle (always 0) */
} ArmLinkAngles;

/**
 * @brief Pick object at position (x, y, z)
 *
 * Path Planning:
 * 1. Open gripper
 * 2. Rotate J0 to target direction
 * 3. Move J2 + J3 together (elbow + wrist)
 * 4. Move J1 (shoulder)
 * 5. Close gripper
 * 6. Lift up
 *
 * @param x Target X position (mm)
 * @param y Target Y position (mm)
 * @param z Target Z position (mm)
 * @return true on success, false if unreachable or error
 */
bool arm_pick(double x, double y, double z);

/**
 * @brief Place object at position (x, y, z)
 *
 * Path Planning:
 * 1. Move to place position
 * 2. Open gripper (release object)
 * 3. Return to REST position
 *
 * @param x Target X position (mm)
 * @param y Target Y position (mm)
 * @param z Target Z position (mm)
 * @return true on success, false if unreachable or error
 */
bool arm_place(double x, double y, double z);

/**
 * @brief Control gripper
 *
 * @param action "open" or "close"
 * @return true on success
 */
bool arm_gripper(const char *action);

/**
 * @brief Move arm to rest position
 *
 * Path Planning:
 * 1. Rotate J0 to 90° (Y-axis)
 * 2. Move J1 → 170°
 * 3. Move J2 → 170°
 * 4. Move J3 → 170°
 * 5. Open gripper
 * 6. Disable servos (optional, based on ESP32 support)
 *
 * @return true on success
 */
bool arm_rest(void);

/**
 * @brief Calculate geometric link angles for gravity compensation
 *
 * @param sv0 J0 servo angle
 * @param sv1 J1 servo angle
 * @param sv2 J2 servo angle
 * @param sv3 J3 servo angle
 * @param[out] angles Output link angles
 */
void arm_calculate_link_angles(double sv0, double sv1, double sv2, double sv3,
                               ArmLinkAngles *angles);

/**
 * @brief Send gravity compensation angles to ESP32
 *
 * @param sv0 J0 servo angle
 * @param sv1 J1 servo angle
 * @param sv2 J2 servo angle
 * @param sv3 J3 servo angle
 */
void arm_send_gravity_angles(double sv0, double sv1, double sv2, double sv3);

/**
 * @brief Request emergency stop for arm operations
 */
void arm_request_stop(void);

/**
 * @brief Clear emergency stop flag
 */
void arm_clear_stop(void);

/**
 * @brief Check if stop has been requested
 * @return true if stop requested
 */
bool arm_is_stop_requested(void);

/* ===== Gripper Offset Configuration ===== */
#define GRIPPER_Y_OFFSET_MM                                                    \
  70.0 /* mm - Gripper position offset from robot center along Y-axis */
#define DEFAULT_GRIP_HEIGHT                                                    \
  -180.0 /* mm - Default Z height for gripping objects */

/**
 * @brief Execute automatic grip based on object position and grip side
 *
 * This function is called when laptop sends "execute_grip" command after
 * robot has reached its destination position.
 *
 * @param robot_x Current robot X position in global frame (meters)
 * @param robot_y Current robot Y position in global frame (meters)
 * @param robot_theta Current robot heading angle (radians)
 * @param object_x Object center X position in global frame (meters)
 * @param object_y Object center Y position in global frame (meters)
 * @param object_length Object length dimension (meters)
 * @param object_width Object width dimension (meters)
 * @param grip_side "top", "bottom", "left", or "right"
 * @return true on success, false if unreachable or error
 */
bool arm_execute_grip(double robot_x, double robot_y, double robot_theta,
                      double object_x, double object_y, double object_length,
                      double object_width, const char *grip_side);

/**
 * @brief Execute automatic place based on object position
 *
 * This function is called when laptop sends "execute_place" command.
 * Robot places the object at the specified position.
 *
 * @param robot_x Current robot X position in global frame (meters)
 * @param robot_y Current robot Y position in global frame (meters)
 * @param robot_theta Current robot heading angle (radians)
 * @param object_x Object center X position in global frame (meters)
 * @param object_y Object center Y position in global frame (meters)
 * @param object_length Object length dimension (meters)
 * @param object_width Object width dimension (meters)
 * @return true on success, false if unreachable or error
 */
bool arm_execute_place(double robot_x, double robot_y, double robot_theta,
                       double object_x, double object_y, double object_length,
                       double object_width, const char *grip_side);

#endif /* ARM_CONTROLLER_H_ */
