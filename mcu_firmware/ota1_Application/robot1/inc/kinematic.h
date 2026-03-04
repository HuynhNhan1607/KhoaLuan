#ifndef KINEMATIC_H_
#define KINEMATIC_H_
#include <math.h>
#include "sys_config.h"
#include "motor_driver.h"
#include "bno055_handler.h"
#include "motion_control.h"

#define INV_WHEEL_RADIUS (1.0f / WHEEL_RADIUS)
#define YAW_COEFFICIENT (-sqrtf(2.0) * ROBOT_RADIUS * sinf(M_PI / 4.0 + WHEEL_ANGLE_OFFSET_FROM_DIAGONAL))
#define WHEEL_RADIUS_SCALE (WHEEL_RADIUS / 4.0)

typedef enum
{
    X_AXIS = 0,
    Y_AXIS = 1
} axis_t;

typedef struct
{
    float velocity[2];
    float yaw_rate;
    float distance[2];
    float yaw_angle;
} odometry_t;

void CalculateWheelSpeed(float *WheelSpeed, float yaw_angle,
                         float lateral_velocity, float longitudinal_velocity, float yaw_speed);
void EstimateRobotPosition(odometry_t *odom, float yaw_angle, wheel_infor_t *wheel_infor, float DeltaTime);

#endif
