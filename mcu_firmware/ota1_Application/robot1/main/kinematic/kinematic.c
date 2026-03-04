#include "kinematic.h"

void CalculateWheelSpeed(float *WheelSpeed, float yaw_angle,
                         float lateral_velocity, float longitudinal_velocity,
                         float yaw_speed) {
  const float cos_yaw = cosf(yaw_angle);
  const float sin_yaw = sinf(yaw_angle);

  const float cos_minus_sin = cos_yaw - sin_yaw;
  const float cos_plus_sin = cos_yaw + sin_yaw;

  const float velocity_component_w13 =
      cos_plus_sin * lateral_velocity - cos_minus_sin * longitudinal_velocity;
  const float velocity_component_w24 =
      cos_minus_sin * lateral_velocity + cos_plus_sin * longitudinal_velocity;

  const float yaw_component = YAW_COEFFICIENT * yaw_speed;

  WheelSpeed[0] = INV_WHEEL_RADIUS * (velocity_component_w13 + yaw_component);
  WheelSpeed[1] = INV_WHEEL_RADIUS * (velocity_component_w24 + yaw_component);
  WheelSpeed[2] = INV_WHEEL_RADIUS * (-velocity_component_w13 + yaw_component);
  WheelSpeed[3] = INV_WHEEL_RADIUS * (-velocity_component_w24 + yaw_component);
}

// static inline void rotate_odometry(odometry_t *odom)
// {
//     float theta = get_heading(); // lấy góc rad từ IMU hoặc EKF
//     convert_euler2radian(&theta);
//     float cos_t = cos(theta);
//     float sin_t = sin(theta);
//     // Xoay velocity
//     float vx = odom->velocity[X_AXIS];
//     float vy = odom->velocity[Y_AXIS];
//     odom->velocity[X_AXIS] = vx * cos_t - vy * sin_t;
//     odom->velocity[Y_AXIS] = vx * sin_t + vy * cos_t;
// }
void EstimateRobotPosition(odometry_t *odom, float yaw_angle,
                           wheel_infor_t *wheel_infor, float DeltaTime) {
  const float cos_yaw = cosf(yaw_angle);
  const float sin_yaw = sinf(yaw_angle);

  const float cos_minus_sin = cos_yaw - sin_yaw;
  const float cos_plus_sin = cos_yaw + sin_yaw;

  odom->velocity[X_AXIS] = (cos_plus_sin * wheel_infor[0].WheelSpeed +
                            cos_minus_sin * wheel_infor[1].WheelSpeed -
                            cos_plus_sin * wheel_infor[2].WheelSpeed -
                            cos_minus_sin * wheel_infor[3].WheelSpeed) *
                           WHEEL_RADIUS_SCALE;

  odom->velocity[Y_AXIS] = (-cos_minus_sin * wheel_infor[0].WheelSpeed +
                            cos_plus_sin * wheel_infor[1].WheelSpeed +
                            cos_minus_sin * wheel_infor[2].WheelSpeed -
                            cos_plus_sin * wheel_infor[3].WheelSpeed) *
                           WHEEL_RADIUS_SCALE;

  odom->yaw_rate = WHEEL_RADIUS_SCALE * (1 / YAW_COEFFICIENT) *
                   (wheel_infor[0].WheelSpeed + wheel_infor[1].WheelSpeed +
                    wheel_infor[2].WheelSpeed + wheel_infor[3].WheelSpeed);

  // rotate_odometry(odom);

  odom->distance[X_AXIS] += odom->velocity[X_AXIS] * DeltaTime;
  odom->distance[Y_AXIS] += odom->velocity[Y_AXIS] * DeltaTime;

  odom->yaw_angle += odom->yaw_rate * DeltaTime;
}