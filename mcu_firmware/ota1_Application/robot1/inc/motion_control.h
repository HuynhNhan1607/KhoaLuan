#ifndef MOTION_CONTROL_H_
#define MOTION_CONTROL_H_

void set_robot_status(float vx, float vy, float omega_theta);
void convert_euler2radian(float *euler);

#endif