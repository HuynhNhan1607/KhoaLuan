#ifndef EKF_H_
#define EKF_H_

#include <stdbool.h>

#define EKF_STATE_DIM 5

#ifndef UWB_CHI2_GATE
#define UWB_CHI2_GATE \
  50.0 // Tăng lên để chấp nhận innovation lớn hơn (cho phép EKF converge từ bất
       // kỳ vị trí ban đầu nào)
#endif

typedef struct
{
  double x[EKF_STATE_DIM];                // State vector [x, y, theta]
  double P[EKF_STATE_DIM][EKF_STATE_DIM]; // Covariance matrix
  double Q[EKF_STATE_DIM][EKF_STATE_DIM]; // Process noise
  double F[EKF_STATE_DIM][EKF_STATE_DIM]; // State transition
} ekf_t;

typedef enum
{
  EKF_SENSOR_IMU = 0,
  EKF_SENSOR_LOCALIZATION,
  EKF_SENSOR_OPTICAL_FLOW
} ekf_sensor_t;

double dt_from_last_predict();
void ekf_update_sensor(ekf_t *ekf, ekf_sensor_t sensor, const double *z);
void ekf_predict_input_body(ekf_t *ekf, double vxb, double vyb, double dt);
void ekf_predict_input_global(ekf_t *ekf, double vxg, double vyg, double dt);
void ekf_init_default(ekf_t *ekf);
void ekf_init_with_position(ekf_t *ekf, double x, double y);

bool ekf_update_pos_vel_xyvxvy(ekf_t *ekf, double z_x, double z_y, double z_vx,
                               double z_vy, double R_x, double R_y, double R_vx,
                               double R_vy, double chi2_gate);

void ekf_publish_position(const ekf_t *ekf);

#endif