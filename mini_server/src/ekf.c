#include "time.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define _USE_MATH_DEFINES
#include "ekf.h"
#include "json_handler.h"
#include "mat_util.h"
#include "socket.h"
#include "sys_config.h"

static inline double wrap_pi(double a)
{
  while (a > M_PI)
    a -= 2 * M_PI;
  while (a <= -M_PI)
    a += 2 * M_PI;
  return a;
}

static struct timespec s_last_predict_ts = {0};

double dt_from_last_predict()
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  double t = ts.tv_sec + ts.tv_nsec * 1e-9;
  double t0 = s_last_predict_ts.tv_sec + s_last_predict_ts.tv_nsec * 1e-9;
  if (s_last_predict_ts.tv_sec == 0 && s_last_predict_ts.tv_nsec == 0)
  {
    s_last_predict_ts = ts;
    return 0.0;
  }
  s_last_predict_ts = ts;
  double dt = t - t0;
  if (dt < 0)
    dt = 0;
  if (dt > 0.3)
    dt = 0.3; // clamp an toàn
  return dt;
}

void ekf_publish_position(const ekf_t *ekf)
{
  char json[256];
  snprintf(json, sizeof(json),
           "{\"id\":\"%s\",\"type\":\"position\",\"source\":\"ekf\",\"data\":{"
           "\"position\":[%.6f,%.6f,%.6f,%.6f,%.6f]}}\n",
           "robot1", ekf->x[0], ekf->x[1], ekf->x[2], ekf->x[3], ekf->x[4]);
  send_to_upstream_server(json, strlen(json));
  // printf("[EKF] send: [x: %.3f, y: %.3f, vx: %.3f, vy: %.3f, th: %.3f]",
  // ekf->x[0], ekf->x[1], ekf->x[2], ekf->x[3], ekf->x[4]); printf("[EKF] Send:
  // %s", json);
  json_handler_add_message(json, strlen(json));
}

static void ekf_init_core(ekf_t *ekf, const double *x0, const double *P0,
                          const double *Q0)
{
  memset(ekf, 0, sizeof(*ekf));
  memcpy(ekf->x, x0, sizeof(double) * EKF_STATE_DIM);
  memcpy(ekf->P, P0, sizeof(double) * EKF_STATE_DIM * EKF_STATE_DIM);
  memcpy(ekf->Q, Q0, sizeof(double) * EKF_STATE_DIM * EKF_STATE_DIM);
  double I[EKF_STATE_DIM * EKF_STATE_DIM];
  mat_identity(I, EKF_STATE_DIM);
  mat_copy((double *)ekf->F, I, EKF_STATE_DIM, EKF_STATE_DIM);
}

void ekf_init_default(ekf_t *ekf)
{
  double x0[5] = {0};
  // Tăng uncertainty ban đầu để EKF có thể accept localization data từ xa
  // clang-format off
  double P0[25] = {
    1.0*1.0,    0,          0,          0,          0,           // x: 1.0m
    0,          1.0*1.0,    0,          0,          0,           // y: 1.0m
    0,          0,          0.5*0.5,    0,          0,           // vx: 0.5 m/s
    0,          0,          0,          0.5*0.5,    0,           // vy: 0.5 m/s
    0,          0,          0,          0,          (3.0*M_PI/180.0)*(3.0*M_PI/180.0)  // theta: 3°
  };

  double Q0[25] = {
    1e-4,       0,          0,                  0,                  0,
    0,          1e-4,       0,                  0,                  0,
    0,          0,          0.1*0.1,            0,                  0,           // Tăng Q_vx: trust optical flow hơn
    0,          0,          0,                  0.1*0.1,            0,           // Tăng Q_vy: trust optical flow hơn
    0,          0,          0,                  0,                  0.0087*0.0087
  };
  // clang-format on
  ekf_init_core(ekf, x0, P0, Q0);
}

// Khởi tạo EKF với vị trí ban đầu từ UWB stable position
void ekf_init_with_position(ekf_t *ekf, double x, double y)
{
  double x0[5] = {x, y, 0.0, 0.0,
                  0.0}; // Vị trí từ UWB, velocity = 0, theta = 0
  // Uncertainty nhỏ hơn vì đã biết vị trí từ UWB
  // clang-format off
  double P0[25] = {
    0.3*0.3,    0,          0,          0,          0,           // x: 10cm (UWB accuracy)
    0,          0.3*0.3,    0,          0,          0,           // y: 10cm
    0,          0,          0.5*0.5,    0,          0,           // vx: 0.5 m/s
    0,          0,          0,          0.5*0.5,    0,           // vy: 0.5 m/s
    0,          0,          0,          0,          (3.0*M_PI/180.0)*(3.0*M_PI/180.0)  // theta: 3°
  };

  double Q0[25] = {
    1e-4,       0,          0,                  0,                  0,
    0,          1e-4,       0,                  0,                  0,
    0,          0,          0.05*0.05,            0,                  0,           // Tăng Q_vx: trust optical flow hơn
    0,          0,          0,                  0.05*0.05,            0,           // Tăng Q_vy: trust optical flow hơn
    0,          0,          0,                  0,                  0.0087*0.0087
  };
  // clang-format on
  ekf_init_core(ekf, x0, P0, Q0);
  printf("[EKF] Initialized with UWB position: x=%.3f, y=%.3f\n", x, y);
}

// EKF predict step with body frame velocities (TRUE EKF with theta dependency)
void ekf_predict_input_body(ekf_t *ekf, double vxb, double vyb, double dt)
{
  if (dt <= 0.0)
    return;

  double xk[EKF_STATE_DIM];
  memcpy(xk, ekf->x, sizeof(xk));
  double th = xk[4];
  double c = cos(th);
  double s = sin(th);

  // Transform body velocities to global frame using current theta
  double vxg = c * vxb - s * vyb;
  double vyg = s * vxb + c * vyb;

  // Propagate state (using global velocities computed from body frame)
  ekf->x[0] = xk[0] + vxg * dt; // x
  ekf->x[1] = xk[1] + vyg * dt; // y
  ekf->x[2] = vxg;              // vx (global)
  ekf->x[3] = vyg;              // vy (global)
  ekf->x[4] = xk[4];            // theta (constant motion model)

  // Jacobian F (NOW depends on theta - this is TRUE EKF):
  // ∂x/∂theta = dt * (-sin(th)*vxb - cos(th)*vyb) = dt * (-vxg*tan(th) - vyg/cos(th)) simplified:
  // ∂x/∂theta = dt * (-sin(th)*vxb - cos(th)*vyb)
  // ∂y/∂theta = dt * (cos(th)*vxb - sin(th)*vyb)
  ekf->F[0][2] = dt * c;                    // ∂x/∂vx
  ekf->F[0][3] = -dt * s;                   // ∂x/∂vy
  ekf->F[0][4] = dt * (-s * vxb - c * vyb); // ∂x/∂theta (EKF nonlinearity!)
  ekf->F[1][2] = dt * s;                    // ∂y/∂vx
  ekf->F[1][3] = dt * c;                    // ∂y/∂vy
  ekf->F[1][4] = dt * (c * vxb - s * vyb);  // ∂y/∂theta (EKF nonlinearity!)

  // P = F P F^T + Q*dt
  double Qd[EKF_STATE_DIM * EKF_STATE_DIM];
  memcpy(Qd, ekf->Q, sizeof(Qd));
  for (int r = 0; r < EKF_STATE_DIM; r++)
    for (int c = 0; c < EKF_STATE_DIM; c++)
      Qd[r * EKF_STATE_DIM + c] *= dt;

  double FP[EKF_STATE_DIM * EKF_STATE_DIM] = {0};
  mat_mult(FP, (double *)ekf->F, (double *)ekf->P, EKF_STATE_DIM, EKF_STATE_DIM,
           EKF_STATE_DIM);
  double Ft[EKF_STATE_DIM * EKF_STATE_DIM] = {0};
  mat_transpose(Ft, (double *)ekf->F, EKF_STATE_DIM, EKF_STATE_DIM);
  double FPFt[EKF_STATE_DIM * EKF_STATE_DIM] = {0};
  mat_mult(FPFt, FP, Ft, EKF_STATE_DIM, EKF_STATE_DIM, EKF_STATE_DIM);

  double Pnew[EKF_STATE_DIM * EKF_STATE_DIM] = {0};
  mat_add(Pnew, FPFt, Qd, EKF_STATE_DIM, EKF_STATE_DIM);
  mat_copy((double *)ekf->P, Pnew, EKF_STATE_DIM, EKF_STATE_DIM);
}

// Linear KF predict step with global frame velocities (NO theta dependency)
void ekf_predict_input_global(ekf_t *ekf, double vxg, double vyg, double dt)
{
  if (dt <= 0.0)
    return;

  double xk[EKF_STATE_DIM];
  memcpy(xk, ekf->x, sizeof(xk));

  // Propagate state directly with global velocities (Linear KF)
  ekf->x[0] = xk[0] + vxg * dt; // x
  ekf->x[1] = xk[1] + vyg * dt; // y
  ekf->x[2] = vxg;              // vx (global)
  ekf->x[3] = vyg;              // vy (global)
  ekf->x[4] = xk[4];            // theta (constant motion model)

  // Jacobian F (NO theta dependency - Linear KF!):
  ekf->F[0][2] = dt;  // ∂x/∂vx
  ekf->F[0][3] = 0.0; // ∂x/∂vy
  ekf->F[0][4] = 0.0; // ∂x/∂theta (NO nonlinearity!)
  ekf->F[1][2] = 0.0; // ∂y/∂vx
  ekf->F[1][3] = dt;  // ∂y/∂vy
  ekf->F[1][4] = 0.0; // ∂y/∂theta (NO nonlinearity!)

  // P = F P F^T + Q*dt
  double Qd[EKF_STATE_DIM * EKF_STATE_DIM];
  memcpy(Qd, ekf->Q, sizeof(Qd));
  for (int r = 0; r < EKF_STATE_DIM; r++)
    for (int c = 0; c < EKF_STATE_DIM; c++)
      Qd[r * EKF_STATE_DIM + c] *= dt;

  double FP[EKF_STATE_DIM * EKF_STATE_DIM] = {0};
  mat_mult(FP, (double *)ekf->F, (double *)ekf->P, EKF_STATE_DIM, EKF_STATE_DIM,
           EKF_STATE_DIM);
  double Ft[EKF_STATE_DIM * EKF_STATE_DIM] = {0};
  mat_transpose(Ft, (double *)ekf->F, EKF_STATE_DIM, EKF_STATE_DIM);
  double FPFt[EKF_STATE_DIM * EKF_STATE_DIM] = {0};
  mat_mult(FPFt, FP, Ft, EKF_STATE_DIM, EKF_STATE_DIM, EKF_STATE_DIM);

  double Pnew[EKF_STATE_DIM * EKF_STATE_DIM] = {0};
  mat_add(Pnew, FPFt, Qd, EKF_STATE_DIM, EKF_STATE_DIM);
  mat_copy((double *)ekf->P, Pnew, EKF_STATE_DIM, EKF_STATE_DIM);
}

// -------- Update yaw 1D (IMU heading) --------
void ekf_update_yaw(ekf_t *ekf, double z_theta, double R_theta)
{
  // y = wrap(z - theta)
  double y = wrap_pi(z_theta - ekf->x[4]);
  double S = ekf->P[4][4] + R_theta;
  if (S <= 0)
    return;
  double K[5];
  for (int i = 0; i < 5; i++)
    K[i] = ekf->P[i][4] / S;

  for (int i = 0; i < 5; i++)
    ekf->x[i] += K[i] * y;
  ekf->x[4] = wrap_pi(ekf->x[4]);

  // Joseph form
  double I[25];
  mat_identity(I, 5);
  double KH[25] = {0}; // K * [0 0 0 0 1]
  for (int r = 0; r < 5; r++)
    for (int c = 0; c < 5; c++)
      KH[r * 5 + c] = K[r] * (c == 4);
  double I_KH[25] = {0};
  mat_sub(I_KH, I, KH, 5, 5);
  double tmp[25] = {0};
  mat_mult(tmp, I_KH, (double *)ekf->P, 5, 5, 5);
  double I_KH_t[25] = {0};
  mat_transpose(I_KH_t, I_KH, 5, 5);
  double Pnew[25] = {0};
  mat_mult(Pnew, tmp, I_KH_t, 5, 5, 5);
  for (int r = 0; r < 5; r++)
    for (int c = 0; c < 5; c++)
      Pnew[r * 5 + c] += K[r] * R_theta * K[c];
  mat_copy((double *)ekf->P, Pnew, 5, 5);
}

// -------- Update UWB (x,y) 2D với Mahalanobis gating --------
bool ekf_update_pos_xy(ekf_t *ekf, double zx, double zy, double Rx, double Ry,
                       double chi2_gate)
{
  // clang-format off
  // H = [[1 0 0 0 0],
  //      [0 1 0 0 0]]
  double H[2 * 5] = {
    1, 0, 0, 0, 0,
    0, 1, 0, 0, 0
  };
  // clang-format on
  double yv[2] = {zx - ekf->x[0], zy - ekf->x[1]};

  double HP[2 * 5] = {0};
  mat_mult(HP, H, (double *)ekf->P, 2, 5, 5);
  double Ht[5 * 2] = {0};
  mat_transpose(Ht, H, 2, 5);
  double S[4] = {0};
  mat_mult(S, HP, Ht, 2, 5, 2);
  S[0] += Rx;
  S[3] += Ry;

  double Sinv[4] = {0};
  if (!mat_inv(Sinv, S, 2))
    return false;
  double d2 = yv[0] * (Sinv[0] * yv[0] + Sinv[1] * yv[1]) +
              yv[1] * (Sinv[2] * yv[0] + Sinv[3] * yv[1]);
  if (d2 > chi2_gate)
  {
    printf("[EKF] Localization REJECTED: d2=%.2f > gate=%.2f, "
           "innovation=[%.3f, %.3f], ekf_pos=[%.3f, %.3f]\n",
           d2, chi2_gate, yv[0], yv[1], ekf->x[0], ekf->x[1]);
    return false;
  }
  // printf("[EKF] Localization ACCEPTED: d2=%.2f < gate=%.2f, innovation=[%.3f,
  // %.3f]\n",
  //        d2, chi2_gate, yv[0], yv[1]);

  double PHt[5 * 2] = {0};
  mat_mult(PHt, (double *)ekf->P, Ht, 5, 5, 2);
  double K[5 * 2] = {0};
  mat_mult(K, PHt, Sinv, 5, 2, 2);

  for (int i = 0; i < 5; i++)
    ekf->x[i] += K[i * 2 + 0] * yv[0] + K[i * 2 + 1] * yv[1];
  ekf->x[4] = wrap_pi(ekf->x[4]);

  // Joseph form
  double I[25];
  mat_identity(I, 5);
  double KH[25] = {0};
  mat_mult(KH, K, H, 5, 2, 5);
  double I_KH[25] = {0};
  mat_sub(I_KH, I, KH, 5, 5);
  double tmp[25] = {0};
  mat_mult(tmp, I_KH, (double *)ekf->P, 5, 5, 5);
  double I_KH_t[25] = {0};
  mat_transpose(I_KH_t, I_KH, 5, 5);
  double Pnew[25] = {0};
  mat_mult(Pnew, tmp, I_KH_t, 5, 5, 5);

  for (int r = 0; r < 5; r++)
    for (int c = 0; c < 5; c++)
      Pnew[r * 5 + c] +=
          K[r * 2 + 0] * Rx * K[c * 2 + 0] + K[r * 2 + 1] * Ry * K[c * 2 + 1];

  mat_copy((double *)ekf->P, Pnew, 5, 5);
  return true;
}

// -------- Update Position + Velocity (x,y,vx,vy) 4D với Mahalanobis gating
// --------
bool ekf_update_pos_vel_xyvxvy(ekf_t *ekf, double z_x, double z_y, double z_vx,
                               double z_vy, double R_x, double R_y, double R_vx,
                               double R_vy, double chi2_gate)
{
  // clang-format off
  // H = [[1 0 0 0 0],
  //      [0 1 0 0 0],
  //      [0 0 1 0 0],
  //      [0 0 0 1 0]] - observe x, y, vx, vy
  double H[4 * 5] = {
    1, 0, 0, 0, 0,
    0, 1, 0, 0, 0,
    0, 0, 1, 0, 0,
    0, 0, 0, 1, 0
  };
  // clang-format on
  double yv[4] = {z_x - ekf->x[0], z_y - ekf->x[1], z_vx - ekf->x[2],
                  z_vy - ekf->x[3]};

  double HP[4 * 5] = {0};
  mat_mult(HP, H, (double *)ekf->P, 4, 5, 5);
  double Ht[5 * 4] = {0};
  mat_transpose(Ht, H, 4, 5);
  double S[16] = {0};
  mat_mult(S, HP, Ht, 4, 5, 4);
  S[0] += R_x;
  S[5] += R_y;
  S[10] += R_vx;
  S[15] += R_vy;

  double Sinv[16] = {0};
  if (!mat_inv(Sinv, S, 4))
    return false;

  // Mahalanobis distance: d2 = y^T * S^-1 * y
  double d2 = 0.0;
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      d2 += yv[i] * Sinv[i * 4 + j] * yv[j];
    }
  }
  if (d2 > chi2_gate)
  {
    printf("[EKF] OpticalFlow REJECTED: d2=%.2f > gate=%.2f, innovation=[%.3f, "
           "%.3f, %.3f, %.3f]\n",
           d2, chi2_gate, yv[0], yv[1], yv[2], yv[3]);
    return false;
  }

  double PHt[5 * 4] = {0};
  mat_mult(PHt, (double *)ekf->P, Ht, 5, 5, 4);
  double K[5 * 4] = {0};
  mat_mult(K, PHt, Sinv, 5, 4, 4);

  for (int i = 0; i < 5; i++)
    for (int j = 0; j < 4; j++)
      ekf->x[i] += K[i * 4 + j] * yv[j];
  ekf->x[4] = wrap_pi(ekf->x[4]);

  // Joseph form
  double I[25];
  mat_identity(I, 5);
  double KH[25] = {0};
  mat_mult(KH, K, H, 5, 4, 5);
  double I_KH[25] = {0};
  mat_sub(I_KH, I, KH, 5, 5);
  double tmp[25] = {0};
  mat_mult(tmp, I_KH, (double *)ekf->P, 5, 5, 5);
  double I_KH_t[25] = {0};
  mat_transpose(I_KH_t, I_KH, 5, 5);
  double Pnew[25] = {0};
  mat_mult(Pnew, tmp, I_KH_t, 5, 5, 5);

  // R diagonal contribution
  double R_diag[4] = {R_x, R_y, R_vx, R_vy};
  for (int r = 0; r < 5; r++)
    for (int c = 0; c < 5; c++)
      for (int k = 0; k < 4; k++)
        Pnew[r * 5 + c] += K[r * 4 + k] * R_diag[k] * K[c * 4 + k];

  mat_copy((double *)ekf->P, Pnew, 5, 5);
  return true;
}

// -------- Update Velocity (vx,vy) 2D với Mahalanobis gating --------
bool ekf_update_vel_vxvy(ekf_t *ekf, double z_vx, double z_vy, double R_vx,
                         double R_vy, double chi2_gate)
{
  // clang-format off
  // H = [[0 0 1 0 0],
  //      [0 0 0 1 0]] - observe vx, vy
  double H[2 * 5] = {
    0, 0, 1, 0, 0,
    0, 0, 0, 1, 0
  };
  // clang-format on
  double yv[2] = {z_vx - ekf->x[2], z_vy - ekf->x[3]};

  double HP[2 * 5] = {0};
  mat_mult(HP, H, (double *)ekf->P, 2, 5, 5);
  double Ht[5 * 2] = {0};
  mat_transpose(Ht, H, 2, 5);
  double S[4] = {0};
  mat_mult(S, HP, Ht, 2, 5, 2);
  S[0] += R_vx;
  S[3] += R_vy;

  double Sinv[4] = {0};
  if (!mat_inv(Sinv, S, 2))
    return false;

  // Mahalanobis distance
  double d2 = yv[0] * (Sinv[0] * yv[0] + Sinv[1] * yv[1]) +
              yv[1] * (Sinv[2] * yv[0] + Sinv[3] * yv[1]);

  if (d2 > chi2_gate)
  {
    printf("[EKF] OpticalFlow Vel REJECTED: d2=%.2f > gate=%.2f, "
           "innovation=[%.3f, %.3f]\n",
           d2, chi2_gate, yv[0], yv[1]);
    return false;
  }

  double PHt[5 * 2] = {0};
  mat_mult(PHt, (double *)ekf->P, Ht, 5, 5, 2);
  double K[5 * 2] = {0};
  mat_mult(K, PHt, Sinv, 5, 2, 2);

  for (int i = 0; i < 5; i++)
    ekf->x[i] += K[i * 2 + 0] * yv[0] + K[i * 2 + 1] * yv[1];
  ekf->x[4] = wrap_pi(ekf->x[4]);

  // Joseph form update covariance
  double I[25];
  mat_identity(I, 5);
  double KH[25] = {0};
  mat_mult(KH, K, H, 5, 2, 5);
  double I_KH[25] = {0};
  mat_sub(I_KH, I, KH, 5, 5);
  double tmp[25] = {0};
  mat_mult(tmp, I_KH, (double *)ekf->P, 5, 5, 5);
  double I_KH_t[25] = {0};
  mat_transpose(I_KH_t, I_KH, 5, 5);
  double Pnew[25] = {0};
  mat_mult(Pnew, tmp, I_KH_t, 5, 5, 5);

  // R diagonal contribution
  double R_diag[2] = {R_vx, R_vy};
  for (int r = 0; r < 5; r++)
    for (int c = 0; c < 5; c++)
      for (int k = 0; k < 2; k++)
        Pnew[r * 5 + c] += K[r * 2 + k] * R_diag[k] * K[c * 2 + k];

  mat_copy((double *)ekf->P, Pnew, 5, 5);
  return true;
}

static inline double deg2rad(double deg) { return deg * M_PI / 180.0; }

void ekf_update_sensor(ekf_t *ekf, ekf_sensor_t sensor, const double *z)
{
  switch (sensor)
  {
  case EKF_SENSOR_IMU:
  {
    // z[0] = yaw (radian). Gợi ý R_theta ~ (1 deg)^2
    double theta_rad = deg2rad(z[0]);
    ekf_update_yaw(ekf, theta_rad, (1.0 * M_PI / 180.0) * (1.0 * M_PI / 180.0));
  }
  break;
  case EKF_SENSOR_LOCALIZATION:
  {
    // z[0]=x (m), z[1]=y (m). Gợi ý Rx=Ry=0.12^2, gating 9.21
    bool accepted =
        ekf_update_pos_xy(ekf, z[0], z[1], 0.3 * 0.3, 0.3 * 0.3, 13.81);
    if (!accepted)
    {
      printf("[EKF] WARNING: Localization update was rejected by gating!\n");
    }
  }
  break;
  case EKF_SENSOR_OPTICAL_FLOW:
  {

#if ROBOT_ID == 2 || OPTICAL_FLOW_UPDATE_POSITION == 1
    // Robot2: UPDATE FULL STATE x, y, vx, vy (cần mat_inv 4x4)
    // z[0]=x, z[1]=y, z[2]=vx, z[3]=vy. R ~ (0.05)^2
    ekf_update_pos_vel_xyvxvy(ekf, z[0], z[1], z[2], z[3], 0.3 * 0.3,
                              0.3 * 0.3, 0.05 * 0.05, 0.05 * 0.05,
                              13.82); // chi2_gate for 4 DOF, 99%
#else
    // Robot1: CHỈ UPDATE POSITION x, y từ optical flow
    // z[0]=x (m), z[1]=y (m). R ~ (0.3)^2
    ekf_update_vel_vxvy(ekf, z[2], z[3], 0.05 * 0.05, 0.05 * 0.05, 9.21);
#endif
  }
  break;
  default: /* bỏ qua */
    break;
  }
}
