/**
 * TÍCH HỢP OPTICAL FLOW VÀO HỆ THỐNG EKF
 *
 * File này xử lý đọc dữ liệu từ UART Optical Flow MTF-02,
 * tích phân để tính position, và update vào EKF.
 */

#include "ekf.h"
#include "json_handler.h"
#include "optical_flow.h"
#include "socket.h"
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

// Global Optical Flow variables (defined here)
optical_flow_t g_optical_flow;
pthread_mutex_t g_optical_mutex = PTHREAD_MUTEX_INITIALIZER;

// External variables from other modules
extern ekf_t g_ekf;
extern pthread_mutex_t g_ekf_mutex;
extern volatile bool g_running;

// Position integration state
static double of_pos_x = 0.0;
static double of_pos_y = 0.0;
static bool of_position_initialized = false;
static struct timespec last_integration_time = {0};

// Angular velocity calculation state
static double last_theta = 0.0;
static struct timespec last_theta_time = {0};
static bool theta_initialized = false;

// Khởi tạo position từ localization
void optical_flow_set_initial_position(double x, double y)
{
  of_pos_x = x;
  of_pos_y = y;
  of_position_initialized = true;
  clock_gettime(CLOCK_MONOTONIC, &last_integration_time);
  printf("[Optical Flow] Position initialized: (%.3f, %.3f)\n", x, y);
}
/**
 * Thread xử lý UART Optical Flow MTF-02
 * Đọc dữ liệu 50Hz từ sensor, tích phân position và cập nhật vào EKF
 */
void *optical_flow_uart_thread(void *arg)
{
  (void)arg;

  // Mở cổng UART
  int fd = open("/dev/ttyTHS0", O_RDWR | O_NOCTTY);
  if (fd < 0)
  {
    perror("[Optical Flow] Lỗi mở UART /dev/ttyTHS0");
    fprintf(stderr,
            "[Optical Flow] Không thể mở UART. Kiểm tra quyền và đường dẫn.\n");
    return NULL;
  }
  printf("[Optical Flow] UART /dev/ttyTHS0 opened successfully (fd=%d)\n", fd);

  // Cấu hình UART 115200 baud
  struct termios tty;
  memset(&tty, 0, sizeof(tty));
  if (tcgetattr(fd, &tty) != 0)
  {
    perror("[Optical Flow] Error getting UART attributes");
    close(fd);
    return NULL;
  }

  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8 | CLOCAL | CREAD;
  tty.c_iflag = IGNPAR;
  tty.c_oflag = 0;
  tty.c_lflag = 0;
  tty.c_cc[VMIN] = 0;  // Non-blocking read
  tty.c_cc[VTIME] = 1; // 0.1 second timeout

  if (tcsetattr(fd, TCSANOW, &tty) != 0)
  {
    perror("[Optical Flow] Error setting UART attributes");
    close(fd);
    return NULL;
  }

  printf("[Optical Flow] UART configured: 115200 baud, 8N1, non-blocking\n");

  // Khởi tạo optical flow processor
  optical_flow_init(&g_optical_flow);

  uint8_t buffer[1024];
  int read_count = 0;
  int valid_packet_count = 0;
  int total_bytes_received = 0;
  int empty_reads = 0;

  printf("[Optical Flow] Thread started, reading at 50Hz...\n");

  while (g_running)
  {
    int n = read(fd, buffer, sizeof(buffer));
    if (n > 0)
    {
      read_count++;
      total_bytes_received += n;
      empty_reads = 0; // Reset empty counter

      // Log first successful read
      if (read_count == 1)
      {
        printf("[Optical Flow] ✓ Data receiving started\n");
      }

      // Tìm header Micolink: EF 0F 00 51
      for (int i = 0; i < n - 25; i++)
      {
        if (buffer[i] == 0xEF && buffer[i + 1] == 0x0F &&
            buffer[i + 3] == 0x51)
        {
          valid_packet_count++;

          // Giải mã dữ liệu
          int16_t vx_raw = (int16_t)(buffer[i + 18] | (buffer[i + 19] << 8));
          int16_t vy_raw = (int16_t)(buffer[i + 20] | (buffer[i + 21] << 8));
          uint8_t quality = buffer[i + 22];

          // Lấy góc theta từ EKF và tính omega_z bằng numerical derivative
          struct timespec now;
          clock_gettime(CLOCK_MONOTONIC, &now);

          double theta, omega_z;
          pthread_mutex_lock(&g_ekf_mutex);
          theta = g_ekf.x[4]; // Heading
          pthread_mutex_unlock(&g_ekf_mutex);

          // Tính omega_z = dθ/dt (EKF already normalizes theta to [-PI, PI])
          if (theta_initialized)
          {
            double dt = (now.tv_sec - last_theta_time.tv_sec) +
                        (now.tv_nsec - last_theta_time.tv_nsec) * 1e-9;

            if (dt > 0.001 && dt < 0.5)
            { // Clamp dt: 1ms to 500ms
              double d_theta = theta - last_theta;

              // ===== FIX: Xử lý wrap-around của theta =====
              // Khi theta wrap từ +PI sang -PI (hoặc ngược lại),
              // d_theta sẽ sai lệch ~2*PI, gây spike omega_z
              if (d_theta > M_PI)
                d_theta -= 2.0 * M_PI;
              else if (d_theta < -M_PI)
                d_theta += 2.0 * M_PI;

              omega_z = d_theta / dt;
            }
            else
            {
              omega_z = 0.0; // Invalid dt, assume zero rotation
            }
          }
          else
          {
            omega_z = 0.0; // First sample, no derivative yet
            theta_initialized = true;
          }

          // Update state for next iteration
          last_theta = theta;
          last_theta_time = now;

          // Xử lý dữ liệu qua offset correction, LPF và coordinate transform
          pthread_mutex_lock(&g_optical_mutex);

          bool valid = optical_flow_process(&g_optical_flow, vx_raw, vy_raw,
                                            theta, omega_z, quality);
          pthread_mutex_unlock(&g_optical_mutex);

          if (valid)
          {
            // Lấy velocity đã filter (global frame)
            double vx_global, vy_global;
            uint8_t flow_quality;
            pthread_mutex_lock(&g_optical_mutex);
            optical_flow_get_velocity(&g_optical_flow, &vx_global, &vy_global);
            flow_quality = g_optical_flow.quality;
            pthread_mutex_unlock(&g_optical_mutex);

            // Tích phân position nếu đã khởi tạo
            if (of_position_initialized)
            {
              struct timespec now;
              clock_gettime(CLOCK_MONOTONIC, &now);

              double dt = (now.tv_sec - last_integration_time.tv_sec) +
                          (now.tv_nsec - last_integration_time.tv_nsec) * 1e-9;

              if (dt > 0 && dt < 0.5) // Clamp dt
              {
                of_pos_x += vx_global * dt;
                of_pos_y += vy_global * dt;
              }

              last_integration_time = now;
            }

            // Gửi JSON message để xử lý tập trung trong json_handler.c
            char json_msg[256];
            snprintf(json_msg, sizeof(json_msg),
                     "{\"id\":\"robot1\",\"type\":\"position\",\"source\":"
                     "\"optical_flow\","
                     "\"data\":{\"position\":[%.6f,%.6f],\"velocity\":[%.6f,%."
                     "6f],\"quality\":%d}}\n",
                     of_pos_x, of_pos_y, vx_global, vy_global, flow_quality);

            // Gửi lên server (chỉ gửi 1 lần tại đây)
            send_to_upstream_server(json_msg, strlen(json_msg));

            // Thêm vào handler để update EKF và log database
            json_handler_add_message(json_msg, strlen(json_msg));
          }

          i += 25;
        }
      }
    }
    else if (n < 0)
    {
      perror("[Optical Flow] Lỗi đọc UART");
    }
    else
    {
      // n == 0: no data available
      empty_reads++;

      // Warn after 5 seconds of no data
      if (empty_reads == 250)
      {
        printf("[Optical Flow] ⚠ Warning: No data received for 5 seconds\n");
      }
    }

    usleep(20000); // 50Hz
  }

  close(fd);
  printf("[Optical Flow] Thread stopped\n");
  return NULL;
}
