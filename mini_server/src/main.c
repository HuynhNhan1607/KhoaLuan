#include "ekf.h"
#include "localize.h"
#include "optical_flow.h"
#include "socket.h"
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

// External Optical Flow variables (defined in optical_flow_integration.c)
extern optical_flow_t g_optical_flow;
extern pthread_mutex_t g_optical_mutex;
extern ekf_t g_ekf;
extern pthread_mutex_t g_ekf_mutex;
#include "trajectory_executor.h"

static struct gpiod_chip *chip = NULL;
struct gpiod_line *line = NULL;

static void on_sigint(int sig) {
  (void)sig;
  g_running = false;
  if (line) {
    gpiod_line_release(line);
  }
  if (chip) {
    gpiod_chip_close(chip);
  }
}

static void gpio_init(void) {
  chip = gpiod_chip_open_by_name("gpiochip1");
  if (!chip) {
    perror("Failed to open GPIO chip");
    return;
  }
  line = gpiod_chip_get_line(chip, ESP_EN);
  if (!line) {
    perror("Failed to get GPIO line");
    gpiod_chip_close(chip);
    chip = NULL;
    return;
  }
  if (gpiod_line_request_output(line, "ESP_EN", 0) < 0) {
    perror("Failed to request GPIO line as output");
    gpiod_chip_close(chip);
    chip = NULL;
    line = NULL;
    return;
  }
  gpiod_line_set_value(line, 1);
  printf("GPIO initialized\n");
}

int main(void) {
  signal(SIGINT, on_sigint);

  gpio_init();
  trajectory_init();

  pthread_t th_server, th_laptop_server, th_localize, th_optical_flow;

  // Start server thread (for ESP32 to connect)
  if (pthread_create(&th_server, NULL, server_thread, NULL) != 0) {
    perror("Failed to create server thread");
    return 1;
  }

  // Start laptop server thread (previously client_thread)
  if (pthread_create(&th_laptop_server, NULL, laptop_server_thread, NULL) !=
      0) {
    perror("Failed to create laptop server thread");
    g_running = false;
    pthread_join(th_server, NULL);
    return 1;
  }

  if (pthread_create(&th_localize, NULL, localize_thread, NULL) != 0) {
    perror("Failed to create localize thread");
    g_running = false;
    pthread_join(th_server, NULL);
    pthread_join(th_laptop_server, NULL);
    return 1;
  }

  // Start Optical Flow thread
  if (pthread_create(&th_optical_flow, NULL, optical_flow_uart_thread, NULL) !=
      0) {
    perror("Failed to create optical flow thread");
    g_running = false;
    pthread_join(th_server, NULL);
    pthread_join(th_laptop_server, NULL);
    pthread_join(th_localize, NULL);
    return 1;
  }

  printf("Mini-Server running. Press Ctrl-C to stop.\n");

  // Wait for threads to complete
  pthread_join(th_server, NULL);
  pthread_join(th_laptop_server, NULL);
  pthread_join(th_localize, NULL);
  pthread_join(th_optical_flow, NULL);

  return 0;
}