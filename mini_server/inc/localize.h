#ifndef LOCALIZE_H_
#define LOCALIZE_H_
#include "cJSON.h"
#include "json_handler.h"
#include "socket.h"
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Command type definitions
#define CMD_GET_POSITION 0x02

// Response type definitions
#define RESP_ERROR 0x40
#define RESP_POSITION 0x41

// Configurable settings
#define DEFAULT_BAUDRATE B115200
#define DEFAULT_TIMEOUT 1
#define UPDATE_INTERVAL 100000
// Butterworth filter coefficients
#define A1 0.509525
#define B1 0.245237
#define B2 0.245237

typedef struct {
  float x;
  float y;
  float z;
} Coordinates;

typedef struct {
  Coordinates coordinates;
  uint8_t quality;
  int error_code;
  bool has_coordinates;
  bool has_error;
} PositionData;

typedef struct {
  char *port;
  int baudrate;
  int timeout;
  int serial_fd;
} DWMDevice;

#define SCALE_FACTOR 1.018320f
#define ALPHA 0.0f
#define THETA 0.222712f // độ
#define XT 0.133362f
#define YT -0.064448f

extern volatile bool g_running;

// Function prototypes
bool dwm_open(DWMDevice *device);
void dwm_close(DWMDevice *device);
bool dwm_send_tlv_command(DWMDevice *device, uint8_t cmd_type, uint8_t *data,
                          size_t data_len);
bool dwm_read_tlv_response(DWMDevice *device, PositionData *position);
bool dwm_get_position(DWMDevice *device, PositionData *position);
void *localize_thread(void *arg);
bool localize_get_stable_position(Coordinates *stable_pos);
void localize_force_recalculate_stable_position(void);

#endif