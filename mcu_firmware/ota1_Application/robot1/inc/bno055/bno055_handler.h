#ifndef BNO055_HANDLER_H
#define BNO055_HANDLER_H

#include "bno055.h"

#define REINIT_TIME 2500
// BNO polling period.
#define BNO_POLLING_MS 20

void imu_plus_task(void *pvParameters);
void reinit_sensor(void *pvParameters);
void dynamic_offset_update_task(void *pvParameters);

float get_heading();

void get_accel(bno055_vec3_t *accel);

void bno055_start(void);

void waitBNO055Calibration(void);

#endif // BNO055_HANDLER_H