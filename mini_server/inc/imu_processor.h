#ifndef IMU_PROCESSOR_H
#define IMU_PROCESSOR_H

#include <stdbool.h>

// Initialize the IMU processor
bool imu_processor_init(void);

// Process raw accelerometer data and return filtered values
void imu_process_accel(float accel_x, float accel_y, float accel_z,
                       float *filtered_accel_x, float *filtered_accel_y,
                       float *filtered_accel_z);

#endif // IMU_PROCESSOR_H