#ifndef NVS_HANDLER_H
#define NVS_HANDLER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "bno055.h"

#define LOAD_OFFSET 1

esp_err_t nvs_init(void);

esp_err_t nvs_save_bno055_calibration(bno055_offsets_t *offsets);

esp_err_t nvs_load_bno055_calibration(bno055_offsets_t *offsets);

bool nvs_has_bno055_calibration(void);

esp_err_t nvs_clear_bno055_calibration(void);

esp_err_t nvs_apply_bno055_calibration(i2c_number_t i2c_num);

#endif // NVS_HANDLER_H