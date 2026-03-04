#include "stdio.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "sys_config.h"

#include "wifi_handler.h"
#include "socket.h"
#include "kinematic.h"
#include "motor_driver.h"

#include "bno055_handler.h"

static const char *TAG = "Main";

extern EventGroupHandle_t bno055_event_group;

void app_main(void)
{

    if (server_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize server");
        return;
    }
    motor_driver_init();
    send_pid_data();
    server_tasks_start();
    printf("Firmware 1\n");
#if ENABLE_IMU == 1
    bno055_start();
    waitBNO055Calibration();
#endif
    motor_control_start();
}
