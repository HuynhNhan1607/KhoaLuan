#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "socket.h"

#include "bno055.h"
#include "bno055_handler.h"

#include "nvs_handler.h"
#include "sys_config.h"
#include "string.h"

#include <math.h>

#define BNO_MODE OPERATION_MODE_IMUPLUS

static const char *TAG_IMU = "BNO055_Handler";

typedef struct bno055_tasks_t
{
    TaskHandle_t blink_led;
    TaskHandle_t imu_plus;
    TaskHandle_t calib;
} bno055_tasks_t;

typedef struct bno055_sensor_data_t
{
    bno055_euler_t euler;     // Orientation (heading, roll, pitch)
    bno055_quaternion_t quat; // Quaternion representation
    bno055_vec3_t lin_accel;  // Linear acceleration
    bno055_vec3_t gravity;    // Gravity vector
} bno055_sensor_data_t;

typedef struct bno055_calibration_t
{
    float yaw_offset;       // Heading reference offset
    bool apply_yaw_offset;  // Whether to apply yaw offset
    float adjusted_heading; // Adjusted heading value
} bno055_calibration_t;

typedef struct bno055_sync_t
{
    SemaphoreHandle_t heading_mutex; // Mutex for heading access
    SemaphoreHandle_t offset_mutex;  // Mutex for offset
} bno055_sync_t;

EventGroupHandle_t bno055_event_group = NULL; // Event group for state flags
static bno055_config_t bno_conf;              // BNO055 configuration
static i2c_number_t i2c_num = 0;              // I2C bus number
// Khởi tạo instances của các struct
static bno055_tasks_t tasks = {NULL, NULL, NULL};
static bno055_sensor_data_t sensor_data = {0};
static bno055_calibration_t calibration = {0};
static bno055_sync_t bno055_sync = {NULL, NULL};

float get_heading()
{
    float result = 0.0f;
    if (bno055_sync.heading_mutex == NULL)
    {
        // ESP_LOGW(TAG_IMU, "%s, Mutex not initialized", __func__);
        return result;
    }

    if (bno055_sync.heading_mutex != NULL && xSemaphoreTake(bno055_sync.heading_mutex, pdMS_TO_TICKS(5)) == pdTRUE)
    {
        result = -calibration.adjusted_heading; // theta bno055 reverse to mecanum
        xSemaphoreGive(bno055_sync.heading_mutex);
    }
    else
    {
        ESP_LOGW(TAG_IMU, "%s, Failed mutex", __func__);
    }
    return result;
}

void handle_sensor_error(i2c_number_t i2c_num, esp_err_t err_code)
{
    TaskHandle_t reinit_task_handle = NULL;
    ESP_LOGE(TAG_IMU, "%s: BNO055 sensor error: 0x%X", __func__, err_code);

    esp_err_t err = bno055_close(i2c_num);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG_IMU, "%s: returned: 0x%X", __func__, err);
    }

    xTaskCreatePinnedToCore(reinit_sensor, "reinit_sensor", 4096, NULL, 5, &reinit_task_handle, 0);

    if (tasks.imu_plus != NULL)
    {
        vTaskSuspend(tasks.imu_plus);
    }
}

void send_calibration_notification(calib_status_t status)
{
    char message[256];
    int message_len = snprintf(message, sizeof(message),
                               "{"
                               "\"id\":\"%s\","
                               "\"type\":\"bno055\","
                               "\"data\":{"
                               "\"event\":\"calibration_complete\","
                               "\"status\":{\"sys\":%d,\"gyro\":%d,\"accel\":%d,\"mag\":%d}"
                               "}"
                               "}\n",
                               ID_ROBOT, status.sys, status.gyro, status.accel, status.mag);

    if (socket_send(message, message_len) != ESP_OK)
    {
        ESP_LOGE(TAG_IMU, "%s, Send Calib Notification Fail", __func__);
    }
}

void bno055_set_reference(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG_IMU, "Setting reference point using Welford's algorithm...");

    esp_err_t err;
    const int SAMPLE_COUNT = 300;
    const float STD_DEV_THRESHOLD_HEADING = 1.0f;

    int heading_n = 0;
    float heading_mean = 0.0f, heading_M2 = 0.0f;

    bno055_euler_t local_euler = {0.0f, 0.0f, 0.0f};

    ESP_LOGI(TAG_IMU, "Collecting %d samples for reference calculation...", SAMPLE_COUNT);

    // Thu thập mẫu và cập nhật thống kê theo thời gian thực
    for (int i = 0; i < SAMPLE_COUNT; i++)
    {
        if (i % 10 == 0)
        {
            ESP_LOGI(TAG_IMU, "Collecting sample %d/%d...", i, SAMPLE_COUNT);
        }

        err = bno055_get_euler(i2c_num, &local_euler);

        if (err != ESP_OK)
        {
            ESP_LOGE(TAG_IMU, "%s: Error reading sensor data: 0x%X", __func__, err);
            continue;
        }

        heading_n++;

        float heading_delta = local_euler.heading - heading_mean;
        heading_mean += heading_delta / heading_n;
        float heading_delta2 = local_euler.heading - heading_mean;
        heading_M2 += heading_delta * heading_delta2;

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Tính độ lệch chuẩn cuối cùng
    float heading_std_dev = sqrtf(heading_M2 / heading_n);

    ESP_LOGI(TAG_IMU, "Stats - Heading: mean=%.2f, std_dev=%.4f",
             heading_mean, heading_std_dev);

    // Xác định xem giá trị có đủ ổn định để thiết lập tham chiếu
    bool heading_stable = (heading_std_dev < STD_DEV_THRESHOLD_HEADING);
    // Thiết lập giá trị tham chiếu
    if (heading_stable)
    {
        calibration.yaw_offset = heading_mean;
        calibration.apply_yaw_offset = true;
        ESP_LOGI(TAG_IMU, "Heading is stable (std_dev=%.4f), reference set: %.2f",
                 heading_std_dev, calibration.yaw_offset);
    }
    else
    {
        ESP_LOGW(TAG_IMU, "%s: Heading not stable (std_dev=%.4f > threshold=%.4f)", __func__, heading_std_dev, STD_DEV_THRESHOLD_HEADING);
        calibration.yaw_offset = local_euler.heading;
        calibration.apply_yaw_offset = true;
    }

    ESP_LOGI(TAG_IMU, "Reference point setting complete");
}

float apply_heading_offset(float raw_heading)
{
    if (!calibration.apply_yaw_offset)
    {
        return raw_heading;
    }
    float adjusted = raw_heading - calibration.yaw_offset;
    while (adjusted > 180.0f)
        adjusted -= 360.0f;
    while (adjusted < -180.0f)
        adjusted += 360.0f;

    return adjusted;
}

void reinit_sensor(void *pvParameters)
{
    esp_err_t err;
    vTaskDelay(pdMS_TO_TICKS(REINIT_TIME));

    while (1)
    {
        err = bno055_open(i2c_num, &bno_conf, BNO_MODE);
        if (err == ESP_OK)
        {
            if (tasks.blink_led != NULL)
            {
                vTaskDelete(tasks.blink_led);
                tasks.blink_led = NULL;
            }
            if (tasks.imu_plus != NULL)
            {
                vTaskResume(tasks.imu_plus);
            }
            vTaskDelete(NULL);
            break;
        }
        else
        {
            ESP_LOGE(TAG_IMU, "%s: Failed to open BNO055, retrying", __func__);
            vTaskDelay(pdMS_TO_TICKS(REINIT_TIME));
        }
    }
}

void calibration_task(void *pvParameters)
{
    esp_err_t err;
    calib_status_t calib_status;
    bno055_offsets_t offsets;
    bool was_calibrated = false;

    ESP_LOGI(TAG_IMU, "Calibration task started");
    while (!was_calibrated)
    {
        if (bno055_is_fully_calibrated(i2c_num, &calib_status, BNO_MODE))
        {
            was_calibrated = true;

            ESP_LOGI(TAG_IMU, "Calib - Sys: %d, Gyro: %d, Accel: %d, Mag: %d",
                     calib_status.sys, calib_status.gyro,
                     calib_status.accel, calib_status.mag);

            // Đọc giá trị offset
            err = bno055_get_offsets(i2c_num, &offsets);
            if (err == ESP_OK)
            {
                ESP_LOGI(TAG_IMU, "Accel offset: %d %d %d    Magnet: %d %d %d    Gyro: %d %d %d Acc_Radius: %d    Mag_Radius: %d",
                         offsets.accel_offset_x, offsets.accel_offset_y, offsets.accel_offset_z,
                         offsets.mag_offset_x, offsets.mag_offset_y, offsets.mag_offset_z,
                         offsets.gyro_offset_x, offsets.gyro_offset_y, offsets.gyro_offset_z,
                         offsets.accel_radius, offsets.mag_radius);
            }

            // Lưu dữ liệu hiệu chuẩn vào NVS
            err = nvs_save_bno055_calibration(&offsets);
            if (err == ESP_OK)
            {
                ESP_LOGI(TAG_IMU, "Calibration data saved successfully");
            }
            else
            {
                ESP_LOGE(TAG_IMU, "%s: Failed to save calibration data: 0x%X", __func__, err);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    bno055_set_reference();
    ESP_LOGI(TAG_IMU, "Calibration task complete");

    send_calibration_notification(calib_status);
    if (bno055_event_group != NULL)
    {
        xEventGroupSetBits(bno055_event_group, BNO055_CALIBRATED_BIT);
        ESP_LOGI(TAG_IMU, "BNO055 calibration complete bit set");
    }
    xTaskCreatePinnedToCore(imu_plus_task, "imu_plus_task", 4096, NULL, 23, &tasks.imu_plus, 1);
    tasks.calib = NULL;
    vTaskDelete(NULL);
}

void imu_plus_task(void *pvParameters)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    esp_err_t err;

    bno055_euler_t local_euler = {0.0f, 0.0f, 0.0f};
    float local_adjusted_heading = 0.0f;

    char message[128];

    while (1)
    {
        // Đọc euler data
        err = bno055_get_euler(i2c_num, &local_euler);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG_IMU, "%s: error: 0x%X", __func__, err);
            handle_sensor_error(i2c_num, err);
            taskYIELD();
            continue;
        }

        local_adjusted_heading = apply_heading_offset(local_euler.heading);

        if (bno055_sync.heading_mutex != NULL && xSemaphoreTake(bno055_sync.heading_mutex, pdMS_TO_TICKS(5)) == pdTRUE)
        {
            sensor_data.euler.heading = local_euler.heading;
            sensor_data.euler.roll = local_euler.roll;
            sensor_data.euler.pitch = local_euler.pitch;
            calibration.adjusted_heading = local_adjusted_heading;
            xSemaphoreGive(bno055_sync.heading_mutex);
        }

        local_adjusted_heading *= -1.0;
        int message_len = snprintf(message, sizeof(message),
                                   "{"
                                   "\"id\":\"%s\","
                                   "\"type\":\"bno055\","
                                   "\"data\":{"
                                   "\"euler\":[%.4f,%.4f,%.4f]"
                                   "}"
                                   "}\n",
                                   ID_ROBOT,
                                   local_adjusted_heading, local_euler.pitch, local_euler.roll);

        if (socket_send(message, message_len) != ESP_OK)
        {
            ESP_LOGE(TAG_IMU, "%s, Fail to send bno055 data", __func__);
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(BNO_POLLING_MS));
    }
}

void bno055_start()
{
    printf("\n\n\n");
    printf("********************\n");
    printf("  BNO055 IMU \n");
    printf("********************\n");
    esp_err_t err;

    bno055_sync.heading_mutex = xSemaphoreCreateMutex();

    if (bno055_event_group == NULL)
    {
        bno055_event_group = xEventGroupCreate();
        if (bno055_event_group == NULL)
        {
            ESP_LOGE(TAG_IMU, "%s: Failed to create event group", __func__);
        }
    }

    bno055_set_default_conf(&bno_conf);
    err = bno055_open(i2c_num, &bno_conf, BNO_MODE);
    ESP_LOGI(TAG_IMU, "bno055_open() returned 0x%02X", err);

    if (err != ESP_OK)
    {
        err = bno055_close(i2c_num);
        ESP_LOGW(TAG_IMU, "%s: Failed to open BNO055, starting reinit process", __func__);
        xTaskCreatePinnedToCore(reinit_sensor, "reinit_sensor", 4096, NULL, 5, NULL, 0);
    }
    else
    {
        xTaskCreatePinnedToCore(calibration_task, "calib_task", 4096, NULL, 5, &tasks.calib, 0);
    }
}

void waitBNO055Calibration()
{
    if (bno055_event_group != NULL)
    {
        ESP_LOGI("Waiting BNO055", "Waiting for BNO055 calibration to complete...");
        EventBits_t bits = xEventGroupWaitBits(
            bno055_event_group,    // Event group handle
            BNO055_CALIBRATED_BIT, // Bits to wait for
            pdFALSE,               // Don't clear bits on exit
            pdTRUE,                // Wait for all bits
            portMAX_DELAY);        // Wait indefinitely

        if (bits & BNO055_CALIBRATED_BIT)
        {
            ESP_LOGI("Waiting BNO055", "BNO055 calibration complete, starting forward kinematics");
        }
    }
}