#include <stdio.h>
#include <string.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "bno055.h"
#include "nvs_handler.h"

static const char *TAG = "nvs_handle";
static const char *NVS_NAMESPACE = "bno055";
static const char *CALIB_KEY = "calib_data";

// Hàm này cần được thêm vào bno055.c và bno055.h
extern esp_err_t bno055_set_offsets(i2c_number_t i2c_num, bno055_offsets_t *offsets);

esp_err_t nvs_init(void)
{
    esp_err_t err = nvs_flash_init();

    // Nếu NVS partition đang bị đầy hoặc cần format
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // Xóa và khởi tạo lại NVS partition
        ESP_LOGI(TAG, "Erasing NVS partition...");
        err = nvs_flash_erase();
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to erase NVS partition: %s", esp_err_to_name(err));
            return err;
        }

        err = nvs_flash_init();
    }

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize NVS: %s", esp_err_to_name(err));
    }
    else
    {
        ESP_LOGI(TAG, "NVS initialized successfully");
    }

    return err;
}

esp_err_t nvs_save_bno055_calibration(bno055_offsets_t *offsets)
{
    if (offsets == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs_handle;
    esp_err_t err;

    // Mở NVS handle
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_blob(nvs_handle, CALIB_KEY, offsets, sizeof(bno055_offsets_t));
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error saving calibration data: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error committing to NVS: %s", esp_err_to_name(err));
    }
    else
    {
        ESP_LOGI(TAG, "Calibration data saved successfully");
    }
    nvs_close(nvs_handle);
    return err;
}

esp_err_t nvs_load_bno055_calibration(bno055_offsets_t *offsets)
{
    if (offsets == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs_handle;
    esp_err_t err;

    // Mở NVS handle
    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
        return err;
    }

    size_t required_size = sizeof(bno055_offsets_t);
    err = nvs_get_blob(nvs_handle, CALIB_KEY, NULL, &required_size);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGE(TAG, "Error querying calibration data size: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGI(TAG, "No calibration data found in NVS");
        nvs_close(nvs_handle);
        return ESP_ERR_NVS_NOT_FOUND;
    }

    // Kiểm tra kích thước dữ liệu
    if (required_size != sizeof(bno055_offsets_t))
    {
        ESP_LOGE(TAG, "Calibration data size mismatch");
        nvs_close(nvs_handle);
        return ESP_ERR_INVALID_SIZE;
    }

    // Đọc dữ liệu từ NVS
    err = nvs_get_blob(nvs_handle, CALIB_KEY, offsets, &required_size);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading calibration data: %s", esp_err_to_name(err));
    }
    else
    {
        ESP_LOGI(TAG, "Calibration data loaded successfully");
    }

    // Đóng NVS handle
    nvs_close(nvs_handle);
    return err;
}

bool nvs_has_bno055_calibration(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;

    // Mở NVS handle
    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK)
    {
        return false;
    }

    size_t required_size = 0;
    err = nvs_get_blob(nvs_handle, CALIB_KEY, NULL, &required_size);
    nvs_close(nvs_handle);
    return (err == ESP_OK && required_size == sizeof(bno055_offsets_t));
}

esp_err_t nvs_apply_bno055_calibration(i2c_number_t i2c_num)
{
    bno055_offsets_t offsets;
    esp_err_t err;

    // Đọc dữ liệu từ NVS
    err = nvs_load_bno055_calibration(&offsets);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "nvs_apply_bno055_calibration(): %s", esp_err_to_name(err));
        return err;
    }

    err = bno055_set_offsets(i2c_num, &offsets);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "nvs_apply_bno055_calibration(): %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "Calibration data applied to sensor");
    return ESP_OK;
}

esp_err_t nvs_clear_bno055_calibration(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;

    // Mở NVS handle
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
        return err;
    }
    err = nvs_erase_key(nvs_handle, CALIB_KEY);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGE(TAG, "Error erasing calibration data: %s", esp_err_to_name(err));
    }
    else
    {
        err = ESP_OK; // Nếu key không tồn tại, cũng coi như thành công
        esp_err_t commit_err = nvs_commit(nvs_handle);
        if (commit_err != ESP_OK)
        {
            ESP_LOGE(TAG, "Error committing to NVS: %s", esp_err_to_name(commit_err));
            err = commit_err;
        }
        else
        {
            ESP_LOGI(TAG, "Calibration data cleared successfully");
        }
    }
    nvs_close(nvs_handle);
    return err;
}