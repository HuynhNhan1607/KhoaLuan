#include "wifi_handler.h"
#include "sys_config.h"

#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "lwip/ip_addr.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define MAX_RETRY_COUNT 10
#define RETRY_DELAY_MS 1000

static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;

static int retry_count = 0;

static const char *TAG_Wifi = "WiFi_Connect";

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        ESP_LOGI(TAG_Wifi, "Wi-Fi started, attempting to connect...");
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
        retry_count++;
        if (retry_count < MAX_RETRY_COUNT)
        {
            ESP_LOGE(TAG_Wifi, "Disconnected. Retry %d... ", retry_count);
            esp_wifi_connect();
        }
        else
        {
            ESP_LOGE(TAG_Wifi, "Failed to connect after %d attempts | SSID: %s | PASS: %s", MAX_RETRY_COUNT, WIFI_SSID, WIFI_PASS);
            // esp_restart();
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG_Wifi, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
    }
}

void connect_to_wifi()
{
    ESP_LOGI(TAG_Wifi, "Initializing Wi-Fi...");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // Initialize network interface and event loop
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Create Wi-Fi station interface
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
#if STATIC_IP == 1
    esp_netif_dhcpc_stop(sta_netif);
    esp_netif_ip_info_t ip_info = {0};
    IP4_ADDR(&ip_info.ip, 192, 168, 5, 3);        // Static IP: 192.168.5.3
    IP4_ADDR(&ip_info.gw, 192, 168, 5, 1);        // Gateway: 192.168.5.1
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0); // Netmask
    ESP_ERROR_CHECK(esp_netif_set_ip_info(sta_netif, &ip_info));
#endif
    // Initialize Wi-Fi
    wifi_init_config_t wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_cfg));

    // Create event group to signal connection success
    wifi_event_group = xEventGroupCreate();

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    // Configure Wi-Fi settings
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE)); // Tắt tiết kiệm điện WiFi
    ESP_LOGI(TAG_Wifi, "WiFi Power Save: DISABLED");

    ESP_LOGI(TAG_Wifi, "Waiting for Wi-Fi connection...");
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
    ESP_LOGI(TAG_Wifi, "Wi-Fi connected successfully!");
}