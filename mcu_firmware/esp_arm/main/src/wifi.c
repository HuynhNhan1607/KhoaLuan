#include "wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include <string.h>

static const char *TAG = "WIFI";

bool g_wifi_connected = false;
bool g_got_ip = false;

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
  {
    ESP_LOGI(TAG, "WiFi STA started, connecting...");
    esp_wifi_connect();
  }
  else if (event_base == WIFI_EVENT &&
           event_id == WIFI_EVENT_STA_DISCONNECTED)
  {
    g_wifi_connected = false;
    g_got_ip = false;
    ESP_LOGW(TAG, "WiFi disconnected, reconnecting...");
    esp_wifi_connect();
  }
  else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED)
  {
    g_wifi_connected = true;
    ESP_LOGI(TAG, "WiFi connected to AP, waiting for IP...");
  }
  else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
  {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    g_got_ip = true;
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    ESP_LOGI(TAG, "Gateway: " IPSTR, IP2STR(&event->ip_info.gw));
    ESP_LOGI(TAG, "Netmask: " IPSTR, IP2STR(&event->ip_info.netmask));
    ESP_LOGI(TAG, "========================================");
  }
}

void wifi_init_sta(void)
{
  esp_event_handler_instance_t instance_got_ip;
  nvs_flash_init();
  esp_netif_init();
  esp_event_loop_create_default();
  esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();

#if STATIC_IP
// Configure Static IP
#if ROBOT_ID == 1
  esp_netif_dhcpc_stop(sta_netif);
  esp_netif_ip_info_t ip_info;
  ip_info.ip.addr = ESP_IP4TOADDR(192, 168, 4, 5);
  ip_info.gw.addr = ESP_IP4TOADDR(192, 168, 4, 1);
  ip_info.netmask.addr = ESP_IP4TOADDR(255, 255, 255, 0);
  esp_netif_set_ip_info(sta_netif, &ip_info);
  ESP_LOGI(TAG, "Using Static IP: 192.168.4.5");
#elif ROBOT_ID == 2
  esp_netif_dhcpc_stop(sta_netif);
  esp_netif_ip_info_t ip_info;
  ip_info.ip.addr = ESP_IP4TOADDR(192, 168, 5, 5);
  ip_info.gw.addr = ESP_IP4TOADDR(192, 168, 5, 1);
  ip_info.netmask.addr = ESP_IP4TOADDR(255, 255, 255, 0);
  esp_netif_set_ip_info(sta_netif, &ip_info);
  ESP_LOGI(TAG, "Using Static IP: 192.168.5.5");
#endif
#else
  // Use DHCP (default behavior)
  ESP_LOGI(TAG, "Using DHCP, waiting for IP assignment...");
#endif

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);

  esp_event_handler_instance_t instance_any_id;
  esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                      &wifi_event_handler, NULL,
                                      &instance_any_id);
  esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                      &wifi_event_handler, NULL,
                                      &instance_got_ip);

  wifi_config_t wifi_config = {0};
  strcpy((char *)wifi_config.sta.ssid, WIFI_SSID);
  strcpy((char *)wifi_config.sta.password, WIFI_PASS);
  wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
  esp_wifi_start();

  ESP_LOGI(TAG,
           "WiFi init finished. Connecting to %s with static IP 192.168.4.5...",
           WIFI_SSID);

  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
  ESP_LOGI(TAG, "WiFi power save disabled");
}
