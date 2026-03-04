#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "servo_handler.h"
#include "socket.h"
#include "wifi.h"

static const char *TAG = "MAIN";

// LED pin for error indication (built-in LED on most ESP32 boards)
#define ERROR_LED_PIN GPIO_NUM_2

// Blink LED forever to indicate error (for debugging via USB)
static void error_blink_halt(const char *error_msg)
{
  ESP_LOGE(TAG, "FATAL ERROR: %s", error_msg);
  ESP_LOGE(TAG, "System halted. Connect USB to read logs.");

  // // Configure LED pin
  // gpio_config_t io_conf = {.pin_bit_mask = (1ULL << ERROR_LED_PIN),
  //                          .mode = GPIO_MODE_OUTPUT,
  //                          .pull_up_en = GPIO_PULLUP_DISABLE,
  //                          .pull_down_en = GPIO_PULLDOWN_DISABLE,
  //                          .intr_type = GPIO_INTR_DISABLE};
  // gpio_config(&io_conf);

  // // Blink forever
  // while (true) {
  //   gpio_set_level(ERROR_LED_PIN, 1);
  //   vTaskDelay(pdMS_TO_TICKS(200));
  //   gpio_set_level(ERROR_LED_PIN, 0);
  //   vTaskDelay(pdMS_TO_TICKS(200));
  // }
}

void app_main(void)
{
  ESP_LOGI(TAG, "=== APP START (TCP Client Mode) ===");

  // Step 1: Init WiFi
  ESP_LOGI(TAG, "Step 1: Init WiFi (Static IP: 192.168.4.5)");
  wifi_init_sta();

  // Wait for WiFi connection with timeout
  ESP_LOGI(TAG, "Waiting for WiFi connection...");
  int wait_count = 0;
  while (!g_wifi_connected && wait_count < 100)
  { // Max 10 seconds
    vTaskDelay(pdMS_TO_TICKS(100));
    wait_count++;
    if (wait_count % 10 == 0)
    {
      ESP_LOGI(TAG, "Still waiting for WiFi... (%d/10s)", wait_count / 10);
    }
  }

  if (!g_wifi_connected)
  {
    error_blink_halt("Failed to connect to WiFi after 10s");
  }

  ESP_LOGI(TAG, "WiFi connected!");

  // Step 2: Auto-init servo (50Hz)
  ESP_LOGI(TAG, "Step 2: Auto-init servo (50Hz)");
  if (servo_auto_init() != ESP_OK)
  {
    error_blink_halt("Servo auto-init failed");
  }

  ESP_LOGI(TAG, "=== Initialization Complete ===");
  ESP_LOGI(TAG, "Free heap: %lu bytes",
           (unsigned long)esp_get_free_heap_size());

  // Create socket mutex
  g_sock_mutex = xSemaphoreCreateMutex();
  if (!g_sock_mutex)
  {
    error_blink_halt("Failed to create socket mutex");
  }
  ESP_LOGI(TAG, "Socket mutex created");

  // Create servo control queue and task
  g_servo_queue = xQueueCreate(10, sizeof(ServoTarget));
  if (!g_servo_queue)
  {
    error_blink_halt("Failed to create servo queue");
  }

  // Create multi-servo queue for synchronized movement
  g_multi_servo_queue = xQueueCreate(5, sizeof(MultiServoTarget));
  if (!g_multi_servo_queue)
  {
    error_blink_halt("Failed to create multi-servo queue");
  }

  if (xTaskCreate(servo_control_task, "servo_ctrl", 4096, NULL, 5, NULL) !=
      pdPASS)
  {
    error_blink_halt("Failed to create servo task");
  }

  if (xTaskCreate(multi_servo_control_task, "multi_servo", 4096, NULL, 5,
                  NULL) != pdPASS)
  {
    error_blink_halt("Failed to create multi-servo task");
  }
  ESP_LOGI(TAG, "Servo control tasks created");

  // Create command queue for sequential processing
  g_cmd_queue = xQueueCreate(CMD_QUEUE_SIZE, sizeof(CmdQueueItem));
  if (!g_cmd_queue)
  {
    error_blink_halt("Failed to create command queue");
  }
  ESP_LOGI(TAG, "Command queue created");

  // Create command processor task (processes commands sequentially)
  if (xTaskCreate(cmd_processor_task, "cmd_proc", 8192, NULL, 5, NULL) !=
      pdPASS)
  {
    error_blink_halt("Failed to create command processor task");
  }
  ESP_LOGI(TAG, "Command processor task created");

  // Create socket task (handles TCP receive only)
  if (xTaskCreate(socket_task, "socket", 8192, NULL, 4, NULL) != pdPASS)
  {
    error_blink_halt("Failed to create socket task");
  }
  ESP_LOGI(TAG, "Socket task created");

  ESP_LOGI(TAG, "=== All tasks running. Main task idling ===");

  // Main task done - just idle forever
  while (true)
  {
    vTaskDelay(pdMS_TO_TICKS(60000)); // Sleep 1 minute
  }
}
