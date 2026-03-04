#include "socket.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "servo_handler.h"
#include <arpa/inet.h>
#include <errno.h>
#include <string.h>

static const char *TAG = "SOCKET";

// Global variables
int g_server_sock = -1;
SemaphoreHandle_t g_sock_mutex = NULL;
QueueHandle_t g_cmd_queue = NULL;

void send_json(const cJSON *obj)
{
  char *str = cJSON_PrintUnformatted(obj);
  if (str)
  {
    if (g_server_sock != -1)
    {
      char buffer[2048];
      snprintf(buffer, sizeof(buffer), "%s\n", str);
      send(g_server_sock, buffer, strlen(buffer), 0);
    }

    // ESP_LOGI(TAG, "[TX] %s", str);
    fflush(stdout);
    cJSON_free(str);
  }
}

void reply_error(const char *type, const char *msg)
{
  cJSON *root = cJSON_CreateObject();
  cJSON_AddBoolToObject(root, "ok", false);
  if (type)
    cJSON_AddStringToObject(root, "type", type);
  if (msg)
    cJSON_AddStringToObject(root, "error", msg);
  send_json(root);
  cJSON_Delete(root);
}

// Command processor task - processes commands sequentially from queue
void cmd_processor_task(void *arg)
{
  ESP_LOGI(TAG, "[CMD-PROCESSOR] Started");
  CmdQueueItem item;

  while (true)
  {
    if (xQueueReceive(g_cmd_queue, &item, portMAX_DELAY) == pdTRUE)
    {
      cJSON *root = cJSON_Parse(item.json_str);
      if (!root)
      {
        ESP_LOGW(TAG, "[CMD] Invalid JSON from queue: %s", item.json_str);
        reply_error(NULL, "invalid json");
        continue;
      }

      const cJSON *cmd = cJSON_GetObjectItemCaseSensitive(root, "cmd");
      if (!cJSON_IsString(cmd) || !cmd->valuestring)
      {
        ESP_LOGW(TAG, "[CMD] Missing cmd field: %s", item.json_str);
        cJSON_Delete(root);
        reply_error(NULL, "cmd missing");
        continue;
      }

      // ESP_LOGI(TAG, "[EXEC] Command: %s", cmd->valuestring);

      if (strcmp(cmd->valuestring, "servo") == 0)
      {
        handle_servo(root);
      }
      else if (strcmp(cmd->valuestring, "servo_us") == 0)
      {
        handle_servo_us(root);
      }
      else if (strcmp(cmd->valuestring, "home") == 0)
      {
        handle_home(root);
      }
      else if (strcmp(cmd->valuestring, "set_limits") == 0)
      {
        handle_set_limits(root);
      }
      else if (strcmp(cmd->valuestring, "servo_off") == 0)
      {
        handle_servo_off(root);
      }
      else if (strcmp(cmd->valuestring, "set_gravity") == 0)
      {
        handle_set_gravity(root);
      }
      else
      {
        reply_error(cmd->valuestring, "unknown cmd");
      }
      cJSON_Delete(root);
    }
  }
}

void socket_task(void *arg)
{
  ESP_LOGI(TAG, "[SOCKET-TASK] Started");

  while (true)
  {
    ESP_LOGI(TAG, "[SOCKET] Connecting to %s:%d...", SERVER_IP, SERVER_PORT);

    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (sock < 0)
    {
      ESP_LOGE(TAG, "[SOCKET] Failed to create socket: errno %d", errno);
      vTaskDelay(pdMS_TO_TICKS(5000));
      continue;
    }

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr);

    int err =
        connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr));
    if (err != 0)
    {
      ESP_LOGE(TAG, "[SOCKET] Connect failed: errno %d", errno);
      shutdown(sock, 0);
      close(sock);
      vTaskDelay(pdMS_TO_TICKS(5000));
      continue;
    }

    ESP_LOGI(TAG, "[SOCKET] Connected successfully!");

    if (xSemaphoreTake(g_sock_mutex, pdMS_TO_TICKS(1000)) == pdTRUE)
    {
      g_server_sock = sock;
      xSemaphoreGive(g_sock_mutex);
    }

    char line[CMD_MAX_LEN];
    int pos = 0;

    while (true)
    {
      char ch;
      int len = recv(sock, &ch, 1, 0);

      if (len < 0)
      {
        ESP_LOGE(TAG, "[SOCKET] recv failed: errno %d", errno);
        break;
      }
      else if (len == 0)
      {
        ESP_LOGI(TAG, "[SOCKET] Server disconnected");
        break;
      }

      if (ch == '\n' || ch == '\r')
      {
        if (pos <= 0)
          continue;
        line[pos] = 0;
        pos = 0;

        size_t free_heap = esp_get_free_heap_size();
        if (free_heap < 8192)
        {
          reply_error(NULL, "low memory");
          ESP_LOGW(TAG, "[SOCKET] Low heap: %u bytes", (unsigned)free_heap);
          continue;
        }

        // ESP_LOGI(TAG, "[RX] %s", line);

        // Queue the command for sequential processing
        CmdQueueItem item;
        strncpy(item.json_str, line, CMD_MAX_LEN - 1);
        item.json_str[CMD_MAX_LEN - 1] = '\0';

        if (xQueueSend(g_cmd_queue, &item, pdMS_TO_TICKS(100)) != pdTRUE)
        {
          ESP_LOGW(TAG, "[SOCKET] Command queue full, dropping command");
          reply_error(NULL, "queue full");
        }
      }
      else if (pos < (int)sizeof(line) - 1)
      {
        line[pos++] = ch;
      }
      else
      {
        pos = 0;
        reply_error(NULL, "line too long");
      }
    }

    if (xSemaphoreTake(g_sock_mutex, pdMS_TO_TICKS(1000)) == pdTRUE)
    {
      g_server_sock = -1;
      xSemaphoreGive(g_sock_mutex);
    }

    shutdown(sock, 0);
    close(sock);
    ESP_LOGI(TAG, "[SOCKET] Connection closed. Reconnecting in 5s...");
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}
