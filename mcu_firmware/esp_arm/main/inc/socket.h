#ifndef SOCKET_H
#define SOCKET_H

#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <stdbool.h>
#include "sys_config.h"

#ifdef __cplusplus
extern "C"
{
#endif

// Server configuration
#if ROBOT_ID == 1
#define SERVER_IP "192.168.4.1"
#elif ROBOT_ID == 2
#define SERVER_IP "192.168.5.1"

#elif ROBOT_ID == 3
#define SERVER_IP "10.231.84.212"
#endif
#define SERVER_PORT 8080

// Command queue configuration
#define CMD_QUEUE_SIZE 20
#define CMD_MAX_LEN 512

  // Command structure for queue
  typedef struct
  {
    char json_str[CMD_MAX_LEN];
  } CmdQueueItem;

  // Global socket variables
  extern int g_server_sock;
  extern SemaphoreHandle_t g_sock_mutex;
  extern QueueHandle_t g_cmd_queue;

  // Socket functions
  void send_json(const cJSON *obj);
  void reply_error(const char *type, const char *msg);
  void socket_task(void *arg);
  void cmd_processor_task(void *arg);

#ifdef __cplusplus
}
#endif

#endif // SOCKET_H
