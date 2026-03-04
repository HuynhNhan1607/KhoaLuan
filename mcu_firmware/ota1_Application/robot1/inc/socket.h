#ifndef SOCKET_H
#define SOCKET_H
#include "esp_err.h"

esp_err_t socket_send(char *msg, int msg_len);

int setup_socket(const char *server_ip, int server_port);
void task_socket(void *pvParameters);

void task_send_encoder(void *pvParameters);

esp_err_t server_init(void);
void server_tasks_start(void);
void send_pid_data(void);

#endif // SOCKET_H