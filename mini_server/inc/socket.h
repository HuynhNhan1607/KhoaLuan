#ifndef SOCKET_H
#define SOCKET_H

#include <gpiod.h>
#include <stdbool.h>
#include <stddef.h>

#define SERVER_PORT 8080   // Port for ESP32 connections
#define FIRMWARE_PORT 8081 // Port for firmware updates
#define LAPTOP_PORT 2004   // Port for laptop connections
#define QUEUE_CAP 10       // Maximum pending connections

// IP settings
// #define ROOT_IP "192.168.1.1" // Example IP - adjust as needed
#define UPLINK_PORT 8080

#define ESP_EN 106

extern volatile bool g_running;

void metrics_bump(const char *p, size_t n);

// Thread functions
void *server_thread(void *arg); // Server for ESP32s
void *
laptop_server_thread(void *arg); // Server for laptop (previously client_thread)
void send_to_upstream_server(const char *buffer, int length);
void send_to_laptop_clients(const char *buffer, int length);
void client_manager_broadcast(const char *buffer, int length);
#endif // SOCKET_H