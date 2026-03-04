#include "client_manager.h"
#include "arm_controller.h"
#include <arpa/inet.h>
#include <pthread.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define MAX_CLIENTS 10

// Client information structure
typedef struct
{
    int sock;
    char ip[INET_ADDRSTRLEN];
    int port;
    bool active;
} client_t;

// Global variables
static client_t clients[MAX_CLIENTS];
static pthread_mutex_t clients_mutex = PTHREAD_MUTEX_INITIALIZER;

void client_manager_init(void)
{
    pthread_mutex_lock(&clients_mutex);
    for (int i = 0; i < MAX_CLIENTS; i++)
    {
        clients[i].sock = -1;
        clients[i].active = false;
    }
    pthread_mutex_unlock(&clients_mutex);
    printf("[CLIENT_MANAGER] Init: ready to manage %d clients\n", MAX_CLIENTS);
}

int client_manager_add(int sock, const char *ip, int port)
{
    int idx = -1;
    pthread_mutex_lock(&clients_mutex);
    for (int i = 0; i < MAX_CLIENTS; i++)
    {
        if (!clients[i].active)
        {
            clients[i].sock = sock;
            strncpy(clients[i].ip, ip, INET_ADDRSTRLEN);
            clients[i].port = port;
            clients[i].active = true;
            idx = i;
            printf("[CLIENT_MANAGER] Added: %s:%d at slot %d\n", ip, port, i);

            // Check if this is ESP32 ARM controller (IP ending with .5)
            const char *last_octet = strrchr(ip, '.');
            if (last_octet != NULL && strcmp(last_octet, ".5") == 0)
            {
                printf("[CLIENT_MANAGER] ESP32 ARM detected at %s - sending REST command\n", ip);
                pthread_mutex_unlock(&clients_mutex);

                // Send arm rest command
                arm_rest();

                pthread_mutex_lock(&clients_mutex);
            }

            break;
        }
    }
    pthread_mutex_unlock(&clients_mutex);
    return idx;
}

void client_manager_remove(int sock)
{
    pthread_mutex_lock(&clients_mutex);
    for (int i = 0; i < MAX_CLIENTS; i++)
    {
        if (clients[i].active && clients[i].sock == sock)
        {
            printf("[CLIENT_MANAGER] Removed: %s:%d from slot %d\n",
                   clients[i].ip, clients[i].port, i);
            clients[i].active = false;
            clients[i].sock = -1;
            break;
        }
    }
    pthread_mutex_unlock(&clients_mutex);
}

void client_manager_broadcast(const char *buffer, int len)
{
    pthread_mutex_lock(&clients_mutex);
    int active_count = 0;
    for (int i = 0; i < MAX_CLIENTS; i++)
    {
        if (clients[i].active)
        {
            active_count++;
            if (send(clients[i].sock, buffer, len, 0) < 0)
            {
                printf("[CLIENT_MANAGER] Error sending to %s:%d\n",
                       clients[i].ip, clients[i].port);
            }
        }
    }
    if (active_count > 0)
    {
        // printf("[CLIENT_MANAGER] Broadcast: sent to %d clients\n", active_count);
    }
    pthread_mutex_unlock(&clients_mutex);
}

void client_manager_broadcast_to_motor(const char *buffer, int len)
{
    // Only send to ESP32 Motor Controller (IP ending with .3)
    pthread_mutex_lock(&clients_mutex);
    int sent_count = 0;
    for (int i = 0; i < MAX_CLIENTS; i++)
    {
        if (clients[i].active)
        {
            // Check if IP ends with .3 (Motor ESP32)
            const char *last_octet = strrchr(clients[i].ip, '.');
            if (last_octet != NULL && strcmp(last_octet, ".3") == 0)
            {
                if (send(clients[i].sock, buffer, len, 0) < 0)
                {
                    printf("[CLIENT_MANAGER] Error sending to motor %s:%d\n",
                           clients[i].ip, clients[i].port);
                }
                else
                {
                    sent_count++;
                }
            }
        }
    }
    pthread_mutex_unlock(&clients_mutex);
}

void client_manager_destroy(void)
{
    pthread_mutex_lock(&clients_mutex);
    for (int i = 0; i < MAX_CLIENTS; i++)
    {
        if (clients[i].active)
        {
            printf("[CLIENT_MANAGER] Closing: %s:%d\n",
                   clients[i].ip, clients[i].port);
            close(clients[i].sock);
            clients[i].active = false;
        }
    }
    pthread_mutex_unlock(&clients_mutex);
    printf("[CLIENT_MANAGER] All clients disconnected\n");
}