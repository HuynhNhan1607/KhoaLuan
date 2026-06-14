#include "client_manager.h"
#include "arm_controller.h"
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/tcp.h>
#include <pthread.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
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

            /* ── Prevent blocking send() from stalling real-time loops ── */
            /* SO_SNDTIMEO: send() will timeout after 10ms instead of     */
            /*              blocking indefinitely (can stall docking loop) */
            struct timeval snd_tv = {.tv_sec = 0, .tv_usec = 10000}; /* 10 ms */
            setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &snd_tv, sizeof(snd_tv));

            /* TCP_NODELAY: disable Nagle algorithm — send small motor    */
            /*              commands immediately without coalescing delay  */
            int nodelay = 1;
            setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

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
            if (send(clients[i].sock, buffer, len, MSG_NOSIGNAL | MSG_DONTWAIT) < 0)
            {
                if (errno == EAGAIN || errno == EWOULDBLOCK)
                    continue; /* buffer full — skip, don't disconnect */
                printf("[CLIENT_MANAGER] Error sending to %s:%d - removing dead client\n",
                       clients[i].ip, clients[i].port);
                close(clients[i].sock);
                clients[i].sock = -1;
                clients[i].active = false;
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
                if (send(clients[i].sock, buffer, len, MSG_NOSIGNAL | MSG_DONTWAIT) < 0)
                {
                    if (errno == EAGAIN || errno == EWOULDBLOCK)
                        continue; /* buffer full — skip, next cycle sends updated cmd */
                    printf("[CLIENT_MANAGER] Error sending to motor %s:%d - removing dead client\n",
                           clients[i].ip, clients[i].port);
                    close(clients[i].sock);
                    clients[i].sock = -1;
                    clients[i].active = false;
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

void client_manager_broadcast_to_arm(const char *buffer, int len)
{
    // Only send to ESP32 ARM Controller (IP ending with .5)
    pthread_mutex_lock(&clients_mutex);
    for (int i = 0; i < MAX_CLIENTS; i++)
    {
        if (clients[i].active)
        {
            const char *last_octet = strrchr(clients[i].ip, '.');
            if (last_octet != NULL && strcmp(last_octet, ".5") == 0)
            {
                if (send(clients[i].sock, buffer, len, MSG_NOSIGNAL | MSG_DONTWAIT) < 0)
                {
                    if (errno == EAGAIN || errno == EWOULDBLOCK)
                        continue; /* buffer full — skip */
                    printf("[CLIENT_MANAGER] Error sending to arm %s:%d - removing dead client\n",
                           clients[i].ip, clients[i].port);
                    close(clients[i].sock);
                    clients[i].sock = -1;
                    clients[i].active = false;
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

void client_manager_print_rtt(void)
{
    pthread_mutex_lock(&clients_mutex);
    for (int i = 0; i < MAX_CLIENTS; i++)
    {
        if (clients[i].active && clients[i].sock != -1)
        {
            struct tcp_info info;
            socklen_t len = sizeof(info);
            if (getsockopt(clients[i].sock, IPPROTO_TCP, TCP_INFO, &info, &len) == 0)
            {
                double rtt = (double)info.tcpi_rtt / 1000.0;
                printf("[LATENCY] ESP32 client %s:%d | TCP RTT = %.3f ms\n",
                       clients[i].ip, clients[i].port, rtt);
            }
            else
            {
                printf("[LATENCY] ESP32 client %s:%d | Failed to get TCP RTT\n",
                       clients[i].ip, clients[i].port);
            }
        }
    }
    pthread_mutex_unlock(&clients_mutex);
}