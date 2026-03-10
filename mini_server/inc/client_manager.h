#ifndef CLIENT_MANAGER_H_
#define CLIENT_MANAGER_H_

void client_manager_init(void);
int client_manager_add(int sock, const char *ip, int port);
void client_manager_remove(int sock);
void client_manager_broadcast(const char *buffer, int len);
void client_manager_broadcast_to_motor(const char *buffer, int len);
void client_manager_broadcast_to_arm(const char *buffer, int len);
void client_manager_destroy(void);

#endif // CLIENT_MANAGER_H_