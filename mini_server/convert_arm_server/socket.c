#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include "client_manager.h"
#include "json_handler.h"
#include "localize.h"
#include "optical_flow.h"
#include "socket.h"
#include "sys_config.h"


extern struct gpiod_line *line;

volatile bool g_running = true;
static int g_upgrade_laptop_fd = -1;

/*Firmware Thread*/
static volatile bool g_firmware_update_in_progress = false;
static pthread_mutex_t g_firmware_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t g_firmware_cond = PTHREAD_COND_INITIALIZER;

static int send_and_wait_ack(int sock,
                             const char *send_msg, // message cần gửi
                             const char *ack_msg,  // ACK kỳ vọng
                             int max_retries, int timeout_ms) {
  for (int attempt = 1; attempt <= max_retries; ++attempt) {
    // gửi message
    if (send(sock, send_msg, strlen(send_msg), 0) < 0) {
      perror("[FIRMWARE] send() failed");
      return -1;
    }
    printf("[FIRMWARE] Sent '%s' (attempt %d/%d)\n", send_msg, attempt,
           max_retries);

    struct pollfd pfd = {.fd = sock, .events = POLLIN};
    int pr = poll(&pfd, 1, timeout_ms);

    if (pr > 0 && (pfd.revents & POLLIN)) {
      char buf[128];
      ssize_t n = recv(sock, buf, sizeof(buf) - 1, 0);
      if (n > 0) {
        buf[n] = '\0';
        if (strstr(buf, ack_msg)) {
          printf("[FIRMWARE] Got ACK '%s'\n", ack_msg);
          return 0; // thành công
        }
      } else {
        perror("[FIRMWARE] recv() error while waiting ACK");
        return -1;
      }
    } else if (pr == 0) {
      printf("[FIRMWARE] No ACK, retrying...\n");
    } else {
      perror("[FIRMWARE] poll() error");
      return -1;
    }
  }
  return -1; // hết retry vẫn fail
}

void *firmware_update_thread(void *arg) {
  (void)arg;
  int server_fd;
  struct sockaddr_in address;
  int firmware_client_fd = -1;
  char buffer[1024];
  bool client_connected = false;
  int total_byte = 0;

  // Set flag that firmware update is in progress
  pthread_mutex_lock(&g_firmware_mutex);
  g_firmware_update_in_progress = true;
  pthread_mutex_unlock(&g_firmware_mutex);

  printf("[FIRMWARE] Starting firmware update thread, listening on port %d\n",
         FIRMWARE_PORT);

  // Create firmware update server socket
  if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
    perror("[FIRMWARE] Socket creation failed");
    goto cleanup;
  }
  int opt = 1;
  if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
    perror("[FIRMWARE] setsockopt failed");
    close(server_fd);
    goto cleanup;
  }
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(FIRMWARE_PORT);

  if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
    perror("[FIRMWARE] bind failed");
    close(server_fd);
    goto cleanup;
  }

  if (listen(server_fd, 1) < 0) {
    perror("[FIRMWARE] listen failed");
    close(server_fd);
    goto cleanup;
  }

  printf("[FIRMWARE] Waiting for client to connect on port %d\n",
         FIRMWARE_PORT);

  while (g_running && g_firmware_update_in_progress && !client_connected) {
    struct sockaddr_in client_addr;
    socklen_t client_addrlen = sizeof(client_addr);

    firmware_client_fd =
        accept(server_fd, (struct sockaddr *)&client_addr, &client_addrlen);

    if (firmware_client_fd < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        usleep(100000); // 100ms
        continue;
      } else {
        perror("[FIRMWARE] accept failed");
        close(server_fd);
        goto cleanup;
      }
    }

    // Client connected
    char client_ip[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, sizeof(client_ip));
    int client_port = ntohs(client_addr.sin_port);

    printf("[FIRMWARE] Client connected from %s:%d\n", client_ip, client_port);
    client_connected = true;

    // Gửi ready cho firmware client
    if (g_upgrade_laptop_fd != -1) {
      if (send_and_wait_ack(g_upgrade_laptop_fd, "ready\n", "okay", 5, 2000) !=
          0) {
        printf("[FIRMWARE] No 'okay' from laptop fd=%d\n", g_upgrade_laptop_fd);
      } else {
        printf("[FIRMWARE] Sent 'ready' to firmware client\n");
      }
    } else {
      printf("[FIRMWARE] Laptop not connect for transfer\n");
    }
  }

  if (client_connected) {
    printf("[FIRMWARE] Starting firmware transfer process\n");
    while (g_running && g_firmware_update_in_progress) {
      ssize_t bytes_read = recv(g_upgrade_laptop_fd, buffer, sizeof(buffer), 0);
      if (bytes_read > 0) {
        total_byte += bytes_read;
        // Check if this is the end of firmware update
        if (strstr(buffer, "COMPLETED") != NULL) {
          printf("[FIRMWARE] Firmware update completed - Received: %d\n",
                 total_byte - 9);

          shutdown(firmware_client_fd, SHUT_RDWR);
          close(firmware_client_fd);
          firmware_client_fd = -1;

          pthread_mutex_lock(&g_firmware_mutex);
          g_firmware_update_in_progress = false;
          pthread_cond_signal(&g_firmware_cond);
          pthread_mutex_unlock(&g_firmware_mutex);
          break;
        }
        if (send(firmware_client_fd, buffer, bytes_read, 0) < 0) {
          perror("[FIRMWARE] Failed to send data to firmware client");
          break;
        }
      } else if (bytes_read == 0) {
        printf("[FIRMWARE] Client closed connection, ending firmware update\n");
        break;
      } else if (errno == EAGAIN || errno == EWOULDBLOCK) {
        usleep(50000); // 50ms
        continue;
      } else {
        perror("[FIRMWARE] recv error");
        break;
      }
    }
    // printf("g_running: %d -- g_firmware_update_in_progress: %d", g_running,
    // g_firmware_update_in_progress);
  }

  // Close client connection
  if (firmware_client_fd != -1) {
    printf("[FIRMWARE] Closing client connection\n");
    close(firmware_client_fd);
  }

  // Close server socket
  printf("[FIRMWARE] Closing firmware update port\n");
  close(server_fd);

cleanup:
  // Clear firmware update flag
  pthread_mutex_lock(&g_firmware_mutex);
  g_firmware_update_in_progress = false;
  pthread_mutex_unlock(&g_firmware_mutex);
  printf("[FIRMWARE] Firmware update thread exiting\n");
  return NULL;
}

// Thay đổi g_client_socket thành mảng quản lý các laptop clients
#define MAX_LAPTOP_CLIENTS 5
static int g_laptop_sockets[MAX_LAPTOP_CLIENTS] = {-1, -1, -1, -1, -1};
static pthread_mutex_t g_laptop_sockets_mutex = PTHREAD_MUTEX_INITIALIZER;

// Hàm gửi data đến tất cả laptop clients đang kết nối
void send_to_laptop_clients(const char *buffer, int length) {
  pthread_mutex_lock(&g_laptop_sockets_mutex);
  for (int i = 0; i < MAX_LAPTOP_CLIENTS; i++) {
    if (g_laptop_sockets[i] != -1) {
      if (send(g_laptop_sockets[i], buffer, length, 0) < 0) {
        perror("[LAPTOP] Failed to send data to laptop client");
        // Close broken connection
        close(g_laptop_sockets[i]);
        g_laptop_sockets[i] = -1;
      }
    }
  }
  // printf("Send: %s to LAPTOP\n", buffer);
  pthread_mutex_unlock(&g_laptop_sockets_mutex);
}

// Thay thế hàm send_to_upstream_server cũ
void send_to_upstream_server(const char *buffer, int length) {
  // Forward to all connected laptop clients
  send_to_laptop_clients(buffer, length);
}

// Thay đổi client_thread thành laptop_server_thread
void *laptop_server_thread(void *arg) {
  (void)arg; // Fix unused parameter warning
  int server_fd;
  struct sockaddr_in address;
  fd_set master_set, working_set;
  int max_fd;

  // Initialize all laptop client sockets to -1 (unused)
  pthread_mutex_lock(&g_laptop_sockets_mutex);
  for (int i = 0; i < MAX_LAPTOP_CLIENTS; i++) {
    g_laptop_sockets[i] = -1;
  }
  pthread_mutex_unlock(&g_laptop_sockets_mutex);

  // Create socket
  if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
    perror("[LAPTOP] Socket creation failed");
    return NULL;
  }

  // Set socket options to reuse address
  int opt = 1;
  if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
    perror("[LAPTOP] setsockopt failed");
    close(server_fd);
    return NULL;
  }

  // Bind socket to port
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY; // Listening on all interfaces
  address.sin_port = htons(LAPTOP_PORT);

  if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
    perror("[LAPTOP] bind failed");
    close(server_fd);
    return NULL;
  }

  if (listen(server_fd, QUEUE_CAP) < 0) {
    perror("[LAPTOP] listen failed");
    close(server_fd);
    return NULL;
  }

  printf(
      "[LAPTOP] Server started, listening for laptop connections on port %d\n",
      LAPTOP_PORT);

  // Initialize the set of active sockets
  FD_ZERO(&master_set);
  FD_SET(server_fd, &master_set);
  max_fd = server_fd;

  // Set non-blocking mode for server socket
  // int flags = fcntl(server_fd, F_GETFL, 0);
  // fcntl(server_fd, F_SETFL, flags | O_NONBLOCK);

  while (g_running) {
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000; // 100ms timeout

    // Copy the master set to the working set
    working_set = master_set;

    // Wait for activity on any socket
    int activity = select(max_fd + 1, &working_set, NULL, NULL, &timeout);

    if (activity < 0 && errno != EINTR) {
      perror("[LAPTOP] select error");
      break;
    }

    for (int i = 0; i <= max_fd; i++) {
      if (FD_ISSET(i, &working_set)) {
        if (i == server_fd) {
          // Accept new connection
          struct sockaddr_in client_addr;
          socklen_t client_addrlen = sizeof(client_addr);
          int new_socket = accept(server_fd, (struct sockaddr *)&client_addr,
                                  &client_addrlen);
          if (new_socket >= 0) {
            pthread_mutex_lock(&g_laptop_sockets_mutex);
            int slot = -1;
            for (int j = 0; j < MAX_LAPTOP_CLIENTS; j++) {
              if (g_laptop_sockets[j] == -1) {
                slot = j;
                break;
              }
            }
            if (slot == -1) {
              // Đã đầy, từ chối kết nối
              char client_ip[INET_ADDRSTRLEN];
              inet_ntop(AF_INET, &client_addr.sin_addr, client_ip,
                        sizeof(client_ip));
              int client_port = ntohs(client_addr.sin_port);
              printf("[LAPTOP] Rejecting connection from %s:%d (max clients "
                     "reached)\n",
                     client_ip, client_port);
              close(new_socket);
            } else {
              g_laptop_sockets[slot] = new_socket;
              FD_SET(new_socket, &master_set);
              if (new_socket > max_fd)
                max_fd = new_socket;

              char client_ip[INET_ADDRSTRLEN];
              inet_ntop(AF_INET, &client_addr.sin_addr, client_ip,
                        sizeof(client_ip));
              int client_port = ntohs(client_addr.sin_port);
              printf("[LAPTOP] New client connected: fd=%d, IP=%s, PORT=%d\n",
                     new_socket, client_ip, client_port);
            }
            pthread_mutex_unlock(&g_laptop_sockets_mutex);
          }
        } else {
          pthread_mutex_lock(&g_firmware_mutex);
          bool updating = g_firmware_update_in_progress;
          if (updating) {
            printf("[CLIENT] Firmware update in progress, pausing client "
                   "thread\n");
            pthread_cond_wait(&g_firmware_cond, &g_firmware_mutex);
            printf(
                "[CLIENT] Firmware update completed, resuming client thread\n");
          }
          pthread_mutex_unlock(&g_firmware_mutex);
          // Receive data from client
          char buffer[1024];
          ssize_t valread = recv(i, buffer, sizeof(buffer), 0);
          if (valread <= 0) {
            // Client disconnected
            close(i);
            FD_CLR(i, &master_set);
            pthread_mutex_lock(&g_laptop_sockets_mutex);
            for (int j = 0; j < MAX_LAPTOP_CLIENTS; j++) {
              if (g_laptop_sockets[j] == i) {
                g_laptop_sockets[j] = -1;
                break;
              }
            }
            pthread_mutex_unlock(&g_laptop_sockets_mutex);
            printf("[LAPTOP] Client disconnected: fd=%d\n", i);
          } else {
            buffer[valread] = '\0';
            printf("[LAPTOP] Received: %s\n", buffer);

            // Logic giống client_thread
            if (strstr(buffer, "Upgrade")) {
              pthread_mutex_lock(&g_firmware_mutex);
              bool already_updating = g_firmware_update_in_progress;
              pthread_mutex_unlock(&g_firmware_mutex);
              // client_manager_broadcast("Upgrade", strlen("Upgrade"));
              if (!already_updating) {
                g_upgrade_laptop_fd = i;
                printf("[LAPTOP] Firmware update request from %d, starting "
                       "update process\n",
                       g_upgrade_laptop_fd);
                pthread_t th_firmware;
                if (pthread_create(&th_firmware, NULL, firmware_update_thread,
                                   NULL) != 0) {
                  perror("[LAPTOP] Failed to create firmware update thread");
                } else {
                  pthread_detach(th_firmware);
                }
              } else {
                printf("[LAPTOP] Firmware update already in progress\n");
              }
            } else if (strstr(buffer, "reset")) {
              gpiod_line_set_value(line, 0);
              usleep(100);
              gpiod_line_set_value(line, 1);
              printf("[LAPTOP] ESP32 reseted\n");
            }

            // Filter: Don't broadcast arm control messages to ESP32
            // These are handled by json_handler which creates proper servo
            // commands
            if (strstr(buffer, "arm_ik_request") ||
                strstr(buffer, "arm_servo_cmd")) {
              printf(
                  "[LAPTOP] Arm message detected, routing to json_handler\n");
              json_handler_add_message(buffer, valread);
            } else {
              // Broadcast other messages to ESP32
              client_manager_broadcast(buffer, valread);
            }
          }
        }
      }
    }
  }

  printf("[LAPTOP] Laptop server shutting down\n");

  // Close all client connections
  pthread_mutex_lock(&g_laptop_sockets_mutex);
  for (int i = 0; i < MAX_LAPTOP_CLIENTS; i++) {
    if (g_laptop_sockets[i] != -1) {
      close(g_laptop_sockets[i]);
      g_laptop_sockets[i] = -1;
    }
  }
  pthread_mutex_unlock(&g_laptop_sockets_mutex);

  close(server_fd);
  return NULL;
}
#if Cal_Freq == 1
#include <stdatomic.h>
#include <stdint.h>

static _Atomic uint32_t enc_cnt = 0, bno_cnt = 0, log_cnt = 0;
static _Atomic uint32_t odo_cnt = 0, loc_cnt = 0, ekf_cnt = 0, bno_pos_cnt = 0,
                        optical_flow_cnt = 0;
void metrics_bump(const char *p, size_t n) {
#define HAS(s) (memmem(p, n, s, strlen(s)) != NULL)
  if (HAS("\"type\":\"encoder\""))
    atomic_fetch_add(&enc_cnt, 1);
  else if (HAS("\"type\":\"bno055\""))
    atomic_fetch_add(&bno_cnt, 1);
  else if (HAS("\"type\":\"log\""))
    atomic_fetch_add(&log_cnt, 1);
  else if (HAS("\"type\":\"position\"") && HAS("\"source\":\"odometry\""))
    atomic_fetch_add(&odo_cnt, 1);
  else if (HAS("\"type\":\"position\"") && HAS("\"source\":\"localization\""))
    atomic_fetch_add(&loc_cnt, 1);
  else if (HAS("\"type\":\"position\"") && HAS("\"source\":\"ekf\""))
    atomic_fetch_add(&ekf_cnt, 1);
  else if (HAS("\"type\":\"position\"") && HAS("\"source\":\"bno055\""))
    atomic_fetch_add(&bno_pos_cnt, 1);
  else if (HAS("\"type\":\"position\"") && HAS("\"source\":\"optical_flow\""))
    atomic_fetch_add(&optical_flow_cnt, 1);
#undef HAS
}

void *metrics_thread(void *arg) {
  (void)arg;
  struct timespec last, now;
  clock_gettime(CLOCK_MONOTONIC, &last);

  for (;;) {
    usleep(200000); // 200ms, đơn giản
    clock_gettime(CLOCK_MONOTONIC, &now);
    double dt = (now.tv_sec - last.tv_sec) + (now.tv_nsec - last.tv_nsec) / 1e9;

    if (dt < 0.95)
      continue; // chưa tới ~1s thì chờ thêm

    // lấy & reset
    uint64_t enc = atomic_exchange(&enc_cnt, 0);
    uint64_t bno = atomic_exchange(&bno_cnt, 0);
    uint64_t lg = atomic_exchange(&log_cnt, 0);
    uint64_t odo = atomic_exchange(&odo_cnt, 0);
    uint64_t loc = atomic_exchange(&loc_cnt, 0);
    uint64_t ekf = atomic_exchange(&ekf_cnt, 0);
    uint64_t bpos = atomic_exchange(&bno_pos_cnt, 0);
    uint64_t optical = atomic_exchange(&optical_flow_cnt, 0);

    printf("[FREQ] dt: %.6f | Enc: %.2f Hz | BNO055: %.2f Hz | Log: %.2f Hz | "
           "Pos-Odo: %.2f Hz | Pos-Loc: %.2f Hz | Pos-EKF: %.2f Hz | "
           "Pos-BNO055: %.2f Hz | Pos-OptFlow: %.2f Hz\n",
           dt, enc / dt, bno / dt, lg / dt, odo / dt, loc / dt, ekf / dt,
           bpos / dt, optical / dt);

    last = now;
  }
  return NULL;
}

#endif

typedef struct {
  char *p;
  size_t n, c;
} RB;

int pump_jsonl(int fd) {
  typedef struct {
    char *p;
    size_t n, c, h;
  } RB; // n: used, c: cap, h: head (offset đã xử lý)
  static RB b[FD_SETSIZE];

  if (fd < 0 || fd >= FD_SETSIZE)
    return -1;

  for (;;) {
    char t[8192];
    ssize_t r = recv(fd, t, sizeof t, 0);
    if (r > 0) {
      // đảm bảo sức chứa
      if (b[fd].n + (size_t)r > b[fd].c) {
        size_t nc = b[fd].c ? b[fd].c : 4096;
        while (nc < b[fd].n + (size_t)r)
          nc <<= 1;
        char *q = (char *)realloc(b[fd].p, nc);
        if (!q)
          return -1;
        b[fd].p = q;
        b[fd].c = nc;
      }
      // append mẻ mới
      memcpy(b[fd].p + b[fd].n, t, (size_t)r);
      b[fd].n += (size_t)r;

      // quét từ vị trí head (b[fd].h), KHÔNG memmove từng dòng
      size_t start = b[fd].h;
      for (;;) {
        if (start >= b[fd].n)
          break;
        void *nl = memchr(b[fd].p + start, '\n', b[fd].n - start);
        if (!nl)
          break;
        size_t line_end = (size_t)((char *)nl - b[fd].p); // vị trí '\n'
        size_t L = line_end - start;                      // không gồm '\n'
        // trim CR/space cuối dòng
        while (L && (b[fd].p[start + L - 1] == '\r' ||
                     b[fd].p[start + L - 1] == ' ' ||
                     b[fd].p[start + L - 1] == '\t'))
          --L;

        if (L) {
          size_t raw_len = (line_end + 1) - start;
          send_to_upstream_server(b[fd].p + start, (int)raw_len);

          json_handler_add_message(b[fd].p + start, (int)L);
        }
        // nhảy sang đầu dòng kế tiếp, KHÔNG memmove
        start = line_end + 1;
      }

      // cập nhật head (đã xử lý đến 'start')
      if (start > b[fd].h)
        b[fd].h = start;

      // gọn bộ đệm: chỉ memmove MỘT LẦN khi cần
      if (b[fd].h == b[fd].n) {
        // đã ăn hết → reset nhanh
        b[fd].n = b[fd].h = 0;
      } else if (b[fd].h > (1u << 15)) {
        // head lớn quá → dồn phần còn lại về đầu 1 lần
        size_t rem = b[fd].n - b[fd].h;
        memmove(b[fd].p, b[fd].p + b[fd].h, rem);
        b[fd].n = rem;
        b[fd].h = 0;
      }

      continue; // drain đến EAGAIN
    }

    if (r == 0) {
      free(b[fd].p);
      b[fd].p = NULL;
      b[fd].n = b[fd].c = b[fd].h = 0;
      return 0; // peer đóng
    }

    if (errno == EAGAIN || errno == EWOULDBLOCK)
      return 1; // hết dữ liệu hiện có
    return -1;  // lỗi khác
  }
}

bool client_connected = false;

// Server function to accept connections on SERVER_PORT
void *server_thread(void *arg) {
  (void)arg; // Fix unused parameter warning
  int server_fd;
  struct sockaddr_in address;
  fd_set master_set, working_set;
  int max_fd;

  client_manager_init();
  if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
    perror("[SERVER] Socket creation failed");
    return NULL;
  }
  // Set socket options to reuse address
  int opt = 1;
  if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
    perror("[SERVER] setsockopt failed");
    close(server_fd);
    return NULL;
  }

  // Bind socket to port
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(SERVER_PORT);

  if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
    perror("[SERVER] bind failed");
    close(server_fd);
    return NULL;
  }

  if (listen(server_fd, QUEUE_CAP) < 0) {
    perror("[SERVER] listen failed");
    close(server_fd);
    return NULL;
  }

  printf("[SERVER] Server started, listening on port %d\n", SERVER_PORT);

  if (!json_handler_init()) {
    fprintf(stderr, "[SERVER] Failed to initialize JSON handler\n");
    close(server_fd);
    return NULL;
  }

#if Cal_Freq == 1
  pthread_t t;
  pthread_create(&t, NULL, metrics_thread, NULL);
  pthread_detach(t);
#endif
  // Initialize the set of active sockets
  FD_ZERO(&master_set);
  FD_SET(server_fd, &master_set);
  max_fd = server_fd;

  // Set non-blocking mode for server socket
  int flags = fcntl(server_fd, F_GETFL, 0);
  fcntl(server_fd, F_SETFL, flags | O_NONBLOCK);

  while (g_running) {
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 50000; // 500ms timeout

    // Copy the master set to the working set
    working_set = master_set;

    // Wait for activity on any socket
    int activity = select(max_fd + 1, &working_set, NULL, NULL, &timeout);

    if (activity < 0 && errno != EINTR) {
      perror("[SERVER] select error");
      break;
    }

    // Check for activity on all sockets
    for (int i = 0; i <= max_fd; i++) {
      if (FD_ISSET(i, &working_set)) {
        // New connection on server socket
        if (i == server_fd) {
          struct sockaddr_in client_addr;
          socklen_t client_addrlen = sizeof(client_addr);
          int client_fd = accept(server_fd, (struct sockaddr *)&client_addr,
                                 &client_addrlen);

          if (client_fd < 0) {
            perror("[SERVER] accept failed");
            continue;
          }
          client_connected = true;
          int sz = 1 << 20; // 1 MB
          if (setsockopt(client_fd, SOL_SOCKET, SO_RCVBUF, &sz, sizeof(sz)) <
              0) {
            perror("[SERVER] setsockopt SO_RCVBUF failed");
            close(client_fd);
            continue;
          }

          // Set non-blocking for client socket too
          flags = fcntl(client_fd, F_GETFL, 0);
          fcntl(client_fd, F_SETFL, flags | O_NONBLOCK);

          // Get client info
          char client_ip[INET_ADDRSTRLEN];
          inet_ntop(AF_INET, &client_addr.sin_addr, client_ip,
                    sizeof(client_ip));
          int client_port = ntohs(client_addr.sin_port);

          printf("[SERVER] New connection from %s:%d\n", client_ip,
                 client_port);

          // Add to client manager
          int client_idx =
              client_manager_add(client_fd, client_ip, client_port);
          if (client_idx < 0) {
            printf("[SERVER] Max clients reached, rejecting connection\n");
            close(client_fd);
            continue;
          }

          // Get stable position for new client (and recalculate)
          printf("[SERVER] Getting stable position for new client...\n");
          localize_force_recalculate_stable_position();

          Coordinates stable_pos;
          if (localize_get_stable_position(&stable_pos)) {
            // Send position to client
            char pos_msg[256];
            snprintf(pos_msg, sizeof(pos_msg), "Localize: X=%.2f Y=%.2f",
                     stable_pos.x, stable_pos.y);
            if (send(client_fd, pos_msg, strlen(pos_msg), 0) > 0) {
              printf("[SERVER] Sent stable position to client: X=%.3f Y=%.3f\n",
                     stable_pos.x, stable_pos.y);
            }

            // Initialize optical flow with this position
            optical_flow_set_initial_position(stable_pos.x, stable_pos.y);
            printf("[SERVER] Optical flow initialized with stable position\n");
          } else {
            printf(
                "[SERVER] Warning: Could not get stable position for client\n");
          }

          // Add to master set
          FD_SET(client_fd, &master_set);
          if (client_fd > max_fd) {
            max_fd = client_fd;
          }
        }
        // Data from client
        else {
          int rc = pump_jsonl(i);
          if (rc <= 0) { // 0: peer đóng, -1: lỗi
            printf("[SERVER] Client disconnected (socket %d)\n", i);
            client_manager_remove(i);
            FD_CLR(i, &master_set);
            close(i);
          }
        }
      }
    }
  }

  for (int i = 0; i <= max_fd; i++) {
    if (FD_ISSET(i, &master_set) && i != server_fd) {
      close(i);
    }
  }

  client_manager_destroy();
  close(server_fd);
  return NULL;
}