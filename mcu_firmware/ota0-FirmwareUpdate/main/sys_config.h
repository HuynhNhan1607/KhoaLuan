#ifndef SYS_CONFIG_H
#define SYS_CONFIG_H

#define ROBOT_ID 2

#if ROBOT_ID == 1
#define WIFI_SSID "Robot1"
#define WIFI_PASS "25102004"
#define SERVER_IP "192.168.4.1"
#define STATIC_IP_THIRD_OCTET 4
#elif ROBOT_ID == 2
#define WIFI_SSID "Robot2"
#define WIFI_PASS "25102004"
#define SERVER_IP "192.168.5.1"
#define STATIC_IP_THIRD_OCTET 5
#endif

#endif // SYS_CONFIG_H