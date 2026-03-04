#ifndef WIFI_H
#define WIFI_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

#include "sys_config.h"

#if ROBOT_ID == 1
// WiFi Configuration
#define WIFI_SSID "Robot1"
#define WIFI_PASS "25102004"
#elif ROBOT_ID == 2
#define WIFI_SSID "Robot2"
#define WIFI_PASS "25102004"
#elif ROBOT_ID == 3
#define WIFI_SSID "Mineooo"
#define WIFI_PASS "14052004"
#endif
    // #define WIFI_SSID "CEEC_Tenda"
    // #define WIFI_PASS "1denmuoi1"

// IP Configuration: 1 = Static IP, 0 = DHCP
#if ROBOT_ID == 3
#define STATIC_IP 0
#else
#define STATIC_IP 1
#endif
    // WiFi status variables
    extern bool g_wifi_connected;
    extern bool g_got_ip;

    // WiFi initialization
    void wifi_init_sta(void);

#ifdef __cplusplus
}
#endif

#endif // WIFI_H
