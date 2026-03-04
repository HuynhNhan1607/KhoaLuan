// Debugging
#define LOG_SERVER 1
// THETA
#define ENABLE_IMU 1

#define BNO055_CALIBRATED_BIT BIT(0)

// Robot Information - conditional based on ROBOT_ID
#if ROBOT_ID == 1
#define ID_ROBOT "robot1"
#define WIFI_SSID "Robot1"
#define SERVER_IP "192.168.4.1"
#elif ROBOT_ID == 2
#define ID_ROBOT "robot2"
#define WIFI_SSID "Robot2"
#define SERVER_IP "192.168.5.1"
#else
#error "Invalid ROBOT_ID! Must be 1 or 2"
#endif

#define WIFI_PASS "25102004"
#define SERVER_PORT 8080
#define STATIC_IP 1

#define WHEEL_RADIUS 0.048   // Bán kính bánh xe (m)
#define ROBOT_RADIUS 0.15121 // Khoảng cách từ tâm robot đến bánh xe (m)

#define WHEEL_ANGLE_OFFSET_FROM_DIAGONAL                                       \
  0.573 // Góc tạo bởi đường đi qua trọng tâm xe và tâm bánh xe với trục x (rad)
#define WEIGHT 2.0 // Trọng lượng robot (kg)
