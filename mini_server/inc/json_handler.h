#ifndef JSON_HANDLER_H
#define JSON_HANDLER_H

#include <stdbool.h>

// Khởi tạo json handler và bắt đầu thread
bool json_handler_init(void);

// Thêm một JSON message vào queue
bool json_handler_add_message(const char *json_message, int length);

// Gửi optical flow position đến laptop server
void send_optical_flow_position_to_laptop(float pos_x, float pos_y, float vel_x,
                                          float vel_y);

// Giải phóng tài nguyên
void json_handler_cleanup(void);

// Parse và xử lý JSON message từ laptop
void parse_json_message(const char *json_str, int length);

#endif // JSON_HANDLER_H