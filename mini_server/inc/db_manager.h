#ifndef DB_MANAGER_H
#define DB_MANAGER_H

#include <sqlite3.h>
#include <stdbool.h>

#define DB_FILE "robot_logs_optical.db"
// Khởi tạo database
void initialize_database(sqlite3 *db);

// Bắt đầu một lần chạy mới
long start_new_run(sqlite3 *db);

// Thực thi câu lệnh SQL
void execute_sql(sqlite3 *db, const char *sql);

// Các hàm log
void log_bno055(sqlite3 *db, long run_id,
                float heading, float pitch, float roll,
                float w, float x, float y, float z,
                float accel_x, float accel_y, float accel_z,
                float gravity_x, float gravity_y, float gravity_z,
                float gyro_raw_x, float gyro_raw_y, float gyro_raw_z,
                const char *status, long timestamp);

void log_encoder(sqlite3 *db, long run_id,
                 float rpm1, float rpm2, float rpm3, float rpm4, long timestamp);

void log_position(sqlite3 *db, long run_id,
                  float ekf_x, float ekf_y, float ekf_vx, float ekf_vy, float ekf_theta,
                  float optical_x, float optical_y, float optical_vx, float optical_vy,
                  float odom_x, float odom_y, float odom_vx, float odom_vy,
                  float loc_x, float loc_y, float loc_vx, float loc_vy,
                  float quality, float optical_quality,
                  long timestamp);

void log_system(sqlite3 *db, long run_id, const char *message, long timestamp);

// Bắt đầu một transaction
void begin_transaction(sqlite3 *db);

// Commit một transaction
void commit_transaction(sqlite3 *db);

// Bắt đầu một transaction
void begin_transaction(sqlite3 *db);

// Commit một transaction
void commit_transaction(sqlite3 *db);

#endif // DB_MANAGER_H