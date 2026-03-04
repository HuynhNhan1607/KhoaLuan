#include <stdio.h>
#include <stdlib.h>
#include <sqlite3.h>
#include <time.h>
#include <unistd.h>
#include <string.h>
#include <sys/time.h>

#include "db_manager.h"

#define DB_FILE "robot_logs_optical.db"
#define MAX_RUNS 15
#define TIME_BUFFER_SIZE 64

void begin_transaction(sqlite3 *db)
{
    execute_sql(db, "BEGIN TRANSACTION;");
}

// Hàm commit một transaction
void commit_transaction(sqlite3 *db)
{
    execute_sql(db, "COMMIT;");
}

// Hàm helper để lấy thời gian hiện tại dạng chuỗi định dạng cho runs table
void get_formatted_time(char *buffer, size_t size)
{
    time_t now = time(NULL);
    struct tm *timeinfo = localtime(&now);
    strftime(buffer, size, "%Y-%m-%d %H:%M:%S", timeinfo);
}

// Hàm lấy timestamp (milliseconds kể từ khi chương trình bắt đầu)
long get_timestamp_ms()
{
    static struct timeval start_time = {0};
    static long last_timestamp = 0;
    struct timeval now;

    if (start_time.tv_sec == 0)
    {
        gettimeofday(&start_time, NULL);
        return 0;
    }

    gettimeofday(&now, NULL);
    long timestamp = (now.tv_sec - start_time.tv_sec) * 1000 +
                     (now.tv_usec - start_time.tv_usec) / 1000;
    if (timestamp <= last_timestamp)
        timestamp = last_timestamp + 1;
    last_timestamp = timestamp;
    return timestamp;
}

// Hàm helper để thực thi SQL và kiểm tra lỗi
void execute_sql(sqlite3 *db, const char *sql)
{
    char *zErrMsg = 0;
    if (sqlite3_exec(db, sql, 0, 0, &zErrMsg) != SQLITE_OK)
    {
        fprintf(stderr, "SQL error: %s\n", zErrMsg);
        fprintf(stderr, "Failed query: %s\n", sql);
        sqlite3_free(zErrMsg);
    }
}

// Hàm khởi tạo database và tất cả các bảng
void initialize_database(sqlite3 *db)
{
    printf("Initializing database...\n");

    // Bật hỗ trợ khóa ngoại (quan trọng cho ON DELETE CASCADE)
    execute_sql(db, "PRAGMA foreign_keys = ON;");

    // Bảng quản lý các lần chạy
    const char *sql_runs =
        "CREATE TABLE IF NOT EXISTS runs ("
        "  run_id INTEGER PRIMARY KEY,"
        "  start_time TEXT NOT NULL"
        ");";
    execute_sql(db, sql_runs);

    // Bảng Log BNO055
    const char *sql_bno055 =
        "CREATE TABLE IF NOT EXISTS bno055_logs ("
        "  run_id INTEGER NOT NULL,"
        "  timestamp INTEGER NOT NULL," // Milliseconds từ khi bắt đầu
        "  heading REAL,"
        "  pitch REAL,"
        "  roll REAL,"
        "  w REAL,"
        "  x REAL,"
        "  y REAL,"
        "  z REAL,"
        "  accel_x REAL,"
        "  accel_y REAL,"
        "  accel_z REAL,"
        "  gravity_x REAL,"
        "  gravity_y REAL,"
        "  gravity_z REAL,"
        "  gyro_raw_x REAL,"
        "  gyro_raw_y REAL,"
        "  gyro_raw_z REAL,"
        "  status TEXT,"
        "  PRIMARY KEY (run_id, timestamp),"
        "  FOREIGN KEY(run_id) REFERENCES runs(run_id) ON DELETE CASCADE"
        ");";
    execute_sql(db, sql_bno055);

    // Bảng Log Encoder
    const char *sql_encoder =
        "CREATE TABLE IF NOT EXISTS encoder_logs ("
        "  run_id INTEGER NOT NULL,"
        "  timestamp INTEGER NOT NULL," // Milliseconds từ khi bắt đầu
        "  rpm1 REAL,"
        "  rpm2 REAL,"
        "  rpm3 REAL,"
        "  rpm4 REAL,"
        "  PRIMARY KEY (run_id, timestamp),"
        "  FOREIGN KEY(run_id) REFERENCES runs(run_id) ON DELETE CASCADE"
        ");";
    execute_sql(db, sql_encoder);

    // Bảng Log Position
    // LÚU Ý: bno055_x/y/vx/vy giờ lưu dữ liệu OPTICAL FLOW (đã tích phân position)
    const char *sql_position =
        "CREATE TABLE IF NOT EXISTS position_logs ("
        "  run_id INTEGER NOT NULL,"
        "  timestamp INTEGER NOT NULL," // Milliseconds từ khi bắt đầu
        "  ekf_x REAL,"
        "  ekf_y REAL,"
        "  ekf_vx REAL,"
        "  ekf_vy REAL,"
        "  ekf_theta REAL,"
        "  optical_x REAL,"  // OPTICAL FLOW position X (tích phân)
        "  optical_y REAL,"  // OPTICAL FLOW position Y (tích phân)
        "  optical_vx REAL," // OPTICAL FLOW velocity X
        "  optical_vy REAL," // OPTICAL FLOW velocity Y
        "  odom_x REAL,"
        "  odom_y REAL,"
        "  odom_vx REAL,"
        "  odom_vy REAL,"
        "  loc_x REAL,"
        "  loc_y REAL,"
        "  loc_vx REAL,"
        "  loc_vy REAL,"
        "  quality REAL,"         // Localization quality
        "  optical_quality REAL," // Optical flow quality
        "  PRIMARY KEY (run_id, timestamp),"
        "  FOREIGN KEY(run_id) REFERENCES runs(run_id) ON DELETE CASCADE"
        ");";
    execute_sql(db, sql_position);

    // Bảng Log System
    const char *sql_system =
        "CREATE TABLE IF NOT EXISTS system_logs ("
        "  run_id INTEGER NOT NULL,"
        "  timestamp INTEGER NOT NULL," // Milliseconds từ khi bắt đầu
        "  message TEXT,"
        "  PRIMARY KEY (run_id, timestamp),"
        "  FOREIGN KEY(run_id) REFERENCES runs(run_id) ON DELETE CASCADE"
        ");";
    execute_sql(db, sql_system);

    printf("Database initialized successfully.\n\n");
}

// Bắt đầu một lần chạy mới, dọn dẹp nếu cần và trả về run_id mới
long start_new_run(sqlite3 *db)
{
    sqlite3_stmt *stmt;
    long oldest_run_id = -1;
    int run_count = 0;

    // 1. Đếm số lần chạy hiện tại
    sqlite3_prepare_v2(db, "SELECT COUNT(*) FROM runs;", -1, &stmt, 0);
    if (sqlite3_step(stmt) == SQLITE_ROW)
    {
        run_count = sqlite3_column_int(stmt, 0);
    }
    sqlite3_finalize(stmt);

    printf("Current run count: %d\n", run_count);

    // 2. Nếu đạt đến giới hạn, tìm và xóa lần chạy cũ nhất
    if (run_count >= MAX_RUNS)
    {
        printf("Max runs reached. Deleting the oldest run.\n");
        // Tìm run_id cũ nhất (có run_id nhỏ nhất)
        sqlite3_prepare_v2(db, "SELECT run_id FROM runs ORDER BY run_id ASC LIMIT 1;", -1, &stmt, 0);
        if (sqlite3_step(stmt) == SQLITE_ROW)
        {
            oldest_run_id = sqlite3_column_int(stmt, 0);
        }
        sqlite3_finalize(stmt);

        if (oldest_run_id != -1)
        {
            char delete_sql[100];
            snprintf(delete_sql, sizeof(delete_sql), "DELETE FROM runs WHERE run_id = %ld;", oldest_run_id);
            printf("Executing: %s (All associated logs will be auto-deleted)\n", delete_sql);
            execute_sql(db, delete_sql);
        }
    }

    // 3. Tạo một lần chạy mới với định dạng thời gian
    char time_str[TIME_BUFFER_SIZE];
    get_formatted_time(time_str, TIME_BUFFER_SIZE);

    char insert_run_sql[150];
    snprintf(insert_run_sql, sizeof(insert_run_sql), "INSERT INTO runs (start_time) VALUES ('%s');", time_str);
    execute_sql(db, insert_run_sql);

    // 4. Lấy run_id của lần chạy vừa tạo
    long new_run_id = sqlite3_last_insert_rowid(db);
    printf("Started new run with run_id: %ld at %s\n", new_run_id, time_str);
    return new_run_id;
}

// Các hàm ghi log cho từng loại dữ liệu
void log_bno055(sqlite3 *db, long run_id,
                float heading, float pitch, float roll,
                float w, float x, float y, float z,
                float accel_x, float accel_y, float accel_z,
                float gravity_x, float gravity_y, float gravity_z,
                float gyro_raw_x, float gyro_raw_y, float gyro_raw_z,
                const char *status, long timestamp)
{
    // long timestamp = get_timestamp_ms();
    char sql[1024];
    snprintf(sql, sizeof(sql),
             "INSERT INTO bno055_logs (run_id, timestamp, heading, pitch, roll, "
             "w, x, y, z, accel_x, accel_y, accel_z, gravity_x, gravity_y, gravity_z, "
             "gyro_raw_x, gyro_raw_y, gyro_raw_z, status) VALUES "
             "(%ld, %ld, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, '%s');",
             run_id, timestamp, heading, pitch, roll, w, x, y, z,
             accel_x, accel_y, accel_z, gravity_x, gravity_y, gravity_z,
             gyro_raw_x, gyro_raw_y, gyro_raw_z, status);
    execute_sql(db, sql);
}

void log_encoder(sqlite3 *db, long run_id, float rpm1, float rpm2, float rpm3, float rpm4, long timestamp)
{
    // long timestamp = get_timestamp_ms();
    char sql[512];
    snprintf(sql, sizeof(sql),
             "INSERT INTO encoder_logs (run_id, timestamp, rpm1, rpm2, rpm3, rpm4) "
             "VALUES (%ld, %ld, %f, %f, %f, %f);",
             run_id, timestamp, rpm1, rpm2, rpm3, rpm4);
    execute_sql(db, sql);
}

void log_position(sqlite3 *db, long run_id,
                  float ekf_x, float ekf_y, float ekf_vx, float ekf_vy, float ekf_theta,
                  float optical_x, float optical_y, float optical_vx, float optical_vy,
                  float odom_x, float odom_y, float odom_vx, float odom_vy,
                  float loc_x, float loc_y, float loc_vx, float loc_vy,
                  float quality, float optical_quality,
                  long timestamp)
{
    char sql[1024];
    snprintf(sql, sizeof(sql),
             "INSERT INTO position_logs (run_id, timestamp, "
             "ekf_x, ekf_y, ekf_vx, ekf_vy, ekf_theta, "
             "optical_x, optical_y, optical_vx, optical_vy, "
             "odom_x, odom_y, odom_vx, odom_vy, "
             "loc_x, loc_y, loc_vx, loc_vy, quality, optical_quality) "
             "VALUES (%ld, %ld, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f);",
             run_id, timestamp,
             ekf_x, ekf_y, ekf_vx, ekf_vy, ekf_theta,
             optical_x, optical_y, optical_vx, optical_vy,
             odom_x, odom_y, odom_vx, odom_vy,
             loc_x, loc_y, loc_vx, loc_vy, quality, optical_quality);
    execute_sql(db, sql);
}

void log_system(sqlite3 *db, long run_id, const char *message, long timestamp)
{
    // long timestamp = get_timestamp_ms();

    char *sql = sqlite3_mprintf(
        "INSERT INTO system_logs (run_id, timestamp, message) VALUES (%ld, %ld, %Q);",
        run_id, timestamp, message);

    execute_sql(db, sql);
    sqlite3_free(sql); // Giải phóng bộ nhớ sau khi thực thi
}
#ifdef DB_MANAGER_TEST
// Mô phỏng việc ghi log cho một lần chạy
void simulate_logging_for_one_run(sqlite3 *db, long run_id)
{
    printf("--- Simulating logging for run_id: %ld ---\n", run_id);

    // Chỉ mô phỏng ghi 2 lô dữ liệu cho nhanh
    for (int batch = 0; batch < 90 * 5; batch++)
    {
        execute_sql(db, "BEGIN TRANSACTION;");

        // Ghi log BNO055
        log_bno055(db, run_id,
                   25.5, 0.2, 1.1,     // heading, pitch, roll
                   0.9, 0.1, 0.0, 0.1, // quaternion: w, x, y, z
                   0.05, 0.02, 9.8,    // linear acceleration
                   0.0, 0.0, 9.81,     // gravity
                   0.01, 0.01, 0.02,   // gyro raw
                   "moving");          // status

        // Ghi log Encoder
        log_encoder(db, run_id, 65.2, 64.8, 65.0, 0.0); // rpm1-4

        // Ghi log Position
        log_position(db, run_id,
                     1.2, 2.3,               // ekf_x, ekf_y
                     1.24, 2.35, 0.1, 0.2,   // bno055_x, y, vx, vy
                     1.22, 2.33, 0.12, 0.21, // odom_x, y, vx, vy
                     1.23, 2.34);            // loc_x, loc_y

        // Ghi log System
        log_system(db, run_id, "System status OK");

        execute_sql(db, "COMMIT;");
        // printf("  Batch %d logged.\n", batch);
        // sleep(1); // Giả lập thời gian trôi qua
    }
    printf("--- Finished logging for run_id: %ld ---\n\n", run_id);
}

int main()
{
    // Xóa file db cũ để chạy lại từ đầu cho dễ demo
    remove(DB_FILE);

    sqlite3 *db;
    if (sqlite3_open(DB_FILE, &db))
    {
        fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
        return 1;
    }

    initialize_database(db);

    // --- Mô phỏng 4 lần chạy liên tiếp ---
    long run1_id = start_new_run(db); // Lần 1
    simulate_logging_for_one_run(db, run1_id);

    long run2_id = start_new_run(db); // Lần 2
    simulate_logging_for_one_run(db, run2_id);

    long run3_id = start_new_run(db); // Lần 3
    simulate_logging_for_one_run(db, run3_id);

    printf("!!! At this point, database contains logs for runs: %ld, %ld, %ld.\n\n", run1_id, run2_id, run3_id);

    long run4_id = start_new_run(db); // Lần 4 - Sẽ xóa dữ liệu của lần 1
    simulate_logging_for_one_run(db, run4_id);

    printf("!!! Now, database should contain logs for runs: %ld, %ld, %ld.\n", run2_id, run3_id, run4_id);
    printf("!!! Logs for run_id %ld should be gone.\n", run1_id);

    sqlite3_close(db);
    return 0;
}
#endif