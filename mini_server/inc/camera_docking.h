#pragma once
/*
 * camera_docking.h — Shared constants between pose_estimate_service (C++)
 *                    and trajectory_executor (C).
 *
 * Include bởi:
 *   - camera/pose_estimate.cpp       (via -I../inc  từ trong camera/)
 *   - mini_server/src/trajectory_executor.c (via -I./inc  từ mini_server/)
 */

/* ---------- TCP vision server ---------- */
#define CAMERA_DOCKING_HOST  "127.0.0.1"
#define CAMERA_DOCKING_PORT  9091

/* ---------- ArUco marker ---------- */
/* ID của marker dán lên vật cần dock */
#define CAMERA_DOCKING_MARKER_ID   0
/* Kích thước cạnh marker (m) */
#define CAMERA_DOCKING_MARKER_SIZE 0.05f

/* ---------- Camera capture ---------- */
#define CAMERA_DOCKING_INDEX       1
#define CAMERA_DOCKING_WIDTH       640
#define CAMERA_DOCKING_HEIGHT      480

/* ---------- Vision loop rate ---------- */
/* usleep delay để đạt ~20 Hz */
#define CAMERA_DOCKING_LOOP_DELAY_US  50000

/* ---------- JSON field names ---------- */
#define CAMERA_DOCKING_JSON_TYPE   "docking_vision"
