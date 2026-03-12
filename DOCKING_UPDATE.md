# Docking Integration Update

Tài liệu này mô tả các thay đổi đã triển khai cho yêu cầu docking bằng camera sau khi robot đi hết trajectory.

## 1) Mục tiêu đã hoàn thành

- Không gửi `arrived` ngay khi vào Acceptance Zone.
- Chạy docking theo thứ tự:
  - căn chỉnh trục `X` trước,
  - sau đó mới tiến/lùi theo `Y`.
- Nếu quét phải rồi trái vẫn không tìm thấy vật thì gửi `not_found`.
- Chỉ gửi `arrived` khi docking thành công.
- Có chế độ test docking độc lập nhưng dùng cùng logic runtime.

## 2) File đã thay đổi

- `mini_server/src/trajectory_executor.c`
- `mini_server/inc/trajectory_executor.h`
- `mini_server/src/json_handler.c`
- `mini_server/src/socket.c`
- `mini_server/build.sh`
- `camera/pose_estimate.cpp` (mới, thay thế hướng Python runtime)
- `camera/README_docking.md` (mới)

## 3) Kiến trúc mới

### 3.1 Camera service tách riêng

- Tạo service C++: `camera/pose_estimate.cpp`.
- Service đọc camera + ArUco (id=0), xuất dữ liệu qua TCP localhost `127.0.0.1:9091`.
- Dữ liệu gửi theo JSON line:

```json
{"type":"docking_vision","found":1,"x":0.01234,"z":0.18500}
```

Trong đó:
- `found`: có thấy vật/marker hay không.
- `x`: lệch ngang (dùng để căn `Vx`).
- `z`: khoảng cách trước-sau (dùng để căn `Vy`).

### 3.2 Docking state machine trong trajectory

Được thêm vào `trajectory_executor.c`:

- `DOCK_STATE_SEARCH_RIGHT`
- `DOCK_STATE_SEARCH_LEFT`
- `DOCK_STATE_ALIGN_X`
- `DOCK_STATE_APPROACH_Y`

Luồng:
1. Trajectory vào Acceptance Zone và hold đủ thời gian.
2. Chuyển sang docking state machine.
3. Hoàn thành docking -> gửi `arrived`.
4. Thất bại/timeout -> gửi `not_found`.

## 4) Mapping vận tốc global (đúng yêu cầu)

- Vật bên trái: `Vx < 0`
- Vật bên phải: `Vx > 0`
- Tiến gần vật: `Vy > 0`
- Lùi ra xa vật: `Vy < 0`

Không xoay hệ trục trong docking.

## 5) Cấu hình docking đã thêm

Trong `mini_server/inc/trajectory_executor.h`:

- `DOCK_SCAN_VX = 0.05f`
- `DOCK_ALIGN_VX = 0.05f`
- `DOCK_APPROACH_VY = 0.05f`
- `DOCK_X_TOL`
- `DOCK_TARGET_Z`
- `DOCK_Y_TOL`
- `DOCK_SCAN_TIMEOUT_MS`
- `DOCK_TOTAL_TIMEOUT_MS`

## 6) Docking test độc lập

- Thêm API: `trajectory_start_docking_test()` trong `trajectory_executor`.
- Thêm command control:

```json
{"type":"control","cmd":"execute_docking_test"}
```

Xử lý:
- `socket.c` forward command tới parser.
- `json_handler.c` gọi `trajectory_start_docking_test()`.

Ack tức thời:
- `docking_test_started` hoặc `docking_busy`.

Kết quả cuối:
- `arrived` hoặc `not_found`.

## 7) Log debug đã bổ sung

### 7.1 Trajectory / Docking

Trong `trajectory_executor.c` đã thêm log:
- state transition (`STATE A -> B`),
- tick định kỳ (state, found, x, z, vision_age, elapsed),
- connect/recv/disconnect vision,
- timeout phải/trái và timeout tổng,
- sự kiện căn xong X, done, not found.

### 7.2 Camera service

Trong `pose_estimate.cpp` đã thêm log:
- trạng thái kết nối mini_server,
- thay đổi trạng thái `found`,
- tick định kỳ với `x/z`.

### 7.3 Docking test command

Trong `json_handler.c` đã thêm log khi:
- nhận `execute_docking_test`,
- trả `docking_test_started` hoặc `docking_busy`.

## 8) Build và vận hành

- `mini_server/build.sh` vẫn build C cho mini_server.
- Có thêm nhánh build camera service C++ nếu có `pkg-config opencv4`.
- Hướng dẫn chạy chi tiết: `camera/README_docking.md`.

## 9) Lưu ý môi trường

- `pose_estimate.cpp` dùng socket/header POSIX (`arpa/inet.h`, `unistd.h`), phù hợp môi trường Linux/Jetson.
- Nếu chạy trên Windows native cần port lại phần socket/hệ thống hoặc chạy qua WSL/Linux target.

## 10) Tóm tắt hành vi cuối cùng

- Trước đây: vào Acceptance Zone -> gửi `arrived` ngay.
- Hiện tại: vào Acceptance Zone -> chạy docking camera -> chỉ gửi `arrived` khi docking thành công; nếu không tìm thấy vật thì gửi `not_found`.
