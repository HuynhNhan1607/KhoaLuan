# Docking Camera Service

Tài liệu này mô tả cách chạy `pose_estimate_service` (C++) và tích hợp với docking trong `mini_server`.

## 1) Build

Build `mini_server` như cũ (chạy từ thư mục `mini_server/`):

```bash
./build.sh robot1
```

Nếu máy có OpenCV (`pkg-config opencv4`), script sẽ build thêm:

- `camera/pose_estimate_service`

Build thủ công camera service (chạy từ thư mục `mini_server/`):

```bash
# Tải aruco headers nếu chưa có (chạy 1 lần)
make -C camera deps

# Build
g++ -Wall -Wextra -std=c++17 \
    -I./inc -Icamera/vendor/include \
    -o camera/pose_estimate_service camera/pose_estimate.cpp \
    $(pkg-config --cflags --libs opencv4) \
    $(find /usr/lib/aarch64-linux-gnu /usr/local/lib /usr/lib -name 'libopencv_aruco.so.*' 2>/dev/null | head -1)
```

## 2) Runtime thứ tự chạy

Tất cả lệnh chạy từ thư mục `mini_server/`:

1. Chạy camera service trước:

```bash
./camera/pose_estimate_service
```

2. Chạy mini server:

```bash
./single_mini-server
```

`trajectory_executor` sẽ kết nối `127.0.0.1:9091` để lấy luồng vision JSON line:

```json
{"type":"docking_vision","found":1,"x":0.01234,"z":0.18500}
```

## 3) Behavior docking trong trajectory

- Khi vào Acceptance Zone và hold đủ thời gian, robot chuyển sang docking.
- Căn chỉnh theo thứ tự:
  - Căn `X` (strafe trái/phải)
  - Căn khoảng cách `Y` (tiến/lùi qua ngưỡng `z`)
- Mapping vận tốc global:
  - `x > 0` -> `Vx > 0` (vật phía phải)
  - `x < 0` -> `Vx < 0` (vật phía trái)
  - `z` xa hơn mục tiêu -> `Vy > 0` (tiến)
  - `z` gần hơn mục tiêu -> `Vy < 0` (lùi)
- Nếu quét phải rồi trái vẫn không thấy vật: gửi `not_found`.
- Chỉ gửi `arrived` sau khi docking xong.

## 4) Docking test độc lập

Gửi command control:

```json
{"type":"control","cmd":"execute_docking_test"}
```

Kết quả ack ngay:

- `docking_test_started` hoặc
- `docking_busy`

Kết quả cuối cùng:

- `arrived` (dock thành công), hoặc
- `not_found` (không tìm thấy vật / timeout).

## 5) Thông số tuning chính

Đặt trong `mini_server/inc/trajectory_executor.h`:

- `DOCK_SCAN_VX = 0.05f`
- `DOCK_ALIGN_VX = 0.05f`
- `DOCK_APPROACH_VY = 0.05f`
- `DOCK_X_TOL`
- `DOCK_TARGET_Z`
- `DOCK_Y_TOL`
- `DOCK_SCAN_TIMEOUT_MS`
- `DOCK_TOTAL_TIMEOUT_MS`
