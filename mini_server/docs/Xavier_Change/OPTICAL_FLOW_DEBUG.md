# HƯỚNG DẪN DEBUG OPTICAL FLOW

## Vấn đề: Optical Flow không nhận được dữ liệu

Nếu bạn thấy log:
```
[Optical Flow] Thread started, waiting for data...
[FREQ] Pos-OptFlow: 0.00 Hz
```

Và không có dữ liệu trong database, hãy làm theo các bước sau:

## Bước 1: Kiểm tra kết nối phần cứng

### 1.1 Kiểm tra thiết bị UART tồn tại
```bash
ls -l /dev/ttyTHS0
```

Nếu không tồn tại, thử:
```bash
ls -l /dev/tty*
```
Tìm xem optical flow sensor kết nối ở cổng nào (ttyUSB0, ttyTHS1, etc.)

### 1.2 Kiểm tra quyền truy cập
```bash
sudo chmod 666 /dev/ttyTHS0
```

### 1.3 Kiểm tra không có process nào đang sử dụng
```bash
sudo lsof | grep ttyTHS0
```

## Bước 2: Test UART trực tiếp

### 2.1 Build chương trình test
```bash
chmod +x build_test_uart.sh
./build_test_uart.sh
```

### 2.2 Chạy test
```bash
sudo ./test_uart
# Hoặc nếu sensor kết nối ở cổng khác:
sudo ./test_uart /dev/ttyUSB0
```

**Kết quả mong đợi:**
- Nếu sensor hoạt động đúng, bạn sẽ thấy:
  ```
  [READ 1] Nhận X bytes:
  EF 0F 00 51 ...
  >>> FOUND MICOLINK HEADER at offset 0! (Packet #1)
      vx_raw=XXX, vy_raw=XXX, quality=XX
  ```

- Nếu KHÔNG thấy dữ liệu:
  - Kiểm tra nguồn điện sensor
  - Kiểm tra kết nối TX/RX (chéo: TX sensor → RX Jetson)
  - Kiểm tra GND chung
  - Kiểm tra baudrate sensor (phải là 115200)

## Bước 3: Kiểm tra log chi tiết

Code đã được update với logging chi tiết. Khi chạy chương trình chính, bạn sẽ thấy:

### 3.1 Nếu mở UART thành công:
```
[Optical Flow] UART /dev/ttyTHS0 opened successfully (fd=X)
[Optical Flow] UART configured: 115200 baud, 8N1, non-blocking
[Optical Flow] Thread started, waiting for data...
```

### 3.2 Nếu nhận được dữ liệu:
```
[Optical Flow] ✓ Bắt đầu nhận dữ liệu từ sensor! (bytes=X)
[Optical Flow] Packet #50: vx_raw=X, vy_raw=X, quality=X
[Optical Flow] Velocity: vx=0.XXX m/s, vy=0.XXX m/s
```

### 3.3 Nếu KHÔNG nhận được dữ liệu:
```
[Optical Flow] ⚠ Cảnh báo: Không nhận được dữ liệu sau 5 giây
[Optical Flow] Kiểm tra lại:
  1. Sensor có nguồn điện?
  2. Kết nối UART đúng (TX->RX, RX->TX)?
  3. Ground chung?
  4. Sensor đã được cấu hình output 115200 baud?
```

## Bước 4: Kiểm tra cấu hình sensor MTF-02

### 4.1 Cấu hình Micolink protocol
Sensor MTF-02 phải được cấu hình:
- Protocol: Micolink
- Baudrate: 115200
- Output rate: 50Hz (khuyến nghị)

### 4.2 Cách kiểm tra/cấu hình
Sử dụng phần mềm cấu hình của nhà sản xuất hoặc AT commands qua UART.

## Bước 5: Thay đổi cổng UART trong code (nếu cần)

Nếu sensor kết nối ở cổng khác (ví dụ /dev/ttyUSB0), sửa trong file:
`src/optical_flow_integration.c`

```c
// Thay đổi dòng này:
int fd = open("/dev/ttyTHS0", O_RDWR | O_NOCTTY);

// Thành:
int fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
```

Sau đó rebuild:
```bash
./build.sh
```

## Bước 6: Kiểm tra trong database

Sau khi sensor hoạt động, dữ liệu sẽ được ghi vào bảng `position_logs` với các cột:
- `optical_x`, `optical_y`: Position đã tích phân
- `optical_vx`, `optical_vy`: Velocity từ sensor

Kiểm tra:
```bash
sqlite3 robot_logs_optical.db
sqlite> SELECT timestamp, optical_x, optical_y, optical_vx, optical_vy 
        FROM position_logs 
        WHERE optical_x IS NOT NULL 
        LIMIT 10;
```

## Các vấn đề thường gặp

### Vấn đề 1: Permission denied
**Giải pháp:**
```bash
sudo chmod 666 /dev/ttyTHS0
# Hoặc thêm user vào dialout group:
sudo usermod -a -G dialout $USER
# Sau đó logout và login lại
```

### Vấn đề 2: Device busy
**Giải pháp:**
```bash
# Tìm process đang dùng:
sudo lsof | grep ttyTHS0
# Kill process đó:
sudo kill -9 <PID>
```

### Vấn đề 3: Dữ liệu bị nhiễu (garbage data)
**Giải pháp:**
- Kiểm tra baudrate phải đúng 115200
- Thêm pull-up resistor trên RX line nếu cần
- Kiểm tra GND chung
- Dùng cáp ngắn hơn
- Thêm điện trở 120Ω nếu dùng RS485

### Vấn đề 4: Quality thấp, data bị reject
Trong log sẽ thấy:
```
[Optical Flow] Packet rejected: quality=5 < 10
```

**Giải pháp:**
- Đảm bảo sensor có bề mặt quan sát rõ ràng
- Khoảng cách phù hợp (tùy model, thường 0.5m - 3m)
- Đủ ánh sáng (nếu dùng camera thường)
- Giảm OPTICAL_FLOW_MIN_QUALITY trong optical_flow.h nếu cần

## Liên hệ hỗ trợ

Nếu vẫn không giải quyết được, cung cấp:
1. Output của `sudo ./test_uart`
2. Log đầy đủ khi chạy chương trình chính
3. Thông tin phần cứng (Jetson model, sensor model)
4. Sơ đồ kết nối
