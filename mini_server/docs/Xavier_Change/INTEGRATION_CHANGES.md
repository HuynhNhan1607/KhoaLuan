# THAY ĐỔI TÍCH HỢP OPTICAL FLOW

## 📝 Tóm tắt thay đổi

### ✅ Đã hoàn thành:

1. **Di chuyển Optical Flow task vào main.c**
   - Đã thêm `optical_flow_uart_thread()` vào [main.c](e:\Multiple_Mobile_Robot\mini_server\src\main.c#L30-L120)
   - Thread này đọc dữ liệu từ `/dev/ttyTHS0` với tốc độ 50Hz
   - Xử lý qua LPF và coordinate transform
   - Update trực tiếp vào EKF với `EKF_SENSOR_OPTICAL_FLOW`

2. **Thay thế BNO055 position → Optical Flow position**
   - Đổi tên hàm: `send_bno055_position_to_laptop()` → `send_optical_flow_position_to_laptop()`
   - JSON source: `"bno055"` → `"optical_flow"`
   - Database: cột bno055 giờ lưu optical flow data

3. **Cập nhật json_handler.c**
   - Thêm external variables cho optical flow
   - Parse BNO055 data → gửi optical flow velocity
   - Xử lý source "optical_flow" trong parse_json_message()
   - Cập nhật flush_queue() để log optical_flow data

---

## 📂 Files đã chỉnh sửa

### 1. [main.c](e:\Multiple_Mobile_Robot\mini_server\src\main.c)
**Thêm:**
- Global variables: `g_optical_flow`, `g_optical_mutex`
- External references: `g_ekf`, `g_ekf_mutex`
- Gọi thread: `pthread_create(&th_optical_flow, NULL, optical_flow_uart_thread, NULL)`
- Thread định nghĩa trong optical_flow_integration.c

**Cấu trúc thread:**
```c
pthread_t th_server, th_laptop_server, th_localize, th_optical_flow;
```

### 2. [optical_flow_integration.c](e:\Multiple_Mobile_Robot\mini_server\src\optical_flow_integration.c)
**Chức năng:**
- Thread function: `optical_flow_uart_thread()` đọc UART 50Hz
- Xử lý LPF + coordinate transform
- **Tích phân position từ velocity**
- Position ban đầu từ localization (quality > 50)
- Update EKF với velocity
- Export: `optical_flow_get_position()`, `optical_flow_set_initial_position()`

### 3. [json_handler.c](e:\Multiple_Mobile_Robot\mini_server\src\json_handler.c)
**Thay đổi:**
- Include: `#include "optical_flow.h"`
- External variables: `extern optical_flow_t g_optical_flow;`
- Function: `send_optical_flow_position_to_laptop()` (thay thế bno055)
- Parse logic: Gửi **optical flow position (đã tích phân) + velocity**
- Localization: Khởi tạo optical flow position khi quality > 50
- Database log: optical_flow data vào cột bno055

### 4. [json_handler.h](e:\Multiple_Mobile_Robot\mini_server\inc\json_handler.h)
**Thêm:**
```c
void send_optical_flow_position_to_laptop(float pos_x, float pos_y, float vel_x, float vel_y);
```

### 5. [optical_flow.h](e:\Multiple_Mobile_Robot\mini_server\inc\optical_flow.h)
**Thêm:**
```c
void *optical_flow_uart_thread(void *arg);
void optical_flow_set_initial_position(double x, double y);
void optical_flow_get_position(double *x, double *y, double *vx, double *vy);
```

### 6. [db_manager.c](e:\Multiple_Mobile_Robot\mini_server\database\db_manager.c)
**Comment:**
- Cột `bno055_x/y/vx/vy` giờ lưu **OPTICAL FLOW data** (đã tích phân position)

---

## 🔄 Luồng dữ liệu mới

```
┌─────────────────────────────────────────────────────────────────┐
│                     OPTICAL FLOW PIPELINE                        │
└─────────────────────────────────────────────────────────────────┘

MTF-02 Sensor (50Hz)
    ↓
/dev/ttyTHS0 UART
    ↓
optical_flow_uart_thread() [optical_flow_integration.c]
    ├─ optical_flow_process() → LPF + Transform
    ├─ TÍCH PHÂN POSITION: pos += vel × dt  ← MỚI!
    ├─ ekf_update_sensor(OPTICAL_FLOW) → EKF velocity update
    └─ Store: (pos_x, pos_y, vel_x, vel_y)
    ↓
┌─────────────────────────────────────────────────────────────────┐
│              POSITION INITIALIZATION (1 lần)                     │
└─────────────────────────────────────────────────────────────────┘

Localization data arrives (quality > 50)
    ↓
optical_flow_set_initial_position(loc_x, loc_y)  ← KHỞI TẠO
    └─ of_pos_x = loc_x, of_pos_y = loc_y
    └─ Bắt đầu tích phân từ vị trí này

┌─────────────────────────────────────────────────────────────────┐
│                   DATA SEND & LOG                                │
└─────────────────────────────────────────────────────────────────┘

BNO055 IMU data (25Hz) → parse_json_message() [json_handler.c]
    ├─ optical_flow_get_position() → (x, y, vx, vy)
    ├─ Create position_data_t with source="optical_flow"
    ├─ json_handler_push_position()
    │     └─ Database: bno055_x/y/vx/vy = optical flow data  ← THAY THẾ
    └─ send_optical_flow_position_to_laptop()
    ↓
Laptop Server receives:
{
  "id": "robot1",
  "type": "position", 
  "source": "optical_flow",
  "data": {
    "position": [x, y],        ← OPTICAL FLOW (tích phân)
    "velocity": [vx, vy]       ← OPTICAL FLOW (filtered)
  }
}
```

---

## ⚙️ Cấu hình UART

- **Port**: `/dev/ttyTHS0`
- **Baud rate**: 115200
- **Header**: `0xEF 0x0F 0x?? 0x51` (Micolink MTF-02)
- **Data offset**: 
  - vx_raw: byte[18-19]
  - vy_raw: byte[20-21]
  - quality: byte[22]

---

## 🎯 Các điểm quan trọng

### 1. Thread Safety
- Sử dụng `g_optical_mutex` khi truy cập `g_optical_flow`
- Sử dụng `g_ekf_mutex` khi update EKF
- Không có race condition giữa threads

### 2. Coordinate System
- Raw data: Optical Flow frame (y ngược chiều)
- → Body frame (vy đổi dấu)
- → LPF filtering
- → Global frame (rotation bằng theta)

### 3. Data Flow
- Optical Flow thread: Update EKF velocity + tích phân position (50Hz)
- JSON handler: Gửi position (đã tích phân) + velocity lên server
- Position initialization: Từ localization khi quality > 50
- **Database: Cột bno055 = Optical Flow data (KHÔNG CÒN BNO055)**

---

## 🧪 Testing Checklist

- [ ] Compile thành công
- [ ] Thread optical_flow khởi động
- [ ] UART đọc được data từ MTF-02
- [ ] LPF hoạt động (velocity smooth)
- [ ] EKF nhận được velocity update
- [ ] Laptop nhận được JSON với source="optical_flow"
- [ ] Database log optical_flow data đúng
- [ ] Không có memory leak hoặc deadlock

---

## 📊 So sánh Before/After

### Before:
- BNO055: tích phân acceleration → drift lớn, không chính xác
- Position source: "bno055"
- Database: bno055_x/y/vx/vy lưu IMU acceleration data

### After:
- **Optical Flow**: đo velocity trực tiếp → chính xác, ít drift
- **Position**: tích phân từ optical flow velocity, khởi tạo từ localization
- JSON source: "optical_flow"
- Database: **bno055_x/y/vx/vy = Optical Flow data** (thay thế hoàn toàn)
- EKF được update velocity bởi optical flow 50Hz

---

## 🚨 Lưu ý

1. **HOÀN TOÀN THAY THẾ BNO055**: 
   - Không còn dùng BNO055 acceleration cho position
   - Database cột `bno055_x/y/vx/vy` = **Optical Flow data**
   
2. **Position Initialization**:
   - Optical flow cần localization ổn định (quality > 50) để khởi tạo
   - Trước khi khởi tạo: position = (0, 0)
   - Sau khi khởi tạo: tích phân liên tục từ velocity

3. **Database schema không đổi**: 
   - Vẫn dùng cột bno055, nhưng **data là optical flow**
   - BNO055 IMU chỉ còn dùng cho heading (theta)

4. **Cần tuning**: 
   - Scale factor optical flow: `0.000122`
   - R noise: `0.05²` m/s
   - Quality threshold: `10`
   - Localization quality init: `> 50`

5. **UART port**: Đảm bảo `/dev/ttyTHS0` có permission đúng

---

**Status**: ✅ Hoàn thành tích hợp
**Test Required**: Compile và test trên hardware
