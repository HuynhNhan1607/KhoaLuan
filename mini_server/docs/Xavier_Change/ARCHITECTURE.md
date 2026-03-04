# KIẾN TRÚC HỆ THỐNG SAU KHI TÍCH HỢP OPTICAL FLOW

## 🏗️ Cấu trúc Module

```
┌─────────────────────────────────────────────────────────────────┐
│                          main.c                                  │
├─────────────────────────────────────────────────────────────────┤
│  • Khởi tạo global: g_optical_flow, g_optical_mutex            │
│  • Khởi động threads:                                           │
│    - server_thread                                              │
│    - laptop_server_thread                                       │
│    - localize_thread                                            │
│    - optical_flow_uart_thread ← GỌI TỪ ĐÂY                     │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│              optical_flow_integration.c                          │
├─────────────────────────────────────────────────────────────────┤
│  void *optical_flow_uart_thread(void *arg)                     │
│  {                                                              │
│    1. Mở UART /dev/ttyTHS0 @ 115200 baud                       │
│    2. Loop 50Hz:                                                │
│       - Đọc raw data (vx_raw, vy_raw, quality)                 │
│       - Lấy theta từ EKF                                        │
│       - optical_flow_process() → LPF + Transform               │
│       - Tích phân position: pos += vel × dt                     │
│       - ekf_update_sensor(OPTICAL_FLOW, velocity)              │
│  }                                                              │
│                                                                 │
│  void optical_flow_set_initial_position(x, y)                  │
│  {                                                              │
│    • Khởi tạo of_pos_x, of_pos_y                               │
│    • Enable tích phân position                                  │
│  }                                                              │
│                                                                 │
│  void optical_flow_get_position(x, y, vx, vy)                 │
│  {                                                              │
│    • Trả về position đã tích phân + velocity                   │
│  }                                                              │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│                  optical_flow.c                                  │
├─────────────────────────────────────────────────────────────────┤
│  • LPF Order 2 (2Hz cutoff @ 50Hz)                             │
│  • Coordinate transform: Optical → Body → Global               │
│  • Filter state management                                      │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│                       ekf.c                                      │
├─────────────────────────────────────────────────────────────────┤
│  • ekf_update_vel_vxvy() - Update velocity state               │
│  • State: [x, y, vx, vy, theta]                                │
│  • Mahalanobis gating (chi2 = 7.38)                            │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│                   json_handler.c                                 │
├─────────────────────────────────────────────────────────────────┤
│  BNO055 data arrives (25Hz):                                    │
│  {                                                              │
│    1. optical_flow_get_position() → (x, y, vx, vy)             │
│    2. Create position_data_t:                                   │
│       • source = "optical_flow"                                 │
│       • pos_x/y = optical flow (tích phân)                      │
│       • vel_x/y = optical flow (filtered)                       │
│    3. json_handler_push_position()                              │
│    4. send_optical_flow_position_to_laptop()                    │
│  }                                                              │
│                                                                 │
│  Localization data (quality > 50):                              │
│  {                                                              │
│    optical_flow_set_initial_position(loc_x, loc_y)  ← INIT    │
│  }                                                              │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│                      db_manager.c                                │
├─────────────────────────────────────────────────────────────────┤
│  position_logs:                                                  │
│  • bno055_x  = OPTICAL FLOW pos_x (tích phân)                   │
│  • bno055_y  = OPTICAL FLOW pos_y (tích phân)                   │
│  • bno055_vx = OPTICAL FLOW vel_x (filtered)                    │
│  • bno055_vy = OPTICAL FLOW vel_y (filtered)                    │
│                                                                 │
│  ⚠️  BNO055 KHÔNG CÒN ĐƯỢC DÙNG CHO POSITION                   │
└─────────────────────────────────────────────────────────────────┘
```

---

## 🔄 Luồng dữ liệu chi tiết

### 1. Optical Flow Data Processing (50Hz)

```
MTF-02 Sensor
    │
    ├─ Raw: vx_raw, vy_raw (int16_t), quality (uint8_t)
    │
    ↓
optical_flow_uart_thread()
    │
    ├─ [1] optical_flow_process()
    │     • Scale: × 0.000122 → m/s
    │     • Transform: Optical → Body (vy = -vy)
    │     • LPF Order 2: 2Hz cutoff
    │     • Rotation: Body → Global (theta từ EKF)
    │     → vx_global, vy_global
    │
    ├─ [2] Position Integration (if initialized)
    │     • dt = now - last_time
    │     • of_pos_x += vx_global × dt
    │     • of_pos_y += vy_global × dt
    │
    ├─ [3] EKF Update
    │     • ekf_update_sensor(OPTICAL_FLOW, [vx, vy])
    │     • H = [0 0 1 0 0; 0 0 0 1 0]
    │     • Update vx, vy state only
    │
    └─ Store: (of_pos_x, of_pos_y, vx_global, vy_global)
```

### 2. Position Initialization (1 lần khi localization ổn định)

```
Localization Data (1Hz)
    │
    ├─ quality check: > 50 ?
    │     YES ↓
    │
optical_flow_set_initial_position(loc_x, loc_y)
    │
    ├─ of_pos_x = loc_x
    ├─ of_pos_y = loc_y
    ├─ of_position_initialized = true
    └─ Reset integration timer
```

### 3. Data Transmission & Logging (25Hz - theo BNO055)

```
BNO055 IMU Trigger (25Hz)
    │
    ├─ Update EKF heading (theta)
    │
    ↓
optical_flow_get_position()
    │
    ├─ Return: (of_pos_x, of_pos_y, vx, vy)
    │
    ↓
Create JSON & Push to Queue
    │
    ├─ source: "optical_flow"
    ├─ position: [x, y]  ← Tích phân
    ├─ velocity: [vx, vy] ← Filtered
    │
    ↓
┌─────────────────────────────────────┐
│  send_to_upstream_server()          │
│  → Laptop nhận JSON                 │
└─────────────────────────────────────┘
    │
┌─────────────────────────────────────┐
│  log_position() → SQLite            │
│  → bno055_x/y/vx/vy = Optical Flow  │
└─────────────────────────────────────┘
```

---

## 📊 Data Rates

| Component           | Rate  | Function                      |
| ------------------- | ----- | ----------------------------- |
| MTF-02 Sensor       | 50Hz  | Raw velocity measurement      |
| Optical Flow Thread | 50Hz  | LPF + Transform + Integration |
| EKF Velocity Update | 50Hz  | Update vx, vy state           |
| BNO055 IMU          | 25Hz  | Heading (theta) update        |
| JSON Transmission   | 25Hz  | Send position + velocity      |
| Database Logging    | 500ms | Batch write to SQLite         |
| UWB Localization    | 1Hz   | Position correction + OF init |

---

## 🎯 Sensor Fusion

```
┌─────────────────────────────────────────────────────────┐
│                  EKF STATE VECTOR                        │
│              [x, y, vx, vy, theta]                       │
└─────────────────────────────────────────────────────────┘
            ↑         ↑      ↑         ↑
            │         │      │         │
    ┌───────┘         │      │         └─────────┐
    │                 │      │                   │
┌───────┐      ┌──────────┐ ┌──────────────┐ ┌──────┐
│ UWB   │      │ Optical  │ │  Encoder     │ │ IMU  │
│ 1Hz   │      │ Flow 50Hz│ │  10Hz        │ │ 25Hz │
└───────┘      └──────────┘ └──────────────┘ └──────┘
   x, y           vx, vy       predict input   theta
 (correct)       (update)      (propagate)    (correct)
```

**Priority:**
1. **IMU (25Hz)**: Heading - độ chính xác cao, update liên tục
2. **Optical Flow (50Hz)**: Velocity - trực tiếp, không drift
3. **Encoder (10Hz)**: Velocity predict - có thể bị wheel slip
4. **UWB (1Hz)**: Position - correct drift, khởi tạo OF position

---

## ⚙️ Tuning Parameters

### Optical Flow
```c
// optical_flow.c
#define OPTICAL_FLOW_MIN_QUALITY 10
const double SCALE = 0.000122;  // Raw → m/s

// LPF 2Hz @ 50Hz
#define LPF_A1  1.647460
#define LPF_A2 -0.700897
#define LPF_B0  0.013359
#define LPF_B1  0.026718
#define LPF_B2  0.013359
```

### EKF
```c
// ekf.c - ekf_update_sensor()
case EKF_SENSOR_OPTICAL_FLOW:
    R_vx = 0.05 * 0.05;     // Measurement noise
    R_vy = 0.05 * 0.05;
    chi2_gate = 7.38;       // 97.5% confidence
```

### Position Init
```c
// json_handler.c
if (pos.quality > 50.0f)    // Localization quality threshold
{
    optical_flow_set_initial_position(pos.pos_x, pos.pos_y);
}
```

---

## ✅ Advantages

1. **High-frequency velocity**: 50Hz vs 1Hz UWB
2. **Direct measurement**: Không tích phân như IMU → ít drift
3. **Position estimation**: Tích phân từ velocity với khởi tạo từ UWB
4. **Redundancy**: Backup nếu UWB mất tín hiệu
5. **Smooth trajectory**: LPF 2Hz lọc nhiễu cao tần

---

## 🚨 Critical Points

1. **Cần khởi tạo position**: Optical flow PHẢI có localization ổn định trước
2. **Database naming**: Cột "bno055" giờ chứa optical flow (legacy naming)
3. **BNO055 chỉ còn IMU**: Không dùng acceleration cho position nữa
4. **Thread safety**: Mutex khi truy cập `g_optical_flow`, `g_ekf`
5. **Integration drift**: Position sẽ drift theo thời gian, cần UWB correct

---

**Status**: ✅ Cấu trúc hoàn chỉnh, ready to test
