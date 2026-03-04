# Virtual Structure Control - Hệ Thống 3 Robots Gắp Vật

## Kiến Trúc Hệ Thống

```
┌─────────────────────────────────────────────────────────────────────────┐
│                              LAPTOP                                      │
│                         (Path Planner)                                   │
│                                                                          │
│   • Có bản đồ môi trường                                                │
│   • Biết vị trí/kích thước vật cần gắp                                  │
│   • CHỈ TÍNH 2 LẦN:                                                     │
│     - Pha 1: Đường đi từ vị trí hiện tại → vật                          │
│     - Pha 2: Quỹ đạo từ vật → đích                                      │
│                                                                          │
└────────────────────────┬────────────────────────────────────────────────┘
                         │ Gửi quỹ đạo (1 lần mỗi pha)
         ┌───────────────┼───────────────┐
         │               │               │
         ▼               ▼               ▼
┌─────────────┐   ┌─────────────┐   ┌─────────────┐
│   ROBOT 1   │   │   ROBOT 2   │   │   ROBOT 3   │
│  Xavier+ESP │◄──►│  Xavier+ESP │◄──►│  Xavier+ESP │
│             │   │             │   │             │
│ Virtual     │   │ Virtual     │   │ Virtual     │
│ Structure + │   │ Structure + │   │ Structure + │
│ Consensus   │   │ Consensus   │   │ Consensus   │
└─────────────┘   └─────────────┘   └─────────────┘
       ↑                 ↑                 ↑
       └─────────────────┴─────────────────┘
         Event-Triggered Communication (ETC)
```

---

## Khái Niệm Cốt Lõi

### 1. Virtual Structure (Cấu Trúc Ảo)

**Định nghĩa**: Coi toàn bộ formation như một "vật thể cứng ảo" di chuyển trong không gian.

```
┌─────────────────────────────────────────────────┐
│              VIRTUAL STRUCTURE                   │
│                                                  │
│              Robot 1 (offset₁)                   │
│                   ▲                              │
│                  /|\                             │
│                 / | \                            │
│                /  ●  \  ← Centroid (tâm ảo)     │
│               /       \                          │
│              ▼         ▼                         │
│         Robot 2     Robot 3                      │
│        (offset₂)   (offset₃)                     │
│                                                  │
│   Cả cấu trúc di chuyển như MỘT KHỐI            │
└─────────────────────────────────────────────────┘
```

**Nguyên lý**:
- **Centroid** = Tâm hình học của formation = `(pos₁ + pos₂ + pos₃) / 3`
- Mỗi robot duy trì **offset cố định** so với centroid
- Khi centroid di chuyển theo quỹ đạo, toàn bộ formation di chuyển theo

**Công thức Virtual Structure**:
```
// Tính centroid từ vị trí thực 3 robot
centroid = (robot1.pos + robot2.pos + robot3.pos) / 3

// Vị trí lý tưởng của robot i
ideal_pos[i] = centroid + formation_offset[i]

// Sai số formation
formation_error[i] = |robot[i].pos - ideal_pos[i]|
```

---

### 2. Consensus (Đồng thuận)

**Định nghĩa**: Mỗi robot điều chỉnh trạng thái của mình dựa trên thông tin từ láng giềng để đạt được sự đồng thuận về centroid.

```
┌────────────────────────────────────────────────────────────────┐
│                     CONSENSUS PROTOCOL                          │
│                                                                  │
│   Robot i nhận thông tin từ láng giềng j, k                     │
│                                                                  │
│   predicted_centroid[i] = (my_pos + neighbor_j.pos + neighbor_k.pos) / 3
│                                                                  │
│   Điều kiện đồng thuận:                                         │
│   • Tất cả robot có CÙNG ước tính về centroid                   │
│   • Mỗi robot đồng ý về vị trí tương đối với nhau               │
│                                                                  │
│   Nếu có bất đồng (do nhiễu, delay):                            │
│   • Mỗi robot điều chỉnh velocity để giảm sai số                │
│   • Hội tụ dần về trạng thái đồng thuận                         │
└────────────────────────────────────────────────────────────────┘
```

**Công thức Consensus**:
```
// Robot i tính toán:

// 1. Centroid dựa trên thông tin nhận được
measured_centroid = (my_measured_pos + neighbor1_pos + neighbor2_pos) / 3

// 2. Vị trí lý tưởng của mình
my_target = measured_centroid + my_offset

// 3. Sai số vị trí
position_error = my_target - my_measured_pos

// 4. Velocity feedback (consensus term)
v_consensus = KP * position_error

// 5. Nếu error lớn, tăng gain để hội tụ nhanh hơn
if |position_error| > FORMATION_SPACING:
    v_consensus = 2.0 * KP * position_error
```

---

### 3. Event-Triggered Communication (ETC)

**Định nghĩa**: Robot CHỈ broadcast trạng thái khi cần thiết, thay vì broadcast liên tục.

```
┌────────────────────────────────────────────────────────────────┐
│              EVENT-TRIGGERED COMMUNICATION                       │
│                                                                  │
│   Thay vì: Gửi vị trí mỗi 50ms (Time-Triggered)                │
│   ETC: Gửi CHỈ KHI prediction error > threshold                 │
│                                                                  │
│   Lợi ích:                                                       │
│   • Giảm băng thông truyền thông 40-60%                         │
│   • Giảm tải xử lý                                               │
│   • Vẫn đảm bảo độ chính xác formation                          │
└────────────────────────────────────────────────────────────────┘
```

**Cơ chế Predictor-Corrector**:

```
┌─────────────────────────────────────────────────────────────────┐
│                 PREDICTOR-CORRECTOR SCHEME                       │
│                                                                  │
│   Mỗi robot lưu trữ:                                             │
│   ├── last_broadcast_pos    (vị trí lúc broadcast gần nhất)     │
│   ├── last_broadcast_vel    (velocity lúc broadcast)            │
│   └── last_broadcast_time   (thời điểm broadcast)               │
│                                                                  │
│   Láng giềng DỰ ĐOÁN vị trí của robot:                          │
│   predicted_pos = last_broadcast_pos + last_broadcast_vel × Δt  │
│                                                                  │
│   Robot so sánh:                                                 │
│   prediction_error = |measured_pos - predicted_pos|             │
│                                                                  │
│   TRIGGER CONDITION:                                             │
│   if prediction_error > TRIGGER_THRESHOLD:                       │
│       broadcast(current_pos, current_vel)  ← GỬI                │
│   else:                                                          │
│       // Không gửi, láng giềng dùng prediction                  │
└─────────────────────────────────────────────────────────────────┘
```

**Điều kiện Event-Trigger**:
```
// Tham số
TRIGGER_THRESHOLD = 0.05  // 5cm

// Tính predicted position (dead reckoning)
dt = current_time - last_broadcast_time
predicted_pos = last_broadcast_pos + last_broadcast_vel * dt

// Tính prediction error
prediction_error = |measured_pos - predicted_pos|

// Điều kiện trigger
if prediction_error > TRIGGER_THRESHOLD:
    // BROADCAST ngay lập tức
    broadcast(measured_pos, current_vel, current_time)
    
    // Cập nhật state
    last_broadcast_pos = measured_pos
    last_broadcast_vel = current_vel
    last_broadcast_time = current_time
```

---

## 2 Pha Hoạt Động

### Pha 1: Tiếp Cận Vật (Approach)

```
Điều kiện: 3 robot ở vị trí BẤT KỲ, chưa gắp vật

Laptop tính:
├── Vị trí vật cần gắp (object_pos)
├── Kích thước vật (object_size)
├── Vị trí formation xung quanh vật
│   ├── Robot 1: grip_pos_1 (phía trên)
│   ├── Robot 2: grip_pos_2 (phía trái)
│   └── Robot 3: grip_pos_3 (phía phải)
└── Đường đi cho mỗi robot: current_pos → grip_pos

Laptop gửi cho mỗi robot:
├── Quỹ đạo riêng tới vị trí gắp
└── (Hoặc chỉ điểm đích grip_pos)

Robot tự điều khiển di chuyển đến vị trí gắp
(Pha này CÓ THỂ không cần Virtual Structure)
```

### Pha 2: Vận Chuyển Vật (Transport) - ÁP DỤNG VIRTUAL STRUCTURE

```
Điều kiện: 3 robot ĐÃ GẮP vật thành công

Laptop tính:
├── Quỹ đạo của VẬT từ vị trí hiện tại → đích
│   (Đây là quỹ đạo của CENTROID)
└── Gửi 1 LẦN cho tất cả robot

Mỗi Robot (Xavier) tự xử lý:
├── Nhận quỹ đạo centroid từ Laptop
├── Áp dụng VIRTUAL STRUCTURE:
│   ├── Nội suy centroid_target tại thời điểm t
│   └── my_target = centroid_target + my_offset
├── Áp dụng CONSENSUS:
│   ├── Nhận vị trí láng giềng (qua ETC)
│   ├── Tính measured_centroid từ 3 vị trí thực
│   └── Điều chỉnh velocity để giảm formation error
├── Áp dụng EVENT-TRIGGERED:
│   ├── So sánh |measured_pos - predicted_pos|
│   └── Broadcast nếu vượt threshold
└── Gửi velocity command xuống ESP32
```

---

## Công Thức Điều Khiển Tổng Hợp (Trên Xavier)

### Vòng Lặp Chính (100Hz)

```
// ═══════════════════════════════════════════════════════
// BƯỚC 1: CẬP NHẬT SENSOR (20Hz, mỗi 50ms)
// ═══════════════════════════════════════════════════════
if (time_since_last_sensor >= 0.05):
    measured_pos = get_position_from_EKF()
    
    // EVENT-TRIGGERED: Kiểm tra có cần broadcast không
    predicted_pos = last_broadcast_pos + last_broadcast_vel * dt
    prediction_error = |measured_pos - predicted_pos|
    
    if prediction_error > TRIGGER_THRESHOLD:  // 5cm
        broadcast_to_neighbors(measured_pos, velocity, current_time)
        last_broadcast_pos = measured_pos
        last_broadcast_vel = velocity
        last_broadcast_time = current_time

// ═══════════════════════════════════════════════════════
// BƯỚC 2: NHẬN THÔNG TIN LÁNG GIỀNG
// ═══════════════════════════════════════════════════════
neighbor1_pos = receive_from_neighbor1()  // Có thể là broadcast mới hoặc prediction
neighbor2_pos = receive_from_neighbor2()

// Nếu không có broadcast mới, dùng prediction
if no_new_broadcast_from_neighbor1:
    neighbor1_pos = neighbor1_last_pos + neighbor1_last_vel * dt_since_broadcast

// ═══════════════════════════════════════════════════════
// BƯỚC 3: VIRTUAL STRUCTURE - Tính centroid
// ═══════════════════════════════════════════════════════
measured_centroid = (my_measured_pos + neighbor1_pos + neighbor2_pos) / 3

// ═══════════════════════════════════════════════════════
// BƯỚC 4: Nội suy target từ quỹ đạo (nhận từ Laptop)
// ═══════════════════════════════════════════════════════
centroid_target = interpolate_trajectory(centroid_trajectory, current_time)
path_direction = get_direction_at(centroid_trajectory, current_time)

// ═══════════════════════════════════════════════════════
// BƯỚC 5: CONSENSUS - Tính target và error
// ═══════════════════════════════════════════════════════
// Target của tôi = centroid hiện tại + offset của tôi + feedforward
my_target = measured_centroid + my_offset
my_target += VELOCITY_NOMINAL * path_direction * ff_scale * DT * 10

// Sai số vị trí
position_error = my_target - my_measured_pos
error_magnitude = |position_error|

// Adaptive gain (consensus với gain thích ứng)
if error_magnitude > FORMATION_SPACING:
    kp_effective = KP_POSITION * 2.0  // Tăng gain khi lệch nhiều
else:
    kp_effective = KP_POSITION

// ═══════════════════════════════════════════════════════
// BƯỚC 6: Tính velocity command
// ═══════════════════════════════════════════════════════
// Feedforward: theo hướng quỹ đạo
dist_to_target = |measured_centroid - centroid_target|
ff_scale = clip(dist_to_target / 0.5, 0.0, 1.0)
v_feedforward = VELOCITY_NOMINAL * path_direction * ff_scale

// Feedback: sửa lỗi formation (consensus term)
v_feedback = kp_effective * position_error

// Kết hợp
v_cmd = v_feedforward + v_feedback

// Giới hạn tốc độ
if |v_cmd| < VELOCITY_DEADZONE:  // 2cm/s
    v_cmd = (0, 0)
else if |v_cmd| > VELOCITY_MAX:  // 30cm/s
    v_cmd = normalize(v_cmd) * VELOCITY_MAX

// ═══════════════════════════════════════════════════════
// BƯỚC 7: Gửi xuống ESP32
// ═══════════════════════════════════════════════════════
send_velocity_to_esp32(v_cmd.x, v_cmd.y)
```

---

## Tham Số Cần Cấu Hình

| Tên | Giá trị | Mô tả |
|-----|---------|-------|
| **Virtual Structure** |||
| `FORMATION_OFFSET[0]` | (0.0, 0.4) | Offset Robot 1 (đỉnh) |
| `FORMATION_OFFSET[1]` | (-0.35, -0.2) | Offset Robot 2 (trái) |
| `FORMATION_OFFSET[2]` | (0.35, -0.2) | Offset Robot 3 (phải) |
| `FORMATION_SPACING` | 0.4 m | Khoảng cách robot-centroid |
| **Consensus** |||
| `KP_POSITION` | 2.5 | Hệ số P điều khiển |
| `ADAPTIVE_GAIN_MULTIPLIER` | 2.0 | Nhân KP khi error lớn |
| **Event-Triggered** |||
| `TRIGGER_THRESHOLD` | 0.05 m | Ngưỡng trigger (5cm) |
| `SENSOR_UPDATE_PERIOD` | 0.05 s | Chu kỳ cập nhật sensor (50ms) |
| **Velocity** |||
| `VELOCITY_NOMINAL` | 0.2 m/s | Tốc độ cơ bản |
| `VELOCITY_MAX` | 0.3 m/s | Tốc độ tối đa |
| `VELOCITY_DEADZONE` | 0.02 m/s | Ngưỡng dừng |
| **Timing** |||
| `DT` | 0.01 s | Chu kỳ điều khiển (10ms = 100Hz) |

---

## Giao Tiếp Event-Triggered Giữa 3 Robot

### Message Format (Broadcast)
```json
{
    "robot_id": 1,
    "pos_x": 2.51,
    "pos_y": 1.83,
    "vel_x": 0.15,
    "vel_y": 0.02,
    "timestamp": 12345678
}
```

### Xử Lý Khi Nhận Broadcast Từ Láng Giềng
```
on_receive_broadcast(neighbor_id, pos, vel, time):
    neighbor[neighbor_id].last_broadcast_pos = pos
    neighbor[neighbor_id].last_broadcast_vel = vel
    neighbor[neighbor_id].last_broadcast_time = time
```

### Dự Đoán Vị Trí Láng Giềng (Khi Không Có Broadcast Mới)
```
get_neighbor_position(neighbor_id, current_time):
    dt = current_time - neighbor[neighbor_id].last_broadcast_time
    predicted = neighbor[neighbor_id].last_broadcast_pos 
              + neighbor[neighbor_id].last_broadcast_vel * dt
    return predicted
```

---

## Những Gì Cần Thêm Mới Vào Project

### Trên XAVIER (mỗi robot)

| Module | Thêm mới | Mô tả |
|--------|----------|-------|
| **Virtual Structure** | `formation_offsets[3]` | Offset của cả 3 robot |
|| `compute_centroid()` | Tính tâm từ 3 vị trí |
|| `centroid_trajectory[]` | Quỹ đạo tâm (nhận từ Laptop) |
|| `interpolate_trajectory()` | Nội suy điểm trên quỹ đạo |
| **Consensus** | `compute_formation_velocity()` | Tính velocity với feedback |
|| `check_formation_error()` | Kiểm tra sai số formation |
|| Adaptive gain logic | Tăng KP khi error lớn |
| **Event-Triggered** | `last_broadcast_pos/vel/time` | Trạng thái broadcast gần nhất |
|| `neighbor_broadcast_state[2]` | Trạng thái broadcast của láng giềng |
|| `check_trigger_condition()` | Kiểm tra điều kiện ETC |
|| `broadcast_state()` | Gửi trạng thái cho láng giềng |
|| `predict_neighbor_pos()` | Dự đoán vị trí láng giềng |

### Trên ESP32
- **Không cần thay đổi**: Vẫn nhận `(vx, vy)` và thực thi

---

## Tóm Tắt Luồng Xử Lý

```
═══════════════════════════════════════════════════════════════
│                   VIRTUAL STRUCTURE                          │
│   Centroid = trung bình vị trí 3 robot                       │
│   Mỗi robot giữ offset cố định với centroid                  │
═══════════════════════════════════════════════════════════════
                         ↓
═══════════════════════════════════════════════════════════════
│                      CONSENSUS                               │
│   Mỗi robot tính centroid từ thông tin láng giềng            │
│   Điều chỉnh velocity để giảm formation error                │
│   Hội tụ về trạng thái đồng thuận                            │
═══════════════════════════════════════════════════════════════
                         ↓
═══════════════════════════════════════════════════════════════
│               EVENT-TRIGGERED COMM                           │
│   CHỈ broadcast khi |measured - predicted| > 5cm             │
│   Láng giềng dùng prediction khi không có broadcast mới      │
│   Tiết kiệm 40-60% băng thông                                │
═══════════════════════════════════════════════════════════════
```

---

## Điều Kiện An Toàn

| Điều kiện | Hành động |
|-----------|-----------|
| Formation error > 10cm | Giảm tốc độ (`ff_scale *= 0.5`) |
| Formation error > 20cm | Dừng lại, cảnh báo |
| Prediction error liên tục cao | Có thể láng giềng gặp sự cố |
| Mất broadcast từ láng giềng > 500ms | Dừng lại, cảnh báo |

---

## Một Dòng Tóm Tắt

> **Virtual Structure định nghĩa hình dạng, Consensus duy trì sự đồng thuận, Event-Triggered tiết kiệm truyền thông — 3 robot phối hợp phân tán để giữ vật không rơi.**
