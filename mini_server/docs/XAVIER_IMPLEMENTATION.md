# 📋 XAVIER Design Specification - Distributed 2-Robot Control

> **Phiên bản:** 2.1  
> **Ngày cập nhật:** 2026-01-16  
> **Cấu hình:** 2 robots vận chuyển vật thể

---

## 📌 Tổng Quan

Hệ thống điều khiển phân tán cho **2 robots** kết hợp **3 phương pháp cốt lõi**:

| Phương Pháp | Vai Trò |
|-------------|---------|
| **Virtual Structure** | Định nghĩa vị trí lý tưởng từ centroid + offset |
| **Consensus** | Đồng thuận về vị trí formation thực tế (2 robots) |
| **Event-Triggered** | Broadcast khi sai lệch > threshold |

```
┌─────────────────────────────────────────────────────────────────┐
│                        LAPTOP                                    │
│                  (Path Planner + Relay)                          │
│                                                                  │
│   • Gửi trajectory 1 lần mỗi pha                                │
│   • Relay message giữa 2 robot                                   │
│   • KHÔNG điều khiển real-time                                   │
└────────────────────────┬────────────────────────────────────────┘
                         │
           ┌─────────────┴─────────────┐
           ▼                           ▼
    ┌─────────────┐             ┌─────────────┐
    │   ROBOT 1   │◄───────────►│   ROBOT 2   │
    │   XAVIER    │   Event-    │   XAVIER    │
    │   + ESP32   │  Triggered  │   + ESP32   │
    └─────────────┘             └─────────────┘
```

---

# PHẦN I: BA PHƯƠNG PHÁP CỐT LÕI

## 1. Virtual Structure (2 Robots)

### Định Nghĩa
Formation 2 robot = **khối cứng ảo** với centroid ở giữa vật thể.

### Formation Offsets

```
           Robot 1
              ▲
              │ offset_1 = (0, +d)
              │
    ──────── ● ────────  Centroid (tâm vật thể)
              │
              │ offset_2 = (0, -d)
              ▼
           Robot 2
```

| Robot | Offset | Vị Trí |
|-------|--------|--------|
| 1 | (0, +d) | Phía trên vật |
| 2 | (0, -d) | Phía dưới vật |

Trong đó `d = object_radius + arm_length + gripper_length`

### Công Thức

```
P_robot_1(progress) = P_centroid(progress) + (0, +d)
P_robot_2(progress) = P_centroid(progress) + (0, -d)
```

---

## 2. Consensus (2 Robots)

### Định Nghĩa
Với 2 robot, consensus đơn giản hơn - centroid thực = trung điểm giữa 2 robot.

### Công Thức

```
measured_centroid = (robot1_actual_pos + robot2_actual_pos) / 2

my_target = measured_centroid + my_offset
```

### So Sánh

| Loại Centroid | Tính Từ | Mục Đích |
|---------------|---------|----------|
| **Trajectory Centroid** | Nội suy trajectory | Vị trí lý tưởng |
| **Measured Centroid** | (robot1 + robot2) / 2 | Điều khiển thực tế |

### Vai Trò
- Mỗi robot biết **vị trí thực** của formation
- Nếu 1 robot bị lệch → cả 2 điều chỉnh để giữ formation

---

## 3. Event-Triggered Communication (2 Robots)

### Định Nghĩa
Mỗi robot CHỈ broadcast cho robot kia khi cần thiết.

### Vị Trí Lý Tưởng vs Thực Tế

```
ideal_pos = centroid_trajectory(progress) + my_offset    ← Láng giềng có thể tự tính
actual_pos = EKF.get_position()                          ← Chỉ tôi biết
deviation = actual_pos - ideal_pos                        ← Sai lệch
```

### Điều Kiện Broadcast

| Điều Kiện | Broadcast? | Lý Do |
|-----------|------------|-------|
| \|deviation\| vượt 5cm (lần đầu) | ✅ CÓ | Láng giềng cần biết tôi lệch |
| deviation vẫn > 5cm | ❌ Không | Láng giềng đã biết |
| PID sửa, về dưới 3cm | ✅ CÓ | Láng giềng cần biết tôi đã sửa |
| Heartbeat > 500ms | ✅ CÓ | Xác nhận còn hoạt động |

### Nội Dung Broadcast

| Field | Mô Tả |
|-------|-------|
| `robot_id` | 1 hoặc 2 |
| `deviation_x, deviation_y` | Sai lệch so với lý tưởng |
| `is_corrected` | True nếu đã sửa xong |
| `progress` | Tiến độ hiện tại |

### Cách Láng Giềng Dự Đoán Vị Trí Của Tôi

```
NẾU chưa nhận broadcast:
    neighbor_pos = ideal_pos (từ trajectory + offset)
    
NẾU đã nhận broadcast:
    neighbor_pos = ideal_pos + neighbor_deviation
    
NẾU nhận broadcast "corrected":
    neighbor_pos = ideal_pos (deviation = 0)
```

---

# PHẦN II: CÁCH 3 PHƯƠNG PHÁP KẾT HỢP

## Luồng Xử Lý Mỗi 10ms (100Hz)

```
┌─────────────────────────────────────────────────────────────────┐
│ BƯỚC 1: VIRTUAL STRUCTURE - Tính vị trí lý tưởng               │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│   ideal_centroid = interpolate(trajectory, progress)            │
│   my_ideal = ideal_centroid + my_offset                          │
│   neighbor_ideal = ideal_centroid + neighbor_offset              │
│                                                                  │
└──────────────────────────┬──────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────────┐
│ BƯỚC 2: ĐỌC VỊ TRÍ THỰC TẾ                                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│   my_actual = EKF.get_position()                                 │
│   neighbor_actual = neighbor_ideal + neighbor_deviation          │
│                      (deviation từ broadcast, = 0 nếu chưa có)   │
│                                                                  │
└──────────────────────────┬──────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────────┐
│ BƯỚC 3: CONSENSUS - Tính measured centroid (2 robots)           │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│   measured_centroid = (my_actual + neighbor_actual) / 2          │
│   my_target = measured_centroid + my_offset                      │
│                                                                  │
└──────────────────────────┬──────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────────┐
│ BƯỚC 4: EVENT-TRIGGERED - Cần broadcast không?                  │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│   my_deviation = my_actual - my_ideal                            │
│                                                                  │
│   NẾU |my_deviation| > 5cm VÀ last_was_within:                   │
│       BROADCAST(deviation, progress)                             │
│                                                                  │
│   NẾU |my_deviation| < 3cm VÀ last_was_outside:                  │
│       BROADCAST(corrected=true)                                  │
│                                                                  │
└──────────────────────────┬──────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────────┐
│ BƯỚC 5: TÍNH VELOCITY                                            │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│   v_feedforward = VELOCITY × path_direction                      │
│   v_feedback = KP × (my_target - my_actual)                      │
│   v_total = v_feedforward + v_feedback                           │
│                                                                  │
└──────────────────────────┬──────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────────┐
│ BƯỚC 6: GỬI XUỐNG ESP32 (100Hz cố định)                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│   ESP32.send_velocity(v_total.x, v_total.y)                      │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## Ví Dụ Cụ Thể (2 Robots)

**Cấu hình:**
- Object radius = 0.10m
- Grip distance d = 0.40m
- Robot 1 offset = (0, +0.40)
- Robot 2 offset = (0, -0.40)

**Kịch bản: Robot 1 bị trượt 7cm**

| Thời điểm | Robot 1 | Robot 2 | Hành động |
|-----------|---------|---------|-----------|
| t=0 | ideal: (5.0, 4.4) | ideal: (5.0, 3.6) | Cả hai đi đúng |
| t=1 | actual: (5.07, 4.4) | actual: (5.0, 3.6) | R1 lệch 7cm |
| t=1 | deviation = 7cm > 5cm | - | **R1 BROADCAST** |
| t=1 | - | Nhận broadcast | R2 biết R1 lệch |
| t=1 | - | measured = (5.035, 4.0) | R2 tính centroid mới |
| t=1 | - | target = (5.035, 3.6) | R2 điều chỉnh |
| t=2 | PID sửa → actual: (5.01, 4.4) | - | R1 về gần ideal |
| t=2 | deviation = 1cm < 3cm | - | **R1 BROADCAST corrected** |
| t=2 | - | Nhận corrected | R2 dùng lại ideal |

---

# PHẦN III: THIẾT KẾ BỔ SUNG

## 1. Distance-Based Trajectory

```
progress = distance_traveled / total_path_length
progress ∈ [0.0, 1.0]
```

**Lợi ích:** Không phụ thuộc timestamp từ server

---

## 2. Progress Synchronization (2 Robots)

```
shared_progress = min(my_progress, neighbor_progress)
centroid_target = interpolate(trajectory, shared_progress)
```

**Lợi ích:** Robot nhanh chờ robot chậm, formation không bị kéo giãn

---

## 3. Deadlock Prevention

```
v_total = v_feedforward + v_feedback

v_feedforward = VELOCITY × direction    ← KHÔNG phụ thuộc láng giềng
v_feedback = KP × error                  ← Sửa formation
```

**Lợi ích:** Feedforward độc lập → robot luôn tiến, không dừng

---

# PHẦN IV: THAM SỐ & STATE

## Tham Số Thiết Kế

| Tham Số | Giá Trị | Mô Tả |
|---------|---------|-------|
| `VELOCITY_NOMINAL` | 0.2 m/s | Tốc độ tiến |
| `VELOCITY_MAX` | 0.3 m/s | Tốc độ tối đa |
| `KP_FORMATION` | 2.5 | Hệ số P feedback |
| `DEVIATION_THRESHOLD` | 0.05m | Ngưỡng broadcast |
| `CORRECTION_HYSTERESIS` | 0.03m | Ngưỡng "đã sửa" |
| `HEARTBEAT_PERIOD` | 500ms | Chu kỳ heartbeat |
| `FORMATION_ERROR_STOP` | 0.20m | Ngưỡng dừng khẩn |

## State Machine

| State | Mô Tả | ESP32 | Network |
|-------|-------|-------|---------|
| `IDLE` | Chờ trajectory | - | - |
| `EXECUTING` | Đang chạy | 100Hz | Event-Triggered |
| `FORMATION_STOP` | Error > 20cm | Stop | Alert |
| `COMPLETED` | Xong | Stop | Status |

---

# PHẦN V: TÓM TẮT

```
┌─────────────────────────────────────────────────────────────────┐
│                                                                  │
│   VIRTUAL STRUCTURE:  Robot_i = Centroid + Offset_i             │
│                       (2 robots đối diện nhau qua vật)          │
│                                                                  │
│   CONSENSUS:          Centroid = (Robot1 + Robot2) / 2          │
│                       (Đồng thuận về vị trí thực)               │
│                                                                  │
│   EVENT-TRIGGERED:    Broadcast khi |deviation| > 5cm           │
│                       Broadcast khi sửa xong (< 3cm)            │
│                                                                  │
│   → 2 ROBOTS PHỐI HỢP PHÂN TÁN, GIỮ VẬT KHÔNG RƠII              │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```
