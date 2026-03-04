# Arm Kinematic Module Integration

## Tổng quan

Module `arm_kinematic` được tạo mới để cung cấp tính toán động học (Forward/Inverse Kinematics) cho cánh tay robot 5-DOF, chạy trên Xavier. Code được port từ Python (`need_add/kinematics.py`) sang C.

---

## Files Đã Thêm

| File | Đường dẫn | Mô tả |
|------|-----------|-------|
| **Header** | `inc/arm_kinematic.h` | Declarations, structures, constants |
| **Source** | `src/arm_kinematic.c` | Implementation đầy đủ FK/IK |

---

## Không Sửa Đổi Files Hiện Tại

✅ Giữ nguyên toàn bộ cấu trúc code hiện tại trong `src/` và `inc/`  
✅ Không thay đổi bất kỳ file nào đang hoạt động

---

## Nội Dung Chi Tiết

### 1. Constants (từ `config.py`)

```c
#define ARM_D1      25.0    // Base height (mm)
#define ARM_A2      105.0   // Link 1 - Shoulder to Elbow (mm)
#define ARM_A3      65.0    // Link 2 - Elbow to Wrist (mm)
#define ARM_D5      180.0   // Tool length - Wrist to TCP (mm)
```

### 2. Structures

| Structure | Mục đích |
|-----------|----------|
| `ServoMapConfig` | Cấu hình mapping góc DH ↔ Servo (offset, direction) |
| `ArmAngles` | Kết quả IK: j0, j1, j2, j3, j4 (góc servo 0-180°) |
| `ArmPosition` | Kết quả FK: x, y, z, phi (vị trí TCP và góc pitch) |
| `FuzzyIKInfo` | Thông tin solution từ Fuzzy IK search |

### 3. Servo Mapping Configuration

```c
// joint_idx: 0=j0, 1=j1, ..., 5=j5
static const ServoMapConfig servo_config[6] = {
    {0.0,  1},  // j0: Base
    {90.0, 1},  // j1: Shoulder
    {90.0, 1},  // j2: Elbow
    {90.0, 1},  // j3: Wrist Pitch
    {0.0,  1},  // j4: Wrist Roll
    {0.0,  1}   // j5: Gripper
};
```

### 4. Functions

| Function | Input | Output | Mô tả |
|----------|-------|--------|-------|
| `arm_map_angle()` | joint_idx, math_angle | servo_angle | Chuyển góc DH → Servo |
| `arm_unmap_angle()` | joint_idx, servo_angle | math_angle | Chuyển góc Servo → DH |
| `arm_forward_kinematics()` | j0, j1, j2, j3 (servo) | ArmPosition* | Tính vị trí TCP từ góc servo |
| `arm_ik_exact()` | r, z, phi | j1*, j2*, j3* | IK chính xác trong mặt phẳng R-Z |
| `arm_ik_fuzzy()` | r, z, phi, tolerances | j1*, j2*, j3* | IK tìm kiếm xấp xỉ khi exact fail |
| `arm_ik_solve()` | x, y, z, phi, tolerances | ArmAngles* | **Entry point chính** - IK 5-DOF đầy đủ |
| `arm_ik_solve_simple()` | x, y, z, phi | ArmAngles* | IK với tolerance mặc định |

---

## Cách Sử Dụng

### Tính IK (Vị trí → Góc Servo)

```c
#include "arm_kinematic.h"

ArmAngles angles;
double x = 100.0, y = 50.0, z = 150.0, phi = 0.0;

if (arm_ik_solve_simple(x, y, z, phi, &angles)) {
    printf("J0=%.1f, J1=%.1f, J2=%.1f, J3=%.1f, J4=%.1f\n",
           angles.j0, angles.j1, angles.j2, angles.j3, angles.j4);
    // Gửi angles.j0..j4 đến ESP32 qua TCP
} else {
    printf("Unreachable position!\n");
}
```

### Tính FK (Góc Servo → Vị trí)

```c
ArmPosition pos;
double j0 = 90, j1 = 90, j2 = 135, j3 = 90;

if (arm_forward_kinematics(j0, j1, j2, j3, &pos)) {
    printf("TCP: X=%.1f, Y=%.1f, Z=%.1f, Phi=%.1f\n",
           pos.x, pos.y, pos.z, pos.phi);
}
```

---

## Workspace Constraints

```
         +Y
          ↑
    Q2    │    Q1
          │
  ───────●───────→ +X
         │
    Q3   │    Q4
         ↓

✅ Reachable: Y ≥ 0 (Q1, Q2)
❌ Unreachable: Y < 0 (Q3, Q4)
```

- **J0 range**: 0° → 180° (servo)
- **Khi J0 = 0°**: Arm hướng về +X
- **Khi J0 = 90°**: Arm hướng về +Y
- **Khi J0 = 180°**: Arm hướng về -X

---

## Build Integration

Thêm vào Makefile hoặc CMakeLists.txt:

```makefile
# Makefile
SRC += src/arm_kinematic.c
INC += -Iinc
```

```cmake
# CMakeLists.txt
add_library(arm_kinematic src/arm_kinematic.c)
target_include_directories(arm_kinematic PUBLIC inc)
```

---

## Giao Tiếp với ESP32

Sau khi tính IK, gửi lệnh servo qua TCP socket với format JSON:

```json
{"cmd": "servo", "ch": 0, "deg": 90.5}
{"cmd": "servo", "ch": 1, "deg": 120.0}
...
```

Mỗi lệnh kết thúc bằng `\n`.
