# BÁO CÁO PHÂN TÍCH CODE PYTHON

**Ngày phân tích:** 2026-01-13  
**Thư mục phân tích:** `/home/huynhan1607/Multiple_Mobile_Robot/server/`  
**Tổng số file Python:** 20 files

---

## 1. CÁC FILE KHÔNG SỬ DỤNG (Unused Files)

### 1.1 Thư mục `unused/` - 7 files test cũ
| File | Mô tả |
|------|-------|
| `unused/test_min_spacing.py` | Test trajectory spacing |
| `unused/test_synchronized_integration.py` | Test synchronized trajectory |
| `unused/test_synchronized_trajectory.py` | Test synchronized trajectory |
| `unused/test_trajectory_manager.py` | Test trajectory manager |
| `unused/test_vector_field_bounds.py` | Test vector field bounds |
| `unused/test_vector_field_regression.py` | Test vector field regression |
| `unused/test_vector_field_tuning.py` | Test vector field tuning |

> **Khuyến nghị:** Có thể xóa toàn bộ thư mục `unused/` vì chúng là các test cũ không còn được sử dụng trong workflow hiện tại.

### 1.2 File độc lập không được import
| File | Mô tả |
|------|-------|
| `CalculateButterWorth.py` | Script tính toán Butterworth filter - chỉ chạy độc lập, không được import bởi file nào khác |

> **Khuyến nghị:** Nếu không cần sử dụng, có thể di chuyển vào thư mục `unused/` hoặc xóa.

---

## 2. FUNCTIONS KHÔNG SỬ DỤNG (Unused Functions)

### 2.1 Trong `trajectory_manager.py`
| Function | Dòng | Trạng thái |
|----------|------|------------|
| `get_path_from_astar()` | 113-129 | **KHÔNG SỬ DỤNG** - Đây là stub function chỉ in warning và return `[]` |

```python
def get_path_from_astar(map_grid, start_cell, goal_cell):
    # ... stub implementation ...
    print("Warning: get_path_from_astar is not yet implemented")
    return []
```

> **Khuyến nghị:** Xóa function này hoặc implement nếu cần A* algorithm.

---

## 3. IMPORTS KHÔNG SỬ DỤNG (Unused Imports)

### 3.1 Trong `server_multi.py` (dòng 17-20)
| Import | Trạng thái |
|--------|------------|
| `get_vector_field_planner` | **KHÔNG SỬ DỤNG** - Đã import nhưng dùng `VectorFieldPlanner()` trực tiếp |
| `get_formation_planner` | **KHÔNG SỬ DỤNG** - Đã import nhưng dùng `FormationPlanner()` trực tiếp |
| `generate_approach_trajectory` | **KHÔNG SỬ DỤNG TRỰC TIẾP** - Chỉ được gọi bởi `generate_multi_robot_approach_trajectories()` |

**Code hiện tại:**
```python
from vector_field import VectorFieldPlanner, get_vector_field_planner  # get_vector_field_planner không dùng
from formation_planner import FormationPlanner, get_formation_planner  # get_formation_planner không dùng
```

> **Khuyến nghị:** Xóa các imports không sử dụng:
> ```python
> from vector_field import VectorFieldPlanner
> from formation_planner import FormationPlanner
> ```

---

## 4. CODE TRÙNG LẶP (Duplicate Code Patterns)

### 4.1 Pattern gửi command tương tự trong `server_multi.py`

Các hàm sau có cấu trúc gần giống nhau:
- `send_command_to_robot()` - dòng 605-615
- `send_kinematic_command()` - dòng 592-603
- `send_position_goal()` - dòng 747-758
- `set_speed()` - dòng 760-771
- `emergency_stop()` - dòng 773-788

**Pattern chung:**
```python
def send_xxx(self, robot_id, ...):
    if robot_id not in self.robot_connections:
        return
    try:
        command = "..."
        sock = self.robot_connections[robot_id]
        sock.sendall((command + "\n").encode())
    except Exception as e:
        print(f"Robot {robot_id}: ... error: {e}")
```

> **Khuyến nghị:** Tạo một helper function `_send_raw_command()` để tránh trùng lặp:
> ```python
> def _send_raw_command(self, robot_id, command, log_msg=None):
>     if robot_id not in self.robot_connections:
>         return False
>     try:
>         sock = self.robot_connections[robot_id]
>         sock.sendall((command + "\n").encode())
>         if log_msg:
>             self.gui.update_monitor(log_msg)
>         return True
>     except Exception as e:
>         print(f"Robot {robot_id}: Command error: {e}")
>         return False
> ```

### 4.2 Pattern kiểm tra connection

Pattern `if robot_id not in self.robot_connections:` xuất hiện **hơn 15 lần** trong `server_multi.py`.

---

## 5. LEGACY CODE / CODE CẦN XEM XÉT

### 5.1 Trong `server_position.py`
| Code | Dòng | Trạng thái |
|------|------|------------|
| `send_destination_to_client()` | 411-416 | **ĐÃ COMMENT OUT** - Code ví dụ đã bị comment |

### 5.2 Trong `server_multi.py`
| Code | Dòng | Ghi chú |
|------|------|---------|
| `decentralized_execution` | 62-64 | Legacy flag, có thể được refactor |

---

## 6. TÓM TẮT VÀ KHUYẾN NGHỊ

### Ưu tiên cao (Nên xóa/sửa ngay)
1. ❌ Xóa thư mục `unused/` (7 files test cũ)
2. ❌ Xóa function `get_path_from_astar()` trong `trajectory_manager.py`
3. ✏️ Xóa imports không dùng: `get_vector_field_planner`, `get_formation_planner`

### Ưu tiên trung bình (Nên refactor)
4. 🔄 Tạo helper function `_send_raw_command()` để giảm code trùng lặp
5. 🔄 Di chuyển `CalculateButterWorth.py` vào thư mục utilities hoặc xóa

### Thống kê
| Loại | Số lượng |
|------|----------|
| Files không dùng | 8 files |
| Functions không dùng | 1 function |
| Imports không dùng | 3 imports |
| Code patterns trùng lặp | 5-6 hàm gửi command |

---

*Báo cáo được tạo tự động bởi công cụ phân tích code.*
