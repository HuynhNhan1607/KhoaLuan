# Chi Tiết Hành Vi Hệ Thống (System Behavior Detail)

Tài liệu này cung cấp mô tả chi tiết về cách thức các module cốt lõi bên trong hệ thống `mini_server` trên Jetson NX phối hợp và xử lý dữ liệu để vận hành robot ở mức độ tối ưu, tin cậy và đảm bảo tính thời gian thực.

## 1. Hành Vi Khởi Tạo Hệ Thống (System Initialization)

Ngay khi Jetson Mini-Server khởi động (`main.c`):
1. **Khởi tạo phần cứng (Hardware Init):** Cấu hình chân `ESP_EN` thông qua thư viện `libgpiod` để cấp quyền hoặc đánh thức hệ thống ESP32. Thiết lập cấu hình quản lý lỗi `SIGINT`/`SIGPIPE` để tránh treo tiến trình khi kết nối TCP đột ngột đứt.
2. **Khởi tạo Module Toán học & Logic:** Các module `trajectory_init()` và `docking_init()` cấu hình sẵn các biến vùng nhớ cho quỹ đạo chạy tự động và thông số độ trễ cho cảm biến VL53L0X (thông số `timing_budget`, `I2C bus`, các chân `XSHUT_LEFT`/`RIGHT`).
3. **Kích hoạt các luồng độc lập:**
   * **`server_thread`**: Chờ đợi mảng ESP32 kết nối truyền tín hiệu điều khiển phần cứng.
   * **`laptop_server_thread`**: Chấp nhận tối đa 5 kết nối độc lập từ Laptop / Web Dashboard.
   * **`localize_thread`**: Lắng nghe và đồng bộ timestamp dữ liệu giữa các cảm biến.
   * **`optical_flow_thread`**: Quên đi độ trễ vòng lặp chính tĩnh, luồng này chạy vòng lặp cực nhanh để lấy độ lệch Frame-by-Frame của cảm biến luồng quang học qua UART.

## 2. Giao Tiếp Và Định Tuyến Dữ Liệu (Data Routing & Sockets)

`socket.c` đóng vai trò là "Nhà ga trung tâm" của mọi thông điệp:
* **Mô hình Event-Driven / Trạng thái chờ (poll/select)**: Luồng Laptop Server sử dụng cơ chế `select()` để hạn chế tiêu hao CPU khi nhàn rỗi, chỉ thức dậy khi có tín hiệu trên port.
* **Tích lũy buffer cho Payload lớn (Accumulation Buffer)**: Khi nhận một gói dữ liệu quỹ đạo dạng JSON (Trajectory JSON) có kích thước lớn, Socket tự động chuyển qua chế độ gom dữ liệu (`g_accumulating_traj = true`). Nó sẽ ghép nối chuỗi cho đến khi gặp điểm ngắt JSON hợp lệ `}}` sau đó mới giao cho bộ phân tích dữ liệu khởi chạy quỹ đạo (`trajectory_load`).
* **Sự kiện nâng cấp OTA (Firmware Upgrade)**: Hệ thống ngăn chặn xung đột bằng `pthread_mutex`. Nếu người dùng ấn "Upgrade", server chủ động tạo ra luồng `firmware_update_thread` chuyển đổi mode, truyền lệnh `Upgrade` xuống ESP32 và tiến hành truyền tải file binary từ máy chủ một cách an toàn cho tới khi nhận được `COMPLETED`.

## 3. Hành Vi Điều Khiển Quỹ Đạo (Trajectory Execution)

Bộ thực thi chuyển động dựa vào Trajectory Executor đảm bảo hai yếu tố là **tọa độ mong muốn** và **thời gian**:
* **Lên lịch chuyển động (Scheduled Start)**: Khi nhận được mảng trajectory, hệ thống chưa chạy ngay, mà chờ một mốc `epoch_time` đồng bộ từ máy chủ (lệnh `"cmd":"execute_trajectory" "time": <epoch>`).  
* **Truy đuổi mục tiêu tuyến tính (Pure Pursuit / PID)**: Sau mỗi khung thời gian (ví dụ 10ms - 100Hz), quỹ đạo đối chiếu thời gian `T` hiện tại so với dữ liệu thời gian của tập dữ liệu tham chiếu để xuất ra vector kỳ vọng (Expected x, y, theta). Vận tốc tuyến tính `dot_x, dot_y, dot_theta` được hiệu chỉnh bằng hệ số lỗi (Error offset) truyền vào lệnh chuỗi ký tự qua TCP cho ESP32.

## 4. Hành Vi Tiếp Cận Chính Xác Bằng VL53L0X (Docking Manager)

Module `docking.c` và thư viện `vl53l0x_manager` chịu trách nhiệm khi robot đi vào giai đoạn "mù" hệ thống định vị hoặc cần vào trạm sạc:
* **Xử lý Dual Sensor:** Cùng một I2C Bus nhưng hai cảm biến được khởi tạo lần lượt. Cảm biến 1 được kéo chân `XSHUT` lên cao trước, thay đổi từ I2C mặc định `0x29` sang địa chỉ mới (vd `0x30`), sau đó đánh thức cảm biến 2 và đổi sang địa chỉ (`0x31`).
* **Vòng lặp cập nhật State Machine (10Hz):**  
  * **Khoảng cách và lệch góc:** Tính toán hiệu số $\Delta$ khoảng cách từ cảm biến Trái và Phải. 
  * Định tuyến tốc độ mượt mà dựa theo $\Delta$. Nếu $\Delta$ lớn (bị nghiêng), `dot_theta` lớn để xoay xe tự động song song với mặt phẳng tiếp xúc.
  * Khi sai số góc xấp xỉ 0 và trung bình khoảng cách đạt mức giới hạn ngưỡng gắp (ví dụ 50mm) $\Rightarrow$ Kích hoạt cờ `SUCCESS_DOCKED` và ngắt động cơ `dot_x:0 dot_y:0`.

## 5. Hành Vi Tích Hợp Cảm Biến Bộ Lọc EKF (Sensor Fusion)

Hành vi `ekf.c` tính toán nội suy mô hình xe:
* Thuật toán trích xuất vị trí tuyệt đối từ biến đổi ma trận phức tạp.
* **Đầu vào (Observation Model):** Nhận 3 luồng dữ liệu độc lập chịu nhiễu khác nhau: `Odometry` (tính toán từ vòng quay bánh xe - bị trượt), `BNO055 IMU` (góc Yaw tuyệt đối hạn chế trượt do quán tính), `Optical Flow` (bù trừ trượt theo mặt phẳng).
* **Cập nhật (Update Step):** Theo tần số cao nhất đạt được, vòng lặp chuẩn đoán mức độ Noise/Covariance từ từng cảm biến. EKF dự đoán tọa độ theo mô hình Omni-directional Kinematic (Mecanum/Omni). Nếu có sự sai số bất thường (như bánh xe bị kẹt), trọng số (Kalman Gain) sẽ ưu tiên cho Optical Flow và IMU bù đắp lại. Tọa độ lọc cuối cùng đưa vào máy trạng thái phục vụ luồng `Trajectory`.

## 6. Xử Lý Tín Hiệu Cánh Tay Robot (Arm Control)

* Mọi lệnh từ Dashboard liên quan tới Robotic Arm (Pick, Place, Sync Kinematic) đều đổ về `arm_controller.c`. Máy trạng thái phân giải IK (Inverse Kinematics) từ một không gian Cartesian ra góc của từng Joint thay vì bắt phần cứng tính toán phức tạp. Góc này được bọc gọn dưới dạng chuỗi đóng gói theo chuẩn để ESP32 chuyển đổi trực tiếp thành PWM điều khiển Servo.
