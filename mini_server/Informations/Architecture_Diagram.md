# Kiến Trúc Hệ Thống Jetson Mini-Server

Tài liệu này trình bày các biểu đồ kiến trúc hệ thống chi tiết cho `mini_server` chạy trên Jetson, bao gồm kiến trúc phần mềm, mô hình đa luồng (multi-threading), và kết nối phần cứng. Các thành phần được thiết kế theo hướng module hóa cao (modular design) để thuận tiện cho việc bảo trì và mở rộng.

## 1. Biểu Đồ Kiến Trúc Tổng Quan (System Architecture)

Hệ thống được chia làm 3 phân hệ chính: Laptop Dashboard (Máy chủ điều khiển trung tâm), Jetson Mini-Server (Máy chủ vi mạch/Edge Computing), và ESP32/Phần cứng dưới quyền (Cơ cấu chấp hành và cảm biến tầng thấp).

```mermaid
graph TD
    %% Define styles
    classDef jetson fill:#2e7d32,stroke:#1b5e20,stroke-width:2px,color:#fff;
    classDef esp fill:#1565c0,stroke:#0d47a1,stroke-width:2px,color:#fff;
    classDef laptop fill:#d84315,stroke:#bf360c,stroke-width:2px,color:#fff;
    classDef hardware fill:#616161,stroke:#424242,stroke-width:2px,color:#fff;

    subgraph "Laptop (Central Server / Dashboard)"
        Dashboard[Web Dashboard/UI]:::laptop
        TrajGen[Grasping & Trajectory Planning]:::laptop
    end

    subgraph "Jetson NX Mini-Server (C Core)"
        SocketApp[Socket & Client Manager]:::jetson
        JSONParse[JSON Handler & Command Router]:::jetson
        
        subgraph "Control Modules"
            ekf[EKF / Localization]:::jetson
            trajExec[Trajectory Executor]:::jetson
            docking[Docking Manager]:::jetson
            armCtrl[Arm Controller & Kinematics]:::jetson
        end
        
        subgraph "Sensor Drivers"
            optFlow[Optical Flow UART]:::jetson
            vl53[VL53L0X I2C Manager]:::jetson
        end
    end

    subgraph "ESP32 (Microcontroller)"
        MotDrv[Motor PID / Control]:::esp
        Sens[BNO055 & Encoders]:::esp
        ArmSe[Arm Servos / Gripper]:::esp
    end

    subgraph "Hardware Components"
        Motors[DC Motors]:::hardware
        OptSensor[PMW3901 Optical Flow]:::hardware
        ToFSensor[2x VL53L0X ToF Sensors]:::hardware
    end

    %% Connections
    Dashboard <-->|"TCP/IP (Port 8080/8081)"| SocketApp
    TrajGen -->|"JSON / Paths"| SocketApp
    
    SocketApp --> JSONParse
    JSONParse --> trajExec
    JSONParse --> armCtrl
    JSONParse --> docking
    
    optFlow --> ekf
    Sens -->|"TCP/IP JSON"| SocketApp
    SocketApp --> ekf
    
    ekf --> trajExec
    trajExec -->|"Velocity Commands"| SocketApp
    docking -->|"Docking Velocities"| SocketApp
    
    SocketApp <-->|"TCP/IP (Port 8000)"| MotDrv
    SocketApp -->|"Servo Commands"| ArmSe
    
    MotDrv --> Motors
    OptSensor -->|UART| optFlow
    ToFSensor -->|I2C| vl53
    vl53 --> docking
```

## 2. Mô Hình Đa Luồng Nội Bộ (Internal Threading Model)

`mini_server` khởi tạo và duy trì các luồng song song (pthread) để xử lý các tác vụ bất đồng bộ, đảm bảo tính thời gian thực (real-time) mà không khóa (block) lẫn nhau.

```mermaid
sequenceDiagram
    participant Main as Main Thread
    participant LS as Laptop Server Thread
    participant ES as ESP32 Server Thread
    participant LT as Localize / EKF Thread
    participant OT as Optical Flow Thread
    participant FM as Firmware Update Thread (Dynamic)

    Main->>Main: Init GPIO, Trajectory, Docking
    Main->>ES: pthread_create(server_thread)
    Main->>LS: pthread_create(laptop_server_thread)
    Main->>LT: pthread_create(localize_thread)
    Main->>OT: pthread_create(optical_flow_thread)
    
    Note over LS: Khởi tạo TCP Listener (Port > 8000)<br/>Chờ Laptop kết nối
    Note over ES: Khởi tạo TCP Listener (Port 8000)<br/>Chờ ESP32 kết nối

    par Real-time Operation
        LS->>LS: Lắng nghe lệnh điều hướng / JSON Trajectory
        ES->>ES: Nhận dữ liệu Odometry + IMU từ ESP32
        OT->>OT: Read UART (PMW3901) liên tục -> buffer
        LT->>LT: Lấy Odometry + IMU + Optical Flow -> Compute EKF
    end
    
    Note over LS: Nếu có yêu cầu OTA Firmware Update
    LS->>FM: Khởi tạo tuyến trình phụ (Firmware)
    FM-->>ES: Broadcase "Upgrade" mode tới ESP32
    FM->>FM: Streaming binary file từ Laptop sang ESP32
```

## 3. Quản Lý Trạng Thái Của Chức Năng Docking (State Machine)

Core logic của tính năng tiếp cận mục tiêu (Docking) sử dụng 2 cảm biến ToF VL53L0X để xác định khoảng cách và góc lệch.

```mermaid
stateDiagram-v2
    [*] --> NOT_INITIALIZED: Power On
    NOT_INITIALIZED --> STANDBY: docking_init() success
    
    STANDBY --> SEARCHING: Bắt đầu Docking (docking_start)
    
    SEARCHING --> ALIGNING: Phát hiện đối tượng 1 bên (Trái hoặc Phải)
    SEARCHING --> APPROACHING: Phát hiện đối tượng 2 bên, delta < threshold
    
    ALIGNING --> APPROACHING: Cả 2 cảm biến đã cân bằng
    ALIGNING --> OBJECT_LOST: Mất tín hiệu
    
    APPROACHING --> SUCCESS_DOCKED: Avg Distance <= Target (e.g. 50mm)
    APPROACHING --> ALIGNING: Cảm biến trả độ lệch (delta) lớn
    APPROACHING --> OBJECT_LOST: Mất tín hiệu
    
    SUCCESS_DOCKED --> STANDBY: Dừng động cơ, hoàn thành
    OBJECT_LOST --> STANDBY: Ngừng quá trình (Timeout / Lỗi)
```

## 4. Đặc tả Giao Tiếp Dữ Liệu Socket (Data Flow)

Luồng đi của dữ liệu từ và đến Jetson Mini-Server, đặc biệt là xử lý `JSON` payload:

```mermaid
flowchart LR
    ESP32[ESP32 Clients] -->|Sensors/Feedback| SM[Socket Manager]
    Laptop[Laptop UI] -->|Command/Trajectory| SM
    
    SM -->|Parses commands| JH[JSON Handler]
    
    JH -->|Trajectory Data| TE[Trajectory Executor]
    JH -->|Arm Commands| AC[Arm Controller]
    JH -->|Docking Start/Stop| DM[Docking Manager]
    
    TE -->|dot_x, dot_y, dot_theta| CM[Client Manager Broadcast]
    DM -->|dot_x, dot_y, dot_theta| CM
    AC -->|Joint Angles / IK Result| CM
    
    CM -->|Raw string TCP| ESP32
```

## 5. Tổ Chức Thư Mục Mã Nguồn

Sơ đồ thể hiện cách phân tách mã nguồn trong Jetson Mini-Server:

*   **/src**
    *   `main.c`: Entry point, khởi tạo hệ thống và quản lý thread.
    *   `socket.c`: Quản lý TCP server cho Laptop và ESP32.
    *   `ekf.c` / `localize.c`: Thuật toán EKF và luồng tính toán vị trí.
    *   `docking.c`: Các API tiếp cận đích thông minh.
    *   `arm_controller.c` / `arm_kinematic.c`: Thuật toán cho cánh tay robot.
    *   `trajectory_executor.c`: Tính toán và di chuyển theo điểm tọa độ định trước.
*   **/vl53l0x**
    *   `vl53l0x_manager.c` & `vl53l0x_c.c`: Driver hạt nhân qua I2C.
    *   `gpio_helper.c`: Điều khiển chân XSHUT cứng của Jetson NX.
*   **/inc**: Các header file định nghĩa dữ liệu public.
