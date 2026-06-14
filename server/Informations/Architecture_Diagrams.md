# Multi-Robot Server System Diagrams

To provide a visual and clearer view of how the system operates conceptually, here are three architectural plots drawn using Mermaid.js syntax. These diagram the application hierarchy, network logic flow, and high-frequency sensor telemetry.

## 1. High-Level Modular Architecture
This diagram outlines how the main Server loop distributes processing tasks across different Python modules, and how the physical robots interface.

```mermaid
graph TD
    classDef main fill:#1f77b4,stroke:#fff,stroke-width:2px,color:#fff
    classDef sub fill:#ff7f0e,stroke:#fff,stroke-width:2px
    classDef vis fill:#2ca02c,stroke:#fff,stroke-width:2px,color:#fff
    classDef edge fill:#d62728,stroke:#fff,stroke-width:4px,color:#fff
    
    subgraph Server_Computer [Server Application]
        GUI[Server GUI<br> server_gui_multi.py]:::main
        MainServer[Core Server Logic<br> server_multi.py]:::main
        GUI -->|User Triggers| MainServer
        
        subgraph Sub_Managers [Sub-Managers]
            MainServer -->|Phase 1 Paths| Approach[Approach Manager<br> approach_manager.py]:::sub
            MainServer -->|Phase 2 Paths| Transport[Transport Manager<br> transport_manager.py]:::sub
            MainServer -->|Offset Geometry| Planners[Formation Planner<br> formation_planner.py]:::sub
            MainServer -->|Queue Push| Logger[Async File Writer<br> _sync_log_worker]:::sub
        end
        
        subgraph Visualization_Tooling [Visualization Tooling]
            GUI --> Visualizer[2D Target Map<br> server_trajection_multi.py]:::vis
            GUI --> RPMPlot[Live Matplotlib Window<br> server_rpm_plot.py]:::vis
        end
    end
    
    subgraph Edge_Hardware [Robot Swarm]
        Robot1[Robot 1<br>Microcontroller]:::edge
        Robot2[Robot 2<br>Microcontroller]:::edge
        Robot3[Robot 3<br>Microcontroller]:::edge
    end
    
    MainServer <-->|TCP Socket / JSON Payload| Robot1
    MainServer <-->|TCP Socket / JSON Payload| Robot2
    MainServer <-->|TCP Socket / JSON Payload| Robot3
```


## 2. Mission Logic Flow (Sequence Diagram)
This timeline dictates the sequential message logic exchanged during Phase 1 (Approach) and Phase 2 (Transport) between the orchestrator and the individual decentralized robot controllers.

```mermaid
sequenceDiagram
    autonumber
    participant UI as Server GUI
    participant Srv as Server Backend
    participant R as Robot (Edge Node)
    
    Note over UI, R: System Connection Stage
    UI->>Srv: Initiate Profile (IP:Port)
    Srv->>R: TCP Socket Bind
    R-->>Srv: Accept Link & Transmit Telemetry
    
    Note over UI, R: Phase 1: Object Approach Sequence
    UI->>Srv: START APPROACH Triggered
    Srv->>Srv: Run A* & Avoid Collisions
    Srv->>R: {"type": "load_trajectory", "data": [waypoints]}
    Srv->>R: {"type": "control", "command": "execute_trajectory"}
    
    Note over R: Robot acts as standalone<br/>execution loop utilizing timers.
    R-->>Srv: {"type": "status", "status": "arrived"}
    
    Note over Srv: Awaits ALL robots marking "arrived"
    Srv->>R: {"type": "control", "command": "execute_grip"}
    
    Note over UI, R: Phase 2: Virtual Structure Transport
    UI->>Srv: START TRANSPORT Triggered (XY Dest)
    Srv->>Srv: Form Cubic Splines & Apply Offsets
    Srv->>R: {"type": "load_trajectory", "data": [waypoints]}
    Srv->>R: {"type": "control", "command": "execute_trajectory"}
    
    R-->>Srv: {"type": "status", "status": "transport_complete"}
    Srv->>R: {"type": "control", "command": "execute_place"}
```


## 3. Telemetry Pipeline & Disk Logging
Due to the high influx of network data (thousands of floats continuously transmitted per second), the Server prevents UI blocking by spreading mathematical parsing vs hard-drive storage writing across distinct threads and thread-lock queues in Python.

```mermaid
graph LR
    classDef edge fill:#d62728,stroke:#fff,stroke-width:2px,color:#fff
    classDef thread1 fill:#1f77b4,stroke:#fff,stroke-width:2px,color:#fff
    classDef thread2 fill:#ff7f0e,stroke:#fff,stroke-width:2px,color:#fff
    
    subgraph Microcontroller [Physical Robot]
        SENS1[Raw Encoders]
        SENS2[BNO055 Yaw]
        SENS3[Odometry]
        SENS1 -.-> JSON_PACK
        SENS2 -.-> JSON_PACK
        SENS3 -.-> JSON_PACK
        JSON_PACK[Serialize JSON]:::edge --> TX[Wi-Fi TX]:::edge
    end
    
    subgraph UI_Master_Thread [Main Socket Loop]
        RX[TCP Payload Receive]:::thread1
        RX --> JSON_PARSE[Decode JSON Dict]:::thread1
        JSON_PARSE --> UPDATE1[Tkinter UI Update]:::thread1
        JSON_PARSE --> UPDATE2[Matplotlib Memory Buffer]:::thread1
        JSON_PARSE --> Q_APPEND[Push to Thread Queue]:::thread1
    end
    
    subgraph Background_Disk_Worker [Background Disk Worker]
        Q_GET[Pull from Thread Queue]:::thread2
        Q_GET --> IO_WRITE[Flush I/O Disk Write]:::thread2
        IO_WRITE --> CSVF[(Telemetry .CSV Logs)]:::thread2
    end
    
    TX == High Rate Transmission ==> RX
    Q_APPEND -.->|Mutex Pipeline| Q_GET
```
