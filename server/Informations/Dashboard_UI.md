# Multi-Robot Server - Dashboard UI Overview

The Multi-Robot UI Dashboard (`server_gui_multi.py`) serves as the central control plane, built using the `tkinter` graphics framework. It provides a tabbed interface splitting operations into global orchestrations, individual edge controls, and network connections.

## 1. Window Layout Structure
The primary window (`1200x900` resolution) utilizes a persistent split:
- **Main Control Notebook**: Contains tabbed categories (`Global Dashboard`, `Robot Control`, `Robot Arm`, `Connection`, `Settings`).
- **Bottom System Monitor**: A persistent text console that logs real-time events, connection updates, and telemetry errors.

---

## 2. Global Dashboard Tab
Designed for Phase 1 (Approach) and Phase 2 (Transport) mass-orchestration. It is split into two panels:

### Left Panel: Live Status & Visualizer
- Maintains real-time data readouts including active coordinates (EKF bounds) and BNO055 headings.
- Handles the embedded `trajectory_visualizer` which can be popped out incrementally as an interactive **2D Map** displaying waypoints, generated splines, and current robot locales.

### Right Panel: Mission Control
- **Object Configuration**: Allows setting arbitrary target configurations (X, Y Cartesian offsets, dimensions).
- **Environment Adjustments**: Allows addition and removal of simulation obstacles (`obs_x`, `obs_y`, radius) integrated into the Vector Field generation matrices.
- **Phase Controls**: Dedicated action triggers for Phase 1 (`START APPROACH`) and Phase 2 (`START TRANSPORT`), bridging into the automated state machines.
- **Test Mode Menu**: A standalone GUI (`_open_test_mode`) allowing users to simulate trajectory math without needing actively connected robotics hardware.

---

## 3. Robot Control Tabs (Per-Robot Setup)
Individual tuning panels mapped mathematically to Robot 1, Robot 2, and Robot 3.

### Tuning Column (Left)
- **Motor Control**: Live monitoring grid isolating encoders vs. designated Set Speed overrides (up to 4 independent wheels per robot).
- **PID Settings**: Contains a PID Auto-Tune trigger requiring target RPM/PWM inputs. Below sits a grid allowing manual injection and fetching of `Proportional`, `Integral`, and `Derivative` constraints isolated per motor, integrating with load/save config mechanisms.

### Action Column (Right)
- **Control Actions**: Triggers for quick physical tests including `Square Test`, `Circle Test`, isolated manual jog controls, and a software ESP `Reset` function.
- **RPM Plot Viewer**: Deploys a standalone real-time graph via `server_rpm_plot.py` utilizing **Matplotlib**. Features dynamically resizable X/Y graphs visualizing all 4-wheel RPM metrics inside an updating `matplotlib.animation` event loop.
- **Sensor Calibration**: A BNO055 telemetry pane showing absolute heading and confirming successful magnetic configuration statuses.
- **Firmware Uploads (OTA)**: Controls for OTA binary flashing over TCP sockets, switching Edge endpoints into _Upgrade Mode_.

---

## 4. Robot Arm Tabs
Dedicated sub-panels built by `server_arm.py` tailored toward Inverse Kinematic deployments. These translate manual positional coordinate requirements (X, Y, Z space relative to the base) into servo PWM angles bridging into specialized "Grip" and "Place" triggers.

---

## 5. Connections & Profiles Tab
Handles the TCP socket bindings.
- **Profile Loader**: Fetches IP/Port layouts off active arrays defined dynamically inside `connection_config.txt` to enable 1-click boot-ups instead of repetitive manual typing.
- **Manual Input**: Fallback fields for edge-cases.
- **Global Actions**: Global emergency disconnect triggers isolating all TCP clients simultaneously.

---

## 6. Settings & Data Logging
- A global toggle switch configuring the CSV background writing behavior for the logging daemon.
- Firmware binary locator establishing base directories for OTA firmware packaging structures.
