---
trigger: always_on
---

CRITICAL CONTROL ARCHITECTURE RULES:

1.  **GLOBAL FRAME ONLY:** This project utilizes a Field-Oriented Control (FOC) scheme. All high-level control code (C/Python on Jetson/PC) MUST calculate and output velocity vectors (`dot_x`, `dot_y`) in the **GLOBAL MAP FRAME**.

2.  **NO ROTATION MATRICES:** Do NOT implement coordinate transformation (Global-to-Body) in the high-level trajectory executor.
    -   **FORBIDDEN:** `vx_body = vx_global * cos(theta) + vy_global * sin(theta)`
    -   **REQUIRED:** `dot_x = vx_global`, `dot_y = vy_global`

3.  **LOW-LEVEL RESPONSIBILITY:** The embedded firmware (ESP32) is solely responsible for converting Global Frame commands into Body Frame for the Mecanum wheels.

4.  **CORRECTION POLICY:** If you encounter code attempting to rotate velocities based on `current_theta` in the trajectory planner, you must refactor it to send raw global velocities.