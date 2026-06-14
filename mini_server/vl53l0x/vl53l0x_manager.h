/**
 * @file vl53l0x_manager.h
 * @brief High-level dual VL53L0X manager with docking state machine
 *
 * Manages two VL53L0X ToF sensors (LEFT / RIGHT) via GPIO XSHUT + I2C,
 * and runs a 5-case docking state machine that outputs velocity commands
 * (vx, vy, omega) for a Mecanum-wheel robot.
 *
 * Dependencies:
 *   - vl53l0x_c.h / vl53l0x_c.c  (sensor driver)
 *   - gpio_helper.h / gpio_helper.c  (libgpiod wrapper)
 *   - libgpiod v1:  sudo apt install libgpiod-dev
 */

#ifndef VL53L0X_MANAGER_H
#define VL53L0X_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /* ═══════════════════════════════════════════════════════════════════════════
     * ENUMS
     * ═══════════════════════════════════════════════════════════════════════════ */

    /**
     * Docking state — maps directly to the 5 cases described in the spec.
     */
    typedef enum
    {
        DOCK_STATE_SEARCHING,     /**< Case 1: Cả 2 không thấy → trượt X tìm kiếm        */
        DOCK_STATE_CENTERING_X,   /**< Case 2: 1 thấy, 1 không → trượt ngang về phía trống */
        DOCK_STATE_ALIGNING_YAW,  /**< Case 3: Cả 2 thấy nhưng lệch → xoay tại chỗ        */
        DOCK_STATE_APPROACHING_Y, /**< Case 4: Đã thẳng hàng → tiến tới mục tiêu           */
        DOCK_STATE_BACKING_Y,     /**< Case 4b: Quá sát → lùi ra cho đến khi đủ khoảng cách */
        DOCK_STATE_DOCKED,        /**< Case 5: Đúng vị trí → dừng, kích hoạt gắp            */
        DOCK_STATE_NOT_FOUND,     /**< Tìm kiếm hết phạm vi mà không thấy vật              */
        DOCK_STATE_ERROR          /**< Lỗi khởi tạo hoặc phần cứng                         */
    } DockState;

    /* ═══════════════════════════════════════════════════════════════════════════
     * DATA STRUCTURES
     * ═══════════════════════════════════════════════════════════════════════════ */

    /**
     * Velocity command output for the Mecanum base.
     *
     * Convention (nhìn từ trên xuống, heading = trục Y):
     *   vx    > 0 : trượt sang PHẢI (lateral)
     *   vy    > 0 : tiến về trước (heading, hướng tới vật)
     *   omega > 0 : xoay ngược chiều kim đồng hồ (CCW)
     *
     * Values are in actual units: vx/vy in m/s, omega in rad/s.
     */
    typedef struct
    {
        float vx;
        float vy;
        float omega;
    } DockCommand;

    /**
     * Full status returned by every update cycle.
     */
    typedef struct
    {
        /* Current state in the docking state machine */
        DockState state;

        /* Velocity command to send to the motor controller */
        DockCommand cmd;

        /* true when state == DOCK_STATE_DOCKED → safe to trigger gripper */
        bool docking_complete;

        /* Raw sensor readings (mm) */
        uint16_t dist_left_mm;
        uint16_t dist_right_mm;

        /* Validity flags (false = timeout OR reading beyond max_range) */
        bool left_valid;
        bool right_valid;

        /* Computed from valid readings */
        int16_t delta_mm;         /**< dist_left − dist_right (signed)  */
        uint16_t avg_distance_mm; /**< average of two valid readings    */
    } DockStatus;

    /**
     * Configuration for the manager — pass to vl53l0x_manager_init().
     * Use vl53l0x_manager_default_config() to get sensible defaults,
     * then modify the fields you need.
     */
    typedef struct
    {
        /* ── I2C ─────────────────────────────────────────────────────────── */
        uint8_t i2c_bus;   /**< I2C bus number (default 8)       */
        uint8_t addr_left; /**< New addr for LEFT (default 0x30) */
        /* RIGHT sensor always uses default 0x29                              */

        /* ── GPIO (libgpiod) ─────────────────────────────────────────────── */
        const char *gpio_chip;        /**< Chip name, e.g. "gpiochip1"     */
        unsigned int gpio_line_left;  /**< Line offset for XSHUT LEFT      */
        unsigned int gpio_line_right; /**< Line offset for XSHUT RIGHT     */

        /* ── Detection thresholds (mm) ───────────────────────────────────── */
        uint16_t max_range_mm;      /**< Above this = no object detected  */
        uint16_t yaw_tolerance_mm;  /**< |delta| <= this = yaw OK         */
        uint16_t yaw_realign_mm;    /**< During approach, realign if |delta| exceeds this */
        uint16_t dock_distance_mm;  /**< Target distance for docking      */
        uint16_t dock_tolerance_mm; /**< ± tolerance on dock distance     */

        /* ── Speeds (m/s for linear, rad/s for angular) ────────────────── */
        float search_vy;         /**< Lateral speed for searching (m/s)      */
        float center_vy;         /**< Lateral speed for centering (m/s)      */
        float align_omega;       /**< Angular speed for yaw alignment (rad/s)*/
        float approach_vx;       /**< Forward speed for approach (m/s)       */
        float approach_yaw_gain; /**< Proportional yaw correction gain       */

        /* ── Search behaviour ────────────────────────────────────────────── */
        int search_direction; /**< +1 = slide right first, −1 = left */

        /* ── Sensor I/O timeout (ms) ─────────────────────────────────────── */
        uint16_t sensor_timeout_ms; /**< Timeout for each sensor read     */

        /* ── Calibration offsets (mm, signed) ───────────────────────────── */
        /* Corrects systematic bias between the two sensors.                  */
        /* raw_reading + offset = calibrated_reading                          */
        /* Example: right reads +20 mm too high → calib_offset_right_mm = -20 */
        int16_t calib_offset_left_mm;  /**< Additive offset for LEFT sensor  */
        int16_t calib_offset_right_mm; /**< Additive offset for RIGHT sensor */
    } DockConfig;

    /**
     * Opaque manager handle.  Allocate on stack or heap; always initialise
     * with vl53l0x_manager_init() before use.
     */
    typedef struct VL53L0XManager VL53L0XManager;

    /* ═══════════════════════════════════════════════════════════════════════════
     * API
     * ═══════════════════════════════════════════════════════════════════════════ */

    /**
     * Return a DockConfig filled with sensible default values.
     * Modify the fields you need, then pass it to vl53l0x_manager_init().
     */
    DockConfig vl53l0x_manager_default_config(void);

    /**
     * Allocate and initialise the manager:
     *   1. Setup GPIO XSHUT (libgpiod)
     *   2. Address-assignment sequence (reset → wake LEFT → change addr → wake RIGHT)
     *   3. Init both VL53L0X sensors
     *
     * @return Pointer to the manager on success, NULL on failure.
     *         Free with vl53l0x_manager_shutdown().
     */
    VL53L0XManager *vl53l0x_manager_init(const DockConfig *config);

    /**
     * Run one docking cycle:
     *   1. Read both sensors
     *   2. Evaluate the 5-case state machine
     *   3. Compute the velocity command
     *
     * Call this in your control loop (typically every 50–100 ms).
     *
     * @return DockStatus with state, command, and sensor data.
     */
    DockStatus vl53l0x_manager_update(VL53L0XManager *mgr);

    /**
     * Reset the state machine back to DOCK_STATE_SEARCHING.
     * Useful after a pick-and-place cycle to restart docking for the next object.
     */
    void vl53l0x_manager_reset(VL53L0XManager *mgr);

    /**
     * Shut down: close I2C, release GPIO lines, free memory.
     */
    void vl53l0x_manager_shutdown(VL53L0XManager *mgr);

    /**
     * Utility: return a human-readable name for a DockState.
     */
    const char *dock_state_name(DockState state);

#ifdef __cplusplus
}
#endif

#endif /* VL53L0X_MANAGER_H */
