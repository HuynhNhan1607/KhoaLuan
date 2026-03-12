/**
 * @file docking.h
 * @brief VL53L0X docking integration for Phase 1 precision grip
 *
 * After trajectory executor brings the robot into the acceptance zone,
 * the docking module takes over motor control and uses dual VL53L0X
 * ToF sensors to precisely align with the target object.
 *
 * When docking is complete, the robot grips at a fixed, known distance
 * instead of relying on EKF position (which has significant error).
 */

#ifndef DOCKING_H
#define DOCKING_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * Initialize the VL53L0X docking system.
     * Must be called once at startup (from main.c).
     *
     * @return true on success, false if sensors failed to initialize.
     *         On failure, docking is disabled and the system falls back
     *         to the old acceptance-zone behavior.
     */
    bool docking_init(void);

    /**
     * Start the docking procedure.
     * Called by trajectory_executor when the robot enters the acceptance zone.
     * Resets the docking state machine to SEARCHING.
     */
    void docking_start(void);

    /**
     * Run one docking cycle:
     *   1. Read VL53L0X sensors
     *   2. Run the 5-case state machine
     *   3. Convert body-frame commands to global frame using cur_theta
     *   4. Send motor commands
     *
     * Call this at DOCKING_LOOP_RATE_HZ (10 Hz) when docking is active.
     *
     * @param cur_theta  Current robot heading from EKF (radians, global frame)
     */
    void docking_update(float cur_theta);

    /**
     * Check if docking is currently active (started but not yet complete).
     */
    bool docking_is_active(void);

    /**
     * Check if docking has completed (robot is precisely aligned).
     * When true, it is safe to trigger the gripper at the fixed distance.
     */
    bool docking_is_complete(void);

    /**
     * Stop docking and send zero velocity.
     */
    void docking_stop(void);

    /**
     * Shut down the VL53L0X system (release GPIO, close I2C).
     */
    void docking_shutdown(void);

#ifdef __cplusplus
}
#endif

#endif /* DOCKING_H */
