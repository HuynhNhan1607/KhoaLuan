#ifndef FORMATION_MANAGER_H
#define FORMATION_MANAGER_H

#include <stdbool.h>

/**
 * Formation Manager - Multi-Robot Coordination
 *
 * Hybrid Virtual Structure + Leader-Follower Control:
 *
 * 1. Lock Offset: Khi gắp vật xong, lock offset = robot_pos - centroid
 * 2. Robot1 (Leader): Thực thi trajectory với target = centroid + offset1
 * 3. Robot2 (Follower): Thực thi trajectory với target = centroid + offset2
 *                      + feedback correction từ Robot1
 *
 * Flow:
 * - Server gửi trajectory của centroid (tâm vật)
 * - Mỗi robot nội suy ra trajectory của mình = centroid + offset
 * - Robot2 thêm correction dựa trên vị trí thực tế của Robot1
 */

// Khởi tạo formation manager
void formation_init(void);

// =========== LEADER-FOLLOWER MODE (OLD) ===========

// Cập nhật vị trí của robot hàng xóm (từ sync_position)
// uncertainty: RMS position error from EKF (sqrt(P[0][0]+P[1][1]))
void formation_update_neighbor(double x, double y, double vx, double vy,
                               double theta, double ts, double pos_uncertainty);

// Lấy vận tốc follow velocity cho follower mode
bool formation_get_follow_velocity(double *vx, double *vy);

// Lấy vận tốc follow velocity cho transport phase (Leader-Follower mode)
bool formation_get_transport_follow_velocity(double *vx, double *vy);

// Bật/tắt chế độ follow
void formation_set_follow_enabled(bool enable);

// Kiểm tra chế độ follow
bool formation_is_follow_enabled(void);

// Dispatch event: User is controlling this robot manually
void formation_dispatch_user_control(void);

// =========== VIRTUAL STRUCTURE TRANSPORT MODE ===========

/**
 * Activate transport mode - called when both robots grip object
 * @param centroid_x, centroid_y: Object (centroid) position
 */
void formation_lock_transport_offset(double centroid_x, double centroid_y);

/**
 * Update centroid target from trajectory
 * @param x, y: New centroid target position
 */
void formation_set_centroid_target(double x, double y);

/**
 * Get robot's target position = centroid + transport_offset
 * For Robot2 (follower), also includes correction from Robot1's actual position
 * @param x, y: Output - robot's target position
 * @return true if transport mode active
 */
bool formation_get_robot_target(double *x, double *y);

/**
 * Get transport offset (robot_pos - centroid at lock time)
 * @param offset_x, offset_y: Output - transport offset
 * @return true if transport mode active
 */
bool formation_get_transport_offset(double *offset_x, double *offset_y);

/**
 * Check if robot is in transport mode
 */
bool formation_is_transport_active(void);

/**
 * Get follower position error (for leader speed scaling)
 * Robot1 calls this to check if Robot2 is keeping up
 * @param error_distance: Output - distance between follower actual and expected position
 * @return true if follower data is valid and error is calculated
 */
bool formation_get_follower_error(double *error_distance);

/**
 * Get locked theta during transport mode
 * @param locked_theta: Output - locked theta value
 * @return true if transport mode active and theta is locked
 */
bool formation_get_locked_theta(double *locked_theta);

/**
 * End transport mode
 */
void formation_end_transport(void);

// =========== COMMON ===========

// Kiểm tra xem formation đã được lock chưa
bool formation_is_locked(void);

// Reset formation
void formation_reset(void);

// Cleanup
void formation_cleanup(void);

#endif // FORMATION_MANAGER_H
