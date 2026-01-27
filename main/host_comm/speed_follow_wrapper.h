#ifndef SPEED_FOLLOW_WRAPPER_H
#define SPEED_FOLLOW_WRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief C包装函数：调整力矩
 * @param speed_follow_ptr SpeedFollowMode实例指针
 * @param increase true=增加，false=减少
 * @return 调整后的力矩值
 */
float speed_follow_adjust_torque(void* speed_follow_ptr, bool increase);

/**
 * @brief C包装函数：调整Kd
 * @param speed_follow_ptr SpeedFollowMode实例指针
 * @param increase true=增加，false=减少
 * @return 调整后的Kd值
 */
float speed_follow_adjust_kd(void* speed_follow_ptr, bool increase);

/**
 * @brief C包装函数：调整Phase2力矩（压腿力矩）
 * @param speed_follow_ptr SpeedFollowMode实例指针
 * @param increase true=增加绝对值，false=减少绝对值
 * @return 调整后的Phase2力矩绝对值
 */
float speed_follow_adjust_phase2_torque(void* speed_follow_ptr, bool increase);

/**
 * @brief C包装函数：获取当前力矩值
 * @param speed_follow_ptr SpeedFollowMode实例指针
 * @return 当前力矩值
 */
float speed_follow_get_current_torque(void* speed_follow_ptr);

/**
 * @brief C包装函数：获取当前Kd值
 * @param speed_follow_ptr SpeedFollowMode实例指针
 * @return 当前Kd值
 */
float speed_follow_get_current_kd(void* speed_follow_ptr);

/**
 * @brief C包装函数：获取当前Phase2力矩绝对值
 * @param speed_follow_ptr SpeedFollowMode实例指针
 * @return 当前Phase2力矩绝对值
 */
float speed_follow_get_current_phase2_torque(void* speed_follow_ptr);

#ifdef __cplusplus
}
#endif

#endif // SPEED_FOLLOW_WRAPPER_H
