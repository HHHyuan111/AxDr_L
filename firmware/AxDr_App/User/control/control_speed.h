/**
 * @file control_speed.h
 * @brief 与硬件无关的角度差分测速接口。
 */

#ifndef AXDR_CONTROL_SPEED_H
#define AXDR_CONTROL_SPEED_H

/**
 * @brief 角度差分测速的显式运行状态。
 */
typedef struct
{
    float delta_angle_rad;
    float previous_angle_rad;
} control_angle_speed_state_t;

/**
 * @brief 清除角度差分并把当前角度作为下一拍基准。
 *
 * @param[out] state 要复位的测速状态。
 * @param[in] angle_rad 当前有效角度，单位 rad。
 */
void control_angle_speed_reset(control_angle_speed_state_t *state,
                               float angle_rad);

/**
 * @brief 根据相邻两个采样周期的角度差计算角速度。
 *
 * @param[in,out] state 当前测速实例独占的历史状态。
 * @param[in] angle_rad 本周期角度，单位 rad，按现有约定处于一圈范围内。
 * @param[in] sample_frequency_hz 调用频率，单位 Hz。
 * @return 角速度，单位 rad/s。
 * @pre state 已清零或由上一拍调用更新，采样频率沿用现有有效参数约定。
 *
 * 角度差跨越正负 pi 时按一圈回绕，保持 2*pi 到 0 附近的速度连续。
 */
float control_angle_speed_step(control_angle_speed_state_t *state,
                               float angle_rad,
                               float sample_frequency_hz);

#endif /* AXDR_CONTROL_SPEED_H */
