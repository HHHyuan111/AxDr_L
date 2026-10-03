/**
 * @file speed_adapter.h
 * @brief 位置差分测速（影子链）：B 库 axdr_speed_estimator 同构实现。
 * @note  窗口差分 + 单圈角解卷绕；窗口可配（沉沙 = 2 拍）。
 *        当前仅作影子观测（关口①对照），验证通过后接管主链速度反馈。
 */

#ifndef SPEED_ADAPTER_H
#define SPEED_ADAPTER_H

#include <stdint.h>

/** @brief 初始化历史环形缓冲。上电调用一次。 */
void speed_est_init(void);

/**
 * @brief 输入本拍单圈机械角 [rad]，更新影子速度。
 * @param pos_rad 单圈角度（0..2π 之间或含回绕的连续读数均可）。
 * @param fs_hz 采样频率 [Hz]（控制周期 20000）。
 */
void speed_est_step(float pos_rad, float fs_hz);

/** @brief 影子速度 [rad/s]（转子侧机械角速度，未经低通）。 */
float speed_est_get(void);

#endif /* SPEED_ADAPTER_H */
