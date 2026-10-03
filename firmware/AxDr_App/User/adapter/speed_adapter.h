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
 * @brief 输入本拍转子位置 [rad]，更新测速输出。
 * @param pos_rad 转子角（推荐多圈连续角 pos_r；单圈角亦可，差分内置解卷绕防御）。
 * @param fs_hz 采样频率 [Hz]（由 control_cycle 以 rate.foc_fs 传入，禁止硬编码）。
 * @note  喂入点在 control_cycle_step（回放缝内），位置无效拍由调用方跳过＝速度冻结。
 */
void speed_est_step(float pos_rad, float fs_hz);

/** @brief 影子速度 [rad/s]（转子侧机械角速度，未经低通）。 */
float speed_est_get(void);

#endif /* SPEED_ADAPTER_H */
