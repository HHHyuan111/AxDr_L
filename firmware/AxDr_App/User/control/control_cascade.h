/**
 * @file control_cascade.h
 * @brief 电流、速度和位置级联控制的可移植执行接口。
 */

#ifndef AXDR_CONTROL_CASCADE_H
#define AXDR_CONTROL_CASCADE_H

#include <stdbool.h>
#include <stdint.h>

#include "control_pid.h"

/**
 * @brief 一个控制环的整数分频状态。
 */
typedef struct
{
    uint32_t count;
    uint32_t divider;
} control_rate_t;

/**
 * @brief 按现有分频规则执行一次 d、q 轴电流控制。
 *
 * @return 本周期实际更新 PI 输出时返回 true。
 *
 * 电流和电流参考使用 A，电压输出使用 V。为保持现有固件行为，达到分频值后
 * 计数器不清零；当前 20 kHz 配置由调用者提供原有 divider。
 */
bool control_cur_step(control_rate_t *rate,
                      pid_para_t *d_pid,
                      pid_para_t *q_pid,
                      float id_ref,
                      float iq_ref,
                      float id_feedback,
                      float iq_feedback,
                      float *v_d,
                      float *v_q);

/**
 * @brief 按整数分频执行一次速度 PDFF，并生成 q 轴电流参考。
 *
 * @param[in] iq_limit_abs q 轴电流绝对限制；不大于零时保持现有“不额外限幅”行为。
 * @return 本周期实际更新速度环输出时返回 true。
 */
bool control_spd_step(control_rate_t *rate,
                      pid_para_t *speed_pid,
                      float speed_ref,
                      float speed_feedback,
                      float iq_limit_abs,
                      float *iq_ref);

/**
 * @brief 按整数分频执行一次位置 PI，并生成速度参考。
 *
 * @param[in] speed_limit_abs 速度绝对限制；不大于零时保持现有“不额外限幅”行为。
 * @return 本周期实际更新位置环输出时返回 true。
 */
bool control_pos_step(control_rate_t *rate,
                      pid_para_t *position_pid,
                      float position_ref,
                      float position_feedback,
                      float speed_limit_abs,
                      float *speed_ref);

#endif /* AXDR_CONTROL_CASCADE_H */
