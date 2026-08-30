/**
 * @file drive_pwm.h
 * @brief Drive 层三相 PWM 启停、相序映射和提交接口。
 */

#ifndef AXDR_DRIVE_PWM_H
#define AXDR_DRIVE_PWM_H

#include "common.h"

/**
 * @brief 启动三相主输出和互补输出。
 */
void drive_pwm_start(void);

/**
 * @brief 停止三相主输出和互补输出。
 */
void drive_pwm_stop(void);

/**
 * @brief 向三个物理 PWM 通道写入 50% 占空比。
 *
 * @param[in,out] pm 电机控制对象，用于记录本周期命令和提交结果。
 * @pre pm 指向快速周期独占写入的有效对象。
 *
 * 该接口只用于现有 START 流程，保持原工程先启动输出、再写 50% 的顺序。
 */
void drive_pwm_set_neutral(pmsm_t *pm);

/**
 * @brief 按电机相序把逻辑三相占空比提交到物理 PWM 通道。
 *
 * @param[in,out] pm 电机控制对象，提供三相占空比、周期编号和电机相序。
 * @pre pm 指向快速周期独占写入的有效对象；dtc_a/b/c 已由控制算法限幅。
 *
 * 本函数不计算 SVPWM，也不再次限幅。相序无效时保留命令记录，但不写 Target。
 */
void drive_pwm_commit(pmsm_t *pm);

#endif /* AXDR_DRIVE_PWM_H */
