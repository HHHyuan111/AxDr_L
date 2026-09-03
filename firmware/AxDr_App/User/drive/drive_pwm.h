/**
 * @file drive_pwm.h
 * @brief Drive 层三相 PWM 启停、相序映射和提交接口。
 */

#ifndef AXDR_DRIVE_PWM_H
#define AXDR_DRIVE_PWM_H

#include <stdbool.h>

#include "foc_fwd.h"

/**
 * @brief 启动三相主输出和互补输出。
 * @return Target 报告六路输出均成功启动时返回 true，否则返回 false。
 */
bool drive_pwm_start(void);

/**
 * @brief 停止三相主输出和互补输出。
 * @return Target 报告六路输出均成功停止时返回 true，否则返回 false。
 */
bool drive_pwm_stop(void);

/**
 * @brief 向三个物理 PWM 通道写入 50% 占空比。
 *
 * @param[in,out] foc 电机控制对象，用于记录本周期命令和提交结果。
 * @pre foc 指向快速周期独占写入的有效对象。
 *
 * 该接口只用于 START 流程，在启动输出前先写入已知的 50% 比较值。
 */
void drive_pwm_set_neutral(foc_t *foc);

/**
 * @brief 按电机相序把逻辑三相占空比提交到物理 PWM 通道。
 *
 * @param[in,out] foc 电机控制对象，提供三相占空比、周期编号和电机相序。
 * @pre foc 指向快速周期独占写入的有效对象；dtc_a/b/c 已由控制算法限幅。
 *
 * @return 占空比和相序有效且已经写入 Target 时返回 true；否则立即关闭三相
 *         输出、撤销本周期命令有效标志并返回 false。
 *
 * 本函数不计算 SVPWM，也不修改有效占空比。NaN、Inf、越界占空比或非法相序
 * 都不会写入比较寄存器。
 */
bool drive_pwm_commit(foc_t *foc);

#endif /* AXDR_DRIVE_PWM_H */
