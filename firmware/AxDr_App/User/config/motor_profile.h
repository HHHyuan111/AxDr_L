/**
 * @file motor_profile.h
 * @brief 电机档案装载：按 MOTOR_SELECTED_MODEL 把配置宏装入 foc_t（值的家在配置层）。
 * @note  本文件不依赖任何硬件头，可主机编译测试（12 号参数规范）。
 */

#ifndef MOTOR_PROFILE_H
#define MOTOR_PROFILE_H

#include "common.h"

/**
 * @brief 装载物理参数、零位、应用限幅与环路整定参数。
 * @note  须在 ctrl_rate/ctrl_filter/cur_pi/spd_pi 初始化之前调用（提供 ibw 等）。
 */
void motor_profile_load(foc_t *foc);

/**
 * @brief 装载晚绑定项：保护阈值覆盖与速度环固化 Kp/Ki（须在 prot_cfg_init 与
 *        spd_pi_init 之后调用——它们会覆盖早期赋值，本函数做最终定版）。
 */
void motor_profile_control_load(foc_t *foc);

#endif /* MOTOR_PROFILE_H */
