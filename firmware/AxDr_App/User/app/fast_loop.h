/**
 * @file fast_loop.h
 * @brief 电机快速控制周期入口。
 *
 * 本模块负责把板级中断与现有电机控制流程连接起来。硬件中断只通知应用层“新的
 * 控制周期已经到来”，具体的采样、反馈更新和控制执行顺序由应用层统一安排。
 */

#ifndef FAST_LOOP_H
#define FAST_LOOP_H

#include "motor_fwd.h"

/**
 * @brief 允许 ADC 中断开始执行快速电机控制。
 *
 * 上电后快速控制默认处于未放行状态。主初始化流程完成电机参数初始化和电流零偏
 * 校准后调用本函数；从下一次 ADC 注入完成中断开始，才会执行完整控制周期。
 */
void fast_loop_enable(void);

/**
 * @brief 执行一次快速电机控制周期。
 *
 * @param[in,out] motor 本周期独占写入的电机控制对象。
 * @pre motor 指向已初始化的有效对象。
 *
 * 本函数由 ADC 注入转换完成中断调用，依次完成编码器采样、ADC 采样、反馈更新和
 * 电机状态机执行。初始化尚未完成时，本函数直接返回，不访问电机控制对象。
 */
void fast_loop_step(pmsm_t *motor);

#endif /* FAST_LOOP_H */
