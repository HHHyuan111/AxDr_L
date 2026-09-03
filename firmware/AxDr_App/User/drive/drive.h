/**
 * @file drive.h
 * @brief 电机 Drive 层快速状态入口。
 */

#ifndef AXDR_DRIVE_H
#define AXDR_DRIVE_H

#include <stdbool.h>

#include "foc_fwd.h"

/**
 * @brief 在功率输出已经关闭时清除 Drive 锁存故障。
 *
 * @param[in,out] foc 电机控制对象。
 * @return PWM 已关闭并完成清除时返回 true；PWM 仍活动时返回 false。
 *
 * 清除后 Drive 回到 STOP，控制器历史状态复位。故障源如果仍存在，后续 START
 * 会在保护链中再次锁存故障。本函数不启动或停止硬件。
 */
bool drive_fault_clear(foc_t *foc);

/**
 * @brief 执行一次 Drive 状态与模式调度。
 *
 * @param[in,out] foc 电机控制对象。
 *
 * @pre foc 必须指向已经初始化、由当前快速周期独占写入的有效对象。
 *
 * 本函数由 20 kHz 快速周期调用。START 请求只执行一次启动动作，随后自动转为
 * RUN 请求；STOP 和 FAULT 会关闭三相 PWM。函数不改变 FOC 公式和控制参数。
 */
void drive_fast_step(foc_t *foc);

#endif /* AXDR_DRIVE_H */
