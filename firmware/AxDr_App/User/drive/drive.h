/**
 * @file drive.h
 * @brief 电机 Drive 层快速状态入口。
 */

#ifndef AXDR_DRIVE_H
#define AXDR_DRIVE_H

#include "common.h"

/**
 * @brief 执行一次 Drive 状态与模式调度。
 *
 * @param[in,out] pm 电机控制对象。
 *
 * @pre pm 必须指向已经初始化、由当前快速周期独占写入的有效对象。
 *
 * 本函数由 20 kHz 快速周期调用。START 请求只执行一次启动动作，随后自动转为
 * RUN 请求；STOP 和 FAULT 会关闭三相 PWM。函数不改变 FOC 公式和控制参数。
 */
void drive_fast_step(pmsm_t *pm);

#endif /* AXDR_DRIVE_H */
