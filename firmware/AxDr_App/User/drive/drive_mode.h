/**
 * @file drive_mode.h
 * @brief Drive 运行模式分派接口。
 */

#ifndef AXDR_DRIVE_MODE_H
#define AXDR_DRIVE_MODE_H

#include <stdbool.h>

#include "motor_fwd.h"

/**
 * @brief 执行当前选定的 PMSM 控制、标定或辨识模式。
 *
 * @param[in,out] pm 电机控制对象。
 * @pre pm 指向快速周期独占写入的有效对象，Drive 已确认当前允许运行。
 *
 * @return 当前模式存在可执行实现时返回 true；模式无效、未实现或本周期控制计算
 *         失败时返回 false。返回 false 后由 Drive 统一关闭功率输出。
 *
 * 本函数只按 mode 分派到现有实现，不决定 START/STOP/FAULT 状态。
 */
bool drive_mode_step(pmsm_t *pm);

#endif /* AXDR_DRIVE_MODE_H */
