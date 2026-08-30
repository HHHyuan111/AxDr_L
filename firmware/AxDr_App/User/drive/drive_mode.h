/**
 * @file drive_mode.h
 * @brief Drive 运行模式分派接口。
 */

#ifndef AXDR_DRIVE_MODE_H
#define AXDR_DRIVE_MODE_H

#include "common.h"

/**
 * @brief 执行当前选定的 PMSM 控制、标定或辨识模式。
 *
 * @param[in,out] pm 电机控制对象。
 * @pre pm 指向快速周期独占写入的有效对象，Drive 已确认当前允许运行。
 *
 * 本函数只按 mode 分派到现有实现，不决定 START/STOP/FAULT 状态。
 */
void drive_mode_step(pmsm_t *pm);

#endif /* AXDR_DRIVE_MODE_H */
