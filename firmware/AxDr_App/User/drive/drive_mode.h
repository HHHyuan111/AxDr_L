/**
 * @file drive_mode.h
 * @brief Drive 运行模式分派接口。
 */

#ifndef AXDR_DRIVE_MODE_H
#define AXDR_DRIVE_MODE_H

#include <stdbool.h>

#include "motor_fwd.h"

/**
 * @brief 判断当前模式是否已经接入正式 Drive 运行链。
 *
 * @param[in] pm 电机控制对象，只读取模式选择。
 * @return 已完成运行链接入的模式返回 true，否则返回 false。
 *
 * 返回 false 不代表对应源码被删除，只表示该模式尚未达到上电运行条件。
 */
bool drive_mode_is_supported(const pmsm_t *pm);

/**
 * @brief 在启动或运行模式前校验模式，并准备发布命令。
 *
 * @param[in,out] pm 电机控制对象；发布模式成功时更新内部控制给定。
 * @return 当前模式可以运行且所需命令有效时返回 true，否则返回 false。
 */
bool drive_mode_prepare(pmsm_t *pm);

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
