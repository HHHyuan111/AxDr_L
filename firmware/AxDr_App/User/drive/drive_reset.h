/**
 * @file drive_reset.h
 * @brief Drive 控制运行状态复位接口。
 */

#ifndef DRIVE_RESET_H
#define DRIVE_RESET_H

#include "foc_fwd.h"

/**
 * @brief 清除一次运行留下的控制器历史和内部给定。
 *
 * @param[in,out] foc 电机控制对象。
 *
 * 保留电机参数、保护配置、命令、位置和当前物理反馈；清除 PID 历史、级联分频、
 * 内部控制给定和上一拍调制结果。本函数不访问硬件。
 */
void drive_control_reset(foc_t *foc);

#endif /* DRIVE_RESET_H */
