/**
 * @file drive_command.h
 * @brief Drive 发布命令到内部控制给定的适配接口。
 */

#ifndef DRIVE_COMMAND_H
#define DRIVE_COMMAND_H

#include <stdbool.h>

#include "motor_fwd.h"

/**
 * @brief 校验并应用一组发布接口命令。
 *
 * @param[in,out] pm 输入 cmd、限幅和极性，输出 ctrl 中的转矩、速度和位置给定。
 * @return 命令、限幅和极性均有效时返回 true，否则返回 false，且不更新给定。
 *
 * 转矩单位 N·m，速度单位 rad/s，位置单位 rad。函数不访问硬件、不分配内存。
 */
bool drive_cmd_apply(pmsm_t *pm);

#endif /* DRIVE_COMMAND_H */
