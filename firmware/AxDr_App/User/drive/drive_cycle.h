/** @file drive_cycle.h
 *  @brief 快速周期内部的运行状态和本拍计算结果。
 */
#ifndef DRIVE_CYCLE_H
#define DRIVE_CYCLE_H

#include "drive.h"
#include "foc_run.h"

/* 只保存本拍给定与计算结果；运行状态只保存在 foc->state。 */
struct drive_cycle
{
    foc_plan_t cmd;
    bool ready; /* 是否有本拍控制给定。 */
    bool valid; /* 控制计算是否成功，不等于已提交硬件。 */
};

#endif
