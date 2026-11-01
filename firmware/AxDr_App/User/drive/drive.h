/**
 * @file drive.h
 * @brief 电机 Drive 层快速状态入口。
 */

#ifndef AXDR_DRIVE_H
#define AXDR_DRIVE_H

#include <stdbool.h>

#include "foc_fwd.h"

typedef struct drive_cycle drive_cycle_t;

/* 下列四步由 control_cycle_step 依次调用；内部上下文定义见 drive_cycle.h。 */
void drive_check_protection(foc_t *foc);
void drive_update_state(foc_t *foc);
void drive_handle_state(foc_t *foc, drive_cycle_t *cycle);
void drive_output(foc_t *foc, drive_cycle_t *cycle);

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

#endif /* AXDR_DRIVE_H */
