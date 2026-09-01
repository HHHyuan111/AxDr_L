/**
 * @file drive_diag.h
 * @brief Drive 状态机与可移植诊断算法之间的适配接口。
 */

#ifndef AXDR_DRIVE_DIAG_H
#define AXDR_DRIVE_DIAG_H

#include <stdbool.h>

#include "motor_fwd.h"

/** 读取调试器写入的诊断请求，并转换成普通 Drive 启停请求。 */
void drive_diag_poll_request(pmsm_t *pm);

/** 当前模式是否选择了已经接入的参数辨识入口。 */
bool drive_diag_is_supported(const pmsm_t *pm);

/** 在 PWM 关闭时启动选定的诊断任务。 */
bool drive_diag_prepare(pmsm_t *pm);

/** 用本周期反馈推进一次算法，并通过现有 FOC/PWM 链执行输出。 */
bool drive_diag_step(pmsm_t *pm);

/** PWM 确认关闭后，结束并释放当前诊断任务。 */
void drive_diag_on_stopped(pmsm_t *pm);

/** Drive 进入故障时中止当前诊断任务。 */
void drive_diag_on_fault(pmsm_t *pm);

#endif /* AXDR_DRIVE_DIAG_H */
