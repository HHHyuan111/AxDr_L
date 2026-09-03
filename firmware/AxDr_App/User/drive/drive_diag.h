/**
 * @file drive_diag.h
 * @brief Drive 状态机与可移植诊断算法之间的适配接口。
 */

#ifndef AXDR_DRIVE_DIAG_H
#define AXDR_DRIVE_DIAG_H

#include <stdbool.h>

#include "foc_fwd.h"

struct diag_runtime;

/** 独立于电机控制对象的诊断运行对象，便于调试器直接观察。 */
extern struct diag_runtime g_diag;

/** 使用当前电机和主编码器参数初始化诊断运行对象。 */
void drive_diag_init(const foc_t *foc);

/** 读取调试器写入的诊断请求，并转换成普通 Drive 启停请求。 */
void drive_diag_poll_request(foc_t *foc);

/** 当前模式是否选择了已经接入的参数辨识入口。 */
bool drive_diag_is_supported(const foc_t *foc);

/** 在 PWM 关闭时启动选定的诊断任务。 */
bool drive_diag_prepare(foc_t *foc);

/** 用本周期反馈推进一次算法，并通过现有 FOC/PWM 链执行输出。 */
bool drive_diag_step(foc_t *foc);

/** PWM 确认关闭后，结束并释放当前诊断任务。 */
void drive_diag_on_stopped(void);

/** Drive 进入故障时中止当前诊断任务。 */
void drive_diag_on_fault(void);

#endif /* AXDR_DRIVE_DIAG_H */
