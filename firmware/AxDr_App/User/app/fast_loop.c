/**
 * @file fast_loop.c
 * @brief ADC 中断中的电机控制入口，每 50 us 执行一次（20 kHz）。
 *
 * 执行顺序：调用 FOC 主流程 → 保存观测结果。
 * 电流单位 A，电压单位 V，机械角和电角度单位 rad。
 * 启停、保护和 PWM 提交由 FOC 主流程内的 Drive 处理。
 */

#include "fast_loop.h"

#include <stdbool.h>

#include "common.h"
#include "cycle_record.h"
#include "debug_snapshot.h"

/* 初始化完成前禁止执行控制；主程序只置位一次，中断只读。 */
static volatile bool fast_loop_enabled = false;

/* 由主程序在控制器初始化、参数恢复之后调用；这不等于使能功率输出。 */
void fast_loop_enable(void)
{
    fast_loop_enabled = true;
}

_RAM_FUNC void fast_loop_step(foc_t *foc)
{
    /* 零偏校准期间 ADC 已运行，但控制器还未就绪。 */
    if (!fast_loop_enabled)
    {
        return;
    }

    /* 采样、控制和观测共用本周期编号。 */
    foc->fast_seq++;

    foc_run(foc);

    /* 保存控制后的结果。周期记录仅在人工启用时保留最近 32 拍。 */
    debug_snapshot_publish(foc);
    cycle_record_publish(foc);
}
