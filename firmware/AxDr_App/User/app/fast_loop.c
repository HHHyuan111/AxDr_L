/**
 * @file fast_loop.c
 * @brief 电机快速控制周期编排。
 *
 * 数据流：ADC 周期中断 -> 编码器与 ADC 采样 -> 反馈更新 -> Drive -> 调试快照。
 * 本文件只负责安排调用顺序，暂时继续使用现有 pmsm_t 控制对象和原有控制函数。
 */

#include "fast_loop.h"

#include <stdbool.h>

#include "common.h"
#include "control_cycle.h"
#include "debug_snapshot.h"

/*
 * 主初始化流程只把本标志从 false 写为 true 一次，ADC 中断只读取它。
 * volatile 保证中断每次都读取最新值；它不承担通用的多线程同步功能。
 */
static volatile bool fast_loop_enabled = false;

void fast_loop_enable(void)
{
    fast_loop_enabled = true;
}

_RAM_FUNC void fast_loop_step(pmsm_t *motor)
{
    bool current_valid;
    bool encoder_valid;
    bool position_valid;
    control_cycle_input_t input;
    control_cycle_output_t output;

    /*
     * ADC 和 TIM1 通道 4 必须先运行，电流零偏校准才能取得持续更新的采样值；
     * 但 pmsm_init() 完成以前，不能让中断访问正在初始化的电机对象。
     */
    if (!fast_loop_enabled)
    {
        return;
    }

    /* 本序号标识一次完整快速周期；自然回绕不改变周期先后关系。 */
    motor->fast_seq++;

    /* 第 1 步：读取编码器，并更新机械角、电角度和多圈位置。 */
    encoder_valid = encoder_sample(&motor->pos_box);
    position_valid = encoder_valid && position_update(motor);

    /* 第 2 步：读取 ADC 原始值，并换算本周期三相电流。 */
    current_valid = foc_adc_sample(motor);

    /* 第 3 步：把本周期物理反馈整理为硬件无关的控制输入。 */
    input = (control_cycle_input_t){
        .seq = motor->fast_seq,
        .i_valid = current_valid,
        .vbus_valid = true,
        .pos_valid = position_valid,
        .current_a_a = motor->foc.i_a,
        .current_b_a = motor->foc.i_b,
        .current_c_a = motor->foc.i_c,
        .bus_voltage_v = motor->foc.vbus,
        .electrical_angle_rad = motor->foc.p_e,
        .rotor_position_rad = motor->foc.mp_r,
        .output_position_rad = motor->foc.mp_m,
    };

    /* 第 4 步：使用显式输入执行反馈更新、状态机和当前控制模式。 */
    control_cycle_step(motor, &input, &output);

    /* 第 5 步：复制本周期最终结果，仅供调试器观察，不参与控制。 */
    debug_snapshot_publish(motor);
}
