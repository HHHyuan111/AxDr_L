/**
 * @file axdr_app.c
 * @brief AxDr 固件应用层快速控制入口。
 *
 * 数据流：ADC 周期中断 -> 编码器与 ADC 采样 -> 反馈更新 -> Drive -> 调试快照。
 * 本文件只负责安排调用顺序，暂时继续使用现有 pmsm_t 控制对象和原有控制函数。
 */

#include "axdr_app.h"

#include <stdbool.h>

#include "common.h"
#include "debug_snapshot.h"
#include "drive.h"
#include "modlue.h"

/*
 * 主初始化流程只把本标志从 false 写为 true 一次，ADC 中断只读取它。
 * volatile 保证中断每次都读取最新值；它不承担通用的多线程同步功能。
 */
static volatile bool fast_control_ready = false;

void axdr_app_start_fast_control(void)
{
    fast_control_ready = true;
}

_RAM_FUNC void axdr_app_fast_step(void)
{
    /*
     * ADC 和 TIM1 通道 4 必须先运行，电流零偏校准才能取得持续更新的采样值；
     * 但 pmsm_init() 完成以前，不能让中断访问正在初始化的 pm 对象。
     */
    if (!fast_control_ready)
    {
        return;
    }

    /* 第 1 步：读取编码器，并更新机械角、电角度和多圈位置。 */
    encoder_sample(&pm.pos_box);
    position_update(&pm);

    /* 第 2 步：读取 ADC 原始值，并换算本周期三相电流。 */
    foc_adc_sample(&pm);

    /* 第 3 步：更新母线电压、控制限幅、转矩和速度反馈。 */
    foc_feedback_update(&pm);

    /* 第 4 步：运行状态机和当前选定的控制模式。 */
    drive_fast_step(&pm);

    /* 第 5 步：复制本周期最终结果，仅供调试器观察，不参与控制。 */
    debug_snapshot_publish(&pm);
}

/**
 * @brief ADC 注入转换完成回调。
 *
 * HAL 报告一次 ADC1 注入转换完成时进入这里。本回调不再展开控制流程，只把本次
 * 回调交给应用层统一入口。当前节点保持原有 ADC EOC 配置和回调触发行为。
 */
_RAM_FUNC void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
    axdr_app_fast_step();
}
