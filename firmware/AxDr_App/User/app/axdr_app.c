/**
 * @file axdr_app.c
 * @brief AxDr 固件应用层快速控制入口。
 *
 * 数据流：ADC 周期中断 -> 编码器与 ADC 采样 -> 反馈更新 -> 电机状态机。
 * 本文件只负责安排调用顺序，暂时继续使用现有 pmsm_t 控制对象和原有控制函数。
 */

#include "axdr_app.h"

#include "common.h"
#include "modlue.h"

_RAM_FUNC void axdr_app_fast_step(void)
{
    /* 第 1 步：读取编码器，并更新机械角、电角度和多圈位置。 */
    encoder_sample(&pm.pos_box);
    position_update(&pm);

    /* 第 2 步：读取 ADC 原始值，并换算本周期三相电流。 */
    foc_adc_sample(&pm);

    /* 第 3 步：更新母线电压、控制限幅、转矩和速度反馈。 */
    foc_feedback_update(&pm);

    /* 第 4 步：运行状态机和当前选定的控制模式。 */
    pmsm_run_state_machine(&pm);
}

/**
 * @brief ADC 注入转换完成回调。
 *
 * ADC1 每完成一组注入采样便进入这里。本回调不再展开控制流程，只把本次控制周期
 * 交给应用层统一入口。当前工程只有 ADC1 使用注入完成中断，因此保持原有触发行为。
 */
_RAM_FUNC void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
    axdr_app_fast_step();
}
