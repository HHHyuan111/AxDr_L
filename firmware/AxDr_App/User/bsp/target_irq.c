/**
 * @file target_irq.c
 * @brief STM32G474 控制周期中断到应用快速周期的连接点 + 中断优先级表。
 */

#include "compiler.h"
#include "fast_loop.h"
#include "foc.h"
#include "main.h"
#include "service_telemetry.h"

/*
 * 中断优先级表（冻结，15 号规范）。
 * ARM Cortex-M：数值越小优先级越高；G4 共 16 级，ST HAL 默认全抢占分组。
 * 背景：B 库曾因 USB(2) 抢占 ADC(0) 导致 119 次 NOCP FPU 异常（其 docs/
 * FPU异常与中断嵌套栈核查）——本表保证低优先级永远无法打断 RT。
 *
 * | IRQn            | 抢占级 | 角色                    |
 * |-----------------|-------|-------------------------|
 * | ADC1_2          | 0     | RT：20kHz 控制管线       |
 * | TIM1_UP_TIM16   | 1     | MID：速度环等分频任务    |
 * | USART3 / USB    | 2     | 命令源与遥测             |
 * | TIM3            | 5     | ABZ Z 脉冲（仅诊断）     |
 * | SysTick         | 15    | HAL 时基（最低）         |
 */
void target_irq_priority_apply(void)
{
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
    HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 1, 0);
    HAL_NVIC_SetPriority(USART3_IRQn, 2, 0);
    HAL_NVIC_SetPriority(USB_LP_IRQn, 2, 0);
    HAL_NVIC_SetPriority(TIM3_IRQn, 5, 0);
    HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/**
 * @brief ADC1 注入转换完成后执行一次快速控制周期。
 *
 * HAL 回调属于 STM32 Target。应用层入口 `fast_loop_step()` 不再依赖 HAL 类型，
 * 更换 MCU 时只需在新 Target 的控制周期中断中调用同一入口。
 */
PLATFORM_FAST_CODE void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    /* ADC1 才是三相电流快环源；ADC2 的注入母线采样不启用中断。
     * 即使底层 HAL 配置被误改为 JEOC，也必须拒绝在序列未完成时
     * 使用半组 JDR 数据。ADC1 已固定为 ADC_EOC_SEQ_CONV。 */
    if ((hadc == NULL) || (hadc->Instance != ADC1) ||
        (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_JEOS) == 0U))
    {
        return;
    }

    fast_loop_step(&g_foc);
    /* 遥测抽取（20kHz 分频组帧，S4）。组装缝：与 fast_loop_step 同为先例——
     * 本文件是快速路径的装配点，service 不反向依赖 bsp。 */
    service_telemetry_capture();
}
