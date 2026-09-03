/**
 * @file target_irq.c
 * @brief STM32G474 控制周期中断到应用快速周期的连接点。
 */

#include "common.h"
#include "fast_loop.h"
#include "main.h"

/**
 * @brief ADC1 注入转换完成后执行一次快速控制周期。
 *
 * HAL 回调属于 STM32 Target。应用层入口 `fast_loop_step()` 不再依赖 HAL 类型，
 * 更换 MCU 时只需在新 Target 的控制周期中断中调用同一入口。
 */
_RAM_FUNC void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
    fast_loop_step(&pm);
}
