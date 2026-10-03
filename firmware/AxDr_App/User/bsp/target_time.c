/**
 * @file target_time.c
 * @brief DWT->CYCCNT 微秒时基实现。
 * @note  核心停则计数停（调试友好）；SystemCoreClock 变化后需重新调用 init。
 */

#include "target_time.h"

#include "main.h"

#define CYCLES_PER_US_DIV 1000000U

void target_time_init(void)
{
    /* ARM CMSIS 官方流程：DEMCR.TRCENA 使能调试组件，再开 CYCCNT */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t target_time_us(void)
{
    return DWT->CYCCNT / (SystemCoreClock / CYCLES_PER_US_DIV);
}

void target_time_delay_us(uint32_t us)
{
    /* 按周期差忙等：无符号减法天然容忍 CYCCNT 回绕 */
    uint32_t start_cycles = DWT->CYCCNT;
    uint32_t wait_cycles = us * (SystemCoreClock / CYCLES_PER_US_DIV);

    while ((DWT->CYCCNT - start_cycles) < wait_cycles)
    {
        /* 忙等 */
    }
}
