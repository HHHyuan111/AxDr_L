/**
 * @file target_pwm.c
 * @brief STM32G474 三相 PWM 硬件执行层。
 *
 * 数据流：通道占空比 -> TIM1 比较值 -> 三相 PWM 引脚。
 * 本文件只操作 TIM1 通道 1、2、3，不参与占空比计算和相序判断。
 */

#include "target_pwm.h"

#include "compiler.h"
#include "tim.h"

bool target_pwm_stop_phase_outputs(void)
{
    bool stopped = true;

    /* 六路停止都要尝试，不能因为前一路失败而跳过其余通道。 */
    stopped &= HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1) == HAL_OK;
    stopped &= HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1) == HAL_OK;
    stopped &= HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2) == HAL_OK;
    stopped &= HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2) == HAL_OK;
    stopped &= HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3) == HAL_OK;
    stopped &= HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3) == HAL_OK;

    return stopped;
}

bool target_pwm_start_phase_outputs(void)
{
    const bool started =
        (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) == HAL_OK) &&
        (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) == HAL_OK) &&
        (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) == HAL_OK) &&
        (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2) == HAL_OK) &&
        (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) == HAL_OK) &&
        (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3) == HAL_OK);

    if (!started)
    {
        (void)target_pwm_stop_phase_outputs();
    }

    return started;
}

/*
 * .RamFunc 告诉链接器把本函数放到 RAM 中运行。快速控制环会频繁更新占空比，
 * 这里继续保留原工程的 RAM 执行方式，不改变实时链路的位置。
 */
PLATFORM_FAST_CODE
void target_pwm_set_duty_ratios(float channel_1_duty_ratio,
                                float channel_2_duty_ratio,
                                float channel_3_duty_ratio)
{
    /* 第 1 步：读取 TIM1 的自动重装值，作为占空比换算使用的周期计数值。 */
    const uint32_t pwm_period_count = __HAL_TIM_GET_AUTORELOAD(&htim1);

    /* 第 2 步：把 0.0f～1.0f 的占空比比例换算成定时器比较计数值。 */
    const uint16_t channel_1_compare_count =
        (uint16_t)(channel_1_duty_ratio * (float)pwm_period_count);
    const uint16_t channel_2_compare_count =
        (uint16_t)(channel_2_duty_ratio * (float)pwm_period_count);
    const uint16_t channel_3_compare_count =
        (uint16_t)(channel_3_duty_ratio * (float)pwm_period_count);

    /* 第 3 步：写入三个比较寄存器，TIM1 随后按当前 PWM 配置产生波形。 */
    htim1.Instance->CCR1 = channel_1_compare_count;
    htim1.Instance->CCR2 = channel_2_compare_count;
    htim1.Instance->CCR3 = channel_3_compare_count;
}
