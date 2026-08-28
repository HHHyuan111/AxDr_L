#include "target_pwm.h"

#include "tim.h"

void target_pwm_start_phase_outputs(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

void target_pwm_stop_phase_outputs(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}

__attribute__((section(".RamFunc"))) void target_pwm_commit_channel_duty(
    float channel_1_duty,
    float channel_2_duty,
    float channel_3_duty)
{
    htim1.Instance->CCR1 =
        (uint16_t)(channel_1_duty * __HAL_TIM_GET_AUTORELOAD(&htim1));
    htim1.Instance->CCR2 =
        (uint16_t)(channel_2_duty * __HAL_TIM_GET_AUTORELOAD(&htim1));
    htim1.Instance->CCR3 =
        (uint16_t)(channel_3_duty * __HAL_TIM_GET_AUTORELOAD(&htim1));
}
