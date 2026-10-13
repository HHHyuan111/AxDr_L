#ifndef AXDR_HOST_TEST_FAKE_MAIN_H
#define AXDR_HOST_TEST_FAKE_MAIN_H

/* common.h 在电脑端只需要 CMSIS 提供的两个类型限定符。 */
#ifndef __IO
#define __IO volatile
#endif
#ifndef __I
#define __I volatile const
#endif

/* 回调门槛的最小替身：区分 ADC 实例及注入序列完成标志。 */
typedef struct {
    unsigned flags;
} ADC_TypeDef;
typedef struct ADC_HandleTypeDef {
    ADC_TypeDef *Instance;
} ADC_HandleTypeDef;
extern ADC_TypeDef axdr_test_adc1;
#define ADC1 (&axdr_test_adc1)
#define ADC_FLAG_JEOS (1U << 6)
#define __HAL_ADC_GET_FLAG(handle, flag) ((handle)->Instance->flags & (flag))
extern unsigned SystemCoreClock;

/* NVIC 最小替身：仅满足 target_irq.c 主机编译（优先级表逻辑本身不在宿主验证范围）。 */
typedef enum
{
    ADC1_2_IRQn = 0,
    TIM1_UP_TIM16_IRQn,
    USART3_IRQn,
    USB_LP_IRQn,
    TIM3_IRQn,
    SysTick_IRQn = 15,
} IRQn_Type;

static inline void HAL_NVIC_SetPriority(IRQn_Type irqn, unsigned preempt, unsigned sub)
{
    (void)irqn;
    (void)preempt;
    (void)sub;
}

#endif
