/**
 * @file target_qenc.c
 * @brief TIM3 正交编码器接口实现（配置值全部迁自沉沙ABZ固件已验证代码）。
 *
 * 硬件事实（AxDrive-L，HALL 接口）：PA6=A / PA7=B / PB0=Z，板上 1k 分压网络。
 * 参数：TI12 双边沿四倍频、PSC=0、ARR=9999（10000 计数/圈）、IC 滤波器=15（B 库实测值）。
 * 方向（dir=-1）与相序的配套属于 device 层语义，本文件只管硬件原始计数。
 */

#include "target_qenc.h"

#include "main.h"
#include "tim.h"

/* 坑③：10000 计数需要 14 位表达；此常量供 device 层换算，本文件不参与位宽裁剪 */
#define QENC_COUNTS_PER_REV 10000U

static volatile uint32_t qenc_z_count = 0U;

void target_qenc_init(void)
{
    TIM_Encoder_InitTypeDef encoder;
    TIM_IC_InitTypeDef ic;

    /* 坑①：本函数必须在 MX_TIM3_Init / MX_ADC2_Init 之后调用（时序由 P3 设备层保证） */
    htim3.Init.Period = 9999U; /* 10000 计数/圈（坑③的来源） */
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    encoder.EncoderMode = TIM_ENCODERMODE_TI12;
    encoder.IC1Polarity = TIM_ICPOLARITY_RISING;
    encoder.IC2Polarity = TIM_ICPOLARITY_RISING;
    encoder.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    encoder.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    encoder.IC1Prescaler = TIM_ICPSC_DIV1;
    encoder.IC2Prescaler = TIM_ICPSC_DIV1;
    encoder.IC1Filter = 15U; /* B 库实测滤波值：抑制 HALL 口毛刺 */
    encoder.IC2Filter = 15U;
    (void)HAL_TIM_Encoder_Init(&htim3, &encoder);

    /* 坑②：CH3 必须显式配置为输入捕获，再以中断方式启动 */
    ic.ICPolarity = TIM_ICPOLARITY_RISING;
    ic.ICSelection = TIM_ICSELECTION_DIRECTTI;
    ic.ICPrescaler = TIM_ICPSC_DIV1;
    ic.ICFilter = 15U;
    (void)HAL_TIM_IC_ConfigChannel(&htim3, &ic, TIM_CHANNEL_3);

    HAL_NVIC_SetPriority(TIM3_IRQn, 5, 0); /* Z 仅诊断：低于 RT(0)/MID(1)，见 15 号优先级表 */
    HAL_NVIC_EnableIRQ(TIM3_IRQn);

    (void)HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    (void)HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
}

int32_t target_qenc_read(void)
{
    /* 坑③：计数值 0..9999（14 位语义），单圈原始值；多圈与方向换算在 device 层 */
    return (int32_t)(__HAL_TIM_GET_COUNTER(&htim3));
}

uint32_t target_qenc_z_count(void)
{
    return qenc_z_count;
}

/**
 * @brief Z 脉冲捕获中断：仅累加诊断计数，不参与控制。
 */
void TIM3_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_CC3) != 0U)
    {
        if (__HAL_TIM_GET_IT_SOURCE(&htim3, TIM_IT_CC3) != 0U)
        {
            __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC3);
            qenc_z_count++;
        }
    }
}
