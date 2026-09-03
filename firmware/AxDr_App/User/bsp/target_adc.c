/**
 * @file target_adc.c
 * @brief STM32G474 电机控制 ADC 原始结果读取层。
 *
 * 数据流：ADC 寄存器和 DMA 缓冲区 -> target_adc_raw_t -> FOC 层。
 * 本文件拥有原始 ADC 硬件读取，FOC 层只接收带有明确含义的原始计数值。
 */

#include "target_adc.h"

#include "adc.h"
#include "compiler.h"
#include "tim.h"

#define TARGET_ADC1_DMA_COUNT (2U)
#define TARGET_ADC2_DMA_COUNT (4U)
#define TARGET_ADC_TRIG_COUNT (3900U)

/* DMA 缓冲区属于当前 ADC Target，其他模块不直接读取或修改。 */
static uint16_t adc1_dma[TARGET_ADC1_DMA_COUNT];
static uint16_t adc2_dma[TARGET_ADC2_DMA_COUNT];

bool target_adc_start(void)
{
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
    {
        return false;
    }
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK)
    {
        return false;
    }
    if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK)
    {
        return false;
    }
    if (HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK)
    {
        return false;
    }
    if (HAL_ADC_Start_DMA(&hadc1,
                          (uint32_t *)adc1_dma,
                          TARGET_ADC1_DMA_COUNT) != HAL_OK)
    {
        return false;
    }
    if (HAL_ADC_Start_DMA(&hadc2,
                          (uint32_t *)adc2_dma,
                          TARGET_ADC2_DMA_COUNT) != HAL_OK)
    {
        return false;
    }
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK)
    {
        return false;
    }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, TARGET_ADC_TRIG_COUNT);
    return true;
}

/*
 * .RamFunc 告诉链接器把本函数放到 RAM 中运行。三相电流读取同时用于零偏采集
 * 和快速控制链路，这里保持原来直接读取 ADC 寄存器时的执行位置。
 */
PLATFORM_FAST_CODE
void target_adc_read_iabc_raw(target_adc_abc_raw_t *iabc)
{
    /* ADC1 注入 Rank3/2/1 分别连接驱动板 IA/IB/IC 采样网络。 */
    iabc->a = (uint16_t)ADC1->JDR3;
    iabc->b = (uint16_t)ADC1->JDR2;
    iabc->c = (uint16_t)ADC1->JDR1;
}

/* 完整采样函数位于快速控制链路，因此也放在 RAM 中运行。 */
PLATFORM_FAST_CODE
void target_adc_read_raw(target_adc_raw_t *adc_raw)
{
    /* 第 1 步：读取 ADC1 注入组的 IA、IB、IC 原始计数值。 */
    target_adc_read_iabc_raw(&adc_raw->i);

    /* 第 2 步：ADC2 DMA 序号 1/2/3 分别对应驱动板 VA/VB/VC。 */
    adc_raw->v.a = adc2_dma[1];
    adc_raw->v.b = adc2_dma[2];
    adc_raw->v.c = adc2_dma[3];

    /* 第 3 步：读取 ADC2 注入组中的直流母线电压原始计数值。 */
    adc_raw->vbus = (uint16_t)ADC2->JDR1;
}
