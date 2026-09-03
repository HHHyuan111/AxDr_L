/**
 * @file target_adc.h
 * @brief 电机控制所需 ADC 原始采样的板级适配接口。
 *
 * 输入：ADC1 注入组结果、ADC2 注入组结果和 ADC2 规则组 DMA 缓冲区。
 * 输出：按板上 IA/IB/IC、VA/VB/VC 和 VBUS 物理采样网络排列的 ADC 原始计数值。
 * 调用时机：ADC1 注入转换完成后的快速控制回调，以及启动时的电流零偏采集。
 * 模块边界：只读取已经完成的 ADC 结果，不触发转换，不处理相序，不扣除零偏，
 *           也不把原始计数换算成安培或伏特。
 */

#ifndef TARGET_ADC_H
#define TARGET_ADC_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 启动控制周期所需的 ADC 采样链。
 *
 * @return ADC 校准、注入组、DMA 和 TIM1 CH4 采样触发全部启动成功时返回 true。
 *
 * 本函数在电机对象初始化前调用，使零偏采集能够读取持续更新的 ADC 结果。
 * 它只建立采样链，不放行快速控制，也不启动三相功率 PWM。
 */
bool target_adc_start(void);

/**
 * @brief A、B、C 三相 ADC 原始值。
 *
 * 该类型既可表示三相电流 i，也可表示三相电压 v。类型名已经说明数据来自 ADC
 * 且尚未换算，因此字段只保留标准的 a、b、c 三相记号。
 */
typedef struct
{
    uint16_t a; /* A 相 ADC 原始值。 */
    uint16_t b; /* B 相 ADC 原始值。 */
    uint16_t c; /* C 相 ADC 原始值。 */
} target_adc_abc_raw_t;

/**
 * @brief 一次电机控制过程当前使用的 ADC 原始计数值。
 */
typedef struct
{
    target_adc_abc_raw_t i; /* 三相电流：i.a、i.b、i.c 对应 IA、IB、IC。 */
    target_adc_abc_raw_t v; /* 三相电压：v.a、v.b、v.c 对应 VA、VB、VC。 */

    uint16_t vbus; /* 直流母线电压 VBUS 的 ADC 原始值。 */
} target_adc_raw_t;

/**
 * @brief 只读取 IA、IB、IC 三相电流采样网络的 ADC 原始计数值。
 *
 * @param[out] iabc 接收 IA、IB、IC 原始值，调用者必须传入有效地址。
 *
 * 启动时的零偏采集只需要电流数据，因此使用该接口，不额外读取相电压和母线电压。
 */
void target_adc_read_iabc_raw(target_adc_abc_raw_t *iabc);

/**
 * @brief 读取电机控制当前可用的 ADC 原始结果。
 *
 * @param[out] adc_raw 接收原始 ADC 值的结构体，调用者必须传入有效地址。
 *
 * 本函数只是把硬件寄存器和 DMA 缓冲区中的现有结果复制出来，不会启动新转换，
 * 也不会等待下一次转换完成。当前保持原工程的数据来源和读取顺序，不宣称 ADC1、
 * ADC2 注入组和 ADC2 DMA 数据已经经过硬件同拍验证。
 */
void target_adc_read_raw(target_adc_raw_t *adc_raw);

#endif /* TARGET_ADC_H */
