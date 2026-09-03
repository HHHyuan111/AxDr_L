/**
 * @file board_adapter.h
 * @brief 板卡 ADC 原始计数到电机三相物理反馈的转换接口。
 */

#ifndef BOARD_ADAPTER_H
#define BOARD_ADAPTER_H

#include <stdbool.h>
#include <stdint.h>

#include "phase_order.h"
#include "target_adc.h"

/** @brief 当前板卡 ADC 换算所需的相序、零偏和比例。 */
typedef struct
{
    phase_order_t phase_order;
    float i_scale; /* 电流换算比例，单位 A/count。 */
    float v_scale; /* 电压换算比例，单位 V/count。 */
    float i_offset_a;
    float i_offset_b;
    float i_offset_c;
} board_adc_cfg_t;

/** @brief 经过相序映射和物理量换算的一拍板卡反馈。 */
typedef struct
{
    target_adc_abc_raw_t i_raw;
    target_adc_abc_raw_t v_raw;
    uint16_t v_bus_raw;
    float ia;
    float ib;
    float ic;
    float vbus;
} board_sample_t;

/**
 * @brief 按电机逻辑相序排列一组三相原始值。
 *
 * @return 相序有效时返回 true；相序非法时返回 false。
 * @pre raw 和 phase 均指向有效且互不重叠的对象。
 */
bool board_phase_map(phase_order_t order,
                     const target_adc_abc_raw_t *raw,
                     target_adc_abc_raw_t *phase);

/**
 * @brief 把 Target ADC 原始结果转换为电机控制使用的反馈。
 *
 * @param[in] cfg 当前板卡换算配置。
 * @param[in] raw 当前 Target ADC 原始计数。
 * @param[out] sample 映射后的原始值和物理量结果。
 * @return 相序有效并完成换算时返回 true，否则返回 false。
 * @pre 三个参数均指向有效且互不重叠的对象。
 */
bool board_adc_convert(const board_adc_cfg_t *cfg,
                       const target_adc_raw_t *raw,
                       board_sample_t *sample);

#endif /* BOARD_ADAPTER_H */
