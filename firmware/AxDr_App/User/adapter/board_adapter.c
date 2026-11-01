/**
 * @file board_adapter.c
 * @brief 板卡 ADC 相序映射和物理量换算实现。
 */

#include "board_adapter.h"

#include "common.h"
#include "compiler.h"

PLATFORM_FAST_CODE bool board_phase_map(phase_order_t order,
                                        const target_adc_abc_raw_t *raw,
                                        target_adc_abc_raw_t *phase)
{
    switch (order)
    {
        case PHASE_ORDER_ABC:
            phase->a = raw->a;
            phase->b = raw->b;
            phase->c = raw->c;
            return true;

        case PHASE_ORDER_ACB:
            phase->a = raw->a;
            phase->b = raw->c;
            phase->c = raw->b;
            return true;

        default:
            return false;
    }
}

PLATFORM_FAST_CODE bool board_adc_convert(const board_adc_cfg_t *cfg,
                                          const target_adc_raw_t *raw,
                                          board_sample_t *sample)
{
    if (!board_phase_map(cfg->phase_order, &raw->i, &sample->i_raw)
        || !board_phase_map(cfg->phase_order, &raw->v, &sample->v_raw))
    {
        return false;
    }

    sample->v_bus_raw = raw->vbus;
    sample->ia = ((float)sample->i_raw.a - cfg->i_offset_a) * cfg->i_scale;
    sample->ib = ((float)sample->i_raw.b - cfg->i_offset_b) * cfg->i_scale;
    sample->ic = ((float)sample->i_raw.c - cfg->i_offset_c) * cfg->i_scale;
    sample->vbus = (float)sample->v_bus_raw * cfg->v_scale;

    return true;
}

PLATFORM_FAST_CODE bool foc_adc_sample(foc_t *foc, board_sample_t *sample)
{
    const board_adc_cfg_t cfg = {
        .phase_order = foc->motor.phase_order,
        .i_scale = foc->board.i_ratio,
        .v_scale = foc->board.v_ratio,
        .i_offset_a = foc->adc.ia_off,
        .i_offset_b = foc->adc.ib_off,
        .i_offset_c = foc->adc.ic_off,
    };
    target_adc_raw_t raw;

    /* 1. 读取已完成的采样；ADC 触发和 DMA 配置由 BSP 负责。 */
    target_adc_read_raw(&raw);

    /* 2. 相序映射、扣除零偏，得到 A 和 V 为单位的本拍结果。 */
    if (!board_adc_convert(&cfg, &raw, sample))
    {
        return false;
    }

    /* 3. 原始计数留作诊断，控制反馈由 control_cycle_step 装入。 */
    foc->adc.raw.ia = sample->i_raw.a;
    foc->adc.raw.ib = sample->i_raw.b;
    foc->adc.raw.ic = sample->i_raw.c;
    foc->adc.raw.va = sample->v_raw.a;
    foc->adc.raw.vb = sample->v_raw.b;
    foc->adc.raw.vc = sample->v_raw.c;
    foc->adc.raw.vbus = sample->v_bus_raw;

    return true;
}
