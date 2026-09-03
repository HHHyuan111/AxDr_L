/**
 * @file board_adapter.c
 * @brief 板卡 ADC 相序映射和物理量换算实现。
 */

#include "board_adapter.h"

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
