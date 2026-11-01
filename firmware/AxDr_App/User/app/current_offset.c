/**
 * @file current_offset.c
 * @brief 上电阻塞采集三相电流零偏。主链位置：主程序初始化（快环放行前）。
 */

#include "current_offset.h"

#include "board_adapter.h"
#include "common.h"
#include "target_adc.h"

#include "main.h" /* HAL_Delay */

/**
 * @brief 采集三路电流采样链的零偏平均值。
 *
 * 电机未进入闭环控制前，本函数连续读取 1000 次原始电流采样值，每次间隔 1 ms，
 * 然后根据 ABC 或 ACB 接线关系，把三条物理采样链的平均值保存为 A、B、C 相零偏。
 */
void cur_offset_init(foc_t *foc)
{
    float ia_sum = 0.0f;
    float ib_sum = 0.0f;
    float ic_sum = 0.0f;

    for (int i = 0; i < 1000; i++)
    {
        target_adc_abc_raw_t raw;
        target_adc_abc_raw_t phase;

        HAL_Delay(1);
        target_adc_read_iabc_raw(&raw);
        if (!board_phase_map(foc->motor.phase_order, &raw, &phase))
        {
            return;
        }

        ia_sum += (float)phase.a;
        ib_sum += (float)phase.b;
        ic_sum += (float)phase.c;
    }

    /* 1000 次采样的平均值。 */
    foc->adc.ia_off = ia_sum * 0.001f;
    foc->adc.ib_off = ib_sum * 0.001f;
    foc->adc.ic_off = ic_sum * 0.001f;
}
