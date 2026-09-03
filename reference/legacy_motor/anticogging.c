//
// Created by disno on 2025/9/18.
//

#include "main.h"
#include "common.h"
#include "drive_pwm.h"

void anticog_init(foc_t *foc)
{
    foc->anticog.map_num = 3840;

    foc->anticog.wr_set = 10.0f;
    foc->anticog.step_value = 800;
    foc->anticog.gap_number = 100;

    foc->anticog.delta_p = M_2PI / (float)foc->anticog.map_num;
}

_RAM_FUNC void anticogging_calibration(foc_t *foc)
{
    anticog_t *x = &foc->anticog;
    x->wr_set = 10.0f; // 转速，建议不要太大
    foc->ctrl.posr_set = x->posr_set;
    foc->ctrl.wr_set   = x->wr_set;
    foc->ctrl.iq_set   = 0.0f;

    if (foc_pos_step(foc,
                foc->ctrl.posr_set,
                foc->ctrl.wr_set,
                foc->ctrl.iq_set,
                foc->sig.p_e)) {
        drive_pwm_commit(foc);
    }

    if (++x->count < 700) {
        return;
    }
    x->count = 0;
    x->number++;

    if (x->number <= x->gap_number)
    {
        // 阶段1：预旋转（CW）
        x->posr_set += x->delta_p;
    }
    else if (x->number <= (x->gap_number + x->map_num))
    {
        // 阶段2：CW 扫描
        float pos_mod = fmodf(x->posr_set, M_2PI);
        if (pos_mod < 0.0f) pos_mod += M_2PI;

        uint16_t index = (uint16_t)(pos_mod * div_M_2PI * (float)x->map_num + 0.5f);
        if (index >= x->map_num) index = 0;

        foc->map.aco_table[index] = foc->sig.i_q; // 记录正向电流
        foc->map.aco_lut = foc->sig.i_q;

        x->posr_set += x->delta_p;
    }
    else
    {
        x->wr_set = 0.0f;  // 停止电机
        foc->flag.bit.anticog_done = 1; // 置完成标志
        foc->req = DRIVE_REQ_STOP;
        foc->mode.sys = release_mode;
    }
}


//     // 计算 index
//     float index_f = foc->sig.sp_r * div_M_2PI * (float)COGGING_MAP_NUM;
//     uint16_t index0 = (uint16_t)index_f;
//     // 将补偿电流加入目标
//     foc->ctrl.iq_lim += map[index0];
