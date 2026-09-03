/**
 * @file position_adapter.c
 * @brief 正式单编码器机械角、电角度和多圈位置更新。
 */

#include "common.h"

/**
 * @brief 按当前正式配置读取本周期编码器原始值。
 *
 * @param[in,out] pos_box 位置反馈配置和编码器数据对象。
 * @return 单 MA732 或 MT6816 配置且读取成功返回 true，其他配置返回 false。
 */
_RAM_FUNC bool encoder_sample(pos_box_t *pos_box)
{
    if (pos_box->pos_mode != Sensorsory_s)
    {
        return false;
    }

    switch (pos_box->sensory1)
    {
        case ENCODER_TYPE_MA732:
            return read_ma732_raw(&pos_box->ma732);

        case ENCODER_TYPE_MT6816:
            return read_mt6816_raw(&pos_box->mt6816);

        default:
            return false;
    }
}

/**
 * @brief 使用本周期新编码器数据更新电角度及单圈、多圈机械位置。
 *
 * @param[in,out] pm 电机对象；输入主编码器原始值，输出控制周期位置反馈。
 * @return 存在有效的新位置样本返回 true，否则不更新位置并返回 false。
 */
_RAM_FUNC bool position_update(pmsm_t *pm)
{
    enc_para_t *enc;

    if (pm->pos_box.pos_mode != Sensorsory_s)
    {
        return false;
    }

    switch (pm->pos_box.sensory1)
    {
        case ENCODER_TYPE_MA732:
            enc = &pm->pos_box.ma732;
            break;

        case ENCODER_TYPE_MT6816:
            enc = &pm->pos_box.mt6816;
            break;

        default:
            return false;
    }

    if (enc->rev_flag == 0U)
    {
        return false;
    }

    enc->rev_flag = 0U;
    encoder_update_angle(enc);
    pm->foc.e_pr = enc->pos;
    position_update_single_encoder(pm);

    pm->pos_box.raw_1 = enc->raw;
    pm->pos_box.bit_1 = enc->bit;
    pm->pos_box.pos_1 = enc->pos;
    return true;
}

/**
 * @brief 根据单圈转子角计算电角度、累计转子位置和输出轴位置。
 *
 * @param[in,out] pm 电机对象；输入 e_pr 和电机参数，更新 p_e、mp_r、mp_m 等反馈。
 */
_RAM_FUNC void position_update_single_encoder(pmsm_t *pm)
{
    pmsm_foc_t *foc = &pm->foc;
    const pmsm_para_t *motor = &pm->para;

    foc->pr_dif = foc->e_pr - foc->pr_lst;
    if (foc->pr_dif > (0.88f * M_2PI))
    {
        foc->rev--;
    }
    if (foc->pr_dif < (-0.88f * M_2PI))
    {
        foc->rev++;
    }

    foc->e_pe = foc->e_pr * motor->pn
        - (uint32_t)(foc->e_pr * motor->pnd_2pi) * M_2PI
        + motor->e_off;
    wrap_0_2pi(foc->e_pe);

    foc->p_e = foc->e_pr * motor->pn
        - (uint32_t)(foc->e_pr * motor->pnd_2pi) * M_2PI
        + motor->e_off;
    wrap_0_2pi(foc->p_e);

    foc->sp_r = foc->e_pr + motor->r_off;
    wrap_0_2pi(foc->sp_r);
    foc->mp_r = foc->e_pr + (float)foc->rev * M_2PI + motor->r_off;

    foc->sp_m = foc->sp_r * motor->div_Gr + motor->m_off;
    wrap_0_2pi(foc->sp_m);
    foc->mp_m = foc->mp_r * motor->div_Gr;
    foc->m_rev = (int32_t)(foc->mp_m * div_M_2PI);
    foc->pr_lst = foc->e_pr;
}
