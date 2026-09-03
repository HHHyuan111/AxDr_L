/**
 * @file position_adapter.c
 * @brief 正式单编码器机械角、电角度和多圈位置更新。
 */

#include "common.h"

/**
 * @brief 按当前正式配置读取本周期编码器原始值。
 *
 * @param[in,out] enc 位置反馈配置和编码器数据对象。
 * @return 单 MA732 或 MT6816 配置且读取成功返回 true，其他配置返回 false。
 */
_RAM_FUNC bool encoder_sample(encoder_state_t *enc)
{
    if (enc->pos_mode != Sensorsory_s)
    {
        return false;
    }

    switch (enc->sensory1)
    {
        case ENCODER_TYPE_MA732:
            return read_ma732_raw(&enc->ma732);

        case ENCODER_TYPE_MT6816:
            return read_mt6816_raw(&enc->mt6816);

        default:
            return false;
    }
}

/**
 * @brief 使用本周期新编码器数据更新电角度及单圈、多圈机械位置。
 *
 * @param[in,out] foc 电机对象；输入主编码器原始值，输出控制周期位置反馈。
 * @return 存在有效的新位置样本返回 true，否则不更新位置并返回 false。
 */
_RAM_FUNC bool position_update(foc_t *foc)
{
    encoder_data_t *enc;

    if (foc->enc.pos_mode != Sensorsory_s)
    {
        return false;
    }

    switch (foc->enc.sensory1)
    {
        case ENCODER_TYPE_MA732:
            enc = &foc->enc.ma732;
            break;

        case ENCODER_TYPE_MT6816:
            enc = &foc->enc.mt6816;
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
    foc->sig.e_pr = enc->pos;
    position_update_single_encoder(foc);

    foc->enc.raw_1 = enc->raw;
    foc->enc.bit_1 = enc->bit;
    foc->enc.pos_1 = enc->pos;
    return true;
}

/**
 * @brief 根据单圈转子角计算电角度、累计转子位置和输出轴位置。
 *
 * @param[in,out] foc 电机对象；输入 e_pr 和电机参数，更新 p_e、mp_r、mp_m 等反馈。
 */
_RAM_FUNC void position_update_single_encoder(foc_t *foc)
{
    foc_sig_t *state = &foc->sig;
    const motor_cfg_t *motor = &foc->motor;

    state->pr_dif = state->e_pr - state->pr_lst;
    if (state->pr_dif > (0.88f * M_2PI))
    {
        state->rev--;
    }
    if (state->pr_dif < (-0.88f * M_2PI))
    {
        state->rev++;
    }

    state->e_pe = state->e_pr * motor->pn
        - (uint32_t)(state->e_pr * motor->pnd_2pi) * M_2PI
        + motor->e_off;
    wrap_0_2pi(state->e_pe);

    state->p_e = state->e_pr * motor->pn
        - (uint32_t)(state->e_pr * motor->pnd_2pi) * M_2PI
        + motor->e_off;
    wrap_0_2pi(state->p_e);

    state->sp_r = state->e_pr + motor->r_off;
    wrap_0_2pi(state->sp_r);
    state->mp_r = state->e_pr + (float)state->rev * M_2PI + motor->r_off;

    state->sp_m = state->sp_r * motor->div_Gr + motor->m_off;
    wrap_0_2pi(state->sp_m);
    state->mp_m = state->mp_r * motor->div_Gr;
    state->m_rev = (int32_t)(state->mp_m * div_M_2PI);
    state->pr_lst = state->e_pr;
}
