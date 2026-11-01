/**
 * @file position_adapter.c
 * @brief 正式单编码器机械角、电角度和多圈位置更新。
 */

#include "position_adapter.h"

#include "common.h"

static void position_update_single_encoder(foc_t *foc, position_sample_t *sample);

/**
 * @brief 按当前正式配置读取本周期编码器原始值。
 *
 * @param[in,out] enc 位置反馈配置和编码器数据对象。
 * @return MA732、MT6816 或 ABZ 读取成功返回 true，其他配置返回 false。
 */
_RAM_FUNC bool encoder_sample(encoder_state_t *enc)
{
    if (enc->source != POSITION_SOURCE_ENCODER)
    {
        return false;
    }

    switch (enc->primary)
    {
        case ENCODER_TYPE_MA732:
            return read_ma732_raw(&enc->ma732);

        case ENCODER_TYPE_MT6816:
            return read_mt6816_raw(&enc->mt6816);

        case ENCODER_TYPE_ABZ:
            return read_abz_raw(&enc->abz);

        default:
            return false;
    }
}

/**
 * @brief 使用本周期新编码器数据更新电角度及单圈、多圈机械位置。
 *
 * @param[in,out] foc 电机对象；提供编码器样本、换算参数和跨圈历史。
 * @param[out] sample 本拍电角度和机械位置，交给控制周期使用。
 * @return 存在有效的新位置样本返回 true，否则不更新位置并返回 false。
 */
_RAM_FUNC bool position_update(foc_t *foc, position_sample_t *sample)
{
    encoder_data_t *enc;

    if (foc->enc.source != POSITION_SOURCE_ENCODER)
    {
        return false;
    }

    switch (foc->enc.primary)
    {
        case ENCODER_TYPE_MA732:
            enc = &foc->enc.ma732;
            break;

        case ENCODER_TYPE_MT6816:
            enc = &foc->enc.mt6816;
            break;

        case ENCODER_TYPE_ABZ:
            enc = &foc->enc.abz;
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
    foc->fb.enc_pos_r = enc->pos;
    position_update_single_encoder(foc, sample);

    foc->enc.raw = enc->raw;
    foc->enc.pos = enc->pos;
    return true;
}

/**
 * @brief 根据单圈转子角计算电角度、累计转子位置和输出轴位置。
 *
 * 跨圈计数、上一拍角度和单圈量仍在 fb 中保留；新控制位置只写入 sample。
 */
static _RAM_FUNC void position_update_single_encoder(foc_t *foc, position_sample_t *sample)
{
    foc_fb_state_t *state = &foc->fb;
    const motor_cfg_t *motor = &foc->motor;

    state->pos_diff = state->enc_pos_r - state->pos_last;
    if (state->pos_diff > (0.88f * M_2PI))
    {
        state->rev--;
    }
    if (state->pos_diff < (-0.88f * M_2PI))
    {
        state->rev++;
    }

    sample->theta_e = state->enc_pos_r * motor->pn
        - (uint32_t)(state->enc_pos_r * motor->pnd_2pi) * M_2PI
        + motor->e_off;
    wrap_0_2pi(sample->theta_e);

    state->pos_r_1t = state->enc_pos_r + motor->r_off;
    wrap_0_2pi(state->pos_r_1t);
    sample->pos_r = state->enc_pos_r + (float)state->rev * M_2PI + motor->r_off;

    state->pos_m_1t = state->pos_r_1t * motor->div_Gr + motor->m_off;
    wrap_0_2pi(state->pos_m_1t);
    sample->pos_m = sample->pos_r * motor->div_Gr;
    state->m_rev = (int32_t)(sample->pos_m * div_M_2PI);
    state->pos_last = state->enc_pos_r;
}
