/**
 * @file encoder_feedback.c
 * @brief 正式单 MA732/MT6816 位置反馈所需的编码器适配。
 */

#include "common.h"
#include "motor_drive_config.h"
#include "target_encoder.h"

/**
 * @brief 初始化正式使用的 MA732 和 MT6816 参数。
 *
 * @param[in,out] pos_box 位置反馈对象；写入位宽、每圈计数和弧度换算系数。
 */
void encoder_init(pos_box_t *pos_box)
{
    pos_box->ma732.dir = MA732_DIRECTION;
    pos_box->ma732.bit = MA732_RESOLUTION_BITS;
    pos_box->ma732.cpr = MA732_COUNTS_PER_REV;
    pos_box->ma732.shift_bit = (uint8_t)log2f(
        (float)pos_box->ma732.cpr / 256.0f);
    pos_box->ma732.factor = M_2PI / (float)pos_box->ma732.cpr;

    pos_box->mt6816.dir = MT6816_DIRECTION;
    pos_box->mt6816.bit = MT6816_RESOLUTION_BITS;
    pos_box->mt6816.cpr = MT6816_COUNTS_PER_REV;
    pos_box->mt6816.shift_bit = (uint8_t)log2f(
        (float)pos_box->mt6816.cpr / 256.0f);
    pos_box->mt6816.factor = M_2PI / (float)pos_box->mt6816.cpr;
}

/**
 * @brief 从 Target 层读取一帧新的 MA732 原始位置。
 *
 * @param[in,out] enc 编码器对象；成功时更新原始计数和新数据标志。
 * @return SPI 读取成功返回 true，否则保持旧原始计数并返回 false。
 */
_RAM_FUNC bool read_ma732_raw(enc_para_t *enc)
{
    uint16_t raw_count;

    if (!target_encoder_read_ma732_raw(&raw_count))
    {
        enc->rev_flag = 0U;
        return false;
    }

    enc->raw = (int32_t)raw_count;
    enc->rev_flag = 1U;
    return true;
}

/**
 * @brief 从 Target 层读取一帧新的 MT6816 原始位置。
 *
 * @param[in,out] enc 编码器对象；成功时更新原始计数和新数据标志。
 * @return 两帧 SPI 读取都成功返回 true，否则保持旧原始计数并返回 false。
 */
_RAM_FUNC bool read_mt6816_raw(enc_para_t *enc)
{
    uint16_t raw_count;

    if (!target_encoder_read_mt6816_raw(&raw_count))
    {
        enc->rev_flag = 0U;
        return false;
    }

    enc->raw = (int32_t)raw_count;
    enc->rev_flag = 1U;
    return true;
}

/**
 * @brief 把 MT6816 原始计数换算为 0～2π 的单圈机械角。
 *
 * @param[in,out] enc 编码器对象；输入 raw 和 factor，输出 pos，单位为弧度。
 */
_RAM_FUNC void encoder_update_angle(enc_para_t *enc)
{
    enc->pos = (float)enc->raw * enc->factor;
    wrap_0_2pi(enc->pos);
}
