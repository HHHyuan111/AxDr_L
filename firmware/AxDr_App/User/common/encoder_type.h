/**
 * @file encoder_type.h
 * @brief 位置反馈层支持的编码器类型。
 */

#ifndef ENCODER_TYPE_H
#define ENCODER_TYPE_H

typedef enum
{
    ENCODER_TYPE_MA732 = 1,
    ENCODER_TYPE_MT6816 = 2,
    ENCODER_TYPE_MT6825 = 3,
    ENCODER_TYPE_HALL = 4,
    ENCODER_TYPE_XHALL = 5,
    ENCODER_TYPE_DMENC = 6,
    ENCODER_TYPE_ABZ = 7, /* TIM3 正交编码器接口（沉沙 2500 线 ABZ，当前唯一验证对象） */
} encoder_type_t;

#endif /* ENCODER_TYPE_H */
