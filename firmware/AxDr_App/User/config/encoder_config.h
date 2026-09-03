/**
 * @file encoder_config.h
 * @brief 当前工程支持的编码器方向、分辨率和默认选择。
 */

#ifndef ENCODER_CONFIG_H
#define ENCODER_CONFIG_H

#include "encoder_type.h"

#define ENCODER_SELECTED_TYPE                 (ENCODER_TYPE_MA732)

#define MA732_DIRECTION                       (1)
#define MA732_RESOLUTION_BITS                 (14U)
#define MA732_COUNTS_PER_REV                  (16384U)

#define MT6816_DIRECTION                      (1)
#define MT6816_RESOLUTION_BITS                (14U)
#define MT6816_COUNTS_PER_REV                 (16384U)

#endif /* ENCODER_CONFIG_H */
