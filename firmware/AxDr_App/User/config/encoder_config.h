/**
 * @file encoder_config.h
 * @brief 当前工程支持的编码器方向、分辨率和默认选择。
 * @note  本阶段唯一验证对象 = 200W 伺服电机 + ABZ（B 库正在使用的组合，18 号盘点 §2.1）。
 *        SPI 磁编（MA732/MT6816）代码保留但不进入主链；值全部迁自 B 库已验证配置。
 */

#ifndef ENCODER_CONFIG_H
#define ENCODER_CONFIG_H

#include "encoder_type.h"

/* 当前正式选用的位置反馈：ABZ 增量编码器（TIM3）。 */
#define ENCODER_SELECTED_TYPE                 (ENCODER_TYPE_ABZ)

/*
 * ABZ 参数（B 库 encoder.c:146-155 + target_config L13-15 实测值）。
 * 方向必须与相序配套：B 库接线实测 -1。2026-10-18 曾依据"首次速度闭环
 * 反向跳闸"改 +1——该证据后被证实被 π 错零位的扭矩反向污染；同日扫场
 * 对齐曲线铁证：场角 +1 电圈（+1.257 rad 机械）编码器读 −1.233，
 * 计数方向与场推进相反 → 翻回 −1（+1 时的症状：电流模式双角弹簧锁轴、
 * 速度模式起步即反向计数顶方向守卫静默停机）。
 * 增量式无绝对零点：e_off 上电为 0，必须先做编码器对齐（Drive 层门控，P6）。
 */
#define ABZ_DIRECTION                         (-1)
#define ABZ_RESOLUTION_BITS                   (14U)    /* 10000 计数需 14 位，B 库坑③ */
#define ABZ_COUNTS_PER_REV                    (10000U) /* 2500 线 × TI12 四倍频 */

/* 测速差分窗口（B 库200W 伺服电机配置：窗口 2 拍，B 库配置 L26）。 */
#define ABZ_SPEED_WINDOW_TICKS                (2U)

/* 以下 SPI 磁编参数保留（代码不在主链，仅存档）。 */
#define MA732_DIRECTION                       (1)
#define MA732_RESOLUTION_BITS                 (14U)
#define MA732_COUNTS_PER_REV                  (16384U)

#define MT6816_DIRECTION                      (1)
#define MT6816_RESOLUTION_BITS                (14U)
#define MT6816_COUNTS_PER_REV                 (16384U)

#endif /* ENCODER_CONFIG_H */
