/**
 * @file target_encoder.h
 * @brief 编码器原始数据读取的板级适配接口。
 *
 * 当前节点只接管项目实际使用的 MT6816：底层负责 SPI1、片选时序、寄存器命令
 * 和响应拼接，上层继续负责原始计数到机械角、电角度及多圈位置的换算。
 */

#ifndef TARGET_ENCODER_H
#define TARGET_ENCODER_H

#include <stdint.h>

/**
 * @brief 读取 MT6816 的 14 位单圈位置原始值。
 *
 * @return 编码器原始计数，范围为 0～16383，单位为 count，不是弧度。
 *
 * 函数依次读取 MT6816 的 0x03 和 0x04 寄存器，并按照当前固件原有格式
 * 拼成 14 位位置值。SPI 外设和片选引脚只在板级适配层中出现。
 */
uint16_t target_encoder_read_mt6816_raw(void);

#endif /* TARGET_ENCODER_H */
