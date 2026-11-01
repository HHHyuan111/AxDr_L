/**
 * @file encoder_adapter.h
 * @brief 编码器初始化、原始读取与角度换算接口。
 */

#ifndef ENCODER_ADAPTER_H
#define ENCODER_ADAPTER_H

#include <stdbool.h>

/* 完整定义见 motor/common.h；本头只用 struct 标签前向声明，
 * 调用方不必包含 motor 层，encoder_data_t/encoder_state_t 与标签类型兼容。 */
struct encoder_data;
struct encoder_state;

void encoder_init(struct encoder_state *enc);
bool encoder_sample(struct encoder_state *enc);
bool read_mt6816_raw(struct encoder_data *enc);
bool read_abz_raw(struct encoder_data *enc);
bool read_ma732_raw(struct encoder_data *enc);
void encoder_update_angle(struct encoder_data *enc);

#endif /* ENCODER_ADAPTER_H */
