/**
 * @file target_qenc.h
 * @brief TIM3 正交编码器接口（ABZ 增量编码器，2500 线 ×4 = 10000 计数/圈）。
 * @note  配置整体迁自沉沙ABZ固件已验证代码（B库 encoder.c:31-98），三个坑随迁：
 *        坑① init 必须晚于 MX_TIM3_Init/MX_ADC2_Init 调用（P3 设备层负责时序），
 *             否则 PA6/PA7 被 ADC2 的 MspInit 重新配置为模拟脚；
 *        坑② TIM3 CH3 必须先 HAL_TIM_IC_ConfigChannel 再 IC_Start_IT，
 *             否则 PB0 被当 OC3 输出钉在低电平；
 *        坑③ 计数值位宽按 14 位（10000 计数）处理，标定 LUT 索引才不越界。
 */

#ifndef TARGET_QENC_H
#define TARGET_QENC_H

#include <stdint.h>

/** @brief 配置 TIM3 为编码器模式并启动计数；Z 脉冲输入捕获仅诊断用。 */
void target_qenc_init(void);

/** @brief 读原始计数值（0..9999 单圈，多圈换算属于 device 层）。 */
int32_t target_qenc_read(void);

/** @brief Z 脉冲累计计数（每圈 +1；用于丢脉冲探测，不参与闭环）。 */
uint32_t target_qenc_z_count(void);

#endif /* TARGET_QENC_H */
