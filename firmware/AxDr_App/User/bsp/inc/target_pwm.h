/**
 * @file target_pwm.h
 * @brief 三相 PWM 板级适配接口。
 *
 * 输入：已经按 TIM1 通道 1、2、3 排列好的归一化占空比。
 * 输出：TIM1 三相主输出和互补输出。
 * 调用时机：电机状态切换时启停输出，快速控制环中更新占空比。
 * 模块边界：不计算 SVPWM，不决定电机相序，不处理 Gate 和故障许可。
 */

#ifndef TARGET_PWM_H
#define TARGET_PWM_H

/**
 * @brief 启动 TIM1 三相主输出和互补输出。
 *
 * TIM1 的一个电机相通道包含主输出 CHx 和互补输出 CHxN。本函数依次启动
 * CH1/CH1N、CH2/CH2N、CH3/CH3N，使定时器能够把当前 CCR 比较值输出到引脚。
 * 本函数只负责打开定时器输出，不计算或修改占空比。
 */
void target_pwm_start_phase_outputs(void);

/**
 * @brief 停止 TIM1 三相主输出和互补输出。
 *
 * 本函数依次停止 CH1/CH1N、CH2/CH2N、CH3/CH3N。停止输出后，原有 CCR
 * 比较值仍然保留；以后重新启动时，是否先更新占空比由上层状态机决定。
 */
void target_pwm_stop_phase_outputs(void);

/**
 * @brief 设置 TIM1 三个物理通道的占空比。
 *
 * @param[in] channel_1_duty_ratio 通道 1 归一化占空比，正常范围为 0.0f～1.0f。
 * @param[in] channel_2_duty_ratio 通道 2 归一化占空比，正常范围为 0.0f～1.0f。
 * @param[in] channel_3_duty_ratio 通道 3 归一化占空比，正常范围为 0.0f～1.0f。
 *
 * 三个参数表示 TIM1 物理通道，不直接表示电机 A、B、C 相；调用者先完成相序映射。
 * 函数先读取 TIM1 的周期计数值，再执行“占空比比例 × 周期计数值”，最后把三个结果
 * 分别写入 CCR1、CCR2、CCR3。TIM1 根据这些比较值自动生成 PWM，CPU 不直接翻转引脚。
 * 为保持原有实时行为，本函数不在板级适配层增加限幅或状态判断。
 */
void target_pwm_set_duty_ratios(float channel_1_duty_ratio,
                                float channel_2_duty_ratio,
                                float channel_3_duty_ratio);

#endif /* TARGET_PWM_H */
