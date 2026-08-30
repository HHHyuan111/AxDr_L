/**
 * @file foc_svm.h
 * @brief 与 PWM 硬件无关的空间矢量调制候选计算接口。
 */

#ifndef AXDR_FOC_SVM_H
#define AXDR_FOC_SVM_H

/**
 * @brief 根据归一化 alpha-beta 电压计算三相占空比候选值。
 *
 * @param[in] v_alpha_norm 以母线电压归一化后的 alpha 轴电压。
 * @param[in] v_beta_norm 以母线电压归一化后的 beta 轴电压。
 * @param[out] duty_a A 相占空比候选值。
 * @param[out] duty_b B 相占空比候选值。
 * @param[out] duty_c C 相占空比候选值。
 * @return 三相占空比都位于 [0, 1] 时返回 0，否则返回 -1。
 *
 * 三个输出指针必须有效且互不重叠。函数始终写出候选值，但不做限幅，也不访问
 * PWM 硬件；返回 -1 时，调用者不得提交这些候选值。
 */
int foc_svm(float v_alpha_norm,
            float v_beta_norm,
            float *duty_a,
            float *duty_b,
            float *duty_c);

#endif /* AXDR_FOC_SVM_H */
