/**
 * @file legacy_control.h
 * @brief 仅供电脑端新旧对照测试使用的旧算法声明。
 *
 * 这里保留迁移前算法的测试入口。生产代码不得包含本头文件。
 */

#ifndef TESTS_LEGACY_CONTROL_H
#define TESTS_LEGACY_CONTROL_H

#include "common.h"

void foc_calc(pmsm_foc_t *foc);
void sin_cos_val(pmsm_foc_t *foc);
void clarke_transform(pmsm_foc_t *foc);
void inverse_clarke(pmsm_foc_t *foc);
void park_transform(pmsm_foc_t *foc);
void inverse_park(pmsm_foc_t *foc);
void svpwm_midpoint(pmsm_foc_t *foc);
void svpwm_sector(pmsm_foc_t *foc);
int svm(float alpha, float beta, float *ta, float *tb, float *tc);

void pid_para_init(pid_para_t *pid_config);
void pid_limit_init(pid_para_t *pid_config,
                    float i_term_max,
                    float i_term_min,
                    float out_max,
                    float out_min);
void pid_clear(pid_para_t *pid);
void pid_reset(pid_para_t *pid, float kp, float ki, float kd);
float parallel_pid_ctrl(pid_para_t *pid, float ref_value, float feedback_value);
float serial_pid_ctrl(pid_para_t *pid, float ref_value, float feedback_value);
float pdff_ctrl(pid_para_t *pid, float ref_value, float feedback_value);

float sat1_datf(float val, float up, float low);
void low_pf_init(lpf_t *filter);
float low_pf(lpf_t *filter, float value);
float angle_speed_calc(float angle_rad, float sample_frequency_hz);

#endif /* TESTS_LEGACY_CONTROL_H */
