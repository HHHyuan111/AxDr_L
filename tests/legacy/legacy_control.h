/**
 * @file legacy_control.h
 * @brief 仅供电脑端新旧对照测试使用的旧算法声明。
 *
 * 这里保留迁移前算法的测试入口。生产代码不得包含本头文件。
 */

#ifndef TESTS_LEGACY_CONTROL_H
#define TESTS_LEGACY_CONTROL_H

#include <stdbool.h>
#include <math.h>

#include "common.h"
#include "control_cascade.h"
#include "foc_control.h"
#include "foc_core.h"

/* 迁移前实现使用的局部常量，仅保留在数值回归测试中。 */
#define LEGACY_MIN(x, y) (((x) < (y)) ? (x) : (y))
#define LEGACY_MAX(x, y) (((x) > (y)) ? (x) : (y))
#define LEGACY_ABS(x)    (((x) >= 0.0f) ? (x) : -(x))
#define SQRT3            (1.73205080757f)
#define TWO_BY_SQRT3     (1.15470053838f)

typedef struct
{
    float theta;
    float sin_val;
    float cos_val;
    float ia;
    float ib;
    float ic;
    float ialpha;
    float ibeta;
    float id;
    float iq;
    float va;
    float vb;
    float vc;
    float valpha;
    float vbeta;
    float vd;
    float vq;
    float inv_vbus;
    float duty_a;
    float duty_b;
    float duty_c;
} legacy_foc_state_t;

void foc_calc(legacy_foc_state_t *foc);
void sin_cos_val(legacy_foc_state_t *foc);
void clarke_transform(legacy_foc_state_t *foc);
void inverse_clarke(legacy_foc_state_t *foc);
void park_transform(legacy_foc_state_t *foc);
void inverse_park(legacy_foc_state_t *foc);
void svpwm_midpoint(legacy_foc_state_t *foc);
void svpwm_sector(legacy_foc_state_t *foc);
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

void legacy_foc_core_prepare(const foc_sample_t *sample, foc_frame_t *frame);
bool legacy_foc_core_modulate(const foc_frame_t *frame,
                              const foc_voltage_t *voltage,
                              foc_duty_t *duty);

bool legacy_control_cur_step(control_rate_t *rate,
                             pid_para_t *d_pid,
                             pid_para_t *q_pid,
                             float id_ref,
                             float iq_ref,
                             float id_feedback,
                             float iq_feedback,
                             float *v_d,
                             float *v_q);
bool legacy_control_spd_step(control_rate_t *rate,
                             pid_para_t *speed_pid,
                             float speed_ref,
                             float speed_feedback,
                             float iq_limit_abs,
                             float *iq_ref);
bool legacy_control_pos_step(control_rate_t *rate,
                             pid_para_t *position_pid,
                             float position_ref,
                             float position_feedback,
                             float speed_limit_abs,
                             float *speed_ref);
bool legacy_foc_ctrl_step(foc_ctrl_t *ctrl,
                          const foc_fb_t *fb,
                          const foc_ref_t *ref,
                          foc_out_t *out);

#endif /* TESTS_LEGACY_CONTROL_H */
