/**
 * @file foc_control.h
 * @brief 与硬件无关的 FOC 电压、电流、速度和位置控制入口。
 */

#ifndef FOC_CONTROL_H
#define FOC_CONTROL_H

#include <stdbool.h>

#include "control_cascade.h"
#include "foc_core.h"

/** @brief FOC 控制链模式。 */
typedef enum
{
    FOC_CTRL_MODE_VOLT = 0,
    FOC_CTRL_MODE_CUR = 1,
    FOC_CTRL_MODE_SPD = 2,
    FOC_CTRL_MODE_POS = 3,
} foc_ctrl_mode_t;

/**
 * @brief FOC 控制器运行上下文。
 *
 * 本对象只保存控制算法跨周期需要的状态。四个 PI 对象由上层创建，
 * 由当前 FOC 控制器独占更新；这里不保存 ADC、定时器或电机板对象。
 */
typedef struct
{
    control_rate_t cur_rate;
    control_rate_t spd_rate;
    control_rate_t pos_rate;
    pid_para_t *id_pi;
    pid_para_t *iq_pi;
    pid_para_t *spd_pi;
    pid_para_t *pos_pi;
    float vd;
    float vq;
    float iq_ref;
    float spd_ref;
} foc_ctrl_t;

/**
 * @brief FOC 本周期使用的反馈。
 *
 * spd 单位为 rad/s，pos 单位为 rad，inv_vbus 是母线电压调制系数。
 */
typedef struct
{
    foc_sample_t sample;
    float inv_vbus;
    float spd;
    float pos;
} foc_fb_t;

/**
 * @brief FOC 本周期的控制给定。
 *
 * 只有 mode 对应的字段参与计算。电压单位 V，电流单位 A，速度单位 rad/s，
 * 位置单位 rad；cur_lim 和 spd_lim 均为绝对限制值。
 */
typedef struct
{
    foc_ctrl_mode_t mode;
    float vd;
    float vq;
    float id_ref;
    float iq_ref;
    float spd_ref;
    float pos_ref;
    float cur_lim;
    float spd_lim;
} foc_ref_t;

/** @brief FOC 本周期的坐标变换、级联参考和 PWM 占空比结果。 */
typedef struct
{
    foc_frame_t frame;
    foc_duty_t pwm;
    float vd;
    float vq;
    float iq_ref;
    float spd_ref;
    bool valid;
} foc_out_t;

/**
 * @brief 执行一次指定模式的完整 FOC 控制链。
 *
 * @param[in,out] ctrl 控制器运行上下文和 PI 引用。
 * @param[in] fb 本周期电流、角度、速度、位置和母线反馈。
 * @param[in] ref 本周期控制模式、给定值和限制值。
 * @param[out] out 坐标变换、级联控制和三相 PWM 计算结果。
 * @return 三相占空比均有效时返回 true；模式非法或占空比无效时返回 false。
 * @pre 四个参数均为有效且互不重叠的对象；ctrl 中使用到的 PI 指针有效。
 *
 * 本函数不访问 HAL、Target 或全局电机对象，不分配内存，也不阻塞。
 */
bool foc_ctrl_step(foc_ctrl_t *ctrl,
                   const foc_fb_t *fb,
                   const foc_ref_t *ref,
                   foc_out_t *out);

#endif /* FOC_CONTROL_H */
