/**
 * @file foc_core.h
 * @brief 与硬件和电机大对象无关的 FOC 核心数据链。
 */

#ifndef AXDR_FOC_CORE_H
#define AXDR_FOC_CORE_H

#include <stdbool.h>

/**
 * @brief 一次 FOC 计算使用的三相电流和电角度输入。
 */
typedef struct
{
    float ia;    /* A 相电流。 */
    float ib;    /* B 相电流。 */
    float ic;    /* C 相电流。 */
    float theta; /* 电角度，单位 rad。 */
} foc_sample_t;

/**
 * @brief 三相电流投影到旋转坐标系后的本周期结果。
 */
typedef struct
{
    float theta;
    float sin;
    float cos;
    float ialpha;
    float ibeta;
    float id;
    float iq;
} foc_frame_t;

/**
 * @brief 本周期需要调制的 d-q 电压命令。
 */
typedef struct
{
    float vd;       /* d 轴电压。 */
    float vq;       /* q 轴电压。 */
    float inv_vbus; /* 现有 SVPWM 使用的母线电压归一化系数。 */
} foc_voltage_t;

/**
 * @brief FOC 生成的静止坐标电压和三相占空比候选值。
 */
typedef struct
{
    float valpha;
    float vbeta;
    float duty_a;
    float duty_b;
    float duty_c;
} foc_duty_t;

/**
 * @brief 把三相电流和电角度整理为本周期 d-q 电流反馈。
 *
 * @param[in] sample 本周期三相电流和电角度。
 * @param[out] frame 接收角度、正余弦和坐标变换结果。
 * @pre 两个指针有效且对象互不重叠。
 */
void foc_core_prepare(const foc_sample_t *sample, foc_frame_t *frame);

/**
 * @brief 把 d-q 电压命令变换为三相占空比候选值。
 *
 * @param[in] frame 与本周期电流反馈相同角度的坐标变换结果。
 * @param[in] voltage 本周期 d-q 电压和母线归一化系数。
 * @param[out] duty 接收 alpha-beta 电压和三相占空比候选值。
 * @return 三相占空比均有效时返回 true，否则返回 false。
 * @pre 三个指针有效且对象互不重叠。
 */
bool foc_core_modulate(const foc_frame_t *frame,
                       const foc_voltage_t *voltage,
                       foc_duty_t *duty);

#endif /* AXDR_FOC_CORE_H */
