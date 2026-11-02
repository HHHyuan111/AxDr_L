/**
 * @file foc_smo.h
 * @brief 反电动势滑模观测器：估计电角度与电角速度（纯算法，主机可测）。
 *
 * 参考 ChenshaMotor_FOC_SMOSimple 的电流误差滑模结构（reference/ 目录），
 * 三处改进：饱和函数替代硬符号（抗抖振）、PLL 提取角度与速度（参考实现
 * 速度仍依赖编码器）、全部状态收进结构体（无文件静态量）。
 */

#ifndef FOC_SMO_H
#define FOC_SMO_H

#include <stdbool.h>

/**
 * @brief 滑模观测器上下文：参数在 init 时固定，状态由 step 独占更新。
 * 单位：电流 A、电压 V、电阻 Ω、电感 H、角频率 rad/s、时间 s。
 */
typedef struct
{
    float rs;      /* 相电阻 Ω */
    float ls;      /* 相电感 H */
    float kslide;  /* 滑模增益（反电动势幅值量级，V） */
    float lpf_wc;  /* 反电动势低通截止角频率 rad/s */
    float pll_kp;  /* PLL 比例增益 */
    float pll_ki;  /* PLL 积分增益 */
    float spd_max; /* 速度与积分限幅（电角速度幅值，rad/s） */
    float ts;      /* 执行周期 s */

    float ia_est;  /* α 轴估计电流 A */
    float ib_est;  /* β 轴估计电流 A */
    float ea_f;    /* α 轴反电动势滤波值 V */
    float eb_f;    /* β 轴反电动势滤波值 V */
    float theta;   /* 估计电角度 rad，[0, 2π) */
    float spd;     /* 估计电角速度 rad/s */
    float spd_f;   /* 速度输出低通值（速度环反馈用，抑制 kp 项噪声） */
    float e_mag;   /* 本拍反电动势幅值 V（失锁检测用） */
    float pll_int; /* PLL 积分状态 */
} foc_smo_t;

/* 用电机参数初始化；pll_kp/pll_ki 为 0 时按默认带宽自动计算。
 * spd_max 是估计速度与 PLL 积分的限幅（额定电角速度 × 1.2 左右）。 */
void foc_smo_init(foc_smo_t *s, float rs, float ls, float kslide,
                  float lpf_wc, float ts, float pll_kp, float pll_ki,
                  float spd_max);

/* 清空估计历史（起动/切换前调用）。 */
void foc_smo_reset(foc_smo_t *s);

/* 切换前预置观测器状态（VESC 法）：角度、速度与反电动势按虚拟轴相位
 * 的期望值预置；电流估计直接取本拍实测值——滑模修正以电流误差驱动，
 * 误差为零时预置的反电动势才能存活下来，PLL 从正确轨迹出发。 */
void foc_smo_seed(foc_smo_t *s, float theta, float we, float flux,
                  float ia_meas, float ib_meas);

/* 每拍入口：输入本拍 αβ 电压指令与本拍 αβ 电流反馈。 */
void foc_smo_step(foc_smo_t *s, float ua, float ub, float ia, float ib);

#endif /* FOC_SMO_H */
