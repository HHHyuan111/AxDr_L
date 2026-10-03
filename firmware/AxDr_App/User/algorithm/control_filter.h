/**
 * @file control_filter.h
 * @brief 与硬件无关的一阶低通滤波器接口。
 */

#ifndef AXDR_CONTROL_FILTER_H
#define AXDR_CONTROL_FILTER_H

/**
 * @brief 一阶低通滤波器的配置和运行状态。
 */
typedef struct
{
    float val;      /* 本周期原始输入。 */
    float val_f;    /* 本周期滤波输出。 */
    float fs;       /* 采样频率，单位 Hz。 */
    float fc;       /* 截止频率，单位 Hz。 */
    float filt_a;   /* 上一拍输出系数。 */
    float filt_b;   /* 本周期输入系数。 */
} lpf_t;

/**
 * @brief 根据采样频率和截止频率计算滤波系数，并清空运行状态。
 *
 * @param[in,out] filter 已设置 fs 和 fc 的滤波器上下文。
 * @pre filter 指向有效对象，fs 和 fc 沿用现有工程的有效参数约定。
 */
void control_lpf_init(lpf_t *filter);

/**
 * @brief 输入一个新样本并执行一拍一阶低通滤波。
 *
 * @param[in,out] filter 已完成初始化的滤波器上下文。
 * @param[in] value 本周期输入，单位由具体信号决定。
 * @return 本周期滤波输出，与 value 使用相同单位。
 * @pre filter 由当前调用者独占写入。
 */
float control_lpf_step(lpf_t *filter, float value);

#endif /* AXDR_CONTROL_FILTER_H */
