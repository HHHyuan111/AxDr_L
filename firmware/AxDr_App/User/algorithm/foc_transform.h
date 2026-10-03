/**
 * @file foc_transform.h
 * @brief 与板卡和电机对象无关的 FOC 角度与坐标变换接口。
 */

#ifndef AXDR_FOC_TRANSFORM_H
#define AXDR_FOC_TRANSFORM_H

/**
 * @brief 计算电角度对应的正弦值和余弦值。
 *
 * @param[in] theta 电角度，单位 rad。
 * @param[out] sin 电角度的正弦值。
 * @param[out] cos 电角度的余弦值。
 * @pre 两个输出指针必须有效且互不重叠。
 */
void foc_sin_cos(float theta,
                 float *sin,
                 float *cos);

/**
 * @brief 把三相静止坐标电流变换为 alpha-beta 电流。
 *
 * @param[in] ia A 相电流。
 * @param[in] ib B 相电流。
 * @param[in] ic C 相电流。
 * @param[out] ialpha alpha 轴电流。
 * @param[out] ibeta beta 轴电流。
 *
 * 三相输入和两轴输出使用同一个电流单位。两个输出指针必须有效且互不重叠。
 */
void foc_clarke(float ia,
                float ib,
                float ic,
                float *ialpha,
                float *ibeta);

/**
 * @brief 把 alpha-beta 电流变换为旋转 d-q 电流。
 *
 * @param[in] ialpha alpha 轴电流。
 * @param[in] ibeta beta 轴电流。
 * @param[in] sin 当前电角度的正弦值。
 * @param[in] cos 当前电角度的余弦值。
 * @param[out] id d 轴电流。
 * @param[out] iq q 轴电流。
 *
 * 输入和输出使用同一个电流单位。两个输出指针必须有效且互不重叠。
 */
void foc_park(float ialpha,
              float ibeta,
              float sin,
              float cos,
              float *id,
              float *iq);

/**
 * @brief 把旋转 d-q 电压变换为 alpha-beta 电压。
 *
 * @param[in] vd d 轴电压。
 * @param[in] vq q 轴电压。
 * @param[in] sin 当前电角度的正弦值。
 * @param[in] cos 当前电角度的余弦值。
 * @param[out] valpha alpha 轴电压。
 * @param[out] vbeta beta 轴电压。
 *
 * 输入和输出使用同一个电压单位。两个输出指针必须有效且互不重叠。
 */
void foc_inv_park(float vd,
                  float vq,
                  float sin,
                  float cos,
                  float *valpha,
                  float *vbeta);

#endif /* AXDR_FOC_TRANSFORM_H */
