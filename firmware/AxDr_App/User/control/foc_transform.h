/**
 * @file foc_transform.h
 * @brief 与板卡和电机对象无关的 FOC 角度与坐标变换接口。
 */

#ifndef AXDR_FOC_TRANSFORM_H
#define AXDR_FOC_TRANSFORM_H

/**
 * @brief 计算电角度对应的正弦值和余弦值。
 *
 * @param[in] theta_e_rad 电角度，单位 rad。
 * @param[out] sin_theta 电角度的正弦值。
 * @param[out] cos_theta 电角度的余弦值。
 * @pre 两个输出指针必须有效且互不重叠。
 */
void foc_sin_cos(float theta_e_rad,
                 float *sin_theta,
                 float *cos_theta);

/**
 * @brief 把三相静止坐标电流变换为 alpha-beta 电流。
 *
 * @param[in] i_a A 相电流。
 * @param[in] i_b B 相电流。
 * @param[in] i_c C 相电流。
 * @param[out] i_alpha alpha 轴电流。
 * @param[out] i_beta beta 轴电流。
 *
 * 三相输入和两轴输出使用同一个电流单位。两个输出指针必须有效且互不重叠。
 */
void foc_clarke(float i_a,
                float i_b,
                float i_c,
                float *i_alpha,
                float *i_beta);

/**
 * @brief 把 alpha-beta 电流变换为旋转 d-q 电流。
 *
 * @param[in] i_alpha alpha 轴电流。
 * @param[in] i_beta beta 轴电流。
 * @param[in] sin_theta 当前电角度的正弦值。
 * @param[in] cos_theta 当前电角度的余弦值。
 * @param[out] i_d d 轴电流。
 * @param[out] i_q q 轴电流。
 *
 * 输入和输出使用同一个电流单位。两个输出指针必须有效且互不重叠。
 */
void foc_park(float i_alpha,
              float i_beta,
              float sin_theta,
              float cos_theta,
              float *i_d,
              float *i_q);

/**
 * @brief 把旋转 d-q 电压变换为 alpha-beta 电压。
 *
 * @param[in] v_d d 轴电压。
 * @param[in] v_q q 轴电压。
 * @param[in] sin_theta 当前电角度的正弦值。
 * @param[in] cos_theta 当前电角度的余弦值。
 * @param[out] v_alpha alpha 轴电压。
 * @param[out] v_beta beta 轴电压。
 *
 * 输入和输出使用同一个电压单位。两个输出指针必须有效且互不重叠。
 */
void foc_inv_park(float v_d,
                  float v_q,
                  float sin_theta,
                  float cos_theta,
                  float *v_alpha,
                  float *v_beta);

#endif /* AXDR_FOC_TRANSFORM_H */
