/**
 * @file math_const.h
 * @brief 数学常量，全部为 float 字面量（带 f 后缀，见 14 号数值政策）。
 * @note  禁止 #define M_PI 等标准名（03 号遗留问题清单）；
 *        MATH_ 前缀与 math.h / CMSIS-DSP 的 PI 宏零冲突（命名决策记录见 12 号）。
 */

#ifndef MATH_CONST_H
#define MATH_CONST_H

#define MATH_PI           3.14159265358979f
#define MATH_2PI          6.28318530717959f
#define MATH_PI_2         1.57079632679490f
#define MATH_SQRT3        1.73205080757f
#define MATH_SQRT3_2      0.86602540378f
#define MATH_1_OVER_SQRT3 0.57735026919f

#endif /* MATH_CONST_H */
