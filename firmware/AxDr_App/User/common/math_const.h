/**
 * @file math_const.h
 * @brief 全项目唯一数学常量源，全部为 float 字面量（14 号数值政策）。
 * @note  MATH_ 前缀名已退役，统一使用 M_ 短名；common.h 包含本文件，
 *        包含 common.h 即可使用全部常量。禁止 #define M_PI 等标准名。
 */

#ifndef MATH_CONST_H
#define MATH_CONST_H

#define M_2PI         (6.28318530716f)
#define div_M_2PI     (0.159154943092391467f)
#define SQRT3_BY_2    (0.86602540378f)
#define ONE_BY_SQRT3  (0.57735026919f)

#endif /* MATH_CONST_H */
