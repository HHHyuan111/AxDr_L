/** @file foc_run.h
 *  @brief 快速周期内部使用的给定和分步计算接口。
 *  对外 foc.h 仍只暴露电机对象指针和初始化/运行入口。
 */
#ifndef FOC_RUN_H
#define FOC_RUN_H

#include "foc.h"
#include "foc_control.h"

/* theta 是控制角度：闭环取反馈，VF/IF/对齐取指定场角。 */
struct foc_plan
{
    foc_ref_t ref;
    float theta;
};

/* Clarke/Park 变换，只更新电流与角度，不执行 PI 或输出 PWM。 */
void foc_update_current(foc_t *foc, float theta, foc_frame_t *frame);

/* 使用本拍坐标执行控制和调制，只生成占空比。 */
bool foc_execute(foc_t *foc, const foc_plan_t *cmd, const foc_frame_t *frame);

#endif
