/**
 * @file current_offset.h
 * @brief 三相电流采样链零偏采集入口。
 */

#ifndef CURRENT_OFFSET_H
#define CURRENT_OFFSET_H

#include "foc_fwd.h"

/* 电机未进入闭环控制前调用：阻塞采集 1000 次，间隔 1 ms，均值写入 foc->adc。 */
void cur_offset_init(foc_t *foc);

#endif /* CURRENT_OFFSET_H */
