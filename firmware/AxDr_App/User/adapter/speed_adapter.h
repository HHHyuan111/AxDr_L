/** @file speed_adapter.h
 *  @brief 转子位置差分测速，输出单位 rad/s。
 */
#ifndef SPEED_ADAPTER_H
#define SPEED_ADAPTER_H
#include <stdbool.h>
#include <stdint.h>
#include "speed_est.h"
void speed_est_init(speed_est_t *est);
/* N 拍差分需要 N+1 个连续有效样本。丢样/序号不连续后重新积累窗口，
 * 期间保持上次速度，位置有效性仍交给保护判断，不拿压缩后的时间计算速度。 */
void speed_est_step(speed_est_t *est, float pos_rad, float fs_hz,
                    uint32_t seq, bool valid);
float speed_est_get(const speed_est_t *est);
#endif
