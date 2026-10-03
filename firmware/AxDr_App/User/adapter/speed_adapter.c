/**
 * @file speed_adapter.c
 * @brief 位置差分测速（影子链）实现。
 *
 * 与 B 库 axdr_speed_estimator 同构：N 点历史环形缓冲 + 窗口差分 + 解卷绕。
 * 差分对单圈角做 ±π 回绕处理，因此跨零点转动不会产生虚假速度尖峰。
 */

#include "speed_adapter.h"

#include <stdbool.h>

#include "encoder_config.h"
#include "math_const.h"

#define EST_HISTORY_DEPTH 8U /* B 库同深度 */

static float est_hist[EST_HISTORY_DEPTH];
static float est_speed;
static uint32_t est_index;
static bool est_ready;

void speed_est_init(void)
{
    uint32_t i;

    for (i = 0U; i < EST_HISTORY_DEPTH; i++)
    {
        est_hist[i] = 0.0f;
    }
    est_index = 0U;
    est_speed = 0.0f;
    est_ready = false;
}

void speed_est_step(float pos_rad, float fs_hz)
{
    static const uint32_t window = ABZ_SPEED_WINDOW_TICKS;
    float diff;

    est_hist[est_index % EST_HISTORY_DEPTH] = pos_rad;
    est_index++;

    if (!est_ready && (est_index >= window))
    {
        est_ready = true;
    }
    if (!est_ready)
    {
        return;
    }

    /* 窗口差分：当前值 减 window 拍前的历史值。
     * est_index 已在写入后自增，当前值位于 (est_index-1)，窗口起点再往前 window 拍。 */
    diff = pos_rad
        - est_hist[(est_index - 1U + EST_HISTORY_DEPTH - window) % EST_HISTORY_DEPTH];

    /* 单圈角解卷绕：差值折回 (-π, π] */
    if (diff > MATH_PI)
    {
        diff -= MATH_2PI;
    }
    else if (diff < -MATH_PI)
    {
        diff += MATH_2PI;
    }
    else
    {
        /* 无回绕 */
    }

    est_speed = diff * fs_hz / (float)window;
}

float speed_est_get(void)
{
    return est_speed;
}
