/** @file speed_adapter.c
 *  @brief 连续位置窗口差分。历史属于各自电机，不使用文件静态状态。
 */
#include "speed_adapter.h"
#include <math.h>
#include "encoder_config.h"
#include "math_const.h"
_Static_assert(ABZ_SPEED_WINDOW_TICKS > 0U &&
               ABZ_SPEED_WINDOW_TICKS < SPEED_HISTORY_DEPTH,
               "Speed window must fit the position history");

void speed_est_init(speed_est_t *est)
{
    *est = (speed_est_t){0};
}

void speed_est_step(speed_est_t *est, float pos_rad, float fs_hz,
                    uint32_t seq, bool valid)
{
    const uint32_t window = ABZ_SPEED_WINDOW_TICKS;
    /* 无效样本不进入窗口；序号用无符号差，正常回绕仍相差一拍。 */
    if (!valid || !isfinite(pos_rad) || !isfinite(fs_hz) || fs_hz <= 0.0f)
    {
        est->count = 0U;
        est->index = 0U;
        return;
    }
    if (est->count != 0U && seq - est->seq != 1U)
    {
        est->count = 0U;
        est->index = 0U;
    }
    if (est->count >= window)
    {
        const uint32_t past = (est->index + SPEED_HISTORY_DEPTH - window) % SPEED_HISTORY_DEPTH;
        float diff = pos_rad - est->hist[past];
        /* 沿用单圈角的解卷绕；窗口内转角须小于 pi。 */
        if (diff > MATH_PI)
        {
            diff -= MATH_2PI;
        }
        else if (diff < -MATH_PI)
        {
            diff += MATH_2PI;
        }
        est->speed = diff * fs_hz / (float)window;
    }
    else
    {
        ++est->count;
    }
    est->hist[est->index] = pos_rad;
    est->index = (est->index + 1U) % SPEED_HISTORY_DEPTH;
    est->seq = seq;
}

float speed_est_get(const speed_est_t *est)
{
    return est->speed;
}
