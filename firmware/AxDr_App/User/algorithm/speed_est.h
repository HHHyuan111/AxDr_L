/** @brief 位置差分测速的独立历史（纯数据，主机可测）。
 * @note  类型放在 L0 纯数据区是因为 foc_t（motor 层）内嵌本结构——
 *        依赖铁律只允许向更低层 include，实现在 adapter/speed_adapter.c。 */
#ifndef SPEED_EST_H
#define SPEED_EST_H
#include <stdint.h>
#define SPEED_HISTORY_DEPTH 8U
typedef struct
{
    float hist[SPEED_HISTORY_DEPTH];
    float speed;
    uint32_t seq;
    uint32_t index;
    uint32_t count;
} speed_est_t;
#endif
