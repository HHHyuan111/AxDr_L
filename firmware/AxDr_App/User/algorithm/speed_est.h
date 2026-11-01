/** @brief 位置差分测速的独立历史；只含数据，不依赖硬件和 FOC 总对象。 */
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
