/** @brief 位置差分测速的独立历史（纯数据，主机可测）。 */
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
