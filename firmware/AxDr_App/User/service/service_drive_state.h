/** @brief 将内部运行状态转换为上位机既有的四个状态码。 */
#ifndef SERVICE_DRIVE_STATE_H
#define SERVICE_DRIVE_STATE_H
#include "common.h"
static inline uint8_t service_drive_state(drive_state_e state)
{
    switch (state)
    {
        case DRIVE_STATE_ALIGN: return (uint8_t)DRIVE_STATE_STARTING;
        case DRIVE_STATE_DIAG:  return (uint8_t)DRIVE_STATE_RUN;
        default:               return (uint8_t)state;
    }
}
#endif
