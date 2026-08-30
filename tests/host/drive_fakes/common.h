/**
 * @file common.h
 * @brief Drive 主机测试所需的最小生产类型替身。
 */

#ifndef TEST_DRIVE_COMMON_H
#define TEST_DRIVE_COMMON_H

#include <stdbool.h>
#include <stdint.h>

#define _RAM_FUNC

typedef enum
{
    DRIVE_REQ_STOP = 0,
    DRIVE_REQ_START,
    DRIVE_REQ_RUN
} drive_req_e;

typedef enum
{
    DRIVE_STATE_STOP = 0,
    DRIVE_STATE_STARTING,
    DRIVE_STATE_RUN,
    DRIVE_STATE_FAULT
} drive_state_e;

typedef struct
{
    uint32_t all;
} pmsm_fault_t;

typedef struct
{
    drive_req_e req;
    drive_state_e state;
    bool pwm_active;
    pmsm_fault_t fault;
} pmsm_t;

void foc_pwm_start(void);
void foc_pwm_stop(void);
void foc_pwm_duty_set(pmsm_t *pm);
void pmsm_reset(pmsm_t *pm);
void pmsm_run_selected_mode(pmsm_t *pm);

#endif /* TEST_DRIVE_COMMON_H */
