/**
 * @file drive.c
 * @brief 电机启停、故障和运行请求的最小状态闭环。
 */

#include "drive.h"
#include "drive_mode.h"
#include "drive_pwm.h"

/**
 * @brief 启动三相 PWM，并把控制器置于已知初始状态。
 *
 * @param[in,out] pm 电机控制对象。
 *
 * 这里保留原硬件顺序：先启动三相输出，再写 50% 占空比，最后清控制器。
 * pwm_active 记录软件已经发出启动动作，避免每个快速周期重复启动硬件。
 */
static _RAM_FUNC void drive_start_pwm(pmsm_t *pm)
{
    if (pm->pwm_active)
    {
        return;
    }

    drive_pwm_start();
    drive_pwm_set_neutral(pm);
    pmsm_reset(pm);
    pm->pwm_active = true;
}

/**
 * @brief 关闭三相 PWM，并清空控制器历史状态。
 *
 * @param[in,out] pm 电机控制对象。
 *
 * 仅在软件记录为已启动时执行一次停止动作，空闲周期不重复调用 HAL 停止接口。
 */
static _RAM_FUNC void drive_stop_pwm(pmsm_t *pm)
{
    if (!pm->pwm_active)
    {
        return;
    }

    drive_pwm_stop();
    pmsm_reset(pm);
    pm->pwm_active = false;
}

/**
 * @brief 执行本周期入口请求对应的动作。
 *
 * @param[in,out] pm 电机控制对象。
 * @param[in] req 本周期入口锁存的请求。
 */
static _RAM_FUNC void drive_exec_action(pmsm_t *pm, drive_req_e req)
{
    switch (req)
    {
        case DRIVE_REQ_STOP:
            drive_stop_pwm(pm);
            break;

        case DRIVE_REQ_START:
            drive_start_pwm(pm);
            break;

        case DRIVE_REQ_RUN:
            if (pm->pwm_active)
            {
                drive_mode_step(pm);
            }
            break;

        default:
            /* 非法请求不允许继续保持功率输出。 */
            drive_stop_pwm(pm);
            break;
    }
}

/**
 * @brief 根据入口请求和动作结果更新 Drive 状态。
 *
 * @param[in,out] pm 电机控制对象。
 * @param[in] req 本周期入口锁存的请求。
 */
static _RAM_FUNC void drive_update_state(pmsm_t *pm, drive_req_e req)
{
    switch (req)
    {
        case DRIVE_REQ_STOP:
            pm->state = DRIVE_STATE_STOP;
            break;

        case DRIVE_REQ_START:
            if (pm->pwm_active)
            {
                pm->state = DRIVE_STATE_STARTING;
            }
            else
            {
                pm->state = DRIVE_STATE_STOP;
            }
            break;

        case DRIVE_REQ_RUN:
            if (pm->pwm_active)
            {
                pm->state = DRIVE_STATE_RUN;
            }
            else
            {
                pm->state = DRIVE_STATE_STOP;
                pm->req = DRIVE_REQ_STOP;
            }
            break;

        default:
            pm->state = DRIVE_STATE_STOP;
            pm->req = DRIVE_REQ_STOP;
            break;
    }
}

_RAM_FUNC void drive_fast_step(pmsm_t *pm)
{
    const drive_req_e req = pm->req;

    /* 每周期从空命令开始；只有实际执行的启动或控制路径可以重新生成命令。 */
    pm->pwm_cmd.seq = pm->fast_seq;
    pm->pwm_cmd.valid = false;

    /* 保持原快速链先执行本周期动作、再汇总故障的先后关系。 */
    drive_exec_action(pm, req);

    if (pm->fault.all > 0U)
    {
        /* 故障在本周期末关闭输出；已关闭时不会重复调用停止接口。 */
        drive_stop_pwm(pm);
        pm->state = DRIVE_STATE_FAULT;
        pm->req = DRIVE_REQ_STOP;
        return;
    }

    drive_update_state(pm, req);

    /* START 是一次性请求；启动成功后，下一快速周期进入正常 RUN。 */
    if ((req == DRIVE_REQ_START) &&
        (pm->req == DRIVE_REQ_START) &&
        (pm->state == DRIVE_STATE_STARTING))
    {
        pm->req = DRIVE_REQ_RUN;
    }
}
