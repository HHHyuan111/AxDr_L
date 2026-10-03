/**
 * @file control_traj.h
 * @brief 与硬件无关的速度和位置轨迹接口。
 */

#ifndef CONTROL_TRAJ_H
#define CONTROL_TRAJ_H

#include <stdbool.h>

/** @brief 速度斜坡跨周期状态。 */
typedef struct
{
    float ref;
} traj_spd_t;

/** @brief 在线位置轨迹跨周期状态。 */
typedef struct
{
    float pos;
    float spd;
    bool done;
} traj_pos_t;

/** @brief 把速度轨迹复位到指定速度。 */
void traj_spd_reset(traj_spd_t *traj, float spd);

/**
 * @brief 按加减速度限制，把速度参考向目标值推进一个周期。
 *
 * @return 本周期新的速度参考，单位 rad/s。
 * @pre acc、dec 和 dt 大于 0，所有参数均为有限值。
 */
float traj_spd_step(traj_spd_t *traj,
                    float target,
                    float acc,
                    float dec,
                    float dt);

/** @brief 从给定位置和速度重新开始位置轨迹。 */
void traj_pos_reset(traj_pos_t *traj, float pos, float spd);

/**
 * @brief 按最大速度、加速度和减速度推进一个在线梯形位置轨迹。
 *
 * 算法根据剩余距离实时计算制动速度，因此短行程自动形成三角形轨迹，长行程
 * 自动形成梯形轨迹，不依赖电机、编码器或 PWM。
 *
 * @return 本周期新的位置参考，单位 rad。
 * @pre spd_lim、acc、dec 和 dt 大于 0，所有参数均为有限值。
 */
float traj_pos_step(traj_pos_t *traj,
                    float target,
                    float spd_lim,
                    float acc,
                    float dec,
                    float dt);

#endif /* CONTROL_TRAJ_H */
