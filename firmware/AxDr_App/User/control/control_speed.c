/**
 * @file control_speed.c
 * @brief 与硬件无关的角度差分测速实现。
 */

#include "control_speed.h"

#define CONTROL_SPEED_RAM_FUNC __attribute__((section(".RamFunc")))

static const float control_speed_pi = 3.14159265358f;
static const float control_speed_two_pi = 6.28318530716f;

CONTROL_SPEED_RAM_FUNC float control_angle_speed_step(
    control_angle_speed_state_t *state,
    float angle_rad,
    float sample_frequency_hz)
{
    float speed_rad_s;

    state->delta_angle_rad = angle_rad - state->previous_angle_rad;
    state->delta_angle_rad = (state->delta_angle_rad > control_speed_pi) ?
                             state->delta_angle_rad - control_speed_two_pi :
                             state->delta_angle_rad;
    state->delta_angle_rad = (state->delta_angle_rad < -control_speed_pi) ?
                             state->delta_angle_rad + control_speed_two_pi :
                             state->delta_angle_rad;

    speed_rad_s = state->delta_angle_rad * sample_frequency_hz;
    state->previous_angle_rad = angle_rad;

    return speed_rad_s;
}
