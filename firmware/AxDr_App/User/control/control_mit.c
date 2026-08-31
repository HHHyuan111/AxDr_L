/**
 * @file control_mit.c
 * @brief MIT 位置、速度和前馈转矩合成的可移植实现。
 */

#include "control_mit.h"

#include "compiler.h"

PLATFORM_FAST_CODE void control_mit_step(
    const control_mit_input_t *input,
    control_mit_output_t *output)
{
    const float position_torque_nm =
        input->position_gain_nm_per_rad *
        (input->position_ref_rad - input->position_feedback_rad);
    const float speed_torque_nm =
        input->speed_gain_nm_s_per_rad *
        (input->speed_ref_rad_s - input->speed_feedback_rad_s);

    output->torque_cmd_nm = position_torque_nm +
                            speed_torque_nm +
                            input->torque_feedforward_nm;
}
