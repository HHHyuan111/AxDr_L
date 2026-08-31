/**
 * @file control_loop.c
 * @brief 电压、电流、速度和位置 FOC 主链的可移植实现。
 */

#include "control_loop.h"

#include "compiler.h"

PLATFORM_FAST_CODE bool control_loop_step(
    control_loop_runtime_t *runtime,
    const control_loop_feedback_t *feedback,
    const control_loop_request_t *request,
    control_loop_output_t *output)
{
    foc_voltage_t voltage;

    *output = (control_loop_output_t){0};
    foc_core_prepare(&feedback->foc_sample, &output->frame);

    switch (request->mode)
    {
        case CONTROL_LOOP_MODE_VOLTAGE:
            runtime->voltage_d_v = request->voltage_d_v;
            runtime->voltage_q_v = request->voltage_q_v;
            break;

        case CONTROL_LOOP_MODE_CURRENT:
            (void)control_cur_step(&runtime->current_rate,
                                   runtime->current_d_pid,
                                   runtime->current_q_pid,
                                   request->current_d_ref_a,
                                   request->current_q_ref_a,
                                   output->frame.i_d,
                                   output->frame.i_q,
                                   &runtime->voltage_d_v,
                                   &runtime->voltage_q_v);
            break;

        case CONTROL_LOOP_MODE_SPEED:
            (void)control_spd_step(&runtime->speed_rate,
                                   runtime->speed_pid,
                                   request->speed_ref_rad_s,
                                   feedback->rotor_speed_rad_s,
                                   request->current_limit_a,
                                   &runtime->current_q_ref_a);
            (void)control_cur_step(&runtime->current_rate,
                                   runtime->current_d_pid,
                                   runtime->current_q_pid,
                                   request->current_d_ref_a,
                                   runtime->current_q_ref_a,
                                   output->frame.i_d,
                                   output->frame.i_q,
                                   &runtime->voltage_d_v,
                                   &runtime->voltage_q_v);
            break;

        case CONTROL_LOOP_MODE_POSITION:
            (void)control_pos_step(&runtime->position_rate,
                                   runtime->position_pid,
                                   request->position_ref_rad,
                                   feedback->rotor_position_rad,
                                   request->speed_limit_rad_s,
                                   &runtime->speed_ref_rad_s);
            (void)control_spd_step(&runtime->speed_rate,
                                   runtime->speed_pid,
                                   runtime->speed_ref_rad_s,
                                   feedback->rotor_speed_rad_s,
                                   request->current_limit_a,
                                   &runtime->current_q_ref_a);
            (void)control_cur_step(&runtime->current_rate,
                                   runtime->current_d_pid,
                                   runtime->current_q_pid,
                                   request->current_d_ref_a,
                                   runtime->current_q_ref_a,
                                   output->frame.i_d,
                                   output->frame.i_q,
                                   &runtime->voltage_d_v,
                                   &runtime->voltage_q_v);
            break;

        default:
            return false;
    }

    voltage = (foc_voltage_t){
        .v_d = runtime->voltage_d_v,
        .v_q = runtime->voltage_q_v,
        .inv_vbus = feedback->inv_bus_voltage,
    };

    output->voltage_d_v = runtime->voltage_d_v;
    output->voltage_q_v = runtime->voltage_q_v;
    output->current_q_ref_a = runtime->current_q_ref_a;
    output->speed_ref_rad_s = runtime->speed_ref_rad_s;
    output->duty_valid = foc_core_modulate(&output->frame,
                                           &voltage,
                                           &output->duty);

    return output->duty_valid;
}
