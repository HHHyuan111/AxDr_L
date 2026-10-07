/**
 * @file diag_runtime.c
 * @brief 电机诊断与参数辨识任务的统一运行实现。
 */

#include "diag_runtime.h"

#include <math.h>
#include <string.h>

#include "algorithm_config.h"

static mc_diag_owner_t diag_owner_from_job(diag_job_e job)
{
    switch (job)
    {
        case DIAG_JOB_CURRENT_SWEEP:
            return MC_DIAG_OWNER_CURRENT_SWEEP;

        case DIAG_JOB_PARAM_IDENT:
        case DIAG_JOB_PARAM_IDENT_LQ:
            return MC_DIAG_OWNER_PARAM;

        case DIAG_JOB_POLE_PAIR_IDENT:
            return MC_DIAG_OWNER_POLE_PAIR;

        case DIAG_JOB_ENCODER_ALIGN:
            return MC_DIAG_OWNER_ENCODER_ALIGN;

        case DIAG_JOB_DEADTIME_TEST:
            return MC_DIAG_OWNER_DEADTIME;

        case DIAG_JOB_NONE:
        default:
            return MC_DIAG_OWNER_NONE;
    }
}

static void diag_stop_command(mc_command_t *command)
{
    memset(command, 0, sizeof(*command));
    command->disable_request = true;
}

static void diag_align_current_command(float current_a,
                                       float theta_e_rad,
                                       mc_command_t *command)
{
    memset(command, 0, sizeof(*command));
    command->mode = MC_CONTROL_CURRENT;
    command->id_ref_a = current_a;
    command->openloop_theta_e_rad = theta_e_rad;
    command->openloop_enable = true;
    command->enable_request = true;
}

static bool diag_common_limits_valid(const diag_profile_t *profile)
{
    return mc_float_is_finite(profile->current_limit_a)
        && mc_float_is_finite(profile->voltage_limit_v)
        && mc_float_is_finite(profile->minimum_vbus_v)
        && (profile->current_limit_a > 0.0f)
        && (profile->voltage_limit_v > 0.0f)
        && (profile->minimum_vbus_v >= 0.0f);
}

static void diag_profile_init(diag_profile_t *profile,
                              const diag_seed_t *seed)
{
    memset(profile, 0, sizeof(*profile));

    profile->control_period_s = seed->control_period_s;
    profile->phase_resistance_ohm = seed->phase_resistance_ohm;
    profile->pole_pairs = seed->pole_pairs;
    profile->encoder_full_scale = seed->encoder_full_scale;
    profile->encoder_direction = seed->encoder_direction;

    /* 算法核心提供通用初值，本项目参数统一由配置头覆盖。 */
    mc_current_sweep_default_config(&profile->sweep);
    profile->sweep.axis = DIAG_SWEEP_AXIS;
    profile->sweep.single_point = DIAG_SWEEP_SINGLE_POINT;
    profile->sweep.single_frequency_hz = DIAG_SWEEP_SINGLE_FREQ_HZ;
    profile->sweep.start_frequency_hz = DIAG_SWEEP_START_FREQ_HZ;
    profile->sweep.end_frequency_hz = DIAG_SWEEP_END_FREQ_HZ;
    profile->sweep.amplitude_a = DIAG_SWEEP_AMPLITUDE_A;
    profile->sweep.offset_a = DIAG_SWEEP_OFFSET_A;
    profile->sweep.requested_points = DIAG_SWEEP_POINT_COUNT;
    profile->sweep.settle_cycles = DIAG_SWEEP_SETTLE_CYCLES;
    profile->sweep.measure_cycles = DIAG_SWEEP_MEASURE_CYCLES;
    profile->sweep.minimum_settle_time_s =
        DIAG_SWEEP_MIN_SETTLE_TIME_S;
    profile->sweep.minimum_measure_time_s =
        DIAG_SWEEP_MIN_MEASURE_TIME_S;
    profile->sweep.minimum_measure_samples =
        DIAG_SWEEP_MIN_MEASURE_SAMPLES;
    profile->sweep.feedback_filter_alpha =
        DIAG_SWEEP_FEEDBACK_FILTER_ALPHA;
    profile->sweep.reject_saturated_points =
        DIAG_SWEEP_REJECT_SATURATED_POINTS;

    /* 参数辨识（mc_param_ident）无 profile 种子：激励层用核内置沉沙
     * 默认档案，平台参数与安全限值在 diag_start_job 桥接时由 seed 与
     * profile 限值覆盖。 */

    mc_pole_pair_ident_default_config(&profile->pole_pair);
    profile->pole_pair.control_period_s = seed->control_period_s;
    profile->pole_pair.phase_resistance_ohm = seed->phase_resistance_ohm;
    profile->pole_pair.drive_current_a = DIAG_POLE_PAIR_DRIVE_CURRENT_A;
    profile->pole_pair.electrical_turns =
        DIAG_POLE_PAIR_ELECTRICAL_TURNS;
    profile->pole_pair.electrical_velocity_rad_s =
        DIAG_POLE_PAIR_ELECTRICAL_SPEED_RAD_S;
    profile->pole_pair.ramp_duration_s =
        DIAG_POLE_PAIR_RAMP_DURATION_S;
    profile->pole_pair.encoder_full_scale = seed->encoder_full_scale;
    profile->pole_pair.min_pole_pairs = DIAG_POLE_PAIR_MIN_COUNT;
    profile->pole_pair.max_pole_pairs = DIAG_POLE_PAIR_MAX_COUNT;

    profile->encoder_align.align_current_a = DIAG_ALIGN_CURRENT_A;
    profile->encoder_align.ramp_duration_s = DIAG_ALIGN_RAMP_DURATION_S;
    profile->encoder_align.hold_duration_s = DIAG_ALIGN_HOLD_DURATION_S;
    profile->encoder_align.target_electrical_angle_rad =
        DIAG_ALIGN_TARGET_ELEC_ANGLE_RAD;
    profile->encoder_align.sample_count = DIAG_ALIGN_SAMPLE_COUNT;
    profile->encoder_align.sample_interval_ticks =
        DIAG_ALIGN_SAMPLE_INTERVAL_TICKS;
    profile->encoder_align.encoder_direction = seed->encoder_direction;
    profile->encoder_align.method = DIAG_ALIGN_METHOD;
    profile->encoder_align.minimum_resultant_ratio =
        DIAG_ALIGN_MIN_RESULTANT_RATIO;

    mc_deadtime_test_default_config(&profile->deadtime_test);
    profile->deadtime_test.target_line_voltage_v =
        DIAG_DEADTIME_TARGET_LINE_VOLTAGE_V;

    profile->decoupling_mode = DIAG_DECOUPLING_MODE;
    profile->decoupling.rs_ohm = seed->phase_resistance_ohm;
    profile->decoupling.ld_h = seed->d_axis_inductance_h;
    profile->decoupling.lq_h = seed->q_axis_inductance_h;
    profile->decoupling.flux_wb = seed->flux_linkage_wb;
    profile->decoupling.pole_pairs = (float)seed->pole_pairs;
    profile->decoupling.dt_s = seed->control_period_s;
    profile->decoupling.include_resistive_feedforward =
        DIAG_DECOUPLING_INCLUDE_RESISTIVE_FF;

    profile->flux.rs_ohm = seed->phase_resistance_ohm;
    profile->flux.pole_pairs = (float)seed->pole_pairs;
    profile->flux.sample_time_s = seed->control_period_s;
    profile->flux.filter_cutoff_hz = DIAG_FLUX_FILTER_CUTOFF_HZ;
    profile->flux.minimum_electrical_speed_rad_s =
        DIAG_FLUX_MIN_ELEC_SPEED_RAD_S;
    profile->flux.reject_voltage_saturation =
        DIAG_FLUX_REJECT_VOLTAGE_SATURATION;
}

void diag_runtime_init(diag_runtime_t *runtime,
                       const diag_seed_t *seed)
{
    if ((runtime == NULL) || (seed == NULL))
    {
        return;
    }

    memset(runtime, 0, sizeof(*runtime));
    mc_diag_manager_init(&runtime->manager);
    mc_flux_observer_init(&runtime->flux);
    diag_profile_init(&runtime->profile, seed);
    runtime->last_status = MC_OK;
}

static bool diag_align_config_valid(const diag_runtime_t *runtime)
{
    const diag_align_config_t *config = &runtime->profile.encoder_align;

    return mc_float_is_finite(config->align_current_a)
        && mc_float_is_finite(config->ramp_duration_s)
        && mc_float_is_finite(config->hold_duration_s)
        && mc_float_is_finite(config->target_electrical_angle_rad)
        && mc_float_is_finite(config->minimum_resultant_ratio)
        && (config->align_current_a > 0.0f)
        && (config->align_current_a <= runtime->profile.current_limit_a)
        && (config->ramp_duration_s > 0.0f)
        && (config->hold_duration_s > 0.0f)
        && (config->sample_count > 0U)
        && (config->sample_count <= DIAG_ALIGN_MAX_SAMPLES)
        && (config->sample_interval_ticks > 0U)
        && ((config->encoder_direction == 1)
            || (config->encoder_direction == -1))
        && (config->minimum_resultant_ratio >= 0.0f)
        && (config->minimum_resultant_ratio <= 1.0f)
        && (runtime->profile.control_period_s > 0.0f)
        && (runtime->profile.encoder_full_scale > 1U)
        && (runtime->profile.pole_pairs > 0U);
}

static mc_status_t diag_align_start(diag_runtime_t *runtime)
{
    diag_align_runtime_t *align = &runtime->encoder_align;
    const float sample_time_s = runtime->profile.control_period_s;

    if (!diag_align_config_valid(runtime))
    {
        return MC_INVALID_ARGUMENT;
    }

    memset(align, 0, sizeof(*align));
    align->ramp_ticks = (uint32_t)(
        runtime->profile.encoder_align.ramp_duration_s / sample_time_s);
    align->hold_ticks = (uint32_t)(
        runtime->profile.encoder_align.hold_duration_s / sample_time_s);
    if ((align->ramp_ticks == 0U) || (align->hold_ticks == 0U))
    {
        return MC_OUT_OF_RANGE;
    }

    align->state = DIAG_ALIGN_RAMP;
    return MC_OK;
}

static mc_status_t diag_start_job(diag_runtime_t *runtime)
{
    switch (runtime->active_job)
    {
        case DIAG_JOB_CURRENT_SWEEP:
        {
            mc_current_sweep_config_t config = runtime->profile.sweep;

            config.safe_current_limit_a = runtime->profile.current_limit_a;
            runtime->command_primed = true;
            return mc_current_sweep_start(&runtime->sweep,
                                          &config,
                                          runtime->profile.control_period_s,
                                          &runtime->command);
        }

        case DIAG_JOB_PARAM_IDENT:
        case DIAG_JOB_PARAM_IDENT_LQ:
        {
            mc_param_ident_config_t config;
            mc_param_ident_mode_t mode =
                (runtime->active_job == DIAG_JOB_PARAM_IDENT_LQ)
                    ? MC_PARAM_IDENT_MODE_LQ
                    : MC_PARAM_IDENT_MODE_FULL;

            /* 激励层用核内置沉沙默认档案；控制周期/极对数取 seed，
             * 安全限值与母线下限取 profile 闸门（0=禁止启动的语义
             * 两层一致）。 */
            mc_param_ident_default_config(&config);
            config.control_period_s = runtime->profile.control_period_s;
            config.pole_pairs = (float)runtime->profile.pole_pairs;
            config.current_limit_a = runtime->profile.current_limit_a;
            config.voltage_limit_v = runtime->profile.voltage_limit_v;
            config.vbus_min_v = runtime->profile.minimum_vbus_v;
            return mc_param_ident_start(&runtime->param_ident,
                                        &config,
                                        mode);
        }

        case DIAG_JOB_POLE_PAIR_IDENT:
        {
            mc_pole_pair_ident_config_t config = runtime->profile.pole_pair;

            config.phase_resistance_ohm =
                runtime->profile.phase_resistance_ohm;
            config.safe_current_limit_a = runtime->profile.current_limit_a;
            config.max_abs_voltage_v = runtime->profile.voltage_limit_v;
            config.min_vbus_v = runtime->profile.minimum_vbus_v;
            return mc_pole_pair_ident_start(&runtime->pole_pair, &config);
        }

        case DIAG_JOB_ENCODER_ALIGN:
            return diag_align_start(runtime);

        case DIAG_JOB_DEADTIME_TEST:
        {
            mc_deadtime_test_config_t config = runtime->profile.deadtime_test;

            config.minimum_vbus_v = runtime->profile.minimum_vbus_v;
            config.maximum_abs_d_axis_voltage_v =
                runtime->profile.voltage_limit_v;
            config.maximum_abs_phase_current_a =
                runtime->profile.current_limit_a;
            runtime->command_primed = true;
            return mc_deadtime_test_start(&runtime->deadtime_test,
                                          &config,
                                          &runtime->command);
        }

        case DIAG_JOB_NONE:
        default:
            return MC_INVALID_ARGUMENT;
    }
}

static void diag_finish_start_failure(diag_runtime_t *runtime,
                                      mc_diag_owner_t owner,
                                      mc_status_t status)
{
    runtime->last_status = status;
    (void)mc_diag_finish(&runtime->manager, owner, status);
    (void)mc_diag_confirm_stopped(&runtime->manager, owner, true);
    (void)mc_diag_release(&runtime->manager, owner);
    runtime->active = false;
    runtime->active_job = DIAG_JOB_NONE;
}

mc_status_t diag_runtime_start(diag_runtime_t *runtime,
                               float initial_vbus_v,
                               bool platform_ready_and_disabled)
{
    mc_diag_owner_t owner;
    mc_status_t status;

    if ((runtime == NULL)
        || !mc_float_is_finite(initial_vbus_v)
        || !diag_common_limits_valid(&runtime->profile))
    {
        return MC_INVALID_ARGUMENT;
    }

    if ((runtime->profile.minimum_vbus_v > 0.0f)
        && (initial_vbus_v < runtime->profile.minimum_vbus_v))
    {
        return MC_OUT_OF_RANGE;
    }

    runtime->active_job = (diag_job_e)runtime->requested_job;
    owner = diag_owner_from_job(runtime->active_job);
    if (owner == MC_DIAG_OWNER_NONE)
    {
        runtime->active_job = DIAG_JOB_NONE;
        return MC_INVALID_ARGUMENT;
    }

    status = mc_diag_acquire(&runtime->manager,
                             owner,
                             platform_ready_and_disabled);
    if (status != MC_OK)
    {
        runtime->active_job = DIAG_JOB_NONE;
        return status;
    }

    memset(&runtime->command, 0, sizeof(runtime->command));
    runtime->command_primed = false;
    runtime->voltage_saturated = false;
    status = diag_start_job(runtime);
    if (status != MC_OK)
    {
        diag_finish_start_failure(runtime, owner, status);
        return status;
    }

    status = mc_diag_begin(&runtime->manager, owner);
    if (status != MC_OK)
    {
        diag_finish_start_failure(runtime, owner, status);
        return status;
    }

    runtime->active = true;
    runtime->last_status = MC_BUSY;
    return MC_OK;
}

static bool diag_sample_exceeds_current_limit(const mc_sample_t *sample,
                                              float current_limit_a)
{
    return (fabsf(sample->ia_a) > current_limit_a)
        || (fabsf(sample->ib_a) > current_limit_a)
        || (fabsf(sample->ic_a) > current_limit_a);
}

static mc_status_t diag_align_step(diag_runtime_t *runtime,
                                   const mc_sample_t *sample,
                                   mc_command_t *command)
{
    diag_align_runtime_t *align = &runtime->encoder_align;
    const diag_align_config_t *config = &runtime->profile.encoder_align;
    const uint32_t full_scale = runtime->profile.encoder_full_scale;
    const uint32_t pole_pairs = runtime->profile.pole_pairs;
    float current_a = config->align_current_a;

    if ((sample->fault_code != 0U)
        || diag_sample_exceeds_current_limit(
            sample,
            runtime->profile.current_limit_a))
    {
        align->state = DIAG_ALIGN_ERROR;
        diag_stop_command(command);
        return MC_FAULT;
    }

    if ((runtime->profile.minimum_vbus_v > 0.0f)
        && (sample->vbus_v < runtime->profile.minimum_vbus_v))
    {
        align->state = DIAG_ALIGN_ERROR;
        diag_stop_command(command);
        return MC_FAULT;
    }

    switch (align->state)
    {
        case DIAG_ALIGN_RAMP:
            current_a *= (float)align->tick / (float)align->ramp_ticks;
            diag_align_current_command(
                current_a,
                config->target_electrical_angle_rad,
                command);
            align->tick++;
            if (align->tick >= align->ramp_ticks)
            {
                align->state = DIAG_ALIGN_HOLD;
                align->tick = 0U;
            }
            return MC_BUSY;

        case DIAG_ALIGN_HOLD:
            diag_align_current_command(
                current_a,
                config->target_electrical_angle_rad,
                command);
            align->tick++;
            if (align->tick >= align->hold_ticks)
            {
                align->state = DIAG_ALIGN_SAMPLE;
                align->sample_tick = 0U;
                align->sample_index = 0U;
            }
            return MC_BUSY;

        case DIAG_ALIGN_SAMPLE:
            diag_align_current_command(
                current_a,
                config->target_electrical_angle_rad,
                command);
            align->sample_tick++;
            if (align->sample_tick >= config->sample_interval_ticks)
            {
                align->sample_tick = 0U;
                align->raw_mechanical_angle_rad[align->sample_index] =
                    MC_TWO_PI_F * (float)sample->encoder_raw
                    / (float)full_scale;
                align->sample_index++;
            }

            if (align->sample_index >= config->sample_count)
            {
                const mc_status_t status = mc_encoder_alignment_solve(
                    align->raw_mechanical_angle_rad,
                    config->sample_count,
                    pole_pairs,
                    config->encoder_direction,
                    config->target_electrical_angle_rad,
                    full_scale,
                    config->method,
                    config->minimum_resultant_ratio,
                    &align->result);

                align->state = (status == MC_OK)
                    ? DIAG_ALIGN_DONE
                    : DIAG_ALIGN_ERROR;
                diag_stop_command(command);
                return (status == MC_OK) ? MC_DONE : status;
            }
            return MC_BUSY;

        case DIAG_ALIGN_DONE:
            diag_stop_command(command);
            return MC_DONE;

        case DIAG_ALIGN_IDLE:
        case DIAG_ALIGN_ERROR:
        default:
            diag_stop_command(command);
            return MC_REJECTED;
    }
}

static mc_status_t diag_step_job(diag_runtime_t *runtime,
                                 const mc_sample_t *sample,
                                 mc_command_t *command)
{
    switch (runtime->active_job)
    {
        case DIAG_JOB_CURRENT_SWEEP:
            return mc_current_sweep_step(&runtime->sweep, sample, command);

        case DIAG_JOB_PARAM_IDENT:
        case DIAG_JOB_PARAM_IDENT_LQ:
            return mc_param_ident_step(&runtime->param_ident,
                                       sample,
                                       command);

        case DIAG_JOB_POLE_PAIR_IDENT:
            return mc_pole_pair_ident_step(
                &runtime->pole_pair,
                sample,
                command);

        case DIAG_JOB_ENCODER_ALIGN:
            return diag_align_step(runtime, sample, command);

        case DIAG_JOB_DEADTIME_TEST:
            return mc_deadtime_test_step(
                &runtime->deadtime_test,
                sample,
                command);

        case DIAG_JOB_NONE:
        default:
            diag_stop_command(command);
            return MC_REJECTED;
    }
}

mc_status_t diag_runtime_step(diag_runtime_t *runtime,
                              const mc_sample_t *sample,
                              mc_command_t *command)
{
    mc_status_t status;

    if ((runtime == NULL) || (sample == NULL) || (command == NULL))
    {
        return MC_INVALID_ARGUMENT;
    }
    if (!runtime->active || (runtime->manager.state != MC_DIAG_RUNNING))
    {
        diag_stop_command(command);
        return MC_REJECTED;
    }

    runtime->last_sample = *sample;
    if (runtime->command_primed)
    {
        *command = runtime->command;
        runtime->command_primed = false;
        return MC_BUSY;
    }

    status = diag_step_job(runtime, sample, command);
    runtime->command = *command;
    runtime->last_status = status;
    if (status != MC_BUSY)
    {
        (void)mc_diag_finish(
            &runtime->manager,
            diag_owner_from_job(runtime->active_job),
            status);
    }

    return status;
}

mc_status_t diag_runtime_abort(diag_runtime_t *runtime,
                               mc_command_t *command)
{
    mc_status_t status;

    if ((runtime == NULL) || (command == NULL))
    {
        return MC_INVALID_ARGUMENT;
    }
    if (!runtime->active)
    {
        diag_stop_command(command);
        return MC_REJECTED;
    }

    switch (runtime->active_job)
    {
        case DIAG_JOB_CURRENT_SWEEP:
            status = mc_current_sweep_abort(&runtime->sweep, command);
            break;

        case DIAG_JOB_PARAM_IDENT:
        case DIAG_JOB_PARAM_IDENT_LQ:
            status = mc_param_ident_abort(&runtime->param_ident, command);
            break;

        case DIAG_JOB_POLE_PAIR_IDENT:
            status = mc_pole_pair_ident_abort(
                &runtime->pole_pair,
                command);
            break;

        case DIAG_JOB_DEADTIME_TEST:
            status = mc_deadtime_test_abort(
                &runtime->deadtime_test,
                command);
            break;

        case DIAG_JOB_ENCODER_ALIGN:
            runtime->encoder_align.state = DIAG_ALIGN_ERROR;
            diag_stop_command(command);
            status = MC_ABORTED;
            break;

        case DIAG_JOB_NONE:
        default:
            diag_stop_command(command);
            status = MC_REJECTED;
            break;
    }

    runtime->command = *command;
    runtime->last_status = status;
    if ((runtime->manager.state == MC_DIAG_RUNNING)
        || (runtime->manager.state == MC_DIAG_PRECHECK))
    {
        (void)mc_diag_finish(
            &runtime->manager,
            diag_owner_from_job(runtime->active_job),
            status);
    }
    return status;
}

void diag_runtime_confirm_stopped(diag_runtime_t *runtime,
                                  bool platform_ready_and_disabled)
{
    mc_diag_owner_t owner;

    if ((runtime == NULL) || !runtime->active)
    {
        return;
    }

    owner = diag_owner_from_job(runtime->active_job);
    if (runtime->manager.state == MC_DIAG_RUNNING)
    {
        (void)mc_diag_finish(&runtime->manager, owner, MC_ABORTED);
        runtime->last_status = MC_ABORTED;
    }

    if (mc_diag_confirm_stopped(
            &runtime->manager,
            owner,
            platform_ready_and_disabled) != MC_OK)
    {
        return;
    }

    (void)mc_diag_release(&runtime->manager, owner);
    runtime->active = false;
    runtime->active_job = DIAG_JOB_NONE; /* 与 diag_finish_start_failure 对齐，防残留旧 job 号 */
    runtime->command_primed = false;
    runtime->voltage_saturated = false;
    memset(&runtime->command, 0, sizeof(runtime->command));
}

static bool diag_job_is_known(diag_job_e job)
{
    switch (job)
    {
        case DIAG_JOB_CURRENT_SWEEP:
        case DIAG_JOB_POLE_PAIR_IDENT:
        case DIAG_JOB_ENCODER_ALIGN:
        case DIAG_JOB_DEADTIME_TEST:
        case DIAG_JOB_PARAM_IDENT:
        case DIAG_JOB_PARAM_IDENT_LQ:
            return true;

        case DIAG_JOB_NONE:
        default:
            return false; /* 含 2/3 退役空洞：旧脚本误触即拒 */
    }
}

mc_status_t diag_runtime_request_start(diag_runtime_t *runtime,
                                       diag_job_e job)
{
    if ((runtime == NULL) || !diag_job_is_known(job))
    {
        return MC_INVALID_ARGUMENT;
    }
    if (runtime->active || (runtime->request != DIAG_REQUEST_NONE))
    {
        return MC_REJECTED;
    }

    runtime->requested_job = (uint32_t)job;
    runtime->request = DIAG_REQUEST_START;
    return MC_OK;
}

void diag_runtime_request_stop(diag_runtime_t *runtime)
{
    if (runtime == NULL)
    {
        return;
    }
    runtime->request = DIAG_REQUEST_STOP;
}

mc_status_t diag_current_pi_for_project(float bandwidth_hz,
                                        float sample_time_s,
                                        float rs_ohm,
                                        float ld_h,
                                        float lq_h,
                                        float *kp_d,
                                        float *ki_d_per_s,
                                        float *kp_q,
                                        float *ki_q_per_s)
{
    float ki_d_per_tick;
    float ki_q_per_tick;
    mc_status_t status;

    if ((ki_d_per_s == NULL) || (ki_q_per_s == NULL)
        || !mc_float_is_finite(sample_time_s)
        || (sample_time_s <= 0.0f))
    {
        return MC_INVALID_ARGUMENT;
    }

    status = mc_current_pi_from_bandwidth(
        bandwidth_hz,
        sample_time_s,
        rs_ohm,
        ld_h,
        lq_h,
        kp_d,
        &ki_d_per_tick,
        kp_q,
        &ki_q_per_tick);
    if (status != MC_OK)
    {
        return status;
    }

    *ki_d_per_s = ki_d_per_tick / sample_time_s;
    *ki_q_per_s = ki_q_per_tick / sample_time_s;
    return MC_OK;
}
