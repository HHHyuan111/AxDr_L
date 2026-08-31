/**
 * @file drive_protection.c
 * @brief 驱动保护连续样本判定和故障锁存实现。
 */

#include "drive_protection.h"

#include <math.h>
#include <string.h>

#include "compiler.h"

static PLATFORM_FAST_CODE bool drive_protection_limit_enabled(
    float threshold,
    uint32_t sample_limit)
{
    return isfinite(threshold) && (threshold > 0.0f) && (sample_limit > 0U);
}

static PLATFORM_FAST_CODE void drive_protection_update_counter(
    uint32_t *count,
    uint32_t sample_limit,
    bool limit_exceeded,
    uint32_t fault_mask,
    uint32_t *latched_faults)
{
    if (!limit_exceeded)
    {
        *count = 0U;
        return;
    }

    if (*count < sample_limit)
    {
        (*count)++;
    }

    if (*count >= sample_limit)
    {
        *latched_faults |= fault_mask;
    }
}

PLATFORM_FAST_CODE uint32_t drive_protection_step(
    drive_protection_state_t *state,
    const drive_protection_config_t *config,
    const drive_protection_sample_t *sample)
{
    const bool currents_valid = sample->currents_valid &&
                                isfinite(sample->current_a_a) &&
                                isfinite(sample->current_b_a) &&
                                isfinite(sample->current_c_a);
    const bool bus_voltage_valid = sample->bus_voltage_valid &&
                                   isfinite(sample->bus_voltage_v);
    const bool mos_temperature_valid = sample->mos_temperature_valid &&
                                       isfinite(sample->mos_temperature_c);
    const bool coil_temperature_valid = sample->coil_temperature_valid &&
                                        isfinite(sample->coil_temperature_c);
    const bool rotor_speed_valid = sample->rotor_speed_valid &&
                                   isfinite(sample->rotor_speed_rad_s);
    const float max_phase_current_a = fmaxf(fabsf(sample->current_a_a),
                                            fmaxf(fabsf(sample->current_b_a),
                                                  fabsf(sample->current_c_a)));

    drive_protection_update_counter(
        &state->over_current_count,
        config->over_current_samples,
        currents_valid && drive_protection_limit_enabled(
            config->over_current_a,
            config->over_current_samples) &&
            (max_phase_current_a > config->over_current_a),
        DRIVE_PROTECTION_FAULT_OVER_CURRENT,
        &state->latched_faults);

    drive_protection_update_counter(
        &state->under_voltage_count,
        config->under_voltage_samples,
        sample->power_stage_active && bus_voltage_valid &&
            drive_protection_limit_enabled(
            config->under_voltage_v,
            config->under_voltage_samples) &&
            (sample->bus_voltage_v < config->under_voltage_v),
        DRIVE_PROTECTION_FAULT_UNDER_VOLTAGE,
        &state->latched_faults);

    drive_protection_update_counter(
        &state->over_voltage_count,
        config->over_voltage_samples,
        bus_voltage_valid && drive_protection_limit_enabled(
            config->over_voltage_v,
            config->over_voltage_samples) &&
            (sample->bus_voltage_v > config->over_voltage_v),
        DRIVE_PROTECTION_FAULT_OVER_VOLTAGE,
        &state->latched_faults);

    drive_protection_update_counter(
        &state->mos_over_temperature_count,
        config->mos_over_temperature_samples,
        mos_temperature_valid && drive_protection_limit_enabled(
            config->mos_over_temperature_c,
            config->mos_over_temperature_samples) &&
            (sample->mos_temperature_c > config->mos_over_temperature_c),
        DRIVE_PROTECTION_FAULT_MOS_OVER_TEMPERATURE,
        &state->latched_faults);

    drive_protection_update_counter(
        &state->coil_over_temperature_count,
        config->coil_over_temperature_samples,
        coil_temperature_valid && drive_protection_limit_enabled(
            config->coil_over_temperature_c,
            config->coil_over_temperature_samples) &&
            (sample->coil_temperature_c > config->coil_over_temperature_c),
        DRIVE_PROTECTION_FAULT_COIL_OVER_TEMPERATURE,
        &state->latched_faults);

    drive_protection_update_counter(
        &state->over_speed_count,
        config->over_speed_samples,
        rotor_speed_valid && drive_protection_limit_enabled(
            config->over_speed_rad_s,
            config->over_speed_samples) &&
            (fabsf(sample->rotor_speed_rad_s) > config->over_speed_rad_s),
        DRIVE_PROTECTION_FAULT_OVER_SPEED,
        &state->latched_faults);

    return state->latched_faults;
}

void drive_protection_reset(drive_protection_state_t *state)
{
    memset(state, 0, sizeof(*state));
}
