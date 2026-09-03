/**
 * @file test_drive_protection.c
 * @brief 在 Host 上验证连续样本保护和故障锁存行为。
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "drive_protection.h"

static bool expect_true(bool condition, const char *message)
{
    if (condition)
    {
        return true;
    }

    fprintf(stderr, "%s\n", message);
    return false;
}

static drive_protection_config_t test_config(void)
{
    return (drive_protection_config_t){
        .under_voltage_v = 10.0f,
        .over_voltage_v = 50.0f,
        .over_current_a = 5.0f,
        .mos_over_temperature_c = 80.0f,
        .coil_over_temperature_c = 90.0f,
        .over_speed_rad_s = 100.0f,
        .under_voltage_samples = 3U,
        .over_voltage_samples = 2U,
        .over_current_samples = 1U,
        .mos_over_temperature_samples = 2U,
        .coil_over_temperature_samples = 2U,
        .over_speed_samples = 2U,
        .invalid_current_samples = 2U,
        .invalid_bus_voltage_samples = 2U,
        .invalid_position_samples = 2U,
    };
}

static drive_protection_sample_t safe_sample(void)
{
    return (drive_protection_sample_t){
        .ia = 1.0f,
        .ib = -1.0f,
        .ic = 0.0f,
        .vbus = 24.0f,
        .temp_mos = 30.0f,
        .temp_coil = 35.0f,
        .spd = 20.0f,
        .i_valid = true,
        .vbus_valid = true,
        .temp_mos_valid = true,
        .temp_coil_valid = true,
        .spd_valid = true,
        .pos_valid = true,
        .pwm_on = true,
    };
}

static bool test_safe_sample_and_reset(void)
{
    const drive_protection_config_t config = test_config();
    const drive_protection_sample_t sample = safe_sample();
    drive_protection_state_t state = {0};

    if (!expect_true(drive_protection_step(&state, &config, &sample) == 0U,
                     "正常反馈不应产生故障。"))
    {
        return false;
    }

    state.latched_faults = DRIVE_PROTECTION_FAULT_OVER_CURRENT;
    state.over_current_count = 1U;
    drive_protection_reset(&state);

    return expect_true(state.latched_faults == 0U,
                       "显式复位应清除锁存故障。") &&
           expect_true(state.over_current_count == 0U,
                       "显式复位应清除连续样本计数。");
}

static bool test_over_current_is_immediate_and_latched(void)
{
    const drive_protection_config_t config = test_config();
    drive_protection_sample_t sample = safe_sample();
    drive_protection_state_t state = {0};

    sample.ib = -5.1f;
    uint32_t faults = drive_protection_step(&state, &config, &sample);
    if (!expect_true((faults & DRIVE_PROTECTION_FAULT_OVER_CURRENT) != 0U,
                     "任一相电流超过阈值应立即锁存过流。"))
    {
        return false;
    }

    sample = safe_sample();
    faults = drive_protection_step(&state, &config, &sample);
    return expect_true((faults & DRIVE_PROTECTION_FAULT_OVER_CURRENT) != 0U,
                       "反馈恢复正常后，过流故障仍应保持锁存。");
}

static bool test_under_voltage_requires_active_power_stage(void)
{
    const drive_protection_config_t config = test_config();
    drive_protection_sample_t sample = safe_sample();
    drive_protection_state_t state = {0};

    sample.vbus = 9.0f;
    sample.pwm_on = false;
    for (unsigned int index = 0U; index < 4U; index++)
    {
        (void)drive_protection_step(&state, &config, &sample);
    }

    if (!expect_true(state.under_voltage_count == 0U,
                     "功率级未请求运行时不应累计欠压计数。"))
    {
        return false;
    }

    sample.pwm_on = true;
    (void)drive_protection_step(&state, &config, &sample);
    (void)drive_protection_step(&state, &config, &sample);
    if (!expect_true(state.latched_faults == 0U,
                     "欠压未达到连续样本数前不应锁存。"))
    {
        return false;
    }

    sample.vbus = 24.0f;
    (void)drive_protection_step(&state, &config, &sample);
    if (!expect_true(state.under_voltage_count == 0U,
                     "正常样本应清除未完成的欠压计数。"))
    {
        return false;
    }

    sample.vbus = 9.0f;
    for (unsigned int index = 0U; index < 3U; index++)
    {
        (void)drive_protection_step(&state, &config, &sample);
    }

    return expect_true(
        (state.latched_faults & DRIVE_PROTECTION_FAULT_UNDER_VOLTAGE) != 0U,
        "连续欠压达到样本数后应锁存故障。");
}

static bool test_other_limits_and_invalid_samples(void)
{
    const drive_protection_config_t config = test_config();
    drive_protection_sample_t sample = safe_sample();
    drive_protection_state_t state = {0};

    sample.ia = NAN;
    sample.vbus = NAN;
    sample.temp_mos = NAN;
    sample.temp_coil = NAN;
    sample.spd = NAN;
    sample.pos_valid = false;
    (void)drive_protection_step(&state, &config, &sample);
    if (!expect_true(state.latched_faults == 0U,
                     "非有限样本应交给输入有效性故障处理，不能误判物理超限。"))
    {
        return false;
    }

    const uint32_t invalid_faults = drive_protection_step(&state, &config, &sample);
    if (!expect_true(
            (invalid_faults & DRIVE_PROTECTION_FAULT_CURRENT_FEEDBACK) != 0U,
            "连续无效电流样本应锁存电流反馈故障。") ||
        !expect_true(
            (invalid_faults & DRIVE_PROTECTION_FAULT_BUS_FEEDBACK) != 0U,
            "连续无效母线样本应锁存母线反馈故障。") ||
        !expect_true(
            (invalid_faults & DRIVE_PROTECTION_FAULT_POSITION_FEEDBACK) != 0U,
            "连续无效位置样本应锁存位置反馈故障。"))
    {
        return false;
    }

    drive_protection_reset(&state);

    sample = safe_sample();
    sample.vbus = 51.0f;
    sample.temp_mos = 81.0f;
    sample.temp_coil = 91.0f;
    sample.spd = -101.0f;
    (void)drive_protection_step(&state, &config, &sample);
    const uint32_t faults = drive_protection_step(&state, &config, &sample);

    return expect_true((faults & DRIVE_PROTECTION_FAULT_OVER_VOLTAGE) != 0U,
                       "连续过压应锁存故障。") &&
           expect_true((faults & DRIVE_PROTECTION_FAULT_MOS_OVER_TEMPERATURE) != 0U,
                       "连续 MOS 过温应锁存故障。") &&
           expect_true((faults & DRIVE_PROTECTION_FAULT_COIL_OVER_TEMPERATURE) != 0U,
                       "连续线圈过温应锁存故障。") &&
           expect_true((faults & DRIVE_PROTECTION_FAULT_OVER_SPEED) != 0U,
                       "连续反向超速也应按绝对值锁存故障。");
}

int main(void)
{
    if (!test_safe_sample_and_reset())
    {
        return 1;
    }

    if (!test_over_current_is_immediate_and_latched())
    {
        return 2;
    }

    if (!test_under_voltage_requires_active_power_stage())
    {
        return 3;
    }

    if (!test_other_limits_and_invalid_samples())
    {
        return 4;
    }

    return 0;
}
