/**
 * @file test_fast_loop.c
 * @brief 使用 Fake 模块验证生产快速周期编排和对象传递。
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "board_adapter.h"
#include "common.h"
#include "drive.h"
#include "drive_cycle.h"
#include "fast_loop.h"
#include "main.h"
#include "position_adapter.h"
#include "target_irq_profile.h"

typedef enum
{
    EVENT_ENCODER_SAMPLE = 0,
    EVENT_POSITION_UPDATE,
    EVENT_ADC_SAMPLE,
    EVENT_FEEDBACK_UPDATE,
    EVENT_DRIVE_STEP,
    EVENT_OBSERVER_STEP,
    EVENT_SNAPSHOT_PUBLISH,
    EVENT_RECORD_PUBLISH
} fast_event_e;

#define EVENT_COUNT_PER_CYCLE (8U)
#define EVENT_CAPACITY        (24U)

foc_t g_foc;
ADC_TypeDef axdr_test_adc1;
unsigned SystemCoreClock = 160000000u;
static unsigned observation_calls;

void service_observation_capture(void)
{
    ++observation_calls;
}

static fast_event_e events[EVENT_CAPACITY];
static size_t event_count;
static const foc_t *last_motor;
static encoder_state_t *last_pos_box;
static bool encoder_valid = true;
static bool current_valid = true;
static bool position_valid = true;
static bool feedback_unchanged_during_sampling = true;
static foc_fb_state_t feedback_before;

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc);

static void log_event(fast_event_e event)
{
    events[event_count] = event;
    event_count++;
}

bool encoder_sample(encoder_state_t *pos_box)
{
    last_pos_box = pos_box;
    log_event(EVENT_ENCODER_SAMPLE);
    return encoder_valid;
}

bool position_update(foc_t *motor, position_sample_t *sample)
{
    last_motor = motor;
    log_event(EVENT_POSITION_UPDATE);
    if (position_valid)
    {
        sample->theta_e = 0.25f;
        sample->pos_r = 3.5f;
        sample->pos_m = 1.75f;
    }
    return position_valid;
}

bool foc_adc_sample(foc_t *motor, board_sample_t *sample)
{
    last_motor = motor;
    log_event(EVENT_ADC_SAMPLE);
    feedback_unchanged_during_sampling = feedback_unchanged_during_sampling
        && (memcmp(&motor->fb, &feedback_before, sizeof(motor->fb)) == 0);
    if (current_valid)
    {
        sample->ia = 1.0f;
        sample->ib = -0.25f;
        sample->ic = -0.75f;
        sample->vbus = 24.0f;
    }
    return current_valid;
}

void ctrl_fb_update(foc_t *motor, float vbus)
{
    motor->fb.vbus = vbus;
    last_motor = motor;
    log_event(EVENT_FEEDBACK_UPDATE);
}

void fake_drive_result(foc_t *motor)
{
    last_motor = motor;
    log_event(EVENT_DRIVE_STEP);
}

void obs_step(const foc_t *motor)
{
    last_motor = motor;
    log_event(EVENT_OBSERVER_STEP);
}

void debug_snapshot_publish(const foc_t *motor)
{
    last_motor = motor;
    log_event(EVENT_SNAPSHOT_PUBLISH);
}

void cycle_record_publish(const foc_t *motor)
{
    last_motor = motor;
    log_event(EVENT_RECORD_PUBLISH);
}

static bool expect_true(bool condition, const char *message)
{
    if (condition)
    {
        return true;
    }

    fprintf(stderr, "%s\n", message);
    return false;
}

static bool expect_cycle_events(size_t first_event)
{
    static const fast_event_e expected[EVENT_COUNT_PER_CYCLE] = {
        EVENT_ENCODER_SAMPLE,
        EVENT_POSITION_UPDATE,
        EVENT_ADC_SAMPLE,
        EVENT_FEEDBACK_UPDATE,
        EVENT_DRIVE_STEP,
        EVENT_OBSERVER_STEP,
        EVENT_SNAPSHOT_PUBLISH,
        EVENT_RECORD_PUBLISH
    };
    size_t index;

    for (index = 0U; index < EVENT_COUNT_PER_CYCLE; index++)
    {
        if (events[first_event + index] != expected[index])
        {
            fprintf(stderr, "快速周期第 %zu 个调用顺序不正确。\n", index);
            return false;
        }
    }

    return true;
}

static bool test_not_ready_does_nothing(foc_t *motor)
{
    motor->fast_seq = 7U;
    event_count = 0U;

    fast_loop_step(motor);

    return expect_true(event_count == 0U,
                       "快速控制未放行时不应调用任何模块。") &&
           expect_true(motor->fast_seq == 7U,
                       "快速控制未放行时不应增加周期编号。");
}

static bool test_explicit_motor_cycle(foc_t *motor)
{
    encoder_valid = true;
    current_valid = true;
    fast_loop_enable();
    event_count = 0U;
    last_motor = NULL;
    last_pos_box = NULL;

    feedback_before = motor->fb;
    fast_loop_step(motor);

    if (!expect_true(event_count == EVENT_COUNT_PER_CYCLE,
                      "一次快速周期应依次调用八个步骤。") ||
        !expect_cycle_events(0U) ||
        !expect_true(motor->fast_seq == 8U,
                     "放行后的快速周期应增加一次周期编号。") ||
        !expect_true(last_motor == motor,
                     "各步骤必须使用调用者传入的同一个电机对象。") ||
        !expect_true(last_pos_box == &motor->enc,
                     "编码器采样必须写入当前电机对象的位置成员。"))
    {
        return false;
    }

    if (!expect_true(feedback_unchanged_during_sampling,
                     "采样结果不能提前装入控制反馈。") ||
        !expect_true(motor->fb.ia == 1.0f && motor->fb.ib == -0.25f &&
                     motor->fb.ic == -0.75f && motor->fb.vbus == 24.0f &&
                     motor->fb.theta_e == 0.25f && motor->fb.pos_r == 3.5f &&
                     motor->fb.pos_m == 1.75f,
                     "新采样数值应经 input 完整传入控制周期。"))
    {
        return false;
    }

    feedback_before = motor->fb;
    fast_loop_step(motor);

    return expect_true(event_count == (2U * EVENT_COUNT_PER_CYCLE),
                       "两次快速周期应各执行一遍完整步骤。") &&
           expect_cycle_events(EVENT_COUNT_PER_CYCLE) &&
           expect_true(motor->fast_seq == 9U,
                       "第二次快速周期应继续增加周期编号。");
}

static bool test_sample_validity_is_forwarded(foc_t *motor)
{
    encoder_valid = false;
    current_valid = false;
    event_count = 0U;

    feedback_before = motor->fb;
    fast_loop_step(motor);

    return expect_true(event_count == (EVENT_COUNT_PER_CYCLE - 1U),
                       "编码器采样失败后不应继续换算位置。") &&
           expect_true(!motor->fb.i_valid,
                       "ADC 采样失败必须传到控制周期。") &&
           expect_true(motor->fb.vbus_valid,
                       "现有母线 ADC 直读结果应保持有效。") &&
           expect_true(!motor->fb.pos_valid,
                       "编码器采样失败必须传到控制周期。") &&
           expect_true(motor->fb.ia == feedback_before.ia &&
                       motor->fb.ib == feedback_before.ib &&
                       motor->fb.ic == feedback_before.ic &&
                       motor->fb.vbus == feedback_before.vbus &&
                       motor->fb.theta_e == feedback_before.theta_e &&
                       motor->fb.pos_r == feedback_before.pos_r &&
                       motor->fb.pos_m == feedback_before.pos_m,
                       "无效采样应保留上拍数值，不能用未初始化数据覆盖。");
}

static bool test_position_conversion_failure(foc_t *motor)
{
    encoder_valid = true;
    position_valid = false;
    current_valid = true;
    event_count = 0U;
    motor->fb.theta_e = 0.5f;
    motor->fb.pos_r = 7.0f;
    motor->fb.pos_m = 3.5f;
    feedback_before = motor->fb;
    fast_loop_step(motor);
    position_valid = true;

    return expect_cycle_events(0U) &&
           expect_true(!motor->fb.pos_valid && motor->fb.i_valid &&
                       motor->fb.theta_e == 0.5f && motor->fb.pos_r == 7.0f &&
                       motor->fb.pos_m == 3.5f && motor->fb.vbus == 24.0f,
                       "读到编码器但换算失败时，只保留旧位置，电流仍正常更新。");
}

static bool test_hal_callback_uses_firmware_motor(void)
{
    ADC_HandleTypeDef adc = {ADC1};
    ADC_TypeDef other_adc = {ADC_FLAG_JEOS};
    ADC_HandleTypeDef other_handle = {&other_adc};
    encoder_valid = true;
    current_valid = true;
    g_foc.fast_seq = 20U;
    event_count = 0U;
    last_motor = NULL;
    last_pos_box = NULL;

    HAL_ADCEx_InjectedConvCpltCallback(NULL);
    HAL_ADCEx_InjectedConvCpltCallback(&other_handle);
    axdr_test_adc1.flags = 0U;
    HAL_ADCEx_InjectedConvCpltCallback(&adc);
    if (!expect_true(event_count == 0U && g_foc.fast_seq == 20U && observation_calls == 0U,
                     "空句柄、其他 ADC 和未完成序列不能执行控制周期。"))
    {
        return false;
    }
    axdr_test_adc1.flags = ADC_FLAG_JEOS;
    feedback_before = g_foc.fb;
    HAL_ADCEx_InjectedConvCpltCallback(&adc);

    return expect_true(event_count == EVENT_COUNT_PER_CYCLE && observation_calls == 1U,
                       "ADC 回调应触发一次完整快速周期。") &&
           expect_cycle_events(0U) &&
           expect_true(g_foc.fast_seq == 21U,
                       "ADC 回调应更新固件全局电机对象的周期编号。") &&
           expect_true(last_motor == &g_foc,
                       "只有硬件回调边界应选择固件全局电机对象。") &&
           expect_true(last_pos_box == &g_foc.enc,
                       "硬件回调应采样固件全局电机的位置对象。");
}

int main(void)
{
    foc_t motor = {0};

    if (!test_not_ready_does_nothing(&motor))
    {
        return 1;
    }

    if (!test_explicit_motor_cycle(&motor))
    {
        return 2;
    }

    if (!test_sample_validity_is_forwarded(&motor))
    {
        return 3;
    }

    if (!test_hal_callback_uses_firmware_motor())
    {
        return 4;
    }

    if (!test_position_conversion_failure(&motor) ||
        !expect_true(feedback_unchanged_during_sampling,
                     "所有分支的控制反馈都应在采样完成后装入。"))
    {
        return 6;
    }

    /* 周期计数自然回绕、超期及间隔异常分别记账，不把间隔异常称为漏采。 */
    {
        target_irq_profile_t profile;
        if (!expect_true(target_irq_profile_begin(0xfffff000u, 0u) == 0u,
                         "首拍没有已知间隔异常。")) return 5;
        target_irq_profile_end(0xfffff000u, 0xfffff100u);
        if (!expect_true(target_irq_profile_begin(0x00000f40u, 1u) == 1u,
                         "ADC 溢出必须在本拍采集前报告。")) return 5;
        target_irq_profile_end(0x00000f40u, 0x00002e80u);
        if (!expect_true(target_irq_profile_begin(0x00004dc0u, 0u) == 1u,
                         "入口间隔异常必须在本拍采集前报告。")) return 5;
        target_irq_profile_end(0x00004dc0u, 0x00004ec0u);
        target_irq_profile_read(&profile);
        if (!expect_true(profile.count == 3u && profile.last_cycles == 256u &&
                         profile.max_cycles == 8000u && profile.overruns == 1u &&
                         profile.gap_count == 1u && profile.max_interval_cycles == 16000u &&
                         profile.adc_overruns == 1u && profile.core_clock_hz == 160000000u,
                         "中断计时应正确处理回绕、50微秒超期和75微秒以上间隔。"))
        {
            return 5;
        }
    }

    return 0;
}

void drive_check_protection(foc_t *foc) { (void)foc; }
void drive_update_state(foc_t *foc)
{ (void)foc; }
void drive_handle_state(foc_t *foc, drive_cycle_t *cycle)
{ (void)foc; (void)cycle; }
void drive_output(foc_t *foc, drive_cycle_t *cycle)
{ (void)cycle; fake_drive_result(foc); }

void foc_update_current(foc_t *foc, float theta, foc_frame_t *frame)
{ (void)foc; *frame = (foc_frame_t){.theta = theta}; }

bool foc_execute(foc_t *foc, const foc_plan_t *cmd, const foc_frame_t *frame)
{ (void)foc; (void)cmd; (void)frame; return true; }
