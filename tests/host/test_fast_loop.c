/**
 * @file test_fast_loop.c
 * @brief 使用 Fake 模块验证生产快速周期编排和对象传递。
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

#include "common.h"
#include "fast_loop.h"
#include "main.h"

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

static fast_event_e events[EVENT_CAPACITY];
static size_t event_count;
static const foc_t *last_motor;
static encoder_state_t *last_pos_box;
static bool encoder_valid = true;
static bool current_valid = true;

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

bool position_update(foc_t *motor)
{
    last_motor = motor;
    log_event(EVENT_POSITION_UPDATE);
    return true;
}

bool foc_adc_sample(foc_t *motor)
{
    last_motor = motor;
    log_event(EVENT_ADC_SAMPLE);
    return current_valid;
}

void ctrl_fb_update(foc_t *motor, float vbus)
{
    (void)vbus;
    last_motor = motor;
    log_event(EVENT_FEEDBACK_UPDATE);
}

void drive_fast_step(foc_t *motor)
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

    fast_loop_step(motor);

    return expect_true(event_count == (EVENT_COUNT_PER_CYCLE - 1U),
                       "编码器采样失败后不应继续换算位置。") &&
           expect_true(!motor->fb.i_valid,
                       "ADC 采样失败必须传到控制周期。") &&
           expect_true(motor->fb.vbus_valid,
                       "现有母线 ADC 直读结果应保持有效。") &&
           expect_true(!motor->fb.pos_valid,
                       "编码器采样失败必须传到控制周期。");
}

static bool test_hal_callback_uses_firmware_motor(void)
{
    encoder_valid = true;
    current_valid = true;
    g_foc.fast_seq = 20U;
    event_count = 0U;
    last_motor = NULL;
    last_pos_box = NULL;

    HAL_ADCEx_InjectedConvCpltCallback(NULL);

    return expect_true(event_count == EVENT_COUNT_PER_CYCLE,
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

    return 0;
}
