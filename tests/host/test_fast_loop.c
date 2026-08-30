/**
 * @file test_fast_loop.c
 * @brief 使用 Fake 模块验证生产快速周期编排和对象传递。
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

#include "axdr_app.h"

typedef enum
{
    EVENT_ENCODER_SAMPLE = 0,
    EVENT_POSITION_UPDATE,
    EVENT_ADC_SAMPLE,
    EVENT_FEEDBACK_UPDATE,
    EVENT_DRIVE_STEP,
    EVENT_SNAPSHOT_PUBLISH
} fast_event_e;

#define EVENT_COUNT_PER_CYCLE (6U)
#define EVENT_CAPACITY        (18U)

pmsm_t pm;

static fast_event_e events[EVENT_CAPACITY];
static size_t event_count;
static const pmsm_t *last_motor;
static pos_box_t *last_pos_box;

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc);

static void log_event(fast_event_e event)
{
    events[event_count] = event;
    event_count++;
}

void encoder_sample(pos_box_t *pos_box)
{
    last_pos_box = pos_box;
    log_event(EVENT_ENCODER_SAMPLE);
}

void position_update(pmsm_t *motor)
{
    last_motor = motor;
    log_event(EVENT_POSITION_UPDATE);
}

void foc_adc_sample(pmsm_t *motor)
{
    last_motor = motor;
    log_event(EVENT_ADC_SAMPLE);
}

void foc_feedback_update(pmsm_t *motor)
{
    last_motor = motor;
    log_event(EVENT_FEEDBACK_UPDATE);
}

void drive_fast_step(pmsm_t *motor)
{
    last_motor = motor;
    log_event(EVENT_DRIVE_STEP);
}

void debug_snapshot_publish(const pmsm_t *motor)
{
    last_motor = motor;
    log_event(EVENT_SNAPSHOT_PUBLISH);
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
        EVENT_SNAPSHOT_PUBLISH
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

static bool test_not_ready_does_nothing(pmsm_t *motor)
{
    motor->fast_seq = 7U;
    event_count = 0U;

    axdr_app_fast_step(motor);

    return expect_true(event_count == 0U,
                       "快速控制未放行时不应调用任何模块。") &&
           expect_true(motor->fast_seq == 7U,
                       "快速控制未放行时不应增加周期编号。");
}

static bool test_explicit_motor_cycle(pmsm_t *motor)
{
    axdr_app_start_fast_control();
    event_count = 0U;
    last_motor = NULL;
    last_pos_box = NULL;

    axdr_app_fast_step(motor);

    if (!expect_true(event_count == EVENT_COUNT_PER_CYCLE,
                     "一次快速周期应依次调用六个步骤。") ||
        !expect_cycle_events(0U) ||
        !expect_true(motor->fast_seq == 8U,
                     "放行后的快速周期应增加一次周期编号。") ||
        !expect_true(last_motor == motor,
                     "各步骤必须使用调用者传入的同一个电机对象。") ||
        !expect_true(last_pos_box == &motor->pos_box,
                     "编码器采样必须写入当前电机对象的位置成员。"))
    {
        return false;
    }

    axdr_app_fast_step(motor);

    return expect_true(event_count == (2U * EVENT_COUNT_PER_CYCLE),
                       "两次快速周期应各执行一遍完整步骤。") &&
           expect_cycle_events(EVENT_COUNT_PER_CYCLE) &&
           expect_true(motor->fast_seq == 9U,
                       "第二次快速周期应继续增加周期编号。");
}

static bool test_hal_callback_uses_firmware_motor(void)
{
    pm.fast_seq = 20U;
    event_count = 0U;
    last_motor = NULL;
    last_pos_box = NULL;

    HAL_ADCEx_InjectedConvCpltCallback(NULL);

    return expect_true(event_count == EVENT_COUNT_PER_CYCLE,
                       "ADC 回调应触发一次完整快速周期。") &&
           expect_cycle_events(0U) &&
           expect_true(pm.fast_seq == 21U,
                       "ADC 回调应更新固件全局电机对象的周期编号。") &&
           expect_true(last_motor == &pm,
                       "只有硬件回调边界应选择固件全局电机对象。") &&
           expect_true(last_pos_box == &pm.pos_box,
                       "硬件回调应采样固件全局电机的位置对象。");
}

int main(void)
{
    pmsm_t motor = {0};

    if (!test_not_ready_does_nothing(&motor))
    {
        return 1;
    }

    if (!test_explicit_motor_cycle(&motor))
    {
        return 2;
    }

    if (!test_hal_callback_uses_firmware_motor())
    {
        return 3;
    }

    return 0;
}
