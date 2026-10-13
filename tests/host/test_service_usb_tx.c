#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "service_usb_tx.h"

static const uint8_t *inflight;
static uint16_t inflight_length;
static uint32_t submit_calls;
static int next_status;
static uint32_t locked;
static uint32_t lock_calls;

static uint32_t lock_usb(void)
{
    const uint32_t old = locked;
    locked = 1u;
    ++lock_calls;
    return old;
}

static void unlock_usb(uint32_t state)
{
    locked = state;
}

static int submit(const uint8_t *data, uint16_t length)
{
    assert(locked != 0u);
    ++submit_calls;
    if (next_status == 0)
    {
        assert(inflight == NULL);
        inflight = data;
        inflight_length = length;
    }
    return next_status;
}

static service_usb_tx_stats_t snapshot(void)
{
    service_usb_tx_stats_t out;
    service_usb_tx_get_stats(&out);
    assert(locked == 0u);
    return out;
}

static void complete(void)
{
    const uint8_t *data = inflight;
    const uint16_t length = inflight_length;
    assert(data != NULL);
    inflight = NULL;
    inflight_length = 0u;
    service_usb_tx_complete(data, length);
    assert(locked == 0u);
}

static void reset(void)
{
    inflight = NULL;
    inflight_length = 0u;
    submit_calls = 0u;
    next_status = 0;
    locked = 0u;
    lock_calls = 0u;
    service_usb_tx_init(submit, lock_usb, unlock_usb);
    service_usb_tx_set_link(1u);
}

static void test_delayed_multipacket_and_priority(void)
{
    uint8_t frame[212];
    uint8_t expected[212];
    uint8_t received[212];
    uint8_t command[160];
    reset();
    for (uint16_t i = 0u; i < sizeof(frame); ++i) { frame[i] = (uint8_t)i; }
    memcpy(expected, frame, sizeof(frame));
    memset(command, 0xCC, sizeof(command));
    assert(service_usb_tx_enqueue_data(frame, sizeof(frame)) == 0);
    memset(frame, 0xA5, sizeof(frame));
    service_usb_tx_poll();
    assert(inflight_length == sizeof(expected));
    memcpy(received, inflight, 64u);

    /* 第一包之后源对象和后续队列都改变，余下各包仍必须读取同一份原始帧。 */
    assert(service_usb_tx_enqueue_data(frame, sizeof(frame)) == 0);
    assert(service_usb_tx_enqueue_command(command, sizeof(command)) == 0);
    memset(command, 0x44, sizeof(command));
    assert(service_usb_tx_enqueue_command(command, sizeof(command)) == 0);
    for (uint16_t offset = 64u; offset < sizeof(expected); offset += 64u)
    {
        uint16_t length = (uint16_t)(sizeof(expected) - offset);
        if (length > 64u) { length = 64u; }
        for (unsigned i = 0u; i < 200u; ++i) { service_usb_tx_poll(); }
        assert(submit_calls == 1u);
        memcpy(&received[offset], &inflight[offset], length);
    }
    assert(memcmp(received, expected, sizeof(expected)) == 0);
    assert(service_usb_tx_enqueue_data(frame, sizeof(frame)) == 1);
    assert(snapshot().data_completed == 0u);
    complete();

    service_usb_tx_poll();
    assert(inflight_length == 160u && inflight[0] == 0xCCu);
    complete();
    service_usb_tx_poll();
    assert(inflight[0] == 0x44u);
    complete();
    service_usb_tx_poll();
    assert(inflight_length == sizeof(frame) && inflight[0] == 0xA5u);
    complete();
    assert(snapshot().command_completed == 2u);
    assert(snapshot().data_completed == 2u);
    assert(snapshot().data_full == 1u);
}

static void test_busy_errors_and_full_queue(void)
{
    uint8_t command[160];
    service_usb_tx_stats_t stats;
    reset();
    for (uint8_t i = 0u; i < SERVICE_USB_TX_COMMAND_CAPACITY; ++i)
    {
        memset(command, i, sizeof(command));
        assert(service_usb_tx_enqueue_command(command, sizeof(command)) == 0);
    }
    assert(service_usb_tx_enqueue_command(command, sizeof(command)) == 1);
    assert(service_usb_tx_enqueue_command(NULL, 2u) == 2);
    assert(service_usb_tx_enqueue_command(command, 0u) == 2);
    assert(service_usb_tx_enqueue_command(command, 161u) == 2);
    next_status = 1;
    service_usb_tx_poll();
    assert(submit_calls == 1u && inflight == NULL);
    service_usb_tx_poll();
    assert(submit_calls == 2u && snapshot().command_pending == 4u);
    assert(snapshot().command_completed == 0u);
    next_status = 2;
    service_usb_tx_poll();
    assert(snapshot().command_pending == 3u);
    next_status = 0;
    for (uint8_t i = 1u; i < SERVICE_USB_TX_COMMAND_CAPACITY; ++i)
    {
        service_usb_tx_poll();
        assert(inflight[0] == i);
        complete();
    }
    stats = snapshot();
    assert(stats.command_accepted == 4u && stats.command_completed == 3u);
    assert(stats.command_dropped == 1u && stats.submit_errors == 1u);
    assert(stats.command_full == 1u && stats.busy_count == 2u);
    assert(stats.invalid_count == 3u && stats.command_peak == 4u);
}

static void test_disconnect_reset_and_zlp(void)
{
    uint8_t frame[512];
    uint8_t command[160];
    const uint8_t *cancelled;
    reset();
    memset(frame, 0xDE, sizeof(frame));
    memset(command, 0xCA, sizeof(command));
    assert(service_usb_tx_enqueue_data(frame, sizeof(frame)) == 0);
    service_usb_tx_poll();
    /* 整 64 字节倍数发送仍持有帧，不能在最后数据包之后、ZLP 之前释放。 */
    for (unsigned packet = 0u; packet < 8u; ++packet)
    {
        assert(memcmp(&inflight[packet * 64u], &frame[packet * 64u], 64u) == 0);
        service_usb_tx_poll();
    }
    assert(snapshot().active != 0u && snapshot().data_completed == 0u);
    service_usb_tx_complete(inflight, 64u);
    service_usb_tx_complete(command, sizeof(command));
    assert(snapshot().unexpected_completions == 2u);
    assert(snapshot().active != 0u);
    assert(service_usb_tx_enqueue_command(command, sizeof(command)) == 0);
    assert(service_usb_tx_enqueue_data(frame, sizeof(frame)) == 0);
    cancelled = inflight;
    /* 模拟底层先停止传输、关闭端点，之后通知应用断开。 */
    inflight = NULL;
    service_usb_tx_set_link(0u);
    assert(snapshot().command_dropped == 1u && snapshot().data_dropped == 2u);
    service_usb_tx_complete(cancelled, sizeof(frame));
    assert(snapshot().unexpected_completions == 3u);
    assert(service_usb_tx_enqueue_command(command, sizeof(command)) == 2);
    assert(service_usb_tx_enqueue_data(frame, sizeof(frame)) == 2);
    service_usb_tx_poll();
    assert(submit_calls == 1u);
    service_usb_tx_set_link(1u);
    service_usb_tx_poll();
    assert(submit_calls == 1u && snapshot().command_pending == 0u);
    assert(service_usb_tx_enqueue_data(frame, sizeof(frame)) == 0);
    service_usb_tx_poll();
    complete();
    assert(snapshot().data_completed == 1u && snapshot().offline_count == 2u);
}

static void test_wrap_and_sustained_priority(void)
{
    uint8_t frame[128];
    uint8_t command[32];
    reset();
    memset(frame, 0x42, sizeof(frame));
    for (uint16_t i = 0u; i < 1000u; ++i)
    {
        memset(command, (uint8_t)i, sizeof(command));
        assert(service_usb_tx_enqueue_data(frame, sizeof(frame)) == 0);
        assert(service_usb_tx_enqueue_command(command, sizeof(command)) == 0);
        service_usb_tx_poll();
        assert(inflight_length == sizeof(command));
        assert(memcmp(inflight, command, sizeof(command)) == 0);
        complete();
        service_usb_tx_poll();
        assert(inflight_length == sizeof(frame));
        complete();
    }
    assert(snapshot().command_completed == 1000u && snapshot().data_completed == 1000u);
    assert(snapshot().command_pending == 0u && snapshot().data_pending == 0u);
    assert(lock_calls > 0u);
}

int main(void)
{
    test_delayed_multipacket_and_priority();
    test_busy_errors_and_full_queue();
    test_disconnect_reset_and_zlp();
    test_wrap_and_sustained_priority();
    puts("test_service_usb_tx: all pass");
    return 0;
}
