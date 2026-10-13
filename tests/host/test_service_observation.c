/** @file test_service_observation.c
 * @brief 被动观测的节拍、拥塞、重配、冻结读回和取消边界回归。
 * 可选参数为既有目录，输出由真实固件 C 实现生成的协议交叉验证样本。
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "axdr_command_core.h"
#include "axdr_observation_contract.h"
#include "common.h"
#include "service_observation.h"

foc_t g_foc;
static int failures;
static unsigned checks;
static uint8_t response[AXDR_COMMAND_MAX_RESPONSE_SIZE];
static uint16_t response_length;
static uint8_t ack;
static int tx_result;
static uint32_t tx_calls;
static axdr_scope_stream_frame_v1_t tx_frame;
static foc_t before_sample;

static void expect(const char *name, bool condition)
{
    ++checks;
    if (!condition)
    {
        ++failures;
        printf("FAIL %s\n", name);
    }
}

static int transmit_copy(const uint8_t *data, uint16_t length)
{
    ++tx_calls;
    expect("frame length", length == sizeof(tx_frame));
    memcpy(&tx_frame, data, sizeof(tx_frame));
    return tx_result;
}

static uint8_t request(uint16_t opcode, const void *payload, uint16_t length)
{
    return service_observation_route(NULL, opcode, payload, length, response,
                                     sizeof(response), &response_length, &ack);
}

static void step(uint32_t tick, float sample)
{
    g_foc.fast_seq = tick;
    g_foc.pwm_commit.seq = tick;
    g_foc.fb.ia = sample;
    g_foc.fb.ib = -sample - 0.25f;
    g_foc.fb.ic = 0.25f;
    g_foc.fb.iq = sample * 2.0f;
    before_sample = g_foc;
    service_observation_capture();
    expect("observation never writes FOC", memcmp(&before_sample, &g_foc, sizeof(g_foc)) == 0);
}

static void reset(void)
{
    memset(&g_foc, 0, sizeof(g_foc));
    g_foc.fb.i_valid = true;
    g_foc.fb.vbus_valid = true;
    g_foc.fb.pos_valid = true;
    g_foc.state = DRIVE_STATE_RUN;
    g_foc.pwm_active = true;
    g_foc.pwm_commit.valid = true;
    service_observation_init();
    service_observation_bind_tx(transmit_copy);
    tx_result = 0;
    tx_calls = 0u;
    memset(&tx_frame, 0, sizeof(tx_frame));
}

static axdr_scope_stream_config_request_t stream_request(uint16_t rate)
{
    axdr_scope_stream_config_request_t config;
    memset(&config, 0, sizeof(config));
    config.enabled = 1u;
    config.channel_count = 4u;
    config.sample_rate_hz = rate;
    config.channel_ids[0] = 1u;
    config.channel_ids[1] = 2u;
    config.channel_ids[2] = 3u;
    config.channel_ids[3] = 5u;
    return config;
}

static axdr_scope_capture_arm_request_t capture_request(void)
{
    axdr_scope_capture_arm_request_t config;
    memset(&config, 0, sizeof(config));
    config.channel_count = 4u;
    config.trigger_type = AXDR_SCOPE_TRIGGER_MANUAL;
    config.sample_rate_hz = 20000u;
    config.channel_ids[0] = 1u;
    config.channel_ids[1] = 2u;
    config.channel_ids[2] = 3u;
    config.channel_ids[3] = 32u;
    return config;
}

static axdr_observation_diagnostics_t diagnostics(void)
{
    axdr_observation_diagnostics_t out;
    expect("diagnostics accepted", request(AXDR_OPCODE_OBSERVATION_DIAGNOSTICS, NULL, 0u) == 0u);
    memcpy(&out, response, sizeof(out));
    expect("diagnostics size", response_length == 80u && out.payload_size == 80u);
    return out;
}

static axdr_observation_capture_state_t state(void)
{
    axdr_observation_capture_state_t out;
    expect("state accepted", request(AXDR_OPCODE_OBSERVATION_STATE, NULL, 0u) == 0u);
    memcpy(&out, response, sizeof(out));
    expect("state size", response_length == 52u && out.payload_size == 52u);
    return out;
}

static void test_stream(void)
{
    axdr_scope_stream_config_request_t config = stream_request(1000u);
    axdr_scope_stream_config_payload_t accepted;
    axdr_observation_diagnostics_t diag;
    axdr_scope_stream_frame_v1_t saved;
    uint32_t tick;
    uint32_t packet;
    reset();
    for (tick = 1u; tick <= 100u; ++tick) step(tick, (float)tick);
    service_observation_poll();
    expect("default off", tx_calls == 0u);
    diag = diagnostics();
    expect("unavailable timing not forged zero", diag.irq_max_cycles == UINT32_MAX && diag.adc_overruns == UINT32_MAX);
    expect("enable stream", request(AXDR_OPCODE_OBSERVATION_CONFIGURE, &config, sizeof(config)) == 0u);
    memcpy(&accepted, response, sizeof(accepted));
    expect("config response", accepted.stream_id == 1u && accepted.sample_rate_hz == 1000u);
    expect("mailbox protects channels", request(AXDR_OPCODE_OBSERVATION_CONFIGURE, &config, sizeof(config)) == AXDR_REASON_BUSY);
    for (tick = 101u; tick <= 181u; ++tick) step(tick, (float)tick);
    service_observation_capture();
    service_observation_poll();
    expect("five samples in one packet", tx_calls == 1u && tx_frame.sample_count == 5u);
    expect("20 control ticks per sample", tx_frame.first_control_tick == 101u && tx_frame.sample_period_ticks == 20u);
    expect("sample major layout", tx_frame.values[0] == 101.0f && tx_frame.values[4] == 121.0f && tx_frame.values[19] == 362.0f);
    expect("unused floats zero", tx_frame.values[20] == 0.0f && tx_frame.values[39] == 0.0f);
    expect("stream CRC", tx_frame.crc32 == axdr_command_crc32((const uint8_t *)&tx_frame, 208u));
    expect("duplicate tick ignored", diagnostics().produced_samples == 5u);

    /* BUSY 保留最老包；ISR 环满后丢新包，不能撕裂正在主循环读取的槽。 */
    for (tick = 182u; tick <= 281u; ++tick) step(tick, (float)tick);
    tx_result = 1;
    service_observation_poll();
    saved = tx_frame;
    for (tick = 282u; tick <= 1381u; ++tick) step(tick, (float)tick);
    service_observation_poll();
    expect("BUSY payload preserved", memcmp(saved.values, tx_frame.values, sizeof(saved.values)) == 0 && tx_frame.packet_sequence == 2u);
    diag = diagnostics();
    expect("queue bounded", diag.queued_packets == 8u);
    expect("full queue exact loss", diag.produced_samples == 65u && diag.dropped_samples == 20u);
    tx_result = 0;
    for (packet = 2u; packet <= 9u; ++packet)
    {
        service_observation_poll();
        expect("queued packets preserve order", tx_frame.packet_sequence == packet);
    }
    expect("queue drains", diagnostics().queued_packets == 0u);
    for (tick = 1382u; tick <= 1481u; ++tick) step(tick, (float)tick);
    tx_result = -1;
    service_observation_poll();
    expect("gap visible after backlog", tx_frame.packet_sequence == 14u && (tx_frame.flags & 1u) != 0u);
    expect("TX error drops exact sample count", diagnostics().dropped_samples == 25u);

    /* 重配期间旧包不能混入新会话。 */
    tx_result = 0;
    for (tick = 1482u; tick <= 1581u; ++tick) step(tick, (float)tick);
    config.sample_rate_hz = 500u;
    config.channel_ids[3] = 32u;
    expect("reconfigure", request(AXDR_OPCODE_OBSERVATION_CONFIGURE, &config, sizeof(config)) == 0u);
    packet = tx_calls;
    service_observation_poll();
    expect("old session discarded", tx_calls == packet);
    g_foc.ref.id = -0.5f;
    for (tick = 1582u; tick <= 1742u; ++tick) step(tick, (float)tick);
    service_observation_poll();
    expect("500 Hz new session", tx_frame.stream_id == 2u && tx_frame.sample_period_ticks == 40u && tx_frame.packet_sequence == 1u);
    expect("new channel applied atomically", tx_frame.channel_ids[3] == 32u && tx_frame.values[3] == -0.5f);
    expect("cancel accepted", request(AXDR_OPCODE_OBSERVATION_CANCEL, NULL, 0u) == 0u);
    packet = tx_calls;
    for (tick = 1743u; tick <= 1900u; ++tick) step(tick, (float)tick);
    service_observation_poll();
    expect("cancel stops stream", tx_calls == packet);

    /* 超过一圈队列索引，持续收发不覆盖且控制 tick 自然回绕。 */
    reset();
    config = stream_request(1000u);
    expect("wrap stream config", request(AXDR_OPCODE_OBSERVATION_CONFIGURE, &config, sizeof(config)) == 0u);
    for (tick = 0u; tick <= 2000u; ++tick)
    {
        step(UINT32_MAX - 39u + tick, (float)tick);
        service_observation_poll();
    }
    expect("tick wrap and repeated ring wrap", tx_calls == 20u && tx_frame.packet_sequence == 20u && diagnostics().dropped_samples == 0u);
    expect("tick wrapped timestamp", tx_frame.first_control_tick == 1860u);

    reset();
    expect("invalid feedback config", request(AXDR_OPCODE_OBSERVATION_CONFIGURE, &config, sizeof(config)) == 0u);
    g_foc.fb.i_valid = false;
    for (tick = 1u; tick <= 81u; ++tick) step(tick, 0.5f);
    service_observation_poll();
    expect("invalid feedback is NaN", isnan(tx_frame.values[0]) && isnan(tx_frame.values[3]) && (tx_frame.flags & 2u));

    reset();
    config.channel_ids[0] = 4u;
    config.channel_ids[1] = 5u;
    config.channel_ids[2] = 6u;
    config.channel_ids[3] = 14u;
    expect("stop-cache config", request(AXDR_OPCODE_OBSERVATION_CONFIGURE, &config, sizeof(config)) == 0u);
    g_foc.state = DRIVE_STATE_STOP;
    g_foc.pwm_active = false;
    g_foc.fb.id = 99.0f;
    g_foc.out.vd = 99.0f;
    g_foc.out.duty_a = 0.75f;
    for (tick = 1u; tick <= 81u; ++tick) step(tick, 99.0f);
    service_observation_poll();
    expect("stop caches not fresh samples", isnan(tx_frame.values[0]) && isnan(tx_frame.values[1]) && isnan(tx_frame.values[2]) && isnan(tx_frame.values[3]));
    g_foc.state = DRIVE_STATE_STARTING;
    g_foc.pwm_active = true;
    g_foc.pwm_commit.duty_a = 0.5f;
    for (tick = 82u; tick <= 181u; ++tick) step(tick, 99.0f);
    service_observation_poll();
    expect("neutral-start has no FOC cache", isnan(tx_frame.values[0]) && isnan(tx_frame.values[1]) && isnan(tx_frame.values[2]));
    expect("duty reports actual neutral commit", tx_frame.values[3] == 0.5f);
    g_foc.align_ticks = 1u;
    for (tick = 182u; tick <= 281u; ++tick) step(tick, 0.5f);
    service_observation_poll();
    expect("computed alignment feedback available", tx_frame.values[0] == 99.0f && tx_frame.values[1] == 1.0f && tx_frame.values[2] == 99.0f);
    for (tick = 282u; tick <= 381u; ++tick)
    {
        g_foc.fast_seq = tick;
        /* 故意不更新 commit.seq，模拟当前拍没有执行新的控制提交。 */
        service_observation_capture();
    }
    service_observation_poll();
    expect("stale output sequence rejected", isnan(tx_frame.values[0]) && isnan(tx_frame.values[3]));
}

static void test_capture(void)
{
    axdr_scope_capture_arm_request_t config = capture_request();
    axdr_scope_stream_config_request_t stream = stream_request(1000u);
    axdr_scope_capture_read_request_t read;
    axdr_scope_capture_chunk_payload_t chunk;
    axdr_observation_capture_state_t snapshot;
    uint8_t captured[16384];
    uint32_t index;
    float value;
    reset();
    g_foc.ref.id = -0.75f;
    expect("capture accepted", request(AXDR_OPCODE_OBSERVATION_CAPTURE, &config, sizeof(config)) == 0u);
    memcpy(&snapshot, response, sizeof(snapshot));
    expect("capture planned dimensions", snapshot.state == AXDR_SCOPE_CAPTURE_ARMED && snapshot.total_samples == 1024u && snapshot.total_bytes == 16384u);
    read.capture_id = snapshot.capture_id;
    read.byte_offset = 0u;
    expect("unpublished capture not readable", request(AXDR_OPCODE_OBSERVATION_READ, &read, sizeof(read)) == AXDR_REASON_BUSY);
    for (index = 0u; index < 1024u; ++index)
    {
        step(UINT32_MAX - 39u + index, (float)index + 0.125f);
        if (index == 0u)
        {
            snapshot = state();
            expect("first sample trigger", snapshot.state == AXDR_SCOPE_CAPTURE_TRIGGERED && snapshot.captured_samples == 1u);
            expect("stream and capture exclusive", request(AXDR_OPCODE_OBSERVATION_CONFIGURE, &stream, sizeof(stream)) == AXDR_REASON_BUSY);
        }
    }
    snapshot = state();
    expect("complete contiguous capture", snapshot.state == AXDR_SCOPE_CAPTURE_COMPLETE && snapshot.captured_samples == 1024u && snapshot.dropped_samples == 0u);
    expect("capture timestamp wraps", snapshot.first_control_tick == UINT32_MAX - 39u);
    for (index = 1024u; index < 1124u; ++index) step(UINT32_MAX - 39u + index, -999.0f);
    for (index = 0u; index < sizeof(captured); index += chunk.payload_length)
    {
        read.byte_offset = index;
        expect("frozen chunk accepted", request(AXDR_OPCODE_OBSERVATION_READ, &read, sizeof(read)) == 0u);
        memcpy(&chunk, response, sizeof(chunk));
        expect("chunk exact metadata", chunk.byte_offset == index && chunk.total_bytes == sizeof(captured) && chunk.capture_id == read.capture_id);
        expect("chunk bounded", chunk.payload_length > 0u && chunk.payload_length <= 120u);
        if (chunk.payload_length == 0u) break;
        memcpy(captured + index, chunk.data, chunk.payload_length);
    }
    expect("last chunk exact length", chunk.payload_length == 64u && chunk.flags == 1u);
    for (index = 0u; index < 1024u; ++index)
    {
        memcpy(&value, captured + index * 16u, 4u);
        expect("all frozen A samples", value == (float)index + 0.125f);
        memcpy(&value, captured + index * 16u + 4u, 4u);
        expect("all frozen B samples", value == -(float)index - 0.375f);
        memcpy(&value, captured + index * 16u + 12u, 4u);
        expect("all Id references", value == -0.75f);
    }
    read.byte_offset = 16384u;
    expect("read past end rejected", request(AXDR_OPCODE_OBSERVATION_READ, &read, sizeof(read)) == AXDR_REASON_OUT_OF_RANGE);
    read.byte_offset = 0u;
    ++read.capture_id;
    expect("wrong capture id rejected", request(AXDR_OPCODE_OBSERVATION_READ, &read, sizeof(read)) == AXDR_REASON_INVALID_PAYLOAD);
    --read.capture_id;
    expect("cancel frozen capture accepted", request(AXDR_OPCODE_OBSERVATION_CANCEL, NULL, 0u) == 0u);
    step(1085u, 0.0f);
    expect("cancel preserves complete capture", state().state == AXDR_SCOPE_CAPTURE_COMPLETE);
    expect("frozen data still readable", request(AXDR_OPCODE_OBSERVATION_READ, &read, sizeof(read)) == 0u);

    expect("rearm capture", request(AXDR_OPCODE_OBSERVATION_CAPTURE, &config, sizeof(config)) == 0u);
    step(1086u, 1.0f);
    step(1088u, 2.0f);
    snapshot = state();
    expect("capture gap stops acquisition", snapshot.state == AXDR_SCOPE_CAPTURE_ERROR && snapshot.captured_samples == 1u && snapshot.dropped_samples == 1u);
    read.capture_id = snapshot.capture_id;
    expect("incomplete capture cannot impersonate contiguous data", request(AXDR_OPCODE_OBSERVATION_READ, &read, sizeof(read)) == AXDR_REASON_INVALID_STATE);
    expect("cancel failed capture", request(AXDR_OPCODE_OBSERVATION_CANCEL, NULL, 0u) == 0u);
    step(1089u, 0.0f);
    expect("cancel clears partial", state().state == AXDR_SCOPE_CAPTURE_IDLE);

    expect("arm before immediate cancel", request(AXDR_OPCODE_OBSERVATION_CAPTURE, &config, sizeof(config)) == 0u);
    expect("cancel pending arm", request(AXDR_OPCODE_OBSERVATION_CANCEL, NULL, 0u) == 0u);
    step(1090u, 0.0f);
    expect("pending arm did not start", state().state == AXDR_SCOPE_CAPTURE_IDLE);
    expect("arm before disconnect", request(AXDR_OPCODE_OBSERVATION_CAPTURE, &config, sizeof(config)) == 0u);
    step(1091u, 0.0f);
    service_observation_reset_link();
    step(1092u, 0.0f);
    expect("disconnect cancels partial", state().state == AXDR_SCOPE_CAPTURE_IDLE && diagnostics().stream_id == 0u);

    expect("arm before hardware timing gap", request(AXDR_OPCODE_OBSERVATION_CAPTURE, &config, sizeof(config)) == 0u);
    step(1093u, 0.0f);
    service_observation_note_timing_gap();
    step(1094u, 1.0f); /* 软件序号连续，真实中断间隔仍可能异常。 */
    snapshot = state();
    expect("hardware gap invalidates contiguous tick capture", snapshot.state == AXDR_SCOPE_CAPTURE_ERROR && snapshot.captured_samples == 1u && snapshot.dropped_samples == UINT32_MAX);
    expect("gap event count not invented sample loss", diagnostics().capture_gaps == 1u);
    for (index = 1095u; index <= 2120u; ++index) step(index, 1.0f);
    expect("timing error can never become complete", state().state == AXDR_SCOPE_CAPTURE_ERROR);
    expect("rearm before first-sample gap", request(AXDR_OPCODE_OBSERVATION_CAPTURE, &config, sizeof(config)) == 0u);
    service_observation_note_timing_gap();
    step(2121u, 2.0f);
    expect("before capture window gap is fresh origin", state().state == AXDR_SCOPE_CAPTURE_TRIGGERED && state().captured_samples == 1u);
}

static void test_requests(void)
{
    axdr_scope_stream_config_request_t stream = stream_request(1000u);
    axdr_scope_capture_arm_request_t capture = capture_request();
    axdr_scope_signal_directory_request_t dir = {0u, 2u, 0u};
    axdr_scope_signal_directory_payload_t page;
    uint8_t unaligned[sizeof(stream) + 1u];
    uint16_t length;
    uint32_t i;
    reset();
    expect("unknown opcode", request(0x37u, NULL, 0u) == AXDR_REASON_UNKNOWN_OPCODE);
    expect("bad config length", request(AXDR_OPCODE_OBSERVATION_CONFIGURE, &stream, sizeof(stream) - 1u) == AXDR_REASON_INVALID_PAYLOAD);
    stream.sample_rate_hz = 2000u;
    expect("unqualified rate rejected", request(AXDR_OPCODE_OBSERVATION_CONFIGURE, &stream, sizeof(stream)) == AXDR_REASON_INVALID_PAYLOAD);
    stream.sample_rate_hz = 1000u;
    stream.channel_count = 5u;
    expect("too many channels", request(AXDR_OPCODE_OBSERVATION_CONFIGURE, &stream, sizeof(stream)) == AXDR_REASON_INVALID_PAYLOAD);
    stream.channel_count = 4u;
    stream.channel_ids[3] = 1u;
    expect("duplicate channel", request(AXDR_OPCODE_OBSERVATION_CONFIGURE, &stream, sizeof(stream)) == AXDR_REASON_INVALID_PAYLOAD);
    stream.channel_ids[3] = 33u;
    expect("unimplemented channel", request(AXDR_OPCODE_OBSERVATION_CONFIGURE, &stream, sizeof(stream)) == AXDR_REASON_INVALID_PAYLOAD);
    stream.channel_ids[3] = 5u;
    stream.channel_ids[7] = 1u;
    expect("hidden channel must zero", request(AXDR_OPCODE_OBSERVATION_CONFIGURE, &stream, sizeof(stream)) == AXDR_REASON_INVALID_PAYLOAD);
    stream.channel_ids[7] = 0u;
    expect("short output cannot mutate config", service_observation_route(NULL, AXDR_OPCODE_OBSERVATION_CONFIGURE,
        (const uint8_t *)&stream, sizeof(stream), response, 1u, &length, &ack) == AXDR_REASON_INVALID_LENGTH);
    memcpy(unaligned + 1u, &stream, sizeof(stream));
    expect("unaligned request safe", request(AXDR_OPCODE_OBSERVATION_CONFIGURE, unaligned + 1u, sizeof(stream)) == 0u);
    step(1u, 0.0f);
    expect("capture while stream rejected", request(AXDR_OPCODE_OBSERVATION_CAPTURE, &capture, sizeof(capture)) == AXDR_REASON_BUSY);
    stream.enabled = 0u;
    expect("disable", request(AXDR_OPCODE_OBSERVATION_CONFIGURE, &stream, sizeof(stream)) == 0u);
    step(2u, 0.0f);
    capture.pretrigger_percent = 50u;
    expect("unsupported pretrigger", request(AXDR_OPCODE_OBSERVATION_CAPTURE, &capture, sizeof(capture)) == AXDR_REASON_INVALID_PAYLOAD);
    capture.pretrigger_percent = 0u;
    capture.trigger_type = AXDR_SCOPE_TRIGGER_RISING;
    expect("unsupported trigger", request(AXDR_OPCODE_OBSERVATION_CAPTURE, &capture, sizeof(capture)) == AXDR_REASON_INVALID_PAYLOAD);
    capture.trigger_type = AXDR_SCOPE_TRIGGER_MANUAL;
    capture.trigger_level = NAN;
    expect("nonfinite trigger rejected", request(AXDR_OPCODE_OBSERVATION_CAPTURE, &capture, sizeof(capture)) == AXDR_REASON_INVALID_PAYLOAD);
    expect("state payload rejected", request(AXDR_OPCODE_OBSERVATION_STATE, &capture, 1u) == AXDR_REASON_INVALID_PAYLOAD);
    for (i = 0u; i < 17u; i += 2u)
    {
        dir.start_index = (uint16_t)i;
        expect("directory page", request(AXDR_OPCODE_OBSERVATION_DIRECTORY, &dir, sizeof(dir)) == 0u);
        memcpy(&page, response, sizeof(page));
        expect("only real directory entries", page.total_entries == 17u && page.entries[0].channel_id == (i == 16u ? 32u : i + 1u));
    }
    expect("last directory page size", page.returned_entries == 1u);
    dir.start_index = UINT16_MAX;
    expect("directory huge start safe", request(AXDR_OPCODE_OBSERVATION_DIRECTORY, &dir, sizeof(dir)) == 0u);
    memcpy(&page, response, sizeof(page));
    expect("huge start returns empty", page.returned_entries == 0u);
    service_observation_set_timing_stats(1u, 2u, 3u, 160000000u);
    service_observation_set_tx_stats(4u, 5u, 6u, 7u);
    service_observation_set_irq_stats(8u, 9u, 10u, 11u);
    {
        axdr_observation_diagnostics_t diag = diagnostics();
        expect("diagnostic fields real setters", diag.irq_last_cycles == 1u && diag.irq_overruns == 3u && diag.tx_aborted == 6u && diag.irq_count == 8u && diag.adc_overruns == 11u);
    }
}

static void save_file(const char *directory, const char *name, const void *bytes, size_t length)
{
    char path[1024];
    FILE *file;
    const int n = snprintf(path, sizeof(path), "%s/%s", directory, name);
    expect("fixture path fits", n > 0 && (size_t)n < sizeof(path));
    if ((n < 0) || ((size_t)n >= sizeof(path))) return;
    file = fopen(path, "wb");
    expect("fixture file opens", file != NULL);
    if (file == NULL) return;
    expect("fixture written", fwrite(bytes, 1u, length, file) == length);
    expect("fixture closed", fclose(file) == 0);
}

static void save_command_fixture(const char *directory, const char *name,
                                 uint16_t opcode, const void *payload, uint16_t length)
{
    axdr_command_core_t core;
    uint8_t wire_request[128] = {0};
    uint8_t wire_response[AXDR_COMMAND_MAX_RESPONSE_SIZE];
    uint32_t value32 = AXDR_COMMAND_REQUEST_MAGIC;
    uint16_t value16 = AXDR_COMMAND_VERSION;
    uint16_t response_size;
    const uint16_t wire_length = 20u + length;
    memcpy(wire_request, &value32, 4u);
    memcpy(wire_request + 4u, &value16, 2u);
    memcpy(wire_request + 6u, &wire_length, 2u);
    value32 = 1u;
    memcpy(wire_request + 8u, &value32, 4u);
    memcpy(wire_request + 12u, &opcode, 2u);
    memcpy(wire_request + 14u, &length, 2u);
    if (length != 0u) memcpy(wire_request + 16u, payload, length);
    value32 = axdr_command_crc32(wire_request, wire_length - 4u);
    memcpy(wire_request + wire_length - 4u, &value32, 4u);
    axdr_command_core_init(&core);
    response_size = axdr_command_core_process(&core, wire_request, wire_length,
        service_observation_route, NULL, wire_response, sizeof(wire_response));
    expect("fixture command accepted", wire_response[15] == 0u);
    save_file(directory, name, wire_response, response_size);
}

static void fixtures(const char *directory)
{
    axdr_scope_stream_config_request_t stream = stream_request(1000u);
    axdr_scope_capture_arm_request_t capture = capture_request();
    axdr_scope_capture_read_request_t read = {1u, 0u};
    uint32_t i;
    reset();
    expect("fixture stream config", request(AXDR_OPCODE_OBSERVATION_CONFIGURE, &stream, sizeof(stream)) == 0u);
    for (i = 0u; i <= 80u; ++i) step(UINT32_MAX - 39u + i, 0.125f + (float)i);
    service_observation_poll();
    save_file(directory, "stream.axsf.bin", &tx_frame, sizeof(tx_frame));
    reset();
    g_foc.ref.id = -0.75f;
    expect("fixture capture arm", request(AXDR_OPCODE_OBSERVATION_CAPTURE, &capture, sizeof(capture)) == 0u);
    for (i = 0u; i < 1024u; ++i) step(UINT32_MAX - 39u + i, (float)i + 0.125f);
    save_command_fixture(directory, "capture-state.axca.bin", AXDR_OPCODE_OBSERVATION_STATE, NULL, 0u);
    save_command_fixture(directory, "capture-chunk.axca.bin", AXDR_OPCODE_OBSERVATION_READ, &read, sizeof(read));
    service_observation_set_timing_stats(1200u, 1600u, 0u, 160000000u);
    service_observation_set_tx_stats(7u, 2u, 1u, 3u);
    service_observation_set_irq_stats(1024u, 0u, 8001u, 0u);
    save_command_fixture(directory, "diagnostics.axca.bin", AXDR_OPCODE_OBSERVATION_DIAGNOSTICS, NULL, 0u);
}

int main(int argc, char **argv)
{
    service_observation_capture(); /* 启动 ADC 早于服务初始化时必须无动作。 */
    test_stream();
    test_capture();
    test_requests();
    if (argc > 1) fixtures(argv[1]);
    printf("Passive observation: %u checks, %d failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
