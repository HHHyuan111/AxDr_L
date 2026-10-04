#include "axdr_safety_runtime.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define AXDR_SAFETY_BOOT_ID_FALLBACK 0x42333131u
#define AXDR_SAFETY_CHALLENGE_XOR 0xA5D35A3Cu
#define AXDR_SAFETY_SESSION_XOR 0x6D2B79F5u
#define AXDR_SAFETY_SHUTDOWN_GRACE_MS 20u

static uint32_t nonzero(uint32_t value, uint32_t fallback)
{
    return value == 0u ? fallback : value;
}

axdr_phase_current_guard_result_t axdr_safety_phase_current_guard(
    float phase_current_a_A,
    float phase_current_b_A,
    float phase_current_c_A,
    float absolute_limit_A)
{
    if (!isfinite(absolute_limit_A) ||
        (absolute_limit_A <= 0.0f) ||
        !isfinite(phase_current_a_A) ||
        !isfinite(phase_current_b_A) ||
        !isfinite(phase_current_c_A))
    {
        return AXDR_PHASE_CURRENT_GUARD_INVALID;
    }
    if ((phase_current_a_A >= absolute_limit_A) ||
        (phase_current_a_A <= -absolute_limit_A) ||
        (phase_current_b_A >= absolute_limit_A) ||
        (phase_current_b_A <= -absolute_limit_A) ||
        (phase_current_c_A >= absolute_limit_A) ||
        (phase_current_c_A <= -absolute_limit_A))
    {
        return AXDR_PHASE_CURRENT_GUARD_OVER_LIMIT;
    }
    return AXDR_PHASE_CURRENT_GUARD_CLEAR;
}

axdr_bus_voltage_guard_result_t axdr_safety_bus_voltage_guard(
    float bus_voltage_V,
    float minimum_voltage_V,
    float maximum_voltage_V,
    uint8_t under_voltage_active)
{
    if (!isfinite(bus_voltage_V) ||
        !isfinite(minimum_voltage_V) ||
        !isfinite(maximum_voltage_V) ||
        (minimum_voltage_V < 0.0f) ||
        (maximum_voltage_V <= minimum_voltage_V))
    {
        return AXDR_BUS_VOLTAGE_GUARD_INVALID;
    }
    if (bus_voltage_V > maximum_voltage_V)
    {
        return AXDR_BUS_VOLTAGE_GUARD_OVER_VOLTAGE;
    }
    if ((under_voltage_active != 0u) &&
        (bus_voltage_V < minimum_voltage_V))
    {
        return AXDR_BUS_VOLTAGE_GUARD_UNDER_VOLTAGE;
    }
    return AXDR_BUS_VOLTAGE_GUARD_CLEAR;
}

uint8_t axdr_safety_pwm_outputs_active(uint32_t normalized_output_mask)
{
    return (normalized_output_mask & 0x40u) != 0u &&
           (normalized_output_mask & 0x3Fu) != 0u ? 1u : 0u;
}

static uint8_t ready_to_arm(const axdr_safety_runtime_t *runtime)
{
    return runtime != NULL &&
           runtime->interlock_flags ==
               AXDR_SAFETY_INTERLOCK_READY_TO_ARM;
}

static void transition(axdr_safety_runtime_t *runtime,
                       uint8_t state,
                       uint8_t reason,
                       uint8_t shutdown_requested)
{
    if ((runtime->state != state) ||
        (runtime->disarm_reason != reason))
    {
        ++runtime->transition_count;
    }
    runtime->state = state;
    runtime->disarm_reason = reason;
    runtime->run_authorized = 0u;
    runtime->shutdown_requested = shutdown_requested;
    runtime->shutdown_deadline_ms = shutdown_requested != 0u ?
        runtime->tick_ms + AXDR_SAFETY_SHUTDOWN_GRACE_MS : 0u;
    if (state != AXDR_SAFETY_ARMED)
    {
        runtime->session_id = 0u;
        runtime->motor_profile_confirmed = 0u;
    }
}

void axdr_safety_runtime_init(axdr_safety_runtime_t *runtime,
                              uint32_t now_ms,
                              uint32_t boot_id)
{
    if (runtime == NULL)
    {
        return;
    }

    memset(runtime, 0, sizeof(*runtime));
    runtime->tick_ms = now_ms;
    runtime->boot_id = nonzero(boot_id, AXDR_SAFETY_BOOT_ID_FALLBACK);
    runtime->arm_challenge = nonzero(
        runtime->boot_id ^ AXDR_SAFETY_CHALLENGE_XOR,
        AXDR_SAFETY_CHALLENGE_XOR);
    runtime->heartbeat_timeout_ms =
        AXDR_SAFETY_HEARTBEAT_TIMEOUT_DEFAULT_MS;
    runtime->state = AXDR_SAFETY_DISARMED;
    runtime->disarm_reason = AXDR_SAFETY_DISARM_BOOT;
    runtime->shutdown_requested = 1u;
}

void axdr_safety_runtime_observe(axdr_safety_runtime_t *runtime,
                                 uint32_t now_ms,
                                 uint32_t fault_bits,
                                 uint8_t phase_outputs_disabled,
                                 uint8_t gate_raw_low)
{
    uint8_t interlocks = 0u;

    if (runtime == NULL)
    {
        return;
    }

    runtime->tick_ms = now_ms;
    runtime->fault_bits = fault_bits;
    if (phase_outputs_disabled != 0u)
    {
        interlocks |= AXDR_SAFETY_INTERLOCK_PHASE_OUTPUTS_DISABLED;
    }
    if (gate_raw_low != 0u)
    {
        interlocks |= AXDR_SAFETY_INTERLOCK_GATE_RAW_LOW;
    }
    if (fault_bits == 0u)
    {
        interlocks |= AXDR_SAFETY_INTERLOCK_FAULTS_CLEAR;
    }
    runtime->interlock_flags = interlocks;

    if (fault_bits != 0u)
    {
        transition(runtime, AXDR_SAFETY_FAULT_LOCKED,
                   AXDR_SAFETY_DISARM_FAULT_ACTIVE, 1u);
        return;
    }
    if ((runtime->run_authorized == 0u) &&
        ((interlocks &
          (AXDR_SAFETY_INTERLOCK_PHASE_OUTPUTS_DISABLED |
           AXDR_SAFETY_INTERLOCK_GATE_RAW_LOW)) !=
         (AXDR_SAFETY_INTERLOCK_PHASE_OUTPUTS_DISABLED |
          AXDR_SAFETY_INTERLOCK_GATE_RAW_LOW)) &&
        !(runtime->shutdown_requested != 0u &&
          (int32_t)(runtime->shutdown_deadline_ms - now_ms) >= 0))
    {
        transition(runtime, AXDR_SAFETY_FAULT_LOCKED,
                   AXDR_SAFETY_DISARM_INTERLOCK_OPEN, 1u);
        return;
    }
    axdr_safety_runtime_tick(runtime, now_ms);
}

void axdr_safety_runtime_tick(axdr_safety_runtime_t *runtime,
                              uint32_t now_ms)
{
    if (runtime == NULL)
    {
        return;
    }
    runtime->tick_ms = now_ms;
    if ((runtime->state == AXDR_SAFETY_ARMED) &&
        ((uint32_t)(now_ms - runtime->last_heartbeat_ms) >
         runtime->heartbeat_timeout_ms))
    {
        ++runtime->timeout_count;
        transition(runtime, AXDR_SAFETY_DISARMED,
                   AXDR_SAFETY_DISARM_HEARTBEAT_TIMEOUT, 1u);
    }
}

void axdr_safety_runtime_configure_motor_profile(
    axdr_safety_runtime_t *runtime,
    uint32_t profile_id,
    uint32_t profile_revision,
    uint32_t profile_crc32)
{
    if (runtime == NULL)
    {
        return;
    }
    if ((runtime->expected_profile_id != profile_id) ||
        (runtime->expected_profile_revision != profile_revision) ||
        (runtime->expected_profile_crc32 != profile_crc32))
    {
        runtime->motor_profile_confirmed = 0u;
        if (runtime->state == AXDR_SAFETY_ARMED)
        {
            transition(runtime, AXDR_SAFETY_DISARMED,
                       AXDR_SAFETY_DISARM_INTERNAL, 1u);
        }
    }
    runtime->expected_profile_id = profile_id;
    runtime->expected_profile_revision = profile_revision;
    runtime->expected_profile_crc32 = profile_crc32;
}

uint8_t axdr_safety_runtime_confirm_motor_profile(
    axdr_safety_runtime_t *runtime,
    const axdr_motor_profile_confirm_request_t *request)
{
    if ((runtime == NULL) || (request == NULL))
    {
        return AXDR_REASON_INVALID_PAYLOAD;
    }
    if (runtime->state != AXDR_SAFETY_DISARMED)
    {
        return AXDR_REASON_INVALID_STATE;
    }
    if ((runtime->expected_profile_id == 0u) ||
        (runtime->expected_profile_revision == 0u) ||
        (runtime->expected_profile_crc32 == 0u) ||
        (request->profile_id != runtime->expected_profile_id) ||
        (request->profile_revision != runtime->expected_profile_revision) ||
        (request->profile_crc32 != runtime->expected_profile_crc32))
    {
        runtime->motor_profile_confirmed = 0u;
        return AXDR_REASON_MOTOR_PROFILE_MISMATCH;
    }
    runtime->motor_profile_confirmed = 1u;
    return AXDR_REASON_NONE;
}

void axdr_safety_runtime_motor_profile_confirmation(
    const axdr_safety_runtime_t *runtime,
    axdr_motor_profile_confirmation_payload_t *payload)
{
    if (payload == NULL)
    {
        return;
    }
    memset(payload, 0, sizeof(*payload));
    if (runtime == NULL)
    {
        return;
    }
    payload->profile_id = runtime->expected_profile_id;
    payload->profile_revision = runtime->expected_profile_revision;
    payload->profile_crc32 = runtime->expected_profile_crc32;
    payload->confirmed = runtime->motor_profile_confirmed;
}

uint8_t axdr_safety_runtime_motor_profile_confirmed(
    const axdr_safety_runtime_t *runtime)
{
    return runtime != NULL &&
           runtime->expected_profile_id != 0u &&
           runtime->expected_profile_revision != 0u &&
           runtime->expected_profile_crc32 != 0u &&
           runtime->motor_profile_confirmed != 0u ? 1u : 0u;
}

uint8_t axdr_safety_runtime_arm(axdr_safety_runtime_t *runtime,
                                uint32_t now_ms,
                                uint32_t arm_challenge,
                                uint32_t heartbeat_timeout_ms)
{
    uint32_t session_id;

    if (runtime == NULL)
    {
        return AXDR_REASON_INVALID_PAYLOAD;
    }
    if (runtime->state == AXDR_SAFETY_FAULT_LOCKED)
    {
        return AXDR_REASON_FAULT_ACTIVE;
    }
    if (runtime->state != AXDR_SAFETY_DISARMED)
    {
        return AXDR_REASON_INVALID_STATE;
    }
    if (arm_challenge != runtime->arm_challenge)
    {
        return AXDR_REASON_CHALLENGE_MISMATCH;
    }
    if ((heartbeat_timeout_ms <
         AXDR_SAFETY_HEARTBEAT_TIMEOUT_MIN_MS) ||
        (heartbeat_timeout_ms >
         AXDR_SAFETY_HEARTBEAT_TIMEOUT_MAX_MS))
    {
        return AXDR_REASON_INVALID_PAYLOAD;
    }
    if (!ready_to_arm(runtime))
    {
        return runtime->fault_bits != 0u ?
            AXDR_REASON_FAULT_ACTIVE :
            AXDR_REASON_INTERLOCK_OPEN;
    }
    if ((runtime->expected_profile_id != 0u) &&
        (axdr_safety_runtime_motor_profile_confirmed(runtime) == 0u))
    {
        return AXDR_REASON_MOTOR_PROFILE_NOT_CONFIRMED;
    }

    session_id = runtime->arm_challenge ^
                 AXDR_SAFETY_SESSION_XOR ^
                 now_ms ^
                 (runtime->transition_count + 1u);
    runtime->session_id = nonzero(session_id,
                                  AXDR_SAFETY_SESSION_XOR);
    runtime->tick_ms = now_ms;
    runtime->last_heartbeat_ms = now_ms;
    runtime->heartbeat_timeout_ms = heartbeat_timeout_ms;
    transition(runtime, AXDR_SAFETY_ARMED,
               AXDR_SAFETY_DISARM_NONE, 0u);
    /* ARM creates the session only. A validated motion command must call
     * axdr_safety_runtime_authorize_run() before output can be enabled. */
    runtime->run_authorized = 0u;
    return AXDR_REASON_NONE;
}

uint8_t axdr_safety_runtime_disarm(axdr_safety_runtime_t *runtime,
                                   uint32_t now_ms)
{
    if (runtime == NULL)
    {
        return AXDR_REASON_INVALID_PAYLOAD;
    }
    runtime->tick_ms = now_ms;
    if (runtime->state == AXDR_SAFETY_FAULT_LOCKED)
    {
        runtime->session_id = 0u;
        runtime->run_authorized = 0u;
        runtime->shutdown_requested = 1u;
        return AXDR_REASON_NONE;
    }
    transition(runtime, AXDR_SAFETY_DISARMED,
               AXDR_SAFETY_DISARM_HOST_REQUEST, 1u);
    return AXDR_REASON_NONE;
}

uint8_t axdr_safety_runtime_heartbeat(axdr_safety_runtime_t *runtime,
                                      uint32_t now_ms,
                                      uint32_t session_id)
{
    if (runtime == NULL)
    {
        return AXDR_REASON_INVALID_PAYLOAD;
    }
    if (runtime->state != AXDR_SAFETY_ARMED)
    {
        return AXDR_REASON_INVALID_STATE;
    }
    if ((session_id == 0u) ||
        (session_id != runtime->session_id))
    {
        return AXDR_REASON_SESSION_MISMATCH;
    }

    runtime->tick_ms = now_ms;
    runtime->last_heartbeat_ms = now_ms;
    runtime->shutdown_requested = 0u;
    runtime->shutdown_deadline_ms = 0u;
    return AXDR_REASON_NONE;
}

void axdr_safety_runtime_snapshot(
    const axdr_safety_runtime_t *runtime,
    axdr_safety_state_payload_t *payload)
{
    if (payload == NULL)
    {
        return;
    }
    memset(payload, 0, sizeof(*payload));
    if (runtime == NULL)
    {
        return;
    }

    payload->tick_ms = runtime->tick_ms;
    payload->boot_id = runtime->boot_id;
    payload->arm_challenge = runtime->arm_challenge;
    payload->session_id = runtime->session_id;
    payload->last_heartbeat_ms = runtime->last_heartbeat_ms;
    payload->heartbeat_timeout_ms =
        runtime->heartbeat_timeout_ms;
    payload->fault_bits = runtime->fault_bits;
    payload->transition_count = runtime->transition_count;
    payload->timeout_count = runtime->timeout_count;
    payload->state = runtime->state;
    payload->disarm_reason = runtime->disarm_reason;
    payload->interlock_flags = runtime->interlock_flags;
    payload->run_authorized = runtime->run_authorized;
}

uint8_t axdr_safety_runtime_run_authorized(
    const axdr_safety_runtime_t *runtime)
{
    return runtime != NULL &&
           runtime->state == AXDR_SAFETY_ARMED &&
           runtime->run_authorized != 0u &&
           runtime->fault_bits == 0u;
}

uint8_t axdr_safety_runtime_authorize_run(
    axdr_safety_runtime_t *runtime,
    uint32_t now_ms,
    uint32_t session_id)
{
    if (runtime == NULL)
    {
        return AXDR_REASON_INVALID_PAYLOAD;
    }
    axdr_safety_runtime_tick(runtime, now_ms);
    if (runtime->state == AXDR_SAFETY_FAULT_LOCKED)
    {
        return AXDR_REASON_FAULT_ACTIVE;
    }
    if (runtime->state != AXDR_SAFETY_ARMED)
    {
        return AXDR_REASON_INVALID_STATE;
    }
    if ((session_id == 0u) || (session_id != runtime->session_id))
    {
        return AXDR_REASON_SESSION_MISMATCH;
    }
    if (runtime->fault_bits != 0u)
    {
        return AXDR_REASON_FAULT_ACTIVE;
    }
    /* The disabled-output and gate-low interlocks are entry conditions.
     * They naturally deassert after the first accepted motion command.
     * An already-authorized session may renew its lease while running;
     * state, session and fault checks above remain mandatory. */
    if ((runtime->run_authorized == 0u) && !ready_to_arm(runtime))
    {
        return AXDR_REASON_INTERLOCK_OPEN;
    }
    runtime->tick_ms = now_ms;
    runtime->run_authorized = 1u;
    runtime->shutdown_requested = 0u;
    runtime->shutdown_deadline_ms = 0u;
    return AXDR_REASON_NONE;
}

void axdr_safety_runtime_revoke_run(
    axdr_safety_runtime_t *runtime,
    uint32_t now_ms)
{
    if (runtime == NULL)
    {
        return;
    }
    runtime->tick_ms = now_ms;
    runtime->run_authorized = 0u;
    runtime->shutdown_requested = 1u;
    runtime->shutdown_deadline_ms =
        now_ms + AXDR_SAFETY_SHUTDOWN_GRACE_MS;
}
