#ifndef AXDR_SAFETY_RUNTIME_H
#define AXDR_SAFETY_RUNTIME_H

#include <stdint.h>

#include "axdr_command_contract.h"

typedef enum
{
    AXDR_PHASE_CURRENT_GUARD_CLEAR = 0,
    AXDR_PHASE_CURRENT_GUARD_OVER_LIMIT = 1,
    AXDR_PHASE_CURRENT_GUARD_INVALID = 2
} axdr_phase_current_guard_result_t;

typedef enum
{
    AXDR_BUS_VOLTAGE_GUARD_CLEAR = 0,
    AXDR_BUS_VOLTAGE_GUARD_UNDER_VOLTAGE = 1,
    AXDR_BUS_VOLTAGE_GUARD_OVER_VOLTAGE = 2,
    AXDR_BUS_VOLTAGE_GUARD_INVALID = 3
} axdr_bus_voltage_guard_result_t;

typedef struct
{
    uint32_t tick_ms;
    uint32_t boot_id;
    uint32_t arm_challenge;
    uint32_t session_id;
    uint32_t last_heartbeat_ms;
    uint32_t heartbeat_timeout_ms;
    uint32_t fault_bits;
    uint32_t transition_count;
    uint32_t timeout_count;
    uint8_t state;
    uint8_t disarm_reason;
    uint8_t interlock_flags;
    uint8_t run_authorized;
    uint8_t shutdown_requested;
    uint32_t shutdown_deadline_ms;
    uint32_t expected_profile_id;
    uint32_t expected_profile_revision;
    uint32_t expected_profile_crc32;
    uint8_t motor_profile_confirmed;
} axdr_safety_runtime_t;

void axdr_safety_runtime_init(axdr_safety_runtime_t *runtime,
                              uint32_t now_ms,
                              uint32_t boot_id);

void axdr_safety_runtime_observe(axdr_safety_runtime_t *runtime,
                                 uint32_t now_ms,
                                 uint32_t fault_bits,
                                 uint8_t phase_outputs_disabled,
                                 uint8_t gate_raw_low);
void axdr_safety_runtime_tick(axdr_safety_runtime_t *runtime,
                              uint32_t now_ms);

void axdr_safety_runtime_configure_motor_profile(
    axdr_safety_runtime_t *runtime,
    uint32_t profile_id,
    uint32_t profile_revision,
    uint32_t profile_crc32);
uint8_t axdr_safety_runtime_confirm_motor_profile(
    axdr_safety_runtime_t *runtime,
    const axdr_motor_profile_confirm_request_t *request);
void axdr_safety_runtime_motor_profile_confirmation(
    const axdr_safety_runtime_t *runtime,
    axdr_motor_profile_confirmation_payload_t *payload);
uint8_t axdr_safety_runtime_motor_profile_confirmed(
    const axdr_safety_runtime_t *runtime);

uint8_t axdr_safety_runtime_arm(axdr_safety_runtime_t *runtime,
                                uint32_t now_ms,
                                uint32_t arm_challenge,
                                uint32_t heartbeat_timeout_ms);

uint8_t axdr_safety_runtime_disarm(axdr_safety_runtime_t *runtime,
                                   uint32_t now_ms);

uint8_t axdr_safety_runtime_heartbeat(axdr_safety_runtime_t *runtime,
                                      uint32_t now_ms,
                                      uint32_t session_id);

void axdr_safety_runtime_snapshot(
    const axdr_safety_runtime_t *runtime,
    axdr_safety_state_payload_t *payload);

uint8_t axdr_safety_runtime_run_authorized(
    const axdr_safety_runtime_t *runtime);
uint8_t axdr_safety_runtime_authorize_run(
    axdr_safety_runtime_t *runtime,
    uint32_t now_ms,
    uint32_t session_id);
void axdr_safety_runtime_revoke_run(
    axdr_safety_runtime_t *runtime,
    uint32_t now_ms);

axdr_phase_current_guard_result_t axdr_safety_phase_current_guard(
    float phase_current_a_A,
    float phase_current_b_A,
    float phase_current_c_A,
    float absolute_limit_A);
axdr_bus_voltage_guard_result_t axdr_safety_bus_voltage_guard(
    float bus_voltage_V,
    float minimum_voltage_V,
    float maximum_voltage_V,
    uint8_t under_voltage_active);
uint8_t axdr_safety_pwm_outputs_active(uint32_t normalized_output_mask);

#endif /* AXDR_SAFETY_RUNTIME_H */
