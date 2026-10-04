#ifndef AXDR_COMMAND_CONTRACT_H
#define AXDR_COMMAND_CONTRACT_H

#include <stdint.h>
#include "axdr_wire_crc.h"
#include "axdr_scope_contract.h"

/*
 * AxDr USB command wire contract.
 *
 * All multi-byte fields are little-endian. Floating point values are
 * IEEE-754 binary32. This header deliberately has no HAL, BSP, Runtime, or
 * motor-control dependency.
 */

#define AXDR_COMMAND_REQUEST_MAGIC             0x51435841u /* "AXCQ" */
#define AXDR_COMMAND_RESPONSE_MAGIC            0x41435841u /* "AXCA" */
#define AXDR_COMMAND_VERSION                   1u
#define AXDR_COMMAND_REQUEST_HEADER_SIZE       16u
#define AXDR_COMMAND_RESPONSE_HEADER_SIZE      20u
#define AXDR_COMMAND_CRC_SIZE                  4u
#define AXDR_COMMAND_MIN_REQUEST_SIZE          20u
#define AXDR_COMMAND_MIN_RESPONSE_SIZE         24u
#define AXDR_COMMAND_MAX_REQUEST_SIZE          64u
#define AXDR_COMMAND_MAX_RESPONSE_SIZE         160u
#define AXDR_PARAMETER_SCHEMA_VERSION          1u
#define AXDR_PARAMETER_DESCRIPTOR_NAME_SIZE    16u
#define AXDR_PARAMETER_SCHEMA_MAX_PAGE_ENTRIES 4u
#define AXDR_PARAMETER_DRAFT_MAX_ENTRIES       4u
#define AXDR_PARAMETER_REVISION_UNVERSIONED     0u

#define AXDR_CAPABILITY_READ_ONLY_QUERIES      (1u << 0)
#define AXDR_CAPABILITY_RUNTIME_STATE          (1u << 1)
#define AXDR_CAPABILITY_PARAMETER_SCHEMA       (1u << 2)
#define AXDR_CAPABILITY_COMMAND_RETRY_CACHE    (1u << 3)
#define AXDR_CAPABILITY_PARAMETER_DRAFT_VALIDATION (1u << 4)
#define AXDR_CAPABILITY_CONFIG_CONTRACT       (1u << 5)
#define AXDR_CAPABILITY_BOARD_OBSERVATION     (1u << 6)
#define AXDR_CAPABILITY_SAFETY_SESSION        (1u << 7)
#define AXDR_CAPABILITY_USB_MOTION_CONTROL    (1u << 8)
#define AXDR_CAPABILITY_CONTROL_LEASE         (1u << 9)
#define AXDR_CAPABILITY_POSITION_CONTROL      (1u << 10)
#define AXDR_CAPABILITY_ENCODER_ALIGNMENT     (1u << 11)
#define AXDR_CAPABILITY_MOTOR_IDENTIFICATION  (1u << 12)
#define AXDR_CAPABILITY_MOTOR_PROFILE         (1u << 13)
#define AXDR_CAPABILITY_DYNAMIC_CONTROL_LIMITS (1u << 14)
#define AXDR_CAPABILITY_MOTOR_PROFILE_CONFIRMATION (1u << 15)
#define AXDR_CAPABILITY_LINK_DIAGNOSTICS      (1u << 16)
#define AXDR_CAPABILITY_HIGH_RATE_USB_SCOPE   (1u << 17)
#define AXDR_CAPABILITY_SCOPE_SIGNAL_DIRECTORY (1u << 18)
#define AXDR_CAPABILITY_VF_CONTROL            (1u << 19)
#define AXDR_CAPABILITY_IF_CONTROL            (1u << 20)
#define AXDR_CAPABILITY_VDEAD_ACTIVE_TEST     (1u << 21)
#define AXDR_CAPABILITY_SPEED_PI_RUNTIME_TUNING (1u << 22)

typedef enum
{
    AXDR_OPCODE_GET_PROTOCOL_INFO = 0x0001u,
    AXDR_OPCODE_GET_DEVICE_INFO = 0x0002u,
    AXDR_OPCODE_GET_RUNTIME_STATE = 0x0003u,
    AXDR_OPCODE_GET_PARAMETER_SCHEMA = 0x0004u,
    AXDR_OPCODE_VALIDATE_PARAMETER_DRAFT = 0x0005u,
    AXDR_OPCODE_GET_CONFIG_BLOCK = 0x0006u,
    AXDR_OPCODE_GET_BOARD_OBSERVATION = 0x0007u,
    AXDR_OPCODE_GET_SAFETY_STATE = 0x0008u,
    AXDR_OPCODE_SAFETY_ARM = 0x0009u,
    AXDR_OPCODE_SAFETY_DISARM = 0x000Au,
    AXDR_OPCODE_SAFETY_HEARTBEAT = 0x000Bu,
    AXDR_OPCODE_GET_CONTROL_STATE = 0x000Cu,
    AXDR_OPCODE_CONTROL_STOP = 0x000Du,
    AXDR_OPCODE_SET_SPEED = 0x000Eu,
    AXDR_OPCODE_SET_CURRENT = 0x000Fu,
    AXDR_OPCODE_SET_MIT = 0x0010u,
    AXDR_OPCODE_SET_POSITION = 0x0011u,
    AXDR_OPCODE_ALIGN_ENCODER = 0x0012u,
    AXDR_OPCODE_IDENTIFY_RESISTANCE = 0x0013u,
    AXDR_OPCODE_IDENTIFY_INDUCTANCE = 0x0014u,
    AXDR_OPCODE_IDENTIFY_FLUX = 0x0015u,
    AXDR_OPCODE_GET_IDENTIFICATION_STATE = 0x0016u,
    AXDR_OPCODE_GET_MOTOR_PROFILE = 0x0017u,
    AXDR_OPCODE_GET_CONTROL_LIMITS = 0x0018u,
    AXDR_OPCODE_CONFIRM_MOTOR_PROFILE = 0x0019u,
    AXDR_OPCODE_GET_LINK_DIAGNOSTICS = 0x001Au,
    AXDR_OPCODE_CONFIGURE_SCOPE_STREAM = 0x001Bu,
    AXDR_OPCODE_ARM_SCOPE_CAPTURE = 0x001Cu,
    AXDR_OPCODE_GET_SCOPE_CAPTURE_STATE = 0x001Du,
    AXDR_OPCODE_READ_SCOPE_CAPTURE_CHUNK = 0x001Eu,
    AXDR_OPCODE_GET_SCOPE_SIGNAL_DIRECTORY = 0x001Fu,
    /* Opcodes above 0x001F are advertised by dedicated capability bits;
     * supported_opcode_mask intentionally remains a 32-bit legacy field. */
    AXDR_OPCODE_SET_VF = 0x0020u,
    AXDR_OPCODE_SET_IF = 0x0021u,
    AXDR_OPCODE_GET_VDEAD_STATE = 0x0022u,
    AXDR_OPCODE_CONFIGURE_VDEAD = 0x0023u,
    AXDR_OPCODE_SET_VDEAD_COMPENSATION = 0x0024u,
    AXDR_OPCODE_GET_SPEED_PI = 0x0025u,
    AXDR_OPCODE_SET_SPEED_PI = 0x0026u,
    AXDR_OPCODE_RESTORE_SPEED_PI = 0x0027u
} axdr_command_opcode_t;

#define AXDR_SPEED_PI_SCHEMA_VERSION 1u

typedef struct __attribute__((packed))
{
    uint16_t schema_version;
    uint16_t payload_size;
    float kp;
    float ki;
} axdr_speed_pi_set_request_t;

typedef struct __attribute__((packed))
{
    uint16_t schema_version;
    uint16_t payload_size;
    uint32_t generation;
    float current_kp;
    float current_ki;
    float default_kp;
    float default_ki;
    float minimum_kp;
    float maximum_kp;
    float minimum_ki;
    float maximum_ki;
    uint8_t override_active;
    uint8_t ram_only;
    uint16_t reserved;
} axdr_speed_pi_state_payload_t;

#define AXDR_VDEAD_CONTROL_SCHEMA_VERSION 1u
#define AXDR_VDEAD_ESTIMATE_ABS_MIN_V 0.005f
#define AXDR_VDEAD_ESTIMATE_ABS_MAX_V 0.500f
#define AXDR_VDEAD_CURRENT_HYSTERESIS_MIN_A 0.010f
#define AXDR_VDEAD_CURRENT_HYSTERESIS_MAX_A 0.300f
#define AXDR_VDEAD_COMPENSATION_VECTOR_MIN_V 0.100f
#define AXDR_VDEAD_COMPENSATION_VECTOR_MAX_V 1.000f

typedef enum
{
    AXDR_VDEAD_DISABLE_NONE = 0u,
    AXDR_VDEAD_DISABLE_HOST_REQUEST = 1u,
    AXDR_VDEAD_DISABLE_CONTROL_STOP = 2u,
    AXDR_VDEAD_DISABLE_SESSION_MISMATCH = 3u,
    AXDR_VDEAD_DISABLE_FAULT = 4u,
    AXDR_VDEAD_DISABLE_INVALID_CONFIG = 5u
} axdr_vdead_disable_reason_t;

typedef struct __attribute__((packed))
{
    uint32_t session_id;
    uint32_t config_generation;
    float written_estimate_V;
    float current_hysteresis_A;
    float maximum_compensation_vector_V;
} axdr_vdead_configure_request_t;

typedef struct __attribute__((packed))
{
    uint32_t session_id;
    uint32_t config_generation;
    uint8_t enabled;
    uint8_t reserved0;
    uint16_t reserved1;
} axdr_vdead_compensation_request_t;

typedef struct __attribute__((packed))
{
    uint16_t schema_version;
    uint16_t payload_size;
    uint32_t tick_ms;
    uint32_t config_session_id;
    uint32_t config_generation;
    uint32_t command_count;
    uint32_t auto_disable_count;
    uint32_t accepted_samples;
    float written_estimate_V;
    float current_hysteresis_A;
    float maximum_compensation_vector_V;
    float online_estimate_V;
    float batch_estimate_V;
    float residual_V;
    float gamma;
    float d_axis_compensation_V;
    float q_axis_compensation_V;
    uint16_t quality_flags;
    uint8_t estimator_status;
    uint8_t estimator_reason;
    uint8_t compensation_state;
    uint8_t command_enabled;
    uint8_t effective_enabled;
    uint8_t config_valid;
    uint8_t compensation_limited;
    uint8_t disable_reason;
    uint8_t phase_order;
    uint8_t reserved;
} axdr_vdead_state_payload_t;

typedef enum
{
    AXDR_ACK_ACCEPTED = 1u,
    AXDR_ACK_REJECTED = 2u,
    AXDR_ACK_APPLIED = 3u
} axdr_ack_state_t;

typedef enum
{
    AXDR_REASON_NONE = 0u,
    AXDR_REASON_UNSUPPORTED_VERSION = 1u,
    AXDR_REASON_INVALID_LENGTH = 2u,
    AXDR_REASON_BAD_CRC = 3u,
    AXDR_REASON_UNKNOWN_OPCODE = 4u,
    AXDR_REASON_INVALID_PAYLOAD = 5u,
    AXDR_REASON_SEQUENCE_CONFLICT = 6u,
    AXDR_REASON_BUSY = 7u,
    AXDR_REASON_INVALID_STATE = 8u,
    AXDR_REASON_FAULT_ACTIVE = 9u,
    AXDR_REASON_INTERLOCK_OPEN = 10u,
    AXDR_REASON_CHALLENGE_MISMATCH = 11u,
    AXDR_REASON_SESSION_MISMATCH = 12u,
    AXDR_REASON_OUT_OF_RANGE = 13u,
    AXDR_REASON_COMMAND_TIMEOUT = 14u,
    AXDR_REASON_CONTROL_NOT_READY = 15u,
    AXDR_REASON_MOTOR_PROFILE_NOT_CONFIRMED = 16u,
    AXDR_REASON_MOTOR_PROFILE_MISMATCH = 17u
} axdr_ack_reason_t;

#define AXDR_MOTOR_PROFILE_SCHEMA_VERSION 1u
#define AXDR_MOTOR_PROFILE_NAME_SIZE 16u
#define AXDR_MOTOR_PROFILE_REVISION_INITIAL 1u
#define AXDR_CONTROL_LIMITS_SCHEMA_VERSION 2u
#define AXDR_LINK_DIAGNOSTICS_VERSION 1u

#define AXDR_CONTROL_LIMIT_FLAG_PROFILE_VALID       (1u << 0)
#define AXDR_CONTROL_LIMIT_FLAG_COMMISSIONING_LIMIT (1u << 1)
#define AXDR_CONTROL_LIMIT_FLAG_BOARD_LIMIT         (1u << 2)
#define AXDR_CONTROL_LIMIT_FLAG_MODE_LIMIT          (1u << 3)

typedef enum
{
    AXDR_CONTROL_STOPPED = 0u,
    AXDR_CONTROL_SPEED = 1u,
    AXDR_CONTROL_CURRENT = 2u,
    AXDR_CONTROL_MIT = 3u,
    AXDR_CONTROL_POSITION = 4u,
    AXDR_CONTROL_ALIGNMENT = 5u,
    AXDR_CONTROL_IDENTIFICATION = 6u,
    AXDR_CONTROL_VF = 7u,
    AXDR_CONTROL_IF = 8u
} axdr_control_mode_t;

typedef enum
{
    AXDR_CONTROL_STOP_NONE = 0u,
    AXDR_CONTROL_STOP_HOST_REQUEST = 1u,
    AXDR_CONTROL_STOP_LEASE_TIMEOUT = 2u,
    AXDR_CONTROL_STOP_SAFETY_SESSION = 3u,
    AXDR_CONTROL_STOP_FAULT = 4u,
    AXDR_CONTROL_STOP_MODE_CHANGE = 5u
} axdr_control_stop_reason_t;

typedef enum
{
    AXDR_SAFETY_DISARMED = 0u,
    AXDR_SAFETY_ARMED = 1u,
    AXDR_SAFETY_FAULT_LOCKED = 2u
} axdr_safety_state_t;

typedef enum
{
    AXDR_SAFETY_DISARM_NONE = 0u,
    AXDR_SAFETY_DISARM_BOOT = 1u,
    AXDR_SAFETY_DISARM_HOST_REQUEST = 2u,
    AXDR_SAFETY_DISARM_HEARTBEAT_TIMEOUT = 3u,
    AXDR_SAFETY_DISARM_FAULT_ACTIVE = 4u,
    AXDR_SAFETY_DISARM_INTERLOCK_OPEN = 5u,
    AXDR_SAFETY_DISARM_INTERNAL = 6u
} axdr_safety_disarm_reason_t;

#define AXDR_SAFETY_INTERLOCK_PHASE_OUTPUTS_DISABLED (1u << 0)
#define AXDR_SAFETY_INTERLOCK_GATE_RAW_LOW           (1u << 1)
#define AXDR_SAFETY_INTERLOCK_FAULTS_CLEAR           (1u << 2)
#define AXDR_SAFETY_INTERLOCK_READY_TO_ARM            \
    (AXDR_SAFETY_INTERLOCK_PHASE_OUTPUTS_DISABLED |   \
     AXDR_SAFETY_INTERLOCK_GATE_RAW_LOW |             \
     AXDR_SAFETY_INTERLOCK_FAULTS_CLEAR)

#define AXDR_SAFETY_HEARTBEAT_TIMEOUT_MIN_MS 100u
#define AXDR_SAFETY_HEARTBEAT_TIMEOUT_DEFAULT_MS 500u
#define AXDR_SAFETY_HEARTBEAT_TIMEOUT_MAX_MS 2000u

#define AXDR_CONTROL_LEASE_MIN_MS 100u
#define AXDR_CONTROL_LEASE_DEFAULT_MS 300u
#define AXDR_CONTROL_LEASE_MAX_MS 1000u
#define AXDR_CONTROL_RAMP_MAX_MS 5000u
/* 2026-09-22 重定目标：沉沙 200W（3000rpm 额定 / 3200rpm 最高 / iq 台架限 ±8A）。
 * 旧 42EM64 值（300rpm/1A/0.6A/0.2A/1A）见 git 历史。 */
#define AXDR_CONTROL_MAX_SPEED_RAD_S 335.0f /* 3200 rpm */
#define AXDR_CONTROL_MAX_CURRENT_A 8.0f
/* MIT position is an absolute output-shaft multi-turn position in radians. */
#define AXDR_CONTROL_MAX_MIT_POSITION_RAD 20000.0f
#define AXDR_CONTROL_MAX_MIT_VELOCITY_RAD_S 10.47197551f /* 100 rpm */
#define AXDR_CONTROL_MAX_MIT_KP_A_RAD 11.4591559f /* 0.2 A/deg */
#define AXDR_CONTROL_MAX_MIT_KD_A_RAD_S 0.47746483f /* 0.05 A/rpm */
#define AXDR_CONTROL_SPEED_MAX_IQ_A 8.0f
#define AXDR_CONTROL_MIT_MAX_IQ_A 0.35f
#define AXDR_CONTROL_MAX_VF_VOLTAGE_V 2.0f
#define AXDR_CONTROL_MAX_IF_CURRENT_A 4.0f
#define AXDR_CONTROL_TWO_PI_RAD 6.28318530717958647692f
#define AXDR_CONTROL_MAX_POSITION_RAD 20000.0f
#define AXDR_CONTROL_MAX_POSITION_VELOCITY_RAD_S 335.0f /* 3200 rpm */
#define AXDR_CONTROL_MAX_POSITION_ACCEL_RAD_S2 1200.0f
#define AXDR_CONTROL_ALIGNMENT_MAX_ID_A 5.0f
#define AXDR_CONTROL_ALIGNMENT_MIN_EFFECTIVE_ID_A 0.05f
#define AXDR_CONTROL_ALIGNMENT_MIN_HOLD_MS 300u
#define AXDR_IDENTIFICATION_MAX_CURRENT_A 8.0f
#define AXDR_IDENTIFICATION_MAX_INJECTION_V 1.5f
#define AXDR_IDENTIFICATION_MIN_SETTLE_MS 20u
#define AXDR_IDENTIFICATION_MAX_SETTLE_MS 2000u
#define AXDR_IDENTIFICATION_MIN_SAMPLE_MS 10u
#define AXDR_IDENTIFICATION_MAX_SAMPLE_MS 1000u
#define AXDR_IDENTIFICATION_MIN_PULSE_CYCLES 64u
#define AXDR_IDENTIFICATION_MAX_PULSE_CYCLES 1024u
#define AXDR_IDENTIFICATION_MIN_SPEED_RAD_S 3.0f
#define AXDR_IDENTIFICATION_MAX_SPEED_RAD_S 60.0f
#define AXDR_IDENTIFICATION_MIN_ACCEL_RAD_S2 5.0f
#define AXDR_IDENTIFICATION_MAX_ACCEL_RAD_S2 500.0f
#define AXDR_IDENTIFICATION_MIN_BUS_VOLTAGE_V 10.0f
#define AXDR_IDENTIFICATION_MAX_BUS_VOLTAGE_V 30.0f

typedef enum
{
    AXDR_VALUE_FLOAT32 = 1u,
    AXDR_VALUE_UINT32 = 2u
} axdr_parameter_value_type_t;

typedef enum
{
    AXDR_PARAMETER_READ_ONLY = 1u
} axdr_parameter_access_t;

typedef enum
{
    AXDR_DRAFT_VALIDATION_OK = 0u,
    AXDR_DRAFT_VALIDATION_UNKNOWN_PARAMETER = 1u,
    AXDR_DRAFT_VALIDATION_TYPE_MISMATCH = 2u,
    AXDR_DRAFT_VALIDATION_DUPLICATE_PARAMETER = 3u,
    AXDR_DRAFT_VALIDATION_NOT_FINITE = 4u,
    AXDR_DRAFT_VALIDATION_OUT_OF_DOMAIN = 5u,
    AXDR_DRAFT_VALIDATION_NON_INTEGER = 6u
} axdr_parameter_draft_validation_code_t;

typedef enum
{
    AXDR_UNIT_NONE = 0u,
    AXDR_UNIT_VOLT = 1u,
    AXDR_UNIT_AMPERE = 2u,
    AXDR_UNIT_OHM = 3u,
    AXDR_UNIT_HENRY = 4u,
    AXDR_UNIT_WEBER = 5u
} axdr_parameter_unit_t;

typedef enum
{
    AXDR_PARAMETER_RATED_VOLTAGE = 1u,
    AXDR_PARAMETER_RATED_CURRENT = 2u,
    AXDR_PARAMETER_PHASE_RESISTANCE = 3u,
    AXDR_PARAMETER_D_AXIS_INDUCTANCE = 4u,
    AXDR_PARAMETER_Q_AXIS_INDUCTANCE = 5u,
    AXDR_PARAMETER_FLUX_LINKAGE = 6u,
    AXDR_PARAMETER_POLE_PAIRS = 7u,
    AXDR_PARAMETER_GEAR_RATIO = 8u
} axdr_parameter_id_t;

typedef struct __attribute__((packed))
{
    uint32_t magic;
    uint16_t version;
    uint16_t frame_size;
    uint32_t sequence;
    uint16_t opcode;
    uint16_t payload_length;
} axdr_command_request_header_t;

typedef struct __attribute__((packed))
{
    uint32_t magic;
    uint16_t version;
    uint16_t frame_size;
    uint32_t sequence;
    uint16_t opcode;
    uint8_t ack_state;
    uint8_t reason;
    uint16_t payload_length;
    uint16_t reserved;
} axdr_command_response_header_t;

typedef struct __attribute__((packed))
{
    uint16_t command_version;
    uint16_t telemetry_version;
    uint16_t telemetry_frame_size;
    uint16_t max_request_size;
    uint16_t max_response_size;
    uint16_t parameter_schema_version;
    uint32_t capability_flags;
    uint32_t supported_opcode_mask;
    uint16_t parameter_count;
    uint16_t reserved;
} axdr_protocol_info_payload_t;

typedef struct __attribute__((packed))
{
    uint32_t device_family;
    uint32_t board_profile;
    uint16_t firmware_major;
    uint16_t firmware_minor;
    uint16_t firmware_patch;
    uint16_t telemetry_version;
    uint16_t command_version;
    uint16_t reserved;
    uint32_t build_id;
    char product_name[16];
} axdr_device_info_payload_t;

typedef struct __attribute__((packed))
{
    uint32_t tick_ms;
    uint8_t ctrl_bit;
    uint8_t state_bit;
    uint8_t sys_mode;
    uint8_t debug_mode;
    uint8_t release_mode;
    uint8_t calibration_mode;
    uint8_t halt_mode;
    uint8_t position_mode;
    uint8_t encoder_type;
    uint8_t phase_order;
    uint8_t reserved0;
    uint8_t reserved1;
    uint32_t fault_bits;
    uint32_t telemetry_decimation;
    uint32_t telemetry_capture_count;
    uint32_t telemetry_sent_count;
    uint32_t telemetry_drop_count;
} axdr_runtime_state_payload_t;

typedef struct __attribute__((packed))
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
} axdr_safety_state_payload_t;

typedef struct __attribute__((packed))
{
    uint32_t arm_challenge;
    uint32_t heartbeat_timeout_ms;
} axdr_safety_arm_request_t;

typedef struct __attribute__((packed))
{
    uint32_t session_id;
} axdr_safety_heartbeat_request_t;

/* Stable motor identity and electrical/mechanical envelope. The CRC is a
 * firmware-produced fingerprint of all stable fields with profile_crc32 set
 * to zero; status_flags are informational and are not included in the CRC. */
typedef struct __attribute__((packed))
{
    uint16_t schema_version;
    uint16_t payload_size;
    uint32_t profile_id;
    uint32_t revision;
    uint32_t profile_crc32;
    uint32_t status_flags;
    uint8_t encoder_type;
    uint8_t pole_pairs;
    uint8_t name_length;
    uint8_t reserved0;
    char name[AXDR_MOTOR_PROFILE_NAME_SIZE];
    float phase_resistance_ohm;
    float d_axis_inductance_H;
    float q_axis_inductance_H;
    float flux_linkage_Wb;
    float continuous_current_A;
    float peak_current_A;
    float maximum_speed_rad_s;
    float maximum_acceleration_rad_s2;
    float minimum_bus_voltage_V;
    float maximum_bus_voltage_V;
} axdr_motor_profile_payload_t;

typedef struct __attribute__((packed))
{
    uint16_t schema_version;
    uint16_t payload_size;
    uint32_t profile_id;
    uint32_t profile_revision;
    uint32_t profile_crc32;
    uint32_t limit_flags;
    float maximum_current_A;
    float maximum_speed_rad_s;
    float maximum_speed_iq_A;
    float maximum_position_rad;
    float maximum_position_velocity_rad_s;
    float maximum_position_acceleration_rad_s2;
    float maximum_position_iq_A;
    float maximum_mit_position_rad;
    float maximum_mit_velocity_rad_s;
    float maximum_mit_kp_A_per_rad;
    float maximum_mit_kd_A_per_rad_s;
    float maximum_mit_feedforward_A;
    float maximum_mit_iq_A;
    float maximum_alignment_current_A;
    float maximum_vf_voltage_V;
    float maximum_if_current_A;
} axdr_control_limits_payload_t;

typedef struct __attribute__((packed))
{
    uint32_t profile_id;
    uint32_t profile_revision;
    uint32_t profile_crc32;
} axdr_motor_profile_confirm_request_t;

typedef struct __attribute__((packed))
{
    uint32_t profile_id;
    uint32_t profile_revision;
    uint32_t profile_crc32;
    uint8_t confirmed;
    uint8_t reserved0;
    uint16_t reserved1;
} axdr_motor_profile_confirmation_payload_t;

/* Cumulative link counters. Frame rates are derived by the host from two
 * snapshots so they remain meaningful at any UI refresh period. */
typedef struct __attribute__((packed))
{
    uint16_t version;
    uint16_t payload_size;
    uint32_t tick_ms;
    uint32_t rx_frame_count;
    uint32_t tx_frame_count;
    uint32_t bad_crc_count;
    uint32_t sequence_gap_count;
    uint32_t resync_count;
    uint32_t parser_reject_count;
    uint32_t rx_overflow_count;
} axdr_link_diagnostics_payload_t;

typedef struct __attribute__((packed))
{
    uint32_t tick_ms;
    uint32_t session_id;
    uint32_t command_deadline_ms;
    uint32_t applied_count;
    uint32_t timeout_count;
    uint8_t mode;
    uint8_t active;
    uint8_t stop_reason;
    uint8_t alignment_valid;
    float speed_target_rad_s;
    float iq_target_A;
    float mit_position_rad;
    float mit_velocity_rad_s;
} axdr_control_state_payload_t;

typedef struct __attribute__((packed))
{
    uint32_t session_id;
    uint16_t lease_ms;
    uint16_t ramp_ms;
    float target_rad_s;
    float iq_limit_A;
} axdr_speed_request_t;

typedef struct __attribute__((packed))
{
    uint32_t session_id;
    uint16_t lease_ms;
    uint16_t ramp_ms;
    float id_target_A;
    float iq_target_A;
} axdr_current_request_t;

typedef struct __attribute__((packed))
{
    uint32_t session_id;
    uint16_t lease_ms;
    uint16_t ramp_ms;
    /* Output-shaft mechanical speed and q-axis voltage magnitude. */
    float target_rad_s;
    float voltage_q_V;
} axdr_vf_request_t;

typedef struct __attribute__((packed))
{
    uint32_t session_id;
    uint16_t lease_ms;
    uint16_t ramp_ms;
    /* Output-shaft mechanical speed and q-axis current magnitude. */
    float target_rad_s;
    float iq_target_A;
} axdr_if_request_t;

typedef struct __attribute__((packed))
{
    uint32_t session_id;
    uint16_t lease_ms;
    uint16_t ramp_ms;
    float id_target_A;
    float forced_electrical_angle_rad;
} axdr_encoder_alignment_request_t;

typedef struct __attribute__((packed))
{
    uint32_t session_id;
    uint16_t lease_ms;
    uint16_t reserved;
    float position_rad;
    float velocity_rad_s;
    float kp_A_per_rad;
    float kd_A_per_rad_s;
    float feedforward_iq_A;
    float iq_limit_A;
} axdr_mit_request_t;

typedef struct __attribute__((packed))
{
    uint32_t session_id;
    uint16_t lease_ms;
    uint16_t reserved;
    /* Absolute output-shaft multi-turn target. */
    float position_rad;
    float velocity_limit_rad_s;
    float acceleration_limit_rad_s2;
    float iq_limit_A;
} axdr_position_request_t;

typedef enum
{
    AXDR_IDENTIFICATION_IDLE = 0u,
    AXDR_IDENTIFICATION_RUNNING = 1u,
    AXDR_IDENTIFICATION_COMPLETE = 2u,
    AXDR_IDENTIFICATION_FAILED = 3u,
    AXDR_IDENTIFICATION_ABORTED = 4u
} axdr_identification_status_t;

typedef enum
{
    AXDR_IDENTIFICATION_NONE = 0u,
    AXDR_IDENTIFICATION_RESISTANCE = 1u,
    AXDR_IDENTIFICATION_INDUCTANCE = 2u,
    AXDR_IDENTIFICATION_FLUX = 3u
} axdr_identification_kind_t;

typedef enum
{
    AXDR_IDENTIFICATION_PHASE_IDLE = 0u,
    AXDR_IDENTIFICATION_PHASE_STARTUP = 1u,
    AXDR_IDENTIFICATION_PHASE_SETTLE = 2u,
    AXDR_IDENTIFICATION_PHASE_SAMPLE = 3u,
    AXDR_IDENTIFICATION_PHASE_D_AXIS = 4u,
    AXDR_IDENTIFICATION_PHASE_Q_AXIS = 5u,
    AXDR_IDENTIFICATION_PHASE_RAMP = 6u,
    AXDR_IDENTIFICATION_PHASE_DECEL = 7u,
    AXDR_IDENTIFICATION_PHASE_CALCULATE = 8u,
    AXDR_IDENTIFICATION_PHASE_DONE = 9u
} axdr_identification_phase_t;

typedef enum
{
    AXDR_IDENTIFICATION_FAILURE_NONE = 0u,
    AXDR_IDENTIFICATION_FAILURE_HOST_ABORT = 1u,
    AXDR_IDENTIFICATION_FAILURE_LEASE_TIMEOUT = 2u,
    AXDR_IDENTIFICATION_FAILURE_SAFETY_SESSION = 3u,
    AXDR_IDENTIFICATION_FAILURE_FAULT = 4u,
    AXDR_IDENTIFICATION_FAILURE_CURRENT_LIMIT = 5u,
    AXDR_IDENTIFICATION_FAILURE_BUS_VOLTAGE = 6u,
    AXDR_IDENTIFICATION_FAILURE_SPEED_TRACKING = 7u,
    AXDR_IDENTIFICATION_FAILURE_VOLTAGE_SATURATION = 8u,
    AXDR_IDENTIFICATION_FAILURE_INSUFFICIENT_DATA = 9u,
    AXDR_IDENTIFICATION_FAILURE_NONFINITE = 10u,
    AXDR_IDENTIFICATION_FAILURE_IMPLAUSIBLE_RESULT = 11u
} axdr_identification_failure_t;

#define AXDR_IDENTIFICATION_QUALITY_FINITE          (1u << 0)
#define AXDR_IDENTIFICATION_QUALITY_ENOUGH_SAMPLES  (1u << 1)
#define AXDR_IDENTIFICATION_QUALITY_BIDIRECTIONAL   (1u << 2)
#define AXDR_IDENTIFICATION_QUALITY_FIT_R2          (1u << 3)
#define AXDR_IDENTIFICATION_QUALITY_LOW_RESIDUAL    (1u << 4)
#define AXDR_IDENTIFICATION_QUALITY_BUS_STABLE      (1u << 5)
#define AXDR_IDENTIFICATION_QUALITY_CURRENT_BOUNDED (1u << 6)
#define AXDR_IDENTIFICATION_QUALITY_SPEED_TRACKED   (1u << 7)
#define AXDR_IDENTIFICATION_QUALITY_VESC_RATIO      (1u << 8)
#define AXDR_IDENTIFICATION_QUALITY_CURRENT_STABLE  (1u << 9)
#define AXDR_IDENTIFICATION_QUALITY_HFI_BIN0_BIN2   (1u << 10)

#define AXDR_IDENTIFICATION_CANDIDATE_RESISTANCE (1u << 0)
#define AXDR_IDENTIFICATION_CANDIDATE_INDUCTANCE (1u << 1)
#define AXDR_IDENTIFICATION_CANDIDATE_FLUX       (1u << 2)

typedef struct __attribute__((packed))
{
    uint32_t session_id;
    uint16_t lease_ms;
    uint16_t settle_ms;
    float current_max_A;
    uint16_t sample_ms;
    uint16_t reserved_steps;
    uint32_t reserved;
} axdr_identify_resistance_request_t;

typedef struct __attribute__((packed))
{
    uint32_t session_id;
    uint16_t lease_ms;
    uint16_t pulse_cycles;
    float injection_voltage_V;
    float current_limit_A;
    uint32_t reserved;
} axdr_identify_inductance_request_t;

typedef struct __attribute__((packed))
{
    uint32_t session_id;
    uint16_t lease_ms;
    uint16_t settle_ms;
    float iq_limit_A;
    float speed_low_rad_s;
    float speed_high_rad_s;
    float acceleration_rad_s2;
    uint16_t sample_ms;
    uint16_t reserved;
} axdr_identify_flux_request_t;

typedef struct __attribute__((packed))
{
    uint32_t tick_ms;
    uint32_t session_id;
    uint32_t command_deadline_ms;
    uint32_t run_count;
    uint32_t sample_count;
    uint32_t rejected_sample_count;
    uint32_t quality_flags;
    uint8_t status;
    uint8_t kind;
    uint8_t phase;
    uint8_t failure;
    uint8_t candidate_valid;
    uint8_t candidate_mask;
    uint8_t reserved0;
    uint8_t reserved1;
    float resistance_ohm;
    float fixed_voltage_V;
    float resistance_sigma_ohm;
    float d_axis_inductance_H;
    float q_axis_inductance_H;
    float inductance_sigma_H;
    float flux_linkage_Wb;
    float flux_sigma_Wb;
    float fit_rmse;
    float fit_r2;
    float max_current_A;
    float min_bus_voltage_V;
    float max_bus_voltage_V;
    float min_speed_rad_s;
    float max_speed_rad_s;
} axdr_identification_state_payload_t;

typedef struct __attribute__((packed))
{
    uint16_t start_index;
    uint16_t max_entries;
} axdr_parameter_schema_request_t;

typedef struct __attribute__((packed))
{
    uint16_t schema_version;
    uint16_t total_count;
    uint16_t start_index;
    uint16_t returned_count;
} axdr_parameter_schema_page_header_t;

typedef struct __attribute__((packed))
{
    uint16_t parameter_id;
    uint8_t value_type;
    uint8_t access;
    uint8_t unit;
    uint8_t name_length;
    uint16_t reserved;
    char name[AXDR_PARAMETER_DESCRIPTOR_NAME_SIZE];
    float current_value;
} axdr_parameter_descriptor_t;

typedef struct __attribute__((packed))
{
    uint16_t schema_version;
    uint16_t entry_count;
    uint32_t draft_id;
} axdr_parameter_draft_request_header_t;

typedef struct __attribute__((packed))
{
    uint16_t parameter_id;
    uint8_t value_type;
    uint8_t reserved;
    float proposed_value;
} axdr_parameter_draft_entry_t;

typedef struct __attribute__((packed))
{
    uint16_t schema_version;
    uint16_t entry_count;
    uint32_t parameter_revision;
    uint32_t draft_id;
    uint32_t draft_crc32;
    uint8_t overall_valid;
    uint8_t applied;
    uint16_t reserved;
} axdr_parameter_draft_response_header_t;

typedef struct __attribute__((packed))
{
    uint16_t parameter_id;
    uint8_t validation_code;
    uint8_t value_type;
    float proposed_value;
} axdr_parameter_draft_result_t;

#endif /* AXDR_COMMAND_CONTRACT_H */
