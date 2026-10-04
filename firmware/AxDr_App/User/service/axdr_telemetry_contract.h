#ifndef AXDR_TELEMETRY_CONTRACT_H
#define AXDR_TELEMETRY_CONTRACT_H

#include <stdint.h>
#include "axdr_wire_crc.h"

/*
 * AxDr USB telemetry wire contract.
 *
 * Contract headers must remain independent from HAL, BSP, motor-control state,
 * and application services. Multi-byte values are little-endian. Floating
 * point values are IEEE-754 binary32 and use the units stated in field names.
 */

#define AXDR_WIRE_MAGIC                         0x52445841u
#define AXDR_TELEMETRY_V1_VERSION               1u
#define AXDR_TELEMETRY_V1_FRAME_SIZE            160u

typedef struct __attribute__((packed, aligned(4)))
{
    uint32_t magic;
    uint16_t version;
    uint16_t frame_size;
    uint32_t sequence;
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

    uint32_t encoder_raw_count;
    uint32_t fault_bits;

    float cmd_vd_V;
    float cmd_vq_V;
    float cmd_id_A;
    float cmd_iq_A;
    float cmd_wr_rad_s;
    float cmd_wm_rad_s;
    float cmd_posr_rad;
    float cmd_posm_rad;
    float cmd_torm_Nm;

    float encoder_pos_rad;
    float encoder_raw_angle_rad;
    float electrical_angle_rad;
    float forced_electrical_angle_rad;
    float rotor_multi_turn_pos_rad;
    float output_multi_turn_pos_rad;
    float rotor_speed_rad_s;
    float rotor_speed_filtered_rad_s;
    float output_speed_rad_s;

    float phase_current_a_A;
    float phase_current_b_A;
    float phase_current_c_A;
    float current_d_A;
    float current_q_A;
    float voltage_d_V;
    float voltage_q_V;
    float bus_voltage_V;
    float duty_a;
    float duty_b;
    float duty_c;

    uint32_t dropped_frames;
    uint32_t crc32;
} axdr_telemetry_v1_frame_t;

#endif /* AXDR_TELEMETRY_CONTRACT_H */
