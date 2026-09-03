/**
 * @file drive_protection.h
 * @brief 与硬件无关的驱动保护判定和故障锁存接口。
 */

#ifndef DRIVE_PROTECTION_H
#define DRIVE_PROTECTION_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    DRIVE_PROTECTION_FAULT_OVER_CURRENT = (1UL << 0U),
    DRIVE_PROTECTION_FAULT_UNDER_VOLTAGE = (1UL << 1U),
    DRIVE_PROTECTION_FAULT_OVER_VOLTAGE = (1UL << 2U),
    DRIVE_PROTECTION_FAULT_MOS_OVER_TEMPERATURE = (1UL << 3U),
    DRIVE_PROTECTION_FAULT_COIL_OVER_TEMPERATURE = (1UL << 4U),
    DRIVE_PROTECTION_FAULT_POSITION_FEEDBACK = (1UL << 5U),
    DRIVE_PROTECTION_FAULT_CURRENT_FEEDBACK = (1UL << 6U),
    DRIVE_PROTECTION_FAULT_BUS_FEEDBACK = (1UL << 7U),
    DRIVE_PROTECTION_FAULT_OVER_SPEED = (1UL << 8U)
} drive_protection_fault_e;

typedef struct
{
    float under_voltage_v;
    float over_voltage_v;
    float over_current_a;
    float mos_over_temperature_c;
    float coil_over_temperature_c;
    float over_speed_rad_s;
    uint32_t under_voltage_samples;
    uint32_t over_voltage_samples;
    uint32_t over_current_samples;
    uint32_t mos_over_temperature_samples;
    uint32_t coil_over_temperature_samples;
    uint32_t over_speed_samples;
    uint32_t invalid_current_samples;
    uint32_t invalid_bus_voltage_samples;
    uint32_t invalid_position_samples;
} drive_protection_config_t;

typedef struct
{
    float ia;
    float ib;
    float ic;
    float vbus;
    float temp_mos;
    float temp_coil;
    float spd;
    bool i_valid;
    bool vbus_valid;
    bool temp_mos_valid;
    bool temp_coil_valid;
    bool spd_valid;
    bool pos_valid;
    bool pwm_on;
} drive_protection_sample_t;

typedef struct
{
    uint32_t under_voltage_count;
    uint32_t over_voltage_count;
    uint32_t over_current_count;
    uint32_t mos_over_temperature_count;
    uint32_t coil_over_temperature_count;
    uint32_t over_speed_count;
    uint32_t invalid_current_count;
    uint32_t invalid_bus_voltage_count;
    uint32_t invalid_position_count;
    uint32_t latched_faults;
} drive_protection_state_t;

/**
 * @brief 执行一次保护判定并锁存故障位。
 *
 * @param[in,out] state 连续超限计数和已锁存故障，由当前快速周期独占写入。
 * @param[in] config 保护阈值和连续样本数。阈值或样本数为 0 时对应保护关闭。
 * @param[in] sample 本周期物理反馈及有效性。电流单位 A、电压单位 V、
 *                  温度单位摄氏度、速度单位 rad/s。
 * @return 包含历史故障和本周期新增故障的锁存位。
 *
 * @pre 三个指针均有效。函数不分配内存、不等待外设，执行路径有界。
 */
uint32_t drive_protection_step(drive_protection_state_t *state,
                               const drive_protection_config_t *config,
                               const drive_protection_sample_t *sample);

/**
 * @brief 清除保护计数和锁存故障。
 *
 * @param[out] state 要清除的保护状态。
 * @pre 只能在 PWM 已关闭且调用方确认故障源已经消失时使用。
 */
void drive_protection_reset(drive_protection_state_t *state);

#endif /* DRIVE_PROTECTION_H */
