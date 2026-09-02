/**
 * @file motor_drive_config.h
 * @brief 当前电机、编码器和驱动板的工程配置。
 *
 * 更换电机或驱动板时优先修改本文件，不修改通用 FOC、诊断算法和 Target
 * 适配代码。值为 0 的诊断限值表示尚未完成实物确认，主动诊断任务禁止启动。
 */

#ifndef MOTOR_DRIVE_CONFIG_H
#define MOTOR_DRIVE_CONFIG_H

#include "common.h"

/* 当前编译使用的电机型号。 */
#define MOTOR_MODEL_PR60                       (1U)
#define MOTOR_MODEL_2312S                      (2U)
#define MOTOR_SELECTED_MODEL                  (MOTOR_MODEL_PR60)

/* PWM 与 FOC 的硬件执行频率。 */
#define DRIVE_FOC_FREQ_HZ                     (20000.0f)

/* 驱动板采样和功率器件参数。 */
#define DRIVE_ADC_REFERENCE_V                 (3.3f)
#define DRIVE_ADC_FULL_SCALE_COUNT            (4096.0f)
#define DRIVE_CURRENT_SHUNT_OHM               (0.001f)
#define DRIVE_CURRENT_AMP_GAIN                (20.0f)
#define DRIVE_VBUS_DIVIDER_HIGH_OHM           (20000.0f)
#define DRIVE_VBUS_DIVIDER_LOW_OHM            (1000.0f)
#define DRIVE_NTC_NOMINAL_OHM                 (10000.0f)
#define DRIVE_NTC_DIVIDER_OHM                 (10000.0f)
#define DRIVE_NTC_ZERO_CELSIUS_K              (273.15f)
#define DRIVE_NTC_BETA_K                      (3950.0f)
#define DRIVE_HARDWARE_DEADTIME_US            (0.5f)

/* 驱动保护阈值。 */
#define DRIVE_UNDER_VOLTAGE_V                 (15.0f)
#define DRIVE_OVER_VOLTAGE_V                  (60.0f)
#define DRIVE_OVER_CURRENT_A                  (80.0f)
#define DRIVE_MOS_OVER_TEMPERATURE_C          (100.0f)
#define DRIVE_COIL_OVER_TEMPERATURE_C         (100.0f)

/* 主动诊断任务的独立硬上限；实物确认前保持 0。 */
#define DRIVE_DIAG_CURRENT_LIMIT_A            (0.0f)
#define DRIVE_DIAG_VOLTAGE_LIMIT_V            (0.0f)

/* MA732 与 MT6816 编码器配置。 */
#define MA732_DIRECTION                       (1)
#define MA732_RESOLUTION_BITS                 (14U)
#define MA732_COUNTS_PER_REV                  (16384U)
#define MT6816_DIRECTION                      (1)
#define MT6816_RESOLUTION_BITS                (14U)
#define MT6816_COUNTS_PER_REV                 (16384U)

/* PR60 电机参数。 */
#define PR60_RATED_VOLTAGE_V                  (24.0f)
#define PR60_RATED_CURRENT_A                  (0.0f)
#define PR60_RATED_SPEED_RAD_S                (3000.0f / 9.55f)
#define PR60_RATED_TORQUE_NM                  (0.8f)
#define PR60_RATED_POWER_W                    (0.0f)
#define PR60_PEAK_CURRENT_A                   (0.0f)
#define PR60_PEAK_TORQUE_NM                   (2.0f)
#define PR60_PEAK_SPEED_RAD_S                 (3000.0f / 9.55f)
#define PR60_POLE_PAIRS                       (10)
#define PR60_PHASE_RESISTANCE_OHM             (0.162977806f)
#define PR60_D_AXIS_INDUCTANCE_H              (0.000108778855f)
#define PR60_Q_AXIS_INDUCTANCE_H              (0.000112416135f)
#define PR60_AVERAGE_INDUCTANCE_H             (0.000110597495f)
#define PR60_DIFFERENTIAL_INDUCTANCE_H        (3.63728032e-6f)
#define PR60_FLUX_LINKAGE_WB                  (0.00498822471f)
#define PR60_VISCOUS_FRICTION_NM_S            (0.000188353f)
#define PR60_INERTIA_KG_M2                    (7.32527915e-5f)
#define PR60_GEAR_RATIO                       (1.0f)
#define PR60_ACCELERATION_RAD_S2              (20.0f)
#define PR60_DECELERATION_RAD_S2              (20.0f)
#define PR60_PHASE_ORDER                      (ABC_PHASE)
#define PR60_ELECTRICAL_OFFSET_RAD            (1.33748674f)
#define PR60_ROTOR_OFFSET_RAD                 (-0.494569868f)
#define PR60_MECHANICAL_OFFSET_RAD            (0.0f)

/* 2312S 电机参数。 */
#define MOTOR_2312S_RATED_VOLTAGE_V           (24.0f)
#define MOTOR_2312S_RATED_CURRENT_A           (0.0f)
#define MOTOR_2312S_RATED_SPEED_RAD_S         (10000.0f / 9.55f)
#define MOTOR_2312S_RATED_TORQUE_NM           (0.8f)
#define MOTOR_2312S_RATED_POWER_W             (0.0f)
#define MOTOR_2312S_PEAK_CURRENT_A            (0.0f)
#define MOTOR_2312S_PEAK_TORQUE_NM            (2.0f)
#define MOTOR_2312S_PEAK_SPEED_RAD_S          (10000.0f / 9.55f)
#define MOTOR_2312S_POLE_PAIRS                (7)
#define MOTOR_2312S_PHASE_RESISTANCE_OHM      (0.108945489f)
#define MOTOR_2312S_D_AXIS_INDUCTANCE_H       (1.97248246e-5f)
#define MOTOR_2312S_Q_AXIS_INDUCTANCE_H       (2.02818483e-5f)
#define MOTOR_2312S_AVERAGE_INDUCTANCE_H      (2.00033355e-7f)
#define MOTOR_2312S_DIFFERENTIAL_INDUCTANCE_H (5.57023668e-7f)
#define MOTOR_2312S_FLUX_LINKAGE_WB           (0.000884152076f)
#define MOTOR_2312S_VISCOUS_FRICTION_NM_S     (0.000188353f)
#define MOTOR_2312S_INERTIA_KG_M2             (2.19904655e-6f)
#define MOTOR_2312S_GEAR_RATIO                (1.0f)
#define MOTOR_2312S_ACCELERATION_RAD_S2        (200.0f)
#define MOTOR_2312S_DECELERATION_RAD_S2        (200.0f)
#define MOTOR_2312S_PHASE_ORDER               (ACB_PHASE)
#define MOTOR_2312S_ELECTRICAL_OFFSET_RAD     (2.34354496f)
#define MOTOR_2312S_ROTOR_OFFSET_RAD          (2.12998796f)
#define MOTOR_2312S_MECHANICAL_OFFSET_RAD     (0.0f)

/* 两个电机当前共用的应用限制。 */
#define MOTOR_COMMAND_USAGE_RATIO             (0.8f)
#define MOTOR_MAX_POSITION_RAD                (20000.0f)

#endif /* MOTOR_DRIVE_CONFIG_H */
