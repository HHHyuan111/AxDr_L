/**
 * @file motor_config.h
 * @brief 当前可选电机的物理参数、相序、零位和应用限制。
 */

#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#include "phase_order.h"

/* 当前编译使用的电机型号。 */
#define MOTOR_MODEL_PR60                       (1U)
#define MOTOR_MODEL_2312S                      (2U)
#define MOTOR_SELECTED_MODEL                  (MOTOR_MODEL_PR60)

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
#define PR60_PHASE_ORDER                      (PHASE_ORDER_ABC)
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
#define MOTOR_2312S_PHASE_ORDER               (PHASE_ORDER_ACB)
#define MOTOR_2312S_ELECTRICAL_OFFSET_RAD     (2.34354496f)
#define MOTOR_2312S_ROTOR_OFFSET_RAD          (2.12998796f)
#define MOTOR_2312S_MECHANICAL_OFFSET_RAD     (0.0f)

/* 两个电机当前共用的应用限制。 */
#define MOTOR_COMMAND_USAGE_RATIO             (0.8f)
#define MOTOR_MAX_POSITION_RAD                (20000.0f)

#endif /* MOTOR_CONFIG_H */
