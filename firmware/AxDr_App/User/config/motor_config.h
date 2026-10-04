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
#define MOTOR_MODEL_CHENSHA                    (3U)
/* 当前验证对象：沉沙 200W PMSM + ABZ（B 库三闭环已验证组合，18 号盘点 §2.1）。 */
#define MOTOR_SELECTED_MODEL                  (MOTOR_MODEL_CHENSHA)

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
#define MOTOR_2312S_PHASE_ORDER               (PHASE_ORDER_ACB)
#define MOTOR_2312S_ELECTRICAL_OFFSET_RAD     (2.34354496f)
#define MOTOR_2312S_ROTOR_OFFSET_RAD          (2.12998796f)
#define MOTOR_2312S_MECHANICAL_OFFSET_RAD     (0.0f)

/* 两个电机当前共用的应用限制。 */
#define MOTOR_COMMAND_USAGE_RATIO             (0.8f)
#define MOTOR_MAX_POSITION_RAD                (20000.0f)

#endif /* MOTOR_CONFIG_H */

/*
 * 沉沙 200W PMSM 档案（值逐字迁自 B 库，2026-09-23 定版 revision 2）：
 * 物理：axdr_chensha_config.h L5-15；速度/位置整定：axdr_chensha_servo_profile.h L16-22
 * （联轴工况实测固化值，证据 docs/速度环重新整定与模式隔离.md）；
 * 台架限幅：axdr_target_config.h L39-50；保护：foc_drv.c L255-258；
 * r_off=0.0240995 为 B 库 foc_drv.c:527 实测；e_off=0——增量 ABZ 无绝对零点，
 * 每次上电必须先做编码器对齐再进有感闭环（B 库门控同款）。
 */
#define CHENSHA_RATED_VOLTAGE_V               (24.0f)
#define CHENSHA_RATED_CURRENT_A               (11.5f)   /* Arms */
#define CHENSHA_RATED_SPEED_RAD_S             (314.159265f) /* 3000rpm */
#define CHENSHA_RATED_TORQUE_NM               (0.69f)   /* Kt_rms * 11.5A */
#define CHENSHA_RATED_POWER_W                 (200.0f)
#define CHENSHA_PEAK_CURRENT_A                (23.0f)   /* Arms */
#define CHENSHA_PEAK_TORQUE_NM                (1.38f)
#define CHENSHA_PEAK_SPEED_RAD_S              (335.0f)  /* 台架 3200rpm */

#define CHENSHA_POLE_PAIRS                    (5.0f)
#define CHENSHA_PHASE_RESISTANCE_OHM          (0.105125f)
#define CHENSHA_D_AXIS_INDUCTANCE_H           (0.0001037615f)
#define CHENSHA_Q_AXIS_INDUCTANCE_H           (0.0001037615f)
#define CHENSHA_AVERAGE_INDUCTANCE_H          (0.0001037615f)
#define CHENSHA_DIFFERENTIAL_INDUCTANCE_H     (0.0f)
/* 磁链由厂家 Kt 推算（Ke 测量定义待确认，B 库同式） */
#define CHENSHA_FLUX_LINKAGE_WB               (0.06f / (1.41421356237f * 1.5f * 5.0f))
#define CHENSHA_VISCOUS_FRICTION_NM_S         (0.0f)
#define CHENSHA_INERTIA_KG_M2                 (0.000032f)
#define CHENSHA_GEAR_RATIO                    (1.0f)
#define CHENSHA_PHASE_ORDER                   (PHASE_ORDER_ABC)
#define CHENSHA_ELECTRICAL_OFFSET_RAD         (0.0f)
#define CHENSHA_ROTOR_OFFSET_RAD              (0.0240995f)
#define CHENSHA_MECHANICAL_OFFSET_RAD         (0.0f)

/* 三环整定：电流环 Kp=L*ibw / Ki=R*ibw（B 库注明 4000 为整定用值，非实测带宽）。 */
#define CHENSHA_CURRENT_BW_RAD_S              (4000.0f)
/* 速度环：联轴工况实测固化值（覆盖机械模型公式整定），kfp=给定权重 0.5。 */
#define CHENSHA_SPEED_KP_A_PER_RAD_S          (0.332909408f)
#define CHENSHA_SPEED_KI_A_PER_RAD            (10.45865749f)
#define CHENSHA_SPEED_REFERENCE_WEIGHT        (0.5f)
#define CHENSHA_SPEED_FEEDBACK_DAMPING        (0.0f)
#define CHENSHA_POSITION_KP_S_INV             (6.28318530718f)
#define CHENSHA_POSITION_KI                   (0.0f)
#define CHENSHA_POSITION_KD                   (0.0f)

/* 台架限幅（B 库 target_config 沉沙段）。 */
#define CHENSHA_COMMAND_CURRENT_LIMIT_A       (8.0f)    /* 指令电流 */
#define CHENSHA_ALIGN_CURRENT_LIMIT_A        (5.0f)    /* 对齐电流 */
/* 自动对齐使用电流：空载 2A 起步（B 库联轴工况 3A 失败/4A 通过，本台空载足够；
 * 上限受上面 LIMIT 宏约束语义：若 2A 吸不合，按 B 库路径升 3~4A）。 */
#define CHENSHA_ALIGN_CURRENT_A               (2.0f)
#define CHENSHA_ACCEL_LIMIT_RAD_S2           (1200.0f)

/* 保护阈值（B 库沉沙定版，0.1s 延时滤波由 prot_cfg_init 统一配置）。 */
#define CHENSHA_OVER_CURRENT_A                (10.0f)
#define CHENSHA_OVER_VOLTAGE_V                (30.0f)
#define CHENSHA_UNDER_VOLTAGE_V               (10.0f)

/* 档案绑定标识：上位机 ConfirmProfile 匹配用；param_store 持久化零位时
 * 随存随校——任一变更即视为换了电机/机械，存储零位作废走重新对齐。 */
#define MOTOR_PROFILE_ID                      (1U)
#define MOTOR_PROFILE_REVISION                (2U) /* 沉沙档案定版 2026-09-23 */
