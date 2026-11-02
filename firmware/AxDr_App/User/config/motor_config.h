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
#define MOTOR_MODEL_SERVO200W                    (3U)
/* 当前验证对象：200W 伺服电机 + ABZ（B 库三闭环已验证组合，18 号盘点 §2.1）。 */
#define MOTOR_SELECTED_MODEL                  (MOTOR_MODEL_SERVO200W)

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
 * 200W 伺服电机 档案（值逐字迁自 B 库，2026-09-23 定版 revision 2）：
 * 物理：B 库配置头 L5-15；速度/位置整定：B 库伺服配置头 L16-22
 * （联轴工况实测固化值，证据 docs/速度环重新整定与模式隔离.md）；
 * 台架限幅：axdr_target_config.h L39-50；保护：foc_drv.c L255-258；
 * r_off=0.0240995 为 B 库 foc_drv.c:527 实测；e_off=0——增量 ABZ 无绝对零点，
 * 每次上电必须先做编码器对齐再进有感闭环（B 库门控同款）。
 */
#define SERVO200W_RATED_VOLTAGE_V               (24.0f)
#define SERVO200W_RATED_CURRENT_A               (11.5f)   /* Arms */
#define SERVO200W_RATED_SPEED_RAD_S             (314.159265f) /* 3000rpm */
#define SERVO200W_RATED_TORQUE_NM               (0.69f)   /* Kt_rms * 11.5A */
#define SERVO200W_RATED_POWER_W                 (200.0f)
#define SERVO200W_PEAK_CURRENT_A                (23.0f)   /* Arms */
#define SERVO200W_PEAK_TORQUE_NM                (1.38f)
#define SERVO200W_PEAK_SPEED_RAD_S              (335.0f)  /* 台架 3200rpm */

#define SERVO200W_POLE_PAIRS                    (5.0f)
#define SERVO200W_PHASE_RESISTANCE_OHM          (0.105125f)
#define SERVO200W_D_AXIS_INDUCTANCE_H           (0.0001037615f)
#define SERVO200W_Q_AXIS_INDUCTANCE_H           (0.0001037615f)
#define SERVO200W_AVERAGE_INDUCTANCE_H          (0.0001037615f)
#define SERVO200W_DIFFERENTIAL_INDUCTANCE_H     (0.0f)
/* 磁链由厂家 Kt 推算（Ke 测量定义待确认，B 库同式） */
#define SERVO200W_FLUX_LINKAGE_WB               (0.06f / (1.41421356237f * 1.5f * 5.0f))
#define SERVO200W_VISCOUS_FRICTION_NM_S         (0.0f)
#define SERVO200W_INERTIA_KG_M2                 (0.000032f)
#define SERVO200W_GEAR_RATIO                    (1.0f)
#define SERVO200W_PHASE_ORDER                   (PHASE_ORDER_ABC)
#define SERVO200W_ELECTRICAL_OFFSET_RAD         (0.0f)
#define SERVO200W_ROTOR_OFFSET_RAD              (0.0240995f)
#define SERVO200W_MECHANICAL_OFFSET_RAD         (0.0f)

/* 三环整定：电流环 Kp=L*ibw / Ki=R*ibw。
 * 2026-10-18 降 4000→500：电流环带宽 636→80Hz。测量噪声（2.2LSB 宽带，无 RC 抗混叠）
 * 高于环宽的部分被 Kp 全额直通成电压噪声——人耳敏感的 1~5kHz 恰全在直通带，
 * 即电机可听电流噪声根因。厂商同板固件 ibw=500 跑多台电机安静（foc_drv.c:107/166），
 * 为同硬件验证过的安静工作点。速度环固化值不受影响（motor_profile_control_load 晚绑定覆盖）。 */
#define SERVO200W_CURRENT_BW_RAD_S              (500.0f)
/* 速度环：联轴工况实测固化值（覆盖机械模型公式整定），kfp=给定权重 0.5。 */
#define SERVO200W_SPEED_KP_A_PER_RAD_S          (0.332909408f)
#define SERVO200W_SPEED_KI_A_PER_RAD            (10.45865749f)
#define SERVO200W_SPEED_REFERENCE_WEIGHT        (0.5f)
#define SERVO200W_SPEED_FEEDBACK_DAMPING        (0.0f)
#define SERVO200W_POSITION_KP_S_INV             (6.28318530718f)
#define SERVO200W_POSITION_KI                   (0.0f)
#define SERVO200W_POSITION_KD                   (0.0f)

/* 台架限幅（B 库 target_config 200W 伺服电机段）。 */
#define SERVO200W_COMMAND_CURRENT_LIMIT_A       (8.0f)    /* 指令电流 */
#define SERVO200W_ALIGN_CURRENT_LIMIT_A        (5.0f)    /* 对齐电流 */
/* 自动对齐使用电流：照抄参考预定位 CALIB_TARGET_ID=5A（渐变 1s+保持 1s）。
 * B 库联轴工况实测 3A 失败/4A 通过，本台 2026-10-18 用 5A。 */
#define SERVO200W_ALIGN_CURRENT_A               (5.0f)
#define SERVO200W_ACCEL_LIMIT_RAD_S2           (1200.0f)

/* 保护阈值（B 库200W 伺服电机定版，0.1s 延时滤波由 prot_cfg_init 统一配置）。 */
#define SERVO200W_OVER_CURRENT_A                (10.0f)
#define SERVO200W_OVER_VOLTAGE_V                (30.0f)
#define SERVO200W_UNDER_VOLTAGE_V               (10.0f)

/* 档案绑定标识：上位机 ConfirmProfile 匹配用；param_store 持久化零位时
 * 随存随校——任一变更即视为换了电机/机械，存储零位作废走重新对齐。 */
#define MOTOR_PROFILE_ID                      (1U)
#define MOTOR_PROFILE_REVISION                (2U) /* 200W 伺服电机档案定版 2026-09-23 */
