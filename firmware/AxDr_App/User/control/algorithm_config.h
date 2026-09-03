/**
 * @file algorithm_config.h
 * @brief 本项目控制、诊断与参数辨识算法的统一配置。
 *
 * 本文件保存“控制器怎么调、实验怎么做”的参数。电机物理参数、编码器参数、
 * 驱动板能力和保护限值由 board_config.h 与当前电机对象提供。
 *
 * 新增算法时，在本文件增加一个独立参数分组，再由对应初始化代码装入有类型
 * 的配置结构体。算法核心不直接依赖本文件。
 */

#ifndef ALGORITHM_CONFIG_H
#define ALGORITHM_CONFIG_H

#include <stdbool.h>

#include "mc_common.h"
#include "mc_decoupling.h"
#include "mc_encoder_alignment.h"

/* 三闭环执行频率。电流环直接与 FOC 同频，不重复配置。 */
#define CTRL_SPEED_LOOP_FREQ_HZ                  (10000.0f)
#define CTRL_POSITION_LOOP_FREQ_HZ               (5000.0f)
#define CTRL_SPEED_MEASURE_FREQ_HZ               (1000.0f)
#define CTRL_SIGNAL_FILTER_CUTOFF_HZ             (200.0f)

/* PR60 三闭环整定参数。电流、速度 PI 根据这些目标和电机模型自动计算。 */
#define CTRL_PR60_CURRENT_BANDWIDTH_RAD_S         (500.0f)
#define CTRL_PR60_SPEED_DAMPING_RATIO             (4.0f)
#define CTRL_PR60_SPEED_REFERENCE_WEIGHT          (1.1f)
#define CTRL_PR60_SPEED_FEEDBACK_DAMPING          (0.25f)
#define CTRL_PR60_POSITION_KP                     (12.0f)
#define CTRL_PR60_POSITION_KI                     (0.0f)
#define CTRL_PR60_POSITION_KD                     (0.0f)
#define CTRL_PR60_PROFILE_ACCEL_RAD_S2             (20.0f)
#define CTRL_PR60_PROFILE_DECEL_RAD_S2             (20.0f)

/* 2312S 三闭环整定参数；上电测试前应针对该电机重新确认。 */
#define CTRL_2312S_CURRENT_BANDWIDTH_RAD_S        (500.0f)
#define CTRL_2312S_SPEED_DAMPING_RATIO            (4.0f)
#define CTRL_2312S_SPEED_REFERENCE_WEIGHT         (1.1f)
#define CTRL_2312S_SPEED_FEEDBACK_DAMPING          (0.25f)
#define CTRL_2312S_POSITION_KP                     (12.0f)
#define CTRL_2312S_POSITION_KI                     (0.0f)
#define CTRL_2312S_POSITION_KD                     (0.0f)
#define CTRL_2312S_PROFILE_ACCEL_RAD_S2             (200.0f)
#define CTRL_2312S_PROFILE_DECEL_RAD_S2             (200.0f)

/* 三闭环上电初始限幅；进入 FOC 后会根据实时母线电压和电机能力刷新。 */
#define CTRL_CURRENT_PI_INITIAL_LIMIT_V           (11.0f)
#define CTRL_SPEED_PI_INITIAL_LIMIT_A             (20.0f)
#define CTRL_POSITION_PI_INITIAL_LIMIT_RAD_S      (200.0f)

/* FOC 和停机过程中的可调控制参数。 */
#define CTRL_VOLTAGE_UTILIZATION_RATIO            (0.96f)
#define CTRL_QUICK_STOP_SPEED_THRESHOLD_RAD_S     (0.5f)

/* MIT 默认值；正式命令可在运行前覆盖。 */
#define CTRL_MIT_POSITION_GAIN_NM_PER_RAD          (0.0f)
#define CTRL_MIT_SPEED_GAIN_NM_S_PER_RAD           (0.0f)
#define CTRL_MIT_TORQUE_FEEDFORWARD_NM             (0.0f)

/* 电流环扫频：默认先做 100 Hz 单频验证，确认后再切换为自动扫频。 */
#define DIAG_SWEEP_AXIS                         (MC_AXIS_D)
#define DIAG_SWEEP_SINGLE_POINT                 (true)
#define DIAG_SWEEP_SINGLE_FREQ_HZ               (100.0f)
#define DIAG_SWEEP_START_FREQ_HZ                (20.0f)
#define DIAG_SWEEP_END_FREQ_HZ                  (2000.0f)
#define DIAG_SWEEP_AMPLITUDE_A                  (0.2f)
#define DIAG_SWEEP_OFFSET_A                     (0.0f)
#define DIAG_SWEEP_POINT_COUNT                  (128U)
#define DIAG_SWEEP_SETTLE_CYCLES                (12U)
#define DIAG_SWEEP_MEASURE_CYCLES               (20U)
#define DIAG_SWEEP_MIN_SETTLE_TIME_S            (0.20f)
#define DIAG_SWEEP_MIN_MEASURE_TIME_S           (0.20f)
#define DIAG_SWEEP_MIN_MEASURE_SAMPLES          (4000U)
/* 当前电流环使用未滤波的 id/iq，0 表示不做反馈滤波器相位补偿。 */
#define DIAG_SWEEP_FEEDBACK_FILTER_ALPHA        (0.0f)
#define DIAG_SWEEP_REJECT_SATURATED_POINTS      (true)

/* 定子电阻辨识：电流上下限为 0 时禁止启动，实物验证前必须填写。 */
#define DIAG_RS_CURRENT_MIN_A                    (0.0f)
#define DIAG_RS_CURRENT_MAX_A                    (0.0f)
#define DIAG_RS_POINT_COUNT                      (6U)
#define DIAG_RS_TOTAL_DURATION_S                 (8.0f)
#define DIAG_RS_SETTLING_FRACTION                (0.75f)
#define DIAG_RS_INTEGRAL_GAIN_V_PER_A_S          (2.0f)
#define DIAG_RS_DENOMINATOR_EPSILON              (1.0e-9)

/* 偏置电流电感辨识。 */
#define DIAG_L_AUTO_TUNE                         (true)
#define DIAG_L_BIAS_RATIO                        (0.03f)
#define DIAG_L_RIPPLE_RATIO                      (0.25f)
#define DIAG_L_MANUAL_BIAS_CURRENT_A             (0.0f)
#define DIAG_L_INITIAL_INJECTION_VOLTAGE_V       (1.0f)
#define DIAG_L_LEVEL_TICKS                       (10U)
#define DIAG_L_EDGE_SKIP_TICKS                   (3U)
#define DIAG_L_BIAS_SETTLE_TICKS                 (2000U)
#define DIAG_L_BIAS_STABLE_TICKS                 (500U)
#define DIAG_L_BIAS_TIMEOUT_TICKS                (100000U)
#define DIAG_L_BIAS_TOLERANCE_RATIO              (0.05f)
#define DIAG_L_BIAS_INTEGRAL_GAIN_V_PER_A_S      (2.0f)
#define DIAG_L_TUNE_PAIRS                        (20U)
#define DIAG_L_TARGET_ACCEPTED_PAIRS             (1000U)
#define DIAG_L_MAX_MEASURE_PAIR_MULTIPLIER       (3U)
#define DIAG_L_MIN_BIAS_CURRENT_A                (0.5f)
#define DIAG_L_MAX_BIAS_CURRENT_LIMIT_RATIO      (0.4f)
#define DIAG_L_MIN_TARGET_RIPPLE_A               (0.1f)
#define DIAG_L_MIN_CURRENT_DELTA_A               (0.02f)
#define DIAG_L_MIN_POSITIVE_CURRENT_RATIO        (0.2f)
#define DIAG_L_MIN_INJECTION_VOLTAGE_V           (0.05f)
#define DIAG_L_MAX_INJECTION_VBUS_RATIO          (0.10f)
#define DIAG_L_TUNE_SCALE_MIN                    (0.75f)
#define DIAG_L_TUNE_SCALE_MAX                    (1.25f)
#define DIAG_L_TUNE_RIPPLE_FLOOR_A               (0.02f)
#define DIAG_L_MIN_SLOPE_DIFFERENCE_A_S          (1.0f)
#define DIAG_L_MIN_VALID_INDUCTANCE_H            (1.0e-6f)
#define DIAG_L_MAX_VALID_INDUCTANCE_H            (5.0e-3f)
#define DIAG_L_REQUEST_DEADTIME_COMPENSATION     (true)

/* 极对数辨识：驱动电流为 0 时禁止启动，实物验证前必须填写。 */
#define DIAG_POLE_PAIR_DRIVE_CURRENT_A           (0.0f)
#define DIAG_POLE_PAIR_ELECTRICAL_TURNS          (16.0f)
#define DIAG_POLE_PAIR_ELECTRICAL_SPEED_RAD_S    (MC_PI_F)
#define DIAG_POLE_PAIR_RAMP_DURATION_S           (2.0f)
#define DIAG_POLE_PAIR_MIN_COUNT                 (1)
#define DIAG_POLE_PAIR_MAX_COUNT                 (30)

/* 编码器零位校准：校准电流为 0 时禁止启动，实物验证前必须填写。 */
#define DIAG_ALIGN_CURRENT_A                     (0.0f)
#define DIAG_ALIGN_RAMP_DURATION_S               (1.0f)
#define DIAG_ALIGN_HOLD_DURATION_S               (1.0f)
#define DIAG_ALIGN_TARGET_ELEC_ANGLE_RAD         (0.0f)
#define DIAG_ALIGN_SAMPLE_COUNT                  (128U)
#define DIAG_ALIGN_SAMPLE_INTERVAL_TICKS         (20U)
#define DIAG_ALIGN_METHOD                        (MC_ENCODER_ALIGN_CIRCULAR_ELECTRICAL)
#define DIAG_ALIGN_MIN_RESULTANT_RATIO           (0.9f)

/* 死区测试。电压和电流硬上限仍由驱动器配置提供。 */
#define DIAG_DEADTIME_TARGET_LINE_VOLTAGE_V      (0.15f)

/* 在线解耦和磁链观测的算法选项，电机模型参数由当前电机对象提供。 */
#define DIAG_DECOUPLING_MODE                     (MC_DECOUPLING_NONE)
#define DIAG_DECOUPLING_INCLUDE_RESISTIVE_FF     (false)
#define DIAG_FLUX_FILTER_CUTOFF_HZ               (10.0f)
#define DIAG_FLUX_MIN_ELEC_SPEED_RAD_S           (5.0f)
#define DIAG_FLUX_REJECT_VOLTAGE_SATURATION      (true)

#endif /* ALGORITHM_CONFIG_H */
