/**
 * @file diag_algorithm_config.h
 * @brief 本项目诊断与参数辨识算法的默认实验参数。
 *
 * 本文件只保存“怎么做实验”的参数。电机参数、编码器参数、驱动板能力和
 * 保护限值由 motor_drive_config.h 与当前电机对象提供。
 *
 * 新增诊断算法时，在本文件增加一个独立参数分组，再由 diag_runtime.c
 * 装入对应算法的配置结构体。算法核心不直接依赖本文件。
 */

#ifndef DIAG_ALGORITHM_CONFIG_H
#define DIAG_ALGORITHM_CONFIG_H

#include <stdbool.h>

#include "mc_common.h"
#include "mc_decoupling.h"
#include "mc_encoder_alignment.h"

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

#endif /* DIAG_ALGORITHM_CONFIG_H */
