/**
 * @file diag_runtime.h
 * @brief 电机诊断与参数辨识任务的统一运行上下文。
 */

#ifndef DIAG_RUNTIME_H
#define DIAG_RUNTIME_H

#include <stdbool.h>
#include <stdint.h>

#include "mc_bias_bandwidth.h"
#include "mc_current_pi.h"
#include "mc_current_sweep.h"
#include "mc_deadtime_comp.h"
#include "mc_deadtime_test.h"
#include "mc_decoupling.h"
#include "mc_diag_manager.h"
#include "mc_encoder_alignment.h"
#include "mc_flux_observer.h"
#include "mc_param_ident.h"
#include "mc_pole_pair_ident.h"

#define DIAG_ALIGN_MAX_SAMPLES (256U)

/** 调试器或通信层写入的任务请求。 */
typedef enum
{
    DIAG_REQUEST_NONE = 0,
    DIAG_REQUEST_START = 1,
    DIAG_REQUEST_STOP = 2,
} diag_request_e;

/** 同一时间只能运行其中一个主动激励任务。
 *
 * 2/3 是已退役的 RS_IDENT/L_IDENT（HJY 血脉，随 G11 参数辨识接入由
 * mc_param_ident 取代），编号留空洞防止旧调试脚本误触新任务。
 */
typedef enum
{
    DIAG_JOB_NONE = 0,
    DIAG_JOB_CURRENT_SWEEP = 1,
    DIAG_JOB_POLE_PAIR_IDENT = 4,
    DIAG_JOB_ENCODER_ALIGN = 5,
    DIAG_JOB_DEADTIME_TEST = 6,
    DIAG_JOB_PARAM_IDENT = 7,
    DIAG_JOB_PARAM_IDENT_LQ = 8,
} diag_job_e;

typedef enum
{
    DIAG_ALIGN_IDLE = 0,
    DIAG_ALIGN_RAMP,
    DIAG_ALIGN_HOLD,
    DIAG_ALIGN_SAMPLE,
    DIAG_ALIGN_DONE,
    DIAG_ALIGN_ERROR,
} diag_align_state_e;

/** 编码器零位对齐由本项目编排，最终计算复用公开求解器。 */
typedef struct
{
    float align_current_a;
    float ramp_duration_s;
    float hold_duration_s;
    float target_electrical_angle_rad;
    uint32_t sample_count;
    uint32_t sample_interval_ticks;
    int8_t encoder_direction;
    mc_encoder_alignment_method_t method;
    float minimum_resultant_ratio;
} diag_align_config_t;

typedef struct
{
    diag_align_state_e state;
    uint32_t tick;
    uint32_t ramp_ticks;
    uint32_t hold_ticks;
    uint32_t sample_tick;
    uint32_t sample_index;
    float raw_mechanical_angle_rad[DIAG_ALIGN_MAX_SAMPLES];
    mc_encoder_alignment_result_t result;
} diag_align_runtime_t;

/**
 * @brief 一组可在 STOP 状态下修改的诊断参数。
 *
 * current_limit_a 和 voltage_limit_v 是所有主动测试共同遵守的硬上限。
 * 两者默认均为 0，表示尚未确认实物试验条件，任何主动测试都不得启动。
 */
typedef struct
{
    float control_period_s;
    float current_limit_a;
    float voltage_limit_v;
    float minimum_vbus_v;
    float phase_resistance_ohm;
    uint32_t pole_pairs;
    uint32_t encoder_full_scale;
    int8_t encoder_direction;

    mc_current_sweep_config_t sweep;
    mc_pole_pair_ident_config_t pole_pair;
    diag_align_config_t encoder_align;
    mc_deadtime_test_config_t deadtime_test;

    mc_decoupling_mode_t decoupling_mode;
    mc_decoupling_config_t decoupling;
    mc_flux_config_t flux;
    mc_deadtime_config_t deadtime_comp;
} diag_profile_t;

/** 初始化诊断参数所需的当前电机和位置反馈基线。 */
typedef struct
{
    float control_period_s;
    float phase_resistance_ohm;
    float d_axis_inductance_h;
    float q_axis_inductance_h;
    float flux_linkage_wb;
    uint32_t pole_pairs;
    uint32_t encoder_full_scale;
    int8_t encoder_direction;
} diag_seed_t;

/**
 * @brief 单电机诊断上下文。
 *
 * request 和 requested_job 可由调试器在电机停止时写入；其余字段由快速周期
 * 独占写入。各算法上下文分开保存，便于完成后查看结果，不使用动态内存。
 */
typedef struct diag_runtime
{
    volatile uint32_t request;
    volatile uint32_t requested_job;
    diag_job_e active_job;
    mc_status_t last_status;
    bool active;
    bool command_primed;
    bool voltage_saturated;

    mc_diag_manager_t manager;
    diag_profile_t profile;
    mc_command_t command;
    mc_sample_t last_sample;

    mc_current_sweep_t sweep;
    mc_param_ident_t param_ident;
    mc_pole_pair_ident_t pole_pair;
    diag_align_runtime_t encoder_align;
    mc_deadtime_test_t deadtime_test;
    mc_flux_observer_t flux;
    mc_bias_bandwidth_result_t bias_bandwidth;
} diag_runtime_t;

void diag_runtime_init(diag_runtime_t *runtime,
                       const diag_seed_t *seed);

mc_status_t diag_runtime_start(diag_runtime_t *runtime,
                               float initial_vbus_v,
                               bool platform_ready_and_disabled);

mc_status_t diag_runtime_step(diag_runtime_t *runtime,
                              const mc_sample_t *sample,
                              mc_command_t *command);

mc_status_t diag_runtime_abort(diag_runtime_t *runtime,
                               mc_command_t *command);

void diag_runtime_confirm_stopped(diag_runtime_t *runtime,
                                  bool platform_ready_and_disabled);

/**
 * @brief 按本项目 PI 定义计算连续积分增益。
 *
 * HJY 核心返回每拍 Ki，本项目 PID 内部还会乘采样周期，因此这里除以 Ts 后
 * 输出本项目使用的 Ki，单位为 V/(A·s)。
 */
mc_status_t diag_current_pi_for_project(float bandwidth_hz,
                                        float sample_time_s,
                                        float rs_ohm,
                                        float ld_h,
                                        float lq_h,
                                        float *kp_d,
                                        float *ki_d_per_s,
                                        float *kp_q,
                                        float *ki_q_per_s);

#endif /* DIAG_RUNTIME_H */
