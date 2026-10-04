/**
 * @file common.h
 * @brief 正式 FOC 固件共用的数据类型、常量和薄适配入口。
 *
 * 这里只保留已经进入正式运行链的内容。历史辨识、标定和无感结构体已移到
 * reference/legacy_motor，新增算法不再继续堆进本文件。
 */

#ifndef FOC_COMMON_H
#define FOC_COMMON_H

#include <stdbool.h>
#include <stdint.h>

#include "compiler.h"
#include "control_filter.h"
#include "control_pid.h"
#include "control_traj.h"
#include "drive_io.h"
#include "drive_protection.h"
#include "encoder_type.h"
#include "foc.h"
#include "phase_order.h"

#define _RAM_FUNC PLATFORM_FAST_CODE
#define _RAM_DATA PLATFORM_FAST_DATA

/* 正式代码和旧算法对照测试共用的数学常量。 */
#define M_2PI         (6.28318530716f)
#define div_M_2PI     (0.159154943092391467f)
#define SQRT3_BY_2    (0.86602540378f)
#define ONE_BY_SQRT3  (0.57735026919f)
#define SQ(x)         ((x) * (x))

#define wrap_pm_pi(angle)                       \
    do                                           \
    {                                            \
        if ((angle) > (0.5f * M_2PI))           \
        {                                        \
            (angle) -= M_2PI;                    \
        }                                        \
        else if ((angle) < (-0.5f * M_2PI))     \
        {                                        \
            (angle) += M_2PI;                    \
        }                                        \
    } while (0)

#define wrap_0_2pi(angle)                        \
    do                                           \
    {                                            \
        if ((angle) > M_2PI)                     \
        {                                        \
            (angle) -= M_2PI;                    \
        }                                        \
        else if ((angle) < 0.0f)                 \
        {                                        \
            (angle) += M_2PI;                    \
        }                                        \
    } while (0)

/** @brief 当前 FOC 数学链实际执行到哪一级。 */
typedef enum
{
    foc_volt_mode = 0,
    foc_curr_mode,
    foc_vel_mode,
    foc_pos_mode,
} foc_mode_e;

/**
 * @brief 当前采样和由采样计算出的反馈。
 *
 * 实时物理量采用控制领域常用短名：i/v 表示电流/电压，theta/pos/spd 表示
 * 角度/位置/速度，e/r/m 表示电气侧/转子侧/输出侧。电流单位 A、电压单位 V，
 * 角度和位置单位 rad、速度单位 rad/s；字段名不再重复携带单位。
 */
typedef struct
{
    bool i_valid;
    bool vbus_valid;
    bool pos_valid;
    float vbus;
    float inv_vbus;

    int rev;
    int m_rev;
    float enc_pos_r;
    float theta_e;
    float pos_r_1t;
    float pos_r;
    float pos_m;
    float pos_m_1t;
    float pos_diff;
    float pos_last;

    float spd_e;
    float spd_r_raw;
    float spd_r;
    float spd_m;

    float torq_r;
    float torq_r_f;
    float torq_m;
    float torq_m_f;
    float temp_coil;
    float temp_mos;

    float ia;
    float ib;
    float ic;
    float ialpha;
    float ibeta;
    float id;
    float iq;
    float iq_f;
} foc_fb_state_t;

/** @brief 本周期 FOC 计算中间量和输出。 */
typedef struct
{
    foc_mode_e mode;
    float theta;
    float sin;
    float cos;
    float va;
    float vb;
    float vc;
    float valpha;
    float vbeta;
    float vd;
    float vq;
    float duty_a;
    float duty_b;
    float duty_c;
} foc_out_state_t;

typedef enum
{
    DRIVE_REQ_STOP = 0,
    DRIVE_REQ_START,
    DRIVE_REQ_RUN,
} drive_req_e;

typedef enum
{
    DRIVE_STATE_STOP = 0,
    DRIVE_STATE_STARTING,
    DRIVE_STATE_RUN,
    DRIVE_STATE_FAULT,
} drive_state_e;

typedef enum
{
    debug_mode = 0,
    release_mode,
    calibrat_mode,
    halt_mode,
} sys_mode_e;

typedef enum
{
    drag_vf = 0,
    drag_if,         /* 1 */
    volt_op,         /* 2 */
    curr_cl,         /* 3：电流环 */
    spd_curr_cl,     /* 4：速度-电流级联（速度模式） */
    pos_spd_curr_cl, /* 5：位置-速度-电流级联 */
} debug_mode_e;

typedef enum
{
    mit_mode = 0,
    tor_mode,
    vel_mode,
    pos_mode,
    cst_mode,
    csv_mode,
    csp_mode,
} release_mode_e;

typedef enum
{
    quick_mode = 0,
    fault_mode,
} halt_mode_e;

typedef enum
{
    rotor_enc_mod = 0,
    rotor_enc_cali,
    output_enc_mod,
    output_enc_cali,
    iden_pm,
    anticogging_pm,
} calibrat_mode_e;

typedef struct
{
    sys_mode_e sys;
    debug_mode_e debug;
    release_mode_e release;
    calibrat_mode_e calibrat;
    halt_mode_e halt;
} mode_ctrl_e;

/** @brief 位置反馈来源；无感角度只预留类型，当前尚未放行。 */
typedef enum
{
    POSITION_SOURCE_ENCODER = 0,
    POSITION_SOURCE_DUAL_ENCODER,
    POSITION_SOURCE_OBSERVER,
} position_source_t;

typedef enum
{
    OBSERVER_TYPE_NONE = 0,
    OBSERVER_TYPE_NLOB,
    OBSERVER_TYPE_ALOB,
    OBSERVER_TYPE_SCVM,
    OBSERVER_TYPE_ESMO,
    OBSERVER_TYPE_HFSI,
} position_observer_t;

typedef enum
{
    abs_pos_mode = 0,
    rel_pos_mode,
} pos_ctrl_mode_e;

typedef enum
{
    motor_polarity_p = 0,
    motor_polarity_n,
} motor_polarity_e;

/** @brief 当前电机的物理参数和派生换算系数。 */
typedef struct
{
    float rated_voltage;
    float rated_current;
    float rated_speed;
    float rated_torque;
    float rated_power;
    float peak_current;
    float peak_torque;
    float peak_speed;
    phase_order_t phase_order;
    float Rs;
    float Ls;
    float Ld;
    float Lq;
    float Ldif;
    float flux;
    float B;
    float Js;
    float ibw;
    float delta;
    float Kt;
    float div_Kt;
    float e_off;
    float align_current_a; /* 编码器对齐电流 [A]，档案装载 */
    float r_off;
    float m_off;
    float Gr;
    float pn;
    float pnd_2pi;
    float div_Gr;
    float div_pn;
} motor_cfg_t;

/** @brief 当前模式已经应用的参考、轨迹结果和限制。 */
typedef struct
{
    float theta_e;
    float theta_step;
    float vd;
    float vq;
    float id;
    float iq;
    float iq_lim;
    float v_lim;
    float torq_m;
    float torq_r;
    float spd_e;
    float spd_r;
    float spd_r_lim;
    float spd_m;
    float spd_m_ramp;
    float acc_m;
    float dec_m;
    float pos_m;
    float pos_m_ramp;
    float pos_r;
    float pos_m_last;
    float torq_ff;
    float torq_mit;
    float kp;
    float kd;
    float iq_max;
    float iq_min;
    float spd_max;
    float spd_min;
} foc_ref_state_t;

/** @brief 上层可写命令；单位依次为 N·m、rad/s 和 rad。 */
typedef struct
{
    float torq;
    float mit_ff;
    float spd;
    float pos;
    float kp;
    float kd;
} foc_cmd_t;

/** @brief 应用层模式选项、输出轴限制和轨迹状态。 */
typedef struct
{
    pos_ctrl_mode_e pos_ctrl_mode;
    motor_polarity_e polarity;
    float pmax_velm;
    float pmax_posm;
    float pmax_torm;
    float nmax_velm;
    float nmax_posm;
    float nmax_torm;
    bool pos_reached;
    bool vel_reached;
    float abs_pos_ref;
    float rel_pos_ref;
    bool pos_pause;
    bool last_pos_pause;
    float pause_dec;
    float quick_stop_dec;
    bool pos_set_by_flag;
} app_ctrl_t;

/** @brief 快速周期和三级控制环的执行频率、周期及分频计数。 */
typedef struct
{
    uint32_t cur_pid_cnt;
    uint32_t spd_pid_cnt;
    uint32_t pos_pid_cnt;
    float foc_ts;
    float foc_fs;
    float cur_pid_ts;
    float cur_pid_fs;
    float spd_pid_ts;
    float spd_pid_fs;
    float pos_pid_ts;
    float pos_pid_fs;
    uint8_t cur_pid_cnt_val;
    uint8_t spd_pid_cnt_val;
    uint8_t pos_pid_cnt_val;
} ctrl_rate_cfg_t;

/** @brief 一拍 ADC 原始计数。 */
typedef struct
{
    uint16_t ia;
    uint16_t ib;
    uint16_t ic;
    uint16_t vbus;
    uint16_t va;
    uint16_t vb;
    uint16_t vc;
} adc_raw_t;

/** @brief 调试可见的 ADC 原始计数和三相电流零偏。 */
typedef struct
{
    adc_raw_t raw;
    float ia_off;
    float ib_off;
    float ic_off;
} adc_data_t;

/** @brief 当前驱动板 ADC 比例、能力和死区参数。 */
typedef struct
{
    float v_ref;
    float v_adc;
    float i_res;
    float i_op;
    float i_ratio;
    float i_max;
    float v1_res;
    float v2_res;
    float v_op;
    float v_ratio;
    float v_max;
    float dead_time;
} board_cfg_t;

typedef union
{
    struct
    {
        uint32_t ov_curr : 1;
        uint32_t un_volt : 1;
        uint32_t ov_volt : 1;
        uint32_t ov_tmos : 1;
        uint32_t ov_tcoi : 1;
        uint32_t enc_err : 1;
        uint32_t ioff_err : 1;
        uint32_t off_link : 1;
        uint32_t ov_speed : 1;
        uint32_t pwm_err : 1;
    } bit;
    uint32_t all;
} foc_fault_t;

typedef struct
{
    int dir;
    int32_t raw;
    uint8_t rev_flag;
    float pos;
    float pos_last;
    float pos_dif;
    uint32_t cpr;
    uint8_t bit;
    uint8_t shift_bit;
    float factor;
} encoder_data_t;

/** @brief 编码器配置、各器件状态和当前主编码器结果。 */
typedef struct
{
    position_source_t source;
    position_observer_t observer;
    encoder_type_t primary;
    encoder_type_t secondary;
    encoder_data_t ma732;
    encoder_data_t mt6816;
    encoder_data_t abz;
    int32_t raw;
    float pos;
} encoder_state_t;

/** @brief 固件唯一 FOC 实例的配置、状态、反馈和算法上下文。 */
struct foc
{
    uint32_t fast_seq;
    mode_ctrl_e mode;
    drive_req_e req;
    drive_state_e state;
    bool pwm_active;
    drive_pwm_cmd_t pwm_cmd;
    drive_pwm_commit_t pwm_commit;

    board_cfg_t board;
    adc_data_t adc;
    motor_cfg_t motor;
    foc_ref_state_t ref;
    foc_cmd_t cmd;
    app_ctrl_t app;
    foc_fb_state_t fb;
    foc_out_state_t out;
    ctrl_rate_cfg_t rate;
    foc_fault_t fault;
    drive_protection_config_t prot_cfg;
    drive_protection_state_t prot_state;
    encoder_state_t enc;

    /* ABZ 上电自动对齐状态（Drive 层 START 期间执行，对齐完成置 enc_aligned；
     * RAM 语义：复位/掉电清零，STOP/START 不重复对齐）。 */
    bool enc_aligned;
    uint32_t align_ticks;   /* 对齐计时（先保持后取平均） */
    float align_sin_sum;    /* 对齐期 fb.theta_e 圆均值累积 */
    float align_cos_sum;

    pid_para_t id_pi;
    pid_para_t iq_pi;
    pid_para_t spd_pi;
    pid_para_t pos_pi;
    lpf_t iq_lpf;
    lpf_t wr_lpf;
    traj_spd_t spd_traj;
    traj_pos_t pos_traj;
};

void ctrl_fb_update(foc_t *foc, float vbus);
bool foc_adc_sample(foc_t *foc);
void cur_offset_init(foc_t *foc);
void cur_pi_init(foc_t *foc);
void spd_pi_init(foc_t *foc);

bool foc_volt_step(foc_t *foc, float vd_ref, float vq_ref, float angle);
bool foc_cur_step(foc_t *foc, float id_ref, float iq_ref, float angle);
bool foc_spd_step(foc_t *foc, float spd_ref, float cur_lim, float angle);
bool foc_pos_step(foc_t *foc,
                  float pos_ref,
                  float spd_lim,
                  float cur_lim,
                  float angle);

void stop_ramp_step(foc_t *foc, float dec);
void quick_stop_step(foc_t *foc);
void open_volt_step(foc_t *foc);
void open_cur_step(foc_t *foc);
void mit_step(foc_t *foc);
void pv_step(foc_t *foc);
void pp_step(foc_t *foc);
void cst_step(foc_t *foc);
void csv_step(foc_t *foc);
void csp_step(foc_t *foc);

void encoder_init(encoder_state_t *enc);
bool read_mt6816_raw(encoder_data_t *enc);
bool read_abz_raw(encoder_data_t *enc);
bool read_ma732_raw(encoder_data_t *enc);
void encoder_update_angle(encoder_data_t *enc);
bool encoder_sample(encoder_state_t *enc);
bool position_update(foc_t *foc);
void position_update_single_encoder(foc_t *foc);

/* 仅由 tests/legacy 的数值对照实现使用。 */
float sin_f32(float x);
float cos_f32(float x);

#endif /* FOC_COMMON_H */
