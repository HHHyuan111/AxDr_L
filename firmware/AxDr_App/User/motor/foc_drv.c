#include "common.h"
#include "foc_run.h"
#include "motor_profile.h"
#include "speed_adapter.h"

#include <string.h>

#include "algorithm_config.h"
#include "board_adapter.h"
#include "board_config.h"
#include "control_filter.h"
#include "encoder_config.h"
#include "foc_control.h"
#include "drive_diag.h"
#include "motor_config.h"
#include "observer_adapter.h"
#include "target_adc.h"
#include "main.h"

_RAM_DATA foc_t g_foc;


/**
***********************************************************************
* @brief:      board_cfg_init(foc_t *foc)
* @param[in]:  foc FOC 总对象
* @retval:     void
* @details:    驱动板参数初始化，包括电压、电流、分压电阻，放大倍数等相关参数的设置
***********************************************************************
**/
static void board_cfg_init(foc_t *foc)
{
    foc->board.v_ref = DRIVE_ADC_REFERENCE_V;
    foc->board.v_adc = DRIVE_ADC_FULL_SCALE_COUNT;

    foc->board.i_res = DRIVE_CURRENT_SHUNT_OHM;
    foc->board.i_op = DRIVE_CURRENT_AMP_GAIN;

    foc->board.v1_res = DRIVE_VBUS_DIVIDER_HIGH_OHM;
    foc->board.v2_res = DRIVE_VBUS_DIVIDER_LOW_OHM;

    foc->board.v_op = (foc->board.v1_res + foc->board.v2_res) / foc->board.v2_res;
    foc->board.i_ratio = foc->board.v_ref / foc->board.v_adc / foc->board.i_res / foc->board.i_op;
    foc->board.v_ratio = foc->board.v_ref / foc->board.v_adc * foc->board.v_op;

    foc->board.i_max = foc->board.v_adc * foc->board.i_ratio * 0.5f;
    foc->board.v_max = foc->board.v_adc * foc->board.v_ratio;

    foc->board.dead_time = DRIVE_HARDWARE_DEADTIME_US;
}

/**
***********************************************************************
* @brief:      prot_cfg_init(foc_t *foc)
* @param[in]:  foc FOC 总对象
* @retval:     void
* @details:    电机保护参数初始化，包括过流、过压、欠压、过温等保护阈值的设置
***********************************************************************
**/
static void prot_cfg_init(foc_t *foc)
{
    const uint32_t delayed_trip_samples = (uint32_t)(0.1f * foc->rate.foc_fs);

    foc->prot_cfg = (drive_protection_config_t){
        .under_voltage_v = DRIVE_UNDER_VOLTAGE_V,
        .over_voltage_v = DRIVE_OVER_VOLTAGE_V,
        .over_current_a = DRIVE_OVER_CURRENT_A,
        .mos_over_temperature_c = DRIVE_MOS_OVER_TEMPERATURE_C,
        .coil_over_temperature_c = DRIVE_COIL_OVER_TEMPERATURE_C,
        .over_speed_rad_s = foc->motor.peak_speed * foc->motor.Gr,
        .under_voltage_samples = 1U,
        .over_voltage_samples = 1U,
        .over_current_samples = 1U,
        .mos_over_temperature_samples = delayed_trip_samples,
        .coil_over_temperature_samples = delayed_trip_samples,
        .over_speed_samples = delayed_trip_samples,
        .invalid_current_samples = 1U,
        .invalid_bus_voltage_samples = 1U,
        .invalid_position_samples = 3U,
    };

    drive_protection_reset(&foc->prot_state);
}

/**
***********************************************************************
* @brief:      ctrl_rate_init(foc_t *foc)
* @param[in]:  foc FOC 总对象
* @retval:     void
* @details:    电机周期参数初始化，包括FOC、PID等相关周期和采样时间的设置
***********************************************************************
**/
static void ctrl_rate_init(foc_t *foc)
{
    foc->rate.foc_fs = DRIVE_FOC_FREQ_HZ;
    foc->rate.foc_ts = 1.0f / foc->rate.foc_fs;

    foc->rate.cur_pid_fs = foc->rate.foc_fs;
    foc->rate.cur_pid_ts = 1.0f / foc->rate.cur_pid_fs;
    foc->rate.cur_pid_cnt_val = foc->rate.foc_fs * foc->rate.cur_pid_ts;

    foc->rate.spd_pid_fs = CTRL_SPEED_LOOP_FREQ_HZ;
    foc->rate.spd_pid_ts = 1.0f / foc->rate.spd_pid_fs;
    foc->rate.spd_pid_cnt_val = foc->rate.foc_fs * foc->rate.spd_pid_ts;

    foc->rate.pos_pid_fs = CTRL_POSITION_LOOP_FREQ_HZ;
    foc->rate.pos_pid_ts = 1.0f / foc->rate.pos_pid_fs;
    foc->rate.pos_pid_cnt_val = foc->rate.foc_fs * foc->rate.pos_pid_ts;

}

/**
***********************************************************************
* @brief:      ctrl_filter_init(foc_t *foc)
* @param[in]:  foc FOC 总对象
* @retval:     void
* @details:    低通滤波器参数初始化，包括各通道滤波器的截止频率和采样频率设置，并调用初始化函数
***********************************************************************
**/
static void ctrl_filter_init(foc_t *foc)
{
    foc->iq_lpf.fc = CTRL_SIGNAL_FILTER_CUTOFF_HZ;
    foc->iq_lpf.fs = DRIVE_FOC_FREQ_HZ;
    foc->wr_lpf.fc = CTRL_SIGNAL_FILTER_CUTOFF_HZ;
    foc->wr_lpf.fs = DRIVE_FOC_FREQ_HZ;

    control_lpf_init(&foc->iq_lpf);
    control_lpf_init(&foc->wr_lpf);
}

/**
***********************************************************************
* @brief:      foc_init(foc_t *foc)
* @param[in]:  foc FOC 总对象
* @retval:     void
* @details:    PMSM参数及控制器初始化，包括电机、板级、保护、周期、滤波器等参数的设置及相关初始化函数的调用
***********************************************************************
**/
void foc_init(foc_t *foc)
{
    memset(foc, 0, sizeof(*foc));

    motor_profile_load(foc);
    board_cfg_init(foc);
    ctrl_rate_init(foc);
    ctrl_filter_init(foc);

    prot_cfg_init(foc);


    cur_pi_init(foc);
    spd_pi_init(foc);

    control_pid_set_limits(
        &foc->id_pi,
        CTRL_CURRENT_PI_INITIAL_LIMIT_V,
        -CTRL_CURRENT_PI_INITIAL_LIMIT_V,
        CTRL_CURRENT_PI_INITIAL_LIMIT_V,
        -CTRL_CURRENT_PI_INITIAL_LIMIT_V);
    control_pid_set_limits(
        &foc->iq_pi,
        CTRL_CURRENT_PI_INITIAL_LIMIT_V,
        -CTRL_CURRENT_PI_INITIAL_LIMIT_V,
        CTRL_CURRENT_PI_INITIAL_LIMIT_V,
        -CTRL_CURRENT_PI_INITIAL_LIMIT_V);
    control_pid_set_limits(
        &foc->spd_pi,
        CTRL_SPEED_PI_INITIAL_LIMIT_A,
        -CTRL_SPEED_PI_INITIAL_LIMIT_A,
        CTRL_SPEED_PI_INITIAL_LIMIT_A,
        -CTRL_SPEED_PI_INITIAL_LIMIT_A);
    control_pid_set_limits(
        &foc->pos_pi,
        CTRL_POSITION_PI_INITIAL_LIMIT_RAD_S,
        -CTRL_POSITION_PI_INITIAL_LIMIT_RAD_S,
        CTRL_POSITION_PI_INITIAL_LIMIT_RAD_S,
        -CTRL_POSITION_PI_INITIAL_LIMIT_RAD_S);

    /* 档案晚绑定项（沉沙：保护阈值与速度环固化值），须在 prot/spd 初始化后定版。 */
    motor_profile_control_load(foc);
    
    /* 上电默认保持三相功率输出关闭，等待明确的 START 请求。 */
    foc->req = DRIVE_REQ_STOP;
    foc->state = DRIVE_STATE_STOP;
    foc->pwm_active = false;

    foc->enc.source = POSITION_SOURCE_ENCODER;
    foc->enc.primary = ENCODER_SELECTED_TYPE;
    encoder_init(&foc->enc);
    speed_est_init(&foc->speed); /* 清本电机测速历史。 */

    /* 默认使用零电流闭环调试；切换模式前仍需显式发送 START 请求。 */
    foc->mode.sys = debug_mode;
    foc->mode.debug = curr_cl;
    foc->mode.release = csv_mode;

    foc->app.polarity = motor_polarity_p;
    foc->app.pos_ctrl_mode = abs_pos_mode;

    foc->app.pause_dec = foc->ref.dec_m;
    foc->app.quick_stop_dec = foc->ref.dec_m;

    foc->cmd.kp = CTRL_MIT_POSITION_GAIN_NM_PER_RAD;
    foc->cmd.kd = CTRL_MIT_SPEED_GAIN_NM_S_PER_RAD;
    foc->cmd.mit_ff = CTRL_MIT_TORQUE_FEEDFORWARD_NM;

    drive_diag_init(foc);
    obs_init(foc);

    cur_offset_init(foc);
}


/**
***********************************************************************
* @brief:      cur_pi_init(foc_t *foc)
* @param[in]:  foc 指向 PMSM 参数结构体的指针
* @retval:     void
* @details:    电流环 PI 参数计算，包括 kp、ki、ts 的设置
***********************************************************************
**/
_RAM_FUNC void cur_pi_init(foc_t *foc)
{
    foc->id_pi.kp = foc->motor.Ls * foc->motor.ibw;
    foc->id_pi.ki = foc->motor.Rs * foc->motor.ibw;
    foc->id_pi.ts = foc->rate.cur_pid_ts;

    foc->iq_pi.kp = foc->motor.Ls * foc->motor.ibw;
    foc->iq_pi.ki = foc->motor.Rs * foc->motor.ibw;
    foc->iq_pi.ts = foc->rate.cur_pid_ts;
}

/**
***********************************************************************
* @brief:      spd_pi_init(foc_t *foc)
* @param[in]:  foc 指向 PMSM 参数结构体的指针
* @retval:     void
* @details:    速度环 PI 参数计算，包括 kp、ki、ts 的设置
***********************************************************************
**/
_RAM_FUNC void spd_pi_init(foc_t *foc)
{
    float K = (3.0f * foc->motor.pn * foc->motor.flux) / (4.0f * foc->motor.Js);

    foc->spd_pi.kp = (foc->iq_pi.kp/foc->motor.Ls)/(foc->motor.delta*K);
    foc->spd_pi.ki = ((foc->iq_pi.kp/foc->motor.Ls)*(foc->iq_pi.kp/foc->motor.Ls))/(foc->motor.delta*foc->motor.delta*foc->motor.delta*K);
    foc->spd_pi.ts = foc->rate.spd_pid_ts;
}


/**
 * @brief 把现有电机对象映射到可移植控制主链，并将本周期结果写回。
 */
_RAM_FUNC bool foc_execute(
    foc_t *foc,
    const foc_plan_t *cmd,
    const foc_frame_t *frame)
{
    foc_ctrl_t ctrl = {
        .cur_rate = {
            .count = foc->rate.cur_pid_cnt,
            .divider = foc->rate.cur_pid_cnt_val,
        },
        .spd_rate = {
            .count = foc->rate.spd_pid_cnt,
            .divider = foc->rate.spd_pid_cnt_val,
        },
        .pos_rate = {
            .count = foc->rate.pos_pid_cnt,
            .divider = foc->rate.pos_pid_cnt_val,
        },
        .id_pi = &foc->id_pi,
        .iq_pi = &foc->iq_pi,
        .spd_pi = &foc->spd_pi,
        .pos_pi = &foc->pos_pi,
        .vd = foc->out.vd,
        .vq = foc->out.vq,
        .iq_ref = foc->ref.iq_lim,
        .spd_ref = foc->ref.spd_r_lim,
        .deadtime = {
            .enabled = (DRIVE_DEADTIME_COMPENSATION_ENABLE != 0),
            .deadtime_s = foc->board.dead_time * 1.0e-6f,
            .pwm_period_s = foc->rate.foc_ts,
            .current_blend_a = DRIVE_DEADTIME_CURRENT_BLEND_A,
            .polarity = DRIVE_DEADTIME_COMPENSATION_POLARITY,
            .max_duty_delta = DRIVE_DEADTIME_MAX_DUTY_DELTA,
        },
    };
    const foc_fb_t fb = {
        .inv_vbus = foc->fb.inv_vbus,
        .spd = foc->fb.spd_r,
        .pos = foc->fb.pos_r,
    };
    foc_out_t out;
    const bool valid = foc_ctrl_execute(&ctrl, &fb, &cmd->ref, frame, &out);

    foc->out.mode = (foc_mode_e)cmd->ref.mode;

    foc->rate.cur_pid_cnt = ctrl.cur_rate.count;
    foc->rate.spd_pid_cnt = ctrl.spd_rate.count;
    foc->rate.pos_pid_cnt = ctrl.pos_rate.count;
    foc->ref.iq_lim = ctrl.iq_ref;
    foc->ref.spd_r_lim = ctrl.spd_ref;

    foc->out.vd = out.vd;
    foc->out.vq = out.vq;
    foc->out.valpha = out.pwm.valpha;
    foc->out.vbeta = out.pwm.vbeta;
    foc->out.duty_a = out.pwm.duty_a;
    foc->out.duty_b = out.pwm.duty_b;
    foc->out.duty_c = out.pwm.duty_c;

    return valid;
}

/* 电流和角度转换只有这一处；角度由状态/模式在前一步选好。 */
_RAM_FUNC void foc_update_current(foc_t *foc, float theta, foc_frame_t *frame)
{
    const foc_sample_t sample = {
        .ia = foc->fb.ia, .ib = foc->fb.ib, .ic = foc->fb.ic, .theta = theta,
    };
    foc_core_prepare(&sample, frame);
    foc->out.theta = frame->theta;
    foc->out.sin = frame->sin;
    foc->out.cos = frame->cos;
    foc->fb.ialpha = frame->ialpha;
    foc->fb.ibeta = frame->ibeta;
    foc->fb.id = frame->id;
    foc->fb.iq = frame->iq;
}

/**
 * @brief 更新当前控制周期使用的 FOC 运行反馈。
 *
 * @param[in,out] foc 电机控制对象。
 *
 * 本函数不直接读取硬件，也不执行电流环。它使用前面已经更新的 ADC 和角度结果，
 * 依次计算母线电压、控制器限幅、转矩和速度。
 */
_RAM_FUNC void ctrl_fb_update(foc_t *foc, float vbus)
{
    /* 第 1 步：保存本周期母线电压，并计算调制所需的电压系数和余量。 */
    foc->fb.vbus = vbus;

    if (foc->fb.vbus > 0.0f)
    {
        foc->fb.inv_vbus = 1.5f / foc->fb.vbus;
    }
    else
    {
        foc->fb.inv_vbus = 0.0f;
    }

    foc->ref.v_lim = foc->fb.vbus * 0.5f * CTRL_VOLTAGE_UTILIZATION_RATIO;

    /* 第 2 步：把本周期允许的电压、电流和速度范围交给各级 PI 控制器。 */
    foc->id_pi.out_max    =  foc->ref.v_lim;
    foc->id_pi.out_min    = -foc->ref.v_lim;
    foc->iq_pi.out_max    =  foc->ref.v_lim;
    foc->iq_pi.out_min    = -foc->ref.v_lim;

    foc->spd_pi.out_max   =  foc->ref.iq_max;
    foc->spd_pi.out_min   =  foc->ref.iq_min;

    foc->pos_pi.out_max   =  foc->ref.spd_max;
    foc->pos_pi.out_min   =  foc->ref.spd_min;

    /* 第 3 步：滤波 q 轴电流，并换算转子侧和减速器输出侧转矩。 */
    foc->fb.iq_f = control_lpf_step(&foc->iq_lpf, foc->fb.iq);
    foc->fb.torq_r  = foc->fb.iq     * foc->motor.Kt;
    foc->fb.torq_r_f = foc->fb.iq_f   * foc->motor.Kt;
    foc->fb.torq_m  = foc->fb.torq_r  * foc->motor.Gr;
    foc->fb.torq_m_f = foc->fb.torq_r_f * foc->motor.Gr;

    /* 第 4 步：转子侧窗口差分测速（关口①实机验证通过后转正，B 库同构实现）。
     * 旧链为电角度单拍差分（foc->elec_speed_diff），已由 speed_adapter 取代；
     * fb.spd_e 无其他消费者，保留字段由新测速推导以维持可观测性。 */
    {
        float spd_r = speed_est_get(&foc->speed); /* rad/s，转子机械角速度 */
        foc->fb.spd_e = spd_r * foc->motor.pn;
        foc->fb.spd_r_raw = spd_r; /* rad/s */
    }

    foc->fb.spd_r = control_lpf_step(&foc->wr_lpf, foc->fb.spd_r_raw);
    foc->fb.spd_m = foc->fb.spd_r * foc->motor.div_Gr; // rad/s

}

/**
 * @brief 采集三路电流采样链的零偏平均值。
 *
 * 电机未进入闭环控制前，本函数连续读取 1000 次原始电流采样值，每次间隔 1 ms，
 * 然后根据 ABC 或 ACB 接线关系，把三条物理采样链的平均值保存为 A、B、C 相零偏。
 */
void cur_offset_init(foc_t *foc)
{
    float ia_sum = 0.0f;
    float ib_sum = 0.0f;
    float ic_sum = 0.0f;

    for (int i = 0; i < 1000; i++)
    {
        target_adc_abc_raw_t raw;
        target_adc_abc_raw_t phase;

        HAL_Delay(1);
        target_adc_read_iabc_raw(&raw);
        if (!board_phase_map(foc->motor.phase_order, &raw, &phase))
        {
            return;
        }

        ia_sum += (float)phase.a;
        ib_sum += (float)phase.b;
        ic_sum += (float)phase.c;
    }

    /* 0.001f 等于 1/1000，把累加值换算成 1000 次采样的平均值。 */
    foc->adc.ia_off = ia_sum * 0.001f;
    foc->adc.ib_off = ib_sum * 0.001f;
    foc->adc.ic_off = ic_sum * 0.001f;
}
