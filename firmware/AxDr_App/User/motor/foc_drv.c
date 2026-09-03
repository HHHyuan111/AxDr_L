#include "common.h"
#include "algorithm_config.h"
#include "board_adapter.h"
#include "board_config.h"
#include "control_filter.h"
#include "encoder_config.h"
#include "foc_control.h"
#include "drive_diag.h"
#include "motor_config.h"
#include "target_adc.h"

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

    foc->board.Rt_Mos = DRIVE_NTC_NOMINAL_OHM;
    foc->board.Rt_Mos_res = DRIVE_NTC_DIVIDER_OHM;
    foc->board.Rt_Mos_Ka = DRIVE_NTC_ZERO_CELSIUS_K;
    foc->board.Rt_Mos_B = DRIVE_NTC_BETA_K;

    foc->board.Rt_rotor = DRIVE_NTC_NOMINAL_OHM;
    foc->board.Rt_rotor_res = DRIVE_NTC_DIVIDER_OHM;
    foc->board.Rt_rotor_Ka = DRIVE_NTC_ZERO_CELSIUS_K;
    foc->board.Rt_rotor_B = DRIVE_NTC_BETA_K;

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
* @brief:      motor_pr60_init(foc_t *foc)
* @param[in]:  foc FOC 总对象
* @retval:     void
* @details:    PR60 电机参数初始化，包括极对数、电阻、电感、磁链和转动惯量
***********************************************************************
**/
#if MOTOR_SELECTED_MODEL == MOTOR_MODEL_PR60
static void motor_pr60_init(foc_t *foc)
{
    foc->motor.rated_voltage = PR60_RATED_VOLTAGE_V;
    foc->motor.rated_current = PR60_RATED_CURRENT_A;
    foc->motor.rated_speed = PR60_RATED_SPEED_RAD_S;
    foc->motor.rated_torque = PR60_RATED_TORQUE_NM;
    foc->motor.rated_power = PR60_RATED_POWER_W;
    foc->motor.peak_current = PR60_PEAK_CURRENT_A;
    foc->motor.peak_torque = PR60_PEAK_TORQUE_NM;
    foc->motor.peak_speed = PR60_PEAK_SPEED_RAD_S;

    foc->motor.pn = PR60_POLE_PAIRS;
    foc->motor.Rs = PR60_PHASE_RESISTANCE_OHM;
    foc->motor.Ld = PR60_D_AXIS_INDUCTANCE_H;
    foc->motor.Lq = PR60_Q_AXIS_INDUCTANCE_H;
    foc->motor.Ls = PR60_AVERAGE_INDUCTANCE_H;
    foc->motor.Ldif = PR60_DIFFERENTIAL_INDUCTANCE_H;
    foc->motor.flux = PR60_FLUX_LINKAGE_WB;
    foc->motor.B = PR60_VISCOUS_FRICTION_NM_S;
    foc->motor.Js = PR60_INERTIA_KG_M2;

    foc->motor.Gr = PR60_GEAR_RATIO;
    foc->motor.div_pn = 1.0f / foc->motor.pn;
    foc->motor.pnd_2pi = foc->motor.pn / M_2PI;
    foc->motor.div_Gr = 1.0f / foc->motor.Gr;
    foc->motor.Gref = 1.0f;

    foc->motor.Kt = 1.5f * foc->motor.pn * foc->motor.flux;
    foc->motor.div_Kt = 1.0f / foc->motor.Kt;

    foc->ctrl.wm_acc = PR60_ACCELERATION_RAD_S2;
    foc->ctrl.wm_dec = PR60_DECELERATION_RAD_S2;

    foc->motor.phase_order = PR60_PHASE_ORDER;
    foc->motor.e_off = PR60_ELECTRICAL_OFFSET_RAD;
    foc->motor.r_off = PR60_ROTOR_OFFSET_RAD;
    foc->motor.m_off = PR60_MECHANICAL_OFFSET_RAD;

    foc->app.pmax_torm =
        foc->motor.peak_torque * MOTOR_COMMAND_USAGE_RATIO;
    foc->app.nmax_torm = -foc->app.pmax_torm;
    foc->app.pmax_velm =
        foc->motor.peak_speed * MOTOR_COMMAND_USAGE_RATIO;
    foc->app.nmax_velm = -foc->app.pmax_velm;
    foc->app.pmax_posm = MOTOR_MAX_POSITION_RAD;
    foc->app.nmax_posm = -MOTOR_MAX_POSITION_RAD;

    foc->ctrl.pmax_tor =  foc->app.pmax_torm*foc->motor.div_Gr;
    foc->ctrl.nmax_tor =  foc->app.nmax_torm*foc->motor.div_Gr;
    foc->ctrl.pmax_iq  =  foc->ctrl.pmax_tor*foc->motor.div_Kt;
    foc->ctrl.nmax_iq  =  foc->ctrl.nmax_tor*foc->motor.div_Kt;

    foc->ctrl.pmax_vel =  foc->app.pmax_velm*foc->motor.Gr;
    foc->ctrl.nmax_vel =  foc->app.nmax_velm*foc->motor.Gr;
    foc->ctrl.pmax_pos =  foc->app.pmax_posm*foc->motor.Gr;
    foc->ctrl.nmax_pos =  foc->app.nmax_posm*foc->motor.Gr;
}
#endif

#if MOTOR_SELECTED_MODEL == MOTOR_MODEL_2312S
static void motor_2312s_init(foc_t *foc)
{
    foc->motor.rated_voltage = MOTOR_2312S_RATED_VOLTAGE_V;
    foc->motor.rated_current = MOTOR_2312S_RATED_CURRENT_A;
    foc->motor.rated_speed = MOTOR_2312S_RATED_SPEED_RAD_S;
    foc->motor.rated_torque = MOTOR_2312S_RATED_TORQUE_NM;
    foc->motor.rated_power = MOTOR_2312S_RATED_POWER_W;
    foc->motor.peak_current = MOTOR_2312S_PEAK_CURRENT_A;
    foc->motor.peak_torque = MOTOR_2312S_PEAK_TORQUE_NM;
    foc->motor.peak_speed = MOTOR_2312S_PEAK_SPEED_RAD_S;

    foc->motor.pn = MOTOR_2312S_POLE_PAIRS;
    foc->motor.Rs = MOTOR_2312S_PHASE_RESISTANCE_OHM;
    foc->motor.Ld = MOTOR_2312S_D_AXIS_INDUCTANCE_H;
    foc->motor.Lq = MOTOR_2312S_Q_AXIS_INDUCTANCE_H;
    foc->motor.Ls = MOTOR_2312S_AVERAGE_INDUCTANCE_H;
    foc->motor.Ldif = MOTOR_2312S_DIFFERENTIAL_INDUCTANCE_H;
    foc->motor.flux = MOTOR_2312S_FLUX_LINKAGE_WB;
    foc->motor.B = MOTOR_2312S_VISCOUS_FRICTION_NM_S;
    foc->motor.Js = MOTOR_2312S_INERTIA_KG_M2;

    foc->motor.Gr = MOTOR_2312S_GEAR_RATIO;
    foc->motor.div_pn = 1.0f / foc->motor.pn;
    foc->motor.pnd_2pi = foc->motor.pn / M_2PI;
    foc->motor.div_Gr = 1.0f / foc->motor.Gr;
    foc->motor.Gref = 1.0f;

    foc->motor.Kt = 1.5f * foc->motor.pn * foc->motor.flux;
    foc->motor.div_Kt = 1.0f / foc->motor.Kt;

    foc->ctrl.wm_acc = MOTOR_2312S_ACCELERATION_RAD_S2;
    foc->ctrl.wm_dec = MOTOR_2312S_DECELERATION_RAD_S2;

    foc->motor.phase_order = MOTOR_2312S_PHASE_ORDER;
    foc->motor.e_off = MOTOR_2312S_ELECTRICAL_OFFSET_RAD;
    foc->motor.r_off = MOTOR_2312S_ROTOR_OFFSET_RAD;
    foc->motor.m_off = MOTOR_2312S_MECHANICAL_OFFSET_RAD;

    foc->app.pmax_torm =
        foc->motor.peak_torque * MOTOR_COMMAND_USAGE_RATIO;
    foc->app.nmax_torm = -foc->app.pmax_torm;
    foc->app.pmax_velm =
        foc->motor.peak_speed * MOTOR_COMMAND_USAGE_RATIO;
    foc->app.nmax_velm = -foc->app.pmax_velm;
    foc->app.pmax_posm = MOTOR_MAX_POSITION_RAD;
    foc->app.nmax_posm = -MOTOR_MAX_POSITION_RAD;

    foc->ctrl.pmax_tor =  foc->app.pmax_torm*foc->motor.div_Gr;
    foc->ctrl.nmax_tor =  foc->app.nmax_torm*foc->motor.div_Gr;
    foc->ctrl.pmax_iq  =  foc->ctrl.pmax_tor*foc->motor.div_Kt;
    foc->ctrl.nmax_iq  =  foc->ctrl.nmax_tor*foc->motor.div_Kt;

    foc->ctrl.pmax_vel =  foc->app.pmax_velm*foc->motor.Gr;
    foc->ctrl.nmax_vel =  foc->app.nmax_velm*foc->motor.Gr;
    foc->ctrl.pmax_pos =  foc->app.pmax_posm*foc->motor.Gr;
    foc->ctrl.nmax_pos =  foc->app.nmax_posm*foc->motor.Gr;
}
#endif

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

    foc->rate.spd_mea_fs = CTRL_SPEED_MEASURE_FREQ_HZ;
    foc->rate.spd_mea_ts = 1.0f / foc->rate.spd_mea_fs;
    foc->rate.spd_mea_cnt_val = foc->rate.foc_fs * foc->rate.spd_mea_ts;
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
    foc->id_lpf.fc = CTRL_SIGNAL_FILTER_CUTOFF_HZ;
    foc->id_lpf.fs = DRIVE_FOC_FREQ_HZ;
    foc->iq_lpf.fc = CTRL_SIGNAL_FILTER_CUTOFF_HZ;
    foc->iq_lpf.fs = DRIVE_FOC_FREQ_HZ;
    foc->vd_lpf.fc = CTRL_SIGNAL_FILTER_CUTOFF_HZ;
    foc->vd_lpf.fs = DRIVE_FOC_FREQ_HZ;
    foc->vq_lpf.fc = CTRL_SIGNAL_FILTER_CUTOFF_HZ;
    foc->vq_lpf.fs = DRIVE_FOC_FREQ_HZ;

    foc->ibus_lpf.fc = CTRL_SIGNAL_FILTER_CUTOFF_HZ;
    foc->ibus_lpf.fs = DRIVE_FOC_FREQ_HZ;
    foc->vbus_lpf.fc = CTRL_SIGNAL_FILTER_CUTOFF_HZ;
    foc->vbus_lpf.fs = DRIVE_FOC_FREQ_HZ;

    foc->iabs_lpf.fc = CTRL_SIGNAL_FILTER_CUTOFF_HZ;
    foc->iabs_lpf.fs = DRIVE_FOC_FREQ_HZ;

    foc->wr_lpf.fc = CTRL_SIGNAL_FILTER_CUTOFF_HZ;
    foc->wr_lpf.fs = DRIVE_FOC_FREQ_HZ;

    control_lpf_init(&foc->id_lpf);
    control_lpf_init(&foc->iq_lpf);
    control_lpf_init(&foc->vd_lpf);
    control_lpf_init(&foc->vq_lpf);
    control_lpf_init(&foc->ibus_lpf);
    control_lpf_init(&foc->iabs_lpf);
    control_lpf_init(&foc->vbus_lpf);
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

#if MOTOR_SELECTED_MODEL == MOTOR_MODEL_PR60
    motor_pr60_init(foc);
#elif MOTOR_SELECTED_MODEL == MOTOR_MODEL_2312S
    motor_2312s_init(foc);
#else
#error "Unsupported MOTOR_SELECTED_MODEL"
#endif

    board_cfg_init(foc);
    ctrl_rate_init(foc);
    ctrl_filter_init(foc);

    prot_cfg_init(foc);

#if MOTOR_SELECTED_MODEL == MOTOR_MODEL_PR60
    foc->motor.ibw = CTRL_PR60_CURRENT_BANDWIDTH_RAD_S;
    foc->motor.delta = CTRL_PR60_SPEED_DAMPING_RATIO;
    foc->spd_pi.kfp = CTRL_PR60_SPEED_REFERENCE_WEIGHT;
    foc->spd_pi.kf_damp = CTRL_PR60_SPEED_FEEDBACK_DAMPING;
    foc->pos_pi.kp = CTRL_PR60_POSITION_KP;
    foc->pos_pi.ki = CTRL_PR60_POSITION_KI;
    foc->pos_pi.kd = CTRL_PR60_POSITION_KD;
#elif MOTOR_SELECTED_MODEL == MOTOR_MODEL_2312S
    foc->motor.ibw = CTRL_2312S_CURRENT_BANDWIDTH_RAD_S;
    foc->motor.delta = CTRL_2312S_SPEED_DAMPING_RATIO;
    foc->spd_pi.kfp = CTRL_2312S_SPEED_REFERENCE_WEIGHT;
    foc->spd_pi.kf_damp = CTRL_2312S_SPEED_FEEDBACK_DAMPING;
    foc->pos_pi.kp = CTRL_2312S_POSITION_KP;
    foc->pos_pi.ki = CTRL_2312S_POSITION_KI;
    foc->pos_pi.kd = CTRL_2312S_POSITION_KD;
#endif

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
    
    encoder_init(&foc->enc);
    drive_diag_init(foc);

    /* 辨识、标定、轨迹和无感观测器尚未进入正式运行链，上电时不初始化。 */

    /* 上电默认保持三相功率输出关闭，等待明确的 START 请求。 */
    foc->req = DRIVE_REQ_STOP;
    foc->state = DRIVE_STATE_STOP;
    foc->pwm_active = false;

    foc->enc.pos_mode = Sensorsory_s;
    foc->enc.sensory1 = ENCODER_SELECTED_TYPE;

    /* 默认使用零电流闭环调试；切换模式前仍需显式发送 START 请求。 */
    foc->mode.sys = debug_mode;
    foc->mode.debug = curr_cl;
    foc->mode.release = csv_mode;

    foc->app.polarity = motor_polarity_p;
    foc->app.pos_ctrl_mode = abs_pos_mode;

    foc->app.p_curve = tcurve;
    foc->app.v_curve = tcurve;
    foc->app.vel_set_immediate = 1;
    foc->app.pos_set_immediate = 1;

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
static _RAM_FUNC bool foc_ctrl_run(
    foc_t *foc,
    float electrical_angle_rad,
    const foc_ref_t *ref)
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
        .v_d = foc->sig.v_d,
        .v_q = foc->sig.v_q,
        .i_q_ref = foc->ctrl.iq_lim,
        .spd_ref = foc->ctrl.wr_lim,
    };
    const foc_fb_t fb = {
        .sample = {
            .i_a = foc->sig.i_a,
            .i_b = foc->sig.i_b,
            .i_c = foc->sig.i_c,
            .theta = electrical_angle_rad,
        },
        .inv_v_bus = foc->sig.inv_vbus,
        .spd = foc->sig.wr_f,
        .pos = foc->sig.mp_r,
    };
    foc_out_t out;
    const bool valid = foc_ctrl_step(&ctrl, &fb, ref, &out);

    foc->rate.cur_pid_cnt = ctrl.cur_rate.count;
    foc->rate.spd_pid_cnt = ctrl.spd_rate.count;
    foc->rate.pos_pid_cnt = ctrl.pos_rate.count;
    foc->ctrl.iq_lim = ctrl.i_q_ref;
    foc->ctrl.wr_lim = ctrl.spd_ref;

    foc->sig.theta = out.frame.theta;
    foc->sig.sin_val = out.frame.sin_theta;
    foc->sig.cos_val = out.frame.cos_theta;
    foc->sig.i_alph = out.frame.i_alpha;
    foc->sig.i_beta = out.frame.i_beta;
    foc->sig.i_d = out.frame.i_d;
    foc->sig.i_q = out.frame.i_q;
    foc->sig.v_d = out.v_d;
    foc->sig.v_q = out.v_q;
    foc->sig.v_alph = out.pwm.v_alpha;
    foc->sig.v_beta = out.pwm.v_beta;
    foc->sig.dtc_a = out.pwm.duty_a;
    foc->sig.dtc_b = out.pwm.duty_b;
    foc->sig.dtc_c = out.pwm.duty_c;

    return valid;
}

/**
***********************************************************************
* @brief:      foc_volt_step(foc_t *foc, float v_d_ref, float v_q_ref, float angle)
* @param[in]:  foc 指向 PMSM 参数结构体的指针
* @param[in]:  vd_ref d轴电压参考值
* @param[in]:  vq_ref q轴电压参考值
* @param[in]:  pos    电机电气位置（角度/弧度）
* @retval:     true 占空比有效，可提交到 PWM；false 不应更新 PWM
* @details:    电压控制，设置d/q轴电压参考值，完成Clarke、Park变换及SVPWM计算
***********************************************************************
**/
_RAM_FUNC bool foc_volt_step(foc_t *foc, float v_d_ref, float v_q_ref, float angle)
{
    const foc_ref_t ref = {
        .mode = FOC_CTRL_MODE_VOLT,
        .v_d = v_d_ref,
        .v_q = v_q_ref,
    };

    foc->sig.mode = foc_volt_mode;
    return foc_ctrl_run(foc, angle, &ref);
}


/**
***********************************************************************
* @brief:      foc_cur_step(foc_t *foc, float i_d_ref, float i_q_ref, float angle)
* @param[in]:  foc      指向 PMSM 参数结构体的指针
* @param[in]:  id_set  d轴电流设定值
* @param[in]:  iq_set  q轴电流设定值
* @param[in]:  pos     电机电气位置（角度/弧度）
* @retval:     true 占空比有效，可提交到 PWM；false 不应更新 PWM
* @details:    电流环控制，完成Clarke、Park变换、PI调节和SVPWM计算
***********************************************************************
**/
_RAM_FUNC bool foc_cur_step(foc_t *foc, float i_d_ref, float i_q_ref, float angle)
{
    const foc_ref_t ref = {
        .mode = FOC_CTRL_MODE_CUR,
        .i_d_ref = i_d_ref,
        .i_q_ref = i_q_ref,
    };

    foc->sig.mode = foc_curr_mode;
    return foc_ctrl_run(foc, angle, &ref);
}
/**
***********************************************************************
* @brief:      foc_spd_step(foc_t *foc, float spd_ref, float cur_lim, float angle)
* @param[in]:  foc      指向 PMSM 参数结构体的指针
* @param[in]:  vel_set 速度设定值
* @param[in]:  iq_set  q轴电流设定值，实则是电流限制值
* @param[in]:  pos     电机电气位置（角度/弧度）
* @retval:     true 占空比有效，可提交到 PWM；false 不应更新 PWM
* @details:    速度环控制，完成Clarke、Park变换、PI调节、电流限制和SVPWM计算
***********************************************************************
**/
_RAM_FUNC bool foc_spd_step(foc_t *foc, float spd_ref, float cur_lim, float angle)
{
    const foc_ref_t ref = {
        .mode = FOC_CTRL_MODE_SPD,
        .i_d_ref = foc->ctrl.id_set,
        .spd_ref = spd_ref,
        .cur_lim = cur_lim,
    };

    foc->sig.mode = foc_vel_mode;
    return foc_ctrl_run(foc, angle, &ref);
}

/**
***********************************************************************
* @brief:      foc_pos_step(foc_t *foc, float pos_ref, float spd_lim, float cur_lim, float angle)
* @param[in]:  foc       指向 PMSM 参数结构体的指针
* @param[in]:  pos_set  位置设定值
* @param[in]:  vel_set  速度设定值，实则是速度限制
* @param[in]:  iq_set   q轴电流设定值，实则是电流限制值
* @param[in]:  pos      电机电气位置（角度/弧度）
* @retval:     true 占空比有效，可提交到 PWM；false 不应更新 PWM
* @details:    位置环控制，完成Clarke、Park变换、PI调节、速度/电流限制和SVPWM计算
***********************************************************************
**/
_RAM_FUNC bool foc_pos_step(foc_t *foc,
                            float pos_ref,
                            float spd_lim,
                            float cur_lim,
                            float angle)
{
    const foc_ref_t ref = {
        .mode = FOC_CTRL_MODE_POS,
        .i_d_ref = foc->ctrl.id_set,
        .pos_ref = pos_ref,
        .cur_lim = cur_lim,
        .spd_lim = spd_lim,
    };

    foc->sig.mode = foc_pos_mode;
    return foc_ctrl_run(foc, angle, &ref);
}

/**
 * @brief 更新当前控制周期使用的 FOC 运行反馈。
 *
 * @param[in,out] foc 电机控制对象。
 *
 * 本函数不直接读取硬件，也不执行电流环。它使用前面已经更新的 ADC 和角度结果，
 * 依次计算母线电压、控制器限幅、转矩和速度。
 */
_RAM_FUNC void ctrl_fb_update(foc_t *foc, float bus_voltage_v)
{
    /* 第 1 步：保存本周期母线电压，并计算调制所需的电压系数和余量。 */
    foc->sig.vbus = bus_voltage_v;

    if (foc->sig.vbus > 0.0f)
    {
        foc->sig.inv_vbus = 1.5f / foc->sig.vbus;
    }
    else
    {
        foc->sig.inv_vbus = 0.0f;
    }

    foc->sig.vs = foc->sig.vbus * 0.5f * CTRL_VOLTAGE_UTILIZATION_RATIO;

    /* 第 2 步：把本周期允许的电压、电流和速度范围交给各级 PI 控制器。 */
    foc->id_pi.out_max    =  foc->sig.vs;
    foc->id_pi.out_min    = -foc->sig.vs;
    foc->iq_pi.out_max    =  foc->sig.vs;
    foc->iq_pi.out_min    = -foc->sig.vs;

    foc->spd_pi.out_max   =  foc->ctrl.pmax_iq;
    foc->spd_pi.out_min   =  foc->ctrl.nmax_iq;

    foc->pos_pi.out_max   =  foc->ctrl.pmax_vel;
    foc->pos_pi.out_min   =  foc->ctrl.nmax_vel;

    /* 第 3 步：滤波 q 轴电流，并换算转子侧和减速器输出侧转矩。 */
    foc->sig.iq_f = control_lpf_step(&foc->iq_lpf, foc->sig.i_q);
    foc->sig.tor_r  = foc->sig.i_q    * foc->motor.Kt;
    foc->sig.tor_rf = foc->sig.iq_f   * foc->motor.Kt;
    foc->sig.tor_m  = foc->sig.tor_r  * foc->motor.Gr;
    foc->sig.tor_mf = foc->sig.tor_rf * foc->motor.Gr;

    /* 第 4 步：由电角度差得到电角速度，再换算转子速度和输出轴速度。 */
    foc->sig.we = control_angle_speed_step(&foc->elec_speed_diff,
                                           foc->sig.p_e,
                                           DRIVE_FOC_FREQ_HZ);
    foc->sig.wr = foc->sig.we * foc->motor.div_pn; // rad/s;

    foc->sig.wr_f = control_lpf_step(&foc->wr_lpf, foc->sig.wr);
    foc->sig.wm = foc->sig.wr_f * foc->motor.div_Gr; // rad/s

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

/**
***********************************************************************
* @brief:      foc_clear(void)
* @param[in]:  void
* @retval:     void
* @details:    FOC相关参数清零，包括控制参数和FOC结构体的重置
***********************************************************************
**/
_RAM_FUNC void foc_clear(foc_t *foc)
{
    foc->ctrl.vd_set       = 0.0f;
    foc->ctrl.vq_set       = 0.0f;
    foc->ctrl.id_set       = 0.0f;
    foc->ctrl.iq_set       = 0.0f;
    foc->ctrl.iq_lim       = 0.0f;
    foc->ctrl.torm_set     = 0.0f;
    foc->ctrl.tor_set      = 0.0f;
    foc->ctrl.we_set       = 0.0f;
    foc->ctrl.wr_set       = 0.0f;
    foc->ctrl.wr_lim       = 0.0f;
    foc->ctrl.wm_set       = 0.0f;
    foc->ctrl.wm_ref       = 0.0f;
    foc->ctrl.wm_lim       = 0.0f;
    foc->ctrl.wm_diff      = 0.0f;
    foc->ctrl.posm_set     = 0.0f;
    foc->ctrl.posm_ref     = 0.0f;
    foc->ctrl.posr_set     = 0.0f;
    foc->ctrl.wm_lst       = 0.0f;
    foc->ctrl.posm_lst     = 0.0f;
    foc->ctrl.mit_tor_set  = 0.0f;
    foc->ctrl.kp           = 0.0f;
    foc->ctrl.kd           = 0.0f;
}

// 计算温度的函数
void temp_update(foc_t *foc)
{
    float mos_ntc_volt  = (foc->adc.Tmos_bc / (float)foc->board.v_adc) * foc->board.v_ref;  // 转换ADC读数为电压
    
    foc->board.Rt_Mos = mos_ntc_volt*foc->board.Rt_Mos_res/(3.3f-mos_ntc_volt);
    foc->sig.Tmos = 1.0f/(1.0f/(foc->board.Rt_Mos_Ka+25.0f) + logf(foc->board.Rt_Mos/foc->board.Rt_Mos_res)/foc->board.Rt_Mos_B) - foc->board.Rt_Mos_Ka + 0.5;

    float rotor_ntc_volt  = (foc->adc.Trotor / (float)foc->board.v_adc) * foc->board.v_ref;  // 转换ADC读数为电压
    
    foc->board.Rt_rotor = rotor_ntc_volt*foc->board.Rt_rotor_res/(3.3f-rotor_ntc_volt);
    foc->sig.Tcoil = 1.0f/(1.0f/(foc->board.Rt_rotor_Ka+25.0f) + logf(foc->board.Rt_rotor/foc->board.Rt_rotor_res)/foc->board.Rt_rotor_B) - foc->board.Rt_rotor_Ka + 0.5;
}

/**
 * @brief 读取本控制周期的原始 ADC 数据并换算三相电流。
 *
 * @param[in,out] foc 电机控制对象，用于保存原始 ADC 值和换算后的三相电流。
 *
 * 函数依次完成三件事：从板级适配层取得原始计数值；根据 ABC/ACB 接线关系
 * 映射到 A、B、C 相；扣除零偏并乘以电流换算系数。采样触发和 DMA 不在这里处理。
 *
 * @return 相序有效并完成电流换算返回 true，否则返回 false。
 */
_RAM_FUNC bool foc_adc_sample(foc_t *foc)
{
    const board_adc_cfg_t cfg = {
        .phase_order = foc->motor.phase_order,
        .i_scale = foc->board.i_ratio,
        .v_scale = foc->board.v_ratio,
        .i_offset_a = foc->adc.ia_off,
        .i_offset_b = foc->adc.ib_off,
        .i_offset_c = foc->adc.ic_off,
    };
    target_adc_raw_t raw;
    board_sample_t sample;

    /* 第 1 步：从板级适配层取得当前已经完成的 ADC 原始结果。 */
    target_adc_read_raw(&raw);

    /* 第 2 步：适配层统一完成相序映射、零偏扣除和物理量换算。 */
    if (!board_adc_convert(&cfg, &raw, &sample))
    {
        return false;
    }

    foc->adc.ia = sample.i_raw.a;
    foc->adc.ib = sample.i_raw.b;
    foc->adc.ic = sample.i_raw.c;
    foc->adc.va = sample.v_raw.a;
    foc->adc.vb = sample.v_raw.b;
    foc->adc.vc = sample.v_raw.c;
    foc->adc.vbus = sample.v_bus_raw;

    // foc->adc.Trotor   = adc3_seq_buff[4] & 0x0000FFFF;
    // foc->adc.Tmos_ab  = adc3_seq_buff[5] & 0x0000FFFF;
    // foc->adc.Tmos_bc  = adc3_seq_buff[6] & 0x0000FFFF;
    // foc->adc.sin_hall = adc3_seq_buff[7] & 0x0000FFFF;
    // foc->adc.cos_hall = adc3_seq_buff[8] & 0x0000FFFF;

    foc->sig.i_a = sample.i_a;
    foc->sig.i_b = sample.i_b;
    foc->sig.i_c = sample.i_c;
    foc->sig.vbus = sample.v_bus;

    return true;
}
