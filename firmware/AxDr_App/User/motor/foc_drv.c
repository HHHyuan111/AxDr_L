#include "common.h"
#include "control_filter.h"
#include "control_loop.h"
#include "target_adc.h"

/* 快速控制和电角度差分当前都按 20 kHz 执行。 */
#define FOC_FS_HZ (20000.0f)

_RAM_DATA pmsm_t pm;


/**
***********************************************************************
* @brief:      pmsm_board_init(void)
* @param[in]:  void
* @retval:     void
* @details:    驱动板参数初始化，包括电压、电流、分压电阻，放大倍数等相关参数的设置
***********************************************************************
**/
void pmsm_board_init(void)
{
    pm.board.v_ref = 3.3f;
    pm.board.v_adc = 4096.0f;

    pm.board.i_res = 0.001f;
    pm.board.i_op = 20.0f;

    pm.board.v1_res = 20000.0f; //
    pm.board.v2_res = 1000.0f;

    pm.board.v_op = (pm.board.v1_res + pm.board.v2_res) / pm.board.v2_res;
    pm.board.i_ratio = pm.board.v_ref / pm.board.v_adc / pm.board.i_res / pm.board.i_op;
    pm.board.v_ratio = pm.board.v_ref / pm.board.v_adc * pm.board.v_op;

    pm.board.i_max = pm.board.v_adc * pm.board.i_ratio * 0.5f;
    pm.board.v_max = pm.board.v_adc * pm.board.v_ratio;

    pm.board.Rt_Mos = 10000.0f;
    pm.board.Rt_Mos_res = 10000.0f;
    pm.board.Rt_Mos_Ka = 273.15f;
    pm.board.Rt_Mos_B = 3950.0f;

    pm.board.Rt_rotor = 10000.0f;
    pm.board.Rt_rotor_res = 10000.0f;
    pm.board.Rt_rotor_Ka = 273.15f;
    pm.board.Rt_rotor_B = 3950.0f;

    pm.board.dead_time = 0.5f; //us
}

/**
***********************************************************************
* @brief:      pmsm_protect_init(void)
* @param[in]:  void
* @retval:     void
* @details:    电机保护参数初始化，包括过流、过压、欠压、过温等保护阈值的设置
***********************************************************************
**/
void pmsm_protect_init(void)
{
    const uint32_t delayed_trip_samples = (uint32_t)(0.1f * pm.period.foc_fs);

    pm.prot_cfg = (drive_protection_config_t){
        .under_voltage_v = 15.0f,
        .over_voltage_v = 60.0f,
        .over_current_a = 80.0f,
        .mos_over_temperature_c = 100.0f,
        .coil_over_temperature_c = 100.0f,
        .over_speed_rad_s = pm.para.peak_speed * pm.para.Gr,
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

    drive_protection_reset(&pm.prot_state);
}

/**
***********************************************************************
* @brief:      pmsm_pr60_init(void)
* @param[in]:  void
* @retval:     void
* @details:    PMSM 4310 电机参数初始化，包括极对数、电阻、电感、磁链、转动惯量等参数的设置
***********************************************************************
**/
void pmsm_pr60_init(void)
{
    pm.para.rated_voltage = 24.0f;
    pm.para.rated_current = 0.0f;
    pm.para.rated_speed   = 3000.0f/9.55f;
    pm.para.rated_torque  = 0.8f;
    pm.para.rated_power   = 0.0f;
    pm.para.peak_current  = 0.0f;
    pm.para.peak_torque   = 2.0f;
    pm.para.peak_speed    = 3000.0f/9.55f;

    pm.para.pn = 10;
    pm.para.Rs = 0.162977806f;
    pm.para.Ld = 0.000108778855f;
    pm.para.Lq = 0.000112416135f;
    pm.para.Ls = 0.000110597495f;
    pm.para.Ldif = 3.63728032e-06f;
    pm.para.flux = 0.00498822471f;
    pm.para.B  = 0.000188353f;
    pm.para.Js = 7.32527915e-05f;

    pm.para.Gr = 1.0f;
    pm.para.ibw = 500.0f;
    pm.para.delta = 4.0f;

    pm.para.div_pn = 1.0f / pm.para.pn;
    pm.para.pnd_2pi = pm.para.pn / M_2PI;
    pm.para.div_Gr = 1.0f / pm.para.Gr;
    pm.para.Gref = 1.0f;

    pm.para.Kt = 1.5f * pm.para.pn * pm.para.flux;
    pm.para.div_Kt = 1.0f / pm.para.Kt;

    pm.ctrl.wm_acc = 20.0f;
    pm.ctrl.wm_dec = 20.0f;

    pm.para.phase_order = ABC_PHASE;
    pm.para.e_off = 1.33748674f;
    pm.para.r_off = -0.494569868f;
    pm.para.m_off = 0.0f;

    pm.app_ctrl.pmax_torm =  pm.para.peak_torque*0.8f;
    pm.app_ctrl.nmax_torm = -1.0f*pm.para.peak_torque*0.8f;
    pm.app_ctrl.pmax_velm =  pm.para.peak_speed*0.8f;
    pm.app_ctrl.nmax_velm = -1.0f*pm.para.peak_speed*0.8f;
    pm.app_ctrl.pmax_posm =  20000.0f;
    pm.app_ctrl.nmax_posm = -1.0f*20000.0f;

    pm.ctrl.pmax_tor =  pm.app_ctrl.pmax_torm*pm.para.div_Gr;
    pm.ctrl.nmax_tor =  pm.app_ctrl.nmax_torm*pm.para.div_Gr;
    pm.ctrl.pmax_iq  =  pm.ctrl.pmax_tor*pm.para.div_Kt;
    pm.ctrl.nmax_iq  =  pm.ctrl.nmax_tor*pm.para.div_Kt;

    pm.ctrl.pmax_vel =  pm.app_ctrl.pmax_velm*pm.para.Gr;
    pm.ctrl.nmax_vel =  pm.app_ctrl.nmax_velm*pm.para.Gr;
    pm.ctrl.pmax_pos =  pm.app_ctrl.pmax_posm*pm.para.Gr;
    pm.ctrl.nmax_pos =  pm.app_ctrl.nmax_posm*pm.para.Gr;
}

void pmsm_2312s_init(void)
{
	pm.para.rated_voltage = 24.0f;
    pm.para.rated_current = 0.0f;
    pm.para.rated_speed   = 10000.0f/9.55f;
    pm.para.rated_torque  = 0.8f;
    pm.para.rated_power   = 0.0f;
    pm.para.peak_current  = 0.0f;
    pm.para.peak_torque   = 2.0f;
    pm.para.peak_speed    = 10000.0f/9.55f;
	
    pm.para.pn = 7;
    pm.para.Rs = 0.108945489f;
    pm.para.Ld = 1.97248246e-05f;
    pm.para.Lq = 2.02818483e-05f;
    pm.para.Ls = 2.00033355e-07f;
    pm.para.Ldif = 5.57023668e-07f;
    pm.para.flux = 0.000884152076f;
    pm.para.B  = 0.000188353f;
    pm.para.Js = 2.19904655e-06f;

    pm.para.Gr = 1.0f;
    pm.para.ibw = 500.0f;
    pm.para.delta = 4.0f;

    pm.para.div_pn = 1.0f / pm.para.pn;
    pm.para.pnd_2pi = pm.para.pn / M_2PI;
    pm.para.div_Gr = 1.0f / pm.para.Gr;
    pm.para.Gref = 1.0f;

    pm.para.Kt = 1.5f * pm.para.pn * pm.para.flux;
    pm.para.div_Kt = 1.0f / pm.para.Kt;

    pm.ctrl.wm_acc = 200.0f;
    pm.ctrl.wm_dec = 200.0f;

    pm.para.phase_order = ACB_PHASE;
    pm.para.e_off = 2.34354496f;
    pm.para.r_off = 2.12998796f;
    pm.para.m_off = 0.0f;
	
	pm.app_ctrl.pmax_torm =  pm.para.peak_torque*0.8f;
    pm.app_ctrl.nmax_torm = -1.0f*pm.para.peak_torque*0.8f;
    pm.app_ctrl.pmax_velm =  pm.para.peak_speed*0.8f;
    pm.app_ctrl.nmax_velm = -1.0f*pm.para.peak_speed*0.8f;
    pm.app_ctrl.pmax_posm =  20000.0f;
    pm.app_ctrl.nmax_posm = -1.0f*20000.0f;

    pm.ctrl.pmax_tor =  pm.app_ctrl.pmax_torm*pm.para.div_Gr;
    pm.ctrl.nmax_tor =  pm.app_ctrl.nmax_torm*pm.para.div_Gr;
    pm.ctrl.pmax_iq  =  pm.ctrl.pmax_tor*pm.para.div_Kt;
    pm.ctrl.nmax_iq  =  pm.ctrl.nmax_tor*pm.para.div_Kt;

    pm.ctrl.pmax_vel =  pm.app_ctrl.pmax_velm*pm.para.Gr;
    pm.ctrl.nmax_vel =  pm.app_ctrl.nmax_velm*pm.para.Gr;
    pm.ctrl.pmax_pos =  pm.app_ctrl.pmax_posm*pm.para.Gr;
    pm.ctrl.nmax_pos =  pm.app_ctrl.nmax_posm*pm.para.Gr;
}

/**
***********************************************************************
* @brief:      pmsm_peroid_init(void)
* @param[in]:  void
* @retval:     void
* @details:    电机周期参数初始化，包括FOC、PID等相关周期和采样时间的设置
***********************************************************************
**/
void pmsm_peroid_init(void)
{
    pm.period.foc_fs = FOC_FS_HZ;
    pm.period.foc_ts = 0.00005f;

    pm.period.cur_pid_fs = 20000.0f;
    pm.period.cur_pid_ts = 1.0f/pm.period.cur_pid_fs;
    pm.period.cur_pid_cnt_val = pm.period.foc_fs * pm.period.cur_pid_ts;

    pm.period.spd_pid_fs = 10000.0f;
    pm.period.spd_pid_ts = 1.0f/pm.period.spd_pid_fs;
    pm.period.spd_pid_cnt_val = pm.period.foc_fs * pm.period.spd_pid_ts;

    pm.period.pos_pid_fs = 5000.0f;
    pm.period.pos_pid_ts = 1.0f/pm.period.pos_pid_fs;
    pm.period.pos_pid_cnt_val = pm.period.foc_fs * pm.period.pos_pid_ts;

    pm.period.spd_mea_fs = 1000.0f;
    pm.period.spd_mea_ts = 0.001f;
    pm.period.spd_mea_cnt_val = pm.period.foc_fs * pm.period.spd_mea_ts;
}

/**
***********************************************************************
* @brief:      pmsm_lpf_init(void)
* @param[in]:  void
* @retval:     void
* @details:    低通滤波器参数初始化，包括各通道滤波器的截止频率和采样频率设置，并调用初始化函数
***********************************************************************
**/
void pmsm_lpf_init(void)
{
    pm.id_lpf.fc = 200.0f; // Hz
    pm.id_lpf.fs = 20000.0f;
    pm.iq_lpf.fc = 200.0f; // Hz
    pm.iq_lpf.fs = 20000.0f;
    pm.vd_lpf.fc = 200.0f; // Hz
    pm.vd_lpf.fs = 20000.0f;
    pm.vq_lpf.fc = 200.0f; // Hz
    pm.vq_lpf.fs = 20000.0f;

    pm.ibus_lpf.fc = 200.0f; // Hz
    pm.ibus_lpf.fs = 20000.0f; // Hz
    pm.vbus_lpf.fc = 200.0f;
    pm.vbus_lpf.fs = 20000.0f;

    pm.iabs_lpf.fc = 200.0f; // Hz
    pm.iabs_lpf.fs = 20000.0f; // Hz

    pm.wr_lpf.fc = 200.0f; // Hz
    pm.wr_lpf.fs = 20000.0f; // Hz

    control_lpf_init(&pm.id_lpf);
    control_lpf_init(&pm.iq_lpf);
    control_lpf_init(&pm.vd_lpf);
    control_lpf_init(&pm.vq_lpf);
    control_lpf_init(&pm.ibus_lpf);
    control_lpf_init(&pm.iabs_lpf);
    control_lpf_init(&pm.vbus_lpf);
    control_lpf_init(&pm.wr_lpf);
}

/**
***********************************************************************
* @brief:      pmsm_init(void)
* @param[in]:  void
* @retval:     void
* @details:    PMSM参数及控制器初始化，包括电机、板级、保护、周期、滤波器等参数的设置及相关初始化函数的调用
***********************************************************************
**/
void pmsm_init(void)
{
    memset(&pm, 0, sizeof(pm));
    pmsm_pr60_init();

    pmsm_board_init();
    pmsm_peroid_init();
    pmsm_lpf_init();

    pmsm_protect_init();

    foc_cur_pi_calc(&pm);
    foc_spd_pi_calc(&pm);
	
//	pm.iq_pi.kp = 0;
//	pm.iq_pi.ki = 0;
//	pm.spd_pi.kp = 0;
//	pm.spd_pi.ki = 0;

    pm.pos_pi.kp = 12.0f;//pm.spd_pi.kp * 1.4f * pm.period.pos_pid_fs;//24.0f;

    control_pid_set_limits(&pm.id_pi, 11.0f, -11.0f, 11.0f, -11.0f);
    control_pid_set_limits(&pm.iq_pi, 11.0f, -11.0f, 11.0f, -11.0f);
    control_pid_set_limits(&pm.spd_pi, 20.0f, -20.0f, 20.0f, -20.0f);
    control_pid_set_limits(&pm.pos_pi, 200.0f, -200.0f, 200.0f, -200.0f);
    
    encoder_init(&pm.pos_box);

    /* 辨识、标定、轨迹和无感观测器尚未进入正式运行链，上电时不初始化。 */

    /* 上电默认保持三相功率输出关闭，等待明确的 START 请求。 */
    pm.req = DRIVE_REQ_STOP;
    pm.state = DRIVE_STATE_STOP;
    pm.pwm_active = false;

    pm.pos_box.pos_mode = Sensorsory_s;
    pm.pos_box.sensory1 = MA732;

    /* 默认使用零电流闭环调试；切换模式前仍需显式发送 START 请求。 */
    pm.mode.sys = debug_mode;
    pm.mode.debug = curr_cl;
    pm.mode.release = csv_mode;

    pm.app_ctrl.polarity = motor_polarity_p;
    pm.app_ctrl.pos_ctrl_mode = abs_pos_mode;

    pm.app_ctrl.p_curve = tcurve;
    pm.app_ctrl.v_curve = tcurve;
    pm.app_ctrl.vel_set_immediate = 1;
    pm.app_ctrl.pos_set_immediate = 1;

    foc_get_curr_off();
}


/**
***********************************************************************
* @brief:      foc_cur_pi_calc(pmsm_t* pm)
* @param[in]:  pm 指向 PMSM 参数结构体的指针
* @retval:     void
* @details:    电流环 PI 参数计算，包括 kp、ki、ts 的设置
***********************************************************************
**/
_RAM_FUNC void foc_cur_pi_calc(pmsm_t* pm)
{
    pm->id_pi.kp = pm->para.Ls * pm->para.ibw;
    pm->id_pi.ki = pm->para.Rs * pm->para.ibw;
    pm->id_pi.ts = pm->period.cur_pid_ts;

    pm->iq_pi.kp = pm->para.Ls * pm->para.ibw;
    pm->iq_pi.ki = pm->para.Rs * pm->para.ibw;
    pm->iq_pi.ts = pm->period.cur_pid_ts;
}

/**
***********************************************************************
* @brief:      foc_spd_pi_calc(pmsm_t* pm)
* @param[in]:  pm 指向 PMSM 参数结构体的指针
* @retval:     void
* @details:    速度环 PI 参数计算，包括 kp、ki、ts 的设置
***********************************************************************
**/
_RAM_FUNC void foc_spd_pi_calc(pmsm_t* pm)
{
    pm->spd_pi.kfp = 1.1f;
    pm->spd_pi.kf_damp = 0.25f;

    float K = (3.0f * pm->para.pn * pm->para.flux) / (4.0f * pm->para.Js);

    pm->spd_pi.kp = (pm->iq_pi.kp/pm->para.Ls)/(pm->para.delta*K);
    pm->spd_pi.ki = ((pm->iq_pi.kp/pm->para.Ls)*(pm->iq_pi.kp/pm->para.Ls))/(pm->para.delta*pm->para.delta*pm->para.delta*K);
    pm->spd_pi.ts = pm->period.spd_pid_ts;
}


/**
 * @brief 把现有电机对象映射到可移植控制主链，并将本周期结果写回。
 */
static _RAM_FUNC bool foc_run_control_loop(
    pmsm_t *pm,
    float electrical_angle_rad,
    const control_loop_request_t *request)
{
    control_loop_runtime_t runtime = {
        .current_rate = {
            .count = pm->period.cur_pid_cnt,
            .divider = pm->period.cur_pid_cnt_val,
        },
        .speed_rate = {
            .count = pm->period.spd_pid_cnt,
            .divider = pm->period.spd_pid_cnt_val,
        },
        .position_rate = {
            .count = pm->period.pos_pid_cnt,
            .divider = pm->period.pos_pid_cnt_val,
        },
        .current_d_pid = &pm->id_pi,
        .current_q_pid = &pm->iq_pi,
        .speed_pid = &pm->spd_pi,
        .position_pid = &pm->pos_pi,
        .voltage_d_v = pm->foc.v_d,
        .voltage_q_v = pm->foc.v_q,
        .current_q_ref_a = pm->ctrl.iq_lim,
        .speed_ref_rad_s = pm->ctrl.wr_lim,
    };
    const control_loop_feedback_t feedback = {
        .foc_sample = {
            .i_a = pm->foc.i_a,
            .i_b = pm->foc.i_b,
            .i_c = pm->foc.i_c,
            .theta = electrical_angle_rad,
        },
        .inv_bus_voltage = pm->foc.inv_vbus,
        .rotor_speed_rad_s = pm->foc.wr_f,
        .rotor_position_rad = pm->foc.mp_r,
    };
    control_loop_output_t output;
    const bool duty_valid = control_loop_step(&runtime,
                                               &feedback,
                                               request,
                                               &output);

    pm->period.cur_pid_cnt = runtime.current_rate.count;
    pm->period.spd_pid_cnt = runtime.speed_rate.count;
    pm->period.pos_pid_cnt = runtime.position_rate.count;
    pm->ctrl.iq_lim = runtime.current_q_ref_a;
    pm->ctrl.wr_lim = runtime.speed_ref_rad_s;

    pm->foc.theta = output.frame.theta;
    pm->foc.sin_val = output.frame.sin_theta;
    pm->foc.cos_val = output.frame.cos_theta;
    pm->foc.i_alph = output.frame.i_alpha;
    pm->foc.i_beta = output.frame.i_beta;
    pm->foc.i_d = output.frame.i_d;
    pm->foc.i_q = output.frame.i_q;
    pm->foc.v_d = output.voltage_d_v;
    pm->foc.v_q = output.voltage_q_v;
    pm->foc.v_alph = output.duty.v_alpha;
    pm->foc.v_beta = output.duty.v_beta;
    pm->foc.dtc_a = output.duty.duty_a;
    pm->foc.dtc_b = output.duty.duty_b;
    pm->foc.dtc_c = output.duty.duty_c;

    return duty_valid;
}

/**
***********************************************************************
* @brief:      foc_volt(pmsm_t* pm, float vd_ref, float vq_ref, float pos)
* @param[in]:  pm 指向 PMSM 参数结构体的指针
* @param[in]:  vd_ref d轴电压参考值
* @param[in]:  vq_ref q轴电压参考值
* @param[in]:  pos    电机电气位置（角度/弧度）
* @retval:     true 占空比有效，可提交到 PWM；false 不应更新 PWM
* @details:    电压控制，设置d/q轴电压参考值，完成Clarke、Park变换及SVPWM计算
***********************************************************************
**/
_RAM_FUNC bool foc_volt(pmsm_t* pm, float vd_ref, float vq_ref, float pos)
{
    const control_loop_request_t request = {
        .mode = CONTROL_LOOP_MODE_VOLTAGE,
        .voltage_d_v = vd_ref,
        .voltage_q_v = vq_ref,
    };

    pm->foc.mode = foc_volt_mode;
    return foc_run_control_loop(pm, pos, &request);
}


/**
***********************************************************************
* @brief:      foc_curr(pmsm_t* pm, float id_set, float iq_set, float pos)
* @param[in]:  pm      指向 PMSM 参数结构体的指针
* @param[in]:  id_set  d轴电流设定值
* @param[in]:  iq_set  q轴电流设定值
* @param[in]:  pos     电机电气位置（角度/弧度）
* @retval:     true 占空比有效，可提交到 PWM；false 不应更新 PWM
* @details:    电流环控制，完成Clarke、Park变换、PI调节和SVPWM计算
***********************************************************************
**/
_RAM_FUNC bool foc_curr(pmsm_t* pm, float id_set, float iq_set, float pos)
{
    const control_loop_request_t request = {
        .mode = CONTROL_LOOP_MODE_CURRENT,
        .current_d_ref_a = id_set,
        .current_q_ref_a = iq_set,
    };

    pm->foc.mode = foc_curr_mode;
    return foc_run_control_loop(pm, pos, &request);
}
/**
***********************************************************************
* @brief:      foc_vel(pmsm_t* pm, float vel_set, float iq_lim, float pos)
* @param[in]:  pm      指向 PMSM 参数结构体的指针
* @param[in]:  vel_set 速度设定值
* @param[in]:  iq_set  q轴电流设定值，实则是电流限制值
* @param[in]:  pos     电机电气位置（角度/弧度）
* @retval:     true 占空比有效，可提交到 PWM；false 不应更新 PWM
* @details:    速度环控制，完成Clarke、Park变换、PI调节、电流限制和SVPWM计算
***********************************************************************
**/
_RAM_FUNC bool foc_vel(pmsm_t* pm, float vel_set, float iq_set, float pos)
{
    const control_loop_request_t request = {
        .mode = CONTROL_LOOP_MODE_SPEED,
        .current_d_ref_a = pm->ctrl.id_set,
        .speed_ref_rad_s = vel_set,
        .current_limit_a = iq_set,
    };

    pm->foc.mode = foc_vel_mode;
    return foc_run_control_loop(pm, pos, &request);
}

/**
***********************************************************************
* @brief:      foc_pos(pmsm_t* pm, float pos_set, float vel_lim, float iq_lim, float pos)
* @param[in]:  pm       指向 PMSM 参数结构体的指针
* @param[in]:  pos_set  位置设定值
* @param[in]:  vel_set  速度设定值，实则是速度限制
* @param[in]:  iq_set   q轴电流设定值，实则是电流限制值
* @param[in]:  pos      电机电气位置（角度/弧度）
* @retval:     true 占空比有效，可提交到 PWM；false 不应更新 PWM
* @details:    位置环控制，完成Clarke、Park变换、PI调节、速度/电流限制和SVPWM计算
***********************************************************************
**/
_RAM_FUNC bool foc_pos(pmsm_t* pm, float pos_set, float vel_set, float iq_set, float pos)
{
    const control_loop_request_t request = {
        .mode = CONTROL_LOOP_MODE_POSITION,
        .current_d_ref_a = pm->ctrl.id_set,
        .position_ref_rad = pos_set,
        .current_limit_a = iq_set,
        .speed_limit_rad_s = vel_set,
    };

    pm->foc.mode = foc_pos_mode;
    return foc_run_control_loop(pm, pos, &request);
}

/**
 * @brief 更新当前控制周期使用的 FOC 运行反馈。
 *
 * @param[in,out] pm 电机控制对象。
 *
 * 本函数不直接读取硬件，也不执行电流环。它使用前面已经更新的 ADC 和角度结果，
 * 依次计算母线电压、控制器限幅、转矩和速度。
 */
_RAM_FUNC void foc_feedback_update(pmsm_t *pm, float bus_voltage_v)
{
    /* 第 1 步：保存本周期母线电压，并计算调制所需的电压系数和余量。 */
    pm->foc.vbus = bus_voltage_v;

    if (pm->foc.vbus > 0.0f)
    {
        pm->foc.inv_vbus = 1.5f / pm->foc.vbus;
    }
    else
    {
        pm->foc.inv_vbus = 0.0f;
    }

    pm->foc.vs = pm->foc.vbus*0.5f*0.96f;

    /* 第 2 步：把本周期允许的电压、电流和速度范围交给各级 PI 控制器。 */
    pm->id_pi.out_max    =  pm->foc.vs;
    pm->id_pi.out_min    = -pm->foc.vs;
    pm->iq_pi.out_max    =  pm->foc.vs;
    pm->iq_pi.out_min    = -pm->foc.vs;

    pm->spd_pi.out_max   =  pm->ctrl.pmax_iq;
    pm->spd_pi.out_min   =  pm->ctrl.nmax_iq;

    pm->pos_pi.out_max   =  pm->ctrl.pmax_vel;
    pm->pos_pi.out_min   =  pm->ctrl.nmax_vel;

    /* 第 3 步：滤波 q 轴电流，并换算转子侧和减速器输出侧转矩。 */
    pm->foc.iq_f = control_lpf_step(&pm->iq_lpf, pm->foc.i_q);
    pm->foc.tor_r  = pm->foc.i_q    * pm->para.Kt;
    pm->foc.tor_rf = pm->foc.iq_f   * pm->para.Kt;
    pm->foc.tor_m  = pm->foc.tor_r  * pm->para.Gr;
    pm->foc.tor_mf = pm->foc.tor_rf * pm->para.Gr;

    /* 第 4 步：由电角度差得到电角速度，再换算转子速度和输出轴速度。 */
    pm->foc.we = control_angle_speed_step(&pm->elec_speed_diff,
                                           pm->foc.p_e,
                                           FOC_FS_HZ);
    pm->foc.wr = pm->foc.we * pm->para.div_pn; // rad/s;

    pm->foc.wr_f = control_lpf_step(&pm->wr_lpf, pm->foc.wr);
    pm->foc.wm = pm->foc.wr_f * pm->para.div_Gr; // rad/s

}

/**
 * @brief 采集三路电流采样链的零偏平均值。
 *
 * 电机未进入闭环控制前，本函数连续读取 1000 次原始电流采样值，每次间隔 1 ms，
 * 然后根据 ABC 或 ACB 接线关系，把三条物理采样链的平均值保存为 A、B、C 相零偏。
 */
void foc_get_curr_off(void)
{
    float ia_sum = 0.0f;
    float ib_sum = 0.0f;
    float ic_sum = 0.0f;

    for (int i = 0; i < 1000; i++)
    {
        target_adc_abc_raw_t iabc_raw;

        HAL_Delay(1);
        target_adc_read_iabc_raw(&iabc_raw);

        ia_sum += (float)iabc_raw.a;
        ib_sum += (float)iabc_raw.b;
        ic_sum += (float)iabc_raw.c;
    }

    /* 0.001f 等于 1/1000，把累加值换算成 1000 次采样的平均值。 */
    switch (pm.para.phase_order) {
    case ABC_PHASE:
        pm.adc.ia_off = ia_sum * 0.001f;
        pm.adc.ib_off = ib_sum * 0.001f;
        pm.adc.ic_off = ic_sum * 0.001f;
        break;
    case ACB_PHASE:
        pm.adc.ia_off = ia_sum * 0.001f;
        pm.adc.ib_off = ic_sum * 0.001f;
        pm.adc.ic_off = ib_sum * 0.001f;
        break;
    default:
        /* 保留原有行为：相序无效时不更新三相电流零偏。 */
        break;
    }
}

/**
***********************************************************************
* @brief:      foc_clear(void)
* @param[in]:  void
* @retval:     void
* @details:    FOC相关参数清零，包括控制参数和FOC结构体的重置
***********************************************************************
**/
_RAM_FUNC void foc_clear(pmsm_t* pm)
{
    pm->ctrl.vd_set       = 0.0f;
    pm->ctrl.vq_set       = 0.0f;
    pm->ctrl.id_set       = 0.0f;
    pm->ctrl.iq_set       = 0.0f;
    pm->ctrl.iq_lim       = 0.0f;
    pm->ctrl.torm_set     = 0.0f;
    pm->ctrl.tor_set      = 0.0f;
    pm->ctrl.we_set       = 0.0f;
    pm->ctrl.wr_set       = 0.0f;
    pm->ctrl.wr_lim       = 0.0f;
    pm->ctrl.wm_set       = 0.0f;
    pm->ctrl.wm_ref       = 0.0f;
    pm->ctrl.wm_lim       = 0.0f;
    pm->ctrl.wm_diff      = 0.0f;
    pm->ctrl.posm_set     = 0.0f;
    pm->ctrl.posm_ref     = 0.0f;
    pm->ctrl.posr_set     = 0.0f;
    pm->ctrl.wm_lst       = 0.0f;
    pm->ctrl.posm_lst     = 0.0f;
    pm->ctrl.mit_tor_set  = 0.0f;
    pm->ctrl.kp           = 0.0f;
    pm->ctrl.kd           = 0.0f;
}

// 计算温度的函数
void temp_calc(void)
{
    float mos_ntc_volt  = (pm.adc.Tmos_bc / (float)pm.board.v_adc) * pm.board.v_ref;  // 转换ADC读数为电压
    
    pm.board.Rt_Mos = mos_ntc_volt*pm.board.Rt_Mos_res/(3.3f-mos_ntc_volt);
    pm.foc.Tmos = 1.0f/(1.0f/(pm.board.Rt_Mos_Ka+25.0f) + logf(pm.board.Rt_Mos/pm.board.Rt_Mos_res)/pm.board.Rt_Mos_B) - pm.board.Rt_Mos_Ka + 0.5;

    float rotor_ntc_volt  = (pm.adc.Trotor / (float)pm.board.v_adc) * pm.board.v_ref;  // 转换ADC读数为电压
    
    pm.board.Rt_rotor = rotor_ntc_volt*pm.board.Rt_rotor_res/(3.3f-rotor_ntc_volt);
    pm.foc.Tcoil = 1.0f/(1.0f/(pm.board.Rt_rotor_Ka+25.0f) + logf(pm.board.Rt_rotor/pm.board.Rt_rotor_res)/pm.board.Rt_rotor_B) - pm.board.Rt_rotor_Ka + 0.5;
}

/**
 * @brief 读取本控制周期的原始 ADC 数据并换算三相电流。
 *
 * @param[in,out] pm 电机控制对象，用于保存原始 ADC 值和换算后的三相电流。
 *
 * 函数依次完成三件事：从板级适配层取得原始计数值；根据 ABC/ACB 接线关系
 * 映射到 A、B、C 相；扣除零偏并乘以电流换算系数。采样触发和 DMA 不在这里处理。
 *
 * @return 相序有效并完成电流换算返回 true，否则返回 false。
 */
_RAM_FUNC bool foc_adc_sample(pmsm_t* pm)
{
    target_adc_raw_t adc_raw;

    /* 第 1 步：从板级适配层取得当前已经完成的 ADC 原始结果。 */
    target_adc_read_raw(&adc_raw);

    /* 第 2 步：把板上固定采样链映射为电机逻辑上的 A、B、C 相。 */
    switch (pm->para.phase_order) {
    case ABC_PHASE:
        pm->adc.ia = adc_raw.i.a;
        pm->adc.ib = adc_raw.i.b;
        pm->adc.ic = adc_raw.i.c;

        pm->adc.va = adc_raw.v.a;
        pm->adc.vb = adc_raw.v.b;
        pm->adc.vc = adc_raw.v.c;
        break;
    case ACB_PHASE:
        pm->adc.ia = adc_raw.i.a;
        pm->adc.ic = adc_raw.i.b;
        pm->adc.ib = adc_raw.i.c;

        pm->adc.va = adc_raw.v.a;
        pm->adc.vc = adc_raw.v.b;
        pm->adc.vb = adc_raw.v.c;
        break;
    default:
        return false;
    }

    /* 母线电压不参与相序交换，每个控制周期都直接更新。 */
    pm->adc.vbus = adc_raw.vbus;

    // pm->adc.Trotor   = adc3_seq_buff[4] & 0x0000FFFF;
    // pm->adc.Tmos_ab  = adc3_seq_buff[5] & 0x0000FFFF;
    // pm->adc.Tmos_bc  = adc3_seq_buff[6] & 0x0000FFFF;
    // pm->adc.sin_hall = adc3_seq_buff[7] & 0x0000FFFF;
    // pm->adc.cos_hall = adc3_seq_buff[8] & 0x0000FFFF;

    /* 第 3 步：原始计数减去零偏，再乘以换算系数，得到单位为安培的三相电流。 */
    pm->foc.i_a = ((float) pm->adc.ia - pm->adc.ia_off) * pm->board.i_ratio;
    pm->foc.i_b = ((float) pm->adc.ib - pm->adc.ib_off) * pm->board.i_ratio;
    pm->foc.i_c = ((float) pm->adc.ic - pm->adc.ic_off) * pm->board.i_ratio;

    return true;
}
