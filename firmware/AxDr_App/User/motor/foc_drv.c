#include "common.h"
#include "control_filter.h"
#include "control_limit.h"
#include "foc_svm.h"
#include "foc_transform.h"
#include "target_adc.h"
#include "target_pwm.h"

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
    pm.protect.oc_value = 80.0f; // A
    pm.protect.ov_value = 60.0f; // V
    pm.protect.uv_value = 15.0f; // V
    pm.protect.ot_value = 100.0f; //
    pm.protect.omt_value = 100.0f; //
    pm.protect.time_value = 0; // 50us

    pm.protect.oc_time = 0.1f; // s
    pm.protect.ov_time = 0.1f; // s
    pm.protect.uv_time = 0.1f; // s
    pm.protect.ot_time = 0.1f; // s
    pm.protect.omt_time = 0.1f; // s
    pm.protect.link_out_time = 0.1f; // s

    pm.protect.oc_cnt_value = pm.protect.oc_time * pm.period.foc_fs;
    pm.protect.ov_cnt_value = pm.protect.ov_time * pm.period.foc_fs;
    pm.protect.uv_cnt_value = pm.protect.uv_time * pm.period.foc_fs;
    pm.protect.ot_cnt_value = pm.protect.ot_time * pm.period.foc_fs;
    pm.protect.omt_cnt_value = pm.protect.omt_time * pm.period.foc_fs;
    pm.protect.link_out_cnt_value = pm.protect.link_out_time * pm.period.foc_fs;
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
    pmsm_2312s_init();

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
    
    iden_init();
    scvm_init();
    nlob_init();
    alob_init();
    encoder_init(&pm.pos_box);
    cali_init();
    traj_init();

    eh_observer_init();

    /* 上电默认保持三相功率输出关闭，等待明确的 START 请求。 */
    pm.req = DRIVE_REQ_STOP;
    pm.state = DRIVE_STATE_STOP;
    pm.pwm_active = false;

    pm.pos_box.pos_mode = Sensorsory_s; //Sensorsory_s; Sensorsory_d; //Sensorless;
    pm.pos_box.sensory1 = MT6816; //DMENC; //MT6825; //MT6816; //MA732; // DMENC; //Hall; //Xhall;
    pm.pos_box.senless  = Scvm;       //Nlob; //Alob; //Scvm; //Esmo; //Hfsi;

    pm.mode.sys      = debug_mode;        //debug_mode; //release_mode; // calibrat_mode;
    pm.mode.debug    = curr_cl;   //drag_vf; //volt_op; //drag_if; //curr_cl; //spd_curr_cl; //pos_spd_curr_cl;
    pm.mode.release  = vel_mode;        // mit_mode; //tor_mode; //vel_mode; //pos_mode;
    pm.mode.calibrat = iden_pm; // rotor_enc_cali; //iden_pm; //enc_mod;

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
	pm->foc.mode = foc_volt_mode;
    foc_clarke(pm->foc.i_a,
               pm->foc.i_b,
               pm->foc.i_c,
               &pm->foc.i_alph,
               &pm->foc.i_beta);
    pm->foc.theta = pos;
    wrap_0_2pi(pm->foc.theta);
    foc_sin_cos(pm->foc.theta, &pm->foc.sin_val, &pm->foc.cos_val);
    foc_park(pm->foc.i_alph,
             pm->foc.i_beta,
             pm->foc.sin_val,
             pm->foc.cos_val,
             &pm->foc.i_d,
             &pm->foc.i_q);
    pm->foc.v_d = vd_ref;
    pm->foc.v_q = vq_ref;
    foc_inv_park(pm->foc.v_d,
                 pm->foc.v_q,
                 pm->foc.sin_val,
                 pm->foc.cos_val,
                 &pm->foc.v_alph,
                 &pm->foc.v_beta);

    return foc_svm(pm->foc.v_alph * pm->foc.inv_vbus,
                   pm->foc.v_beta * pm->foc.inv_vbus,
                   &pm->foc.dtc_a,
                   &pm->foc.dtc_b,
                   &pm->foc.dtc_c) == 0;
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
	pm->foc.mode = foc_curr_mode;
    foc_clarke(pm->foc.i_a,
               pm->foc.i_b,
               pm->foc.i_c,
               &pm->foc.i_alph,
               &pm->foc.i_beta);
    pm->foc.theta = pos;
    wrap_0_2pi(pm->foc.theta);
    foc_sin_cos(pm->foc.theta, &pm->foc.sin_val, &pm->foc.cos_val);

    foc_park(pm->foc.i_alph,
             pm->foc.i_beta,
             pm->foc.sin_val,
             pm->foc.cos_val,
             &pm->foc.i_d,
             &pm->foc.i_q);

    if (++pm->period.cur_pid_cnt >= pm->period.cur_pid_cnt_val)
    {
        control_pid_parallel_step(&pm->id_pi, id_set, pm->foc.i_d);
        pm->foc.v_d = pm->id_pi.out_value;
        control_pid_parallel_step(&pm->iq_pi, iq_set, pm->foc.i_q);
        pm->foc.v_q = pm->iq_pi.out_value;
    }

    foc_inv_park(pm->foc.v_d,
                 pm->foc.v_q,
                 pm->foc.sin_val,
                 pm->foc.cos_val,
                 &pm->foc.v_alph,
                 &pm->foc.v_beta);

    return foc_svm(pm->foc.v_alph * pm->foc.inv_vbus,
                   pm->foc.v_beta * pm->foc.inv_vbus,
                   &pm->foc.dtc_a,
                   &pm->foc.dtc_b,
                   &pm->foc.dtc_c) == 0;
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
	pm->foc.mode = foc_vel_mode;
    foc_clarke(pm->foc.i_a,
               pm->foc.i_b,
               pm->foc.i_c,
               &pm->foc.i_alph,
               &pm->foc.i_beta);
    pm->foc.theta = pos;
    wrap_0_2pi(pm->foc.theta);
    foc_sin_cos(pm->foc.theta, &pm->foc.sin_val, &pm->foc.cos_val);
    foc_park(pm->foc.i_alph,
             pm->foc.i_beta,
             pm->foc.sin_val,
             pm->foc.cos_val,
             &pm->foc.i_d,
             &pm->foc.i_q);

    if (++pm->period.spd_pid_cnt >= pm->period.spd_pid_cnt_val)
    {
        pm->period.spd_pid_cnt = 0;
        control_pid_pdff_step(&pm->spd_pi, vel_set, pm->foc.wr_f);
        pm->ctrl.iq_lim = pm->spd_pi.out_value;

        if (ABS(iq_set) > 0)
        {
            pm->ctrl.iq_lim = control_limit(pm->ctrl.iq_lim, ABS(iq_set), -ABS(iq_set));
        }
    }

    if (++pm->period.cur_pid_cnt >= pm->period.cur_pid_cnt_val)
    {
        control_pid_parallel_step(&pm->id_pi, pm->ctrl.id_set, pm->foc.i_d);
        pm->foc.v_d = pm->id_pi.out_value;
        control_pid_parallel_step(&pm->iq_pi, pm->ctrl.iq_lim, pm->foc.i_q);
        pm->foc.v_q = pm->iq_pi.out_value;
    }

    foc_inv_park(pm->foc.v_d,
                 pm->foc.v_q,
                 pm->foc.sin_val,
                 pm->foc.cos_val,
                 &pm->foc.v_alph,
                 &pm->foc.v_beta);

    return foc_svm(pm->foc.v_alph * pm->foc.inv_vbus,
                   pm->foc.v_beta * pm->foc.inv_vbus,
                   &pm->foc.dtc_a,
                   &pm->foc.dtc_b,
                   &pm->foc.dtc_c) == 0;
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
	pm->foc.mode = foc_pos_mode;
    foc_clarke(pm->foc.i_a,
               pm->foc.i_b,
               pm->foc.i_c,
               &pm->foc.i_alph,
               &pm->foc.i_beta);
    pm->foc.theta = pos;
    wrap_0_2pi(pm->foc.theta);
    foc_sin_cos(pm->foc.theta, &pm->foc.sin_val, &pm->foc.cos_val);
    foc_park(pm->foc.i_alph,
             pm->foc.i_beta,
             pm->foc.sin_val,
             pm->foc.cos_val,
             &pm->foc.i_d,
             &pm->foc.i_q);

    if (++pm->period.pos_pid_cnt >= pm->period.pos_pid_cnt_val)
    {
        pm->period.pos_pid_cnt = 0;
        control_pid_parallel_step(&pm->pos_pi, pos_set, pm->foc.mp_r);
        pm->ctrl.wr_lim = pm->pos_pi.out_value;

        if (ABS(vel_set) > 0)
        {
            pm->ctrl.wr_lim = control_limit(pm->ctrl.wr_lim, ABS(vel_set), -ABS(vel_set));
        }
    }

    if (++pm->period.spd_pid_cnt >= pm->period.spd_pid_cnt_val)
    {
        pm->period.spd_pid_cnt = 0;
        control_pid_pdff_step(&pm->spd_pi, pm->ctrl.wr_lim, pm->foc.wr_f);
        pm->ctrl.iq_lim = pm->spd_pi.out_value;

        if (ABS(iq_set) > 0)
        {
            pm->ctrl.iq_lim = control_limit(pm->ctrl.iq_lim, ABS(iq_set), -ABS(iq_set));
        }
    }

    if (++pm->period.cur_pid_cnt >= pm->period.cur_pid_cnt_val)
    {
        control_pid_parallel_step(&pm->id_pi, pm->ctrl.id_set, pm->foc.i_d);
        pm->foc.v_d = pm->id_pi.out_value;
        control_pid_parallel_step(&pm->iq_pi, pm->ctrl.iq_lim, pm->foc.i_q);
        pm->foc.v_q = pm->iq_pi.out_value;
    }

    foc_inv_park(pm->foc.v_d,
                 pm->foc.v_q,
                 pm->foc.sin_val,
                 pm->foc.cos_val,
                 &pm->foc.v_alph,
                 &pm->foc.v_beta);

    return foc_svm(pm->foc.v_alph * pm->foc.inv_vbus,
                   pm->foc.v_beta * pm->foc.inv_vbus,
                   &pm->foc.dtc_a,
                   &pm->foc.dtc_b,
                   &pm->foc.dtc_c) == 0;
}

/**
 * @brief 更新当前控制周期使用的 FOC 运行反馈。
 *
 * @param[in,out] pm 电机控制对象。
 *
 * 本函数不直接读取硬件，也不执行电流环。它使用前面已经更新的 ADC 和角度结果，
 * 依次计算母线电压、控制器限幅、转矩和速度，最后更新观测器。
 */
_RAM_FUNC void foc_feedback_update(pmsm_t *pm)
{
    /* 第 1 步：把母线 ADC 计数换算成电压，并计算调制所需的电压系数和余量。 */
    pm->foc.vbus = ((float)pm->adc.vbus * pm->board.v_ratio);
    pm->foc.inv_vbus = 1.5f / (pm->foc.vbus);
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

    /* 第 5 步：把本周期速度和转矩送入现有观测器；当前节点不改观测器算法。 */
    eh_speed_observer(&eh_vobs, pm->foc.wr, pm->foc.tor_r);
    eh_torque_observer(&eh_tobs, pm->foc.we, pm->foc.tor_r);
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
 * @brief 启动三相 PWM 的主输出和互补输出。
 *
 * 当前仅由 Drive 的 START 过程调用。FOC 层不直接操作 TIM1，
 * 而是调用 target_pwm_start_phase_outputs()，让板级适配层完成硬件启动。
 * 本函数只把启动请求交给板级适配层，不修改占空比，也不处理故障状态。
 */
_RAM_FUNC void foc_pwm_start(void)
{
    target_pwm_start_phase_outputs();
}

/**
 * @brief 停止三相 PWM 的主输出和互补输出。
 *
 * 当前由电机状态机在离开运行流程时调用。FOC 层只发出停止请求，
 * 具体关闭 TIM1 主输出和互补输出的操作由板级适配层完成。
 * 本函数只把停止请求交给板级适配层，不等同于完整的 Gate 或故障保护流程。
 */
_RAM_FUNC void foc_pwm_stop(void)
{
    target_pwm_stop_phase_outputs();
}

/**
 * @brief 读取本控制周期的原始 ADC 数据并换算三相电流。
 *
 * @param[in,out] pm 电机控制对象，用于保存原始 ADC 值和换算后的三相电流。
 *
 * 函数依次完成三件事：从板级适配层取得原始计数值；根据 ABC/ACB 接线关系
 * 映射到 A、B、C 相；扣除零偏并乘以电流换算系数。采样触发和 DMA 不在这里处理。
 */
_RAM_FUNC void foc_adc_sample(pmsm_t* pm)
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
        /* 保留原有行为：相序无效时不更新三相电流和相电压原始值。 */
        break;
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
}

/**
 * @brief 将 FOC 算出的三相占空比映射到物理 PWM 通道。
 *
 * @param[in] pm 电机控制对象，使用其中的相序和 A、B、C 三相占空比。
 *
 * FOC 计算函数把结果保存到 dtc_a、dtc_b、dtc_c，并返回占空比是否有效。
 * 调用者确认有效后立即调用本函数；本函数根据接线顺序排列三相占空比，再交给
 * 板级适配层写入 TIM1。
 */
_RAM_FUNC void foc_pwm_commit(pmsm_t* pm)
{
    switch (pm->para.phase_order) {
    case ABC_PHASE:
        /* ABC 接线：TIM1 通道顺序为 A、B、C。 */
        target_pwm_set_duty_ratios(pm->foc.dtc_a,
                                   pm->foc.dtc_b,
                                   pm->foc.dtc_c);
        break;
    case ACB_PHASE:
        /* ACB 接线：交换 B、C 两相后写入物理通道。 */
        target_pwm_set_duty_ratios(pm->foc.dtc_a,
                                   pm->foc.dtc_c,
                                   pm->foc.dtc_b);
        break;
    default:
        /* 保留原有行为：相序无效时不更新本周期占空比。 */
        break;
    }
}

/**
 * @brief 将三路 PWM 固定设置为 50% 占空比。
 *
 * @param[in] pm 为兼容现有接口保留，本函数不使用该参数。
 *
 * Drive 的 START 过程调用本函数后进入 STARTING；此处只保留原有的 50% 设置。
 */
_RAM_FUNC void foc_pwm_duty_set(pmsm_t* pm)
{
    /* 旧接口带有 pm 参数，但固定 50% 占空比不需要读取电机控制数据。 */
    (void)pm;

    /* 三个物理通道都设置为 0.5，即 PWM 周期的一半。 */
    target_pwm_set_duty_ratios(0.5f, 0.5f, 0.5f);
}
