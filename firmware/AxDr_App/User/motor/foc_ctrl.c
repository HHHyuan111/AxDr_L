#include "common.h"
#include "control_limit.h"
#include "control_mit.h"
#include "drive_pwm.h"
#include "modlue.h"

/**
***********************************************************************
* @brief:      force_volt_mode(pmsm_t* pm)
* @param[in]:  pm  指向永磁同步电机（PMSM）控制结构体的指针
* @retval:     void
* @details:    V/f 控制模式下的电压控制，计算电角速度和位置，执行电压控制
***********************************************************************
**/
_RAM_FUNC void force_volt_mode(pmsm_t* pm)
{
    // Calculate electrical angular velocity from reference speed
    pm->ctrl.we_set = pm->ctrl.wr_set * pm->para.pn;
    // Calculate position increment per FOC period
    pm->ctrl.pos_acc = pm->ctrl.we_set * pm->period.foc_ts;
    // Update electrical angle
    pm->ctrl.drag_pe += pm->ctrl.pos_acc;
    // Wrap angle to [0, 2π)
    wrap_0_2pi(pm->ctrl.drag_pe);
    // Apply voltage control
    if (foc_volt(pm, pm->ctrl.vd_set, pm->ctrl.vq_set, pm->ctrl.drag_pe)) {
        drive_pwm_commit(pm);
    }
}
/**
***********************************************************************
* @brief:      force_curr_mode(pmsm_t* pm)
* @param[in]:  pm  指向永磁同步电机（PMSM）控制结构体的指针
* @retval:     void
* @details:    I/F 强制电流控制模式，计算电角速度和位置，执行电流控制
***********************************************************************
**/
_RAM_FUNC void force_curr_mode(pmsm_t* pm)
{
    pm->ctrl.we_set = pm->ctrl.wr_set * pm->para.pn;
    pm->ctrl.pos_acc = pm->ctrl.we_set * pm->period.foc_ts;
    pm->ctrl.drag_pe += pm->ctrl.pos_acc;
    wrap_0_2pi(pm->ctrl.drag_pe);
    if (foc_curr(pm, pm->ctrl.id_set, pm->ctrl.iq_set, pm->ctrl.drag_pe)) {
        drive_pwm_commit(pm);
    }
}

/**
***********************************************************************
* @brief:      pm_mit_mode(pmsm_t* pm)
* @param[in]:  pm  指向永磁同步电机（PMSM）控制结构体的指针
* @retval:     void
* @details:    MIT力矩控制模式下的控制逻辑
***********************************************************************
**/
_RAM_FUNC void pm_mit_mode(pmsm_t* pm)
{
    const control_mit_input_t input = {
        .position_ref_rad = pm->ctrl.posm_set,
        .position_feedback_rad = pm->foc.mp_m,
        .speed_ref_rad_s = pm->ctrl.wm_ref,
        .speed_feedback_rad_s = pm->foc.wm,
        .torque_feedforward_nm = pm->ctrl.torm_set,
        .position_gain_nm_per_rad = pm->ctrl.kp,
        .speed_gain_nm_s_per_rad = pm->ctrl.kd,
    };
    control_mit_output_t output;

    control_mit_step(&input, &output);
    pm->ctrl.mit_tor_set = output.torque_cmd_nm;

    /* 保持现有固件行为：电流参考仍由前馈转矩 torm_set 生成。 */
    pm->ctrl.tor_set = pm->ctrl.torm_set * pm->para.div_Gr;
    pm->ctrl.iq_set = pm->ctrl.tor_set * pm->para.div_Kt;

    if (foc_curr(pm, pm->ctrl.id_set, pm->ctrl.iq_set, pm->foc.p_e)) {
        drive_pwm_commit(pm);
    }
}

/**
***********************************************************************
* @brief:      pt_tor_mode(pmsm_t* pm)
* @param[in]:  pm  指向永磁同步电机（PMSM）控制结构体的指针
* @retval:     void
* @details:    轮廓力矩模式下的控制逻辑，根据目标转速或力矩设定计算电流并进行FOC电流控制
***********************************************************************
**/
_RAM_FUNC void pt_tor_mode(pmsm_t* pm)
{
    if (ABS(pm->ctrl.wm_set) > 0 && pm->ctrl.iq_set != 0)
    {
        if (++pm->period.spd_pid_cnt >= pm->period.spd_pid_cnt_val)
        {
            pm->period.spd_pid_cnt = 0;
            pm->ctrl.wr_set = pm->ctrl.wm_set * pm->para.Gr;
            control_pid_serial_step(&pm->spd_pi, pm->ctrl.wr_set, pm->foc.wr);
            pm->ctrl.iq_set = pm->spd_pi.out_value;
        }
    }

    pm->ctrl.tor_set = pm->ctrl.torm_set * pm->para.div_Gr;
    pm->ctrl.iq_set = pm->ctrl.tor_set * pm->para.div_Kt;

    if (foc_curr(pm, pm->ctrl.id_set, pm->ctrl.iq_set, pm->foc.p_e)) {
        drive_pwm_commit(pm);
    }
}
/**
***********************************************************************
* @brief:      pv_vel_mode(pmsm_t* pm)
* @param[in]:  pm  指向永磁同步电机（PMSM）控制结构体的指针
* @retval:     void
* @details:    轮廓速度模式下的控制逻辑，实现速度曲线平滑跟踪和力矩限幅，调用FOC速度环控制
***********************************************************************
**/
_RAM_FUNC void pv_vel_mode(pmsm_t* pm)
{
    bool update_traj = false;
    if (pm->app_ctrl.vel_set_immediate) {
        update_traj = (pm->ctrl.wm_set != pm->ctrl.wm_lst);
    } else  {
        update_traj = pm->app_ctrl.vel_set_by_flag;
    }

    if (update_traj)
    {
        spd_traj_plan(&pm->traj,    // trajectory pointer
                pm->foc.wm,         // start velocity
                pm->ctrl.wm_set,    // target velocity
                pm->ctrl.wm_acc,    // acceleration
                pm->ctrl.wm_dec,    // deceleration
                pm->app_ctrl.v_curve);// curve
        pm->app_ctrl.vel_reached = 0;
        pm->ctrl.wm_lst = pm->ctrl.wm_set;
        pm->app_ctrl.vel_set_by_flag = 0;
    }
    spd_traj_eval(&pm->traj);

    pm->ctrl.wm_ref = pm->traj.spd_step;
    pm->ctrl.wr_set = pm->ctrl.wm_ref * pm->para.Gr;
    pm->ctrl.tor_set = pm->ctrl.torm_set * pm->para.div_Gr;
    pm->ctrl.iq_set = pm->ctrl.tor_set * pm->para.div_Kt;
    if (foc_vel(pm, pm->ctrl.wr_set, pm->ctrl.iq_set, pm->foc.p_e)) {
        drive_pwm_commit(pm);
    }
}
/**
***********************************************************************
* @brief:      pp_pos_mode(pmsm_t* pm)
* @param[in]:  pm  指向永磁同步电机（PMSM）控制结构体的指针
* @retval:     void
* @details:    轮廓位置模式下的控制逻辑，实现位置-速度-电流级联控制，调用FOC位置环控制
***********************************************************************
**/
_RAM_FUNC void pp_pos_mode(pmsm_t* pm)
{
    bool update_traj = false;

    if (pm->app_ctrl.pos_pause) {
        pmsm_slow_down(pm, pm->app_ctrl.pause_dec);
    }
    else
    {
        switch (pm->app_ctrl.pos_ctrl_mode) {
            case abs_pos_mode:
                if (pm->app_ctrl.pos_set_immediate) {
                    update_traj = (pm->ctrl.posm_set != pm->ctrl.posm_lst)
                                || (pm->ctrl.wm_set != pm->ctrl.wm_lst);
                } else {
                    update_traj = pm->app_ctrl.pos_set_by_flag;
                }
                // 🔑 检测暂停恢复
                if (pm->app_ctrl.last_pos_pause && !pm->app_ctrl.pos_pause) {
                    update_traj = true;
                }
                break;

            case rel_pos_mode:
                pm->app_ctrl.pos_set_immediate = 1; // 相对位置模式始终为标志位更新
                update_traj = pm->app_ctrl.pos_set_by_flag;
                // 🔑 检测暂停恢复
                if (pm->app_ctrl.last_pos_pause && !pm->app_ctrl.pos_pause) {
                    update_traj = true;
                }
                break;
            default:
                break;
            }

        if(pm->cmd.wm_set == 0)
            pm->ctrl.wm_set = pm->app_ctrl.pmax_velm;

        if (update_traj) {
            switch (pm->app_ctrl.pos_ctrl_mode) {
            case abs_pos_mode:
                pm->app_ctrl.abs_pos_ref = pm->ctrl.posm_set;  // 绝对位置
                pos_traj_plan(&pm->traj,
                        pm->foc.mp_m,  // start pos
                        pm->app_ctrl.abs_pos_ref,  // target pos
                        pm->foc.wm,  // start velocity
                        pm->ctrl.wm_set,   // target velocity
                        pm->ctrl.wm_acc,   // acceleration
                        pm->ctrl.wm_dec,   // deceleration
                        pm->app_ctrl.p_curve);    // curve
                pm->app_ctrl.pos_reached = 0;
                pm->ctrl.posm_lst = pm->ctrl.posm_set;
                pm->ctrl.wm_lst   = pm->ctrl.wm_set;
                pm->app_ctrl.pos_set_by_flag = 0;
                break;
            case rel_pos_mode:
                pm->app_ctrl.rel_pos_ref = pm->ctrl.posm_set + pm->foc.mp_m;  // 相对位置
                pos_traj_plan(&pm->traj,
                              pm->foc.mp_m,  // start pos
                              pm->app_ctrl.rel_pos_ref,  // target pos
                              pm->foc.wm,  // start velocity
                              pm->ctrl.wm_set,   // target velocity
                              pm->ctrl.wm_acc,   // acceleration
                              pm->ctrl.wm_dec,   // deceleration
                              pm->app_ctrl.p_curve);    // curve
                pm->app_ctrl.pos_reached = 0;
                pm->ctrl.posm_lst = pm->ctrl.posm_set;
                pm->ctrl.wm_lst   = pm->ctrl.wm_set;
                pm->app_ctrl.pos_set_by_flag = 0;
                break;
            default:
                break;
            }
        }

        pos_traj_eval(&pm->traj);

        pm->ctrl.posm_ref = pm->traj.pos_step;
        pm->ctrl.posr_set = pm->ctrl.posm_ref * pm->para.Gr;
        pm->ctrl.wr_set   = pm->ctrl.wm_set * pm->para.Gr;
        pm->ctrl.tor_set  = pm->ctrl.torm_set * pm->para.div_Gr;
        pm->ctrl.iq_set   = pm->ctrl.tor_set  * pm->para.div_Kt;
        if (foc_pos(pm,
                    pm->ctrl.posr_set,
                    pm->ctrl.wr_set,
                    pm->ctrl.iq_set,
                    pm->foc.p_e)) {
            drive_pwm_commit(pm);
        }
    }
    // 🔑 更新暂停状态记录
    pm->app_ctrl.last_pos_pause = pm->app_ctrl.pos_pause;
}

_RAM_FUNC void cst_tor_mode(pmsm_t* pm)
{
    if (ABS(pm->ctrl.wm_set) > 0 && pm->ctrl.iq_set != 0)
    {
        if (++pm->period.spd_pid_cnt >= pm->period.spd_pid_cnt_val)
        {
            pm->period.spd_pid_cnt = 0;
            pm->ctrl.wr_set = pm->ctrl.wm_set * pm->para.Gr;
            control_pid_serial_step(&pm->spd_pi, pm->ctrl.wr_set, pm->foc.wr);
            pm->ctrl.iq_set = pm->spd_pi.out_value;
        }
    }

    pm->ctrl.tor_set = pm->ctrl.torm_set * pm->para.div_Gr;
    pm->ctrl.iq_set = pm->ctrl.tor_set * pm->para.div_Kt;
    if (foc_curr(pm, pm->ctrl.id_set, pm->ctrl.iq_set, pm->foc.p_e)) {
        drive_pwm_commit(pm);
    }
}

_RAM_FUNC void csv_vel_mode(pmsm_t* pm)
{
    pm->ctrl.wm_ref = pm->ctrl.wm_set;
    pm->ctrl.wr_set = pm->ctrl.wm_ref * pm->para.Gr;
    pm->ctrl.tor_set = pm->ctrl.torm_set * pm->para.div_Gr;
    pm->ctrl.iq_set = pm->ctrl.tor_set * pm->para.div_Kt;
    if (foc_vel(pm, pm->ctrl.wr_set, pm->ctrl.iq_set, pm->foc.p_e)) {
        drive_pwm_commit(pm);
    }
}

_RAM_FUNC void csp_pos_mode(pmsm_t* pm)
{
    pm->ctrl.posm_ref = pm->ctrl.posm_set;
    pm->ctrl.posr_set = pm->ctrl.posm_ref * pm->para.Gr;
    pm->ctrl.wr_set   = pm->ctrl.wm_set * pm->para.Gr;
    pm->ctrl.tor_set  = pm->ctrl.torm_set * pm->para.div_Gr;
    pm->ctrl.iq_set   = pm->ctrl.tor_set  * pm->para.div_Kt;
    if (foc_pos(pm,
                pm->ctrl.posr_set,
                pm->ctrl.wr_set,
                pm->ctrl.iq_set,
                pm->foc.p_e)) {
        drive_pwm_commit(pm);
    }
}

_RAM_FUNC void pmsm_quick_stop_mode(pmsm_t* pm)
{
    pmsm_slow_down(pm, pm->app_ctrl.quick_stop_dec);
    if (fabsf(pm->foc.wr_f) < 0.5f) {
        pm->req = DRIVE_REQ_STOP;
        pm->mode.sys = release_mode;
    }
}

_RAM_FUNC void pmsm_fault_stop_mode(pmsm_t* pm)
{
    if(pm->app_ctrl.fault_stop_dec == 0)
    {
        pm->req = DRIVE_REQ_STOP;
        pm->mode.sys = release_mode;
        return ;
    }
    
    pmsm_slow_down(pm, pm->app_ctrl.fault_stop_dec);
    if (fabsf(pm->foc.wr_f) < 0.5f) {
        pm->req = DRIVE_REQ_STOP;
        pm->mode.sys = release_mode;
    }
}

_RAM_FUNC void pmsm_slow_down(pmsm_t* pm, float dec)
{
    if (pm->ctrl.wm_set > 0.0f)
    {
        pm->ctrl.wr_set -= dec * pm->para.Gr * pm->period.foc_ts;
        if (pm->ctrl.wr_set < 0.0f)
            pm->ctrl.wr_set = 0.0f;
    }
    if (pm->ctrl.wr_set < 0.0f)
    {
        pm->ctrl.wr_set += dec * pm->para.Gr * pm->period.foc_ts;
        if (pm->ctrl.wr_set > 0.0f)
            pm->ctrl.wr_set = 0.0f;
    }
    if (foc_vel(pm, pm->ctrl.wr_set, pm->ctrl.iq_set, pm->foc.p_e)) {
        drive_pwm_commit(pm);
    }
}

_RAM_FUNC void pmsm_reset(pmsm_t* pm)
{
    control_pid_clear(&pm->id_pi);
    control_pid_clear(&pm->iq_pi);
    control_pid_clear(&pm->spd_pi);
    control_pid_clear(&pm->pos_pi);
    foc_clear(pm);
}

_RAM_FUNC void pmsm_ctrl_set(pmsm_t* pm)
{
    if (pm->app_ctrl.polarity == motor_polarity_p)
    {
        pm->ctrl.torm_set = control_limit(pm->cmd.torm_set,
                                          pm->app_ctrl.pmax_torm,
                                          pm->app_ctrl.nmax_torm);
        pm->ctrl.wm_set = control_limit(pm->cmd.wm_set,
                                        pm->app_ctrl.pmax_velm,
                                        pm->app_ctrl.nmax_velm);
        pm->ctrl.posm_set = control_limit(pm->cmd.posm_set,
                                          pm->app_ctrl.pmax_posm,
                                          pm->app_ctrl.nmax_posm);
    }
    else if (pm->app_ctrl.polarity == motor_polarity_n)
    {
        pm->ctrl.torm_set = -1.0f * control_limit(pm->cmd.torm_set,
                                                  pm->app_ctrl.pmax_torm,
                                                  pm->app_ctrl.nmax_torm);
        pm->ctrl.wm_set = -1.0f * control_limit(pm->cmd.wm_set,
                                                pm->app_ctrl.pmax_velm,
                                                pm->app_ctrl.nmax_velm);
        pm->ctrl.posm_set = -1.0f * control_limit(pm->cmd.posm_set,
                                                  pm->app_ctrl.pmax_posm,
                                                  pm->app_ctrl.nmax_posm);
    }

    pm->ctrl.pmax_tor =  pm->app_ctrl.pmax_torm*pm->para.div_Gr;
    pm->ctrl.nmax_tor =  pm->app_ctrl.nmax_torm*pm->para.div_Gr;
    pm->ctrl.pmax_iq  =  pm->ctrl.pmax_tor*pm->para.div_Kt;
    pm->ctrl.nmax_iq  =  pm->ctrl.nmax_tor*pm->para.div_Kt;
    pm->ctrl.pmax_tor_vel =  pm->app_ctrl.pmax_torm_vel*pm->para.Gr;
    pm->ctrl.nmax_tor_vel =  pm->app_ctrl.nmax_torm_vel*pm->para.Gr;

    pm->ctrl.pmax_vel =  pm->app_ctrl.pmax_velm*pm->para.Gr;
    pm->ctrl.nmax_vel =  pm->app_ctrl.nmax_velm*pm->para.Gr;
    pm->ctrl.pmax_pos =  pm->app_ctrl.pmax_posm*pm->para.Gr;
    pm->ctrl.nmax_pos =  pm->app_ctrl.nmax_posm*pm->para.Gr;
}

/**
***********************************************************************
* @brief:      pmsm_fault_check(pmsm_t* pm)
* @param[in]:  pm  指向永磁同步电机（PMSM）控制结构体的指针
* @retval:     void
* @details:    故障检测函数，包括通信超时、线圈温度、MOS温度等多项保护
***********************************************************************
**/
_RAM_FUNC void pmsm_fault_check(pmsm_t* pm)
{
    // Communication timeout protection
    if (pm->protect.link_out_cnt > pm->protect.time_value && pm->protect.time_value > 0)
    {
        pm->protect.link_out_cnt = pm->protect.time_value;
        pm->fault.bit.off_link = 1; // Communication lost
        pm->protect.rst = 1;
    }

    // Communication timeout protection
    if (pm->protect.ov_speed_cnt > pm->protect.ov_speed_value && pm->protect.ov_speed_value > 0)
    {
        pm->protect.ov_speed_cnt = pm->protect.ov_speed_value;
        pm->fault.bit.ov_speed = 1;
        pm->protect.rst = 1;
    }

    // Motor coil temperature protection
    if (pm->foc.Tcoil > pm->protect.omt_value)
    {
        if (++pm->protect.omt_cnt > pm->protect.omt_cnt_value)
        {
            pm->fault.bit.ov_tcoi = 1; // Coil over temperature
            pm->protect.omt_cnt = pm->protect.omt_cnt_value;
            pm->protect.rst = 1;
        }
    }
    else
    {
        if (!pm->protect.rst)
        {
            pm->fault.bit.ov_tcoi = 0;
            pm->protect.omt_cnt = 0;
        }
    }

    // MOSFET temperature protection
    if (pm->foc.Tmos > pm->protect.ot_value)
    {
        if (++pm->protect.ot_cnt > pm->protect.ot_cnt_value)
        {
            pm->fault.bit.ov_tmos = 1; // MOSFET over temperature
            pm->protect.ot_cnt = pm->protect.ot_cnt_value;
            pm->protect.rst = 1;
        }
    }
    else
    {
        if (!pm->protect.rst)
        {
            pm->fault.bit.ov_tmos = 0;
            pm->protect.ot_cnt = 0;
        }
    }

    // voltage protection
    if (pm->foc.vbus < pm->protect.uv_value)
    {
        if (++pm->protect.uv_cnt > pm->protect.uv_cnt_value)
        {
            pm->fault.bit.un_volt = 1;
            pm->protect.uv_cnt = pm->protect.uv_cnt_value;
            pm->protect.rst = 1;
        }
    }
    else
    {
        if (!pm->protect.rst)
        {
            pm->fault.bit.un_volt = 0;
            pm->protect.uv_cnt = 0;
        }
    }
    if (pm->foc.vbus > pm->protect.ov_value)
    {
        if (++pm->protect.ov_cnt > pm->protect.ov_cnt_value)
        {
            pm->fault.bit.ov_volt = 1;
            pm->protect.ov_cnt = pm->protect.ov_cnt_value;
            pm->protect.rst = 1;
        }
    }
    else
    {
        if (!pm->protect.rst)
        {
            pm->fault.bit.ov_volt = 0;
            pm->protect.ov_cnt = 0;
        }
    }

    // ... (similar protection logic for current, voltage, etc.)
}

_RAM_FUNC void pmsm_anticog_comp(pmsm_t* pm)
{
    if (pm->flag.bit.anticog_enable == 1)
    {
        float index_f = pm->foc.sp_r * div_M_2PI * (float)pm->anticog.map_num;
        uint16_t index0 = (uint16_t)index_f;
        // 将补偿电流加入目标
        switch(pm->foc.mode)
		{
			case foc_volt_mode: break;
			case foc_curr_mode: pm->ctrl.iq_set += pm->map.aco_table[index0]; break;
			case foc_vel_mode:  pm->ctrl.iq_lim += pm->map.aco_table[index0]; break;
			case foc_pos_mode:  pm->ctrl.iq_lim += pm->map.aco_table[index0]; break;
		}
    }
//	if (pm->flag.bit.low_vel_high_mag == 1)
		
}
