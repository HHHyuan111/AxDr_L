#include "common.h"
#include "control_mit.h"
#include "drive_pwm.h"
#include "modlue.h"

/**
***********************************************************************
* @brief:      open_volt_step(foc_t *foc)
* @param[in]:  foc  指向永磁同步电机（PMSM）控制结构体的指针
* @retval:     void
* @details:    V/f 控制模式下的电压控制，计算电角速度和位置，执行电压控制
***********************************************************************
**/
_RAM_FUNC void open_volt_step(foc_t *foc)
{
    // Calculate electrical angular velocity from reference speed
    foc->ctrl.we_set = foc->ctrl.wr_set * foc->motor.pn;
    // Calculate position increment per FOC period
    foc->ctrl.pos_acc = foc->ctrl.we_set * foc->rate.foc_ts;
    // Update electrical angle
    foc->ctrl.drag_pe += foc->ctrl.pos_acc;
    // Wrap angle to [0, 2π)
    wrap_0_2pi(foc->ctrl.drag_pe);
    // Apply voltage control
    if (foc_volt_step(foc, foc->ctrl.vd_set, foc->ctrl.vq_set, foc->ctrl.drag_pe)) {
        drive_pwm_commit(foc);
    }
}
/**
***********************************************************************
* @brief:      open_cur_step(foc_t *foc)
* @param[in]:  foc  指向永磁同步电机（PMSM）控制结构体的指针
* @retval:     void
* @details:    I/F 强制电流控制模式，计算电角速度和位置，执行电流控制
***********************************************************************
**/
_RAM_FUNC void open_cur_step(foc_t *foc)
{
    foc->ctrl.we_set = foc->ctrl.wr_set * foc->motor.pn;
    foc->ctrl.pos_acc = foc->ctrl.we_set * foc->rate.foc_ts;
    foc->ctrl.drag_pe += foc->ctrl.pos_acc;
    wrap_0_2pi(foc->ctrl.drag_pe);
    if (foc_cur_step(foc, foc->ctrl.id_set, foc->ctrl.iq_set, foc->ctrl.drag_pe)) {
        drive_pwm_commit(foc);
    }
}

/**
***********************************************************************
* @brief:      pm_mit_mode(foc_t *foc)
* @param[in]:  foc  指向永磁同步电机（PMSM）控制结构体的指针
* @retval:     void
* @details:    MIT力矩控制模式下的控制逻辑
***********************************************************************
**/
_RAM_FUNC void pm_mit_mode(foc_t *foc)
{
    const control_mit_input_t input = {
        .position_ref_rad = foc->ctrl.posm_set,
        .position_feedback_rad = foc->sig.mp_m,
        .speed_ref_rad_s = foc->ctrl.wm_ref,
        .speed_feedback_rad_s = foc->sig.wm,
        .torque_feedforward_nm = foc->ctrl.torm_set,
        .position_gain_nm_per_rad = foc->ctrl.kp,
        .speed_gain_nm_s_per_rad = foc->ctrl.kd,
    };
    control_mit_output_t output;

    control_mit_step(&input, &output);
    foc->ctrl.mit_tor_set = output.torque_cmd_nm;

    /* 保持现有固件行为：电流参考仍由前馈转矩 torm_set 生成。 */
    foc->ctrl.tor_set = foc->ctrl.torm_set * foc->motor.div_Gr;
    foc->ctrl.iq_set = foc->ctrl.tor_set * foc->motor.div_Kt;

    if (foc_cur_step(foc, foc->ctrl.id_set, foc->ctrl.iq_set, foc->sig.p_e)) {
        drive_pwm_commit(foc);
    }
}

/**
***********************************************************************
* @brief:      pt_tor_mode(foc_t *foc)
* @param[in]:  foc  指向永磁同步电机（PMSM）控制结构体的指针
* @retval:     void
* @details:    轮廓力矩模式下的控制逻辑，根据目标转速或力矩设定计算电流并进行FOC电流控制
***********************************************************************
**/
_RAM_FUNC void pt_tor_mode(foc_t *foc)
{
    if (ABS(foc->ctrl.wm_set) > 0 && foc->ctrl.iq_set != 0)
    {
        if (++foc->rate.spd_pid_cnt >= foc->rate.spd_pid_cnt_val)
        {
            foc->rate.spd_pid_cnt = 0;
            foc->ctrl.wr_set = foc->ctrl.wm_set * foc->motor.Gr;
            control_pid_serial_step(&foc->spd_pi, foc->ctrl.wr_set, foc->sig.wr);
            foc->ctrl.iq_set = foc->spd_pi.out_value;
        }
    }

    foc->ctrl.tor_set = foc->ctrl.torm_set * foc->motor.div_Gr;
    foc->ctrl.iq_set = foc->ctrl.tor_set * foc->motor.div_Kt;

    if (foc_cur_step(foc, foc->ctrl.id_set, foc->ctrl.iq_set, foc->sig.p_e)) {
        drive_pwm_commit(foc);
    }
}
/**
***********************************************************************
* @brief:      pv_vel_mode(foc_t *foc)
* @param[in]:  foc  指向永磁同步电机（PMSM）控制结构体的指针
* @retval:     void
* @details:    轮廓速度模式下的控制逻辑，实现速度曲线平滑跟踪和力矩限幅，调用FOC速度环控制
***********************************************************************
**/
_RAM_FUNC void pv_vel_mode(foc_t *foc)
{
    bool update_traj = false;
    if (foc->app.vel_set_immediate) {
        update_traj = (foc->ctrl.wm_set != foc->ctrl.wm_lst);
    } else  {
        update_traj = foc->app.vel_set_by_flag;
    }

    if (update_traj)
    {
        spd_traj_plan(&foc->traj,    // trajectory pointer
                foc->sig.wm,         // start velocity
                foc->ctrl.wm_set,    // target velocity
                foc->ctrl.wm_acc,    // acceleration
                foc->ctrl.wm_dec,    // deceleration
                foc->app.v_curve);// curve
        foc->app.vel_reached = 0;
        foc->ctrl.wm_lst = foc->ctrl.wm_set;
        foc->app.vel_set_by_flag = 0;
    }
    spd_traj_eval(&foc->traj);

    foc->ctrl.wm_ref = foc->traj.spd_step;
    foc->ctrl.wr_set = foc->ctrl.wm_ref * foc->motor.Gr;
    foc->ctrl.tor_set = foc->ctrl.torm_set * foc->motor.div_Gr;
    foc->ctrl.iq_set = foc->ctrl.tor_set * foc->motor.div_Kt;
    if (foc_spd_step(foc, foc->ctrl.wr_set, foc->ctrl.iq_set, foc->sig.p_e)) {
        drive_pwm_commit(foc);
    }
}
/**
***********************************************************************
* @brief:      pp_pos_mode(foc_t *foc)
* @param[in]:  foc  指向永磁同步电机（PMSM）控制结构体的指针
* @retval:     void
* @details:    轮廓位置模式下的控制逻辑，实现位置-速度-电流级联控制，调用FOC位置环控制
***********************************************************************
**/
_RAM_FUNC void pp_pos_mode(foc_t *foc)
{
    bool update_traj = false;

    if (foc->app.pos_pause) {
        stop_ramp_step(foc, foc->app.pause_dec);
    }
    else
    {
        switch (foc->app.pos_ctrl_mode) {
            case abs_pos_mode:
                if (foc->app.pos_set_immediate) {
                    update_traj = (foc->ctrl.posm_set != foc->ctrl.posm_lst)
                                || (foc->ctrl.wm_set != foc->ctrl.wm_lst);
                } else {
                    update_traj = foc->app.pos_set_by_flag;
                }
                // 🔑 检测暂停恢复
                if (foc->app.last_pos_pause && !foc->app.pos_pause) {
                    update_traj = true;
                }
                break;

            case rel_pos_mode:
                foc->app.pos_set_immediate = 1; // 相对位置模式始终为标志位更新
                update_traj = foc->app.pos_set_by_flag;
                // 🔑 检测暂停恢复
                if (foc->app.last_pos_pause && !foc->app.pos_pause) {
                    update_traj = true;
                }
                break;
            default:
                break;
            }

        if(foc->cmd.wm_set == 0)
            foc->ctrl.wm_set = foc->app.pmax_velm;

        if (update_traj) {
            switch (foc->app.pos_ctrl_mode) {
            case abs_pos_mode:
                foc->app.abs_pos_ref = foc->ctrl.posm_set;  // 绝对位置
                pos_traj_plan(&foc->traj,
                        foc->sig.mp_m,  // start pos
                        foc->app.abs_pos_ref,  // target pos
                        foc->sig.wm,  // start velocity
                        foc->ctrl.wm_set,   // target velocity
                        foc->ctrl.wm_acc,   // acceleration
                        foc->ctrl.wm_dec,   // deceleration
                        foc->app.p_curve);    // curve
                foc->app.pos_reached = 0;
                foc->ctrl.posm_lst = foc->ctrl.posm_set;
                foc->ctrl.wm_lst   = foc->ctrl.wm_set;
                foc->app.pos_set_by_flag = 0;
                break;
            case rel_pos_mode:
                foc->app.rel_pos_ref = foc->ctrl.posm_set + foc->sig.mp_m;  // 相对位置
                pos_traj_plan(&foc->traj,
                              foc->sig.mp_m,  // start pos
                              foc->app.rel_pos_ref,  // target pos
                              foc->sig.wm,  // start velocity
                              foc->ctrl.wm_set,   // target velocity
                              foc->ctrl.wm_acc,   // acceleration
                              foc->ctrl.wm_dec,   // deceleration
                              foc->app.p_curve);    // curve
                foc->app.pos_reached = 0;
                foc->ctrl.posm_lst = foc->ctrl.posm_set;
                foc->ctrl.wm_lst   = foc->ctrl.wm_set;
                foc->app.pos_set_by_flag = 0;
                break;
            default:
                break;
            }
        }

        pos_traj_eval(&foc->traj);

        foc->ctrl.posm_ref = foc->traj.pos_step;
        foc->ctrl.posr_set = foc->ctrl.posm_ref * foc->motor.Gr;
        foc->ctrl.wr_set   = foc->ctrl.wm_set * foc->motor.Gr;
        foc->ctrl.tor_set  = foc->ctrl.torm_set * foc->motor.div_Gr;
        foc->ctrl.iq_set   = foc->ctrl.tor_set  * foc->motor.div_Kt;
        if (foc_pos_step(foc,
                    foc->ctrl.posr_set,
                    foc->ctrl.wr_set,
                    foc->ctrl.iq_set,
                    foc->sig.p_e)) {
            drive_pwm_commit(foc);
        }
    }
    // 🔑 更新暂停状态记录
    foc->app.last_pos_pause = foc->app.pos_pause;
}

_RAM_FUNC void cst_step(foc_t *foc)
{
    if (ABS(foc->ctrl.wm_set) > 0 && foc->ctrl.iq_set != 0)
    {
        if (++foc->rate.spd_pid_cnt >= foc->rate.spd_pid_cnt_val)
        {
            foc->rate.spd_pid_cnt = 0;
            foc->ctrl.wr_set = foc->ctrl.wm_set * foc->motor.Gr;
            control_pid_serial_step(&foc->spd_pi, foc->ctrl.wr_set, foc->sig.wr);
            foc->ctrl.iq_set = foc->spd_pi.out_value;
        }
    }

    foc->ctrl.tor_set = foc->ctrl.torm_set * foc->motor.div_Gr;
    foc->ctrl.iq_set = foc->ctrl.tor_set * foc->motor.div_Kt;
    if (foc_cur_step(foc, foc->ctrl.id_set, foc->ctrl.iq_set, foc->sig.p_e)) {
        drive_pwm_commit(foc);
    }
}

_RAM_FUNC void csv_step(foc_t *foc)
{
    foc->ctrl.wm_ref = foc->ctrl.wm_set;
    foc->ctrl.wr_set = foc->ctrl.wm_ref * foc->motor.Gr;
    foc->ctrl.tor_set = foc->ctrl.torm_set * foc->motor.div_Gr;
    foc->ctrl.iq_set = foc->ctrl.tor_set * foc->motor.div_Kt;
    if (foc_spd_step(foc, foc->ctrl.wr_set, foc->ctrl.iq_set, foc->sig.p_e)) {
        drive_pwm_commit(foc);
    }
}

_RAM_FUNC void csp_step(foc_t *foc)
{
    foc->ctrl.posm_ref = foc->ctrl.posm_set;
    foc->ctrl.posr_set = foc->ctrl.posm_ref * foc->motor.Gr;
    foc->ctrl.wr_set   = foc->ctrl.wm_set * foc->motor.Gr;
    foc->ctrl.tor_set  = foc->ctrl.torm_set * foc->motor.div_Gr;
    foc->ctrl.iq_set   = foc->ctrl.tor_set  * foc->motor.div_Kt;
    if (foc_pos_step(foc,
                foc->ctrl.posr_set,
                foc->ctrl.wr_set,
                foc->ctrl.iq_set,
                foc->sig.p_e)) {
        drive_pwm_commit(foc);
    }
}

_RAM_FUNC void quick_stop_step(foc_t *foc)
{
    stop_ramp_step(foc, foc->app.quick_stop_dec);
    if (fabsf(foc->sig.wr_f) < 0.5f) {
        foc->req = DRIVE_REQ_STOP;
        foc->mode.sys = release_mode;
    }
}

_RAM_FUNC void fault_stop_step(foc_t *foc)
{
    if(foc->app.fault_stop_dec == 0)
    {
        foc->req = DRIVE_REQ_STOP;
        foc->mode.sys = release_mode;
        return ;
    }
    
    stop_ramp_step(foc, foc->app.fault_stop_dec);
    if (fabsf(foc->sig.wr_f) < 0.5f) {
        foc->req = DRIVE_REQ_STOP;
        foc->mode.sys = release_mode;
    }
}

_RAM_FUNC void stop_ramp_step(foc_t *foc, float dec)
{
    if (foc->ctrl.wm_set > 0.0f)
    {
        foc->ctrl.wr_set -= dec * foc->motor.Gr * foc->rate.foc_ts;
        if (foc->ctrl.wr_set < 0.0f)
            foc->ctrl.wr_set = 0.0f;
    }
    if (foc->ctrl.wr_set < 0.0f)
    {
        foc->ctrl.wr_set += dec * foc->motor.Gr * foc->rate.foc_ts;
        if (foc->ctrl.wr_set > 0.0f)
            foc->ctrl.wr_set = 0.0f;
    }
    if (foc_spd_step(foc, foc->ctrl.wr_set, foc->ctrl.iq_set, foc->sig.p_e)) {
        drive_pwm_commit(foc);
    }
}

_RAM_FUNC void anticog_comp_step(foc_t *foc)
{
    if (foc->flag.bit.anticog_enable == 1)
    {
        float index_f = foc->sig.sp_r * div_M_2PI * (float)foc->anticog.map_num;
        uint16_t index0 = (uint16_t)index_f;
        // 将补偿电流加入目标
        switch(foc->sig.mode)
		{
			case foc_volt_mode: break;
			case foc_curr_mode: foc->ctrl.iq_set += foc->map.aco_table[index0]; break;
			case foc_vel_mode:  foc->ctrl.iq_lim += foc->map.aco_table[index0]; break;
			case foc_pos_mode:  foc->ctrl.iq_lim += foc->map.aco_table[index0]; break;
		}
    }
//	if (foc->flag.bit.low_vel_high_mag == 1)
		
}
