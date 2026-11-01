/**
 * @file foc_drv.c
 * @brief FOC 总对象初始化。主链位置：主程序初始化（快环放行前）；每拍映射见 foc_mapping.c。
 */

#include "common.h"
#include "foc_run.h"
#include "motor_profile.h"
#include "speed_adapter.h"

#include <string.h>

#include "algorithm_config.h"
#include "board_config.h"
#include "control_filter.h"
#include "encoder_adapter.h"
#include "encoder_config.h"
#include "drive_diag.h"
#include "observer_adapter.h"

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


