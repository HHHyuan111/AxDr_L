/**
 * @file motor_profile.c
 * @brief 电机档案装载实现。PR60/2312S 自 foc_drv.c 原样迁入（零行为变化），
 *        沉沙档案为新增（值出处见 motor_config.h 档案块注释）。
 */

#include "motor_profile.h"

#include "motor_config.h"
#include "algorithm_config.h"

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
    foc->motor.Kt = 1.5f * foc->motor.pn * foc->motor.flux;
    foc->motor.div_Kt = 1.0f / foc->motor.Kt;

    foc->ref.acc_m = CTRL_PR60_PROFILE_ACCEL_RAD_S2;
    foc->ref.dec_m = CTRL_PR60_PROFILE_DECEL_RAD_S2;

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
    foc->app.nmax_posm = -foc->app.pmax_posm;

    foc->ref.iq_max = foc->app.pmax_torm
        * foc->motor.div_Gr * foc->motor.div_Kt;
    foc->ref.iq_min = foc->app.nmax_torm
        * foc->motor.div_Gr * foc->motor.div_Kt;

    foc->ref.spd_max =  foc->app.pmax_velm*foc->motor.Gr;
    foc->ref.spd_min =  foc->app.nmax_velm*foc->motor.Gr;

    foc->motor.ibw = CTRL_PR60_CURRENT_BANDWIDTH_RAD_S;
    foc->motor.delta = CTRL_PR60_SPEED_DAMPING_RATIO;
    foc->spd_pi.kfp = CTRL_PR60_SPEED_REFERENCE_WEIGHT;
    foc->spd_pi.kf_damp = CTRL_PR60_SPEED_FEEDBACK_DAMPING;
    foc->pos_pi.kp = CTRL_PR60_POSITION_KP;
    foc->pos_pi.ki = CTRL_PR60_POSITION_KI;
    foc->pos_pi.kd = CTRL_PR60_POSITION_KD;
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
    foc->motor.Kt = 1.5f * foc->motor.pn * foc->motor.flux;
    foc->motor.div_Kt = 1.0f / foc->motor.Kt;

    foc->ref.acc_m = CTRL_2312S_PROFILE_ACCEL_RAD_S2;
    foc->ref.dec_m = CTRL_2312S_PROFILE_DECEL_RAD_S2;

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
    foc->app.nmax_posm = -foc->app.pmax_posm;

    foc->ref.iq_max = foc->app.pmax_torm
        * foc->motor.div_Gr * foc->motor.div_Kt;
    foc->ref.iq_min = foc->app.nmax_torm
        * foc->motor.div_Gr * foc->motor.div_Kt;

    foc->ref.spd_max =  foc->app.pmax_velm*foc->motor.Gr;
    foc->ref.spd_min =  foc->app.nmax_velm*foc->motor.Gr;

    foc->motor.ibw = CTRL_2312S_CURRENT_BANDWIDTH_RAD_S;
    foc->motor.delta = CTRL_2312S_SPEED_DAMPING_RATIO;
    foc->spd_pi.kfp = CTRL_2312S_SPEED_REFERENCE_WEIGHT;
    foc->spd_pi.kf_damp = CTRL_2312S_SPEED_FEEDBACK_DAMPING;
    foc->pos_pi.kp = CTRL_2312S_POSITION_KP;
    foc->pos_pi.ki = CTRL_2312S_POSITION_KI;
    foc->pos_pi.kd = CTRL_2312S_POSITION_KD;
}
#endif

#if MOTOR_SELECTED_MODEL == MOTOR_MODEL_CHENSHA
/**
 * @brief 沉沙 200W PMSM 档案装载（B 库三闭环验证组合，值出处 motor_config.h 注释）。
 */
static void motor_chensha_init(foc_t *foc)
{
    foc->motor.rated_voltage = CHENSHA_RATED_VOLTAGE_V;
    foc->motor.rated_current = CHENSHA_RATED_CURRENT_A;
    foc->motor.rated_speed = CHENSHA_RATED_SPEED_RAD_S;
    foc->motor.rated_torque = CHENSHA_RATED_TORQUE_NM;
    foc->motor.rated_power = CHENSHA_RATED_POWER_W;
    foc->motor.peak_current = CHENSHA_PEAK_CURRENT_A;
    foc->motor.peak_torque = CHENSHA_PEAK_TORQUE_NM;
    foc->motor.peak_speed = CHENSHA_PEAK_SPEED_RAD_S;

    foc->motor.pn = CHENSHA_POLE_PAIRS;
    foc->motor.Rs = CHENSHA_PHASE_RESISTANCE_OHM;
    foc->motor.Ld = CHENSHA_D_AXIS_INDUCTANCE_H;
    foc->motor.Lq = CHENSHA_Q_AXIS_INDUCTANCE_H;
    foc->motor.Ls = CHENSHA_AVERAGE_INDUCTANCE_H;
    foc->motor.Ldif = CHENSHA_DIFFERENTIAL_INDUCTANCE_H;
    foc->motor.flux = CHENSHA_FLUX_LINKAGE_WB;
    foc->motor.B = CHENSHA_VISCOUS_FRICTION_NM_S;
    foc->motor.Js = CHENSHA_INERTIA_KG_M2;

    foc->motor.Gr = CHENSHA_GEAR_RATIO;
    foc->motor.div_pn = 1.0f / foc->motor.pn;
    foc->motor.pnd_2pi = foc->motor.pn / M_2PI;
    foc->motor.div_Gr = 1.0f / foc->motor.Gr;
    foc->motor.Kt = 1.5f * foc->motor.pn * foc->motor.flux;
    foc->motor.div_Kt = 1.0f / foc->motor.Kt;

    foc->ref.acc_m = CHENSHA_ACCEL_LIMIT_RAD_S2;
    foc->ref.dec_m = CHENSHA_ACCEL_LIMIT_RAD_S2;

    foc->motor.phase_order = CHENSHA_PHASE_ORDER;
    foc->motor.e_off = CHENSHA_ELECTRICAL_OFFSET_RAD;
    foc->motor.r_off = CHENSHA_ROTOR_OFFSET_RAD;
    foc->motor.m_off = CHENSHA_MECHANICAL_OFFSET_RAD;

    foc->app.pmax_posm = MOTOR_MAX_POSITION_RAD;
    foc->app.nmax_posm = -foc->app.pmax_posm;

    /* 台架限幅直接定版（B 库 target_config 沉沙段）：指令电流 ±8A、速度 ±335 rad/s。
     * 不走 peak_torque×usage_ratio 链，避免换算歧义——这两个数就是验收基准。 */
    foc->app.pmax_torm = CHENSHA_COMMAND_CURRENT_LIMIT_A * foc->motor.Kt;
    foc->app.nmax_torm = -foc->app.pmax_torm;
    foc->app.pmax_velm = CHENSHA_PEAK_SPEED_RAD_S;
    foc->app.nmax_velm = -foc->app.pmax_velm;

    foc->ref.iq_max = CHENSHA_COMMAND_CURRENT_LIMIT_A;
    foc->ref.iq_min = -CHENSHA_COMMAND_CURRENT_LIMIT_A;
    foc->ref.spd_max = CHENSHA_PEAK_SPEED_RAD_S * foc->motor.Gr;
    foc->ref.spd_min = -foc->ref.spd_max;

    /* 环路整定（电流环 Kp/Ki 由 cur_pi_init 按 ibw 公式计算）。 */
    foc->motor.ibw = CHENSHA_CURRENT_BW_RAD_S;
    foc->motor.delta = 1.0f;
    foc->spd_pi.kfp = CHENSHA_SPEED_REFERENCE_WEIGHT;
    foc->spd_pi.kf_damp = CHENSHA_SPEED_FEEDBACK_DAMPING;
    foc->pos_pi.kp = CHENSHA_POSITION_KP_S_INV;
    foc->pos_pi.ki = CHENSHA_POSITION_KI;
    foc->pos_pi.kd = CHENSHA_POSITION_KD;
}
#endif

void motor_profile_load(foc_t *foc)
{
#if MOTOR_SELECTED_MODEL == MOTOR_MODEL_PR60
    motor_pr60_init(foc);
#elif MOTOR_SELECTED_MODEL == MOTOR_MODEL_2312S
    motor_2312s_init(foc);
#elif MOTOR_SELECTED_MODEL == MOTOR_MODEL_CHENSHA
    motor_chensha_init(foc);
#else
#error "Unsupported MOTOR_SELECTED_MODEL"
#endif
}

void motor_profile_control_load(foc_t *foc)
{
#if MOTOR_SELECTED_MODEL == MOTOR_MODEL_CHENSHA
    /* 保护阈值定版：覆盖 prot_cfg_init 的板级默认（B 库沉沙 10A/30V/10V）。 */
    foc->prot_cfg.over_current_a = CHENSHA_OVER_CURRENT_A;
    foc->prot_cfg.over_voltage_v = CHENSHA_OVER_VOLTAGE_V;
    foc->prot_cfg.under_voltage_v = CHENSHA_UNDER_VOLTAGE_V;

    /* 速度环固化值：覆盖 spd_pi_init 的机械模型公式（联轴实测证据，B 库 servo profile）。 */
    foc->spd_pi.kp = CHENSHA_SPEED_KP_A_PER_RAD_S;
    foc->spd_pi.ki = CHENSHA_SPEED_KI_A_PER_RAD;
#else
    (void)foc;
#endif
}
