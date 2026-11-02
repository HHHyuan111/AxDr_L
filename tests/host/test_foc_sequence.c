/**
 * @file test_foc_sequence.c
 * @brief 冻结旧实现与新实现共用的连续输入试验，比较状态、PI 历史和实际 PWM 调用。
 * 真实 Drive/模式/控制/调制，硬件和诊断任务用替身；不代表板端验证。
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "common.h"
#include "control_cycle.h"
#include "drive.h"
#ifndef AXDR_BASELINE
#include "drive_cycle.h"
#endif
#include "board_adapter.h"
#include "position_adapter.h"
#include "speed_adapter.h"
#include "observer_adapter.h"
obs_t g_obs;


static unsigned starts, stops, writes;
static float duty_a, duty_b, duty_c;
static bool stop_ok = true;

/* 初始化/零偏校准不属于本试验，误调用立即失败。 */
void motor_profile_load(foc_t *f) { (void)f; abort(); }
void motor_profile_control_load(foc_t *f) { (void)f; abort(); }
void encoder_init(encoder_state_t *e) { (void)e; abort(); }
void drive_diag_init(const foc_t *f) { (void)f; abort(); }
void obs_init(const foc_t *f) { (void)f; abort(); }
void HAL_Delay(unsigned int ms) { (void)ms; abort(); }
void target_adc_read_iabc_raw(target_adc_abc_raw_t *p) { (void)p; abort(); }
bool board_phase_map(phase_order_t o, const target_adc_abc_raw_t *r, target_adc_abc_raw_t *p)
{ (void)o; (void)r; (void)p; abort(); }

bool target_pwm_start_phase_outputs(void) { ++starts; return true; }
bool target_pwm_stop_phase_outputs(void) { ++stops; return stop_ok; }
void target_pwm_set_duty_ratios(float a, float b, float c)
{ ++writes; duty_a = a; duty_b = b; duty_c = c; }
bool encoder_sample(encoder_state_t *e) { (void)e; abort(); }
bool position_update(foc_t *f, position_sample_t *p) { (void)f; (void)p; abort(); }
bool foc_adc_sample(foc_t *f, board_sample_t *p) { (void)f; (void)p; abort(); }
void obs_step(const foc_t *f) { (void)f; }
bool drive_cmd_apply(foc_t *f) { (void)f; return true; }
void drive_diag_poll_request(foc_t *f) { (void)f; }
void drive_diag_on_fault(void) {}
void drive_diag_on_stopped(void) {}
bool drive_diag_is_supported(const foc_t *f) { (void)f; return false; }
bool drive_diag_prepare(foc_t *f) { (void)f; return false; }
#ifdef AXDR_BASELINE
bool drive_diag_step(foc_t *f) { (void)f; return false; }
#else
bool drive_diag_step(foc_t *f, foc_plan_t *p) { (void)f; (void)p; return false; }
bool drive_diag_check_output(foc_t *f, const foc_plan_t *p)
{ (void)f; (void)p; return true; }
#endif

static foc_t make_motor(unsigned scene)
{
    foc_t f = {0};
    f.rate.foc_fs = 20000.0f;
    f.rate.foc_ts = 0.00005f;
    f.rate.cur_pid_cnt_val = 1U;
    f.rate.spd_pid_cnt_val = 4U;
    f.rate.pos_pid_cnt_val = 8U;
    f.motor.pn = 7.0f;
    f.motor.Gr = 5.0f; f.motor.div_Gr = 0.2f;
    f.motor.Kt = 0.1f; f.motor.div_Kt = 10.0f;
    f.motor.phase_order = scene == 17U ? PHASE_ORDER_ACB : PHASE_ORDER_ABC;
    f.motor.align_current_a = 2.0f;
    f.enc.primary = ENCODER_TYPE_ABZ;
    f.enc_aligned = true;
    f.req = DRIVE_REQ_START;
    f.mode.sys = debug_mode;
    f.mode.debug = (debug_mode_e)(scene % 6U);
    if (scene >= 6U && scene < 12U)
    {
        const release_mode_e modes[] = {mit_mode, vel_mode, pos_mode, cst_mode, csv_mode, csp_mode};
        f.mode.sys = release_mode; f.mode.release = modes[scene-6U];
    }
    if (scene == 12U) { f.mode.sys = halt_mode; f.mode.halt = quick_mode; }
    if (scene == 13U)
    { f.enc_aligned = false; f.rate.foc_fs = 10.0f; f.rate.foc_ts = 0.1f; }
    f.ref.iq_max = 5.0f; f.ref.iq_min = -5.0f;
    f.ref.spd_max = 50.0f; f.ref.spd_min = -50.0f;
    f.iq_lpf.filt_a = f.wr_lpf.filt_a = 0.8f;
    f.iq_lpf.filt_b = f.wr_lpf.filt_b = 0.2f;
    f.app.pmax_torm = 3.0f; f.app.nmax_torm = -3.0f;
    f.app.pmax_velm = 10.0f; f.app.quick_stop_dec = 2.0f; f.app.pause_dec = 1.0f;
    f.app.pos_ctrl_mode = abs_pos_mode;
    const pid_para_t pi = {.kp=0.4f,.ki=10.0f,.ts=0.00005f,
        .kfp=1.0f,.kf_damp=0.2f,.i_term_max=10.0f,.i_term_min=-10.0f};
    f.id_pi = pi; f.iq_pi = pi; f.spd_pi = pi; f.pos_pi = pi;
    f.prot_cfg = (drive_protection_config_t){.under_voltage_v=10.0f,.over_voltage_v=40.0f,
        .over_current_a=5.0f,.under_voltage_samples=1,.over_voltage_samples=1,
        .over_current_samples=1,.invalid_current_samples=1,.invalid_bus_voltage_samples=1,
        .invalid_position_samples=1};
    return f;
}

static void set_ref(foc_t *f, unsigned tick)
{
    f->ref.vd = 1.1f; f->ref.vq = 1.5f;
    f->ref.id = 0.3f; f->ref.iq = 2.0f;
    f->ref.spd_r = tick < 120U ? 3.0f : -2.0f;
    f->ref.spd_m = f->ref.spd_r * 0.2f;
    f->ref.pos_r = 0.1f; f->ref.pos_m = 0.02f;
    f->ref.acc_m = 2.0f; f->ref.dec_m = 3.0f;
    f->ref.torq_m = 0.4f; f->ref.torq_ff = 0.1f;
    f->ref.kp = 0.5f; f->ref.kd = 0.1f;
}

static void trace(unsigned scene, unsigned tick, const foc_t *f)
{
    printf("%u %u %u %u %u %u %u %u %u %u %u %u %u %u",
        scene,tick,(unsigned)f->state,(unsigned)f->req,(unsigned)f->fault.all,
        (unsigned)f->pwm_active,(unsigned)f->pwm_cmd.valid,
        (unsigned)f->rate.cur_pid_cnt,(unsigned)f->rate.spd_pid_cnt,(unsigned)f->rate.pos_pid_cnt,
        starts,stops,writes,(unsigned)f->align_ticks);
    const float values[] = {f->fb.ia,f->fb.ib,f->fb.ic,f->fb.id,f->fb.iq,
        f->fb.ialpha,f->fb.ibeta,f->fb.spd_r,f->fb.spd_m,f->fb.iq_f,
        f->out.theta,f->out.sin,f->out.cos,f->out.vd,f->out.vq,f->out.valpha,f->out.vbeta,
        f->out.duty_a,f->out.duty_b,f->out.duty_c,duty_a,duty_b,duty_c,
        f->ref.theta_e,f->ref.spd_r_lim,f->ref.iq_lim,f->ref.torq_mit,f->ref.spd_m_ramp,
        f->ref.pos_m_ramp,f->motor.e_off,f->id_pi.i_term,f->iq_pi.i_term,
        f->spd_pi.i_term,f->pos_pi.i_term};
    for (unsigned i=0;i<sizeof(values)/sizeof(values[0]);++i) printf(" %a",(double)values[i]);
    putchar('\n');
}

#ifndef AXDR_BASELINE
static void check_stage_contract(void)
{
    foc_t f = make_motor(3U);
    f.pwm_active = true; f.req = DRIVE_REQ_RUN; f.state = DRIVE_STATE_RUN;
    f.fb.i_valid = f.fb.vbus_valid = f.fb.pos_valid = true;
    f.fb.vbus = 24.0f; f.fb.inv_vbus = 1.5f/24.0f;
    f.fb.ia = 0.4f; f.fb.ib = -0.1f; f.fb.ic = -0.3f; f.fb.theta_e = 0.7f;
    set_ref(&f,0);
    drive_cycle_t cycle; foc_frame_t frame;
    starts=stops=writes=0;
    drive_check_protection(&f);
    drive_update_state(&f);
    assert(f.state == DRIVE_STATE_RUN);
    drive_handle_state(&f,&cycle);
    assert(cycle.ready && writes==0 && starts==0 && stops==0);
    foc_update_current(&f,cycle.cmd.theta,&frame);
    assert(frame.theta == 0.7f && f.fb.id == frame.id && writes==0);
    cycle.valid = foc_execute(&f,&cycle.cmd,&frame);
    assert(cycle.valid && writes==0);
    drive_output(&f,&cycle);
    assert(writes==1 && f.state==DRIVE_STATE_RUN);

    /* 本拍无有效计算，必须停止，不能写入上拍占空比。 */
    drive_check_protection(&f); drive_update_state(&f); drive_handle_state(&f,&cycle);
    cycle.valid = false; drive_output(&f,&cycle);
    assert(!f.pwm_active && f.req==DRIVE_REQ_STOP && writes==1 && stops==1);

    /* 对齐计算失败同样停止且清进度，不留下半次对齐。 */
    f=make_motor(13U); f.pwm_active=true; f.align_ticks=5;
    f.state=DRIVE_STATE_ALIGN; cycle=(drive_cycle_t){.ready=true,.valid=false};
    drive_output(&f,&cycle);
    assert(!f.pwm_active && f.align_ticks==0 && !f.enc_aligned && f.req==DRIVE_REQ_STOP);

    /* PWM 提交层先关断时，对齐进度也必须作废。 */
    f=make_motor(13U); f.pwm_active=true; f.align_ticks=5;
    f.out.duty_a=NAN; f.out.duty_b=0.4f; f.out.duty_c=0.6f;
    f.state=DRIVE_STATE_ALIGN; cycle=(drive_cycle_t){.ready=true,.valid=true};
    drive_output(&f,&cycle);
    assert(!f.pwm_active && f.align_ticks==0 && !f.enc_aligned && !f.pwm_cmd.valid);

    /* 写前发现 NaN 或非法相序，不得写入硬件；停机失败保持 FAULT。 */
    for (unsigned fault=0;fault<3;++fault)
    {
        f=make_motor(3U);f.pwm_active=true;f.req=DRIVE_REQ_RUN;
        f.out.duty_a=0.2f;f.out.duty_b=0.4f;f.out.duty_c=0.6f;
        if (fault==0) f.out.duty_b=NAN;
        if (fault==1) f.motor.phase_order=(phase_order_t)99;
        if (fault==2) { stop_ok=false; f.fault.bit.ov_curr=1; }
        f.state=DRIVE_STATE_RUN; cycle=(drive_cycle_t){.ready=true,.valid=true};
        const unsigned before=writes;
        drive_output(&f,&cycle);
        assert(writes==before && f.req==DRIVE_REQ_STOP);
        if (fault==2) assert(f.state==DRIVE_STATE_FAULT && f.pwm_active);
        else assert(!f.pwm_active);
        stop_ok=true;
    }
    puts("stage contract: PASS (deferred PWM, current frame, invalid calculation, alignment abort, NaN, phase order, stop failure)");
}

/* S3：VF/IF 给定映射——频率斜坡、输出轴→转子换算、虚拟角度积分与回绕。 */
static void check_vf_if_mapping(void)
{
    foc_t f = make_motor(0U);
    foc_plan_t cmd;
    float ramp_before;
    float theta_before;
    unsigned i;

    /* VF：目标 1 rad/s（输出轴）、acc=2 rad/s²、q 轴电压 1.5V */
    f.ref.spd_m = 1.0f;
    f.ref.acc_m = 2.0f;
    f.ref.dec_m = 2.0f;
    f.ref.vq = 1.5f;
    f.ref.vd = 0.0f;
    f.ref.theta_e = 0.0f;

    (void)open_volt_step(&f, &cmd);
    assert(cmd.ref.mode == FOC_CTRL_MODE_VOLT);
    assert(cmd.ref.vq == 1.5f && cmd.ref.vd == 0.0f);
    assert(cmd.theta == f.ref.theta_e);
    /* 第一拍：spd_m_ramp = acc*ts = 1e-4；spd_r = ramp*Gr = 5e-4 */
    assert(fabsf(f.ref.spd_m_ramp - 1.0e-4f) < 1.0e-7f);
    assert(fabsf(f.ref.spd_r - 5.0e-4f) < 1.0e-9f);

    /* 连跑 100 拍：斜坡单调逼近目标，虚拟角度持续前进 */
    theta_before = f.ref.theta_e;
    ramp_before = f.ref.spd_m_ramp;
    for (i = 0; i < 100; ++i)
    {
        (void)open_volt_step(&f, &cmd);
    }
    assert(f.ref.theta_e > theta_before);
    assert(f.ref.spd_m_ramp > ramp_before && f.ref.spd_m_ramp <= 1.0f);

    /* 斜坡饱和：acc=2 rad/s² 到目标 1 rad/s 需 10000 拍（0.5s） */
    for (i = 0; i < 11000; ++i)
    {
        (void)open_volt_step(&f, &cmd);
    }
    assert(f.ref.spd_m_ramp == 1.0f);

    /* 回绕：斜坡满速时每拍推进 0.00175 rad，从 2π 前出发应回到 [0, 2π) */
    f.ref.theta_e = 6.283f;
    (void)open_volt_step(&f, &cmd);
    assert(f.ref.theta_e > 0.0f && f.ref.theta_e < 6.28f);

    /* IF：目标 -5 rad/s、ramp=0 语义（一拍到位）、id=0/iq=2A */
    f = make_motor(0U);
    f.ref.spd_m = -5.0f;
    f.ref.acc_m = 5.0f * f.rate.foc_fs;
    f.ref.dec_m = f.ref.acc_m;
    f.ref.iq = 2.0f;
    f.ref.id = 0.0f;
    (void)open_cur_step(&f, &cmd);
    assert(cmd.ref.mode == FOC_CTRL_MODE_CUR);
    assert(cmd.ref.iq_ref == 2.0f && cmd.ref.id_ref == 0.0f);
    assert(f.ref.spd_m_ramp == -5.0f);
    assert(f.ref.spd_r == -25.0f);
    puts("vf/if mapping: PASS (ramp, unit conversion, theta integration, wrap, instant ramp)");
}
#endif

int main(int argc, char **argv)
{
#ifndef AXDR_BASELINE
    if (argc>1 && strcmp(argv[1],"--stages")==0)
    {
        check_stage_contract();
        check_vf_if_mapping();
        return 0;
    }
#else
    (void)argc; (void)argv;
#endif
    for (unsigned scene=0;scene<20;++scene)
    {
        foc_t f=make_motor(scene); speed_est_init(&f.speed); starts=stops=writes=0;
        duty_a=duty_b=duty_c=0;
        for (unsigned tick=0;tick<256;++tick)
        {
            set_ref(&f,tick);
            if (tick==200U) f.req=DRIVE_REQ_STOP;
            if (scene==18U && tick==80U) f.mode.debug=(debug_mode_e)99;
            if (scene==19U)
            {
                f.mode.sys=release_mode; f.mode.release=pos_mode;
                f.app.pos_ctrl_mode=rel_pos_mode;
                f.app.pos_pause=(tick>=60U && tick<100U);
                f.app.pos_set_by_flag=(tick==160U);
            }
            control_cycle_input_t input={.seq=tick+1,.i_valid=true,.vbus_valid=true,.pos_valid=true,
                .ia=0.4f*sinf(0.03f*(float)tick),.ib=0.2f*cosf(0.03f*(float)tick),.ic=-0.1f,
                .vbus=24.0f,.theta_e=0.002f*(float)tick,.pos_r=0.0001f*(float)tick,
                .pos_m=0.00002f*(float)tick};
            if (scene==13U) input.theta_e=0.4f;
            if (scene==14U && tick==80U) input.ia=10.0f;
            if (scene==15U && tick==80U) input.vbus=5.0f;
            if (scene==16U && tick==80U) input.pos_valid=false;
            control_cycle_output_t output;
            control_cycle_step(&f,&input,&output);
            trace(scene,tick,&f);
        }
    }
    return 0;
}
