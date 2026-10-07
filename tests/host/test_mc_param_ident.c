/* mc_param_ident 主机单测。数值基准移植自 G11 参考工程
 * motor_ident\tests\test_ident_estimators.py（Monte Carlo 真值）：
 *   RS_TRUE=0.069Ω / L_TRUE=80.85µH / VDT_TRUE=0.42V / σ_i=25mA /
 *   DT=50µs / CMD_DELAY=1 / MEAS_DELAY=3 / HALF_PERIOD=6(=300µs)。
 * 被控对象=解析 ZOH 一阶 RL+死区压降（电流过零分段解析，照 py propagate）
 * +机械环（磁链链）；上拍 command 回灌下拍 sample（命令延迟 1 拍，
 * 与估计器的 PWM 双缓冲管线补偿假设一致）。 */

#include "mc_param_ident.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int g_failures;

#define CHECK_TRUE(expr) do {                                                \
    if (!(expr)) {                                                          \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr);             \
        g_failures++;                                                       \
    }                                                                       \
} while (0)

#define CHECK_NEAR(actual, expected, tolerance) do {                        \
    const double a_ = (double)(actual);                                     \
    const double e_ = (double)(expected);                                   \
    const double t_ = (double)(tolerance);                                  \
    if (fabs(a_ - e_) > t_) {                                              \
        printf("FAIL %s:%d: %.9g != %.9g (tol %.3g)\n",                   \
               __FILE__, __LINE__, a_, e_, t_);                             \
        g_failures++;                                                       \
    }                                                                       \
} while (0)

/* =============================================================================
 * 随机数：xorshift32 + Box-Muller 高斯
 * ============================================================================ */

static uint32_t s_rng;

static void rng_seed(uint32_t seed)
{
    s_rng = seed ? seed : 0x9E3779B9u;
}

static uint32_t rng_u32(void)
{
    uint32_t x = s_rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return s_rng = x;
}

static float rng_gauss(void)
{
    const float u1 = ((rng_u32() >> 8) + 0.5f) / 16777216.0f;
    const float u2 = (float)(rng_u32() >> 8) / 16777216.0f;
    return sqrtf(-2.0f * logf(u1)) * cosf(6.2831853f * u2);
}

/* =============================================================================
 * 被控对象（真值基准见文件头）
 * ============================================================================ */

#define PLANT_RS_TRUE    (0.069f)
#define PLANT_L_TRUE     (80.85e-6f)
#define PLANT_VDT_TRUE   (0.42f)
#define PLANT_FLUX_TRUE  (0.0058f)
#define PLANT_DT         (50e-6f)
#define PLANT_THETA0     (0.7f)     /* 非零锁存角：验证 openloop 角不变量 */
/* 3/π ≈ 0.9549：旋转几何下电流矢量方向的逆变器死区等效压降
 * (4/π)·v_phase 与锁轴 per-axis 等效 (4/3)·v_phase 之比。镜像核
 * call site 的 MC_IDENT_FLUX_VDT_DQ_SCALE：L 段（VOLTAGE 模式锁轴）
 * 解出 per-axis 值，FLUX 段（CURRENT 模式旋转）毛电压携带 ×本因子
 * 的分量——host 由此验证换算机制自洽，因子本身靠真机
 * DEADTIME_TEST 交叉验证。 */
#define PLANT_VDT_ROT_SCALE (0.9549296586f)

typedef struct {
    float rs, ld, lq, vdt, flux, pp;
    float j_kg_m2, b_nm_s;          /* 机械惯量 / 粘滞摩擦 */
    float tau_cl;                   /* 电流环等效一阶时间常数 */
    float id, iq;
    float id_prev, iq_prev;         /* 上拍电流（di/dt 重构） */
    float omega_mech, theta_e;
    float vd_actual, vq_actual;
} plant_t;

static float plant_signf(float x)
{
    return (x >= 0.0f) ? 1.0f : -1.0f;
}

/* 一个 ZOH 拍内的解析演化：u 恒定、死区压降按电流符号分段，
 * 电流过零时刻解析求根后切换极性续算（|u|<=vdt 时死区锁止于 0）。 */
static float plant_step_axis(float i, float u, float dt,
                             float rs, float l_h, float vdt)
{
    const float tau = l_h / rs;
    float t_rem = dt;
    int seg;

    for (seg = 0; seg < 3 && t_rem > 0.0f; ++seg) {
        const float s = plant_signf(i);
        const float i_ss = (u - vdt * s) / rs;
        if (i * i_ss < 0.0f) {
            const float t_cross = tau * logf((i - i_ss) / (-i_ss));
            if (t_cross > 0.0f && t_cross < t_rem) {
                if (fabsf(u) <= vdt) return 0.0f; /* 死区锁止 */
                i = 0.0f;
                t_rem -= t_cross;
                continue;
            }
        }
        i = i_ss + (i - i_ss) * expf(-t_rem / tau);
        t_rem = 0.0f;
    }
    return i;
}

static void plant_reset(plant_t *p)
{
    memset(p, 0, sizeof(*p));
    p->rs = PLANT_RS_TRUE;
    p->ld = PLANT_L_TRUE;
    p->lq = PLANT_L_TRUE;
    p->vdt = PLANT_VDT_TRUE;
    p->flux = PLANT_FLUX_TRUE;
    p->pp = 5.0f;
    p->j_kg_m2 = 0.002f;
    p->b_nm_s = 0.002f;
    p->tau_cl = 2e-4f;
    p->theta_e = PLANT_THETA0;
}

static void plant_tick(plant_t *p, const mc_command_t *cmd, float dt)
{
    const float omega_e = p->pp * p->omega_mech;

    p->id_prev = p->id;
    p->iq_prev = p->iq;

    if (cmd->mode == MC_CONTROL_CURRENT) {
        /* 完美电流环：一阶跟踪指令；回报电压=毛电压（PI 输出含顶掉
         * 逆变器死区扰动的分量，与固件 build_sample 取 foc->out.vd 的
         * 语义一致）。死区沿电流矢量方向的等效分量按运动状态分两档：
         * 静止锁轴（PREPARE/RS）= vdt（锁轴几何，与 L 段 VOLTAGE 注入
         * 同语义）；旋转（FLUX 拖转）= vdt×3/π（旋转几何基波等效）。
         * 两档比值即核 call site 3/π 换算的物理依据；host 由此验证换
         * 算机制自洽，因子本身靠真机 DEADTIME_TEST 交叉验证。Rs 阶梯
         * 各档电流同号，死区分量在差分中共模消掉，不影响电阻。 */
        const float a = 1.0f - expf(-dt / p->tau_cl);
        const float vdt_cur = p->vdt *
            ((fabsf(p->omega_mech) > 1.0f) ? PLANT_VDT_ROT_SCALE : 1.0f);
        p->id += (cmd->id_ref_a - p->id) * a;
        p->iq += (cmd->iq_ref_a - p->iq) * a;
        p->vd_actual = p->rs * p->id + p->ld * (p->id - p->id_prev) / dt
                       - omega_e * p->lq * p->iq
                       + vdt_cur * plant_signf(p->id);
        p->vq_actual = p->rs * p->iq + p->lq * (p->iq - p->iq_prev) / dt
                       + omega_e * (p->ld * p->id + p->flux)
                       + vdt_cur * plant_signf(p->iq);
    } else {
        p->id = plant_step_axis(p->id, cmd->vd_ref_v, dt, p->rs, p->ld, p->vdt);
        p->iq = plant_step_axis(p->iq, cmd->vq_ref_v, dt, p->rs, p->lq, p->vdt);
        p->vd_actual = cmd->vd_ref_v; /* 驱动回报电压=命令值（无测量噪声） */
        p->vq_actual = cmd->vq_ref_v;
    }

    {
        const float kt = 1.5f * p->pp * p->flux;
        p->omega_mech += (kt * p->iq - p->b_nm_s * p->omega_mech)
                         / p->j_kg_m2 * dt;
        p->theta_e += omega_e * dt;
    }
}

static mc_sample_t plant_sample(const plant_t *p, float noise_a)
{
    mc_sample_t s;
    memset(&s, 0, sizeof(s));
    s.ia_a = p->id + noise_a * rng_gauss();
    s.ib_a = p->iq + noise_a * rng_gauss();
    s.ic_a = noise_a * rng_gauss();
    s.i_alpha_a = s.ia_a;
    s.id_a = p->id + noise_a * rng_gauss();
    s.iq_a = p->iq + noise_a * rng_gauss();
    s.vd_v = p->vd_actual;
    s.vq_v = p->vq_actual;
    s.theta_mech_rad = p->theta_e / p->pp;
    s.theta_elec_rad = p->theta_e;
    s.omega_mech_rad_s = p->omega_mech;
    s.vbus_v = 24.0f;
    s.dt_s = PLANT_DT;
    s.current_limit_a = 15.0f;
    return s;
}

/* =============================================================================
 * 回调记录与链路运行器
 * ============================================================================ */

typedef struct {
    int on_start_cnt;
    int on_finish_cnt;
    int apply_cnt;
    int finish_success;
    mc_param_ident_fault_t finish_fault;
} ident_cb_rec_t;

static ident_cb_rec_t g_cb;
static mc_command_t g_last_cmd;

static void cb_reset(void)
{
    memset(&g_cb, 0, sizeof(g_cb));
}

static void cb_on_start(void *user)
{
    (void)user;
    g_cb.on_start_cnt++;
}

static void cb_on_finish(bool success, mc_param_ident_fault_t fault, void *user)
{
    (void)user;
    g_cb.on_finish_cnt++;
    g_cb.finish_success = success ? 1 : 0;
    g_cb.finish_fault = fault;
}

static void cb_apply(const mc_param_ident_result_t *result, void *user)
{
    (void)result;
    (void)user;
    g_cb.apply_cnt++;
}

typedef enum {
    INJECT_NONE = 0,
    INJECT_VBUS_LOW,
    INJECT_FOC_FAULT,
    INJECT_NAN_CURRENT,
    INJECT_SPINNING
} inject_case_t;

/* 快速时序配置：窗口结构参数与 py 基准一致（toggle=300µs=6 拍），
 * 只压缩平台/采样时长以控制 host 用例耗时。 */
static void fast_config(mc_param_ident_config_t *cfg)
{
    mc_param_ident_default_config(cfg);
    cfg->control_period_s = PLANT_DT;
    cfg->pole_pairs = 5.0f;
    cfg->current_limit_a = 15.0f;
    cfg->voltage_limit_v = 24.0f;
    cfg->rs_current_lo_a = 4.0f;   /* py 基准阶梯端点（六档 4→10A） */
    cfg->rs_current_hi_a = 10.0f;
    cfg->align_time_s = 0.12f;     /* 覆盖 4A@50A/s 软启斜坡 */
    cfg->rs_settle_s = 0.15f;      /* 覆盖档间斜坡（ΔI=1.2A→24ms） */
    cfg->rs_sample_s = 0.1f;
    cfg->l_settle_s = 8e-3f;
    cfg->l_sample_s = 0.1f;
    cfg->l_inject_lo_v = 1.5f;
    cfg->l_inject_hi_v = 3.0f;
    cfg->flux_spinup_s = 0.6f;
    cfg->flux_sample_s = 0.2f;
    /* 默认档案已回填真机标称（Rs 0.118/L 100µ）；plant 是 0.069Ω 假电机，
     * 偏差门必须清零隔离——host 测核算法，不测沉沙档案匹配。 */
    cfg->nominal_r = 0.0f;
    cfg->tol_r = 0.0f;
    cfg->nominal_l = 0.0f;
    cfg->tol_l = 0.0f;
    cfg->nominal_flux = 0.0f;
    cfg->tol_flux = 0.0f;
}

static mc_status_t run_chain(mc_param_ident_t *ident, plant_t *p,
                             const mc_param_ident_config_t *cfg,
                             mc_param_ident_mode_t mode, float noise_a,
                             uint32_t seed, inject_case_t inject,
                             mc_param_ident_state_t stop_at,
                             int check_invariants, uint32_t *ticks_used)
{
    mc_param_ident_io_t io;
    mc_command_t cmd, applied;
    mc_sample_t s;
    mc_status_t st = MC_BUSY;
    uint32_t t = 0u;

    memset(ident, 0, sizeof(*ident)); /* ctx 契约：零初始化 */
    memset(&io, 0, sizeof(io));
    io.on_start = cb_on_start;
    io.on_finish = cb_on_finish;
    io.apply_results = cb_apply;
    mc_param_ident_attach(ident, &io);

    memset(&cmd, 0, sizeof(cmd));
    memset(&applied, 0, sizeof(applied));
    cb_reset();

    st = mc_param_ident_start(ident, cfg, mode);
    if (st != MC_OK) return st;

    rng_seed(seed);
    for (t = 0u; t < 400000u; ++t) {
        plant_tick(p, &applied, PLANT_DT);
        s = plant_sample(p, noise_a);

        switch (inject) {
        case INJECT_VBUS_LOW:
            if (mc_param_ident_get_state(ident) == MC_PARAM_IDENT_RS_SAMPLE) {
                s.vbus_v = 0.5f;
            }
            break;
        case INJECT_FOC_FAULT:
            if (mc_param_ident_get_state(ident) == MC_PARAM_IDENT_RS_SAMPLE) {
                s.fault_code = 0x1u;
            }
            break;
        case INJECT_NAN_CURRENT:
            if (mc_param_ident_get_state(ident) == MC_PARAM_IDENT_RS_SAMPLE) {
                s.id_a = NAN;
            }
            break;
        case INJECT_SPINNING:
            if (mc_param_ident_get_state(ident) == MC_PARAM_IDENT_L_SAMPLE) {
                s.omega_mech_rad_s = 10.0f; /* pp=5 → 477eRPM，超停转门限 */
            }
            break;
        case INJECT_NONE:
        default:
            break;
        }

        st = mc_param_ident_step(ident, &s, &cmd);
        applied = cmd;

        if (check_invariants) {
            if (st == MC_BUSY) {
                CHECK_TRUE(cmd.enable_request);
                CHECK_TRUE(!cmd.disable_request);
                CHECK_TRUE(cmd.mode == MC_CONTROL_CURRENT ||
                           cmd.mode == MC_CONTROL_VOLTAGE);
                if (cmd.mode == MC_CONTROL_CURRENT) {
                    CHECK_NEAR(cmd.vd_ref_v, 0.0f, 0.0f);
                    CHECK_NEAR(cmd.vq_ref_v, 0.0f, 0.0f);
                } else {
                    CHECK_NEAR(cmd.id_ref_a, 0.0f, 0.0f);
                    CHECK_NEAR(cmd.iq_ref_a, 0.0f, 0.0f);
                }
                if (cmd.openloop_enable) {
                    CHECK_NEAR(cmd.openloop_theta_e_rad, PLANT_THETA0, 1e-6f);
                }
            } else {
                CHECK_TRUE(cmd.disable_request);
            }
        }

        if (st != MC_BUSY) break;
        if (stop_at != MC_PARAM_IDENT_IDLE &&
            mc_param_ident_get_state(ident) == stop_at) break;
    }
    g_last_cmd = cmd;
    if (ticks_used) *ticks_used = t;
    return st;
}

static double mean_of(const double *v, int n)
{
    double sum = 0.0;
    int i;
    for (i = 0; i < n; ++i) sum += v[i];
    return sum / (double)n;
}

static double pstdev_of(const double *v, int n)
{
    const double m = mean_of(v, n);
    double sum = 0.0;
    int i;
    for (i = 0; i < n; ++i) sum += (v[i] - m) * (v[i] - m);
    return sqrt(sum / (double)n);
}

/* =============================================================================
 * 1. 配置与闸门
 * ============================================================================ */

static void test_config_gates(void)
{
    mc_param_ident_config_t cfg;
    mc_param_ident_t ident;
    mc_command_t cmd;
    mc_sample_t sample;
    uint16_t progress = 0u;

    /* ctx 契约：使用前必须零初始化（io 回调域非法指针会崩）。 */
    memset(&ident, 0, sizeof(ident));

    /* 默认值抽查：算法层=G11 原值，激励层=沉沙档案，安全闸门=关。 */
    mc_param_ident_default_config(&cfg);
    CHECK_NEAR(cfg.pole_pairs, 5.0f, 0.0f);
    CHECK_NEAR(cfg.rs_current_lo_a, 2.0f, 0.0f);
    CHECK_NEAR(cfg.rs_current_hi_a, 6.0f, 0.0f); /* 六点阶梯末档≤0.8×8A 闸 */
    CHECK_NEAR(cfg.l_inject_lo_v, 1.5f, 0.0f);
    CHECK_NEAR(cfg.l_inject_hi_v, 3.0f, 0.0f);
    CHECK_NEAR(cfg.l_toggle_s, 0.3e-3f, 0.0f);
    CHECK_NEAR(cfg.l_fit_min_r2, 0.80f, 0.0f);
    CHECK_NEAR(cfg.l_level_tolerance, 0.35f, 0.0f);
    CHECK_NEAR(cfg.vbus_min_v, 1.0f, 0.0f);
    CHECK_NEAR(cfg.current_limit_a, 0.0f, 0.0f); /* 闸门：未确认禁止启动 */
    CHECK_NEAR(cfg.voltage_limit_v, 0.0f, 0.0f);
    /* 真机标称档案（2026-10-07 回填）；磁链门暂缓（低 SNR）。 */
    CHECK_NEAR(cfg.nominal_r, 0.118f, 0.0f);
    CHECK_NEAR(cfg.tol_r, 0.12f, 0.0f);
    CHECK_NEAR(cfg.nominal_l, 1.00e-4f, 0.0f);
    CHECK_NEAR(cfg.tol_l, 0.25f, 0.0f);
    CHECK_NEAR(cfg.nominal_flux, 0.0f, 0.0f);

    /* 闸门未开（默认 limit=0）拒绝。 */
    CHECK_TRUE(mc_param_ident_start(&ident, NULL, MC_PARAM_IDENT_MODE_LQ)
               == MC_INVALID_ARGUMENT);
    CHECK_TRUE(mc_param_ident_start(&ident, &cfg, MC_PARAM_IDENT_MODE_LQ)
               == MC_INVALID_ARGUMENT);

    fast_config(&cfg);
    CHECK_TRUE(mc_param_ident_start(&ident, &cfg, MC_PARAM_IDENT_MODE_LQ)
               == MC_OK);
    /* 运行中重复 start 拒绝。 */
    CHECK_TRUE(mc_param_ident_start(&ident, &cfg, MC_PARAM_IDENT_MODE_LQ)
               == MC_REJECTED);
    /* 主动中止 → 独立终态，step 返回 MC_ABORTED。 */
    CHECK_TRUE(mc_param_ident_abort(&ident, &cmd) == MC_ABORTED);
    CHECK_TRUE(cmd.disable_request);
    CHECK_TRUE(mc_param_ident_step(&ident, &sample, &cmd) == MC_ABORTED);
    /* 终态后可重启。 */
    CHECK_TRUE(mc_param_ident_start(&ident, &cfg, MC_PARAM_IDENT_MODE_LQ)
               == MC_OK);
    (void)mc_param_ident_abort(&ident, &cmd);

    /* 非法参数：周期/极对/限幅/模式/NaN。 */
    CHECK_TRUE(mc_param_ident_start(&ident, &cfg, (mc_param_ident_mode_t)99)
               == MC_INVALID_ARGUMENT);
    {
        mc_param_ident_config_t bad = cfg;
        bad.control_period_s = 0.0f;
        CHECK_TRUE(mc_param_ident_start(&ident, &bad,
                                        MC_PARAM_IDENT_MODE_LQ)
                   == MC_INVALID_ARGUMENT);
        bad = cfg;
        bad.pole_pairs = 0.0f;
        CHECK_TRUE(mc_param_ident_start(&ident, &bad,
                                        MC_PARAM_IDENT_MODE_LQ)
                   == MC_INVALID_ARGUMENT);
        bad = cfg;
        bad.current_limit_a = -1.0f;
        CHECK_TRUE(mc_param_ident_start(&ident, &bad,
                                        MC_PARAM_IDENT_MODE_LQ)
                   == MC_INVALID_ARGUMENT);
        bad = cfg;
        bad.test_current_a = NAN; /* 范围检查挡不住 NaN（14 号政策） */
        CHECK_TRUE(mc_param_ident_start(&ident, &bad,
                                        MC_PARAM_IDENT_MODE_LQ)
                   == MC_INVALID_ARGUMENT);
    }

    /* IDLE 态 step 拒绝；进度查询。 */
    memset(&ident, 0, sizeof(ident));
    memset(&sample, 0, sizeof(sample));
    CHECK_TRUE(mc_param_ident_step(&ident, &sample, &cmd) == MC_REJECTED);
    progress = mc_param_ident_get_progress_permille(&ident);
    CHECK_TRUE(progress == 0u);
    /* 空指针安全。 */
    CHECK_TRUE(mc_param_ident_get_state(NULL) == MC_PARAM_IDENT_IDLE);
    CHECK_TRUE(mc_param_ident_get_result(NULL) == NULL);
    CHECK_TRUE(mc_param_ident_get_progress_permille(NULL) == 0u);
    CHECK_TRUE(!mc_param_ident_save(&ident));   /* 无 io.save 回调 */
    CHECK_TRUE(!mc_param_ident_load(&ident));   /* 无 io.load 回调 */
    CHECK_TRUE(mc_param_ident_apply(&ident) == MC_REJECTED); /* 无 apply */
}

/* =============================================================================
 * 2. 纯估计器：Rs 阶梯最小二乘 / sign 平均 / L 回归求解 / 磁链公式
 * ============================================================================ */

static void test_pure_estimators(void)
{
    mc_param_ident_l_reg_t reg;
    mc_param_ident_l_gate_t gate;
    float rs = 0.0f, l_h = 0.0f, vdt = 0.0f, r2 = 0.0f;
    int k;

    /* Rs 六点阶梯最小二乘：v=Rs·i+Voff，常量偏置（死区/管压降/ADC 偏置）
     * 在 C(6,2)=15 点对差分中消去，精确恢复 Rs。 */
    {
        static const float im[6] = { 2.0f, 2.8f, 3.6f, 4.4f, 5.2f, 6.0f };
        float vm[6];
        for (k = 0; k < 6; ++k) vm[k] = 0.069f * im[k] + 0.42f;
        CHECK_TRUE(mc_param_ident_solve_rs_ls(vm, im, 6u, &rs) == MC_OK);
        CHECK_NEAR(rs, 0.069f, 1e-5f);

        /* 两点退化=旧双平台差分同值。 */
        {
            const float iv[2] = { 4.0f, 10.0f };
            const float vv[2] = { 0.696f, 1.110f };
            CHECK_TRUE(mc_param_ident_solve_rs_ls(vv, iv, 2u, &rs) == MC_OK);
            CHECK_NEAR(rs, 0.069f, 1e-6f);
        }
        /* 档间电流无差异（ΣΔI²≈0）拒绝。 */
        {
            const float iv[2] = { 4.0f, 4.0f };
            const float vv[2] = { 0.696f, 0.696f };
            CHECK_TRUE(mc_param_ident_solve_rs_ls(vv, iv, 2u, &rs)
                       == MC_OUT_OF_RANGE);
        }
        /* 参数防线：n<2 / 空指针 / 非有限输入。 */
        CHECK_TRUE(mc_param_ident_solve_rs_ls(vm, im, 1u, &rs)
                   == MC_INVALID_ARGUMENT);
        CHECK_TRUE(mc_param_ident_solve_rs_ls(NULL, im, 6u, &rs)
                   == MC_INVALID_ARGUMENT);
        vm[0] = NAN;
        CHECK_TRUE(mc_param_ident_solve_rs_ls(vm, im, 6u, &rs)
                   == MC_NUMERIC_ERROR);
    }

    /* sign 平均：过零线性插值 / 同号直通 / 零。 */
    CHECK_NEAR(mc_param_ident_sign_average(2.0f, -1.0f), 1.0f / 3.0f, 1e-7f);
    CHECK_NEAR(mc_param_ident_sign_average(-1.0f, 2.0f), 1.0f / 3.0f, 1e-7f);
    CHECK_NEAR(mc_param_ident_sign_average(2.0f, 3.0f), 1.0f, 0.0f);
    CHECK_NEAR(mc_param_ident_sign_average(-1.0f, -2.0f), -1.0f, 0.0f);
    CHECK_NEAR(mc_param_ident_sign_average(0.0f, 0.0f), 0.0f, 0.0f);

    /* L 联合回归：合成 y=L·x+VDT·z 窗口（x/z 双极变化），解应精确恢复。 */
    memset(&reg, 0, sizeof(reg));
    gate.min_di_a = 0.05f;
    gate.nominal_l = 0.0f;
    for (k = 0; k < 120; ++k) {
        const float dt_meas = 3.0f * PLANT_DT;
        const float x = ((k % 2) ? 1.0f : -1.0f) * (8000.0f + 137.0f * (float)k);
        const float z = ((k % 3) ? 1.0f : -1.0f) * (0.2f + 0.006f * (float)(k % 17));
        const float y = PLANT_L_TRUE * x + PLANT_VDT_TRUE * z;
        /* i1-i0 = x·dt 使内部 x=di/dt 精确合成。 */
        mc_param_ident_l_reg_add(&reg, &gate, y, 0.0f, x * dt_meas,
                                 1.0f, z, dt_meas);
    }
    CHECK_TRUE(reg.rejected == 0u);
    CHECK_TRUE(mc_param_ident_l_reg_solve(&reg, &l_h, &vdt, &r2));
    CHECK_NEAR(l_h, PLANT_L_TRUE, 0.05e-6f);
    CHECK_NEAR(vdt, PLANT_VDT_TRUE, 1e-4f);
    CHECK_NEAR(r2, 1.0f, 1e-9f);

    /* 档内解：用联合 VDT 在单档仅拟合 L。 */
    CHECK_TRUE(mc_param_ident_l_reg_solve_level(&reg, vdt, &l_h));
    CHECK_NEAR(l_h, PLANT_L_TRUE, 0.1e-6f);

    /* 退化路径：单极样本不足 24 → 解失败。 */
    memset(&reg, 0, sizeof(reg));
    for (k = 0; k < 120; ++k) {
        const float dt_meas = 3.0f * PLANT_DT;
        const float x = 8000.0f + 137.0f * (float)k;
        const float z = 0.5f;
        const float y = PLANT_L_TRUE * x + PLANT_VDT_TRUE * z;
        mc_param_ident_l_reg_add(&reg, &gate, y, 0.0f, x * dt_meas,
                                 1.0f, z, dt_meas);
    }
    CHECK_TRUE(!mc_param_ident_l_reg_solve(&reg, &l_h, &vdt, &r2));

    /* 门控拒绝：di 过小 / u·di 反号 → 只计 rejected。 */
    memset(&reg, 0, sizeof(reg));
    mc_param_ident_l_reg_add(&reg, &gate, 1.0f, 0.0f, 1e-4f, 0.0f, 0.0f,
                             3.0f * PLANT_DT);              /* di<min */
    mc_param_ident_l_reg_add(&reg, &gate, -1.5f, 0.0f, 0.5f, 0.0f, 0.5f,
                             3.0f * PLANT_DT);              /* u·di<=0 */
    CHECK_TRUE(reg.accepted == 0u);
    CHECK_TRUE(reg.rejected == 2u);
}

static void test_flux_estimator_analytic(void)
{
    mc_param_ident_flux_eval_input_t in;
    float lambda = 0.0f;

    memset(&in, 0, sizeof(in));
    in.vq = 1.0f;
    in.iq = 0.5f;
    in.id = 0.0f;
    in.omega_e = 10.0f;
    in.phase_resistance = 0.069f;
    in.phase_inductance = 80.85e-6f;
    /* λ=(vq−Rs·iq)/ωe=(1−0.0345)/10=0.09655 */
    CHECK_TRUE(mc_param_ident_eval_flux_sample(&in, &lambda));
    CHECK_NEAR(lambda, 0.09655f, 1e-6f);

    /* id 项：λ 再扣 Ld·id。 */
    in.id = 1.0f;
    CHECK_TRUE(mc_param_ident_eval_flux_sample(&in, &lambda));
    CHECK_NEAR(lambda, 0.09655f - 80.85e-6f, 1e-9f);
    in.id = 0.0f;

    /* 反转符号一致性（不能用绝对值：再生/反转会系统性错）。 */
    in.vq = -1.0f;
    in.iq = -0.5f;
    in.omega_e = -10.0f;
    CHECK_TRUE(mc_param_ident_eval_flux_sample(&in, &lambda));
    CHECK_NEAR(lambda, 0.09655f, 1e-6f);

    /* 拒绝路径：|ωe| 过小、λ<=0。 */
    in.omega_e = 5.0f; /* 门限含等号 */
    CHECK_TRUE(!mc_param_ident_eval_flux_sample(&in, &lambda));
    in.omega_e = 10.0f;
    in.vq = 0.05f;
    in.iq = 1.0f; /* (0.05−0.069)/10 = −0.0019 < 0 */
    CHECK_TRUE(!mc_param_ident_eval_flux_sample(&in, &lambda));
}

/* =============================================================================
 * 3. Rs 六点阶梯（噪声链，到 L 入口为止）
 * ============================================================================ */

static void test_rs_staircase(void)
{
    mc_param_ident_config_t cfg;
    mc_param_ident_t ident;
    plant_t plant;
    const float sigma_a = 0.025f; /* py 基准 σ_i=25mA */

    fast_config(&cfg);
    plant_reset(&plant);
    CHECK_TRUE(run_chain(&ident, &plant, &cfg, MC_PARAM_IDENT_MODE_LQ, sigma_a,
                         7u, INJECT_NONE, MC_PARAM_IDENT_L_SAMPLE, 0, NULL)
               == MC_BUSY);
    CHECK_TRUE(mc_param_ident_get_state(&ident) == MC_PARAM_IDENT_L_SAMPLE);
    /* 六点阶梯点对差分消死区：|err|<1%（py 断言）。 */
    CHECK_NEAR(ident.result.phase_resistance, PLANT_RS_TRUE,
               0.01f * PLANT_RS_TRUE);
    {
        mc_param_ident_quality_t q;
        mc_param_ident_get_quality(&ident, &q);
        CHECK_NEAR(q.rs_current_lo_a, 4.0f, 0.05f);
        CHECK_NEAR(q.rs_current_hi_a, 10.0f, 0.05f);
        /* 阶梯端点毛电压 = R·i + 死区锁轴等效分量（差分共模消掉）。 */
        CHECK_NEAR(q.rs_voltage_lo_v,
                   0.069f * 4.0f + PLANT_VDT_TRUE, 0.01f);
        CHECK_NEAR(q.rs_voltage_hi_v,
                   0.069f * 10.0f + PLANT_VDT_TRUE, 0.01f);
    }
}

/* =============================================================================
 * 4. 命令不变量（LQ 全链逐拍）
 * ============================================================================ */

static void test_command_invariants(void)
{
    mc_param_ident_config_t cfg;
    mc_param_ident_t ident;
    plant_t plant;

    fast_config(&cfg);
    plant_reset(&plant);
    CHECK_TRUE(run_chain(&ident, &plant, &cfg, MC_PARAM_IDENT_MODE_LQ, 0.0f,
                         11u, INJECT_NONE, MC_PARAM_IDENT_IDLE, 1, NULL)
               == MC_DONE);
    CHECK_TRUE(g_last_cmd.disable_request);
}

/* =============================================================================
 * 5. LQ 全链 Monte Carlo（py 基准移植：100 seed）
 * ============================================================================ */

#define MC_SEEDS (100)

static void test_full_chain_lq_monte_carlo(void)
{
    mc_param_ident_config_t cfg;
    mc_param_ident_t ident;
    plant_t plant;
    double ld_est[MC_SEEDS], lq_est[MC_SEEDS], rs_est[MC_SEEDS];
    int seed;
    int all_done = 1;
    int all_gates = 1;

    fast_config(&cfg);
    for (seed = 0; seed < MC_SEEDS; ++seed) {
        mc_param_ident_quality_t q;
        plant_reset(&plant);
        if (run_chain(&ident, &plant, &cfg, MC_PARAM_IDENT_MODE_LQ, 0.025f,
                      (uint32_t)(seed + 1u), INJECT_NONE,
                      MC_PARAM_IDENT_IDLE, 0, NULL) != MC_DONE) {
            all_done = 0;
            continue;
        }
        mc_param_ident_get_quality(&ident, &q);
        rs_est[seed] = (double)ident.result.phase_resistance;
        ld_est[seed] = (double)ident.result.phase_inductance;
        lq_est[seed] = (double)ident.result.phase_inductance_q;
        /* 每种子质量门（py：R²>0.80、档间<0.35、Rs<1%）。 */
        if (!(q.ld.fit_r2 > 0.80f) || !(q.lq.fit_r2 > 0.80f) ||
            !(q.ld.level_delta < 0.35f) || !(q.lq.level_delta < 0.35f) ||
            fabs((double)ident.result.phase_resistance - PLANT_RS_TRUE)
                > 0.01 * PLANT_RS_TRUE) {
            all_gates = 0;
        }
    }
    CHECK_TRUE(all_done);
    CHECK_TRUE(all_gates);

    /* 聚合统计（py：|mean_L−L|/L<3%、pstdev/L<1%）。 */
    CHECK_NEAR(mean_of(ld_est, MC_SEEDS), PLANT_L_TRUE, 0.03 * PLANT_L_TRUE);
    CHECK_NEAR(mean_of(lq_est, MC_SEEDS), PLANT_L_TRUE, 0.03 * PLANT_L_TRUE);
    CHECK_TRUE(pstdev_of(ld_est, MC_SEEDS) / PLANT_L_TRUE < 0.01);
    CHECK_TRUE(pstdev_of(lq_est, MC_SEEDS) / PLANT_L_TRUE < 0.01);
    CHECK_NEAR(mean_of(rs_est, MC_SEEDS), PLANT_RS_TRUE, 0.01 * PLANT_RS_TRUE);
}

/* =============================================================================
 * 6. FULL 全链（无噪声）：磁链/Ke/Kt 精确恢复 + 回调契约
 * ============================================================================ */

static void test_full_chain_flux(void)
{
    mc_param_ident_config_t cfg;
    mc_param_ident_t ident;
    plant_t plant;
    const mc_param_ident_result_t *result;
    uint32_t ticks = 0u;

    fast_config(&cfg);
    plant_reset(&plant);
    CHECK_TRUE(run_chain(&ident, &plant, &cfg, MC_PARAM_IDENT_MODE_FULL, 0.0f,
                         3u, INJECT_NONE, MC_PARAM_IDENT_IDLE, 1, &ticks)
               == MC_DONE);
    result = mc_param_ident_get_result(&ident);
    CHECK_TRUE(result != NULL);
    CHECK_TRUE(result->fault_code == MC_PARAM_IDENT_FAULT_NONE);

    /* 磁链 1%；Ke/Kt 换算公式互锁。 */
    CHECK_NEAR(result->flux_linkage_wb, PLANT_FLUX_TRUE, 0.01f * PLANT_FLUX_TRUE);
    CHECK_NEAR(result->back_emf_v_per_rad,
               PLANT_FLUX_TRUE * 5.0f * 1.2247448714f, 0.01f * PLANT_FLUX_TRUE * 5.0f);
    CHECK_NEAR(result->torque_kt_nm_a, 1.5f * 5.0f * PLANT_FLUX_TRUE,
               0.01f * 1.5f * 5.0f * PLANT_FLUX_TRUE);
    /* Rs/Ld 同链恢复。 */
    CHECK_NEAR(result->phase_resistance, PLANT_RS_TRUE, 0.01f * PLANT_RS_TRUE);
    CHECK_NEAR(result->phase_inductance, PLANT_L_TRUE, 0.03f * PLANT_L_TRUE);

    /* 回调契约：on_start 恰 1 次、apply 恰 1 次、on_finish(true) 恰 1 次。 */
    CHECK_TRUE(g_cb.on_start_cnt == 1);
    CHECK_TRUE(g_cb.apply_cnt == 1);
    CHECK_TRUE(g_cb.on_finish_cnt == 1);
    CHECK_TRUE(g_cb.finish_success == 1);
    CHECK_TRUE(g_cb.finish_fault == MC_PARAM_IDENT_FAULT_NONE);
    /* 终态契约：DONE、进度满、停机命令。 */
    CHECK_TRUE(mc_param_ident_get_state(&ident) == MC_PARAM_IDENT_DONE);
    CHECK_TRUE(mc_param_ident_get_progress_permille(&ident) == 1000u);
    CHECK_TRUE(g_last_cmd.disable_request);
    CHECK_TRUE(ticks > 0u);
    {
        mc_param_ident_quality_t q;
        mc_param_ident_get_quality(&ident, &q);
        CHECK_TRUE(q.flux_std_wb < 0.20f * PLANT_FLUX_TRUE);
        CHECK_TRUE(q.flux_speed_mean_erpm > 0.70f * 900.0f);
        CHECK_TRUE(q.flux_speed_mean_erpm < 1.30f * 900.0f);
        CHECK_NEAR(q.ld.estimate_lo_h, q.ld.estimate_hi_h,
                   0.35f * PLANT_L_TRUE);
    }
}

/* =============================================================================
 * 6b. 磁链死区修正（毛电压 plant；估计器单点 + 全链扫描 + 零死区）
 * ============================================================================ */

static void test_flux_deadtime_correction(void)
{
    /* 单点对照：毛电压 vq 输入，不修正 λ̂ 偏 +vdt_rot/ωe（沉沙量级
     * ≈ +77%），修正后精确恢复——符号/量级错误在此露馅。 */
    {
        mc_param_ident_flux_eval_input_t in;
        float lambda = 0.0f;
        const float omega_e = 5.0f * (900.0f / 60.0f * 6.2831853f); /* 900eRPM */
        const float vdt_rot = PLANT_VDT_TRUE * PLANT_VDT_ROT_SCALE;

        memset(&in, 0, sizeof(in));
        in.iq = 2.0f;
        in.id = 0.1f;
        in.omega_e = omega_e;
        in.phase_resistance = PLANT_RS_TRUE;
        in.phase_inductance = PLANT_L_TRUE;
        in.vq = PLANT_RS_TRUE * in.iq
                + omega_e * (PLANT_L_TRUE * in.id + PLANT_FLUX_TRUE)
                + vdt_rot; /* 毛电压（PI 输出含死区顶掉分量） */

        in.deadtime_v = 0.0f;
        CHECK_TRUE(mc_param_ident_eval_flux_sample(&in, &lambda));
        CHECK_NEAR(lambda, PLANT_FLUX_TRUE + vdt_rot / omega_e,
                   0.02f * PLANT_FLUX_TRUE);
        in.deadtime_v = vdt_rot;
        CHECK_TRUE(mc_param_ident_eval_flux_sample(&in, &lambda));
        CHECK_NEAR(lambda, PLANT_FLUX_TRUE, 0.01f * PLANT_FLUX_TRUE);
    }

    /* 全链扫描：vdt ∈ {0.2, 0.42, 0.8}V，核自动取 L 段解出的死区值
     * （×3/π）修正，flux 误差 <5%（修正失效则分别偏 +38%/+80%/+152%，
     * 符号反则 λ<=0 直接拒绝）。 */
    {
        static const float vdts[3] = { 0.20f, 0.42f, 0.80f };
        int k;
        for (k = 0; k < 3; ++k) {
            mc_param_ident_config_t cfg;
            mc_param_ident_t ident;
            plant_t plant;
            const mc_param_ident_result_t *result;

            fast_config(&cfg);
            plant_reset(&plant);
            plant.vdt = vdts[k];
            CHECK_TRUE(run_chain(&ident, &plant, &cfg,
                                 MC_PARAM_IDENT_MODE_FULL, 0.0f, 7u,
                                 INJECT_NONE, MC_PARAM_IDENT_IDLE, 0, NULL)
                       == MC_DONE);
            result = mc_param_ident_get_result(&ident);
            CHECK_NEAR(result->flux_linkage_wb, PLANT_FLUX_TRUE,
                       0.05f * PLANT_FLUX_TRUE);
            /* 交叉核对：Rs 平台残差直测的死区值应跟随 plant 设定
             * （L 段回归值有梯形积分系统偏差，不作此断言）。 */
            CHECK_NEAR(ident.rt.vdt_platform_v, vdts[k], 0.02f);
        }
    }

    /* 零死区：L 段解出 deadtime≈0 不过修，flux 不受损。 */
    {
        mc_param_ident_config_t cfg;
        mc_param_ident_t ident;
        plant_t plant;
        const mc_param_ident_result_t *result;

        fast_config(&cfg);
        plant_reset(&plant);
        plant.vdt = 0.0f;
        CHECK_TRUE(run_chain(&ident, &plant, &cfg, MC_PARAM_IDENT_MODE_FULL,
                             0.0f, 7u, INJECT_NONE, MC_PARAM_IDENT_IDLE,
                             0, NULL) == MC_DONE);
        result = mc_param_ident_get_result(&ident);
        CHECK_NEAR(result->flux_linkage_wb, PLANT_FLUX_TRUE,
                   0.01f * PLANT_FLUX_TRUE);
    }
}

/* =============================================================================
 * 6c. FULL 链 Monte Carlo（毛电压 plant，20 seed）
 * ============================================================================ */

static void test_full_chain_flux_monte_carlo(void)
{
    mc_param_ident_config_t cfg;
    mc_param_ident_t ident;
    plant_t plant;
    enum { FLUX_SEEDS = 20 };
    double flux_est[FLUX_SEEDS];
    int seed;
    int all_done = 1;

    fast_config(&cfg);
    for (seed = 0; seed < FLUX_SEEDS; ++seed) {
        plant_reset(&plant);
        if (run_chain(&ident, &plant, &cfg, MC_PARAM_IDENT_MODE_FULL, 0.025f,
                      (uint32_t)(seed + 1u), INJECT_NONE,
                      MC_PARAM_IDENT_IDLE, 0, NULL) != MC_DONE) {
            all_done = 0;
            continue;
        }
        flux_est[seed] = (double)ident.result.flux_linkage_wb;
    }
    CHECK_TRUE(all_done);
    CHECK_NEAR(mean_of(flux_est, FLUX_SEEDS), PLANT_FLUX_TRUE,
               0.05f * PLANT_FLUX_TRUE);
    CHECK_TRUE(pstdev_of(flux_est, FLUX_SEEDS) / PLANT_FLUX_TRUE < 0.10);
}

/* =============================================================================
 * 7. 故障路径（注入式，表驱动）
 * ============================================================================ */

static void test_fault_paths(void)
{
    static const struct {
        inject_case_t inject;
        mc_status_t expect_status;
        mc_param_ident_fault_t expect_fault;
        int use_nominal_r;
    } cases[] = {
        { INJECT_VBUS_LOW,     MC_FAULT,        MC_PARAM_IDENT_FAULT_VBUS_LOW,           0 },
        { INJECT_FOC_FAULT,    MC_FAULT,        MC_PARAM_IDENT_FAULT_FOC_FAULT,          0 },
        { INJECT_NAN_CURRENT,  MC_NUMERIC_ERROR, MC_PARAM_IDENT_FAULT_FOC_FAULT,         0 },
        { INJECT_SPINNING,     MC_FAULT,        MC_PARAM_IDENT_FAULT_LS_NOT_STANDSTILL,  0 },
        { INJECT_NONE,         MC_FAULT,        MC_PARAM_IDENT_FAULT_RS_DEVIATION,       1 },
    };
    size_t i;

    for (i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        mc_param_ident_config_t cfg;
        mc_param_ident_t ident;
        mc_param_ident_fault_snapshot_t snap;
        plant_t plant;

        fast_config(&cfg);
        if (cases[i].use_nominal_r) {
            cfg.nominal_r = 1.0f; /* 真值 0.069 → 偏差门触发 */
            cfg.tol_r = 0.10f;
        }
        plant_reset(&plant);
        CHECK_TRUE(run_chain(&ident, &plant, &cfg, MC_PARAM_IDENT_MODE_FULL,
                             0.0f, 5u, cases[i].inject,
                             MC_PARAM_IDENT_IDLE, 0, NULL)
                   == cases[i].expect_status);
        CHECK_TRUE(mc_param_ident_get_state(&ident) == MC_PARAM_IDENT_ERROR);
        mc_param_ident_get_fault_snapshot(&ident, &snap);
        CHECK_TRUE(snap.valid == 1u);
        CHECK_TRUE(snap.fault_code == cases[i].expect_fault);
        CHECK_TRUE(ident.result.fault_code == cases[i].expect_fault);
        CHECK_TRUE(g_cb.on_finish_cnt == 1);
        CHECK_TRUE(g_cb.finish_success == 0);
        CHECK_TRUE(g_cb.finish_fault == cases[i].expect_fault);
        CHECK_TRUE(g_cb.apply_cnt == 0); /* 故障收尾不得应用结果 */
        CHECK_TRUE(g_last_cmd.disable_request);
    }
}

/* ============================================================================ */

int main(void)
{
    test_config_gates();
    test_pure_estimators();
    test_flux_estimator_analytic();
    test_rs_staircase();
    test_command_invariants();
    test_full_chain_lq_monte_carlo();
    test_full_chain_flux();
    test_flux_deadtime_correction();
    test_full_chain_flux_monte_carlo();
    test_fault_paths();

    if (g_failures == 0) {
        printf("mc_param_ident：全部测试通过\n");
        return 0;
    }
    printf("mc_param_ident：%d 项断言失败\n", g_failures);
    return 1;
}
