/**
 * @file mc_param_ident.h
 * @brief 电机参数辨识（分阶段：Rs 双档差分 / Ld·Lq 二元回归 / 磁链闭环恒速）
 *
 * 来源：suanfa 参考工程 G11-50 `app\User\motor_ident\motor_param_ident.c/h`
 * （1383+498 行，2026-10-05 迁入）。P8 资产回流 A1 三段拆分的第一段
 * （asset-ident-core）：纯核 + 零全局 + host 单测；drive/service 接入归
 * 后续 adapt/svc PR。
 *
 * 与 G11 原版的四处语义差异（评审记录，详见 P8 计划 §7）：
 *  1. FINISHING 0.2s 收尾折叠：完成/失败当拍即返回 MC_DONE/MC_FAULT 并发
 *     disable_request，安全关断尾巴移交诊断调度层（比原版更保守）。
 *  2. 主动中止独立终态 ABORTED：原版 stop() 走 FINISHING 且 fault==NONE
 *     会被误判成功并误 apply_results；本版中止不触发结果应用。
 *  3. FOC 使能丢失检测并入 fault_code 去抖（原版 sample.enabled 字段
 *     删除；调度层掉线时 step 不再被调用，由 owner 状态机接管）。
 *  4. 上电自动加载/应用移交 adapt/service 层；本模块仅提供
 *     mc_param_ident_apply()/save()/load() 手动入口。
 *
 * 接口换形说明（G11 platform 回调 → mc 家族 step 推拉）：
 *  - 输入：mc_sample_t 每拍由调度层组装（id_a/iq_a 即未经 dq 低通的
 *    Park 原始电流，drive_diag 每拍重 Park，与原版 id_raw 语义坍缩）。
 *  - 输出：mc_command_t 每拍全量重填（幂等）：PREPARE/RS/L/LQ 段
 *    openloop_enable=true + 锁存角（转子 d 轴锁定）；FLUX 段
 *    openloop_enable=false（编码器角闭环拖转）；终态 disable_request。
 *  - apply_results/on_start/on_finish/save/load 保留为 io 回调（可全 NULL）。
 *
 * 使用前提与边界：
 *  - PREPARE 锁角使用 sample->theta_elec_rad，要求编码器已完成校准
 *    （先跑对齐/极对数辨识任务）。
 *  - L 回归的 1/3 拍命令/测量延迟补偿按 PWM 双缓冲管线结构设定，
 *    支持控制频率约 10~40kHz（host 数值基准锁 20kHz/50us）。
 *  - Ke 约定：线线 RMS / 机械 rad/s（Ke = flux * pp * sqrt(3/2)）；
 *    Kt 约定：dq 峰值电流（Kt = 1.5 * pp * flux）。
 *  - 磁链死区修正：驱动回报的 vq 为电流环 PI 输出（毛电压，自动顶掉
 *    逆变器死区压降），λ̂ 若不扣除该项会带 +vdt/ωe 正偏置（小磁链/
 *    低速电机可达数十百分点）。FLUX 段自动取 Rs 平台残差直测的死区
 *    压降（rt.vdt_platform_v = vd−Rs·id 两档平均）乘 3/π（锁轴→旋转
 *    几何换算，见 mc_param_ident.c MC_IDENT_FLUX_VDT_DQ_SCALE 注释）
 *    注入估计器；一阶近似残差 3~4%（A5 死区补偿回流 foc 链后消除）。
 *    L 段回归的 quality.ld.deadtime_drop_v 因梯形积分误差有 -5~-10%
 *    系统偏差，仅作诊断输出，不用于修正。
 */

#ifndef MC_PARAM_IDENT_H
#define MC_PARAM_IDENT_H

#include <stdbool.h>
#include <stdint.h>

#include "mc_common.h"

/* =============================================================================
 * 状态与故障定义
 * ============================================================================*/

typedef enum {
    MC_PARAM_IDENT_IDLE = 0,        /* 空闲 */
    MC_PARAM_IDENT_PREPARE,         /* 准备：固定电角度闭环 Id 锁转子 */
    MC_PARAM_IDENT_RS_SETTLE,       /* Rs 电流平台稳定（双档各来一次） */
    MC_PARAM_IDENT_RS_SAMPLE,       /* Rs 平台采样累积 */
    MC_PARAM_IDENT_L_SAMPLE,        /* Ld 双档×双极脉冲 + 二元回归 */
    MC_PARAM_IDENT_LQ_SAMPLE,       /* Lq 双档×双极脉冲 + 二元回归 */
    MC_PARAM_IDENT_FLUX_SPINUP,     /* 磁链：编码器闭环斜坡拖转 */
    MC_PARAM_IDENT_FLUX_SAMPLE,     /* 磁链：恒速窗口采样 */
    MC_PARAM_IDENT_DONE,            /* 成功完成 */
    MC_PARAM_IDENT_ERROR,           /* 故障终止 */
    MC_PARAM_IDENT_ABORTED,         /* 主动中止（非故障，不应用结果） */
} mc_param_ident_state_t;

typedef enum {
    MC_PARAM_IDENT_FAULT_NONE = 0,
    MC_PARAM_IDENT_FAULT_RS_NO_SAMPLE = 1,        /* Rs 样本不足/电流未建立 */
    MC_PARAM_IDENT_FAULT_RS_OUT_OF_RANGE = 2,     /* Rs 超量程或平台电压异常 */
    MC_PARAM_IDENT_FAULT_LS_NO_SAMPLE = 3,        /* L 有效窗口不足 */
    MC_PARAM_IDENT_FAULT_LS_OUT_OF_RANGE = 4,     /* L 超量程 */
    MC_PARAM_IDENT_FAULT_FLUX_NO_SAMPLE = 5,      /* 磁链样本不足/波动过大 */
    MC_PARAM_IDENT_FAULT_FLUX_SPEED_NOT_REACHED = 6, /* 拖转未进转速窗口 */
    MC_PARAM_IDENT_FAULT_LS_FIT_QUALITY = 7,      /* L 回归质量门控失败 */
    MC_PARAM_IDENT_FAULT_RS_UNSTABLE = 8,         /* Rs 平台电流波动超限 */
    MC_PARAM_IDENT_FAULT_LS_NOT_STANDSTILL = 9,   /* L 注入期转子未静止 */
    MC_PARAM_IDENT_FAULT_FOC_FAULT = 10,          /* 驱动故障（去抖后） */
    MC_PARAM_IDENT_FAULT_VBUS_LOW = 11,           /* 母线电压过低（去抖后） */
    MC_PARAM_IDENT_FAULT_OVERCURRENT = 12,        /* 采样电流超限 */
    MC_PARAM_IDENT_FAULT_RS_DEVIATION = 20,       /* Rs 偏离标称容差 */
    MC_PARAM_IDENT_FAULT_LS_DEVIATION = 21,       /* L 偏离标称容差 */
    MC_PARAM_IDENT_FAULT_FLUX_DEVIATION = 22,     /* 磁链偏离标称容差 */
    MC_PARAM_IDENT_FAULT_KE_DEVIATION = 23,       /* Ke 偏离标称容差 */
    MC_PARAM_IDENT_FAULT_KT_DEVIATION = 24,       /* Kt 偏离标称容差 */
} mc_param_ident_fault_t;

/* MC_PARAM_IDENT_FAULT_LS_FIT_QUALITY 的细分原因，可按位组合。 */
typedef enum {
    MC_PARAM_IDENT_L_QUALITY_FAIL_NONE        = 0u,
    MC_PARAM_IDENT_L_QUALITY_FAIL_SOLVE       = (1u << 0),
    MC_PARAM_IDENT_L_QUALITY_FAIL_R2          = (1u << 1),
    MC_PARAM_IDENT_L_QUALITY_FAIL_LEVEL_DELTA = (1u << 2),
    MC_PARAM_IDENT_L_QUALITY_FAIL_VSWITCH     = (1u << 3),
    MC_PARAM_IDENT_L_QUALITY_FAIL_SAMPLE      = (1u << 4),
} mc_param_ident_l_quality_fail_t;

/* 辨识模式（末段选择）：FULL=自由轴全参数（磁链），LQ=锁轴测 Lq。 */
typedef enum {
    MC_PARAM_IDENT_MODE_FULL = 0,    /* Rs/Ld/磁链/Ke/Kt；需解锁自由旋转 */
    MC_PARAM_IDENT_MODE_LQ   = 1     /* Rs/Ld/Lq；电流环整定用，无需锁轴 */
} mc_param_ident_mode_t;

/* =============================================================================
 * 数据结构
 * ============================================================================*/

/* 辨识结果（成功时全部有效；LQ 模式 flux/Ke/Kt 为零）。 */
typedef struct {
    float phase_resistance;         /* 相电阻(Ω) */
    float phase_inductance;         /* d 轴电感(H) */
    float phase_inductance_q;       /* q 轴电感(H)，LQ 模式有效 */
    float flux_linkage_wb;          /* 磁链(Wb)，FULL 模式有效 */
    float back_emf_v_per_rad;       /* Ke：线线RMS/(机械rad/s) = flux·pp·√1.5 */
    float torque_kt_nm_a;           /* Kt：Nm/(dq峰值A) = 1.5·pp·flux */
    mc_param_ident_fault_t fault_code; /* 故障码（NONE=成功） */
} mc_param_ident_result_t;

/* 磁链单样本估计器输入（algo_ops 可替换）。 */
typedef struct {
    float vq;                       /* q 轴实际电压(V)，有符号（电流环 PI 输出=毛电压） */
    float iq;                       /* q 轴电流(A)，有符号 */
    float id;                       /* d 轴电流(A)，有符号 */
    float omega_e;                  /* 实测电角速度(rad/s)，有符号 */
    float phase_resistance;         /* 相电阻(Ω) */
    float phase_inductance;         /* d 轴电感(H) */
    float deadtime_v;               /* q 轴等效死区压降(V)；0=不修正 */
} mc_param_ident_flux_eval_input_t;

typedef struct {
    /* 返回 true 表示得到有效磁链样本；NULL 项回落内置默认实现。 */
    bool (*eval_flux_sample)(const mc_param_ident_flux_eval_input_t *in,
                             float *out_flux_wb);
} mc_param_ident_algo_ops_t;

/* 故障快照（触发时刻全现场，触发后冻结）。 */
typedef struct {
    uint8_t valid;                  /* 1=快照有效 */
    mc_param_ident_fault_t fault_code;
    uint16_t progress;              /* 触发时进度(permille) */
    mc_param_ident_state_t state;   /* 触发时状态 */
    uint32_t tick;                  /* 触发时状态内 tick */
    float align_v_cmd;              /* 触发时 d 轴电压指令(V) */
    float flux_vq_cmd;              /* 触发时 q 轴实际电压(V) */
    float sample_ia;                /* 触发时 ia(A) */
    float sample_ib;                /* 触发时 ib(A) */
    float sample_ic;                /* 触发时 ic(A) */
    float sample_id;                /* 触发时 id(A) */
    float sample_iq;                /* 触发时 iq(A) */
    float sample_vd;                /* 触发时 vd(V) */
    float sample_vq;                /* 触发时 vq(V) */
    float sample_omega_mech_rad_s;  /* 触发时机械角速度(rad/s) */
    float sample_vbus;              /* 触发时母线电压(V) */
    float filt_id;                  /* 低通 id(A) */
    float filt_iq;                  /* 低通 iq(A) */
    float filt_vd;                  /* 低通 vd(V) */
    float filt_vq;                  /* 低通 vq(V) */
    uint32_t rs_cnt;                /* Rs 累计样本数 */
    uint32_t l_cnt;                 /* Ls 累计样本数 */
    uint32_t flux_cnt;              /* 磁链累计样本数 */
    float l_est_lo;                 /* 当前轴低激励档拟合值(H) */
    float l_est_hi;                 /* 当前轴高激励档拟合值(H) */
    float l_fit_r2;                 /* 当前轴联合拟合决定系数 */
    float l_deadtime_v;             /* 拟合等效开关/死区压降(V) */
    uint32_t l_reject_cnt;          /* L 回归拒绝窗口数 */
    uint32_t l_pos_cnt;             /* 正脉冲有效窗口数 */
    uint32_t l_neg_cnt;             /* 负脉冲有效窗口数 */
    float l_level_delta;            /* 低/高档 L 相对差异 */
    uint8_t l_quality_fail_mask;    /* mc_param_ident_l_quality_fail_t 位掩码 */
} mc_param_ident_fault_snapshot_t;

/* 辨识配置（单位见注释；0 值策略见 default_config 与 start 校验）。 */
typedef struct {
    float test_current_a;           /* 对齐电流及磁链速度外环基准(A) */
    float align_voltage_v;          /* PREPARE/Rs 阶段允许的最大 |Vd|(V) */
    float align_time_s;             /* 固定电角度闭环 Id 锁转子时间(s) */
    float rs_settle_s;              /* Rs 平台稳定时间(s) */
    float rs_sample_s;              /* Rs 平台采样时间(s) */
    float l_sample_s;               /* 电感采样时间(s)，Ld/Lq 共用 */
    float l_toggle_s;               /* 电感方波注入半周期(s) */
    float l_settle_s;               /* 每注入档前零电压退流时间(s) */
    float rs_current_lo_a;          /* Rs 双档差分低电平电流(A) */
    float rs_current_hi_a;          /* Rs 双档差分高电平电流(A) */
    float l_inject_lo_v;            /* L 双档差分低电平注入电压(V) */
    float l_inject_hi_v;            /* L 双档差分高电平注入电压(V) */
    float flux_voltage_v;           /* 磁链闭环拖转允许的最大 |Vq|(V) */
    float flux_target_erpm;         /* 磁链测试转速(erpm) */
    float flux_spinup_s;            /* 磁链加速斜坡时间(s) */
    float flux_sample_s;            /* 磁链采样时间(s) */
    float control_period_s;         /* 控制周期(s)，如 20kHz 填 5e-5f */
    float pole_pairs;               /* 极对数 */
    float current_limit_a;          /* 安全电流限幅(A)；0=禁止启动（闸门） */
    float voltage_limit_v;          /* 安全电压限幅(V)；0=禁止启动（闸门） */
    float min_id_a;                 /* 最小 d 轴电流阈值(A) */
    float min_di_a;                 /* 最小电流变化阈值(A) */
    float r_min;                    /* 电阻下限(Ω) */
    float r_max;                    /* 电阻上限(Ω) */
    float l_min;                    /* 电感下限(H) */
    float l_max;                    /* 电感上限(H) */
    float l_fit_min_r2;             /* L 回归最小 R² (0~1) */
    float l_level_tolerance;        /* 低/高档 L 一致性容差(比例) */
    float nominal_r;                /* 标称电阻(Ω)；0=不检查偏差 */
    float nominal_l;                /* 标称电感(H)；0=不检查 */
    float nominal_flux;             /* 标称磁链(Wb)；0=不检查 */
    float nominal_ke;               /* 标称 Ke(V/(rad/s))；0=不检查 */
    float nominal_kt;               /* 标称 Kt(Nm/A)；0=不检查 */
    float tol_r;                    /* 电阻容差(比例，0=不检查) */
    float tol_l;                    /* 电感容差(0=不检查) */
    float tol_flux;                 /* 磁链容差(0=不检查) */
    float tol_ke;                   /* Ke 容差(0=不检查) */
    float tol_kt;                   /* Kt 容差(0=不检查) */
    float vbus_min_v;               /* 最小母线电压(V) */
    float filter_alpha;             /* 一阶低通系数(0.01~0.5) */
} mc_param_ident_config_t;

/* L 回归质量：estimate_lo/hi 用于暴露死区/饱和/采样时序问题；
 * combined 才是最终采纳值。 */
typedef struct {
    float estimate_lo_h;
    float estimate_hi_h;
    float combined_h;
    float fit_r2;
    float deadtime_drop_v;          /* 等效开关/死区压降(V) */
    uint32_t accepted;
    uint32_t rejected;
    uint32_t positive;
    uint32_t negative;
    float level_delta;
    uint8_t fail_mask;              /* mc_param_ident_l_quality_fail_t 位掩码 */
} mc_param_ident_l_quality_t;

/* 最近一次辨识质量诊断（成功失败均保留）。 */
typedef struct {
    float rs_current_lo_a;          /* Rs 低档实测均值(A) */
    float rs_current_hi_a;          /* Rs 高档实测均值(A) */
    float rs_voltage_lo_v;          /* Rs 低档电压均值(V) */
    float rs_voltage_hi_v;          /* Rs 高档电压均值(V) */
    float rs_current_std_max_a;     /* 两档电流 std 较大值(A) */
    mc_param_ident_l_quality_t ld;
    mc_param_ident_l_quality_t lq;
    float flux_std_wb;              /* 磁链样本 std(Wb) */
    float flux_speed_mean_erpm;     /* 采样窗口平均电转速(erpm) */
} mc_param_ident_quality_t;

/* u = R*i + L*di/dt + Vswitch*avg(sign(i)) 的在线最小二乘充分统计量。 */
typedef struct {
    float xx, xz, zz;
    float xy, zy, yy;
    uint32_t accepted;
    uint32_t rejected;
    uint32_t positive;
    uint32_t negative;
} mc_param_ident_l_reg_t;

/* l_reg_add 的门控参数（从 cfg 摘出，供纯估计器独立使用）。 */
typedef struct {
    float min_di_a;                 /* 窗口最小电流变化(A) */
    float nominal_l;                /* 标称电感(H)，0=不设标称窗 */
} mc_param_ident_l_gate_t;

/* 内部累加器（Rs/磁链长和式用 double：万级样本累加精度优先）。 */
typedef struct {
    double flux_acc;
    double rs_i_sum, rs_v_sum;
    double rs_i_sq_sum;
    double flux_sq_sum, flux_speed_sum;
    uint32_t rs_cnt, l_cnt, flux_cnt;
    mc_param_ident_l_reg_t l_reg;
} mc_param_ident_acc_t;

/* 低通滤波器状态。 */
typedef struct {
    float id, vd, vq, iq;
} mc_param_ident_filter_t;

/* 运行时变量。 */
typedef struct {
    uint32_t tick;
    uint32_t toggle_tick;
    float test_current_amp;         /* test_current 的安全钳制值(A) */
    int8_t test_sign;
    float dt;                       /* = cfg.control_period_s */
    float align_v_cmd;              /* 当前 d 轴实际电压跟踪(V) */
    float flux_vq_cmd;              /* 当前 q 轴实际电压跟踪(V) */
    float openloop_angle;           /* PREPARE 锁存的电角度(rad) */
    uint16_t progress;              /* 0~1000 */
    uint16_t foc_fault_streak;
    uint16_t vbus_low_streak;
    uint16_t rs_ready_streak;
    uint8_t rs_retry_count;
    uint8_t flux_retry_count;
    uint8_t rs_level;               /* Rs 双档：0=低电平级 1=高电平级 */
    float rs_v_lvl0, rs_i_lvl0;     /* Rs 低档 (vd,id) 均值缓存 */
    float rs_std_lvl0;
    float vdt_platform_v;           /* Rs 平台残差直测死区(V)：vd−Rs·id 两档平均 */
    float rs_current_cmd;           /* Rs 电流指令（含软启动斜坡） */
    uint8_t l_level;                /* L 双档：0=低 1=高 */
    uint8_t l_phase;                /* 0=零电压退流 1=双极脉冲采样 */
    float l_edge_current;           /* 测量窗口起点电流(A) */
    float l_prev_current;
    float l_prev_voltage;           /* 上一拍已写入 PWM 的轴电压(V) */
    float l_current_sum;            /* 窗口梯形电流积分和 */
    float l_sign_sum;               /* 窗口 avg(sign(i)) 积分和 */
    float l_voltage_sum;            /* 窗口实际电压命令和 */
    mc_param_ident_l_reg_t l_reg_lo; /* 低档充分统计量缓存 */
    float l_est_lo;
    float flux_iq_int;              /* 磁链速度外环积分项 */
    float flux_speed_cmd;           /* 磁链段机械角速度指令(rad/s) */
} mc_param_ident_runtime_t;

/* 模块回调集（全部可 NULL；attach 时整体拷贝进 ctx）。 */
typedef struct {
    void (*on_start)(void *user_ctx);   /* PREPARE 首拍、任何命令输出前 */
    void (*on_finish)(bool success, mc_param_ident_fault_t fault_code,
                      void *user_ctx);  /* 终态（DONE/ERROR/ABORTED）当拍 */
    void (*apply_results)(const mc_param_ident_result_t *result,
                          void *user_ctx); /* 成功后自动调用一次 */
    bool (*load)(mc_param_ident_result_t *out_result, void *user_ctx);
    bool (*save)(const mc_param_ident_result_t *result, void *user_ctx);
    void *user_ctx;
} mc_param_ident_io_t;

/* 模块上下文（调用方持有；约 1KB）。必须零初始化（静态或 ={0}），
 * io 回调域在未 attach 时保持零。 */
typedef struct {
    mc_param_ident_state_t state;
    mc_param_ident_mode_t mode;
    mc_param_ident_result_t result;
    mc_param_ident_quality_t quality;
    mc_param_ident_fault_snapshot_t fault_snapshot;
    mc_param_ident_config_t cfg;
    mc_param_ident_acc_t acc;
    mc_param_ident_filter_t filt;
    mc_param_ident_runtime_t rt;
    mc_sample_t last_sample;        /* 最近采样副本（故障快照用） */
    mc_param_ident_algo_ops_t algo_ops;
    mc_param_ident_io_t io;
} mc_param_ident_t;

/* =============================================================================
 * 公共接口
 * ============================================================================*/

/* 填默认配置：算法时序/门限为 G11 原值；激励幅值为沉沙电机档案
 * （5 对极，平台电流限 8A：Rs 平台 2/4A）；current_limit_a 与
 * voltage_limit_v 保持 0（闸门：未确认试验条件禁止启动，start 拒绝）；
 * nominal/tol 全 0（待沉沙首次辨识后回填，0=不检查偏差）。 */
void mc_param_ident_default_config(mc_param_ident_config_t *cfg);

/* 挂接回调集（可 NULL；整体拷贝）。 */
void mc_param_ident_attach(mc_param_ident_t *ctx, const mc_param_ident_io_t *io);

/* 启动辨识。cfg 为 NULL 时用默认配置；current_limit_a/voltage_limit_v/
 * control_period_s/pole_pairs 非法（<=0 或非有限）返回 MC_INVALID_ARGUMENT；
 * 已在运行返回 MC_REJECTED。 */
mc_status_t mc_param_ident_start(mc_param_ident_t *ctx,
                                 const mc_param_ident_config_t *cfg,
                                 mc_param_ident_mode_t mode);

/* 状态机单步：sample 由调度层每拍组装；command 每拍全量重填。
 * 返回 MC_BUSY=继续 / MC_DONE=成功 / MC_FAULT=故障终止 /
 * MC_ABORTED=主动中止 / MC_REJECTED=非运行态 / MC_NUMERIC_ERROR=采样非有限。 */
mc_status_t mc_param_ident_step(mc_param_ident_t *ctx,
                                const mc_sample_t *sample,
                                mc_command_t *command);

/* 主动中止（非故障）：不触发 apply_results；on_finish 收 (false, fault)。 */
mc_status_t mc_param_ident_abort(mc_param_ident_t *ctx, mc_command_t *command);

/* 手动应用结果（io.apply_results 委托；成功收尾时已自动调用过一次）。 */
mc_status_t mc_param_ident_apply(const mc_param_ident_t *ctx);

/* 持久化手动入口（io.load/save 委托）。 */
bool mc_param_ident_save(const mc_param_ident_t *ctx);
bool mc_param_ident_load(mc_param_ident_t *ctx);

/* 查询接口。 */
mc_param_ident_state_t mc_param_ident_get_state(const mc_param_ident_t *ctx);
uint16_t mc_param_ident_get_progress_permille(const mc_param_ident_t *ctx);
const mc_param_ident_result_t *mc_param_ident_get_result(const mc_param_ident_t *ctx);
void mc_param_ident_get_quality(const mc_param_ident_t *ctx,
                                mc_param_ident_quality_t *out_quality);
void mc_param_ident_get_fault_snapshot(const mc_param_ident_t *ctx,
                                       mc_param_ident_fault_snapshot_t *out_snapshot);

/* 算法回调替换（NULL 项回落内置默认；传 NULL 整体恢复默认）。 */
void mc_param_ident_set_algo_ops(mc_param_ident_t *ctx,
                                 const mc_param_ident_algo_ops_t *ops);

/* =============================================================================
 * 纯估计器（公开供 host 单测与导入工具复用，家族 mc_rs_ident_solve 先例）
 * ============================================================================*/

/* Rs 双平台差分：Rs=(vd_hi-vd_lo)/(id_hi-id_lo)，di<=min_di 返回 MC_OUT_OF_RANGE。 */
mc_status_t mc_param_ident_solve_rs(float vd_lo_v, float id_lo_a,
                                    float vd_hi_v, float id_hi_a,
                                    float min_di_a, float *out_rs_ohm);

/* 单窗口加入 RL+开关压降回归：u_avg - Rs*i_avg = L*di/dt + Vswitch*sign_avg。
 * 门控不过（di 过小/方向异常/超标称窗/非有限）只计 rejected。 */
void mc_param_ident_l_reg_add(mc_param_ident_l_reg_t *reg,
                              const mc_param_ident_l_gate_t *gate,
                              float u_cmd_v, float i0_a, float i1_a,
                              float i_avg_a, float sign_avg, float dt_meas_s);

/* 联合解 (L, Vswitch, R²)；正/负窗口各不足 24 个或近共线返回 false。 */
bool mc_param_ident_l_reg_solve(const mc_param_ident_l_reg_t *reg,
                                float *out_l_h, float *out_vdt_v, float *out_r2);

/* 用联合解出的共同开关压降，在单激励档内仅拟合 L（档间一致性检查用）。 */
bool mc_param_ident_l_reg_solve_level(const mc_param_ident_l_reg_t *reg,
                                      float switch_drop_v, float *out_l_h);

/* 相邻采样间 avg(sign(i)) 的线性插值（电流过零的处理）。 */
float mc_param_ident_sign_average(float i0_a, float i1_a);

/* 内置磁链估计器：λ=(vq-Rs·iq-deadtime_v·sign(iq))/ωe - Ld·id；
 * |ωe|<=5rad/s 或 λ<=0 拒绝；deadtime_v=0 时不做死区修正。 */
bool mc_param_ident_eval_flux_sample(const mc_param_ident_flux_eval_input_t *in,
                                     float *out_flux_wb);

#endif /* MC_PARAM_IDENT_H */
