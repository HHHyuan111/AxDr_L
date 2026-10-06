/**
 * @file service_param_store.h
 * @brief S5 参数持久化：编码器对齐零位 e_off 存内部 Flash，上电免对齐直进 RUN。
 *
 * 策略：单页单副本 + magic/版本/档案绑定/CRC 四重校验（任何不过即作废，
 * 走重新对齐，代价 1.5s）；对齐完成边沿自动保存（每上电周期至多一次）。
 * 换电机/机械重装 → 改 motor_config.h 的 MOTOR_PROFILE_REVISION → 自动作废重对齐。
 */

#ifndef SERVICE_PARAM_STORE_H
#define SERVICE_PARAM_STORE_H

#include <stdint.h>

/** 辨识档案数据（10 float，40B）：结果 + 质量摘要。
 *  逐字段 >0 为有效（LQ 复测的 flux/Ke/Kt 为 0，落盘时与旧档案合并）。 */
typedef struct
{
    float rs_ohm;          /* 相电阻(Ω) */
    float ld_h;            /* d 轴电感(H) */
    float lq_h;            /* q 轴电感(H) */
    float flux_wb;         /* 磁链(Wb)，FULL 模式有效 */
    float ke_v_per_rad;    /* Ke：线线RMS/(机械rad/s) */
    float kt_nm_a;         /* Kt：Nm/(dq峰值A) */
    float ld_r2;           /* Ld 回归决定系数 */
    float lq_r2;           /* Lq 回归决定系数 */
    float flux_std_wb;     /* 磁链样本 std(Wb) */
    float deadtime_v;      /* Ld 段回归死区压降(V)，A5 回流对照用 */
} service_ident_data_t;

/** @brief 上电装载：读 Flash 校验后应用零位（在 foc_init 之后、fast_loop_enable 之前调）。 */
void service_param_store_boot(void);

/** @brief 主循环节拍：监测对齐完成边沿并保存（每周期至多尝试一次，失败不重试）。 */
void service_param_store_poll(void);

/** @brief 本次上电零位是否来自 Flash（1=已装载或已保存，0=待对齐）。 */
uint8_t service_param_store_loaded(void);

/** @brief 辨识成功暂存（ISR 安全：仅拷贝+置标志，Flash 擦写在主循环 poll）。
 *  由 drive_diag 的 on_finish 回调在成功收尾拍调用。 */
void service_param_store_ident_stage(const service_ident_data_t *data);

/** @brief 主循环节拍：有暂存则合并落盘页 2（逐字段 >0 者胜，失败不重试）。 */
void service_param_store_ident_poll(void);

/** @brief 读辨识档案（boot 用）。1=记录有效，0=无档/作废（出厂/换电机/坏 CRC）。 */
uint8_t service_param_store_ident_read(service_ident_data_t *out);

#endif /* SERVICE_PARAM_STORE_H */
