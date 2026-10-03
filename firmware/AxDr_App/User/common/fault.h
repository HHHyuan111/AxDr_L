/**
 * @file fault.h
 * @brief 运行时故障位图。
 * @note  检测方置位，Drive 层据此进入 FAULT 状态并关断输出；
 *        必须显式清除后才能再次启动。新增保护项 = 追加一位（15 号保护插槽）。
 */

#ifndef FAULT_H
#define FAULT_H

typedef enum
{
    ERR_NONE = 0,
    ERR_OC = 1u << 0,    /* 过流 */
    ERR_OV = 1u << 1,    /* 母线过压 */
    ERR_UV = 1u << 2,    /* 母线欠压 */
    ERR_OT_MOS = 1u << 3,/* MOS 超温 */
    ERR_ENC = 1u << 4,   /* 位置反馈无效 */
    ERR_COMM_LOST = 1u << 5, /* 命令超时 */
    ERR_CONFIG = 1u << 6,/* 配置损坏 / 校验失败 */
} fault_t;

#endif /* FAULT_H */
