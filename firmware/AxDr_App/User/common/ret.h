/**
 * @file ret.h
 * @brief 可能失败的公开函数的返回码。
 * @note  范式（12 号命名决策）：默认返回 void；只有确实可能失败的函数
 *        （初始化 / IO / 配置读写 / 参数设置）才返回 ret_e，调用方必须检查。
 */

#ifndef RET_H
#define RET_H

typedef enum
{
    RET_OK = 0,      /* 成功 */
    RET_PARAM,       /* 参数非法 */
    RET_BUSY,        /* 当前状态不允许该操作 */
    RET_UNSUPPORTED, /* 操作不受支持 */
    RET_IO,          /* 底层 IO 失败 */
} ret_e;

#endif /* RET_H */
