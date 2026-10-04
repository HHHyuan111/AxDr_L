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

/** @brief 上电装载：读 Flash 校验后应用零位（在 foc_init 之后、fast_loop_enable 之前调）。 */
void service_param_store_boot(void);

/** @brief 主循环节拍：监测对齐完成边沿并保存（每周期至多尝试一次，失败不重试）。 */
void service_param_store_poll(void);

/** @brief 本次上电零位是否来自 Flash（1=已装载或已保存，0=待对齐）。 */
uint8_t service_param_store_loaded(void);

#endif /* SERVICE_PARAM_STORE_H */
