/**
 * @file board_config.h
 * @brief 当前驱动板的采样、功率级、保护和执行频率配置。
 */

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/* PWM 与 FOC 的硬件执行频率。 */
#define DRIVE_FOC_FREQ_HZ                     (20000.0f)

/* 驱动板采样和功率器件参数。 */
#define DRIVE_ADC_REFERENCE_V                 (3.3f)
#define DRIVE_ADC_FULL_SCALE_COUNT            (4096.0f)
#define DRIVE_CURRENT_SHUNT_OHM               (0.001f)
#define DRIVE_CURRENT_AMP_GAIN                (20.0f)
#define DRIVE_VBUS_DIVIDER_HIGH_OHM           (20000.0f)
#define DRIVE_VBUS_DIVIDER_LOW_OHM            (1000.0f)
#define DRIVE_HARDWARE_DEADTIME_US            (0.5f)

/* 驱动保护阈值。 */
#define DRIVE_UNDER_VOLTAGE_V                 (15.0f)
#define DRIVE_OVER_VOLTAGE_V                  (60.0f)
#define DRIVE_OVER_CURRENT_A                  (80.0f)
#define DRIVE_MOS_OVER_TEMPERATURE_C          (100.0f)
#define DRIVE_COIL_OVER_TEMPERATURE_C         (100.0f)

/* 主动诊断任务的独立硬上限；实物确认前保持 0。 */
#define DRIVE_DIAG_CURRENT_LIMIT_A            (0.0f)
#define DRIVE_DIAG_VOLTAGE_LIMIT_V            (0.0f)

#endif /* BOARD_CONFIG_H */
