/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 系统启动与后台任务入口。电机快环由 ADC 中断调用。
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fast_loop.h"
#include "foc.h"
#include "current_offset.h"
#include "drive_diag.h"
#include "rtt_log.h"
#include "target_adc.h"
#include "target_time.h"
#include "target_irq_profile.h"
#include "moldue.h"
#include "lcd.h"
#include "service_command.h"
#include "service_param_store.h"
#include "service_telemetry.h"
#include "service_usb.h"
#include "service_usb_tx.h"
#include "service_observation.h"
#include "target_reset.h"
#include "usbd_cdc_if.h"
#include "axdr_command_contract.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t observation_usb_generation;
static uint32_t observation_stats_tick;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void app_motor_init(void);
static void app_services_init(void);
static void app_display_init(void);
static void app_background_poll(void);
static void app_observation_update(void);

static int usb_cdc_tx_status(const uint8_t *data, uint16_t length);
static uint32_t usb_tx_lock(void);
static void usb_tx_unlock(uint32_t enabled);
static int usb_command_tx(const uint8_t *data, uint16_t length);

/* 当前板级接口尚未提供独立声明头。 */
extern void target_irq_priority_apply(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 阅读顺序：main → app_motor_init / app_background_poll。
 * 20 kHz 电机控制入口在 User/app/fast_loop.c，不在 while 循环中执行。
 * 本文件内部实现放在 USER CODE 4 区，保留 CubeMX 的生成区域。
 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* 复位原因最早锁存：之后任何代码清掉 RCC->CSR 都会丢原因。 */
  target_reset_latch();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USB 外设初始化前准备发送队列，供后续 USB 回调使用。 */
  service_usb_tx_init(usb_cdc_tx_status, usb_tx_lock, usb_tx_unlock);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USB_Device_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_USART3_UART_Init();
  MX_FDCAN1_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  /* 1. 采样启动、控制器初始化、参数恢复，然后放行电机快环。 */
  app_motor_init();

  /* 2. 连接通信服务，并显示初始电机状态。 */
  app_services_init();
  app_display_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 3. 后台处理安全会话、通信和参数保存；控制快环由中断执行。 */
    app_background_poll();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* 电机初始化 -------------------------------------------------------------- */

/* ADC 先提供零偏校准所需的采样；控制器和参数就绪后才允许执行快环。
 * 每一步完成都写 RTT 日志：上电卡住时看日志即知卡在哪一步。 */
static void app_motor_init(void)
{
    rtt_log_init();
    HAL_Delay(1000);
    HAL_TIM_Base_Start(&htim3);

    /* 在 ADC 中断启动前初始化计时器，避免产生虚假的采样间隔。 */
    target_time_init();
    if (!target_adc_start())
    {
        Error_Handler();
    }
    target_irq_priority_apply();
    rtt_log_str("adc started");

    /* 恢复参数时快环仍被禁止；ABZ 上电后仍需重新对齐。 */
    foc_init(&g_foc);
    rtt_log_str("foc init done");
    cur_offset_init(&g_foc);
    rtt_log_str("current offset done");
    service_param_store_boot();
    rtt_log_str("param restore done");
    {
        /* 上电自增 boot_id（追加记录，页满才擦）：上位机以此区分重启。 */
        uint32_t boot_id = 0u;

        (void)service_param_store_boot_id_read(&boot_id);
        boot_id += 1u;
        service_param_store_boot_id_commit(boot_id);
        rtt_log_u32("boot_id", boot_id);
        rtt_log_u32("reset_reason", target_reset_reason());
    }
    drive_diag_ident_load();
    fast_loop_enable();
    rtt_log_str("fast loop enabled");
}

/* 通信与显示初始化 -------------------------------------------------------- */

static void app_services_init(void)
{
    service_telemetry_init();
    service_observation_init();

    /* 命令应答和波形数据共用发送队列，实际发送由后台任务完成。 */
    service_usb_bind_tx(usb_command_tx);
    service_telemetry_bind_tx(service_usb_tx_enqueue_data);
    service_observation_bind_tx(service_usb_tx_enqueue_data);
}

static void app_display_init(void)
{
    LCD_Init();
    LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
    HAL_Delay(1000);
    display_foc();
}

/* 后台任务 ---------------------------------------------------------------- */

/* 每轮先检查安全会话，再处理通信，最后处理待保存的参数。 */
static void app_background_poll(void)
{
    /* 安全会话与观测链路状态。 */
    service_safety_poll();
    app_observation_update();

    /* 接收命令、准备数据、发送应答与波形。 */
    service_usb_poll();
    service_observation_poll();
    service_telemetry_poll();
    service_usb_tx_poll();

    /* 参数保存放在后台，避免在控制中断中执行 Flash 擦写。 */
    service_param_store_poll();
    service_param_store_ident_poll();
}

/* 连接变化时重置观测；每 50 ms 汇总一次通信和中断统计。 */
static void app_observation_update(void)
{
    const uint32_t generation = service_usb_tx_link_generation();
    const uint32_t tick = HAL_GetTick();
    if (generation != observation_usb_generation)
    {
        observation_usb_generation = generation;
        service_observation_reset_link();
    }
    if ((uint32_t)(tick - observation_stats_tick) >= 50u)
    {
        service_usb_tx_stats_t tx;
        target_irq_profile_t irq;
        observation_stats_tick = tick;
        service_usb_tx_get_stats(&tx);
        target_irq_profile_read(&irq);
        service_observation_set_tx_stats(tx.command_completed + tx.data_completed,
            tx.submit_errors + tx.unexpected_completions +
                service_usb_stats()->tx_queue_full + service_usb_stats()->tx_errors,
            tx.aborted_frames, tx.queue_peak);
        service_observation_set_timing_stats(irq.last_cycles, irq.max_cycles,
            irq.overruns, irq.core_clock_hz);
        service_observation_set_irq_stats(irq.count, irq.gap_count,
            irq.max_interval_cycles, irq.adc_overruns);
    }
}

/* USB 发送适配 ------------------------------------------------------------ */

/* 将 USB 驱动的发送结果转换为服务接口约定：0 成功，1 忙，其他值为错误。 */
static int usb_cdc_tx_status(const uint8_t *data, uint16_t length)
{
    const uint8_t status = CDC_Transmit_FS((uint8_t *)data, length);
    return (status == USBD_OK) ? 0 : ((status == USBD_BUSY) ? 1 : 2);
}

/* 队列和 CDC 回调共享少量状态，仅屏蔽 USB IRQ，不屏蔽 20 kHz ADC 控制环。 */
static uint32_t usb_tx_lock(void)
{
    const uint32_t enabled = NVIC_GetEnableIRQ(USB_LP_IRQn);
    NVIC_DisableIRQ(USB_LP_IRQn);
    __DSB();
    __ISB();
    return enabled;
}

static void usb_tx_unlock(uint32_t enabled)
{
    __DMB();
    if (enabled != 0u)
    {
        NVIC_EnableIRQ(USB_LP_IRQn);
    }
}

static int usb_command_tx(const uint8_t *data, uint16_t length)
{
    const uint32_t enabled = usb_tx_lock();
    /* 命令处理可被 USB 复位打断；检查与入队同在一个临界区。 */
    const int result = service_usb_response_is_current() != 0u
        ? service_usb_tx_enqueue_command(data, length) : 2;
    usb_tx_unlock(enabled);
    return result;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
