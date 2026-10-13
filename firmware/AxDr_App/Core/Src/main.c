/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "drive_diag.h"
#include "target_adc.h"
#include "target_irq_profile.h"
#include "modlue.h"
#include "lcd.h"
#include "service_command.h"
#include "service_param_store.h"
#include "service_telemetry.h"
#include "service_usb.h"
#include "service_usb_tx.h"
#include "service_observation.h"
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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* CDC 发送缝适配：service 的 0=OK/1=BUSY/其他=错误 ←→ USBD 状态码。
 * 组装层专属：唯一允许同时看见 service 缝与 CDC 的地方。 */
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
    /* 命令处理可被 USB 复位打断；检查与入队之间不能跨越连接切换。 */
    const int result = service_usb_response_is_current() != 0u
        ? service_usb_tx_enqueue_command(data, length) : 2;
    usb_tx_unlock(enabled);
    return result;
}
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

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
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
  HAL_Delay(1000);
  HAL_TIM_Base_Start(&htim3);
  // HAL_TIM_Base_Start_IT(&htim1);

  /* 调试器可能已开启 DWT。必须先归零时基再开 ADC IRQ，避免把归零
   * 误记为一次超长采样间隔；此后不再改写周期计数器。 */
  extern void target_time_init(void);
  target_time_init();

  if (!target_adc_start())
  {
    Error_Handler();
  }

  /* P2 冻结的中断优先级表与微秒时基（见 15 号规范）；须在全部外设初始化后应用。 */
  extern void target_irq_priority_apply(void);
  target_irq_priority_apply();

  foc_init(&g_foc);
  /* S5 零位装载：须在 fast_loop_enable 之前——快环一开 START 即可进来。
   * 绝对值编码器装载即免对齐；ABZ 原点每上电重置，START 时重对齐。 */
  service_param_store_boot();
  /* 辨识档案装载（P8 A1）：有效记录即应用电机参数并分轴重整定电流环，
   * 同样须在 fast_loop_enable 之前。 */
  drive_diag_ident_load();
  fast_loop_enable();

  /* S4/S3 service 装配：发送缝绑定 + 遥测复位。CDC 收路径在
   * usbd_cdc_if.c 的 CDC_Receive_FS（USB 中断 → service_usb_rx 环）。 */
  service_telemetry_init();
  service_observation_init();
  service_usb_bind_tx(usb_command_tx);
  service_telemetry_bind_tx(service_usb_tx_enqueue_data);
  service_observation_bind_tx(service_usb_tx_enqueue_data);

  LCD_Init();
  LCD_Fill(0,0,LCD_W,LCD_H,BLACK);
  HAL_Delay(1000);
  display_foc();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* service 慢路径五 poll（自由节拍）：安全链执法（ARM 后生效）、
     * 命令帧解析分发、遥测取帧发送、对齐零位保存（边沿触发）、
     * 辨识档案落盘（on_finish 暂存，逐字段合并）。
     * 均设计为空转廉价；save 时页擦除约 22ms（Bank2 擦写不 stall Bank1 取指）。 */
    service_safety_poll();
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
    service_usb_poll();
    service_observation_poll();
    service_telemetry_poll();
    service_usb_tx_poll();
    service_param_store_poll();
    service_param_store_ident_poll();
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
