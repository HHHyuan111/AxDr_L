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
#include "modlue.h"
#include "lcd.h"
#include "service_command.h"
#include "service_param_store.h"
#include "service_telemetry.h"
#include "service_usb.h"
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

/* 命令响应发送缝：CDC_Transmit_FS 只登记调用方指针、由 USB 中断异步
 * 分包取数，而 service_usb 的响应缓冲是 process_request 的栈帧——函数
 * 返回即失效。>64B 的响应（电机档案 104B/控制限幅 108B）第 2 包起读
 * 到的是复用后的栈垃圾：CRC 必错，宿主静默丢弃后必然 ACK 超时。
 * （≤64B 单包在返回前已整体装入端点 FIFO，故旧版只有长响应出错；
 * 遥测帧的 pending 缓冲是静态的，不受影响。）拷入静态帧再交 CDC。 */
static uint8_t command_tx_frame[AXDR_COMMAND_MAX_RESPONSE_SIZE];
/* 命令响应与遥测共用同一个 CDC 单发送槛（hcdc->TxState）。遥测走
 * service_telemetry_poll 高频压发，命令响应若恰好撞上 TxState 忙，
 * CDC_Transmit_FS 直接回 USBD_BUSY——旧版本在此处 (void) 丢弃，
 * 宿主侧必然 ACK 超时（GetLinkDiagnostics 轮询频率高，撞车概率
 * 最大，表现为该命令持续超时；GetControlState 偶发）。
 * 本函数运行在主循环（service_usb_poll），非中断上下文，短暂自旋
 * 等 TxState 让出是安全的：单包 CDC 发送通常几十到几百微秒完成，
 * 不阻塞任何中断，不影响定时器中断里的 FOC 控制环。 */
#define USB_CDC_TX_BUSY_RETRY_LIMIT 64u
static void usb_cdc_tx(const uint8_t *data, uint16_t length)
{
    uint32_t retry;

    if (length == 0u || length > sizeof(command_tx_frame))
    {
        return;
    }
    memcpy(command_tx_frame, data, length);
    for (retry = 0u; retry < USB_CDC_TX_BUSY_RETRY_LIMIT; ++retry)
    {
        const int status = usb_cdc_tx_status(command_tx_frame, length);
        if (status != 1)
        {
            /* 0=已登记发送 / 2=其他错误（非忙碌，重试无意义） */
            return;
        }
        /* status==1：USBD_BUSY，上一包（通常是遥测）仍占用 TxState */
    }
    /* 重试耗尽仍忙：放弃本次响应，行为退化为旧版（宿主侧重试兜底），
     * 不在此处死等，避免极端情况下拖慢主循环其他 poll。 */
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

  if (!target_adc_start())
  {
    Error_Handler();
  }

  /* P2 冻结的中断优先级表与微秒时基（见 15 号规范）；须在全部外设初始化后应用。 */
  extern void target_irq_priority_apply(void);
  target_irq_priority_apply();
  extern void target_time_init(void);
  target_time_init();

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
  service_usb_bind_tx(usb_cdc_tx);
  service_telemetry_bind_tx(usb_cdc_tx_status);

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
    service_usb_poll();
    service_telemetry_poll();
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
