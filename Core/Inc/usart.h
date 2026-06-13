/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* ============================================================================
 * 串口定义
 * 
 * 命名规则：用途_UART
 * - DEBUG_UART:  UART1 (PA9 TX, PA10 RX) - 事件帧上送 + 调试日志
 * - RS485_UART:  UART4 (PA0 TX, PA1 RX, PE9 DE/RE) - RS485通信
 * - LOG_UART:    USART3 (PB10 TX, PB11 RX) - 额外日志输出
 * 
 * 功能说明：
 * - DEBUG_UART: 用于事件帧上送，波特率460800
 * - RS485_UART: 用于RS485总线通信，波特率9600，带DE/RE控制
 * - LOG_UART:   用于额外日志输出，波特率115200
 * 
 * 【2026-06-13 变更】UART7 → UART1，DMA 冲突修复
 *   - 原: UART7 (PE7/PE8) - 因 PE8 与 LCD 数据总线冲突，改为 PA9/PA10
 *   - 新: UART1 (PA9/PA10) - PA9=TX, PA10=RX
 *   - DMA: DMA1_Stream2（避免与 DAC 的 DMA1_Stream1 冲突）
 * ============================================================================*/

/* DEBUG_UART - 调试串口（UART1）*/
extern UART_HandleTypeDef hdebug_uart;
extern DMA_HandleTypeDef  hdma_uart1_tx;  /* 2026-06-13: TX DMA 句柄（DMA1_Stream2，零阻塞发送） */
#define DEBUG_UART_INSTANCE      USART1
#define DEBUG_UART_BAUDRATE      2000000    /* 2M 波特率 */

/* RS485_UART - RS485通信串口（UART4）*/
extern UART_HandleTypeDef hrs485_uart;
#define RS485_UART_INSTANCE      UART4
#define RS485_UART_BAUDRATE      9600

/* LOG_UART - 额外日志串口（USART3）*/
extern UART_HandleTypeDef hlog_uart;
#define LOG_UART_INSTANCE        USART3
#define LOG_UART_BAUDRATE        115200

/* RS485 DE/RE控制引脚 */
#define RS485_DE_RE_PIN          GPIO_PIN_9
#define RS485_DE_RE_PORT         GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/***********************************************************
函数名：MX_DEBUG_UART_Init
参数：  无
返回值：无
描述：  初始化调试串口UART1（事件帧上送+调试日志）
        波特率460800, 8N1, TX=DMA1_Stream2(零阻塞)
        【2026-06-13 变更】UART7 → UART1 (PA9/PA10), DMA1_Stream2
***********************************************************/
void MX_DEBUG_UART_Init(void);

/***********************************************************
函数名：MX_RS485_UART_Init
参数：  无
返回值：无
描述：  初始化RS485通信串口UART4，波特率9600, 8N1
        DE/RE控制引脚PE10
***********************************************************/
void MX_RS485_UART_Init(void);

/***********************************************************
函数名：MX_LOG_UART_Init
参数：  无
返回值：无
描述：  初始化日志串口USART3，波特率115200, 8N1
***********************************************************/
void MX_LOG_UART_Init(void);

#define RS485_SetTxMode() HAL_GPIO_WritePin(RS485_DE_RE_PORT, RS485_DE_RE_PIN, GPIO_PIN_SET)
#define RS485_SetRxMode() HAL_GPIO_WritePin(RS485_DE_RE_PORT, RS485_DE_RE_PIN, GPIO_PIN_RESET)

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */
