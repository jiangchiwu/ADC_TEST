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
 * - DEBUG_UART:  UART7 (PE7 RX, PE8 TX) - 调试日志打印
 * - RS485_UART:  UART4 (PA0 TX, PA1 RX, PE9 DE/RE) - RS485通信
 * - LOG_UART:    USART3 (PB10 TX, PB11 RX) - 额外日志输出
 * 
 * 功能说明：
 * - DEBUG_UART: 用于调试信息输出，波特率115200
 * - RS485_UART: 用于RS485总线通信，波特率9600，带DE/RE控制
 * - LOG_UART:   用于额外日志输出，波特率115200
 * ============================================================================*/

/* DEBUG_UART - 调试串口（UART7）*/
extern UART_HandleTypeDef hdebug_uart;
extern DMA_HandleTypeDef  hdma_uart7_tx;  /* 2026-06-07: TX DMA 句柄（零阻塞发送） */
#define DEBUG_UART_INSTANCE      UART7
#define DEBUG_UART_BAUDRATE      115200

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

void MX_DEBUG_UART_Init(void);
void MX_RS485_UART_Init(void);
void MX_LOG_UART_Init(void);

#define RS485_SetTxMode() HAL_GPIO_WritePin(RS485_DE_RE_PORT, RS485_DE_RE_PIN, GPIO_PIN_SET)
#define RS485_SetRxMode() HAL_GPIO_WritePin(RS485_DE_RE_PORT, RS485_DE_RE_PIN, GPIO_PIN_RESET)

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */
