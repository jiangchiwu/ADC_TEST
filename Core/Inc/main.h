/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* ============================================================================
 * 按键定义
 * KEY: PC13 - 外部按键输入（低电平有效）
 * ============================================================================*/
#define KEY_Pin GPIO_PIN_13
#define KEY_GPIO_Port GPIOC

/* ============================================================================
 * 测试模式选择（通过宏定义选择单独测试某个模块）
 * 
 * 测试模式说明：
 * - TEST_MODE_NONE:     正常运行模式（所有功能启用）
 * - TEST_MODE_LED:      LED测试模式（测试LED指示灯）
 * - TEST_MODE_PWM:      PWM测试模式（1S低电平+1S 50KHz PWM）
 * - TEST_MODE_DAC:      DAC测试模式（输出正弦波）
 * - TEST_MODE_ADC:      ADC测试模式（采集并打印数据）
 * - TEST_MODE_UART:     UART测试模式（串口回环测试）
 * ============================================================================*/
#define TEST_MODE_NONE     0
#define TEST_MODE_LED      1
#define TEST_MODE_PWM      2
#define TEST_MODE_DAC      3
#define TEST_MODE_ADC      4
#define TEST_MODE_UART     5

/* 当前测试模式（修改此宏选择测试模式）*/
#define CURRENT_TEST_MODE  TEST_MODE_PWM

/* ============================================================================
 * LED指示灯定义（共阴极接法，低电平点亮）
 * 
 * 命名规则：功能_LED
 * - RUN_LED:    PB12 - 运行状态指示灯（系统正常运行时常亮）
 * - HEART_LED:  PB13 - 心跳指示灯（1Hz闪烁，表示系统正在运行）
 * - READY_LED:  PB14 - 就绪指示灯（系统初始化完成后常亮）
 * 
 * 硬件说明：
 * - 采用共阴极接法，LED阳极通过限流电阻接VCC
 * - GPIO输出低电平时LED点亮，高电平时LED熄灭
 * - 端口时钟：所有LED均位于GPIOB，时钟使能只需调用一次
 * ============================================================================*/

/* RUN_LED - 运行状态指示灯 */
#define RUN_LED_PORT_CLK_EN()   __HAL_RCC_GPIOB_CLK_ENABLE()
#define RUN_LED_PIN             GPIO_PIN_12
#define RUN_LED_PORT            GPIOB

/* HEART_LED - 心跳指示灯（1Hz闪烁）*/
#define HEART_LED_PORT_CLK_EN() __HAL_RCC_GPIOB_CLK_ENABLE()
#define HEART_LED_PIN           GPIO_PIN_13
#define HEART_LED_PORT          GPIOB

/* READY_LED - 就绪指示灯 */
#define READY_LED_PORT_CLK_EN() __HAL_RCC_GPIOB_CLK_ENABLE()
#define READY_LED_PIN           GPIO_PIN_14
#define READY_LED_PORT          GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
