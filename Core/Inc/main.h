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
/* ====================================================================== */
/*  公共函数声明                                                           */
/* ====================================================================== */

/***********************************************************
函数名：Error_Handler
参数：  无
返回值：无
描述：  错误处理函数，点亮LED后进入死循环
        在初始化失败时调用（HAL外设初始化返回非HAL_OK时）
***********************************************************/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/***********************************************************
函数名：systick_tx_poll
参数：  无
返回值：无
描述：  SysTick中断驱动的UART DMA发送轮询函数
        每1ms由SysTick_Handler调用，检查tx_ring环形队列：
        - 若有待发事件帧且UART7 DMA空闲，启动DMA异步发送
        - 若UART忙或无待发帧，立即返回（<1μs）
        三级流水线第三级：Step A(触发) → Step B(FFT) → SysTick(TX)
修改记录：
***********************************************************/
void systick_tx_poll(void);
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
/* ⚠️ 警告：此宏定义的测试模式在 6 通道 ADC+FFT 生产路径下不被使用。
 *
 * main.c 中 main() 函数实际走以下路径（由 ENABLE_DAC_SIGNAL_SOURCE 控制）：
 *   - ENABLE_DAC_SIGNAL_SOURCE=0（生产）：6 通道 ADC+FFT 事件检测循环
 *   - ENABLE_DAC_SIGNAL_SOURCE=1（测试）：DAC 闭环测试
 *
 * TEST_MODE_PWM 是 CubeMX 模板遗留/早期实验的 PWM 独立测试模式，
 * 在 2026-06-07 的最新代码中看不见被正式路径使用。
 *
 * 此宏的值不应被依赖。后续重构建议完全移除这组 TEST_MODE_* 宏
 * 或将其统一到 ENABLE_DAC_SIGNAL_SOURCE / 运行时按键选择模式。
 *
 * 另见：docs/CODE_REVIEW_NOTES.md::测试模式与生产路径 */
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


/* USB_RST - CH340 USB RST*/
#define  USB_RST_PORT_CLK_EN() __HAL_RCC_GPIOE_CLK_ENABLE()
#define  USB_RST_PIN           GPIO_PIN_10
#define  USB_RST_PORT          GPIOE


/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
