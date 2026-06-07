/**
  ******************************************************************************
  * File Name          : DEBUG.h
  * Description        : This file provides code for the configuration
  *                      of the DEBUG instances.
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __debug_H
#define __debug_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdarg.h>
#include <stdio.h>

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_DEBUG_Init(void);

/* USER CODE BEGIN Prototypes */
int SEGGER_RTT_Write(int ch, const char *s, int len);
int SEGGER_RTT_printf(const char *fmt, ...);
void debug_init(void);
void debug_printf(const char *fmt, ...);
void debug_poll(void);                  /* v6.4: 已改为 no-op，避免和 tx_ring_poll 抢 UART */
uint32_t debug_get_drop_bytes(void);    /* 已丢弃字节数（缓冲满时） */

/* 非阻塞二进制 push：把任意字节流（如事件帧）放入环形缓冲，立即返回 */
void debug_push_raw(const uint8_t *buf, uint16_t len);

/* === 调试日志总开关 ===
 * 设为 0 时所有 debug_printf 调用被预处理器替换为空语句
 * 用于让串口只输出二进制事件帧，方便 PC 工具解析（不会被ASCII日志干扰）
 * 设为 1 时正常输出调试日志（性能诊断、过滤统计、FFT详情等）
 *
 * 注意：debug.c 本身需要保留函数定义，在 debug.c 顶部
 * 定义 DEBUG_PRINTF_INTERNAL 避开此宏替换 */
#define ENABLE_DEBUG_LOG  0

#if (ENABLE_DEBUG_LOG == 0) && !defined(DEBUG_PRINTF_INTERNAL)
  /* 关闭：把 debug_printf 替换为 do{}while(0)，编译器会优化掉
   * 注意：参数仍会被语法检查，但不会求值（避免副作用问题） */
  #define debug_printf(...)  do { } while(0)
#endif
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ debug_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
