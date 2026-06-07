/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the UART instances.
  * 
  * 串口配置：
  * - DEBUG_UART (UART7): 调试日志打印，波特率 2000000 (2M)
  * - RS485_UART (UART4): RS485通信，波特率9600
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
/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include <stdio.h>

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

UART_HandleTypeDef hdebug_uart;
UART_HandleTypeDef hrs485_uart;
UART_HandleTypeDef hlog_uart;

/* 2026-06-07: UART7 TX DMA 句柄（DMA1 Stream0）—— 实现零阻塞发送 */
DMA_HandleTypeDef  hdma_uart7_tx;

/* UART7 init function */
/**
 * @brief   初始化调试串口 UART7（事件帧上送 + 调试日志）
 * @param   None
 * @retval  None
 *
 * 【硬件连接】
 *   STM32 UART7 TX (PE8)  --> CH340 USB-Serial 芯片 RXD
 *   STM32 UART7 RX (PE7)  --> CH340 USB-Serial 芯片 TXD
 *   CH340 USB 端 --> PC (Windows 识别为 COMxx, 通常 COM13)
 *
 * 【关键配置参数】
 *   - 波特率: 460800 baud
 *     * 单字节传输时间 = 10 bit / 460800 ≈ 21.7 μs
 *     * 19 字节事件帧 ≈ 412 μs
 *     * 必须与 PC 端 verify_event_system.py 的 BAUDRATE 一致
 *   - 数据位: 8 bit
 *   - 停止位: 1 bit
 *   - 校验位: 无 (无 Parity，使用应用层累加和校验)
 *   - 流控: 无 (CH340 不支持硬件流控)
 *   - 过采样: 16x (UART_OVERSAMPLING_16，标准模式)
 *   - 模式: TX + RX (双向，但本项目实际只用 TX)
 *
 * 【波特率选型说明】
 *   实测对比 CH340 + Windows + pyserial 在不同波特率下的到达率：
 *     115200 baud → 到达率 22%（CH340 latency 16ms 包碎片严重）
 *     460800 baud → 到达率 100%（120 evt/s 自检模式）
 *     921600 baud → CH340 不稳定，丢帧严重
 *     2000000 baud → CH340 严重丢帧
 *   ★ 460800 是 CH340 的最佳工作点（USB 包大小 + latency 平衡）
 *
 * 【与 PC 端协同的关键约束】
 *   ① UART 必须独占给事件帧使用，禁用所有 ASCII debug 输出
 *      （否则 ASCII 字节插入二进制帧流，PC 端解析失败）
 *   ② 板上发送速率必须 ≤ 120 evt/s（CH340 硬件极限）
 *   ③ PC 端用 in_waiting + read(avail) 非阻塞模式读取
 */
void MX_DEBUG_UART_Init(void)
{
  hdebug_uart.Instance = UART7;
  /* === 波特率：460800 ===
   *
   * 经过完整对比测试，460800 baud 是 CH340 + Windows + pyserial 最稳定工作点：
   *   - 单帧 19B 传输时间 ~412μs，CH340 packet aggregation 友好
   *   - 6 帧紧接发送 ~2.47ms，刚好对应 1 个 USB Bulk IN 包
   *   - 实测 120 evt/s 自检模式 PC 端 100% 接收
   *
   * 注意：必须与 PC 端 verify_event_system.py 的 DEFAULT_BAUDRATE 完全一致 */
  hdebug_uart.Init.BaudRate = 460800;
  hdebug_uart.Init.WordLength = UART_WORDLENGTH_8B;    /* 8 数据位（事件帧字节流） */
  hdebug_uart.Init.StopBits = UART_STOPBITS_1;          /* 1 停止位（标准串口） */
  hdebug_uart.Init.Parity = UART_PARITY_NONE;           /* 无校验（应用层用累加和） */
  hdebug_uart.Init.Mode = UART_MODE_TX_RX;              /* 全双工（仅用 TX） */
  hdebug_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;     /* 无硬件流控（CH340 不支持） */
  hdebug_uart.Init.OverSampling = UART_OVERSAMPLING_16; /* 16x 过采样（标准模式） */
  if (HAL_UART_Init(&hdebug_uart) != HAL_OK)
  {
    Error_Handler();
  }
}

/* UART4 init function */
void MX_RS485_UART_Init(void)
{
  hrs485_uart.Instance = UART4;
  hrs485_uart.Init.BaudRate = 9600;
  hrs485_uart.Init.WordLength = UART_WORDLENGTH_8B;
  hrs485_uart.Init.StopBits = UART_STOPBITS_1;
  hrs485_uart.Init.Parity = UART_PARITY_NONE;
  hrs485_uart.Init.Mode = UART_MODE_TX_RX;
  hrs485_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hrs485_uart.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&hrs485_uart) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USART3 init function */
void MX_LOG_UART_Init(void)
{
  hlog_uart.Instance = USART3;
  hlog_uart.Init.BaudRate = 115200;
  hlog_uart.Init.WordLength = UART_WORDLENGTH_8B;
  hlog_uart.Init.StopBits = UART_STOPBITS_1;
  hlog_uart.Init.Parity = UART_PARITY_NONE;
  hlog_uart.Init.Mode = UART_MODE_TX_RX;
  hlog_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlog_uart.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&hlog_uart) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==UART7)
  {
    __HAL_RCC_UART7_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_UART7;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* === 2026-06-07 v7 - DMA 零阻塞最终版（彻底验证）===
     * 关键设计：
     *   - HAL_UART_Transmit_DMA 启动时自动 SET CR3.DMAT；TC 中断自动 CLEAR
     *   - 必须用 LINKDMA 把 hdma_uart7_tx 绑定到 huart 的 hdmatx 字段
     *   - DMA1 Stream0 中断 + UART7 中断都要启用（缺一不可！）
     *   - 主循环 tx_ring_poll 启动下一帧；中断回调只清状态 */
    __HAL_RCC_DMA1_CLK_ENABLE();

    hdma_uart7_tx.Instance                 = DMA1_Stream0;
    hdma_uart7_tx.Init.Request             = DMA_REQUEST_UART7_TX;
    hdma_uart7_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_uart7_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_uart7_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_uart7_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart7_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_uart7_tx.Init.Mode                = DMA_NORMAL;
    hdma_uart7_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;
    hdma_uart7_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if(HAL_DMA_Init(&hdma_uart7_tx) != HAL_OK) { Error_Handler(); }
    __HAL_LINKDMA(uartHandle, hdmatx, hdma_uart7_tx);

    /* DMA1 Stream0 中断 - 优先级 6 (高于 SysTick 15, 低于 ADC DMA 0) */
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

    /* UART7 中断 - DMA TC 完成时 HAL_DMA_IRQHandler → HAL_UART_TxCpltCallback 触发 */
    HAL_NVIC_SetPriority(UART7_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(UART7_IRQn);
  }
  else if(uartHandle->Instance==UART4)
  {
    __HAL_RCC_UART4_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
  }
  else if(uartHandle->Instance==USART3)
  {
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{
  if(uartHandle->Instance==UART7)
  {
    __HAL_RCC_UART7_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_7|GPIO_PIN_8);
  }
  else if(uartHandle->Instance==UART4)
  {
    __HAL_RCC_UART4_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1);
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_9);
  }
  else if(uartHandle->Instance==USART3)
  {
    __HAL_RCC_USART3_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);
  }
}

/* USER CODE BEGIN 1 */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&hdebug_uart, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

int __io_getchar(void)
{
  uint8_t ch;
  HAL_UART_Receive(&hdebug_uart, &ch, 1, HAL_MAX_DELAY);
  return ch;
}

/* USER CODE END 1 */