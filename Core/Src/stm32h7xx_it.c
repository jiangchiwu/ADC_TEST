/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_it.h"
#include "adc.h"      /* hdma_adc1/2/3 */
#include "usart.h"    /* hdebug_uart, hdma_uart7_tx */

/* USER CODE BEGIN EV */
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern DMA_HandleTypeDef hdma_adc3;
/* USER CODE END EV */

void NMI_Handler(void)
{
  while (1)
  {
  }
}

void HardFault_Handler(void)
{
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

/* USER CODE BEGIN 1 */

void DMA2_Stream0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_adc1);
}

void DMA2_Stream1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_adc2);
}

void BDMA_Channel0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_adc3);
}

/* 2026-06-07 v7：UART7 TX DMA 完成中断 */
void DMA1_Stream0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_uart7_tx);
}

/* UART7 IRQ：DMA TC 完成由 HAL_DMA_IRQHandler 路由到此，
 * 触发 HAL_UART_TxCpltCallback，并自动复位 gState 到 READY */
void UART7_IRQHandler(void)
{
  HAL_UART_IRQHandler(&hdebug_uart);
}

/* USER CODE END 1 */