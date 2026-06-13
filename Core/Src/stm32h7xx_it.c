/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   中断服务程序（ISR）文件
  *
  * 本文件包含所有中断服务程序的实现，包括：
  *   - Cortex-M7 系统异常处理（NMI、HardFault、MemManage 等）
  *   - SysTick 定时器中断（1ms 周期，驱动 HAL 时基 + UART DMA 发送）
  *   - ADC DMA 传输完成/半完成中断（DMA2_Stream0/1, BDMA_Channel0）
  *   - UART1 TX DMA 传输完成中断（DMA1_Stream2）
  *   - UART1 全局中断（错误处理 + DMA TC 路由）
  *
  * 中断优先级映射（数值越小优先级越高）：
  *   ADC DMA (DMA2_Stream0/1, BDMA_Channel0) : 优先级 5,0
  *   UART TX DMA (DMA1_Stream2)              : 优先级 6,0
  *   UART1 全局                               : 优先级 6,0
  *   SysTick                                  : 优先级 15,0（最低）
  *
  * 嵌套关系：
  *   SysTick 可被 ADC DMA 和 UART DMA 抢占（优先级 15 > 5/6）
  *   ADC DMA 可抢占 UART DMA（优先级 5 < 6）
  *   DAC DMA1_Stream1 中断已禁用（dac.c 中手动清除中断使能位）
 * 
 * 【2026-06-13 变更】DMA 冲突修复
 *   - UART1 TX DMA: DMA1_Stream1 → DMA1_Stream2（避免与 DAC 冲突）
 *   - DAC 仍使用 DMA1_Stream1
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"         /* systick_tx_poll() 声明 */
#include "stm32h7xx_it.h"
#include "adc.h"          /* hdma_adc1/2/3 DMA 句柄 */
#include "usart.h"        /* hdebug_uart, hdma_uart1_tx 句柄 */

/* USER CODE BEGIN EV */
/* ADC DMA 句柄 — 在 adc.c 中定义，此处 extern 引用 */
extern DMA_HandleTypeDef hdma_adc1;    /* ADC1 DMA2_Stream0 句柄 */
extern DMA_HandleTypeDef hdma_adc2;    /* ADC2 DMA2_Stream1 句柄 */
extern DMA_HandleTypeDef hdma_adc3;    /* ADC3 BDMA_Channel0 句柄 */

/* 【2026-06-10 架构重构】SysTick 中断驱动的 UART DMA 发送
 * 通过 main.c 导出的 systick_tx_poll() 函数检查 tx_ring 并发送，
 * 避免 extern static 变量的链接问题。
 * 设计原因：tx_ring_head/tail 是 main.c 的 static 变量，
 *           无法在 stm32h7xx_it.c 中直接访问，
 *           因此通过函数调用封装访问，保持模块封装性。 */
/* USER CODE END EV */

/***********************************************************
函数名：NMI_Handler
参数：  无
返回值：无
描述：  不可屏蔽中断处理函数
        NMI 通常由看门狗或外部硬件触发，发生时系统无法恢复
        进入死循环等待硬件复位
修改记录：
***********************************************************/
void NMI_Handler(void)
{
  while (1)
  {
  }
}

/***********************************************************
函数名：HardFault_Handler
参数：  无
返回值：无
描述：  硬件错误中断处理函数
        当 CPU 检测到总线错误、对齐错误等硬件异常时触发
        常见原因：空指针解引用、栈溢出、非法内存访问
        进入死循环等待调试器连接或硬件复位
修改记录：
***********************************************************/
void HardFault_Handler(void)
{
  while (1)
  {
  }
}

/***********************************************************
函数名：MemManage_Handler
参数：  无
返回值：无
描述：  内存管理错误处理函数
        当 MPU（内存保护单元）检测到访问违规时触发
        例如：向只读区域写入、访问非允许的内存区域
        进入死循环等待调试器连接或硬件复位
修改记录：
***********************************************************/
void MemManage_Handler(void)
{
  while (1)
  {
  }
}

/***********************************************************
函数名：BusFault_Handler
参数：  无
返回值：无
描述：  总线错误处理函数
        当 AHB/APB 总线传输发生错误时触发
        例如：访问无效外设地址、DMA 传输目标不可达
        进入死循环等待调试器连接或硬件复位
修改记录：
***********************************************************/
void BusFault_Handler(void)
{
  while (1)
  {
  }
}

/***********************************************************
函数名：UsageFault_Handler
参数：  无
返回值：无
描述：  用法错误处理函数
        当 CPU 检测到未定义指令、除零等用法错误时触发
        进入死循环等待调试器连接或硬件复位
修改记录：
***********************************************************/
void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

/***********************************************************
函数名：SVC_Handler
参数：  无
返回值：无
描述：  系统服务调用（SVCall）处理函数
        由 SVC 指令触发，用于 RTOS 系统调用
        本项目未使用 RTOS，此函数为空实现
修改记录：
***********************************************************/
void SVC_Handler(void)
{
}

/***********************************************************
函数名：DebugMon_Handler
参数：  无
返回值：无
描述：  调试监控中断处理函数
        由调试硬件触发，用于调试事件监控
        本项目未使用调试监控功能，此函数为空实现
修改记录：
***********************************************************/
void DebugMon_Handler(void)
{
}

/***********************************************************
函数名：PendSV_Handler
参数：  无
返回值：无
描述：  Pendable 系统服务中断处理函数
        最低优先级的系统异常，通常由 RTOS 用于上下文切换
        本项目未使用 RTOS，此函数为空实现
修改记录：
***********************************************************/
void PendSV_Handler(void)
{
}

/***********************************************************
函数名：SysTick_Handler
参数：  无
返回值：无
描述：  SysTick 定时器中断处理函数（1ms 周期）
        功能1: 调用 HAL_IncTick() 更新 HAL 时基（供 HAL_Delay/HAL_GetTick 使用）
        功能2: 调用 systick_tx_poll() 检查 tx_ring 环形队列，
              若有待发事件帧且 UART7 DMA 空闲，则启动 DMA 异步发送
        优先级: 15,0（最低优先级，可被所有其他中断抢占）
        执行时间: 无待发帧时 < 1μs，有待发帧时 < 5μs
修改记录：
***********************************************************/
void SysTick_Handler(void)
{
  /* 更新 HAL 时基计数器（1ms 递增） */
  HAL_IncTick();

  /* 【2026-06-10 架构重构】SysTick 1ms 中断驱动 UART DMA 发送
   * 三级流水线的第三级：检查 tx_ring + UART DMA 状态 → 发送
   * 设计要点：
   *   - 主循环只 push 到 tx_ring（零阻塞）
   *   - SysTick 中断负责 pop + 启动 DMA（1ms 定时检查，不依赖主循环速度）
   *   - HAL_UART_Transmit_DMA 在中断中调用是安全的：
   *     主循环 push 只做 memcpy（无 HAL 调用），
   *     TxCpltCallback 只递增计数器（无 HAL 调用），
   *     不会触发 HAL 内部锁的递归死锁 */
  systick_tx_poll();
}

/* USER CODE BEGIN 1 */

/***********************************************************
函数名：DMA2_Stream0_IRQHandler
参数：  无
返回值：无
描述：  ADC1 DMA 传输中断处理函数
        DMA2_Stream0 连接 ADC1，传输优先级 5,0
        中断源：DMA 半传输完成(HT)、全传输完成(TC)、传输错误(TE)
        HAL_DMA_IRQHandler 内部判断中断源并回调：
          HT → HAL_ADC_ConvHalfCpltCallback (adc.c)
          TC → HAL_ADC_ConvCpltCallback (adc.c)
          TE → HAL_ADC_ErrorCallback (adc.c)
修改记录：
***********************************************************/
void DMA2_Stream0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_adc1);
}

/***********************************************************
函数名：DMA2_Stream1_IRQHandler
参数：  无
返回值：无
描述：  ADC2 DMA 传输中断处理函数
        DMA2_Stream1 连接 ADC2，传输优先级 5,0
        功能与 DMA2_Stream0_IRQHandler 相同，对应 ADC2
        3 个 ADC 的 DMA 中断处理逻辑完全一致
修改记录：
***********************************************************/
void DMA2_Stream1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_adc2);
}

/***********************************************************
函数名：BDMA_Channel0_IRQHandler
参数：  无
返回值：无
描述：  ADC3 BDMA 传输中断处理函数
        BDMA_Channel0 连接 ADC3，传输优先级 5,0
        ADC3 使用 BDMA（D3 域 DMA）而非 DMA2（D1 域），
        因为 ADC3 缓冲区位于 SRAM4（0x38000000，D3 域），
        只有 BDMA 才能直接访问 D3 域的 SRAM4
修改记录：
***********************************************************/
void BDMA_Channel0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_adc3);
}

/***********************************************************
函数名：DMA1_Stream2_IRQHandler
参数：  无
返回值：无
描述：  UART1 TX DMA 传输完成中断处理函数
        DMA1_Stream2 连接 UART1_TX，传输优先级 6,0
        当 UART1 DMA 发送完成时触发，HAL 内部路由到：
          TC → HAL_UART_TxCpltCallback (main.c 中实现，仅递增计数器)
        执行流程：
          1. HAL_DMA_IRQHandler 清除 DMA 中断标志
          2. 调用 UART_DMATransmitCplt
          3. 复位 huart->gState 为 HAL_UART_STATE_READY
          4. 触发 HAL_UART_TxCpltCallback（main.c 中仅 diag_uart_q_send_cnt++）
        【2026-06-13 变更】从 DMA1_Stream1 改为 DMA1_Stream2（避免与 DAC 冲突）
修改记录：
***********************************************************/
void DMA1_Stream2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_uart1_tx);
}

/***********************************************************
函数名：USART1_IRQHandler
参数：  无
返回值：无
描述：  UART1 全局中断处理函数
        处理 UART1 的所有中断事件，优先级 6,0
        主要中断源：
          - DMA TC 完成由 HAL_DMA_IRQHandler 路由到此
          - UART 错误中断（ORE/FRE/NE）由 HAL_UART_IRQHandler 直接处理
        中断处理流程：
          1. HAL_UART_IRQHandler 检查 UART 中断标志
          2. 若是 DMA TC：调用 UART_DMATransmitCplt，复位 gState 为 READY
          3. 若是 UART 错误：调用 HAL_UART_ErrorCallback，可能中止 DMA 传输
        注意：DMA TC 完成时，中断先进入 DMA1_Stream1_IRQHandler，
              再通过 HAL 机制触发 USART1_IRQHandler，
              最终到达 HAL_UART_TxCpltCallback
        【2026-06-12 变更】UART7 → UART1
修改记录：
***********************************************************/
void USART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&hdebug_uart);
}

/* USER CODE BEGIN 2 */

/* 【2026-06-13 变更】UART7 改为调试日志打印，不再使用 DMA
 *   - UART1 (PA9/PA10): 事件帧上送 (DMA1_Stream2)
 *   - UART7 (PE7/PE8): 调试日志打印 (普通阻塞模式)
 */

/***********************************************************
函数名：DMA1_Stream0_IRQHandler
参数：  无
返回值：无
描述：  UART7 TX DMA 传输完成中断处理函数
        【2026-06-12】UART7 已不再使用 DMA，此中断处理保留但不激活
        如需恢复 UART7 DMA 支持，需重新配置 DMA1_Stream0 和中断优先级
修改记录：
***********************************************************/
void DMA1_Stream0_IRQHandler(void)
{
  /* UART7 DMA 已禁用，如需启用请在 usart.c 中重新配置 */
}

/***********************************************************
函数名：UART7_IRQHandler
参数：  无
返回值：无
描述：  UART7 全局中断处理函数
        【2026-06-12】UART7 改为调试日志打印，使用普通阻塞模式（printf）
        此中断处理程序保留用于错误处理
修改记录：
***********************************************************/
void UART7_IRQHandler(void)
{
  /* UART7 使用轮询模式，不启用中断 */
  /* 如需中断模式，参考 USART1_IRQHandler 实现 */
}

/* USER CODE END 2 */