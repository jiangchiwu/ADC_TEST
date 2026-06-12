/**
  ******************************************************************************
  * @file    test.c
  * @brief   Test functions
  ******************************************************************************
  */

#include "test.h"
#include "tim.h"
#include "debug.h"

/* 粗略的微秒延迟函数 */
static void Rough_Delay_us(uint32_t us)
{
  uint32_t count = us * 30;
  while(count--) {
    __NOP();
  }
}

/**
  * @brief  PWM测试主函数
  * @retval None
  */
void test_pwm_main(void)
{
  uint32_t cycle_count = 0;
  
  debug_printf("=== PWM Waveform Output Test ===\r\n");
  debug_printf("=================================\r\n");
  debug_printf("CH1: PC6 (TIM8_CH1)\r\n");
  debug_printf("CH2: PC7 (TIM8_CH2)\r\n");
  debug_printf("PWM Frequency: 50 KHz\r\n");
  debug_printf("Period: 20 us\r\n");
  debug_printf("Duty Cycle: 50%% for both channels\r\n");
  debug_printf("Phase Difference: CH2 lags CH1 by 20 us (1 period)\r\n");
  debug_printf("Pattern: 1s LOW + 1s 50KHz PWM\r\n");
  debug_printf("=================================\r\n\r\n");
  
  while(1) {
    cycle_count++;
    
    /* 低电平阶段 - 停止两个通道的PWM输出，确保输出低电平 */
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
    
    debug_printf("[Cycle %lu] PWM State: LOW (0%% Duty)\r\n", cycle_count);
    debug_printf("  CH1 (PC6): PWM Stopped, Output = LOW\r\n");
    debug_printf("  CH2 (PC7): PWM Stopped, Output = LOW\r\n");
    debug_printf("  Duration: 1000 ms\r\n\r\n");
    
    HAL_Delay(1000);
    
    /* 50KHz PWM阶段 - 通过延时使能通道实现20us相位差
     * 1. 先复位计数器，确保从0开始
     * 2. 启动CH1的PWM -> PC6第一个上升沿立即出现
     * 3. 延迟20us
     * 4. 启动CH2的PWM -> PC7第一个上升沿延迟20us
     * 这样从1s低电平开始的第一个上升沿正好相差20us
     */
    /* 复位计数器，确保从0开始 */
    __HAL_TIM_SET_COUNTER(&htim8, 0);
    
    /* 设置CCR为1200 (50%占空比) */
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 1200);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 1200);
    
    /* CH1 (PC6) 立即启动PWM - 第一个上升沿立即出现 */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    
    /* 延迟20us = 1个PWM周期 */
    Rough_Delay_us(20);
    
    /* CH2 (PC7) 延迟启动PWM - 第一个上升沿正好比CH1晚20us */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    
    /* 串口输出PWM波形信息 */
    debug_printf("[Cycle %lu] PWM State: 50 KHz PWM\r\n", cycle_count);
    debug_printf("  CH1 (PC6 - TIM8_CH1): CCR = 1200, Duty = 50%%\r\n");
    debug_printf("  CH2 (PC7 - TIM8_CH2): CCR = 1200, Duty = 50%%\r\n");
    debug_printf("  First Rising Edge: CH2 lags CH1 by ~20 us (1 full period)\r\n");
    debug_printf("  Period: 20 us, High Time: 10 us\r\n");
    debug_printf("  Duration: 1000 ms\r\n\r\n");
    
    HAL_Delay(1000);
    
    /* 每5个周期打印一次汇总信息 */
    if(cycle_count % 5 == 0) {
      debug_printf("=================================\r\n");
      debug_printf("PWM Waveform Summary:\r\n");
      debug_printf("  Total Cycles: %lu\r\n", cycle_count);
      debug_printf("  CH1 Pin: PC6 (TIM8_CH1)\r\n");
      debug_printf("  CH2 Pin: PC7 (TIM8_CH2)\r\n");
      debug_printf("  Phase Difference: ~20 us (1 PWM period)\r\n");
      debug_printf("  Both Channels: 50%% Duty Cycle\r\n");
      debug_printf("  Pattern: 1s LOW + 1s 50KHz PWM\r\n");
      debug_printf("=================================\r\n\r\n");
    }
  }
}




void test_main(void)
{
  // MX_DEBUG_UART_Init();
  // debug_init();
  debug_printf("test_main\r\n");

}
 /* hal_init 硬件初始化 */
void hal_init(void)
{
  
}