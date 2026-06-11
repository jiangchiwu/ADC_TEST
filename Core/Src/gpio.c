/* USER CODE BEGIN Header */
/**
  ******************************************************************************
 CH3(PC2): 116 116 116 116 116 116 116 116
  CH4(PC3): 117 117 117 117 117 117 117 117
  CH5(PA3): 117 117 117 117 117 117 117 117
  CH6(PA4): 116 116 116 116 116 116 116 116
>>> EVENT CH1(PC0) [F#80312]: T=1150s 861ms 246us | freq=39990.4 Hz | base=117 noise=0 thr=8 | trans@436 peak@439(+-0.00) rkOff=0.00 (target=50KHz 1.0V)
    BEFORE: 117 117 117 116 115 113 110 107 104 102 |<PEAK>|  101 102 106 112 121 131 142 152 161 166 167 164 156 144 128 108  88  69  53  40  34  33  40  52  71  94 121 148 169 185 197 205 208 202 188 168 141 110  78  47  19   0   0   0   0   0  18  42  73 107
>>> EVENT CH2(PC1) [F#80312]: T=1150s 861ms 127us | freq=38288.7 Hz | base=117 noise=2 thr=8 | trans@332 peak@336(+-0.50) rkOff=0.17 (target=50KHz 1.0V)
    BEFORE: 111 112 114 116 119 123 126 129 132 133 |<PEAK>|  133 132 130 126 121 115 109 104 100  96  95  95  97 101 106 112 119 126 132 136 139 140 139 136 132 126 120 114 108 103 100  98  98 100 103 108 113 118 122 126 129 130 130 129 126 123 120 116 113 110  * @file    gpio.c
  * @brief   GPIO配置文件 - 管理所有GPIO引脚的初始化和控制
  * 
  * 主要功能：
  * - LED指示灯初始化（PB12/PB13/PB14）
  * - 按键输入配置（PC13）
  * - 系统时钟相关引脚配置
  * 
  * 硬件连接：
  * - RUN_LED  (PB12): 运行状态指示
  * - HEART_LED(PB13): 心跳指示（1Hz闪烁）
  * - READY_LED(PB14): 就绪指示（系统初始化完成）
  * - KEY      (PC13): 用户按键
  * 
  * 驱动方式：
  * - LED采用共阴极接法，低电平点亮
  * - KEY采用上拉输入，低电平触发
  * 
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @brief  GPIO初始化函数
  * 
  * 配置说明：
  * - PC14/PC15: 32kHz外部晶振输入输出
  * - PH0/PH1:   25MHz外部晶振输入输出
  * - PA13/PA14: SWD调试接口
  * - PB12/PB13/PB14: LED输出（推挽输出，无上下拉）
  * 
  * @param  None
  * @retval None
  */
/***********************************************************
函数名：MX_GPIO_Init
参数：  无
返回值：无
描述：  GPIO初始化函数
        配置以下GPIO引脚：
        1. LED指示灯（PB12/PB13/PB14）- 推挽输出，初始高电平（熄灭）
           采用共阴极接法，低电平点亮LED
        2. CH340 USB复位引脚（PE10）- 推挽输出，初始高电平（不复位状态）
        时钟使能：GPIOB（LED）和 GPIOE（USB RST）
修改记录：
***********************************************************/
void MX_GPIO_Init(void)
{
  /* GPIO初始化结构体，初始化为0避免未初始化成员导致问题 */
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* ========================================================================
   * 步骤1: 使能GPIO端口时钟
   * GPIOB: RUN_LED/HEART_LED/READY_LED所在端口
   * GPIOE: USB_RST所在端口（与GPIOB分开使能，但PE10的使能在步骤4）
   * ========================================================================*/
	RUN_LED_PORT_CLK_EN();
	HEART_LED_PORT_CLK_EN(); 
	READY_LED_PORT_CLK_EN(); 

  /* ========================================================================
   * 步骤2: 设置GPIO初始输出电平
   * 由于LED采用共阴极接法（低电平点亮），初始状态设置为高电平（熄灭）
   * GPIO_PIN_SET = 高电平 = LED熄灭
   * GPIO_PIN_RESET = 低电平 = LED点亮
   * 必须在配置为输出模式之前设置电平，避免上电瞬间LED闪烁
   * ========================================================================*/
  HAL_GPIO_WritePin(RUN_LED_PORT, RUN_LED_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HEART_LED_PORT, HEART_LED_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(READY_LED_PORT, READY_LED_PIN, GPIO_PIN_SET);

  /* ========================================================================
   * 步骤3: 配置LED引脚为推挽输出模式
   * 
   * 配置参数说明：
   * - Mode:     GPIO_MODE_OUTPUT_PP - 推挽输出模式
   *             推挽输出可以提供更大的驱动能力，适合驱动LED
   * - Pull:     GPIO_NOPULL - 无上下拉电阻
   *             LED有独立的限流电阻，不需要内部上下拉
   * - Speed:    GPIO_SPEED_FREQ_LOW - 低速模式
   *             LED状态变化频率低，低速模式足够且更省电
   * 
   * 三个LED共用同一组参数（Mode/Pull/Speed），仅Pin不同
   * ========================================================================*/
  
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Pin = RUN_LED_PIN;
  HAL_GPIO_Init(RUN_LED_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = HEART_LED_PIN;
  HAL_GPIO_Init(HEART_LED_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = READY_LED_PIN;
  HAL_GPIO_Init(READY_LED_PORT, &GPIO_InitStruct);
	
  /* ========================================================================
   * 步骤4: 配置CH340 USB复位引脚
   * PE10: USB_RST - CH340 USB-Serial芯片复位控制
   * 默认高电平（不复位状态），低电平触发CH340复位
   * 【2026-06-09 修复】HAL_GPIO_WritePin 需要 3 个参数 (Port, Pin, PinState)
   * ========================================================================*/
	USB_RST_PORT_CLK_EN();
	HAL_GPIO_WritePin(USB_RST_PORT, USB_RST_PIN, GPIO_PIN_SET);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
