/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dac.c
  * @brief   DAC1_CH2 (PA5) sine wave + DC generator
  *          - DC mode: hold 1.65V (2048)
  *          - SINE mode: 50KHz sine wave via TIM7 + DMA
  *          Note: PA4 (DAC1_CH1) is NOT used to avoid conflict with ADC CH6
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "dac.h"
#include "tim.h"
#include <math.h>

/* USER CODE BEGIN 0 */

#define SINE_TABLE_LEN 32       /* 32点波形表 */
#define DAC_OFFSET     2047     /* 中心: 1.65V (4095/2)，最大输出 ±1.65V = 3.3Vpp */
#define DAC_AMPLITUDE  1241     /* 默认振幅 1V */
#define DAC_DC_VALUE   2047     /* 直流电平: 1.65V，与正弦中心一致避免起跳阶跃 */
#define DAC_PWM_HIGH   3103     /* PWM高电平: 2.5V */
#define DAC_PWM_LOW    621      /* PWM低电平: 0.5V */

static uint32_t dac_sine_table[SINE_TABLE_LEN] __attribute__((at(0x2406F000), aligned(32)));
static uint32_t dac_pwm_table[SINE_TABLE_LEN] __attribute__((at(0x2406F080), aligned(32)));
static uint8_t sine_table_inited = 0;
static uint16_t current_sine_amp = DAC_AMPLITUDE;  /* 当前正弦振幅 */
volatile uint32_t dac_diag_tim_clk_hz = 0;
volatile uint32_t dac_diag_arr = 0;
volatile uint32_t dac_diag_actual_freq_hz = 0;

TIM_HandleTypeDef htim7;

/* USER CODE END 0 */

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;
DMA_HandleTypeDef hdma_dac1_ch2;

/* DAC1 init - 仅使用CH2 (PA5)，避开PA4 */
void MX_DAC1_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  __HAL_RCC_DAC12_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* 仅 PA5 = DAC1_OUT2，PA4 留给 ADC CH6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  hdac1.Instance = DAC1;
  
  /* 重置DAC */
  DAC1->CR = 0;
  DAC1->MCR = 0;
  
  /* DMA1_Stream1 - TIM7_UP drives writes to DAC1_CH2 data register.
   * This avoids DAC TSEL2 encoding differences and keeps PA5 timing locked to TIM7.
   */
  hdma_dac1_ch2.Instance = DMA1_Stream1;
  hdma_dac1_ch2.Init.Request = DMA_REQUEST_TIM7_UP;
  hdma_dac1_ch2.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_dac1_ch2.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_dac1_ch2.Init.MemInc = DMA_MINC_ENABLE;
  hdma_dac1_ch2.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_dac1_ch2.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_dac1_ch2.Init.Mode = DMA_CIRCULAR;
  hdma_dac1_ch2.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_dac1_ch2.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  HAL_DMA_Init(&hdma_dac1_ch2);
}

/* TIM7 init - DAC2 trigger timer (用于正弦波) */
void MX_TIM7_Init(void)
{
  __HAL_RCC_TIM7_CLK_ENABLE();
  
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 124;  /* 默认 125 -> 触发 ≈ 1.6MHz (50KHz × 32) */
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_Base_Init(&htim7);

  {
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig);
  }
}

/* USER CODE BEGIN 1 */

/* 初始化波形表（使用 current_sine_amp 作为振幅）*/
static void dac_sine_table_init(void)
{
  int i;
  float phase;
  for(i = 0; i < SINE_TABLE_LEN; i++) {
    phase = 2.0f * 3.14159265358979f * (float)i / (float)SINE_TABLE_LEN;
    dac_sine_table[i] = (uint32_t)(DAC_OFFSET + (float)current_sine_amp * sinf(phase));
    /* PWM 方波表: 前半HIGH，后半LOW */
    if(i < SINE_TABLE_LEN / 2) {
      dac_pwm_table[i] = DAC_PWM_HIGH;
    } else {
      dac_pwm_table[i] = DAC_PWM_LOW;
    }
  }
}

/* 设置正弦波振幅（单位: 毫伏 mV, 范围 50~1650）*/
void DAC_Set_Sine_Amplitude_mV(uint16_t amp_mv)
{
  /* mV -> DAC value: amp_mv / 3300 * 4095 */
  uint32_t amp_dac = (uint32_t)amp_mv * 4095U / 3300U;
  /* 限制范围: 不能超过 DAC_OFFSET 也不能为 0 */
  if(amp_dac > DAC_OFFSET) amp_dac = DAC_OFFSET;
  if(amp_dac < 10) amp_dac = 10;
  current_sine_amp = (uint16_t)amp_dac;
  /* 重新生成正弦表 */
  dac_sine_table_init();
  sine_table_inited = 1;
}

/* 设置 CH2 正弦波频率 */
void DAC_Set_CH2_Freq(uint32_t sine_freq_hz)
{
  uint32_t tim_clk_hz;
  uint32_t trigger_rate;
  uint32_t arr;
  uint32_t ppre1;

  if(sine_freq_hz == 0U) return;

  tim_clk_hz = HAL_RCC_GetPCLK1Freq();
  ppre1 = (RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) >> RCC_D2CFGR_D2PPRE1_Pos;
  if(ppre1 >= 4U) {
    tim_clk_hz *= 2U;
  }

  trigger_rate = sine_freq_hz * SINE_TABLE_LEN;
  arr = (tim_clk_hz + (trigger_rate / 2U)) / trigger_rate;
  if(arr < 2U) arr = 2U;
  dac_diag_tim_clk_hz = tim_clk_hz;
  dac_diag_arr = arr - 1U;
  dac_diag_actual_freq_hz = tim_clk_hz / (arr * SINE_TABLE_LEN);
  htim7.Instance->ARR = arr - 1U;
  htim7.Instance->CNT = 0;
  htim7.Instance->EGR = TIM_EGR_UG;
}

/* 占位符，CH1 不可用 */
void DAC_Set_CH1_Freq(uint32_t sine_freq_hz) { (void)sine_freq_hz; }

/* 启动DAC输出直流模式 - 1.65V
 * 不需要DMA和定时器触发，直接软件写值
 */
void DAC_Output_DC(void)
{
  uint32_t cr;

  /* 停止DMA和触发 */
  HAL_DMA_Abort(&hdma_dac1_ch2);
  HAL_TIM_Base_Stop(&htim7);
  htim7.Instance->DIER &= ~TIM_DIER_UDE;
  
  /* 关闭CH2输出但保留HAL配置的触发源 */
  DAC1->CR &= ~((1UL << 16) | (1UL << 17) | (1UL << 28) | (1UL << 30));
  
  /* 写入直流值 */
  DAC1->DHR12R2 = DAC_DC_VALUE;
  
  /* 配置 DAC CH2: 软件触发模式 (TEN=0, 直接输出DHR的值)
   * EN2 = 1
   * TEN2 = 0 (no trigger, direct output)
   */
  cr = DAC1->CR;
  cr &= ~((1UL << 17) | (1UL << 28) | (1UL << 30));
  cr |= (1UL << 16);       /* EN2 only */
  DAC1->CR = cr;
}

/* 启动DAC输出PWM方波（用查表实现，频率 = TIM7触发频率 / 32） */
void DAC_Output_PWM(uint32_t freq_hz)
{
  uint32_t cr;
  volatile int d;

  if(!sine_table_inited) {
    dac_sine_table_init();
    sine_table_inited = 1;
  }
  
  /* 完全关闭DAC并停止之前的DMA和定时器 */
  DAC1->CR &= ~((1UL << 16) | (1UL << 17) | (1UL << 28) | (1UL << 30));
  HAL_DMA_Abort(&hdma_dac1_ch2);
  HAL_TIM_Base_Stop(&htim7);
  htim7.Instance->DIER &= ~TIM_DIER_UDE;
  
  /* 短暂延迟让DAC完全关闭 */
  for(d = 0; d < 100; d++);
  
  /* 设置频率 */
  DAC_Set_CH2_Freq(freq_hz);
  
  SCB_CleanDCache_by_Addr((uint32_t*)dac_pwm_table, SINE_TABLE_LEN * 4);

  /* 启动 DMA1_Stream1 -> DAC1->DHR12R2 (使用PWM表) */
  HAL_DMA_Start(&hdma_dac1_ch2, (uint32_t)dac_pwm_table,
                (uint32_t)&DAC1->DHR12R2, SINE_TABLE_LEN);
  
  /* 启动 TIM7 Update DMA 触发 */
  htim7.Instance->DIER |= TIM_DIER_UDE;
  HAL_TIM_Base_Start(&htim7);
  
  /* DAC CH2 direct output; DMA is triggered by TIM7_UP through DMAMUX. */
  cr = DAC1->CR;
  cr &= ~((1UL << 17) | (1UL << 28) | (1UL << 30));
  cr |= (1UL << 16);
  DAC1->CR = cr;
}

/* 启动DAC输出正弦波 (50KHz) */
void DAC_Output_Sine(uint32_t freq_hz)
{
  uint32_t cr;
  volatile int d;

  if(!sine_table_inited) {
    dac_sine_table_init();
    sine_table_inited = 1;
  }
  
  /* 完全关闭DAC并停止之前的DMA和定时器 */
  DAC1->CR &= ~((1UL << 16) | (1UL << 17) | (1UL << 28) | (1UL << 30));
  HAL_DMA_Abort(&hdma_dac1_ch2);
  HAL_TIM_Base_Stop(&htim7);
  htim7.Instance->DIER &= ~TIM_DIER_UDE;
  
  /* 短暂延迟让DAC完全关闭 */
  for(d = 0; d < 100; d++);
  
  /* 设置频率 */
  DAC_Set_CH2_Freq(freq_hz);
  
  SCB_CleanDCache_by_Addr((uint32_t*)dac_sine_table, SINE_TABLE_LEN * 4);

  /* 启动 DMA1_Stream1 -> DAC1->DHR12R2 */
  HAL_DMA_Start(&hdma_dac1_ch2, (uint32_t)dac_sine_table,
                (uint32_t)&DAC1->DHR12R2, SINE_TABLE_LEN);
  
  /* 启动 TIM7 Update DMA 触发 */
  htim7.Instance->DIER |= TIM_DIER_UDE;
  HAL_TIM_Base_Start(&htim7);
  
  /* DAC CH2 direct output; DMA is triggered by TIM7_UP through DMAMUX. */
  cr = DAC1->CR;
  cr &= ~((1UL << 17) | (1UL << 28) | (1UL << 30));
  cr |= (1UL << 16);
  DAC1->CR = cr;
}

/* 原有兼容接口 */
void MY_DAC_Start(void)
{
  /* 默认启动正弦模式 50KHz */
  DAC_Output_Sine(50000);
}

void MY_DAC_Stop(void)
{
  HAL_DMA_Abort(&hdma_dac1_ch2);
  DAC1->CR = 0;
  HAL_TIM_Base_Stop(&htim7);
  htim7.Instance->DIER &= ~TIM_DIER_UDE;
}

/* ==== 简化版HAL函数 ==== */
HAL_StatusTypeDef HAL_DAC_Init(DAC_HandleTypeDef* hdac)
{
  if(hdac->Instance == DAC1) __HAL_RCC_DAC12_CLK_ENABLE();
  return HAL_OK;
}

HAL_StatusTypeDef HAL_DAC_Start(DAC_HandleTypeDef* hdac, uint32_t Channel)
{
  if(Channel == DAC_CHANNEL_1) hdac->Instance->CR |= (1UL << 0);
  else if(Channel == DAC_CHANNEL_2) hdac->Instance->CR |= (1UL << 16);
  return HAL_OK;
}

HAL_StatusTypeDef HAL_DAC_Stop(DAC_HandleTypeDef* hdac, uint32_t Channel)
{
  if(Channel == DAC_CHANNEL_1) hdac->Instance->CR &= ~(1UL << 0);
  else if(Channel == DAC_CHANNEL_2) hdac->Instance->CR &= ~(1UL << 16);
  return HAL_OK;
}

HAL_StatusTypeDef HAL_DAC_SetValue(DAC_HandleTypeDef* hdac, uint32_t Channel, uint32_t Alignment, uint32_t Data)
{
  if(Channel == DAC_CHANNEL_1) hdac->Instance->DHR12R1 = Data;
  else if(Channel == DAC_CHANNEL_2) hdac->Instance->DHR12R2 = Data;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_DAC_Start_DMA(DAC_HandleTypeDef* hdac, uint32_t Channel, uint32_t* pData, uint32_t Length, uint32_t Alignment)
{
  if(Channel == DAC_CHANNEL_1) hdac->Instance->CR |= (1UL << 12) | (1UL << 0);  /* DMAEN1=bit12 */
  else if(Channel == DAC_CHANNEL_2) hdac->Instance->CR |= (1UL << 28) | (1UL << 16);  /* DMAEN2=bit28 */
  return HAL_OK;
}

HAL_StatusTypeDef HAL_DAC_Stop_DMA(DAC_HandleTypeDef* hdac, uint32_t Channel)
{
  if(Channel == DAC_CHANNEL_1) hdac->Instance->CR &= ~(1UL << 12);
  else if(Channel == DAC_CHANNEL_2) hdac->Instance->CR &= ~(1UL << 28);
  return HAL_OK;
}

/* USER CODE END 1 */
