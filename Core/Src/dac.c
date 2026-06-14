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

/* 非阻塞 DMA 停止函数
 * 替代 HAL_DMA_Abort()（内部有 5ms 轮询超时，会阻塞主循环导致 ADC 帧丢失）
 * 原理：直接操作 DMA Stream CR 寄存器禁用，再清理 HAL 状态，零等待
 * 仅用于 DAC DMA（DMA1_Stream1），不涉及中断回调清理 */
static void DAC_DMA_Stop_NonBlocking(DMA_HandleTypeDef *hdma)
{
  DMA_Stream_TypeDef *stream = (DMA_Stream_TypeDef *)hdma->Instance;

  /* 1. 禁用 DMA Stream */
  stream->CR &= ~DMA_SxCR_EN;

  /* 2. 等待 EN 位清零（硬件需要几个时钟周期完成当前传输） */
  {
    volatile uint32_t timeout = 100U;
    while((stream->CR & DMA_SxCR_EN) && (--timeout)) { ; }
  }

  /* 3. 清除所有 DMA 中断标志（防止残留中断触发） */
  /* DMA1 Stream1 的 LIFCR 寄存器位: CT1IF=bit11, CB1IF=bit10, CTCIF=bit9, CHTIF=bit8, CTEIF=bit6 */
  DMA1->LIFCR = (DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1
               | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1);

  /* 4. 更新 HAL 状态 */
  hdma->State = HAL_DMA_STATE_READY;
}

#define SINE_TABLE_LEN 32       /* v9: 32→256→32（256 导致 DMA1 ISR 风暴卡死 UART TX，回退到 32）
                                 *     32 点 × 40KHz = 1.28 MHz DMA 触发频率（可承受）
                                 *     合成谐波只能到 (32/2-1)=15 次以下，2/3 次谐波仍 OK */
#define DAC_OFFSET     2047     /* 中心: 1.65V (4095/2)，最大输出 ±1.65V = 3.3Vpp */
#define DAC_AMPLITUDE  1241     /* 默认振幅 1V */
#define DAC_DC_VALUE   2047     /* 直流电平: 1.65V，与正弦中心一致避免起跳阶跃 */
#define DAC_PWM_HIGH   3103     /* PWM高电平: 2.5V */
#define DAC_PWM_LOW    621      /* PWM低电平: 0.5V */

/* AXI SRAM (0x24000000~0x2407FFFF, 512KB)
 * 必须用 .at() 固定地址（不能用 section(".AXI_SRAM")），原因：
 *   uVision 自动生成的 sct 用 .ANY (+RW +ZI) 把所有 RW 数据塞入 DTCM (0x20000000)，
 *   导致 dac_sine_table 落到 DMA1 不可访问的 DTCM 区，PA5 永远只输出 DC。
 * 修复：用 at() 强制固定到 AXI SRAM 0x2406E000（距 ADC1 buffer 0x24070000 留 8KB 安全间隔）。 */
static uint32_t dac_sine_table[SINE_TABLE_LEN] __attribute__((at(0x2406E000), aligned(32)));
static uint32_t dac_pwm_table[SINE_TABLE_LEN]  __attribute__((at(0x2406E400), aligned(32)));
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

  /* DMA1_Stream1 - TIM7_UP request periodically writes waveform samples to DAC1->DHR12R2. */
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
  DMAMUX1_Channel1->CCR = DMA_REQUEST_TIM7_UP;
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
    dac_sine_table[i] = (uint32_t)(DAC_OFFSET + (float)current_sine_amp * cosf(phase));
    /* PWM 方波表: 前半HIGH，后半LOW */
    if(i < SINE_TABLE_LEN / 2) {
      dac_pwm_table[i] = DAC_PWM_HIGH;
    } else {
      dac_pwm_table[i] = DAC_PWM_LOW;
    }
  }
}

/* =============================================================================
 * v9 增强：合成波形表 - 基波 + 谐波 + 白噪声
 * =============================================================================
 *  amp_fund_mv : 基波幅值 (mV, 单端，<=1650)
 *  h2_pct      : 二次谐波占基波幅值百分比 (0..100)
 *  h3_pct      : 三次谐波占基波幅值百分比 (0..100)
 *  noise_pct   : 白噪声占基波幅值百分比 (0..100)，使用 LFSR 伪随机
 *
 *  应用：模拟真实信号（带谐波失真 + 背景噪声），用于压力测试 ADC + FFT 鲁棒性
 * ========================================================================= */
static uint32_t dac_lfsr = 0xACE1u;
static uint16_t dac_quick_rand(void)
{
  /* 16-bit LFSR Galois, period 65535 */
  uint16_t lsb;
  lsb = (uint16_t)(dac_lfsr & 1U);
  dac_lfsr >>= 1;
  if(lsb) dac_lfsr ^= 0xB400u;
  return (uint16_t)dac_lfsr;
}

void DAC_Build_Composite_Waveform(uint16_t amp_fund_mv,
                                  uint8_t h2_pct, uint8_t h3_pct,
                                  uint8_t noise_pct)
{
  int i;
  float fund_dac;
  float h2_dac;
  float h3_dac;
  float noise_dac;
  float phase;
  float val;
  int32_t ival;
  uint32_t amp_dac;

  if(amp_fund_mv == 0U) amp_fund_mv = 1000U;
  if(amp_fund_mv > 1650U) amp_fund_mv = 1650U;
  amp_dac = (uint32_t)amp_fund_mv * 4095U / 3300U;
  if(amp_dac > DAC_OFFSET) amp_dac = DAC_OFFSET;
  fund_dac  = (float)amp_dac;
  h2_dac    = fund_dac * (float)h2_pct / 100.0f;
  h3_dac    = fund_dac * (float)h3_pct / 100.0f;
  noise_dac = fund_dac * (float)noise_pct / 100.0f;

  for(i = 0; i < SINE_TABLE_LEN; i++) {
    phase = 2.0f * 3.14159265358979f * (float)i / (float)SINE_TABLE_LEN;
    val  = fund_dac * cosf(phase);
    val += h2_dac   * cosf(2.0f * phase);
    val += h3_dac   * cosf(3.0f * phase);
    if(noise_pct > 0U) {
      /* (-1..+1) × noise_dac */
      val += ((float)dac_quick_rand() / 32767.5f - 1.0f) * noise_dac;
    }
    ival = (int32_t)(val + (float)DAC_OFFSET + 0.5f);
    if(ival < 0) ival = 0;
    if(ival > 4095) ival = 4095;
    dac_sine_table[i] = (uint32_t)ival;
  }
  current_sine_amp = (uint16_t)amp_dac;
  sine_table_inited = 1;

  /* AXI SRAM 上的表必须 clean DCache 后 DMA 才能读到最新数据 */
  SCB_CleanDCache_by_Addr((uint32_t*)dac_sine_table, SINE_TABLE_LEN * 4);
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
  DAC_DMA_Stop_NonBlocking(&hdma_dac1_ch2);
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
  DAC_DMA_Stop_NonBlocking(&hdma_dac1_ch2);
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
  /* 关闭 DAC DMA 所有中断使能位（同 DAC_Output_Sine 注释） */
  ((DMA_Stream_TypeDef*)hdma_dac1_ch2.Instance)->CR &= ~(DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE);
  
  /* TIM7_UP 直接触发 DMA 写 DHR12R2；DAC CH2 关闭触发，DHR 写入后 PA5 直接跟随。 */
  cr = DAC1->CR;
  cr &= ~((0xFUL << 18) | (1UL << 17) | (1UL << 28) | (1UL << 30));
  cr |= (1UL << 16);      /* EN2 */
  DAC1->CR = cr;

  htim7.Instance->DIER |= TIM_DIER_UDE;
  HAL_TIM_Base_Start(&htim7);
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
  DAC_DMA_Stop_NonBlocking(&hdma_dac1_ch2);
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
  /* 【关键】关闭 DAC DMA 的所有中断使能位
   * HAL_DMA_Start 内部会 enable HTIE/TCIE/TEIE/DMEIE，但 DAC 在循环模式下根本不需要 ISR，
   * 每秒 256×40K=10M 次 HT/TC 中断会让 CPU 100% 卡在 IRQHandler 里，主循环饿死。 */
  ((DMA_Stream_TypeDef*)hdma_dac1_ch2.Instance)->CR &= ~(DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE);
  DMA1->LIFCR = (DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1
               | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1);
  ((DMA_Stream_TypeDef*)hdma_dac1_ch2.Instance)->CR |= DMA_SxCR_EN;

  /* TIM7_UP 直接触发 DMA 写 DHR12R2；DAC CH2 关闭触发，DHR 写入后 PA5 直接跟随。 */
  cr = DAC1->CR;
  cr &= ~((0xFUL << 18) | (1UL << 17) | (1UL << 28) | (1UL << 30));
  cr |= (1UL << 16);      /* EN2 */
  DAC1->CR = cr;

  htim7.Instance->DIER |= TIM_DIER_UDE;
  HAL_TIM_Base_Start(&htim7);
}

/* 原有兼容接口 */
void MY_DAC_Start(void)
{
  /* 默认启动正弦模式 50KHz */
  DAC_Output_Sine(50000);
}

void MY_DAC_Stop(void)
{
  DAC_DMA_Stop_NonBlocking(&hdma_dac1_ch2);
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
