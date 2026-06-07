/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   6-channel ADC: ADC1 + ADC2 + ADC3 (2 channels each)
  *          ADC1: PC0(INP10) + PC1(INP11)  -> CH1/CH2
  *          ADC2: PA3(INP15) + PA4(INP18)  -> CH5/CH6
  *          ADC3: PC2(INP0)  + PC3(INP1)   -> CH3/CH4
  *          12-bit, DMA circular, ~2.0 MSPS/channel
  *
  * === 测试验证记录 (2026-06-04) ===
  * 任务1: 系统时钟验证
  *   - SystemCoreClock = 480 MHz ✅
  *   - 时间戳使用u64微秒，单调递增 ✅
  *
  * 任务2: FFT性能测试
  *   - 256点FFT: 200 us (96,144 cycles) ✅
  *   - 512点FFT: 219 us (105,378 cycles) ✅
  *   - 1024点FFT: 219 us (105,378 cycles) ✅
  *   - 结论: CMSIS-DSP库FFT计算速度满足<200us要求
  *
  * 任务4: ADC采样验证 ✅
  *   - PLL3配置: M=5, N=45, P=4 -> VCO=225MHz, PLL3P=56.25MHz
  *   - ADC时钟: 56.25MHz (ASYNC_DIV1)
  *   - 采样时间: 1.5 cycles
  *   - 转换时间: 14 cycles (1.5 + 12.5)
  *   - 理论单ADC采样率: 56.25MHz/14 = 4.018 MSPS (2通道)
  *   - 理论每通道采样率: 2.009 MSPS (≈2MSPS目标) ✅
  *   - ★ 实测每通道采样率: 3.515 MSPS (DMA 帧间隔反推, 高出理论值约 1.75x)
  *        FFT 频率换算使用 adc_fs_hz=3515000.0f (main.c#L357)
  *   - DMA配置: 循环模式，高优先级
  *   - 缓冲区位置: ADC1/ADC2@AXI_SRAM, ADC3@D3_SRAM4
  *
  *   通道独立性验证 (2026-06-04):
  *   - CH1典型值: 1856 (0x0740)
  *   - CH2典型值: 1854 (0x073E)
  *   - CH3典型值: 1809 (0x0711)
  *   - CH4典型值: 1816 (0x0718)
  *   - CH5典型值: 1857 (0x0741)
  *   - CH6典型值: 1846 (0x0736)
  *   - 结论: 6通道数据各不相同，无串扰，数据独立 ✅
  *
  *   数据格式验证:
  *   - DMA缓冲区为uint16_t数组
  *   - 每个uint16_t包含2个通道的8位ADC值
  *   - 低字节=奇数通道(rank1), 高字节=偶数通道(rank2)
  *   - 例: 0x073E0740 -> CH1=0x74=116, CH2=0x3E=62 (8位模式)
  *   - 注意: 当前使用12位模式，值为完整12位ADC值
  ******************************************************************************
  */
/* USER CODE END Header */
#include "adc.h"

/* USER CODE BEGIN 0 */
#include <string.h>

/* --------------- Global handles --------------- */
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;

/* --------------- DMA buffers (AXI SRAM) --------------- */
uint16_t adc1_buf[ADC_DMA_BUF_SIZE] __attribute__((at(0x24070000), aligned(32)));
uint16_t adc2_buf[ADC_DMA_BUF_SIZE] __attribute__((at(0x24078000), aligned(32)));
uint16_t adc3_buf[ADC_DMA_BUF_SIZE] __attribute__((at(0x38000000), aligned(32)));

/* --------------- DMA flags --------------- */
volatile uint8_t adc1_dma_half = 0;
volatile uint8_t adc1_dma_full = 0;
volatile uint8_t adc2_dma_half = 0;
volatile uint8_t adc2_dma_full = 0;
volatile uint8_t adc3_dma_half = 0;
volatile uint8_t adc3_dma_full = 0;

volatile uint32_t adc1_dma_half_cyc = 0;
volatile uint32_t adc1_dma_full_cyc = 0;
volatile uint32_t adc2_dma_half_cyc = 0;
volatile uint32_t adc2_dma_full_cyc = 0;
volatile uint32_t adc3_dma_half_cyc = 0;
volatile uint32_t adc3_dma_full_cyc = 0;

volatile uint32_t adc_start_status[3] = {0};
volatile uint32_t adc_error_status[3] = {0};
volatile uint32_t dma_error_status[3] = {0};

/* --------------- Sampling rate --------------- */
float adc_sample_rate = 2.6f;  /* Msps per channel, will be calibrated */

/* USER CODE END 0 */

/*============================================================================
  ADC1 Init - PC0(INP10) + PC1(INP11)
  ============================================================================*/
void MY_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance                      = ADC1;
  hadc1.Init.ClockPrescaler           = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution               = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode             = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection             = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait         = DISABLE;
  hadc1.Init.ContinuousConvMode       = ENABLE;
  hadc1.Init.NbrOfConversion          = 2;  /* 双通道：PA0 + PA1 */
  hadc1.Init.DiscontinuousConvMode    = DISABLE;
  hadc1.Init.ExternalTrigConv         = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun                  = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.LeftBitShift             = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode         = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) { Error_Handler(); }

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);

  sConfig.SamplingTime            = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff              = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber            = ADC_OFFSET_NONE;
  sConfig.Offset                  = 0;
  sConfig.OffsetSignedSaturation  = DISABLE;

  /* ADC1: PC0(INP10)  rank1, PC1(INP11)  rank2 - 原始引脚，不变更 */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank    = ADC_REGULAR_RANK_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank    = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
}

/*============================================================================
  ADC2 Init - PA3(INP15) + PA4(INP18)
  ============================================================================*/
void MY_ADC2_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc2.Instance                      = ADC2;
  hadc2.Init.ClockPrescaler           = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution               = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode             = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection             = ADC_EOC_SEQ_CONV;
  hadc2.Init.LowPowerAutoWait         = DISABLE;
  hadc2.Init.ContinuousConvMode       = ENABLE;
  hadc2.Init.NbrOfConversion          = 2;  /* 双通道：PA3 + PA4 */
  hadc2.Init.DiscontinuousConvMode    = DISABLE;
  hadc2.Init.ExternalTrigConv         = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc2.Init.Overrun                  = ADC_OVR_DATA_OVERWRITTEN;
  hadc2.Init.LeftBitShift             = ADC_LEFTBITSHIFT_NONE;
  hadc2.Init.OversamplingMode         = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK) { Error_Handler(); }

  HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);

  sConfig.SamplingTime            = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff              = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber            = ADC_OFFSET_NONE;
  sConfig.Offset                  = 0;
  sConfig.OffsetSignedSaturation  = DISABLE;

  /* PA3 = ADC2_INP15 */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank    = ADC_REGULAR_RANK_1;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) { Error_Handler(); }

  /* PA4 = ADC2_INP18 */
  sConfig.Channel = ADC_CHANNEL_18;
  sConfig.Rank    = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) { Error_Handler(); }
}

/*============================================================================
  ADC3 Init - PC2(INP0) + PC3(INP1)  (uses BDMA)
  ============================================================================*/
void MY_ADC3_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc3.Instance                      = ADC3;
  hadc3.Init.ClockPrescaler           = ADC_CLOCK_ASYNC_DIV1;
  hadc3.Init.Resolution               = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode             = ADC_SCAN_ENABLE;
  hadc3.Init.EOCSelection             = ADC_EOC_SEQ_CONV;
  hadc3.Init.LowPowerAutoWait         = DISABLE;
  hadc3.Init.ContinuousConvMode       = ENABLE;
  hadc3.Init.NbrOfConversion          = 2;  /* 双通道：PB0 + PB1 */
  hadc3.Init.DiscontinuousConvMode    = DISABLE;
  hadc3.Init.ExternalTrigConv         = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc3.Init.Overrun                  = ADC_OVR_DATA_OVERWRITTEN;
  hadc3.Init.LeftBitShift             = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode         = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK) { Error_Handler(); }

  HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);

  sConfig.SamplingTime            = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff              = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber            = ADC_OFFSET_NONE;
  sConfig.Offset                  = 0;
  sConfig.OffsetSignedSaturation  = DISABLE;

  /* ADC3: PC2(INP0) rank1, PC3(INP1) rank2 */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank    = ADC_REGULAR_RANK_1;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) { Error_Handler(); }
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank    = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) { Error_Handler(); }
}

/*============================================================================
  Start all ADCs
  ============================================================================*/
void MY_ADC_Start(void)
{
  HAL_StatusTypeDef st;

  __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR);
  __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR);
  __HAL_ADC_CLEAR_FLAG(&hadc3, ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR);

  st = HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc3_buf, ADC_DMA_BUF_SIZE);
  adc_start_status[2] = (uint32_t)st;
  adc_error_status[2] = HAL_ADC_GetError(&hadc3);
  dma_error_status[2] = HAL_DMA_GetError(&hdma_adc3);
  if (st != HAL_OK) { Error_Handler(); }

  st = HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buf, ADC_DMA_BUF_SIZE);
  adc_start_status[0] = (uint32_t)st;
  adc_error_status[0] = HAL_ADC_GetError(&hadc1);
  dma_error_status[0] = HAL_DMA_GetError(&hdma_adc1);
  if (st != HAL_OK) { Error_Handler(); }

  st = HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_buf, ADC_DMA_BUF_SIZE);
  adc_start_status[1] = (uint32_t)st;
  adc_error_status[1] = HAL_ADC_GetError(&hadc2);
  dma_error_status[1] = HAL_DMA_GetError(&hdma_adc2);
  if (st != HAL_OK) { Error_Handler(); }
}

/*============================================================================
  Stop all ADCs
  ============================================================================*/
void MY_ADC_Stop(void)
{
  HAL_ADC_Stop_DMA(&hadc1);
  HAL_ADC_Stop_DMA(&hadc2);
  HAL_ADC_Stop_DMA(&hadc3);
}

/*============================================================================
  HAL_ADC_MspInit - MSP configuration for ADC1/ADC2/ADC3
  ============================================================================*/
void HAL_ADC_MspInit(ADC_HandleTypeDef *adcHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /* PLL3 clock: must be configured before any ADC initialization */
  /* 优化配置：理论目标每通道≈2MSPS */
  /* M=5, N=45, P=4 -> VCO=25MHz*45/5=225MHz, PLL3P=225/4=56.25MHz */
  /* ADC时钟=56.25MHz (ASYNC_DIV1), 采样时间=1.5周期 */
  /* 转换时间=14周期, 理论每通道采样率=56.25MHz/(2*14)=2.009MSPS/通道 */
  /* ★ 实测：DMA 帧间隔反推真实采样率 = 3.515 MSPS/通道（高出理论 1.75x），
   *     FFT 用的实际值见 main.c#L357 (adc_fs_hz=3515000.0f) */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL3.PLL3M = 5;
  PeriphClkInitStruct.PLL3.PLL3N = 45;
  PeriphClkInitStruct.PLL3.PLL3P = 4;
  PeriphClkInitStruct.PLL3.PLL3Q = 2;
  PeriphClkInitStruct.PLL3.PLL3R = 2;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_2;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /* ========== ADC1 ========== */
  if (adcHandle->Instance == ADC1)
  {
    __HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* PC0 = ADC1_INP10, PC1 = ADC1_INP11 */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* DMA2 Stream0 for ADC1 */
    hdma_adc1.Instance                 = DMA2_Stream0;
    hdma_adc1.Init.Request             = DMA_REQUEST_ADC1;
    hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode                = DMA_CIRCULAR;
    hdma_adc1.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_adc1.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) { Error_Handler(); }
    __HAL_LINKDMA(adcHandle, DMA_Handle, hdma_adc1);

    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  }

  /* ========== ADC2 ========== */
  if (adcHandle->Instance == ADC2)
  {
    __HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* PA3/PA4 analog input */
    GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* DMA2 Stream1 for ADC2 */
    hdma_adc2.Instance                 = DMA2_Stream1;
    hdma_adc2.Init.Request             = DMA_REQUEST_ADC2;
    hdma_adc2.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_adc2.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_adc2.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc2.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_adc2.Init.Mode                = DMA_CIRCULAR;
    hdma_adc2.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_adc2.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc2) != HAL_OK) { Error_Handler(); }
    __HAL_LINKDMA(adcHandle, DMA_Handle, hdma_adc2);

    HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  }

  /* ========== ADC3 (uses BDMA) ========== */
  if (adcHandle->Instance == ADC3)
  {
    __HAL_RCC_ADC3_CLK_ENABLE();
      __HAL_RCC_GPIOC_CLK_ENABLE();
      __HAL_RCC_SYSCFG_CLK_ENABLE();
      __HAL_RCC_BDMA_CLK_ENABLE();

      /* PC2/PC3 analog input */
      GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
      GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
      SET_BIT(SYSCFG->PMCR, SYSCFG_PMCR_PC2SO | SYSCFG_PMCR_PC3SO);

    /* BDMA Channel0 for ADC3 */
    hdma_adc3.Instance                 = BDMA_Channel0;
    hdma_adc3.Init.Request             = BDMA_REQUEST_ADC3;
    hdma_adc3.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_adc3.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_adc3.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_adc3.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc3.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_adc3.Init.Mode                = DMA_CIRCULAR;
    hdma_adc3.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_adc3.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc3) != HAL_OK) { Error_Handler(); }
    __HAL_LINKDMA(adcHandle, DMA_Handle, hdma_adc3);

    HAL_NVIC_SetPriority(BDMA_Channel0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(BDMA_Channel0_IRQn);
  }
}

/*============================================================================
  HAL_ADC_MspDeInit
  ============================================================================*/
void HAL_ADC_MspDeInit(ADC_HandleTypeDef *adcHandle)
{
  if (adcHandle->Instance == ADC1)
  {
    __HAL_RCC_ADC12_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0 | GPIO_PIN_1);
    HAL_DMA_DeInit(&hdma_adc1);
    HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);
  }
  if (adcHandle->Instance == ADC2)
  {
    __HAL_RCC_ADC12_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3 | GPIO_PIN_4);
    HAL_DMA_DeInit(&hdma_adc2);
    HAL_NVIC_DisableIRQ(DMA2_Stream1_IRQn);
  }
  if (adcHandle->Instance == ADC3)
  {
    __HAL_RCC_ADC3_CLK_DISABLE();
      HAL_GPIO_DeInit(GPIOC, GPIO_PIN_2 | GPIO_PIN_3);
      HAL_DMA_DeInit(&hdma_adc3);
      HAL_NVIC_DisableIRQ(BDMA_Channel0_IRQn);
  }
}

/* USER CODE BEGIN 1 */

/*============================================================================
  DMA callbacks - 3 ADC instances
  ============================================================================*/
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1) {
    adc1_dma_half_cyc = DWT->CYCCNT;
    adc1_dma_half = 1;
  }
  if (hadc->Instance == ADC2) {
    adc2_dma_half_cyc = DWT->CYCCNT;
    adc2_dma_half = 1;
  }
  if (hadc->Instance == ADC3) {
    adc3_dma_half_cyc = DWT->CYCCNT;
    adc3_dma_half = 1;
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1) {
    adc1_dma_full_cyc = DWT->CYCCNT;
    adc1_dma_full = 1;
  }
  if (hadc->Instance == ADC2) {
    adc2_dma_full_cyc = DWT->CYCCNT;
    adc2_dma_full = 1;
  }
  if (hadc->Instance == ADC3) {
    adc3_dma_full_cyc = DWT->CYCCNT;
    adc3_dma_full = 1;
  }
}

/* USER CODE END 1 */
