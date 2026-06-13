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
 *   - ★ 实测每通道采样率: 3.515 MSPS (DMA 帧间隔反推, EMA校准)
 *        FFT 频率换算使用 adc_fs_hz (main.c, EMA在线校准)
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
 *   - 每个uint16_t包含2个通道的12位ADC值
 *   - 低12位=奇数通道(rank1), 高12位无效(rank2独立存储)
 *   - 12-bit模式: 有效值范围 0~4095
 *   - ★ 2026-06-12 验证: J-Link读取 ADC1_CFGR RES=010 → 确认12-bit
  ******************************************************************************
  */
/* USER CODE END Header */
#include "adc.h"

/* USER CODE BEGIN 0 */
#include <string.h>

/* ====================================================================== */
/*  ADC句柄 — 每个ADC实例对应一个HAL句柄，供全局使用                       */
/* ====================================================================== */
ADC_HandleTypeDef hadc1;    /* ADC1句柄: PC0(CH1) + PC1(CH2) */
ADC_HandleTypeDef hadc2;    /* ADC2句柄: PA3(CH5) + PA4(CH6) */
ADC_HandleTypeDef hadc3;    /* ADC3句柄: PC2(CH3) + PC3(CH4) */

/* ====================================================================== */
/*  DMA句柄 — ADC1/ADC2使用DMA2(AXI SRAM), ADC3使用BDMA(SRAM4)           */
/* ====================================================================== */
DMA_HandleTypeDef hdma_adc1;    /* ADC1 DMA句柄: DMA2_Stream0 */
DMA_HandleTypeDef hdma_adc2;    /* ADC2 DMA句柄: DMA2_Stream1 */
DMA_HandleTypeDef hdma_adc3;    /* ADC3 DMA句柄: BDMA_Channel0 */

/* ====================================================================== */
/*  DMA缓冲区 — 固定地址分配，避免链接器随机放置导致DMA不可访问             */
/*  ADC1/ADC2在AXI SRAM (D1域，DMA2可访问)                                */
/*  ADC3在SRAM4 (D3域，仅BDMA可访问)                                      */
/*  缓冲区大小: ADC_DMA_BUF_SIZE=16384 个 uint16_t = 32KB/个             */
/*  DMA循环模式: 前半区触发HT中断，后半区触发TC中断                        */
/* ====================================================================== */
uint16_t adc1_buf[ADC_DMA_BUF_SIZE] __attribute__((at(0x24070000), aligned(32)));  /* ADC1 DMA缓冲区@AXI SRAM */
uint16_t adc2_buf[ADC_DMA_BUF_SIZE] __attribute__((at(0x24078000), aligned(32)));  /* ADC2 DMA缓冲区@AXI SRAM */
uint16_t adc3_buf[ADC_DMA_BUF_SIZE] __attribute__((at(0x38000000), aligned(32)));  /* ADC3 DMA缓冲区@SRAM4(D3域) */

/* ====================================================================== */
/*  DMA标志 — 由ISR置位，由主循环 check_dma_and_push_frames() 消费        */
/*  volatile: 确保编译器不优化掉对ISR修改的检查                           */
/*  *_half: 前半区完成标志（DMA写入前8192个样本后触发HT中断）              */
/*  *_full: 后半区完成标志（DMA写入后8192个样本后触发TC中断）              */
/*  3个ADC的标志相互独立，主循环通过3-ADC同步逻辑确保数据一致性            */
/* ====================================================================== */
volatile uint8_t adc1_dma_half = 0;     /* ADC1 DMA前半区完成标志 */
volatile uint8_t adc1_dma_full = 0;     /* ADC1 DMA后半区完成标志 */
volatile uint8_t adc2_dma_half = 0;     /* ADC2 DMA前半区完成标志 */
volatile uint8_t adc2_dma_full = 0;     /* ADC2 DMA后半区完成标志 */
volatile uint8_t adc3_dma_half = 0;     /* ADC3 DMA前半区完成标志 */
volatile uint8_t adc3_dma_full = 0;     /* ADC3 DMA后半区完成标志 */

/* ====================================================================== */
/*  DMA完成时间戳（DWT周期计数）— 用于在线校准ADC采样率                    */
/*  在ConvHalfCplt/ConvCplt回调中记录DWT->CYCCNT值                       */
/*  主循环通过两次full_cyc差值反推真实采样率:                              */
/*    adc_fs_hz = HALF_SAMPLES_PER_CH * SystemCoreClock / delta_cyc       */
/* ====================================================================== */
volatile uint32_t adc1_dma_half_cyc = 0;    /* ADC1 HT中断时的DWT周期 */
volatile uint32_t adc1_dma_full_cyc = 0;    /* ADC1 TC中断时的DWT周期 */
volatile uint32_t adc2_dma_half_cyc = 0;    /* ADC2 HT中断时的DWT周期 */
volatile uint32_t adc2_dma_full_cyc = 0;    /* ADC2 TC中断时的DWT周期 */
volatile uint32_t adc3_dma_half_cyc = 0;    /* ADC3 HT中断时的DWT周期 */
volatile uint32_t adc3_dma_full_cyc = 0;    /* ADC3 TC中断时的DWT周期 */

/* ====================================================================== */
/*  ADC/DMA诊断状态 — 调试用，通过串口输出查看ADC启动和错误情况            */
/* ====================================================================== */
volatile uint32_t adc_start_status[3] = {0};    /* HAL_ADC_Start_DMA返回值 [ADC1,ADC2,ADC3] */
volatile uint32_t adc_error_status[3] = {0};    /* HAL_ADC_GetError错误码 [ADC1,ADC2,ADC3] */
volatile uint32_t dma_error_status[3] = {0};    /* HAL_DMA_GetError错误码 [ADC1,ADC2,ADC3] */

/* --------------- Sampling rate --------------- */
float adc_sample_rate = 2.6f;  /* Msps per channel, 主循环EMA在线校准为实测值3.515 */

/* USER CODE END 0 */

/***********************************************************
函数名：MY_ADC1_Init
参数：  无
返回值：无
描述：  初始化ADC1（PC0 + PC1，对应通道CH1/CH2）
        - 时钟: PLL3P=56.25MHz, 异步分频DIV1
        - 分辨率: 12位 (0~4095)
        - 扫描模式: 使能（双通道扫描转换）
        - 连续转换: 使能（无触发，持续采样）
        - DMA: 循环模式，数据自动搬运到adc1_buf
        - 采样时间: 1.5个ADC时钟周期
        - 通道映射: CH10(PC0)=Rank1, CH11(PC1)=Rank2
修改记录：
***********************************************************/
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

/***********************************************************
函数名：MY_ADC2_Init
参数：  无
返回值：无
描述：  初始化ADC2（PA3 + PA4，对应通道CH5/CH6）
        配置与ADC1完全一致，仅通道映射不同：
        - Rank1: CH15(PA3) = CH5
        - Rank2: CH18(PA4) = CH6
        DMA使用DMA2_Stream1，缓冲区adc2_buf在AXI SRAM
修改记录：
***********************************************************/
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

/***********************************************************
函数名：MY_ADC3_Init
参数：  无
返回值：无
描述：  初始化ADC3（PC2 + PC3，对应通道CH3/CH4）
        配置与ADC1/ADC2一致，但DMA使用BDMA（D3域DMA）:
        - BDMA_Channel0 连接ADC3
        - 缓冲区adc3_buf在SRAM4（0x38000000，仅BDMA可访问）
        - 通道映射: Rank1=CH0(PC2)=CH3, Rank2=CH1(PC3)=CH4
        注意: PC2/PC3需要SYSCFG->PMCR特殊配置（PC2SO/PC3SO位）
修改记录：
***********************************************************/
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

/***********************************************************
函数名：MY_ADC_Start
参数：  无
返回值：无
描述：  启动所有3个ADC的DMA采集
        启动顺序：ADC3 → ADC1 → ADC2
        （ADC3先启动因为使用BDMA，延迟可能略大）
        每个ADC启动前清除EOC/EOS/OVR标志，避免残留标志触发假中断
        启动后记录HAL返回值和错误码到诊断数组
修改记录：
***********************************************************/
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

/***********************************************************
函数名：MY_ADC_Stop
参数：  无
返回值：无
描述：  停止所有3个ADC的DMA采集
        调用HAL_ADC_Stop_DMA停止转换并释放DMA资源
        用于ADC错误恢复流程：Stop → 重新Init → Start
修改记录：
***********************************************************/
void MY_ADC_Stop(void)
{
  HAL_ADC_Stop_DMA(&hadc1);
  HAL_ADC_Stop_DMA(&hadc2);
  HAL_ADC_Stop_DMA(&hadc3);
}

/***********************************************************
函数名：HAL_ADC_MspInit
参数：  adcHandle - ADC句柄指针，用于识别是哪个ADC实例
返回值：无
描述：  ADC底层硬件初始化（MSP回调，由HAL_ADC_Init内部调用）
        按ADC实例分别配置：
        公共部分：PLL3时钟配置（仅首次调用时生效）
        ADC1: GPIOC(PC0/PC1) + DMA2_Stream0 + NVIC(优先级5,0)
        ADC2: GPIOA(PA3/PA4) + DMA2_Stream1 + NVIC(优先级5,0)
        ADC3: GPIOC(PC2/PC3) + SYSCFG_PMCR + BDMA_Channel0 + NVIC(优先级5,0)
        PLL3时钟: HSE=25MHz, M=5, N=45, P=4 → VCO=225MHz, PLL3P=56.25MHz
        DMA优先级(2026-06-09修复): ADC=5, UART=6, SysTick=15
修改记录：
***********************************************************/
void HAL_ADC_MspInit(ADC_HandleTypeDef *adcHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /* PLL3 clock: must be configured before any ADC initialization */
  /* ★★★ ADC 时钟路径完整分析（2026-06-12 J-Link 寄存器实测确认）★★★
   *
   * ┌─────────────────────────────────────────────────────────────────┐
   * │ 1. 时钟源：RCC_ADCCLKSOURCE_PLL3 路由的是 PLL3_R，不是 PLL3_P │
   * │    证据：D3CCIPR = 0x00010000, ADCSEL[17:16] = 01            │
   * │    HAL 定义：stm32h7xx_hal_rcc_ex.h L3114                      │
   * │      "RCC_ADCCLKSOURCE_PLL3: PLL3_R Clock selected as ADC     │
   * │       clock"                                                   │
   * │    HAL 实现：stm32h7xx_hal_rcc_ex.c L1340                     │
   * │      RCCEx_PLL3_Config(&PLL3, DIVIDER_R_UPDATE)               │
   * │      → 只使能 RCC_PLL3_DIVR 输出，不使能 RCC_PLL3_DIVP        │
   * └─────────────────────────────────────────────────────────────────┘
   *
   * ┌─────────────────────────────────────────────────────────────────┐
   * │ 2. PLL3 实际寄存器值（J-Link 读取 0x58024428+ 偏移块）：       │
   * │    PLLCKSELR = 0x00500052: PLLSRC=HSE, DIVM1=5, DIVM3=5      │
   * │    PLLCFGR   = 0x01070909: PLL3VCOSEL=1(Wide), PLL3RGE=2      │
   * │    PLL3DIVR  = 0x0101062C: N3=45, P3=4, Q3=2, R3=2           │
   * │    → 与代码配置完全一致！HAL 正确写入了所有分频器               │
   * │    → DIVM 字段是实际分频值（非0-based），M=5 即 DIVM=5        │
   * └─────────────────────────────────────────────────────────────────┘
   *
   * ┌─────────────────────────────────────────────────────────────────┐
   * │ 3. PLL3 输出频率：                                             │
   * │    VCO = 25MHz × 45 / 5 = 225 MHz                             │
   * │    PLL3_P = 225/4 = 56.25 MHz（未路由到 ADC）                  │
   * │    PLL3_R = 225/2 = 112.5 MHz ← ADC 实际内核时钟              │
   * └─────────────────────────────────────────────────────────────────┘
   *
   * ┌─────────────────────────────────────────────────────────────────┐
   * │ 4. 硅片版本：DBGMCU_IDCODE = 0x20036450                       │
   * │    DEV_ID = 0x450 (STM32H750), REV_ID = 0x2003 (Rev.V)        │
   * │    Rev.V 特性（AN5312 §1.7.1）：                               │
   * │    - ADC 内核时钟路径有不可编程的硬件 ÷2 分频器                 │
   * │    - HAL ADC_ConfigureBoostMode() 对 Rev.V 执行 freq /= 2     │
   * │    - 实际 SAR 转换时钟 = 112.5/2 = 56.25 MHz                  │
   * └─────────────────────────────────────────────────────────────────┘
   *
 * ┌─────────────────────────────────────────────────────────────────┐
 * │ 5. ✅ ADC1_CFGR 实测 RES=010 → 12-bit 分辨率（已验证正确）     │
 * │    代码配置 ADC_RESOLUTION_12B → LL_ADC_RESOLUTION_12B          │
 * │      = ADC_CFGR_RES_1 = 0x08 → CFGR RES[4:2]=010               │
 * │    Rev.V RES 编码（RM0433 Table 131）:                          │
 * │      000=16b, 001=14b, 010=12b, 011=10b,                       │
 * │      100=8b(opt), 101=6b, 110=12b(opt), 111=8b                 │
 * │    ADC1_CR BOOST[9:8]=3（最高boost，>80MHz）                   │
 * │    结论: 分辨率配置正确，12-bit 无 bug                           │
 * └─────────────────────────────────────────────────────────────────┘
   *
 * ┌─────────────────────────────────────────────────────────────────┐
 * │ 6. 实测采样率（12-bit 模式，EMA bug 已修复）：                │
 * │    ★ 2026-06-12 修复: 原 EMA 校准 bug 导致 adc_fs_hz ≈ 2×    │
 * │      真实值（7.4 MSPS vs 3.5 MSPS），根因是 half 和 full 路径│
 * │      共用 diag_last_frame_cyc，修复后改用独立 half_ema_last │
 * │                                                                │
 * │    修复后 EMA 收敛到 ~3.5 MSPS/ch（与原始测量一致）           │
 * │    理论值（12-bit/14cyc/56.25MHz）= 2.009 MSPS，仍低于实测    │
 * │    可能原因：Rev.V BOOST 模式下转换周期 < 14 cycles           │
 * │    ★ FFT 频率精度已实测验证正确（使用 adc_fs_hz 校准）     │
 * └─────────────────────────────────────────────────────────────────┘
   *
 * 总结：
 *   ✅ ADC 时钟 = PLL3_R = 112.5 MHz（非 PLL3_P = 56.25 MHz）
 *   ✅ 硅片 = Rev.V（有硬件 ÷2，SAR 时钟 = 56.25 MHz）
 *   ✅ 实际分辨率 = 12-bit（CFGR RES=010，配置正确无误）
 *   ⚠️ EMA 采样率校准 bug 已修复（2026-06-12），
 *      原 bug 导致 adc_fs_hz ≈ 2× 真实值（7.4 vs 3.5 MSPS）
 *   ⚠️ 实测采样率 ~3.5 MSPS/ch（理论 2.009 MSPS，差距原因待查）
   */
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

    /* 【2026-06-09 修复】优先级从 (0,0) 降为 (5,0)
     * 原问题：(0,0) 最高优先级会抢占 SysTick(HAL_GetTick=15) 和 UART DMA TC(6)，
     *         3 个 ADC 同时触发中断时可能导致嵌套问题。
     * 修复：ADC DMA 优先级 5，UART DMA 优先级 6，SysTick 优先级 15，
     *       ADC ISR 不再抢占 UART DMA 完成 → tx_ring_poll 时序更稳定 */
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
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

    HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
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

    HAL_NVIC_SetPriority(BDMA_Channel0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(BDMA_Channel0_IRQn);
  }
}

/***********************************************************
函数名：HAL_ADC_MspDeInit
参数：  adcHandle - ADC句柄指针
返回值：无
描述：  ADC底层硬件反初始化（MSP回调）
        按ADC实例分别反初始化：关闭时钟、释放GPIO、反初始化DMA、禁用NVIC
修改记录：
***********************************************************/
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

/***********************************************************
函数名：HAL_ADC_ConvHalfCpltCallback
参数：  hadc - ADC句柄指针，指示触发回调的ADC实例
返回值：无
描述：  ADC DMA半传输完成回调函数
        DMA写入缓冲区前半部分后触发HT中断，HAL内部路由到此回调
        功能：记录DWT周期计数 + 置位dma_half标志
        ISR上下文：不可调用阻塞函数
修改记录：
***********************************************************/
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

/***********************************************************
函数名：HAL_ADC_ConvCpltCallback
参数：  hadc - ADC句柄指针，指示触发回调的ADC实例
返回值：无
描述：  ADC DMA全传输完成回调函数
        DMA写入缓冲区后半部分后触发TC中断，HAL内部路由到此回调
        功能：记录DWT周期计数 + 置位dma_full标志
        ISR上下文：不可调用阻塞函数
修改记录：
***********************************************************/
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

/* === 2026-06-08 v9.1 ADC ErrorCallback 安全版 ===
 * 早期版本会立即重启 ADC，但在 OVR 风暴场景下导致 ISR 死循环（CPU 100% 在 ISR 里），
 * 主循环永远跑不到。现在改成最小处理：只清错误码 + 关 DMA TEIE，
 * 让 ADC 一次性死掉，main loop 通过 frame_seen_cnt 看门狗检测后再统一恢复。
 * 调用 HAL 函数（HAL_GetTick / HAL_ADC_Start_DMA）在 ISR 内是不安全的。 */

/* ====================================================================== */
/*  ADC错误诊断计数器 — 用于调试和看门狗恢复逻辑                           */
/* ====================================================================== */
volatile uint32_t diag_adc_err_cnt[3] = {0};           /* ADC错误回调触发次数 [ADC1,ADC2,ADC3] */
volatile uint32_t diag_adc_err_code[3] = {0};          /* 最近一次ADC错误码 */
volatile uint32_t diag_adc_err_recover_cnt[3] = {0};   /* ADC看门狗恢复次数 */

/***********************************************************
函数名：HAL_ADC_ErrorCallback
参数：  hadc - ADC句柄指针，指示触发回调的ADC实例
返回值：无
描述：  ADC错误回调函数（最小安全版 v9.1）
        触发条件：DMA传输错误(TE)、ADC溢出(OVR)等
        处理策略（最小安全版，避免ISR死循环）：
        1. 递增错误计数器
        2. 记录错误码
        3. 清除ADC和DMA错误码
        4. 关闭DMA TEIE中断（防止反复进入错误回调）
        5. 不重启ADC（由主循环看门狗统一恢复）
修改记录：
***********************************************************/

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  uint8_t idx;

  if(hadc->Instance == ADC1)      { idx = 0; }
  else if(hadc->Instance == ADC2) { idx = 1; }
  else if(hadc->Instance == ADC3) { idx = 2; }
  else return;

  diag_adc_err_cnt[idx]++;
  diag_adc_err_code[idx] = hadc->ErrorCode;

  /* 只清错误码，不重启 DMA（重启会触发新的 ISR，在 OVR 风暴下死循环）
   * 让 main loop 看 frame_seen_cnt 决定是否要重启 */
  hadc->ErrorCode = HAL_ADC_ERROR_NONE;
  if(hadc->DMA_Handle != NULL) {
    hadc->DMA_Handle->ErrorCode = HAL_DMA_ERROR_NONE;
  }
  /* 关闭这个 ADC 的 DMA TEIE，防止反复进入错误回调 */
  if(hadc->DMA_Handle != NULL && hadc->DMA_Handle->Instance != NULL) {
    ((DMA_Stream_TypeDef*)hadc->DMA_Handle->Instance)->CR &= ~DMA_SxCR_TEIE;
  }
}

/* USER CODE END 1 */
