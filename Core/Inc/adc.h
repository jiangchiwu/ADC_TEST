/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   6-channel ADC driver: ADC1 + ADC2 + ADC3 (2 channels each)
  *          ADC1: PC0(INP10) + PC1(INP11)  → 逻辑通道 CH1/CH2
  *          ADC2: PA3(INP15) + PA4(INP18)  → 逻辑通道 CH5/CH6
  *          ADC3: PC2(INP0)  + PC3(INP1)   → 逻辑通道 CH3/CH4
  *          8-bit resolution, DMA circular mode, ~2.6 MSPS/channel
  *
  *          注：PA3/PA4 在 ADC3 上不可用（仅 ADC1/2），故分配到 ADC2；
  *              PC2/PC3 同时可用于 ADC1/2/3，分配到 ADC3 以分散负载。
  *
  *          逻辑通道映射（保持与原 6 通道方案 兼容）：
  *            CH1 = PC0 (ADC1 rank1)
  *            CH2 = PC1 (ADC1 rank2)
  *            CH3 = PC2 (ADC3 rank1)
  *            CH4 = PC3 (ADC3 rank2)
  *            CH5 = PA3 (ADC2 rank1)
  *            CH6 = PA4 (ADC2 rank2)
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* --------------- ADC handles --------------- */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

/* --------------- DMA handles --------------- */
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern DMA_HandleTypeDef hdma_adc3;

/* --------------- 通道数定义 --------------- */
#define ADC_NCH              6      /* 6 个逻辑通道（保持原有 main.c 兼容） */
#define ADC_CH_PER_INST      2      /* 每个 ADC 2 通道 */
#define ADC_INST_COUNT       3      /* ADC1 + ADC2 + ADC3 */

/* --------------- DMA buffers --------------- */
/* 每个 ADC 2 通道，单通道 ~2.6 MSPS，缓冲区时长 ~3 ms（单半区 ~1.5ms）
 * 每个 ADC 的缓冲区大小：4096 samples/ch * 2 ch = 8192 samples (uint16)
 */
#define ADC_HALF_SCANS       4096   /* 每个 ADC 半区每通道样本数 */
#define ADC_DMA_BUF_SIZE     (ADC_HALF_SCANS * ADC_CH_PER_INST * 2)  /* 16384 = full buf per ADC */

extern uint16_t adc1_buf[ADC_DMA_BUF_SIZE] __attribute__((section(".AXI_SRAM"), aligned(32)));
extern uint16_t adc2_buf[ADC_DMA_BUF_SIZE] __attribute__((section(".AXI_SRAM"), aligned(32)));
extern uint16_t adc3_buf[ADC_DMA_BUF_SIZE] __attribute__((section(".AXI_SRAM"), aligned(32)));

/* --------------- DMA flags (3 个 ADC 各自一组) --------------- */
extern volatile uint8_t adc1_dma_half;
extern volatile uint8_t adc1_dma_full;
extern volatile uint8_t adc2_dma_half;
extern volatile uint8_t adc2_dma_full;
extern volatile uint8_t adc3_dma_half;
extern volatile uint8_t adc3_dma_full;

/* DMA 完成时刻 CYCCNT（用于精确时间戳） */
extern volatile uint32_t adc1_dma_half_cyc;
extern volatile uint32_t adc1_dma_full_cyc;
extern volatile uint32_t adc2_dma_half_cyc;
extern volatile uint32_t adc2_dma_full_cyc;
extern volatile uint32_t adc3_dma_half_cyc;
extern volatile uint32_t adc3_dma_full_cyc;

/* --------------- Sampling rate (per channel) --------------- */
extern float adc_sample_rate;  /* Msps per channel, calibrated online */

/* --------------- Public functions --------------- */
void MY_ADC1_Init(void);
void MY_ADC2_Init(void);
void MY_ADC3_Init(void);
void MY_ADC_Start(void);   /* 同步启动 3 个 ADC */
void MY_ADC_Stop(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */
