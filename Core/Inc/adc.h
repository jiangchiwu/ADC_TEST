/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   6-channel ADC driver: ADC1 + ADC2 + ADC3 (2 channels each)
  *          ADC1: PC0(INP10) + PC1(INP11)  → 逻辑通道 CH1/CH2
  *          ADC2: PA3(INP15) + PA4(INP18)  → 逻辑通道 CH5/CH6
  *          ADC3: PC2(INP0)  + PC3(INP1)   → 逻辑通道 CH3/CH4
  *          12-bit resolution, DMA circular mode, ~3.515 MSPS/channel
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

/* ====================================================================== */
/*  ADC句柄 — 每个ADC实例对应一个HAL句柄                                   */
/* ====================================================================== */
extern ADC_HandleTypeDef hadc1;    /* ADC1: PC0(CH1) + PC1(CH2) */
extern ADC_HandleTypeDef hadc2;    /* ADC2: PA3(CH5) + PA4(CH6) */
extern ADC_HandleTypeDef hadc3;    /* ADC3: PC2(CH3) + PC3(CH4) */

/* ====================================================================== */
/*  DMA句柄 — ADC1/ADC2使用DMA2, ADC3使用BDMA                             */
/* ====================================================================== */
extern DMA_HandleTypeDef hdma_adc1;    /* ADC1 DMA: DMA2_Stream0 */
extern DMA_HandleTypeDef hdma_adc2;    /* ADC2 DMA: DMA2_Stream1 */
extern DMA_HandleTypeDef hdma_adc3;    /* ADC3 DMA: BDMA_Channel0 */

/* ====================================================================== */
/*  通道数定义 — 6逻辑通道，3个ADC实例各2通道                             */
/* ====================================================================== */
#define ADC_NCH              6      /* 6 个逻辑通道（保持原有 main.c 兼容） */
#define ADC_CH_PER_INST      2      /* 每个 ADC 2 通道 */
#define ADC_INST_COUNT       3      /* ADC1 + ADC2 + ADC3 */

/* ====================================================================== */
/*  DMA缓冲区大小定义                                                     */
/*  每个ADC缓冲区16384个uint16_t = 32KB                                  */
/*  DMA循环模式: 前半8192=HT中断, 后半8192=TC中断                        */
/*  半区每通道4096样本(ADC_HALF_SCANS), 约1.165ms@3.515MSPS              */
/* ====================================================================== */
#define ADC_HALF_SCANS       4096   /* 每个 ADC 半区每通道样本数 */
#define ADC_DMA_BUF_SIZE     (ADC_HALF_SCANS * ADC_CH_PER_INST * 2)  /* 16384 = full buf per ADC */

/* ====================================================================== */
/*  DMA缓冲区 — 固定地址分配                                              */
/*  ADC1/ADC2在AXI SRAM(0x24070000/0x24078000), ADC3在SRAM4(0x38000000)  */
/* ====================================================================== */
extern uint16_t adc1_buf[ADC_DMA_BUF_SIZE] __attribute__((aligned(32)));
extern uint16_t adc2_buf[ADC_DMA_BUF_SIZE] __attribute__((aligned(32)));
extern uint16_t adc3_buf[ADC_DMA_BUF_SIZE] __attribute__((aligned(32)));

/* ====================================================================== */
/*  DMA标志 — ISR置位, 主循环消费                                         */
/* ====================================================================== */
extern volatile uint8_t adc1_dma_half;    /* ADC1 DMA前半区完成标志 */
extern volatile uint8_t adc1_dma_full;    /* ADC1 DMA后半区完成标志 */
extern volatile uint8_t adc2_dma_half;    /* ADC2 DMA前半区完成标志 */
extern volatile uint8_t adc2_dma_full;    /* ADC2 DMA后半区完成标志 */
extern volatile uint8_t adc3_dma_half;    /* ADC3 DMA前半区完成标志 */
extern volatile uint8_t adc3_dma_full;    /* ADC3 DMA后半区完成标志 */

/* ====================================================================== */
/*  DMA完成时间戳（DWT周期计数）— 用于采样率校准                          */
/* ====================================================================== */
extern volatile uint32_t adc1_dma_half_cyc;    /* ADC1 HT中断DWT周期 */
extern volatile uint32_t adc1_dma_full_cyc;    /* ADC1 TC中断DWT周期 */
extern volatile uint32_t adc2_dma_half_cyc;    /* ADC2 HT中断DWT周期 */
extern volatile uint32_t adc2_dma_full_cyc;    /* ADC2 TC中断DWT周期 */
extern volatile uint32_t adc3_dma_half_cyc;    /* ADC3 HT中断DWT周期 */
extern volatile uint32_t adc3_dma_full_cyc;    /* ADC3 TC中断DWT周期 */

/* ====================================================================== */
/*  采样率 — 主循环EMA在线校准                                             */
/* ====================================================================== */
extern float adc_sample_rate;  /* Msps per channel, calibrated online */

/* ====================================================================== */
/*  ADC错误诊断计数器                                                      */
/* ====================================================================== */
extern volatile uint32_t diag_adc_err_cnt[3];           /* ADC错误回调触发次数 */
extern volatile uint32_t diag_adc_err_code[3];          /* 最近一次ADC错误码 */
extern volatile uint32_t diag_adc_err_recover_cnt[3];   /* ADC看门狗恢复次数 */

/* ====================================================================== */
/*  公共函数接口                                                           */
/* ====================================================================== */
void MY_ADC1_Init(void);      /* 初始化ADC1 (PC0+PC1) */
void MY_ADC2_Init(void);      /* 初始化ADC2 (PA3+PA4) */
void MY_ADC3_Init(void);      /* 初始化ADC3 (PC2+PC3) */
void MY_ADC_Start(void);      /* 同步启动3个ADC DMA采集 */
void MY_ADC_Stop(void);       /* 停止3个ADC DMA采集 */

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */
