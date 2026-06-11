/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dac.h
  * @brief   DAC sine wave + DC generator on PA5 (DAC1_CH2)
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __DAC_H__
#define __DAC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

extern DAC_HandleTypeDef hdac1;
extern DMA_HandleTypeDef hdma_dac1_ch1;
extern DMA_HandleTypeDef hdma_dac1_ch2;
extern TIM_HandleTypeDef htim7;
extern volatile uint32_t dac_diag_actual_freq_hz;

void MX_DAC1_Init(void);
void MX_TIM7_Init(void);

/* DAC控制接口 */
void MY_DAC_Start(void);
void MY_DAC_Stop(void);

/* 信号模式切换 */
void DAC_Output_DC(void);              /* 输出直流 1.5V */
void DAC_Output_Sine(uint32_t freq_hz);  /* 输出指定频率正弦波 */
void DAC_Output_PWM(uint32_t freq_hz);   /* 输出指定频率方波 */

/* 振幅控制 - 在 DAC_Output_Sine 之前调用 */
void DAC_Set_Sine_Amplitude_mV(uint16_t amp_mv);  /* 设置正弦振幅 50~1500 mV */

/* v9: 合成波形 - 基波+谐波+噪声叠加，用于压力测试 */
void DAC_Build_Composite_Waveform(uint16_t amp_fund_mv,
                                  uint8_t h2_pct, uint8_t h3_pct,
                                  uint8_t noise_pct);

/* 频率设置 */
void DAC_Set_CH1_Freq(uint32_t sine_freq_hz);  /* 占位 - CH1不可用 */
void DAC_Set_CH2_Freq(uint32_t sine_freq_hz);  /* PA5 (CH2) 频率设置 */

#ifdef __cplusplus
}
#endif

#endif /* __DAC_H__ */
