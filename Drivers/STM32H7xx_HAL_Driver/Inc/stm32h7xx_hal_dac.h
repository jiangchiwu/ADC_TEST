#ifndef __STM32H7xx_HAL_DAC_H
#define __STM32H7xx_HAL_DAC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal_def.h"

#define DAC_CHANNEL_1             0x00000000U
#define DAC_CHANNEL_2             0x00000010U
#define DAC_ALIGN_12B_R           0x00000000U

typedef struct {
  DAC_TypeDef* Instance;
  DMA_HandleTypeDef* DMA_Handle1;
  DMA_HandleTypeDef* DMA_Handle2;
  HAL_LockTypeDef Lock;
  uint32_t State;
} DAC_HandleTypeDef;

HAL_StatusTypeDef HAL_DAC_Init(DAC_HandleTypeDef* hdac);
void HAL_DAC_MspInit(DAC_HandleTypeDef* hdac);
void HAL_DAC_MspDeInit(DAC_HandleTypeDef* hdac);
HAL_StatusTypeDef HAL_DAC_Start(DAC_HandleTypeDef* hdac, uint32_t Channel);
HAL_StatusTypeDef HAL_DAC_Stop(DAC_HandleTypeDef* hdac, uint32_t Channel);
HAL_StatusTypeDef HAL_DAC_SetValue(DAC_HandleTypeDef* hdac, uint32_t Channel, uint32_t Alignment, uint32_t Data);
HAL_StatusTypeDef HAL_DAC_Start_DMA(DAC_HandleTypeDef* hdac, uint32_t Channel, uint32_t* pData, uint32_t Length, uint32_t Alignment);
HAL_StatusTypeDef HAL_DAC_Stop_DMA(DAC_HandleTypeDef* hdac, uint32_t Channel);

#ifdef __cplusplus
}
#endif

#endif