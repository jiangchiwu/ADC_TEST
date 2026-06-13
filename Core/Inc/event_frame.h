/**
 *****************************************************************************
 * @file    event_frame.h
 * @brief   ADC 事件二进制上送帧格式 + 帧组装/校验工具
 *****************************************************************************
 */

#ifndef __EVENT_FRAME_H__
#define __EVENT_FRAME_H__

#include <stdint.h>

#define EVT_FRAME_HEADER_0     0xA5U
#define EVT_FRAME_PAYLOAD_LEN  0x09U
#define EVT_FRAME_TOTAL_LEN    12U

static inline uint8_t evt_sum8(const uint8_t *data, uint8_t len)
{
  uint8_t sum = 0;
  uint8_t i;
  for(i = 0; i < len; i++) {
    sum += data[i];
  }
  return sum;
}

static inline void evt_pack_frame(uint8_t *buf,
                                  uint8_t  ch_num,
                                  uint64_t ts_us,
                                  uint32_t freq_hz)
{
  uint16_t freq16 = (freq_hz > 0xFFFFU) ? 0xFFFFU : (uint16_t)freq_hz;

  buf[0]  = EVT_FRAME_HEADER_0;
  buf[1]  = EVT_FRAME_PAYLOAD_LEN;
  buf[2]  = ch_num;
  buf[3]  = (uint8_t)(ts_us & 0xFFU);
  buf[4]  = (uint8_t)((ts_us >> 8) & 0xFFU);
  buf[5]  = (uint8_t)((ts_us >> 16) & 0xFFU);
  buf[6]  = (uint8_t)((ts_us >> 24) & 0xFFU);
  buf[7]  = (uint8_t)((ts_us >> 32) & 0xFFU);
  buf[8]  = (uint8_t)((ts_us >> 40) & 0xFFU);
  buf[9]  = (uint8_t)(freq16 & 0xFFU);
  buf[10] = (uint8_t)((freq16 >> 8) & 0xFFU);
  buf[11] = evt_sum8(&buf[2], 9U);
}

#endif /* __EVENT_FRAME_H__ */
