/**
 *****************************************************************************
 * @file    event_frame.h
 * @brief   ADC 突发事件二进制上送帧格式 v3 + 帧组装/校验工具
 * @author  F:/ADC_FFT 项目维护
 * @version v3.0
 * @date    2026
 *
 * =============================================================================
 *  设计目标
 * =============================================================================
 *   STM32H750 实时检测 6 通道 ADC（PC0/PC1/PC2/PC3/PA3/PA4）信号从
 *   直流（DC）跳变到正弦突发（AC burst）后，组装一个固定 19 字节的
 *   二进制事件帧，通过 UART7（460800 baud, CH340 USB-Serial）上送
 *   到 PC，PC 端解析该帧获得：
 *     1) 触发通道号（1..6）
 *     2) 精确触发时间戳（μs 计数器, DWT）
 *     3) 通过 FFT 1024 点 计算得到的基波频率（Hz 整数）
 *
 * =============================================================================
 *  帧结构 (固定 19 字节, 小端字节序)
 * =============================================================================
 *
 *   字段        偏移   长度    类型              说明
 *   ──────────────────────────────────────────────────────────────────────
 *   HEADER_0     [0]   1 B    0xAA              帧头第一字节
 *   HEADER_1     [1]   1 B    0x55              帧头第二字节
 *   LEN          [2]   1 B    uint8_t = 14      Payload 长度（不含 Head/Tail）
 *   CH           [3]   1 B    uint8_t           通道号 1..6
 *   TS_US        [4]   8 B    uint64_t LE       触发时间（μs，DWT 计数器值）
 *   FREQ_HZ      [12]  4 B    uint32_t LE       基波频率（Hz 整数）
 *   SUM          [16]  1 B    uint8_t           累加和校验
 *                                               范围 frame[3..15]=13B
 *                                               累加值取低 8 bit
 *   TAIL_0       [17]  1 B    0x0D              帧尾第一字节（CR）
 *   TAIL_1       [18]  1 B    0x0A              帧尾第二字节（LF）
 *
 *   总长度 = 2 + 1 + 1 + 8 + 4 + 1 + 2 = 19 字节
 *
 *   传输时间 @ 460800 baud：
 *     1 字节 = 10 bit (1 起始 + 8 数据 + 1 停止) / 460800 ≈ 21.7 μs
 *     单帧 19 字节 ≈ 412 μs
 *     6 帧 burst（紧接发送）≈ 2.47 ms，对应一个 USB Bulk IN 包
 *
 * =============================================================================
 *  累加和算法 (与 evt_sum8 实现一致)
 * =============================================================================
 *
 *     uint8_t sum = 0;
 *     for(i = 3; i <= 15; i++) sum += frame[i];
 *     // 自动溢出截断，等价于 sum &= 0xFF
 *     frame[16] = sum;
 *
 *   覆盖范围：CH(1B) + TS_US(8B) + FREQ_HZ(4B) = 13 B
 *   不覆盖：  帧头 frame[0..1]、LEN frame[2]、SUM 自身 frame[16]、帧尾 frame[17..18]
 *
 * =============================================================================
 *  关键注意事项
 * =============================================================================
 *   1. 字节序：所有多字节字段使用 小端字节序 (Little-Endian)。
 *      x86/AMD64 PC 与 ARM Cortex-M7 均为小端，memcpy 直接复制即可。
 *
 *   2. UART 独占：在事件帧发送期间，严禁通过同一 UART 输出任何 ASCII 调试
 *      数据，否则 ASCII 字节会插入二进制帧流中，破坏 PC 端解析。
 *      固件中所有 debug_printf / debug_poll 都被
 *      #if !ENABLE_DAC_SIGNAL_SOURCE 包围。
 *
 *   3. 板端发送速率上限：实测 CH340 + Windows + pyserial 稳定接收上限
 *      约 120 evt/s；超过该速率会出现丢帧；通过提高 EVENT_COOLDOWN_MS
 *      可降低板端发送速率。
 *
 *   4. PC 端读取策略：建议用 ser.in_waiting + ser.read(n_avail) 模式
 *      （每 1~5 ms 轮询），避免 ser.read(N) 阻塞模式触发 OVERLAPPED
 *      I/O 错误（CH340 驱动 Bug）。
 *
 *   5. 时间戳来源：STM32 端使用 DWT 计数器（基于 SystemCoreClock=480MHz），
 *      DWT_GetUs() 返回值除以 480 得到 μs。
 *
 *   6. FREQ_HZ = 0 表示 FFT 未检出有效频率（信号 SNR 不足）。
 *
 * =============================================================================
 *  实测帧示例 (Hex Dump)
 * =============================================================================
 *
 *   CH=4, ts=9123456us=0x008B3680, freq=40000Hz=0x9C40:
 *
 *     AA 55 0E 04  80 36 8B 00 00 00 00 00  40 9C 00 00  9E  0D 0A
 *     │──│  │  │   │─────── TS_US 8B ────│  │─FREQ 4B─│  │   │─Tail│
 *     Head LEN CH                                        SUM
 *
 *   累加和验证：
 *     0x04 + 0x80 + 0x36 + 0x8B + 0x00 + 0x00 + 0x00 + 0x00 +
 *     0x00 + 0x40 + 0x9C + 0x00 + 0x00 = 0x39E
 *     低 8 bit = 0x9E ✓
 *
 * =============================================================================
 *  上位机解析工具 (Python)
 * =============================================================================
 *   F:/ADC_FFT/test_scripts/verify_event_system.py
 *   F:/ADC_FFT/test_scripts/dist/verify_event_system.exe (PyInstaller 打包)
 *****************************************************************************
 */

#ifndef __EVENT_FRAME_H__
#define __EVENT_FRAME_H__

#include <stdint.h>
#include <string.h>

/* ====================================================================== */
/*  帧格式常量 (与 verify_event_system.py 完全对应，禁止单方面修改)        */
/* ====================================================================== */
#define EVT_FRAME_HEADER_0     0xAA     /**< 帧头第一字节                  */
#define EVT_FRAME_HEADER_1     0x55     /**< 帧头第二字节                  */
#define EVT_FRAME_TAIL_0       0x0D     /**< 帧尾第一字节 (CR)             */
#define EVT_FRAME_TAIL_1       0x0A     /**< 帧尾第二字节 (LF)             */
#define EVT_FRAME_PAYLOAD_LEN  14       /**< Payload: CH(1)+TS(8)+FREQ(4)+SUM(1) */
#define EVT_FRAME_TOTAL_LEN    19       /**< 总长度: HEAD(2)+LEN(1)+PAYLOAD(14)+TAIL(2) */

/* ====================================================================== */
/*  内联函数实现                                                          */
/* ====================================================================== */

/**
 * @brief   计算 8 位累加和（自动溢出截断）
 *
 * @param[in]  data  待校验数据指针
 * @param[in]  len   待校验数据长度（事件帧固定为 13）
 * @retval     uint8_t  低 8 位累加和
 *
 * @details
 *   对 frame[3..15] 范围（CH+TS+FREQ）做逐字节累加，
 *   uint8_t 自动溢出截断等同于取低 8 bit。
 *   该算法简单高效，仅用于检测传输错误，不是密码学校验。
 *
 * @note    不含帧头(frame[0..1])、LEN(frame[2])、SUM 自身(frame[16])、帧尾(frame[17..18])。
 */
static inline uint8_t evt_sum8(const uint8_t *data, uint8_t len)
{
  uint8_t sum = 0;
  uint8_t i;
  for(i = 0; i < len; i++)
  {
    sum += data[i];   /* uint8 溢出自动截断 = sum & 0xFF */
  }
  return sum;
}

/**
 * @brief   组装一个完整的 19 字节事件帧到指定 buf
 *
 * @param[out] buf       输出 buffer 指针，必须 ≥ EVT_FRAME_TOTAL_LEN 字节
 * @param[in]  ch_num    通道号 1..6（不做范围检查，调用方保证）
 * @param[in]  ts_us     触发时间戳（μs，DWT 计数器值除以 SystemCoreClock/1e6 得到）
 * @param[in]  freq_hz   基波频率（Hz 整数，0 表示 FFT 未检出有效频率）
 *
 * @details
 *   组装顺序（与帧结构表完全一致）：
 *     HEAD(2B) -> LEN(1B) -> CH(1B) -> TS_US(8B) -> FREQ_HZ(4B) -> SUM(1B) -> TAIL(2B)
 *
 *   多字节字段使用小端字节序（memcpy 直接复制即可，因为 ARM Cortex-M7 为小端架构）。
 *
 *   累加和计算范围 frame[3..15]，覆盖 CH+TS_US+FREQ_HZ 共 13 字节。
 *
 * @note
 *   1. 调用前需确保 buf 指向 ≥ 19 字节的可写区域。
 *   2. 本函数本身不调用 UART 发送，仅完成内存组装；
 *      调用方需自行 HAL_UART_Transmit。
 *   3. 性能：在 480 MHz Cortex-M7 上执行 < 0.5 μs。
 *
 * @par     使用示例
 * @code
 *   #include "event_frame.h"
 *   uint8_t frame[EVT_FRAME_TOTAL_LEN];
 *   uint64_t ts_us  = DWT_GetUs();
 *   uint32_t freq   = 40000U;
 *   evt_pack_frame(frame, 4U, ts_us, freq);
 *   HAL_UART_Transmit(&hdebug_uart, frame, EVT_FRAME_TOTAL_LEN, 100);
 * @endcode
 *
 * @par     批量发送示例 (6 通道打包 1 个 USB Bulk 包)
 * @code
 *   uint8_t multi_buf[EVT_FRAME_TOTAL_LEN * 6];  // 114 字节
 *   uint16_t off = 0;
 *   for(uint8_t ch = 0; ch < 6; ch++) {
 *     if(triggered_mask & (1U << ch)) {
 *       evt_pack_frame(&multi_buf[off], ch + 1, ts_us[ch], freq[ch]);
 *       off += EVT_FRAME_TOTAL_LEN;
 *     }
 *   }
 *   if(off > 0) HAL_UART_Transmit(&hdebug_uart, multi_buf, off, 100);
 * @endcode
 */
static inline void evt_pack_frame(uint8_t *buf,
                                  uint8_t  ch_num,
                                  uint64_t ts_us,
                                  uint32_t freq_hz)
{
  buf[0]  = EVT_FRAME_HEADER_0;          /* [0]   = 0xAA           */
  buf[1]  = EVT_FRAME_HEADER_1;          /* [1]   = 0x55           */
  buf[2]  = EVT_FRAME_PAYLOAD_LEN;       /* [2]   = 14 (固定)      */
  buf[3]  = ch_num;                      /* [3]   通道号 1..6      */
  memcpy(&buf[4],  &ts_us,   8);         /* [4..11]  小端 uint64   */
  memcpy(&buf[12], &freq_hz, 4);         /* [12..15] 小端 uint32   */
  buf[16] = evt_sum8(&buf[3], 13);       /* [16]  累加和 CH+TS+FREQ */
  buf[17] = EVT_FRAME_TAIL_0;            /* [17]  = 0x0D (CR)      */
  buf[18] = EVT_FRAME_TAIL_1;            /* [18]  = 0x0A (LF)      */
}

#endif /* __EVENT_FRAME_H__ */
