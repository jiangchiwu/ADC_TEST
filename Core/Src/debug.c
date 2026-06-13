/**
  ******************************************************************************
  * File Name          : DEBUG.c
  * Description        : 非阻塞 debug 输出（环形缓冲 + 后台 flush）
  *                      关键：debug_printf 把数据塞入环形缓冲后立即返回，
  *                      实际 UART 发送由 debug_poll() 在主循环空闲时间分块完成，
  *                      避免阻塞主循环影响 ADC DMA 帧的及时消费。
  ******************************************************************************
  */

/* DEBUG_PRINTF_INTERNAL：在 include debug.h 之前定义，让本文件的 debug_printf
 * 函数定义不被 debug.h 中的宏替换覆盖（外部使用方仍走宏替换/空操作） */
#define DEBUG_PRINTF_INTERNAL

/* Includes ------------------------------------------------------------------*/
#include "debug.h"
#include "usart.h"

/* USER CODE BEGIN 0 */

#include <string.h>

/* === 大容量环形缓冲（必须足够装下事件突发字节流） === */
#define DBG_RING_SIZE   (16 * 1024)   /* 16 KB ≈ 360 ms @ 460800 baud */
static volatile uint8_t  dbg_ring[DBG_RING_SIZE];
static volatile uint16_t dbg_head = 0;   /* 写入位置 */
static volatile uint16_t dbg_tail = 0;   /* 读取位置 */
static volatile uint32_t dbg_drop_bytes = 0;

/* 单次格式化的临时缓冲 */
static char debug_buffer[512];

/* USER CODE END 0 */

/* DEBUG init function */
void MX_DEBUG_Init(void)
{
  /* UART7 已经在 usart.c 中初始化 */
}

/* USER CODE BEGIN 1 */

void debug_init(void)
{
  /* UART7 已经在 usart.c 中初始化为 2000000 (2M) 8N1 */
  dbg_head = 0;
  dbg_tail = 0;
  dbg_drop_bytes = 0;
  /* 【2026-06-12 修复】清零 dbg_ring，避免 AXI_SRAM 残留数据被 DMA 发送 */
  memset((void*)dbg_ring, 0, DBG_RING_SIZE);
}

/* 非阻塞：vsnprintf 后塞入环形缓冲，立即返回 */
void debug_printf(const char *fmt, ...)
{
  va_list va;
  int len;
  uint16_t i;
  uint16_t head;

  va_start(va, fmt);
  len = vsnprintf(debug_buffer, sizeof(debug_buffer), fmt, va);
  va_end(va);

  if(len <= 0) return;
  if(len > (int)sizeof(debug_buffer)) len = sizeof(debug_buffer);

  head = dbg_head;
  for(i = 0; i < (uint16_t)len; i++) {
    uint16_t next = (uint16_t)((head + 1) % DBG_RING_SIZE);
    if(next == dbg_tail) {
      /* 缓冲满：丢弃多余字节，避免阻塞 */
      dbg_drop_bytes += (uint32_t)(len - i);
      break;
    }
    dbg_ring[head] = (uint8_t)debug_buffer[i];
    head = next;
  }
  dbg_head = head;
}

/* 非阻塞二进制 push：用于事件帧（19B）/打包帧（114B）等任意字节流
 * 性能：把整段 memcpy 到环形缓冲，耗时仅 ~1us，不阻塞主循环。
 * 缓冲满时丢弃剩余字节并累计 dbg_drop_bytes（可通过 debug_get_drop_bytes 观测）。
 *
 * 注意：此函数不是中断安全（与 debug_printf 共享环形缓冲），
 *       但 main 单线程调用，没有问题。 */
void debug_push_raw(const uint8_t *buf, uint16_t len)
{
  uint16_t i;
  uint16_t head = dbg_head;
  for(i = 0; i < len; i++) {
    uint16_t next = (uint16_t)((head + 1) % DBG_RING_SIZE);
    if(next == dbg_tail) {
      dbg_drop_bytes += (uint32_t)(len - i);
      break;
    }
    dbg_ring[head] = buf[i];
    head = next;
  }
  dbg_head = head;
}

/* 主循环周期性调用：把环形缓冲数据非阻塞地分块写入 UART
 * 【2026-06-12 优化】改用 DMA 发送替代阻塞式 HAL_UART_Transmit(timeout=3ms)
 * 原问题：HAL_UART_Transmit 阻塞最多 3ms，期间主循环无法消费 ADC DMA 帧
 * 新方案：HAL_UART_Transmit_DMA 非阻塞发送，立即返回主循环
 *         下一轮 poll 时检查 UART 状态，BUSY 则跳过等下轮 */
void debug_poll(void)
{
  uint16_t avail;
  uint16_t chunk;
  uint16_t head = dbg_head;
  uint16_t tail = dbg_tail;
  static uint8_t tmp[64];
  uint16_t i;

  if(head == tail) return;

  /* UART BUSY（上轮 DMA 还在发或事件帧在发）→ 跳过本轮 */
  if(HAL_UART_GetState(&hdebug_uart) != HAL_UART_STATE_READY) return;

  if(head > tail) avail = head - tail;
  else            avail = (uint16_t)(DBG_RING_SIZE - tail + head);

  chunk = (avail > 64) ? 64 : avail;
  for(i = 0; i < chunk; i++) {
    tmp[i] = dbg_ring[tail];
    tail = (uint16_t)((tail + 1) % DBG_RING_SIZE);
  }
  /* DMA 发送：非阻塞，64B@460800≈1.4ms 在后台完成 */
  if(HAL_UART_Transmit_DMA(&hdebug_uart, tmp, chunk) == HAL_OK) {
    dbg_tail = tail;
  }
  /* DMA 发送失败（UART 不就绪等）→ 不更新 tail，下轮重试 */
}

uint32_t debug_get_drop_bytes(void)
{
  return dbg_drop_bytes;
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
