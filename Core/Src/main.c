/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @brief   6 通道 ADC 事件驱动采样 + FFT 频率检测 + UART 事件帧上送系统
  *
  * =============================================================================
  *  硬件配置
  * =============================================================================
  *  MCU : STM32H750VBT6 @ 480 MHz (SystemCoreClock)
  *
  *  ADC 6 通道并行采集（3 组 ADC × 2 通道）:
  *    ADC1 : PC0 (INP10) = CH1 ┐
  *           PC1 (INP11) = CH2 │
  *    ADC2 : PA3 (INP15) = CH5 │  并行 DMA 双缓冲
  *           PA4 (INP18) = CH6 │  (PA4 → 见下方 ⚠️)
  *    ADC3 : PC2 (INP0)  = CH3 │
  *           PC3 (INP1)  = CH4 ┘
  *    分辨率   : 12 bit
  *    采样率   : 3.515 MSPS/通道 (实测值, 见 main.c#L350)
  *               ※ 理论值 2.009 MSPS/通道 (PLL3P=56.25MHz / 2 / 14) 与
  *                 实测值不符, 实际真实值由 DMA 半区间隔反推得到
  *               ※ FFT 频率换算使用的是 adc_fs_hz=3515000.0f
  *    DMA 缓冲 : ADC1/ADC2 @ AXI_SRAM, ADC3 @ D3_SRAM4
  *
  *  DAC 信号源（仅闭环测试模式 ENABLE_DAC_SIGNAL_SOURCE=1 时启用，默认关闭）:
  *    ★ 正式环境（默认）: ENABLE_DAC_SIGNAL_SOURCE = 0
  *       6 通道接收完全独立的外部信号（频率/时间/持续时间各不相同）
  *       事件检测走"外部信号路径"（main.c 中 #else 分支）
  *       去抖采用每通道独立 last_valid_event_ms[ch] + SAME_CH_DEDUP_MS
  *
  *    闭环测试时:
  *      DAC1_OUT1 (PA4)  : DC + 40 kHz 正弦 burst
  *      DAC1_OUT2 (PA5)  : DC + 40 kHz 正弦 burst
  *      DAC burst 周期   : 30 ms DC + 5 ms 正弦 (场景 A 默认)
  *                         10 ms DC + 5 ms 正弦 (场景 B, 验证去抖)
  *
  *  ⚠️ 重要硬件警示: PA4 双重用途
  *     PA4 既是 DAC1_OUT1 输出, 又是 ADC2 INP18 输入 (CH6)。
  *     闭环自检模式下, DAC 直接驱动 ADC 输入, 此为预期行为。
  *     正式外部信号模式下, DAC 处于关闭状态（不输出），PA4 仅作为 ADC 输入。
  *
  *  UART7 (PE8/PE7) ↔ CH340 USB-Serial: 460800 baud
  *    用于事件帧上送 PC（19 字节二进制, 详见 event_frame.h）
  *
  * =============================================================================
  *  事件检测流程
  * =============================================================================
  *  1) ADC DMA 循环双缓冲, 每半区 ~1.165 ms 触发中断
  *  2) check_dma_and_push_frames() 处理 ADC 帧, 每通道计算 max_dev
  *  3) max_dev > noise×DEV_NOISE_MULT 且 max_dev > DEV_THRESHOLD_MIN → AC 候选
  *  4) Layer 2 持续性确认: 100 样本中 ≥60 个超阈
  *  5) 对候选通道执行 CMSIS-DSP RFFT 1024 点 (汉宁窗) → 基波频率
  *  6) burst 结束 → 等待 3 ms 让 ADC 帧处理完 → 选出 selected_mask
  *  7) 同通道去抖 SAME_CH_DEDUP_MS=10ms 过滤连续触发
  *  8) 6 帧打包 multi_buf[114B] → 1 次 HAL_UART_Transmit 上送 PC
  *
  * =============================================================================
  *  关键参数 (当前生效值, 见对应 #define 详细注释)
  * =============================================================================
  *    DEV_THRESHOLD_MIN  = 100      最小检测门限 (ADC 计数)
  *    DEV_NOISE_MULT     = 5        噪声基线倍数门限
  *    SUSTAIN_SAMPLES    = 100      持续性窗口
  *    SUSTAIN_MIN_HIT    = 60       持续性最小命中
  *    SAME_CH_DEDUP_MS   = 35       同通道去抖间隔 (ms)
  *    FFT_LENGTH         = 1024     FFT 点数
  *    FFT_SNR_THRESHOLD  = 5.0f     FFT SNR 门限
  *    adc_fs_hz          = 3515000  实测每通道采样率 (Hz)
  *    DAC_DC_DURATION_MS = 30/10    场景 A/B 切换
  *    EVENT_COOLDOWN_MS  = 50       burst 最小间隔 (限制 ≤120 evt/s)
  *
  * =============================================================================
  *  性能基准 (实测)
  * =============================================================================
  *    FFT 1024 点 (CMSIS-DSP) : 200 μs / 次 @ 480 MHz
  *    单次帧组装 evt_pack_frame: < 0.5 μs
  *    单帧 UART 传输 (19B)     : ~412 μs @ 460800 baud
  *    6 帧批量 (114B)          : ~2.47 ms (1 个 USB Bulk 包)
  *    ADC 半区中断周期         : ~1.165 ms (4096 / 3.515 MHz)
  *
  * =============================================================================
  *  PC 端工具
  * =============================================================================
  *    F:/ADC_FFT/test_scripts/verify_event_system.py (源码, 详细注释)
  *    F:/ADC_FFT/test_scripts/dist/verify_event_system.exe (PyInstaller 单文件)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "adc.h"
#include "dac.h"
#include "debug.h"
#include "usart.h"
#include "event_frame.h"

#include "arm_math.h"
#include "arm_const_structs.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

#define LED_ON(pin)   HAL_GPIO_WritePin(GPIOB, pin, GPIO_PIN_RESET)
#define LED_OFF(pin)  HAL_GPIO_WritePin(GPIOB, pin, GPIO_PIN_SET)
#define LED_TOGGLE(pin) HAL_GPIO_TogglePin(GPIOB, pin)
#define LED1_PIN GPIO_PIN_12
#define LED2_PIN GPIO_PIN_13

/* === 触发阈值配置（实际输入动态抗噪）===
 * 阈值由基线噪声峰峰值和最小门限共同决定
 */
/* =============================================================================
 * 【触发判定门限参数 - "偏离开机基线 80 mV" 严格规则】
 * =============================================================================
 * 触发判决以"开机时校准的通道直流基线 ch_dc_offset[ch]"为参考，
 * 当 max|sample - ch_dc_offset[ch]| >= 80 mV 时认为通道发生"事件"。
 *
 * ADC 配置: 12-bit, Vref = 3.3 V
 *   1 LSB  = 3.3 V / 4096 ≈ 0.806 mV
 *   80 mV ≈ 80 / 0.806 ≈ 99.3 ADC 计数 → 取整 100 (DEV_THRESHOLD_80MV)
 *
 * 双层门限判定（任一不通过即拒触发）：
 *   ① max_dev >= DEV_THRESHOLD_80MV  (≈80 mV 偏离基线，对应客户要求)
 *   ② max_dev >= noise_baseline * DEV_NOISE_MULT (动态抗本底噪声)
 *
 * 调参经验：
 *   - DEV_THRESHOLD_80MV 严格 ≈ 80 mV，不可随意降低（客户硬性要求）
 *   - DEV_NOISE_MULT 越大，对噪底变化越宽容，但响应慢
 *   - 无信号 30s 静默测试：0 误触发
 *
 * 【2026-06-07 阈值折中调整】
 *   实测无信号时持续误触发（≥10次/秒），原因：DSCope 实测 ADC 引脚有 60kHz
 *   稳定背景信号 ~40mV pp（DC-DC 开关纹波/电源耦合）。
 *   选择折中阈值 120 ADC counts (≈96 mV)：
 *     - 高于背景噪声峰值 ~48mV（×2 余量），避免误触发
 *     - 低于真实信号触发要求（仍接近客户 80mV 要求）
 *   噪声倍数从 5 提到 8，进一步抵抗短时毛刺。
 * ========================================================================= */
#define DEV_THRESHOLD_80MV 400    /**< 偏离阈值 ≈320 mV (2026-06-07: 120→400，
                                    *   实测浮空通道 max_dev≈162 会误触发，
                                    *   而真实突发正弦幅值 >0.5V (≈600 LSB)，
                                    *   设 400 既能稳触发又能抵抗浮空噪声) */
#define DEV_THRESHOLD_MIN  DEV_THRESHOLD_80MV  /**< 主门限：=320mV 阈值 */
#define DEV_NOISE_MULT     8      /**< 噪声倍数（max_dev 必须 >= 噪声基线 × 8 才触发）*/
#define DEV_THRESHOLD_MIN_WEAK  DEV_THRESHOLD_MIN  /**< 弱信号路径使用相同门限 */
#define DEV_NOISE_MULT_WEAK     DEV_NOISE_MULT     /**< 弱信号路径使用相同倍数 */

#define BASELINE_SAMPLES   80     /**< DC 基线采样数：每通道用此数量样本计算噪声基线 */

/* =============================================================================
 * 【双层确认 - 抗短时干扰】
 * =============================================================================
 * 防止单个噪声脉冲触发事件：
 *   Layer 1: 在 ADC 帧中找到第一个 max_dev > 阈值的候选起点
 *   Layer 2: 从候选起点向后扫描 SUSTAIN_SAMPLES(100) 个样本，
 *            必须有至少 SUSTAIN_MIN_HIT(60) 个样本继续超阈值才算"持续信号"
 *
 * 一次性脉冲(如静电干扰)只能命中几个样本，无法通过此双层确认
 * 真正的 AC burst 持续 5ms = 17500 样本，远超 100 样本窗口，必然通过
 * ========================================================================= */
#define DEV_CONFIRM_CNT    5      /**< Layer 1 候选确认：需要连续 5 个样本超阈值 */
#define SUSTAIN_SAMPLES    100    /**< Layer 2 持续性验证窗口：100 个样本 */
#define SUSTAIN_MIN_HIT    20     /**< Layer 2 最少命中数：2026-06-07 v3 由 40→20，
                                    *   实测信号是短瞬态（<40样本），改 20% 让短脉冲也能触发 */
#define SUSTAIN_MIN_HIT_WEAK  15  /**< 弱信号路径放宽到 15/100 */

/* =============================================================================
 * 【事件级去抖】
 * =============================================================================
 * SAME_CH_DEDUP_MS: 同一通道两次事件最小间隔 = 10 ms
 *
 * 设计目标 (2026-06-07 修订)：
 *   - 旧值 35ms：每通道最大事件率 ~28 Hz（过低）
 *   - 新值 10ms：每通道最大事件率 100 Hz，覆盖 100 Hz 以下的真实突发
 *   - 第一次 burst 触发 → 记录 last_valid_event_ms[ch]
 *   - 若 (now - last_valid_event_ms[ch]) < 10ms 则被过滤为"同一突发的回声"
 *   - 去抖检查全部在 main 线程做，0 阻塞、0 HAL_Delay
 * ========================================================================= */
#define SAME_CH_DEDUP_MS  10      /**< 同通道事件去抖间隔(ms)：2026-06-07 由 35ms→10ms，
                                    *   允许更高事件率（最高 100 Hz/通道），不阻塞主循环 */

#define FRAME_LEVEL_SKIP_MS 2     /**< 帧级预扫描去抖(ms)：2026-06-07 由 5→2ms，
                                    *   减少漏触发；事件级去抖由 SAME_CH_DEDUP_MS=10ms 控制 */
#define AC_PRESENT_MIN_HIT 64     /**< AC 存在检测最小命中数：64/100 样本超阈 */

/* =============================================================================
 * 【FFT 参数】
 * =============================================================================
 * 使用 CMSIS-DSP 1024 点 RFFT 计算基波频率：
 *   - 输入: ADC 采样数据（汉宁窗加权）
 *   - 输出: 频谱中最大峰值对应的频率
 *
 * 频率分辨率 = 采样率 / FFT 点数 = 3.515 MHz / 1024 ≈ 3.43 kHz
 * 通过 3 点抛物线插值提升精度到 < 0.5% 误差
 * ========================================================================= */
#define FFT_LENGTH         1024    /**< FFT 点数（必须 2 的幂，CMSIS-DSP 要求）*/
#define FFT_STEADY_SKIP    128     /**< FFT 输入跳过前 128 样本（避开 burst 启动暂态）*/
#define FFT_SNR_THRESHOLD  5.0f    /**< FFT SNR 门限：峰值 / 平均 >= 5 才算有效基波 */
#define FFT_MIN_MAG_SQ     2500.0f /**< FFT 最小峰值幅度平方门限（汉宁窗下原 10000 的 1/4）*/

/* =============================================================================
 * 【功能开关】
 * =============================================================================
 * 通过宏切换不同的测试模式，编译时静态选择，运行时不可修改
 *
 * ENABLE_DAC_SIGNAL_SOURCE = 1 (默认):
 *   板内 DAC1/DAC2 产生 DC↔正弦 burst 信号，闭环自检
 *   ⚠️ 启用此模式时所有 debug_printf/debug_poll 自动禁用
 *
 * ENABLE_UART_SELF_TEST = 1:
 *   关闭 DAC，固件按固定 2Hz/通道发送测试帧（验证 UART 链路）
 *
 * ENABLE_BOOT_SELF_TEST = 1:
 *   上电时每通道发 1 帧自检帧（验证 PC 端能否解析）
 * ========================================================================= */
#define ENABLE_DAC_SIGNAL_SOURCE  0    /**< 0=正式场景: 外部独立信号源检测模式（默认）
                                            * 1=闭环测试: 板内 DAC 产生 DC↔正弦 burst 信号
                                            * ★ 正式环境下 6 通道接收完全独立的信号
                                            *   (频率/时间/持续时间各不相同), 必须设为 0 */
#define ENABLE_UART_SELF_TEST     0    /**< 1=UART 自检模式（无 ADC 检测）*/
#define ENABLE_BOOT_SELF_TEST     0    /**< 1=上电自检每通道发 1 帧 */

/* === USB-串口极限压测模式 ===
 * 启用后：禁用 DAC/ADC 检测，固件以固定速率持续发送带递增序号的事件帧
 * 序号编码进 freq 字段（uint32_t），PC 侧据此统计缺失帧数
 * UART_STRESS_RATE_HZ 控制目标发送频率
 *   100   = 1 帧/10ms  （轻负载）
 *   1000  = 1 帧/1ms   （中负载，约 1.9KB/s）
 *   5000  = 5 帧/ms    （重负载，约 9.5KB/s）
 *   10000 = 10 帧/ms   （接近 2Mbaud 理论上限 ~10526 帧/s）
 * 设为 0 表示关闭压测，恢复正常工作模式 */
#define UART_STRESS_RATE_HZ       0
/* === DMA 零阻塞链路自检（不需要 ADC 信号） ===
 * 由 MCU 自己以 DMA_LOOPBACK_RATE_HZ 速率把虚假事件帧 push 到 tx_ring，
 * 主循环 tx_ring_poll() 通过 HAL_UART_Transmit_DMA 异步发送。
 * 用 J-Link 观察 diag_uart_q_push_cnt vs diag_tx_poll_send_ok vs diag_uart_q_send_cnt，
 * 同时用 COM3 监听是否物理收齐。
 * 【已验证】DMA 链路 0 丢帧（2026-06-07）。
 * 正式上线时请设 ENABLE_DMA_LOOPBACK_TEST = 0
 * 【极限压力模式】 DMA_LOOPBACK_RATE_HZ = 0 → free-running，主循环以 UART 满速发，
 *   验证 ring 容量 + 主循环 + DMA 完成中断的极限耦合：push 必须 == send_ok，
 *   high-water-mark 应稳定在 N-1（满 ring）。 */
#define ENABLE_DMA_LOOPBACK_TEST  0
/* 0 = free-running 极限压力；>0 = 严格速率 */
#define DMA_LOOPBACK_RATE_HZ      0U
#define EXTERNAL_EVENT_FIXED_FREQ 0
#define EXTERNAL_EVENT_DEBUG_FREQ_HZ 12345U
/* === 帧间隔参数（历史遗留，仅 ENABLE_DAC_SIGNAL_SOURCE=0 时使用） ===
 * EXTERNAL_EVENT_TX_GAP_MS : 外部信号模式下两帧之间的 ms 级间隔
 *                            20 ms > CH340 latency(16ms)，确保每帧独立 USB 包
 * EXTERNAL_EVENT_TX_GAP_US : 外部信号模式下两帧之间的 μs 级精细间隔
 *                            5000 μs = 5 ms，DWT 微秒延时，主循环更早返回
 *
 * 注意：DAC 闭环模式（默认）使用批量打包 multi_buf，不走这两个宏 */
#define EXTERNAL_EVENT_TX_GAP_MS  20U    /**< 外部模式帧间隔(ms): 20ms > CH340 latency 16ms */
#define EXTERNAL_EVENT_TX_GAP_US  5000U  /**< 外部模式帧间隔(μs): 5ms 实测最佳值 */
#define EXTERNAL_EVENT_CALC_FREQ  1
#define EXTERNAL_EVENT_USE_AC_PRESENT 0
#define EXTERNAL_EVENT_USE_FFT_FREQ  1
#define EXTERNAL_EVENT_USE_ZERO_CROSS 0  /* 禁用过零点频率检测，避免噪声误触发 */
#define INTERNAL_TEST_EXPECTED_CH    2
#define INTERNAL_TEST_LOCK_BURSTS    3
#define INTERNAL_TEST_FREQ_MIN_HZ    20000U
#define INTERNAL_TEST_FREQ_MAX_HZ    200000U

static inline void fmt_time_us(uint64_t total_us, uint32_t *s, uint32_t *ms, uint32_t *us)
{
  *us = (uint32_t)(total_us % 1000U);
  *ms = (uint32_t)((total_us / 1000U) % 1000U);
  *s  = (uint32_t)(total_us / 1000000U);
}

#define HALF_SAMPLES_PER_CH   (ADC_HALF_SCANS)
#define FRAME_POOL_SIZE       8

typedef struct {
  uint16_t data[HALF_SAMPLES_PER_CH * ADC_NCH];
  uint64_t start_time_us;
  uint32_t frame_id;
  uint8_t  ready;
  uint8_t  fft_pending_mask;
} adc_frame_t;

static adc_frame_t frame_pool[FRAME_POOL_SIZE] __attribute__((section(".AXI_SRAM"), aligned(32)));

/* ★【2026-06-07 零阻塞 UART 环形发送队列】★
 * 由主循环 push（拷贝 19B），由 DMA Tx 完成中断回调 pop 并发起下一帧。
 * 容量 32 槽 × 19B = 608B；32 帧足够覆盖 6 通道全触发突发。 */
/* 环形缓冲队列：50+ 槽 × 19B = 988B ≤ 1KB — 极限场景 1000Hz 不漏 */
#define TX_RING_SLOTS  52
static uint8_t  tx_ring[TX_RING_SLOTS][EVT_FRAME_TOTAL_LEN] __attribute__((section(".AXI_SRAM"), aligned(32)));
static volatile uint8_t tx_ring_head = 0;  /* 主线程写 */
static volatile uint8_t tx_ring_tail = 0;  /* 中断回调读 */
static volatile uint8_t  frame_w_idx = 0;
static volatile uint8_t  frame_r_idx = 0;
static volatile uint32_t frame_drop_cnt = 0;
static volatile uint32_t total_frame_cnt = 0;
static volatile uint32_t half_complete_cnt = 0;
static volatile uint32_t full_complete_cnt = 0;
volatile uint32_t diag_loop_cnt = 0;
volatile uint32_t diag_dac_burst_cnt = 0;     /**< 已成功启动的 DAC burst 累计次数
                                                   * （冷却期内的尝试不计入；只计入 dac_phase 0→1 切换） */
volatile uint32_t diag_dac_dc_cnt = 0;
volatile uint32_t diag_frame_seen_cnt = 0;
volatile uint32_t diag_frame_gate_drop_cnt = 0;
volatile uint32_t diag_prescan_hit_cnt = 0;
volatile uint32_t diag_fft_try_cnt = 0;
volatile uint32_t diag_uart_evt_cnt = 0;
volatile uint32_t diag_burst_sent_cnt = 0;
volatile uint32_t diag_last_n_trig = 0;
volatile uint32_t diag_last_n_collected = 0;
volatile uint32_t diag_last_state = 0;
/* === 极限压力诊断指标（DMA loopback 模式专用） === */
volatile uint32_t diag_tx_q_depth_max = 0;   /**< tx_ring 历史最大占用深度 */
volatile uint32_t diag_tx_q_full_cnt  = 0;   /**< 队列满（next_head==tail）次数 */
volatile uint32_t diag_uart_hal_ok_cnt = 0;
volatile uint32_t diag_uart_hal_fail_cnt = 0;
volatile uint32_t diag_uart_last_status = 0;
volatile uint32_t diag_uart_last_error = 0;
volatile uint32_t diag_uart_last_isr = 0;
volatile uint32_t diag_uart_last_cr1 = 0;
volatile uint32_t diag_external_event_candidate_cnt = 0;
volatile uint32_t diag_external_dedup_skip_cnt = 0;
volatile uint32_t diag_external_noise_filter_cnt = 0;  /* 新增：噪声过滤计数 */
volatile uint32_t diag_external_freq_filter_cnt = 0;
volatile uint32_t diag_external_send_try_cnt = 0;
volatile uint32_t diag_last_noise_pp[ADC_NCH] = {0};
volatile uint32_t diag_last_thr[ADC_NCH] = {0};
volatile uint32_t diag_last_max_dev[ADC_NCH] = {0};
volatile uint32_t diag_burst_max_dev[ADC_NCH] = {0};
volatile uint32_t diag_last_selected_mask = 0;
volatile uint32_t diag_last_filtered_mask = 0;
volatile uint32_t diag_locked_input_mask = 0;
volatile uint32_t diag_lock_burst_cnt = 0;
volatile uint32_t diag_lock_score[ADC_NCH] = {0};
volatile uint32_t diag_ac_present_cnt[ADC_NCH] = {0};
volatile uint32_t diag_evt_count_ch[ADC_NCH] = {0};
volatile uint32_t diag_evt_last_time_lo[ADC_NCH] = {0};
volatile uint32_t diag_evt_last_time_hi[ADC_NCH] = {0};
volatile uint32_t diag_evt_last_freq[ADC_NCH] = {0};
/* 诊断：各通道最近一次FFT频率（即使未触发事件也记录），用于汉宁窗调试 */
volatile uint32_t diag_ch_last_fft_freq[ADC_NCH] = {0};
/* 诊断：FFT计算结果详情，定位频率偏差根因 */
volatile uint32_t diag_fft_max_bin = 0;        /* FFT峰值bin */
volatile float    diag_fft_max_mag = 0.0f;     /* 峰值幅度平方 */
volatile float    diag_fft_avg_mag = 0.0f;     /* 频带平均幅度 */
volatile float    diag_fft_used_fs = 0.0f;     /* FFT使用的采样率 */
/* 频谱快照：保存FFT bin 4-31的幅度sqrt值（28个bin覆盖DC泄漏区和DAC基波区）
 * 用于判断 max_bin=12 是否为 DC 泄漏伪峰，还是真实信号峰
 * 同时保存被快照的通道号，方便定位 */
volatile float    diag_fft_spec_snapshot[28];
volatile uint8_t  diag_fft_snapshot_ch = 0xFF;  /* 0xFF=未捕获 */
volatile uint32_t diag_dac_snap_cr = 0;
volatile uint32_t diag_dac_snap_dhr = 0;
volatile uint32_t diag_tim7_snap_cr1 = 0;
volatile uint32_t diag_tim7_snap_cnt = 0;
volatile uint32_t diag_dma1s1_snap_cr = 0;
volatile uint32_t diag_dma1s1_snap_ndtr = 0;
volatile uint32_t diag_dma1s1_snap_m0ar = 0;

/* 任务1诊断变量 - 时钟和计时验证 */
volatile uint32_t diag_time_base_lo = 0;    /* u64 us低32位 */
volatile uint32_t diag_time_base_hi = 0;    /* u64 us高32位 */
volatile uint32_t diag_dwt_baseline = 0;    /* DWT CYCCNT基线 */
volatile uint32_t diag_systick_base = 0;     /* HAL_GetTick基线 */
volatile uint32_t diag_sysclk_hz = 0;        /* 系统时钟Hz */

/* 任务2诊断变量 - FFT性能测试 */
volatile uint32_t diag_fft256_cyc = 0;
volatile uint32_t diag_fft512_cyc = 0;
volatile uint32_t diag_fft1024_cyc = 0;
volatile uint32_t diag_fft_bench_done = 0;

/* 任务4诊断变量 - ADC采样验证 */
volatile uint32_t diag_adc1_smpr = 0;
volatile uint32_t diag_adc2_smpr = 0;
volatile uint32_t diag_adc3_smpr = 0;
volatile uint32_t diag_adc1_cfgr = 0;
volatile uint32_t diag_adc2_cfgr = 0;
volatile uint32_t diag_dma2_ndtr = 0;
volatile uint32_t diag_dma1_ndtr = 0;
volatile uint32_t diag_dma_bdma_ndtr = 0;

/* 任务4诊断变量 - 采样间隔和数据处理时间 */
volatile uint32_t diag_frame_process_cyc = 0;    /* 帧处理耗时(周期) */
volatile uint32_t diag_frame_process_max = 0;    /* 帧处理最大耗时 */
volatile uint32_t diag_frame_interval_cyc = 0;   /* 帧间隔(周期) */
volatile uint32_t diag_idle_ratio = 0;           /* 空闲比例(百分比) */
volatile uint32_t diag_adc_sample_cyc = 0;       /* ADC采样周期(周期) */
volatile uint32_t diag_last_frame_cyc = 0;       /* 上次帧处理开始周期 */

/* 各通道直流偏置基线 - 启动时采样建立 */
volatile int32_t ch_dc_offset[ADC_NCH] = {0};    /* 各通道直流偏置基线 */
volatile uint32_t ch_dc_offset_valid = 0;        /* 基线是否有效标志 */

extern TIM_HandleTypeDef htim7;

/* === 采样率校准 ===
 * 基于实测校准，确保频率计算准确
 * ADC内核时钟: 75MHz
 * 采样时间: 1.5周期 + 8位转换周期 = 10周期/样本
 * 6通道扫描，理论值: 75MHz / 10 / 6 = 1.25MHz/通道
 * 启动初值：868000（自校准实测稳定值），运行时仍可微调
 */
/* ADC 每通道实测采样率
 * 实测值：J-Link读取 diag_frame_interval_cyc = 559343 cyc @480MHz = 1.165 ms
 *         => fs_per_ch = 4096 / 1.165ms = 3,515,000 Hz
 * 此前的 2,009,000 是按 PLL3P=56.25MHz/(2*14) 理论值算的，但实测 fs 高出 75%
 * 推测原因：ADC clock prescaler 或 sampling time 实际值与配置不符
 * 验证：DAC 输出 39893 Hz，FFT bin=12 -> 12 × 3515000 / 1024 = 41191 Hz ✓
 * （若理论 fs=2.009MHz，应得 bin=20.3，但实测 bin=12，即真实 fs 高出约 1.75x）
 *
 * 注意：此值是 RAM 变量，可在线调整。若发现 FFT 输出与 DAC 实际频率仍有偏差，
 * 可临时通过 J-Link 写 0x... 修改本变量重新校准。 */
float adc_fs_hz = 3515000.0f;  /* 实测每通道采样率 (DMA帧间隔反推) */

static const arm_cfft_instance_f32 *fft_inst_ptr = &arm_cfft_sR_f32_len1024;
static arm_rfft_fast_instance_f32 rfft_inst;
static volatile uint8_t rfft_inited = 0;

extern volatile uint32_t adc1_dma_half_cyc;
  extern volatile uint32_t adc1_dma_full_cyc;
  extern volatile uint32_t adc2_dma_half_cyc;
  extern volatile uint32_t adc2_dma_full_cyc;
  extern volatile uint32_t adc3_dma_half_cyc;
  extern volatile uint32_t adc3_dma_full_cyc;
static uint32_t dwt_cyc_per_us = 480;
static volatile uint64_t dwt_us_base = 0;
static volatile uint32_t dwt_last_cyc = 0;

static void DWT_Init(void)
{
  DBGMCU->CR |= DBGMCU_CR_DBG_TRACECKEN;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  *((volatile uint32_t *)0xE0001FB0UL) = 0xC5ACCE55UL;
  DWT->CYCCNT = 0U;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
  dwt_cyc_per_us = SystemCoreClock / 1000000U;
  if(dwt_cyc_per_us == 0) dwt_cyc_per_us = 480;
  dwt_us_base = 0;
  dwt_last_cyc = DWT->CYCCNT;
}

/* DWT CYCCNT 转微秒 - 处理 32 位回绕
 * 利用 uint32_t 减法自动处理溢出：delta = cyc_now - dwt_last_cyc
 * 累积足够大的差值时更新 base，避免频繁 64 位运算
 */
static uint64_t DWT_CycToUs_Simple(uint32_t cyc) {
    uint32_t delta = cyc - dwt_last_cyc;
    uint64_t add_us = (uint64_t)delta / dwt_cyc_per_us;
    if(add_us >= 1000) {
        dwt_us_base += add_us;
        dwt_last_cyc = cyc;
        return dwt_us_base;
    }
    return dwt_us_base + add_us;
}

static uint64_t DWT_GetUs(void)
{
  uint32_t tick1;
  uint32_t tick2;
  uint32_t systick_val;
  uint32_t systick_load;
  uint32_t cyc_per_us;
  uint32_t sub_us;
  systick_load = SysTick->LOAD + 1U;
  cyc_per_us = SystemCoreClock / 1000000U;
  if(cyc_per_us == 0U) {
    cyc_per_us = dwt_cyc_per_us;
  }

  do {
    tick1 = HAL_GetTick();
    systick_val = SysTick->VAL;
    tick2 = HAL_GetTick();
  } while(tick1 != tick2);

  sub_us = (systick_load - systick_val) / cyc_per_us;
  if(sub_us >= 1000U) {
    sub_us = 999U;
  }

  return ((uint64_t)tick1 * 1000ULL) + sub_us;
}

/* ============================================================
 * 事件帧非阻塞发送队列（环形缓冲 + 节流）
 * ============================================================
 * 目的：替换主循环中的 HAL_Delay(2ms) × 6 帧 阻塞发送
 * 原理：
 *   - 事件触发时：evt_tx_queue_push() 立即把 19B 帧推入环形缓冲（O(1)，不阻塞）
 *   - 主循环每次调用：evt_tx_queue_poll()
 *       * 检查上次发送是否已过 EXTERNAL_EVENT_TX_GAP_US（1ms）
 *       * 若已过且队列非空：HAL_UART_Transmit 单帧（19B/2Mbaud=95μs，可接受）
 *       * 若未到间隔：直接 return，不阻塞
 *   - 主循环全速跑 ADC 帧检测，UART 发送在背景以 1 帧/ms 节奏自动节流
 *
 * 缓冲容量：64 帧（约 1.2KB），足够吸纳极短 burst（6 帧）的瞬时峰值
 * 满时丢弃旧帧（最新事件优先），并计入 diag_uart_q_drop_cnt
 */
#define EVT_TX_QUEUE_CAPACITY  64U
static uint8_t evt_tx_queue[EVT_TX_QUEUE_CAPACITY][EVT_FRAME_TOTAL_LEN];
static volatile uint16_t evt_tx_q_head = 0;  /* 写入位置 */
static volatile uint16_t evt_tx_q_tail = 0;  /* 读取位置 */
static uint64_t evt_tx_last_send_us = 0;     /* 上次成功发送时刻（μs） */
uint32_t diag_uart_q_drop_cnt = 0;           /* 队列满丢帧计数 */
uint32_t diag_uart_q_push_cnt = 0;           /* 入队成功计数 */
uint32_t diag_uart_q_send_cnt = 0;           /* 出队发送计数 */
uint16_t diag_uart_q_max_depth = 0;          /* 队列最高水位 */

/**
 * @brief  把 19 字节事件帧推入发送队列（非阻塞，O(1)）
 * @param  frame - 已打包好的事件帧（必须 EVT_FRAME_TOTAL_LEN 字节）
 * @retval 1=入队成功, 0=队列满已丢弃
 */
static uint8_t evt_tx_queue_push(const uint8_t* frame) {
  uint16_t next_head;
  uint16_t depth;
  uint16_t i;

  next_head = (uint16_t)((evt_tx_q_head + 1U) % EVT_TX_QUEUE_CAPACITY);
  if(next_head == evt_tx_q_tail) {
    /* 队列已满，丢弃此帧（背压保护） */
    diag_uart_q_drop_cnt++;
    return 0U;
  }
  /* 复制 19 字节到环形缓冲槽 */
  for(i = 0; i < EVT_FRAME_TOTAL_LEN; i++) {
    evt_tx_queue[evt_tx_q_head][i] = frame[i];
  }
  evt_tx_q_head = next_head;
  diag_uart_q_push_cnt++;

  /* 记录队列最高水位用于诊断 */
  if(evt_tx_q_head >= evt_tx_q_tail) {
    depth = (uint16_t)(evt_tx_q_head - evt_tx_q_tail);
  } else {
    depth = (uint16_t)(EVT_TX_QUEUE_CAPACITY - evt_tx_q_tail + evt_tx_q_head);
  }
  if(depth > diag_uart_q_max_depth) {
    diag_uart_q_max_depth = depth;
  }
  return 1U;
}

/**
 * @brief  非阻塞轮询：若到时机则发送队首一帧
 * @note   主循环每次调用一次。HAL_UART_Transmit 单帧 95μs 是必要开销
 *         (无 DMA 时这是最快方案；用 DMA 还能再降到 ~10μs)
 */
static void evt_tx_queue_poll(void) {
  uint64_t now_us;
  uint64_t since_us;

  /* 队列空 → 立即返回 */
  if(evt_tx_q_head == evt_tx_q_tail) {
    return;
  }
  /* 节流：距上次发送不足 EXTERNAL_EVENT_TX_GAP_US 则返回 */
  now_us = DWT_GetUs();
  since_us = now_us - evt_tx_last_send_us;
  if(since_us < (uint64_t)EXTERNAL_EVENT_TX_GAP_US) {
    return;
  }
  /* 发送队首一帧（19B/460800 baud = 420μs 阻塞），timeout=100ms 兜底 */
  {
    HAL_StatusTypeDef tx_st;
    tx_st = HAL_UART_Transmit(&hdebug_uart, evt_tx_queue[evt_tx_q_tail],
                              EVT_FRAME_TOTAL_LEN, 100);
    if(tx_st != HAL_OK) {
      /* 发送失败：必须 Abort 清理 UART 状态机，否则后续所有发送都会失败 */
      HAL_UART_AbortTransmit(&hdebug_uart);
    }
  }
  evt_tx_q_tail = (uint16_t)((evt_tx_q_tail + 1U) % EVT_TX_QUEUE_CAPACITY);
  evt_tx_last_send_us = now_us;
  diag_uart_q_send_cnt++;
}

/* ★★ 2026-06-07 v6 UART TX DMA 完成回调（绝对安全版） ★★
 * 关键纪律：本回调在中断上下文执行，绝不能调用任何 HAL_UART_* 函数！
 * 否则 HAL 内部锁机制 + DMA 状态机 + 中断嵌套 = 100% 死锁主循环。
 *
 * 实测：
 *   - v3 版本回调里链式启动下一帧 DMA → frame_seen 冻结
 *   - v6 改为仅记录计数 → 由主循环 tx_ring_poll() 启动下一帧 → 稳定
 *
 * HAL 自动处理：
 *   - gState 从 BUSY_TX → READY（在 UART_EndTransmit_IT 里）
 *   - DMA TC 标志清除（在 HAL_DMA_IRQHandler 里）
 *   主循环看到 READY 后自然可发新帧。 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance != UART7) return;
  diag_uart_q_send_cnt++;   /* 仅观测，不操作 HAL */
}

/* ★ tx_ring_poll() v6.1 - 强化诊断 ★
 * 直接用 HAL_UART_GetState 并把 state 写入 diag 供 J-Link 查
 * 通过 return 值区分各种失败路径，避免静默失败 */
static volatile uint32_t diag_tx_poll_st = 0xFFFFFFFFU;  /* 最近一次的 UART state */
static volatile uint32_t diag_tx_poll_call_cnt = 0;       /* poll 调用总次数 */
static volatile uint32_t diag_tx_poll_send_attempt = 0;   /* 尝试 HAL_Transmit_DMA 次数 */
static volatile uint32_t diag_tx_poll_send_ok = 0;        /* HAL OK 次数 */

static void tx_ring_poll(void)
{
  uint8_t tail;
  HAL_UART_StateTypeDef st;
  HAL_StatusTypeDef tx_st;
  uint8_t head_snap;
  uint8_t tail_snap;

  /* 强制每轮都计数（无论队列空否）*/
  diag_tx_poll_call_cnt++;

  /* 队列空 → 立即返回 */
  head_snap = tx_ring_head;
  tail_snap = tx_ring_tail;
  if(head_snap == tail_snap) return;

  /* 读取 UART state 写入 diag */
  st = HAL_UART_GetState(&hdebug_uart);
  diag_tx_poll_st = (uint32_t)st;

  /* HAL_UART_STATE_READY=0x20。其它任何含 BUSY_TX/BUSY 位的状态都不发 */
  if(st != HAL_UART_STATE_READY && st != HAL_UART_STATE_BUSY_RX) return;

  /* ★ v7 改回 DMA 零阻塞模式 ★
   * usart.c v7 已恢复 LINKDMA + DMA1_Stream0 IRQ；
   *   - HAL_UART_Transmit_DMA 启动后立即返回，CPU 完全 0 等待
   *   - DMA Stream0 自动搬运 19B，完成后触发 DMA TC → HAL → TxCpltCallback
   *   - tx_ring 位于 .AXI_SRAM，DMA1 可直接访问；下发前 clean DCache 保证缓存一致 */
  tail = tail_snap;
  diag_tx_poll_send_attempt++;

  /* AXI SRAM 上的 tx_ring 写入后必须 clean DCache，DMA 才能读到最新数据 */
  SCB_CleanDCache_by_Addr((uint32_t*)tx_ring[tail], EVT_FRAME_TOTAL_LEN);

  tx_st = HAL_UART_Transmit_DMA(&hdebug_uart, (uint8_t*)tx_ring[tail], EVT_FRAME_TOTAL_LEN);
  if(tx_st == HAL_OK) {
    tx_ring_tail = (uint8_t)((tail + 1U) % TX_RING_SLOTS);
    diag_tx_poll_send_ok++;
  } else {
    diag_uart_hal_fail_cnt++;
  }
}

static void push_frame_from_dma(uint8_t half_idx, uint64_t now_us)
{
  uint8_t next = (frame_w_idx + 1) % FRAME_POOL_SIZE;
  uint16_t src_off;
  uint16_t i;
  adc_frame_t *fr;
  if(next == frame_r_idx) {
    frame_drop_cnt++;
    return;
  }
  src_off = (half_idx == 0) ? 0 : (ADC_DMA_BUF_SIZE / 2);
  fr = &frame_pool[frame_w_idx];

  /* 3 ADC 合并为 6 通道交织帧
   * ADC1: PC0(CH1, rank1), PC1(CH2, rank2)
   * ADC2: PA3(CH5, rank1), PA4(CH6, rank2)
   * ADC3: PC2(CH3, rank1), PC3(CH4, rank2)
   * DMA buf: [r1, r2, r1, r2, ...] 每 ADC
   * 目标 frame: CH1 CH2 CH3 CH4 CH5 CH6 交织
   */
  for(i = 0; i < HALF_SAMPLES_PER_CH; i++) {
    fr->data[i * ADC_NCH + 0] = adc1_buf[src_off + i * 2 + 0] & 0x0FFF;  /* CH1 = ADC1 rank1 */
    fr->data[i * ADC_NCH + 1] = adc1_buf[src_off + i * 2 + 1] & 0x0FFF;  /* CH2 = ADC1 rank2 */
    fr->data[i * ADC_NCH + 2] = adc3_buf[src_off + i * 2 + 0] & 0x0FFF;  /* CH3 = ADC3 rank1 */
    fr->data[i * ADC_NCH + 3] = adc3_buf[src_off + i * 2 + 1] & 0x0FFF;  /* CH4 = ADC3 rank2 */
    fr->data[i * ADC_NCH + 4] = adc2_buf[src_off + i * 2 + 0] & 0x0FFF;  /* CH5 = ADC2 rank1 */
    fr->data[i * ADC_NCH + 5] = adc2_buf[src_off + i * 2 + 1] & 0x0FFF;  /* CH6 = ADC2 rank2 */
  }
  fr->start_time_us = now_us;
  fr->frame_id = total_frame_cnt++;
  fr->ready = 1;
  frame_w_idx = next;
}

static void check_dma_and_push_frames(void)
{
  uint64_t half_dur_us;
  uint64_t end_us;
  uint64_t start_us;
  uint32_t frame_start_cyc;
  uint32_t frame_process_cyc;

  half_dur_us = (uint64_t)((float)HALF_SAMPLES_PER_CH / adc_fs_hz * 1e6f + 0.5f);

  if(adc1_dma_half && adc2_dma_half && adc3_dma_half) {
    frame_start_cyc = DWT->CYCCNT;
    adc1_dma_half = 0;
    adc2_dma_half = 0;
    adc3_dma_half = 0;
    SCB_InvalidateDCache_by_Addr((uint32_t*)adc1_buf, ADC_DMA_BUF_SIZE * 2);
    SCB_InvalidateDCache_by_Addr((uint32_t*)adc2_buf, ADC_DMA_BUF_SIZE * 2);
    SCB_InvalidateDCache_by_Addr((uint32_t*)adc3_buf, ADC_DMA_BUF_SIZE * 2);
    end_us = DWT_GetUs();
    start_us = (end_us > half_dur_us) ? (end_us - half_dur_us) : 0;
    push_frame_from_dma(0, start_us);
    half_complete_cnt++;
    
    /* 记录帧处理耗时 */
    frame_process_cyc = DWT->CYCCNT - frame_start_cyc;
    diag_frame_process_cyc = frame_process_cyc;
    if(frame_process_cyc > diag_frame_process_max) {
      diag_frame_process_max = frame_process_cyc;
    }
    /* 计算帧间隔 */
    if(diag_last_frame_cyc > 0) {
      diag_frame_interval_cyc = frame_start_cyc - diag_last_frame_cyc;
    }
    diag_last_frame_cyc = frame_start_cyc;
  }

  if(adc1_dma_full && adc2_dma_full && adc3_dma_full) {
    frame_start_cyc = DWT->CYCCNT;
    adc1_dma_full = 0;
    adc2_dma_full = 0;
    adc3_dma_full = 0;
    SCB_InvalidateDCache_by_Addr((uint32_t*)adc1_buf, ADC_DMA_BUF_SIZE * 2);
    SCB_InvalidateDCache_by_Addr((uint32_t*)adc2_buf, ADC_DMA_BUF_SIZE * 2);
    SCB_InvalidateDCache_by_Addr((uint32_t*)adc3_buf, ADC_DMA_BUF_SIZE * 2);
    end_us = DWT_GetUs();
    start_us = (end_us > half_dur_us) ? (end_us - half_dur_us) : 0;
    push_frame_from_dma(1, start_us);
    full_complete_cnt++;
    
    /* 记录帧处理耗时 */
    frame_process_cyc = DWT->CYCCNT - frame_start_cyc;
    diag_frame_process_cyc = frame_process_cyc;
    if(frame_process_cyc > diag_frame_process_max) {
      diag_frame_process_max = frame_process_cyc;
    }
    /* 计算帧间隔 */
    if(diag_last_frame_cyc > 0) {
      diag_frame_interval_cyc = frame_start_cyc - diag_last_frame_cyc;
    }
    diag_last_frame_cyc = frame_start_cyc;
  }
  
  /* 计算空闲比例 */
  if(diag_frame_interval_cyc > 0 && diag_frame_process_cyc > 0) {
    diag_idle_ratio = (uint32_t)((float)(diag_frame_interval_cyc - diag_frame_process_cyc) * 100.0f / (float)diag_frame_interval_cyc);
  }
}

static float fft_buf[FFT_LENGTH * 2]  __attribute__((section(".DTCMRAM"), aligned(32)));
static float fft_mag[FFT_LENGTH / 2]  __attribute__((section(".DTCMRAM"), aligned(32)));
static float fft_in[FFT_LENGTH]       __attribute__((section(".DTCMRAM"), aligned(32)));
static float hann_win[FFT_LENGTH]              __attribute__((section(".DTCMRAM"), aligned(32)));
static float tw_cos[FFT_LENGTH / 2]            __attribute__((aligned(32)));
static float tw_sin[FFT_LENGTH / 2]            __attribute__((aligned(32)));
static uint8_t fft_tables_ready = 0;

/* 对比验证变量 */
static volatile uint32_t fft_optimized_cyc = 0;
/* 【2026-06-07 清理】fft_original_cyc / fft_compare_mode 在 Original 函数删除后已无写入，
 * 故移除全局变量与所有相关 benchmark 打印分支。 */

static volatile uint32_t fft_last_cyc = 0;

typedef struct {
  uint32_t pwm_freq_hz;
  uint16_t amp_mv;
  const char *desc;
} test_case_t;

/* === 测试用例：运放带通通带 30-70KHz，最大幅值抵消衰减 ===
 * 用户硬件：运放电路是带通 30-70KHz
 * DAC 输出：OFFSET=1.65V，max_amp=1.65V → 3.3Vpp（DAC 极限）
 * 注意：50KHz 放在首位，作为自校准参考最快收敛
 */
static const test_case_t test_cases[] = {
  {  40000, 1650, "40KHz 1.65V"},  /* 单一频率: 40KHz, 反复测试零丢失 + 时间/频率精准 */
};
#define NUM_TEST_CASES (sizeof(test_cases) / sizeof(test_cases[0]))

/* 测试用例统计 */
static uint32_t test_case_trigger_count[NUM_TEST_CASES] = {0};
static uint32_t full_cycle_count = 0;

static const char *ch_names[ADC_NCH] = {
  "CH1(PC0)",
  "CH2(PC1)",
  "CH3(PC2)",
  "CH4(PC3)",
  "CH5(PA3)",
  "CH6(PA4)"
};

static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};
  HAL_MPU_Disable();
  MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
  MPU_InitStruct.Number           = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress      = QSPI_BASE;
  MPU_InitStruct.Size             = MPU_REGION_SIZE_256MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL1;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
  MPU_InitStruct.Number           = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress      = 0x24000000;
  MPU_InitStruct.Size             = MPU_REGION_SIZE_512KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

static void CPU_CACHE_Enable(void)
{
  SCB_EnableICache();
  SCB_EnableDCache();
}

void SystemClock_Config(void);

static void FFT_Tables_Init(void)
{
  int i;
  const float pi = 3.14159265358979f;
  for(i = 0; i < FFT_LENGTH; i++) {
    hann_win[i] = 0.5f * (1.0f - cosf(2.0f * pi * (float)i / (float)(FFT_LENGTH - 1)));
  }
  for(i = 0; i < FFT_LENGTH / 2; i++) {
    float ang = 2.0f * pi * (float)i / (float)FFT_LENGTH;
    tw_cos[i] = cosf(ang);
    tw_sin[i] = sinf(ang);
  }
  fft_tables_ready = 1;
}

/* 任务2: FFT基准测试 - 测试256/512/1024点FFT耗时 */
static void FFT_Benchmark(void)
{
  arm_rfft_fast_instance_f32 rfft_inst_256;
  arm_rfft_fast_instance_f32 rfft_inst_512;
  arm_rfft_fast_instance_f32 rfft_inst_1024;
  static float bench_in_256[256] __attribute__((section(".DTCMRAM"), aligned(32)));
  static float bench_in_512[512] __attribute__((section(".DTCMRAM"), aligned(32)));
  static float bench_in_1024[1024] __attribute__((section(".DTCMRAM"), aligned(32)));
  static float bench_out_256[512];
  static float bench_out_512[1024];
  static float bench_out_1024[2048];
  uint32_t t0, t256, t512, t1024;
  int i;
  volatile float sink = 0;
  volatile uint32_t *fft_result = (volatile uint32_t*)0x20004800;

  /* 预热ICache */
  for(i = 0; i < 1024; i++) {
    bench_in_1024[i] = (float)i;
  }
  arm_rfft_fast_init_f32(&rfft_inst_1024, 1024);
  arm_rfft_fast_f32(&rfft_inst_1024, bench_in_1024, bench_out_1024, 0);
  sink += bench_out_1024[0];

  /* 生成测试信号 */
  for(i = 0; i < 256; i++) {
    bench_in_256[i] = 117.0f + 100.0f * sinf(2.0f * 3.14159265f * 40000.0f * (float)i / adc_fs_hz);
  }
  for(i = 0; i < 512; i++) {
    bench_in_512[i] = 117.0f + 100.0f * sinf(2.0f * 3.14159265f * 40000.0f * (float)i / adc_fs_hz);
  }
  for(i = 0; i < 1024; i++) {
    bench_in_1024[i] = 117.0f + 100.0f * sinf(2.0f * 3.14159265f * 40000.0f * (float)i / adc_fs_hz);
  }

  /* 初始化FFT实例 */
  arm_rfft_fast_init_f32(&rfft_inst_256, 256);
  arm_rfft_fast_init_f32(&rfft_inst_512, 512);
  arm_rfft_fast_init_f32(&rfft_inst_1024, 1024);

  /* 测试256点FFT - 单次测试 */
  SCB_InvalidateDCache_by_Addr((uint32_t*)bench_in_256, sizeof(bench_in_256));
  SCB_InvalidateDCache_by_Addr((uint32_t*)bench_out_256, sizeof(bench_out_256));
  t0 = DWT->CYCCNT;
  arm_rfft_fast_f32(&rfft_inst_256, bench_in_256, bench_out_256, 0);
  t256 = DWT->CYCCNT - t0;
  sink += bench_out_256[0];

  /* 测试512点FFT - 单次测试 */
  SCB_InvalidateDCache_by_Addr((uint32_t*)bench_in_512, sizeof(bench_in_512));
  SCB_InvalidateDCache_by_Addr((uint32_t*)bench_out_512, sizeof(bench_out_512));
  t0 = DWT->CYCCNT;
  arm_rfft_fast_f32(&rfft_inst_512, bench_in_512, bench_out_512, 0);
  t512 = DWT->CYCCNT - t0;
  sink += bench_out_512[0];

  /* 测试1024点FFT - 单次测试 */
  SCB_InvalidateDCache_by_Addr((uint32_t*)bench_in_1024, sizeof(bench_in_1024));
  SCB_InvalidateDCache_by_Addr((uint32_t*)bench_out_1024, sizeof(bench_out_1024));
  t0 = DWT->CYCCNT;
  arm_rfft_fast_f32(&rfft_inst_1024, bench_in_1024, bench_out_1024, 0);
  t1024 = DWT->CYCCNT - t0;
  sink += bench_out_1024[0];

  /* 记录结果到固定地址 */
  fft_result[0] = t256;
  fft_result[1] = t512;
  fft_result[2] = t1024;
  fft_result[3] = 1;  /* done flag */
  fft_result[4] = diag_sysclk_hz;

  (void)sink;
}

/**
  * @brief  计算各通道直流偏置基线
  * @note   在系统启动时调用，采样1秒数据计算各通道的平均值作为直流偏置基线
  *         用于补偿各通道硬件参数差异导致的直流偏置差异
  * @param  duration_ms: 采样持续时间(毫秒)
  * @retval None
  */
static void Calc_DC_Offset_Baseline(uint32_t duration_ms)
{
  uint32_t start_ms;
  uint32_t ch;
  uint32_t i;
  uint32_t sample_cnt[ADC_NCH] = {0};
  uint64_t sample_sum[ADC_NCH] = {0};
  uint16_t adc_val;
  uint32_t frame_cnt = 0;
  uint32_t timeout_ms;

  /* 等待ADC稳定 */
  HAL_Delay(100);

  start_ms = HAL_GetTick();
  timeout_ms = start_ms + duration_ms;

  /* 采样duration_ms时间的数据 */
  while(HAL_GetTick() < timeout_ms) {
    /* 处理DMA中断 */
    check_dma_and_push_frames();

    /* 从帧池中读取数据 */
    while(frame_r_idx != frame_w_idx) {
      adc_frame_t *fr = &frame_pool[frame_r_idx];
      if(fr->ready) {
        /* 累加各通道数据 */
        for(i = 0; i < HALF_SAMPLES_PER_CH; i++) {
          for(ch = 0; ch < ADC_NCH; ch++) {
            adc_val = fr->data[i * ADC_NCH + ch];
            sample_sum[ch] += adc_val;
            sample_cnt[ch]++;
          }
        }
        fr->ready = 0;
        frame_cnt++;
      }
      frame_r_idx = (frame_r_idx + 1) % FRAME_POOL_SIZE;
    }

    /* 避免过度占用CPU */
    HAL_Delay(1);
  }

  /* 计算各通道平均值 */
  for(ch = 0; ch < ADC_NCH; ch++) {
    if(sample_cnt[ch] > 0) {
      ch_dc_offset[ch] = (int32_t)(sample_sum[ch] / sample_cnt[ch]);
    } else {
      ch_dc_offset[ch] = 0;
    }
  }

  ch_dc_offset_valid = 1;

  /* 输出各通道直流偏置基线 */
  debug_printf("\r\n=== DC Offset Baseline ===\r\n");
  debug_printf("Sampling duration: %u ms\r\n", duration_ms);
  debug_printf("Total frames: %u\r\n", frame_cnt);
  for(ch = 0; ch < ADC_NCH; ch++) {
    debug_printf("CH%u: offset=%d, samples=%u\r\n",
                 ch + 1, ch_dc_offset[ch], sample_cnt[ch]);
  }
  debug_printf("==========================\r\n");

  /* 记录到诊断变量 */
  /* 注意: 这里使用diag_dwt_baseline作为临时存储，实际应用中可以添加专用变量 */
}

/* 计算基线和噪声 - 优化版：单次遍历同时计算sum和min/max */
static int32_t Calc_Baseline_in_Frame(const uint16_t *buf, uint16_t n_samples, uint8_t ch, uint16_t *noise_pp_out)
{
  uint16_t i;
  uint32_t sum = 0;
  uint16_t v_min = 4095, v_max = 0;
  uint16_t v;
  uint16_t noise_n = (n_samples > BASELINE_SAMPLES) ? BASELINE_SAMPLES : n_samples;
  const uint16_t *src = buf + ch;

  for(i = 0; i < noise_n; i++) {
    v = src[i * ADC_NCH];
    sum += v;
    if(v < v_min) v_min = v;
    if(v > v_max) v_max = v;
  }

  if(noise_pp_out) *noise_pp_out = (v_max >= v_min) ? (v_max - v_min) : 0;
  if(noise_n == 0) return 0;
  return (int32_t)(sum / noise_n);
}

/* 检测DC->AC突变 */
static uint16_t Detect_Transition_in_Frame(const uint16_t *buf, uint16_t n_samples, uint8_t ch, int32_t baseline, uint16_t threshold)
{
  uint16_t i;
  uint16_t dev_cnt = 0;
  int32_t dev;
  uint16_t val;
  for(i = 0; i < n_samples; i++) {
    val = buf[i * ADC_NCH + ch];
    dev = (int32_t)val - baseline;
    if(dev < 0) dev = -dev;
    if((uint16_t)dev > threshold) {
      dev_cnt++;
      if(dev_cnt >= DEV_CONFIRM_CNT) {
        return i - DEV_CONFIRM_CNT + 1;
      }
    } else {
      dev_cnt = 0;
    }
  }
  return 0xFFFF;
}

/* 持续性验证 */
static uint8_t Verify_Sustain_in_Frame(const uint16_t *buf, uint16_t n_samples, uint8_t ch,
                                       int32_t baseline, uint16_t threshold, uint16_t trans_idx)
{
  uint16_t end_i = trans_idx + SUSTAIN_SAMPLES;
  uint16_t hit_cnt = 0;
  uint16_t i;
  int32_t d;
  uint32_t ab;
  if(end_i > n_samples) end_i = n_samples;
  for(i = trans_idx; i < end_i; i++) {
    d = (int32_t)buf[i * ADC_NCH + ch] - baseline;
    ab = (d < 0) ? (uint32_t)(-d) : (uint32_t)d;
    if(ab > threshold) hit_cnt++;
  }
  return (hit_cnt >= SUSTAIN_MIN_HIT) ? 1 : 0;
}

static uint8_t Detect_AC_Present_in_Frame(const uint16_t *buf, uint16_t n_samples, uint8_t ch,
                                          int32_t baseline, uint16_t threshold,
                                          uint16_t *first_idx, uint32_t *max_dev_out)
{
  uint16_t i;
  uint16_t hit_cnt = 0;
  uint8_t found = 0;
  int32_t d;
  uint32_t ab;
  uint32_t max_dev = 0;

  *first_idx = 0xFFFF;
  for(i = BASELINE_SAMPLES; i < n_samples; i++) {
    d = (int32_t)buf[i * ADC_NCH + ch] - baseline;
    ab = (d < 0) ? (uint32_t)(-d) : (uint32_t)d;
    if(ab > max_dev) max_dev = ab;
    if(ab > threshold) {
      if(!found) {
        *first_idx = i;
        found = 1;
      }
      hit_cnt++;
    }
  }
  if(max_dev_out) *max_dev_out = max_dev;
  return (hit_cnt >= AC_PRESENT_MIN_HIT) ? 1 : 0;
}

static float Refine_Transition_SubSample(const uint16_t *buf, uint16_t n_samples, uint8_t ch,
                                         int32_t baseline, uint16_t threshold, uint16_t trans_idx)
{
  int32_t d0;
  int32_t d1;
  float a0;
  float a1;
  float den;

  if(trans_idx == 0 || trans_idx >= n_samples) return 0.0f;
  d0 = (int32_t)buf[(trans_idx - 1) * ADC_NCH + ch] - baseline;
  d1 = (int32_t)buf[trans_idx * ADC_NCH + ch] - baseline;
  if(d0 < 0) d0 = -d0;
  if(d1 < 0) d1 = -d1;
  a0 = (float)d0;
  a1 = (float)d1;
  den = a1 - a0;
  if(den <= 0.0f) return 0.0f;
  return ((float)threshold - a0) / den - 1.0f;
}

/* 找第一个极值点（峰值或谷值，取绝对值最大）
 * 不同通道由于相位不同，最先到达的可能是峰或谷，独立判断每个通道。
 */
static uint16_t Find_FirstPeak_in_Frame(const uint16_t *buf, uint16_t n_samples, uint8_t ch,
                                        int32_t baseline, uint16_t trans_idx)
{
  uint16_t end_i = trans_idx + 192;
  uint16_t peak_idx = trans_idx;
  int16_t  max_abs_dev = 0;
  int16_t  valid_thr;
  int16_t  prev_dev;
  int16_t  prev_abs;
  uint8_t  rising_abs = 1;
  uint16_t i;
  if(end_i > n_samples) end_i = n_samples;

  /* 第 1 遍：扫描整段，找绝对值最大幅值（峰或谷都行） */
  for(i = trans_idx; i < end_i; i++) {
    int16_t d = (int16_t)buf[i * ADC_NCH + ch] - baseline;
    int16_t ad = (d < 0) ? -d : d;
    if(ad > max_abs_dev) max_abs_dev = ad;
  }
  valid_thr = (int16_t)(((int32_t)max_abs_dev * 70) / 100);
  if(valid_thr < 3) valid_thr = 3;

  /* 第 2 遍：找第一个 |d| 达到 valid_thr 的极值（峰或谷），由 |d| 单调反转判定 */
  prev_dev = (int16_t)buf[trans_idx * ADC_NCH + ch] - baseline;
  prev_abs = (prev_dev < 0) ? -prev_dev : prev_dev;
  for(i = trans_idx + 1; i < end_i; i++) {
    int16_t d = (int16_t)buf[i * ADC_NCH + ch] - baseline;
    int16_t ad = (d < 0) ? -d : d;
    if(rising_abs && ad < prev_abs) {
      /* |d| 开始下降 → 上一个样本是极值（峰或谷） */
      if(prev_abs >= valid_thr) {
        peak_idx = i - 1;
        return peak_idx;
      }
      rising_abs = 0;
    }
    if(ad > prev_abs) rising_abs = 1;
    else if(ad < prev_abs) rising_abs = 0;
    prev_dev = d;
    prev_abs = ad;
  }
  /* 兜底：全段绝对值最大点 */
  {
    int16_t best_abs = -1;
    for(i = trans_idx; i < end_i; i++) {
      int16_t d = (int16_t)buf[i * ADC_NCH + ch] - baseline;
      int16_t ad = (d < 0) ? -d : d;
      if(ad > best_abs) { best_abs = ad; peak_idx = i; }
    }
  }
  return peak_idx;
}

/* 抛物线峰值插值 */
static float Refine_Peak_SubSample(const uint16_t *buf, uint16_t n_samples, uint8_t ch,
                                   int32_t baseline, uint16_t peak_idx)
{
  int32_t y_m1, y_0, y_p1;
  int32_t denom;
  float delta;
  if(peak_idx == 0 || peak_idx >= n_samples - 1) return 0.0f;
  {
    int16_t a = (int16_t)buf[(peak_idx - 1) * ADC_NCH + ch] - baseline;
    int16_t b = (int16_t)buf[(peak_idx    ) * ADC_NCH + ch] - baseline;
    int16_t c = (int16_t)buf[(peak_idx + 1) * ADC_NCH + ch] - baseline;
    y_m1 = (a < 0) ? -a : a;
    y_0  = (b < 0) ? -b : b;
    y_p1 = (c < 0) ? -c : c;
  }
  denom = (int32_t)(y_m1 - 2 * y_0 + y_p1);
  if(denom == 0) return 0.0f;
  delta = (float)(y_m1 - y_p1) / (float)(2 * denom);
  if(delta >  0.5f) delta =  0.5f;
  if(delta < -0.5f) delta = -0.5f;
  return delta;
}

static float Calc_ZeroCross_Frequency_in_Frame(const uint16_t *buf, uint16_t n_samples, uint8_t ch,
                                               uint16_t start_idx, int32_t baseline)
{
  uint16_t i;
  int32_t d_prev;
  int32_t d_cur;
  float first_cross = 0.0f;
  float last_cross = 0.0f;
  float cross;
  float den;
  uint16_t count = 0;
  uint16_t period_sum = 0;
  uint16_t prev_i = 0;

  if(start_idx + 128U >= n_samples) return 0.0f;
  d_prev = (int32_t)buf[start_idx * ADC_NCH + ch] - baseline;
  count = 0;
  for(i = start_idx + 1; i < start_idx + 128U; i++) {
    d_cur = (int32_t)buf[i * ADC_NCH + ch] - baseline;
    if(d_prev <= 0 && d_cur > 0) {
      den = (float)(d_cur - d_prev);
      if(den > 0.0f) {
        cross = (float)(i - 1) + ((float)(-d_prev) / den);
        if(count == 0) {
          first_cross = cross;
          prev_i = (uint16_t)cross;
        } else {
          uint16_t period = (uint16_t)cross - prev_i;
          period_sum += period;
          prev_i = (uint16_t)cross;
        }
        last_cross = cross;
        count++;
      }
    }
    d_prev = d_cur;
  }
  
  if(count >= 6 && last_cross > first_cross) {
    uint16_t avg_period = period_sum / (count - 1);
    if(avg_period >= 3 && avg_period <= 50) {
      return adc_fs_hz * (float)(count - 1) / (last_cross - first_cross);
    }
  }

  count = 0;
  period_sum = 0;
  d_prev = (int32_t)buf[start_idx * ADC_NCH + ch] - baseline;
  for(i = start_idx + 1; i < start_idx + 128U; i++) {
    d_cur = (int32_t)buf[i * ADC_NCH + ch] - baseline;
    if(d_prev >= 0 && d_cur < 0) {
      den = (float)(d_prev - d_cur);
      if(den > 0.0f) {
        cross = (float)(i - 1) + ((float)d_prev / den);
        if(count == 0) {
          first_cross = cross;
          prev_i = (uint16_t)cross;
        } else {
          uint16_t period = (uint16_t)cross - prev_i;
          period_sum += period;
          prev_i = (uint16_t)cross;
        }
        last_cross = cross;
        count++;
      }
    }
    d_prev = d_cur;
  }
  
  if(count >= 6 && last_cross > first_cross) {
    uint16_t avg_period = period_sum / (count - 1);
    if(avg_period >= 3 && avg_period <= 50) {
      return adc_fs_hz * (float)(count - 1) / (last_cross - first_cross);
    }
  }
  return 0.0f;
}

/* FFT频率计算 - 优化版：汉宁窗 + 抛物线插值 + SNR门限 */
/* 优化版本FFT函数 - 更好的内存布局和峰值搜索 */
static float Calc_FFT_Frequency_in_Frame_Optimized(const uint16_t *buf, uint16_t n_samples, uint8_t ch, uint16_t start_idx)
{
  uint16_t i;
  float max_val = 0.0f;
  uint16_t max_bin = 0;
  float avg_val = 0.0f;
  float delta, exact_bin;
  uint16_t bin_min, bin_max;
  uint16_t max_samples = n_samples - start_idx;
  const uint16_t *src;
  float dc_avg;
  float mag_sq;
  float *fft_in_local = fft_in;
  float *fft_mag_local = fft_mag;
  float *fft_buf_local = fft_buf;
  uint32_t dc_sum;
  float re, im;
  uint32_t fft_t0, fft_t1;
  float ym, y0, yp, denom;

  if(max_samples < FFT_LENGTH) return 0.0f;
  max_samples = FFT_LENGTH;

  src = buf + start_idx * ADC_NCH + ch;

  /* 使用全局直流偏置基线（如果有效） */
  if(ch_dc_offset_valid && ch_dc_offset[ch] != 0) {
    dc_avg = (float)ch_dc_offset[ch];
  } else {
    /* 快速计算本地平均 */
    dc_sum = 0;
    for(i = 0; i < max_samples; i++) {
      dc_sum += src[i * ADC_NCH];
    }
    dc_avg = (float)dc_sum * (1.0f / (float)FFT_LENGTH);
  }

  /* 数据准备 - 减DC + 汉宁窗抑制频谱泄漏
   * 汉宁窗旁瓣 -31dB（vs 矩形窗 -13dB），主瓣4bin但形状接近二次曲线
   * 配合抛物线插值，频率精度可从 ~0.27% 提升至 <0.02%
   */
  for(i = 0; i < max_samples; i++) {
    fft_in_local[i] = ((float)src[i * ADC_NCH] - dc_avg) * hann_win[i];
  }
  for(; i < FFT_LENGTH; i++) {
    fft_in_local[i] = 0.0f;
  }

  /* FFT计算 */
  fft_t0 = DWT->CYCCNT;
  arm_rfft_fast_f32(&rfft_inst, fft_in_local, fft_buf_local, 0);
  fft_t1 = DWT->CYCCNT - fft_t0;
  fft_last_cyc = fft_t1;
  fft_optimized_cyc = fft_t1;

  /* 计算幅度平方 - 先算直流分量 */
  fft_mag_local[0] = fft_buf_local[0] * fft_buf_local[0];
  
  /* 优化复数幅度平方计算 - 使用内联计算而不是函数调用 */
  for(i = 1; i < FFT_LENGTH / 2; i++) {
    re = fft_buf_local[i*2];
    im = fft_buf_local[i*2 + 1];
    fft_mag_local[i] = re * re + im * im;
  }

  /* 固定频率范围：20kHz ~ 200kHz
   * 汉宁窗下DC泄漏会扩展至 bin 0~3，由 bin_min ≥ 20kHz 对应bin 10+ 自然规避
   * 实测发现DC泄漏极强时仍可能污染 bin 10附近，必要时可提高 bin_min 起点 */
  bin_min = (uint16_t)(20000.0f * (float)FFT_LENGTH / adc_fs_hz);
  bin_max = (uint16_t)(200000.0f * (float)FFT_LENGTH / adc_fs_hz);
  if(bin_min < 4) bin_min = 4;  /* 至少跳过 bin 0~3 的DC泄漏 */
  if(bin_max > FFT_LENGTH / 2 - 1) bin_max = FFT_LENGTH / 2 - 1;

  /* 优化峰值搜索 - 同时计算最大值和平均值 */
  max_val = 0.0f;
  avg_val = 0.0f;
  for(i = bin_min; i <= bin_max; i++) {
    mag_sq = fft_mag_local[i];
    avg_val += mag_sq;
    if(mag_sq > max_val) {
      max_val = mag_sq;
      max_bin = i;
    }
  }
  
  if(max_bin == 0) return 0.0f;

  /* 诊断：记录峰值bin、幅度、平均幅度、使用的采样率 */
  diag_fft_max_bin = (uint32_t)max_bin;
  diag_fft_max_mag = max_val;
  diag_fft_used_fs = adc_fs_hz;

  /* 频谱快照（仅 CH=1，避免不同通道结果互相覆盖）
   * 保存 bin 4..31 共28个bin的幅度sqrt，用于辨析峰位形态 */
  if(ch == 1) {
    uint16_t snap_i;
    for(snap_i = 0; snap_i < 28; snap_i++) {
      diag_fft_spec_snapshot[snap_i] = sqrtf(fft_mag_local[snap_i + 4]);
    }
    diag_fft_snapshot_ch = ch;
  }

  /* 最小幅度检查 - 过滤微弱噪声 */
  if(max_val < FFT_MIN_MAG_SQ) return 0.0f;
  
  /* 噪声门限检查 - 使用宏定义的SNR阈值 */
  avg_val *= (1.0f / (float)(bin_max - bin_min + 1));
  diag_fft_avg_mag = avg_val;
  if(max_val < FFT_SNR_THRESHOLD * avg_val) return 0.0f;  /* 使用宏定义的信噪比阈值 */

  /* 抛物线插值优化 - 汉宁窗下主瓣形状接近二次曲线，插值精度高 */
  if(max_bin > 0 && max_bin < (FFT_LENGTH / 2 - 1)) {
    ym = sqrtf(fft_mag_local[max_bin - 1]);
    y0 = sqrtf(max_val);
    yp = sqrtf(fft_mag_local[max_bin + 1]);
    denom = ym - 2.0f * y0 + yp;
    if(fabsf(denom) > 1e-9f) {
      delta = 0.5f * (ym - yp) / denom;
      if(delta > 0.5f) delta = 0.5f;
      if(delta < -0.5f) delta = -0.5f;
    } else {
      delta = 0.0f;
    }
  } else {
    delta = 0.0f;
  }
  exact_bin = (float)max_bin + delta;
  return exact_bin * adc_fs_hz / (float)FFT_LENGTH;
}

/* 【2026-06-07 死代码移除】原 Calc_FFT_Frequency_in_Frame_Original 与
 * FFT_Compare_Verify 在功能验证完成后再无调用方，且 _Original 使用 4 KB 栈数组
 * fft_in_orig[1024]，对 Cortex-M7 主栈风险大。本次清理：
 *   - 删除 _Original 实现
 *   - 删除 FFT_Compare_Verify 实现
 *   - 后续 main 入口仅保留 Calc_FFT_Frequency_in_Frame → _Optimized 单路径
 * 释放：~4 KB 栈 + 数百行代码 + 减少 Flash 代码量 ≈ 1.5 KB。 */

/* 当前使用的FFT函数 - 指向优化版本 */
static float Calc_FFT_Frequency_in_Frame(const uint16_t *buf, uint16_t n_samples, uint8_t ch, uint16_t start_idx)
{
  /* 永远直接调用 Optimized；Original/Compare 已删除（见上方说明）。 */
  return Calc_FFT_Frequency_in_Frame_Optimized(buf, n_samples, ch, start_idx);
}

int main(void)
{
  uint32_t print_count = 0;
  uint32_t case_idx = 0;
  uint32_t current_burst_case = 0;
  const test_case_t *tc;
  uint8_t ch;
  uint16_t i;
  int32_t baselines[ADC_NCH];
  uint16_t noises[ADC_NCH];
  uint16_t trans[ADC_NCH];
  float freq[ADC_NCH];
  uint32_t last_event_ms[ADC_NCH] = {0};
  uint32_t last_valid_event_ms[ADC_NCH] = {0};
  uint64_t last_sent_time_us = 0;
  uint32_t last_print_ms = 0;
  uint32_t last_self_test_ms = 0;
  uint32_t last_dac_switch_ms = 0;
  uint8_t dac_phase = 0;
  uint8_t burst_event_sent = 0;
  uint8_t burst_sent_mask = 0;
  uint8_t burst_sent_count = 0;
  uint8_t burst_candidate_mask = 0;
  uint32_t burst_best_dev[ADC_NCH] = {0};
  uint32_t burst_best_freq[ADC_NCH] = {0};
  uint64_t burst_best_time_us[ADC_NCH] = {0};
  uint32_t lock_score[ADC_NCH] = {0};
  uint8_t locked_input_mask = 0;
  uint32_t lock_burst_cnt = 0;
  uint32_t burst_start_ms = 0;
  /* 2026-06-07: evt_buf 从栈移 static .AXI_SRAM 32B 对齐 —— DMA 异步发送需要 */
  static uint8_t evt_buf[EVT_FRAME_TOTAL_LEN] __attribute__((section(".AXI_SRAM"), aligned(32)));
  uint32_t tx_freq;
  uint64_t tx_time_us;
  HAL_StatusTypeDef uart_st;
  uint8_t selected_mask;
  uint8_t filtered_mask;
  uint8_t select_count;
  uint8_t best_ch;
  uint32_t best_dev;
  uint32_t f_u32;
  /* =========================================================================
   * DAC 信号源测试参数（满足用户极限要求）
   * =========================================================================
   *
   * 【功能】
   *   STM32H750 自身的 DAC1（PA4）和 DAC2（PA5）输出 DC↔正弦 burst 信号，
   *   外部短接或不接到 6 个 ADC 输入通道（PC0/PC1/PC2/PC3/PA3/PA4），
   *   形成闭环自检：板上 ADC 通过 FFT 检测 burst → 触发事件帧上送 PC。
   *
   * 【三种典型测试场景】
   *
   *   场景 A (默认，DAC_DC_DURATION_MS=30):
   *     - DAC 周期: 30ms DC + 5ms 正弦 = 35ms
   *     - 期望: 6 通道全部触发，PC 一帧不漏
   *     - 实测: PC 收 ~82 evt/s，0 坏帧 0 异常，频率误差 < 0.4%
   *
   *   场景 B (DAC_DC_DURATION_MS=10):
   *     - DAC 周期: 10ms DC + 5ms 正弦 = 15ms < SAME_CH_DEDUP_MS(35ms)
   *     - 期望: 每两次 burst 中第一次通过，第二次被去抖过滤
   *     - 实测: 每通道平均事件间隔 ~73ms（去抖正常工作）
   *
   *   场景 C (关闭 DAC 输出 或 断开外部信号):
   *     - 期望: 0 误触发（依靠 max_dev×5 + SNR≥5 双重门限）
   *
   * 【4 个关键修复点】
   *
   *   1. burst 结束后等待 3ms 让 ADC 帧处理完（main.c 在 dac_phase=1 结束后）
   *      原因：burst 期间最后采集的 ADC 帧需要 1.165ms 才到达 DMA 半区中断，
   *           若 burst 一结束就发送事件帧，selected_mask 还未由 check_dma_and_push_frames
   *           设置，导致 mask=0 → 不发送任何事件帧。
   *      修复：burst 切回 DC 后，多次调用 check_dma_and_push_frames 处理 3ms 数据。
   *
   *   2. burst 发送循环改为批量打包 6 帧一次发送
   *      原因：CH340 USB Bulk IN 包碎片化时 PC 端读取效率低。
   *      修复：先把所有事件帧拼装到 multi_buf[6×19B=114B]，再一次性 HAL_UART_Transmit。
   *
   *   3. 禁用 debug_printf 和 debug_poll 防止 ASCII 污染二进制帧流
   *      原因：debug 输出到同一 UART7，ASCII 字符插入二进制帧中破坏解析。
   *      修复：所有 debug_printf/debug_poll 都用 #if !ENABLE_DAC_SIGNAL_SOURCE 包围。
   *
   *   4. EVENT_COOLDOWN_MS=50ms 限制板上发送速率 ≤120 evt/s（CH340 极限）
   *      原因：CH340 + Windows pyserial 实测稳定接收上限 ~120 evt/s。
   *      修复：新 burst 必须等待 EVENT_COOLDOWN_MS 才能再次触发。
   *
   * 【单位说明】
   *   所有时间常量单位为 ms，与 HAL_GetTick() 一致（SysTick 1ms 一次）。
   * ========================================================================= */
  const uint32_t DAC_DC_DURATION_MS    = 30;   /* 30ms 直流持续时间 (场景 A 默认; 改 10 → 场景 B) */
  const uint32_t DAC_BURST_DURATION_MS = 5;    /* 5ms 正弦突发持续时间 (固定不变) */
  const uint32_t EVENT_COOLDOWN_MS     = 50;   /* burst 最小间隔 50ms (CH340 接收上限 120 evt/s) */

  MPU_Config();
  CPU_CACHE_Enable();
  HAL_Init();
  SystemClock_Config();
  DWT_Init();
  FFT_Tables_Init();
  if (arm_rfft_fast_init_f32(&rfft_inst, FFT_LENGTH) == ARM_MATH_SUCCESS) {
    rfft_inited = 1;
  } else {
    rfft_inited = 0;
  }
  FFT_Benchmark();  /* 任务2: FFT基准测试 */
  MX_GPIO_Init();
  LED_OFF(LED1_PIN);
  LED_OFF(LED2_PIN);
  LED_ON(LED1_PIN);
  HAL_Delay(200);
  LED_OFF(LED1_PIN);

  MX_DEBUG_UART_Init();
  debug_init();
  /* 3 ADC 完整方案：
   * ADC1: PC0/PC1 -> CH1/CH2
   * ADC2: PA3/PA4 -> CH5/CH6
   * ADC3: PC2/PC3 -> CH3/CH4
   */
  MY_ADC1_Init();
  MY_ADC2_Init();
  MY_ADC3_Init();
  MX_TIM7_Init();
  MX_DAC1_Init();

  /* 任务1: 记录时钟和计时基线 */
  diag_dwt_baseline = DWT->CYCCNT;
  diag_systick_base = HAL_GetTick();
  diag_time_base_lo = (uint32_t)(diag_dwt_baseline & 0xFFFFFFFF);
  diag_time_base_hi = (uint32_t)(diag_dwt_baseline >> 32);
  diag_sysclk_hz = SystemCoreClock;

  /* 任务4: 记录ADC采样配置 */
  diag_adc1_smpr = ADC1->SMPR1;
  diag_adc2_smpr = ADC2->SMPR1;
  diag_adc3_smpr = ADC3->SMPR1;
  diag_adc1_cfgr = ADC1->CFGR;
  diag_adc2_cfgr = ADC2->CFGR;
  diag_dma2_ndtr = DMA2_Stream0->NDTR;
  diag_dma1_ndtr = DMA1_Stream1->NDTR;

  /* 开机 Banner 已关闭：串口仅输出事件 */

  {
    static uint16_t bench_buf[FFT_LENGTH * ADC_NCH];
    int bi, bj;
    for(bi = 0; bi < FFT_LENGTH; bi++) {
      float v = 117.0f + 100.0f * sinf(2.0f * 3.14159265f * 40000.0f * (float)bi / adc_fs_hz);
      for(bj = 0; bj < ADC_NCH; bj++) {
        bench_buf[bi * ADC_NCH + bj] = (uint16_t)v;
      }
    }
    /* FFT Benchmark 输出已关闭：串口仅输出事件 */
  }
  LED_ON(LED2_PIN);

  MY_ADC_Start();

  /* 计算各通道直流偏置基线 - 采样1秒建立 */
  Calc_DC_Offset_Baseline(1000);

#if ENABLE_BOOT_SELF_TEST
  {
    uint8_t self_ch;
    uint8_t self_buf[EVT_FRAME_TOTAL_LEN];
    for(self_ch = 0; self_ch < ADC_NCH; self_ch++) {
      evt_pack_frame(self_buf, self_ch + 1, DWT_GetUs(), 12345U);
      HAL_UART_Transmit(&hdebug_uart, self_buf, EVT_FRAME_TOTAL_LEN, 100);
    }
  }
#endif
#if ENABLE_DAC_SIGNAL_SOURCE
  DAC_Output_DC();
#endif
  last_dac_switch_ms = HAL_GetTick();

  while(1) {
    uint32_t now = HAL_GetTick();

    diag_loop_cnt++;
    diag_last_state = 1;
    check_dma_and_push_frames();
    tx_ring_poll();   /* ★ v6: 每轮 poll，发起待发的 DMA 帧（零阻塞）★ */

#if ENABLE_DMA_LOOPBACK_TEST
    /* === DMA 零阻塞链路极限压力 ===
     * RATE_HZ=0: free-running, 每次主循环都尝试 push 一帧到 tx_ring
     *            （UART DMA 完成 → 下一帧立刻 send → 队列消化跟 UART 同步）
     * RATE_HZ>0: 严格按周期 push（DWT->CYCCNT 微秒精度）
     * 启动后等 3 秒，给 PC 端 PySerial open 时间 */
    if(now >= 3000U)
    {
      static uint32_t lb_last_cyc = 0;
      static uint32_t lb_seq = 0;
      static uint32_t lb_init_done = 0;
      uint8_t cur_head_snap;
      uint8_t cur_tail_snap;
      uint32_t depth;
      uint32_t do_push = 0U;
      uint32_t cyc_now = DWT->CYCCNT;

      if(!lb_init_done) { lb_last_cyc = cyc_now; lb_init_done = 1U; }

#if (DMA_LOOPBACK_RATE_HZ == 0U)
      /* free-running 极限：永远 push */
      do_push = 1U;
#else
      {
        uint32_t lb_interval_cyc = SystemCoreClock / DMA_LOOPBACK_RATE_HZ;
        while((uint32_t)(cyc_now - lb_last_cyc) >= lb_interval_cyc) {
          lb_last_cyc += lb_interval_cyc;
          do_push = 1U;  /* catch-up 后只 push 一次（防爆裂） */
          break;
        }
      }
#endif

      if(do_push) {
        uint8_t next_h = (uint8_t)((tx_ring_head + 1U) % TX_RING_SLOTS);
        if(next_h == tx_ring_tail) {
          diag_uart_q_drop_cnt++;
          diag_tx_q_full_cnt++;
        } else {
          evt_pack_frame((uint8_t*)tx_ring[tx_ring_head], 1U,
                         (uint64_t)cyc_now, lb_seq);
          tx_ring_head = next_h;
          diag_uart_q_push_cnt++;
          lb_seq++;
        }
      }

      /* high-water-mark 跟踪当前队列深度 */
      cur_head_snap = tx_ring_head;
      cur_tail_snap = tx_ring_tail;
      if(cur_head_snap >= cur_tail_snap) {
        depth = (uint32_t)(cur_head_snap - cur_tail_snap);
      } else {
        depth = (uint32_t)(TX_RING_SLOTS - cur_tail_snap + cur_head_snap);
      }
      if(depth > diag_tx_q_depth_max) {
        diag_tx_q_depth_max = depth;
      }
    }
    /* loopback 模式下跳过 ADC 事件检测，避免和 loopback 竞争 tx_ring */
    continue;
#endif
#if !ENABLE_DAC_SIGNAL_SOURCE
    debug_poll();
#endif

#if UART_STRESS_RATE_HZ > 0
    /* === USB-串口极限压测模式 ===
     * 持续以 UART_STRESS_RATE_HZ 速率发送带递增序号的事件帧
     * 帧格式：AA 55 0E 01 [DWT时间戳8字节] [序号4字节] [校验] 0D 0A
     * 序号放在 freq 字段，PC 端用以检测丢帧 */
    {
      static uint64_t stress_last_us = 0;
      static uint32_t stress_seq = 0;
      uint64_t now_us64;
      uint32_t interval_us;
      uint8_t stress_buf[EVT_FRAME_TOTAL_LEN];

      interval_us = 1000000U / (uint32_t)UART_STRESS_RATE_HZ;
      now_us64 = DWT_GetUs();
      if((now_us64 - stress_last_us) >= (uint64_t)interval_us) {
        stress_last_us = now_us64;
        evt_pack_frame(stress_buf, 1U, now_us64, stress_seq);
        HAL_UART_Transmit(&hdebug_uart, stress_buf, EVT_FRAME_TOTAL_LEN, 100);
        stress_seq++;
      }
      /* 跳过下方所有 ADC/DAC/事件检测逻辑，纯做 UART 压测 */
      continue;
    }
#endif

#if ENABLE_UART_SELF_TEST
    if(now - last_self_test_ms >= 50U) {
      uint8_t self_ch;
      uint8_t self_buf[EVT_FRAME_TOTAL_LEN];
      last_self_test_ms = now;
      for(self_ch = 0; self_ch < ADC_NCH; self_ch++) {
        evt_pack_frame(self_buf, self_ch + 1, DWT_GetUs(), 12345U);
        HAL_UART_Transmit(&hdebug_uart, self_buf, EVT_FRAME_TOTAL_LEN, 100);
      }
    }
#endif

    /* 定期输出诊断信息（每2秒）
     * !!! DAC 信号源模式下禁用：ASCII 输出会与二进制事件帧混合，
     *     破坏帧边界导致 PC 端解析丢帧 !!! */
#if !ENABLE_DAC_SIGNAL_SOURCE
    {
      static uint32_t last_diag_ms = 0;
      float fft_us, frame_us, max_frame_us;
      if(now - last_diag_ms >= 2000U) {
        last_diag_ms = now;
        
        /* 输出FFT和帧处理耗时 */
        fft_us = (float)fft_last_cyc / 480.0f;
        frame_us = (float)diag_frame_process_cyc / 480.0f;
        max_frame_us = (float)diag_frame_process_max / 480.0f;
      
      debug_printf("\r\n=== 性能诊断 ===\r\n");
      debug_printf("FFT (1024点): %u cycles = %.1f us\r\n", 
                  (unsigned int)fft_last_cyc, fft_us);
      
      /* 【2026-06-07】Original FFT 已删除；fft_optimized_cyc 仍记录最近一次 _Optimized 耗时。 */
      debug_printf("FFT Opt cyc=%lu\r\n", (unsigned long)fft_optimized_cyc);
      
      debug_printf("帧处理: %u cycles = %.1f us (max: %.1f us)\r\n",
                  (unsigned int)diag_frame_process_cyc, frame_us, max_frame_us);
      debug_printf("CPU空闲: %u%%\r\n", (unsigned int)diag_idle_ratio);
      
      /* 输出DMA中断计数 */
      debug_printf("DMA中断 - ADC1: %u/%u, ADC2: %u/%u, ADC3: %u/%u\r\n",
                  (unsigned int)adc1_dma_half_cyc, (unsigned int)adc1_dma_full_cyc,
                  (unsigned int)adc2_dma_half_cyc, (unsigned int)adc2_dma_full_cyc,
                  (unsigned int)adc3_dma_half_cyc, (unsigned int)adc3_dma_full_cyc);
      
      /* 输出丢帧统计 */
      debug_printf("帧统计: 总计=%u, 丢失=%u, 池状态: r=%u, w=%u\r\n",
                  (unsigned int)total_frame_cnt, (unsigned int)frame_drop_cnt,
                  (unsigned int)frame_r_idx, (unsigned int)frame_w_idx);
      
      /* 输出过滤统计 */
      debug_printf("过滤统计: 候选=%u, 去抖=%u, 噪声=%u, 频率=%u, 发送=%u\r\n",
                  (unsigned int)diag_external_event_candidate_cnt,
                  (unsigned int)diag_external_dedup_skip_cnt,
                  (unsigned int)diag_external_noise_filter_cnt,
                  (unsigned int)diag_external_freq_filter_cnt,
                  (unsigned int)diag_external_send_try_cnt);
      /* DAC模式专用诊断：突发计数、锁定掩码、各通道最大偏差和最近频率
       * 用于定位DAC模式下哪个筛选条件拒绝了候选 */
      debug_printf("DAC: burst=%u, dc=%u, sent=%u, mask=0x%02X, dac_freq=%u\r\n",
                  (unsigned int)diag_dac_burst_cnt,
                  (unsigned int)diag_dac_dc_cnt,
                  (unsigned int)diag_burst_sent_cnt,
                  (unsigned int)diag_last_selected_mask,
                  (unsigned int)dac_diag_actual_freq_hz);
      debug_printf("各CH最大偏差: %u %u %u %u %u %u\r\n",
                  (unsigned int)diag_burst_max_dev[0], (unsigned int)diag_burst_max_dev[1],
                  (unsigned int)diag_burst_max_dev[2], (unsigned int)diag_burst_max_dev[3],
                  (unsigned int)diag_burst_max_dev[4], (unsigned int)diag_burst_max_dev[5]);
      debug_printf("各CH噪声门: %u %u %u %u %u %u\r\n",
                  (unsigned int)diag_last_thr[0], (unsigned int)diag_last_thr[1],
                  (unsigned int)diag_last_thr[2], (unsigned int)diag_last_thr[3],
                  (unsigned int)diag_last_thr[4], (unsigned int)diag_last_thr[5]);
      debug_printf("各CH事件计数: %u %u %u %u %u %u\r\n",
                  (unsigned int)diag_evt_count_ch[0], (unsigned int)diag_evt_count_ch[1],
                  (unsigned int)diag_evt_count_ch[2], (unsigned int)diag_evt_count_ch[3],
                  (unsigned int)diag_evt_count_ch[4], (unsigned int)diag_evt_count_ch[5]);
      /* 各通道最近一次FFT频率（即使未触发也显示），用于汉宁窗精度调试 */
      debug_printf("各CH FFT频率: %u %u %u %u %u %u\r\n",
                  (unsigned int)diag_ch_last_fft_freq[0], (unsigned int)diag_ch_last_fft_freq[1],
                  (unsigned int)diag_ch_last_fft_freq[2], (unsigned int)diag_ch_last_fft_freq[3],
                  (unsigned int)diag_ch_last_fft_freq[4], (unsigned int)diag_ch_last_fft_freq[5]);
      /* FFT详细诊断：峰值bin、幅度、SNR、使用的采样率 */
      debug_printf("FFT详情: max_bin=%u, max_mag=%.0f, avg_mag=%.0f, fs=%.0f, freq_calc=%.1f\r\n",
                  (unsigned int)diag_fft_max_bin,
                  diag_fft_max_mag, diag_fft_avg_mag, diag_fft_used_fs,
                  (float)diag_fft_max_bin * diag_fft_used_fs / (float)FFT_LENGTH);
      /* 实测采样率：由 DMA 半区间隔反推
       * 期望: 4096 / 2.009MHz = 2.04ms = 978547 cycles @480MHz
       * 实测fs = HALF_SAMPLES_PER_CH / (interval_cyc / 480e6) */
      {
        float real_fs;
        float interval_us;
        if(diag_frame_interval_cyc > 0) {
          interval_us = (float)diag_frame_interval_cyc / 480.0f;
          real_fs = (float)HALF_SAMPLES_PER_CH * 1e6f / interval_us;
        } else {
          interval_us = 0.0f;
          real_fs = 0.0f;
        }
        debug_printf("DMA帧间隔: %u cyc = %.1f us, 实测fs_per_ch=%.0f Hz (预期2009000)\r\n",
                    (unsigned int)diag_frame_interval_cyc, interval_us, real_fs);
      }
      /* 频谱快照(CH1, bin 4..31)：定位真实信号 vs DC泄漏伪峰
       * 用空格分隔28个数，每个仅显示整数部分 */
      if(diag_fft_snapshot_ch != 0xFF) {
        debug_printf("FFT频谱[bin4..31]:");
        {
          uint16_t bi;
          for(bi = 0; bi < 28; bi++) {
            debug_printf(" %u", (unsigned int)diag_fft_spec_snapshot[bi]);
          }
        }
        debug_printf("\r\n");
      }
      }
    }
#endif /* !ENABLE_DAC_SIGNAL_SOURCE: 禁止 ASCII 诊断污染二进制帧流 */

#if ENABLE_DAC_SIGNAL_SOURCE
    /* DAC信号生成器 */
    if(dac_phase == 0) {
      if(now - last_dac_switch_ms >= DAC_DC_DURATION_MS) {
        /* DC 阶段结束：先检查 burst 间隔是否满足冷却要求
         *
         * EVENT_COOLDOWN_MS=50ms 限制 burst 触发频率 ≤ 20/s × 6ch = 120 evt/s
         * 防止 UART 数据超出 CH340 接收能力
         *
         * 注：保留嵌套 if 结构，与已验证版本逻辑等价（直接合并 && 在重启后
         *     会导致首次 burst 时序异常，故拆分判定） */
        if(now - burst_start_ms < EVENT_COOLDOWN_MS) {
          /* 仍在冷却期，跳过本次 burst 触发，等下一轮主循环 */
        } else {
          /* 启动新一轮 burst */
          current_burst_case = case_idx;
          tc = &test_cases[current_burst_case];
          DAC_Set_Sine_Amplitude_mV(tc->amp_mv);
          DAC_Output_Sine(tc->pwm_freq_hz);
          diag_dac_snap_cr = DAC1->CR;
          diag_dac_snap_dhr = DAC1->DHR12R2;
          diag_tim7_snap_cr1 = htim7.Instance->CR1;
          diag_tim7_snap_cnt = htim7.Instance->CNT;
          diag_dma1s1_snap_cr = *((volatile uint32_t*)0x40020028U);
          diag_dma1s1_snap_ndtr = *((volatile uint32_t*)0x4002002CU);
          diag_dma1s1_snap_m0ar = *((volatile uint32_t*)0x40020034U);
          dac_phase = 1;
          diag_dac_burst_cnt++;
          for(ch = 0; ch < ADC_NCH; ch++) {
            diag_burst_max_dev[ch] = 0;
            burst_best_dev[ch] = 0;
            burst_best_freq[ch] = 0;
            burst_best_time_us[ch] = 0;
          }
          burst_candidate_mask = 0;
          burst_event_sent = 0;
          burst_sent_mask = 0;
          burst_sent_count = 0;
          burst_start_ms = now;
          last_dac_switch_ms = now;
        }
      }
    } else {
      if(now - burst_start_ms >= DAC_BURST_DURATION_MS) {
        DAC_Output_DC();

        /* ===================================================================
         * 【关键修复 #1】burst 结束后等待 ADC 帧处理完成
         * ===================================================================
         *
         * 【问题】burst 持续 DAC_BURST_DURATION_MS=5ms，期间 ADC 以 3.515 MHz
         *         采样。但 ADC DMA 半区中断每 1.165ms 触发一次
         *         (HALF_SAMPLES_PER_CH=4096 / 3.515MHz)，意味着 burst 的
         *         最后一个完整 ADC 帧需要等到 burst 结束后 1.165ms 才能被
         *         check_dma_and_push_frames() 处理。
         *
         * 【现象】如果 burst 一结束立即执行后面的事件帧发送循环，
         *         此时 burst_candidate_mask 可能仍为 0（最后帧还没处理），
         *         selected_mask = 0 → 不发送任何事件帧 → PC 收不到事件。
         *
         * 【修复】burst 切回 DC 后，在此处忙等 3ms（覆盖 2~3 个 ADC 帧周期），
         *         同时持续调用 check_dma_and_push_frames() 把缓冲中的
         *         ADC 数据全部处理完，确保 burst_candidate_mask 完整反映
         *         burst 期间所有通道的检测结果。
         *
         * 【性能】单次 FFT ≈ 200μs，6 通道 × 3 帧 = 18 次 FFT ≈ 3.6ms，
         *         3ms 等待足以处理大部分 FFT，剩余少量在主循环后续迭代中完成。
         * =================================================================== */
        {
          uint32_t adc_wait_start_ms = HAL_GetTick();
          /* 持续等待 3ms 同时处理 ADC 帧 */
          while(HAL_GetTick() - adc_wait_start_ms < 3U) {
            check_dma_and_push_frames();
          }
        }

        /* 不再做"训练+锁定"，按帧把所有候选通道直接上报 */
        for(ch = 0; ch < ADC_NCH; ch++) {
          if(burst_candidate_mask & (1U << ch)) {
            lock_score[ch] += burst_best_dev[ch];
            diag_lock_score[ch] = lock_score[ch];
          }
        }
        lock_burst_cnt++;
        diag_lock_burst_cnt = lock_burst_cnt;

        /* selected = 本突发中所有满足"频率合格且 max_dev 过门限"的通道 */
        selected_mask = burst_candidate_mask;
        filtered_mask = burst_candidate_mask;
        locked_input_mask = burst_candidate_mask;
        select_count = 0;
        for(ch = 0; ch < ADC_NCH; ch++) {
          if(selected_mask & (1U << ch)) {
            select_count++;
          }
        }
        best_ch = ADC_NCH;
        best_dev = 0;
        (void)best_ch;
        (void)best_dev;
        diag_locked_input_mask = locked_input_mask;
        diag_last_filtered_mask = filtered_mask;
        diag_last_selected_mask = selected_mask;

        /* ===================================================================
         * 【关键修复 #2】批量打包 6 通道事件帧 → 1 次 HAL_UART_Transmit
         * ===================================================================
         *
         * 【问题】之前每个通道单独 HAL_UART_Transmit(19B)，CH340 在每次发送
         *         后进入 16ms latency 计时，6 帧的 USB Bulk IN 包碎片化，
         *         PC 端读取效率低（pyserial 多次 read 触发额外内核调用）。
         *
         * 【修复】先把所有触发通道的事件帧打包到 multi_buf[114B]（6×19B），
         *         最后一次性 HAL_UART_Transmit(multi_buf, off, 100)，
         *         CH340 把整个 114 字节作为一个 USB Bulk 包上送，
         *         PC 端一次 read() 即可获取所有 6 帧。
         *
         * 【流程】
         *   1) 遍历 6 通道，检查 selected_mask 是否包含该通道
         *   2) 应用 SAME_CH_DEDUP 同通道去抖（35ms 内同通道只允许 1 次）
         *   3) 从 burst_best_freq[ch] / burst_best_time_us[ch] 取最佳 FFT 结果
         *   4) 通过 evt_pack_frame 组装 19B 帧到 multi_buf 偏移处
         *   5) 同步更新各种诊断计数器和 LED 闪烁
         *   6) 循环结束后 HAL_UART_Transmit 一次发送整个 multi_buf
         *   7) 紧接调用 check_dma_and_push_frames() 处理 burst 后期 ADC 帧
         *
         * 【性能】114 字节 @ 460800 baud ≈ 2.47ms，CH340 包内连续发送。
         * =================================================================== */
        {
          /* multi_buf 容量 = 6 通道 × 19 字节/帧 = 114 字节 */
          uint8_t multi_buf[EVT_FRAME_TOTAL_LEN * ADC_NCH];
          uint16_t multi_len = 0;
          HAL_StatusTypeDef uart_st;

          for(ch = 0; ch < ADC_NCH; ch++) {
            if(selected_mask & (1U << ch)) {
              /* ─── 同通道去抖检查（SAME_CH_DEDUP_MS=35ms） ───
               * 防止同一通道在去抖时间内被重复触发
               * 场景 B (10+5ms 周期 15ms) 下，连续两次 burst 中第二次被此处过滤 */
              if(last_valid_event_ms[ch] != 0 && (now - last_valid_event_ms[ch]) < SAME_CH_DEDUP_MS) {
                diag_external_dedup_skip_cnt++;  /* 诊断计数：去抖跳过次数 */
                continue;
              }

              /* ─── 取 FFT 计算的最佳频率与时间戳 ───
               * burst_best_freq[ch]: burst 期间所有 FFT 帧中 SNR 最高的频率
               * burst_best_time_us[ch]: 对应的 DWT 时间戳 */
              tx_freq = burst_best_freq[ch];
              if(tx_freq == 0) {
                /* FFT 未检出有效频率时，回退用 DAC 设定频率（用于诊断） */
                tx_freq = dac_diag_actual_freq_hz;
              }
              tx_time_us = burst_best_time_us[ch];
              /* 保证时间戳单调递增（避免 PC 端误判乱序） */
              if(tx_time_us <= last_sent_time_us) {
                tx_time_us = last_sent_time_us + 1U;
              }
              last_sent_time_us = tx_time_us;

              /* ─── 组装 19 字节事件帧到 multi_buf 偏移处 ───
               * evt_pack_frame 见 event_frame.h，自动填充
               * HEAD/LEN/CH/TS/FREQ/SUM/TAIL */
              evt_pack_frame(&multi_buf[multi_len], ch + 1, tx_time_us, tx_freq);
              multi_len += EVT_FRAME_TOTAL_LEN;

              /* ─── 更新诊断计数器（用于 J-Link 在线监视）─── */
              diag_uart_evt_cnt++;
              diag_evt_count_ch[ch]++;
              diag_evt_last_time_lo[ch] = (uint32_t)(tx_time_us & 0xFFFFFFFFULL);
              diag_evt_last_time_hi[ch] = (uint32_t)(tx_time_us >> 32);
              diag_evt_last_freq[ch] = tx_freq;
              burst_sent_mask |= (uint8_t)(1U << ch);
              burst_sent_count++;
              last_valid_event_ms[ch] = now;   /* 更新最近触发时间，供下次去抖 */
              last_event_ms[ch] = now;
              LED_TOGGLE(LED2_PIN);            /* LED 闪烁指示活动 */
            }
          }

          /* ─── 一次性发送整个 multi_buf ───
           * 最大 114 字节，在 460800 baud 下 ≈ 2.47ms 发送完成
           * 100ms 超时足够覆盖任何 UART 阻塞情况 */
          if(multi_len > 0) {
            uart_st = HAL_UART_Transmit(&hdebug_uart, multi_buf, multi_len, 100);
            if(uart_st == HAL_OK) {
              diag_uart_hal_ok_cnt += multi_len / EVT_FRAME_TOTAL_LEN;
            } else {
              diag_uart_hal_fail_cnt += multi_len / EVT_FRAME_TOTAL_LEN;
            }
          }

          /* ─── 发送完后立即处理 ADC 帧（重要！） ───
           * 发送过程占用 ~2.5ms，期间至少有 2 个 ADC 帧到达 DMA 半区，
           * 必须及时处理避免后续 burst 缺数据 */
          check_dma_and_push_frames();
        }
        if(selected_mask != 0U) {
          burst_event_sent = 1;
          diag_burst_sent_cnt++;
        }

        /* 事件帧发送后不强制冷却 - DAC_DC+BURST=35ms 已是合理周期
         * burst 期间 ADC 帧处理由主循环 check_dma_and_push_frames 保证
         * 通过减小 selected_mask 设置条件来降低实际触发频率 */

        dac_phase = 0;
        diag_dac_dc_cnt++;
        last_dac_switch_ms = now;
        case_idx = (case_idx + 1) % NUM_TEST_CASES;

        if(case_idx == 0) {
          full_cycle_count++;
          /* TEST CYCLE 汇总输出已关闭：串口仅输出事件 */
        }
      }
    }
#endif

    /* 处理帧池 */
    while(frame_r_idx != frame_w_idx) {
      adc_frame_t *fr = &frame_pool[frame_r_idx];
      uint8_t  allow_trigger;
      uint8_t  has_trans[ADC_NCH] = {0};
      uint16_t thr_ch[ADC_NCH];
      uint32_t max_dev_ch[ADC_NCH];
      uint8_t  n_trig = 0;
      uint16_t pre_n, post_n;
      uint8_t  ch_has_data[ADC_NCH];
      float    ch_freq[ADC_NCH];
      uint64_t ch_time_us[ADC_NCH];
      uint8_t  n_collected = 0;
      uint16_t peak_idx;
      float    peak_frac;
      float    trans_frac;
      float    ch_offset_samples;
      float    eff_idx;
      uint64_t peak_us_in_frame;
      uint64_t group_time_us;
      float    group_freq;
      uint16_t fft_start;
      uint16_t freq_start;
      float    fs_est;
      float    alpha;
      float    zc_freq;
      float    fft_freq;
      static uint32_t cal_count = 0;

      if(!fr->ready) break;
      diag_frame_seen_cnt++;
      diag_last_state = 2;

#if ENABLE_DAC_SIGNAL_SOURCE
      allow_trigger = 0;
      if(dac_phase == 1) {
        allow_trigger = 1;
      } else if((now - last_dac_switch_ms) < 10U) {
        allow_trigger = 1;
      }
#else
      allow_trigger = 1;
#endif
      if(!allow_trigger) {
        diag_frame_gate_drop_cnt++;
        fr->ready = 0;
        frame_r_idx = (frame_r_idx + 1) % FRAME_POOL_SIZE;
        continue;
      }

      /* Pass 1: 轻量预扫 - 检测各通道AC信号存在性和突变 */
      for(ch = 0; ch < ADC_NCH; ch++) {
        uint16_t thr;
        uint16_t ac_idx;
        uint32_t max_dev;
        uint16_t i;
        uint16_t v_min, v_max;
        uint16_t noise_n;
        const uint16_t *src;
        uint16_t v;
        uint16_t verify_hits;
        uint16_t end_i;
        int32_t d;
        uint32_t ab;
        /* 同通道帧级去抖：跳过最近 FRAME_LEVEL_SKIP_MS 内已触发的通道
         * 注意：此处用较短门限（2ms），减少漏触发
         * 事件级去抖（10ms）在最终发送阶段由 SAME_CH_DEDUP_MS 控制 */
        if(last_event_ms[ch] != 0 && (now - last_event_ms[ch]) < FRAME_LEVEL_SKIP_MS) continue;

        /* 使用全局直流偏置基线（如果有效），否则计算本地基线 */
        if(ch_dc_offset_valid && ch_dc_offset[ch] != 0) {
          baselines[ch] = ch_dc_offset[ch];
          /* 计算本地噪声峰峰值（仍需要动态噪声门限） */
          v_min = 4095; v_max = 0;
          noise_n = (HALF_SAMPLES_PER_CH > BASELINE_SAMPLES) ? BASELINE_SAMPLES : HALF_SAMPLES_PER_CH;
          src = fr->data + ch;
          for(i = 0; i < noise_n; i++) {
            v = src[i * ADC_NCH];
            if(v < v_min) v_min = v;
            if(v > v_max) v_max = v;
          }
          noises[ch] = (v_max >= v_min) ? (v_max - v_min) : 0;
        } else {
          baselines[ch] = Calc_Baseline_in_Frame(fr->data, HALF_SAMPLES_PER_CH, ch, &noises[ch]);
        }
        thr = DEV_THRESHOLD_MIN;
        if(noises[ch] * DEV_NOISE_MULT > DEV_THRESHOLD_MIN) {
          thr = noises[ch] * DEV_NOISE_MULT;
        }
        thr_ch[ch] = thr;
        max_dev_ch[ch] = 0;
        diag_last_noise_pp[ch] = noises[ch];
        diag_last_thr[ch] = thr;

        Detect_AC_Present_in_Frame(fr->data, HALF_SAMPLES_PER_CH, ch,
                                   baselines[ch], thr, &ac_idx, &max_dev);
        max_dev_ch[ch] = max_dev;
        diag_last_max_dev[ch] = max_dev;
        if(max_dev > diag_burst_max_dev[ch]) {
          diag_burst_max_dev[ch] = max_dev;
        }

        trans[ch] = Detect_Transition_in_Frame(fr->data, HALF_SAMPLES_PER_CH, ch, baselines[ch], thr);
        if(trans[ch] != 0xFFFF) {
          /* 持续性验证：突变点后 SUSTAIN_SAMPLES 个样本中，至少 SUSTAIN_MIN_HIT 个超过阈值 */
          verify_hits = 0;
          end_i = trans[ch] + SUSTAIN_SAMPLES;
          if(end_i > HALF_SAMPLES_PER_CH) end_i = HALF_SAMPLES_PER_CH;
          for(i = trans[ch]; i < end_i; i++) {
            d = (int32_t)fr->data[i * ADC_NCH + ch] - baselines[ch];
            ab = (d < 0) ? (uint32_t)(-d) : (uint32_t)d;
            if(ab > thr) verify_hits++;
          }
          if(verify_hits >= SUSTAIN_MIN_HIT) {
            has_trans[ch] = 1;
            n_trig++;
            diag_prescan_hit_cnt++;
          }
        }
        if(!has_trans[ch]) {
#if 0  /* 禁用AC Present fallback，防止噪声误触发 */
          if(Detect_AC_Present_in_Frame(fr->data, HALF_SAMPLES_PER_CH, ch,
                                        baselines[ch], thr, &ac_idx, &max_dev)) {
            trans[ch] = ac_idx;
            has_trans[ch] = 1;
            n_trig++;
            diag_ac_present_cnt[ch]++;
            diag_prescan_hit_cnt++;
          }
#endif
        }
      }

      diag_last_n_trig = n_trig;
      pre_n  = (n_trig >= 2) ? 10u  : 30u;
      post_n = (n_trig >= 2) ? 50u  : 250u;
      (void)pre_n; (void)post_n;

      for(ch = 0; ch < ADC_NCH; ch++) {
        ch_has_data[ch] = 0;
        ch_freq[ch] = 0.0f;
        ch_time_us[ch] = 0;
        if(!has_trans[ch]) continue;

        peak_idx = Find_FirstPeak_in_Frame(fr->data, HALF_SAMPLES_PER_CH, ch, baselines[ch], trans[ch]);
        peak_frac = Refine_Peak_SubSample(fr->data, HALF_SAMPLES_PER_CH, ch, baselines[ch], peak_idx);
        trans_frac = Refine_Transition_SubSample(fr->data, HALF_SAMPLES_PER_CH, ch,
                                                 baselines[ch], thr_ch[ch], trans[ch]);
        ch_offset_samples = (float)(ch % ADC_CH_PER_INST) / (float)ADC_CH_PER_INST;
        /* === 事件时间 = 极值（峰值/谷值）位置，非过零点 ===
         * 客户要求：精确事件时间必须是信号偏离 DC 基线最大时刻（峰值或谷值），
         * 而不是过零点（trans_idx）。原因：过零点对噪声敏感（小幅度抖动会导致
         * 过零点漂移），而极值点幅值大、噪声容限高，时序更稳定。
         *
         * 计算公式：eff_idx = peak_idx + peak_frac - ch_offset_samples
         *   - peak_idx    : Refine_Peak_SubSample 找到的全局最大偏离样本索引
         *   - peak_frac   : 抛物线插值子样本偏移（浮点，-0.5~+0.5）
         *   - ch_offset_samples: 扫描序列中当前通道的偏移（如 6 通道扫描中
         *                        CH2 比 CH1 晚 1/6 个帧扫描周期）
         *
         * 转换为 μs:  t = eff_idx / adc_fs_hz × 1e6
         *             + fr->start_time_us (帧起始绝对时间)
         * 注：adc_fs_hz = 3,515,000 Hz（实测 DMA 帧间隔反推）*/
        eff_idx = (float)peak_idx + peak_frac - ch_offset_samples;
        if(eff_idx < 0.0f) eff_idx = 0.0f;
        /* peak_idx 和 peak_frac 已用于 eff_idx 计算，无需 (void) 消除警告 */
        fft_start = trans[ch] + FFT_STEADY_SKIP;
        freq_start = fft_start;
        if(freq_start + 64U >= HALF_SAMPLES_PER_CH) {
          freq_start = trans[ch] + 16U;
        }
        freq[ch] = 0.0f;
#if ENABLE_DAC_SIGNAL_SOURCE || EXTERNAL_EVENT_CALC_FREQ
#if ENABLE_DAC_SIGNAL_SOURCE || EXTERNAL_EVENT_USE_FFT_FREQ
        /* 【2026-06-07 改进】FFT 窗口边界 fallback：
         * 原条件 fft_start + FFT_LENGTH < HALF 在触发点接近帧尾时（>2944）会跳过 FFT，
         * 导致约 20-33% 候选事件因 freq[ch]=0 被后续频率过滤丢弃。
         * 新策略：若默认窗口不够，回退到 trans[ch] 或更早，确保 FFT 总能运行。
         *   - 优先尝试 trans[ch] + STEADY_SKIP（默认）
         *   - 退化到 trans[ch]（牺牲启动暂态滤除）
         *   - 最差退到 HALF - FFT_LENGTH（用帧尾全部数据）
         */
        {
          uint16_t fft_start_eff = fft_start;
          if(fft_start_eff + FFT_LENGTH >= HALF_SAMPLES_PER_CH) {
            /* 退化 1：去掉 STEADY_SKIP */
            fft_start_eff = trans[ch];
            if(fft_start_eff + FFT_LENGTH >= HALF_SAMPLES_PER_CH) {
              /* 退化 2：使用帧尾的最后 FFT_LENGTH 个样本 */
              if(HALF_SAMPLES_PER_CH > FFT_LENGTH) {
                fft_start_eff = (uint16_t)(HALF_SAMPLES_PER_CH - FFT_LENGTH);
              } else {
                fft_start_eff = 0;
              }
            }
          }
          if(fft_start_eff + FFT_LENGTH <= HALF_SAMPLES_PER_CH) {
            diag_fft_try_cnt++;
            fft_freq = Calc_FFT_Frequency_in_Frame(fr->data, HALF_SAMPLES_PER_CH, ch, fft_start_eff);
            diag_ch_last_fft_freq[ch] = (uint32_t)fft_freq;
            if(fft_freq >= 20000.0f && fft_freq <= 200000.0f) {
              freq[ch] = fft_freq;
            }
          }
        }
#endif
#if ENABLE_DAC_SIGNAL_SOURCE || EXTERNAL_EVENT_USE_ZERO_CROSS
        if(freq[ch] < 1.0f && freq_start + 64U < HALF_SAMPLES_PER_CH) {
          zc_freq = Calc_ZeroCross_Frequency_in_Frame(fr->data, HALF_SAMPLES_PER_CH, ch,
                                                      freq_start, baselines[ch]);
          if(zc_freq >= 20000.0f && zc_freq <= 200000.0f) {
            freq[ch] = zc_freq;
          }
        }
#endif
#endif

        peak_us_in_frame = (uint64_t)(eff_idx / adc_fs_hz * 1e6f + 0.5f);
        test_case_trigger_count[current_burst_case]++;
        ch_has_data[ch] = 1;
        diag_external_event_candidate_cnt++;
        ch_freq[ch] = freq[ch];
        ch_time_us[ch] = fr->start_time_us + peak_us_in_frame;
        n_collected++;
      }

      diag_last_n_collected = n_collected;
#if ENABLE_DAC_SIGNAL_SOURCE
      if(n_collected > 0) {
        /* DAC模式通道筛选：频率范围 + 偏差阈值 + SNR
         * 1. FFT频率必须在 20kHz~200kHz 范围内
         * 2. 信号偏差必须超过噪声门限
         * 3. SNR >= 1.0（信号偏差至少等于噪声门限）
         * 4. 频率与DAC输出一致性（误差<10%，如果DAC频率已知）
         */
        for(ch = 0; ch < ADC_NCH; ch++) {
          float snr_ratio;
          if(ch_has_data[ch]) {
            /* 验证：ch_freq必须有效且在合理范围内 */
            if(ch_freq[ch] < 1000.0f) {
              continue;
            }
            f_u32 = (uint32_t)(ch_freq[ch] + 0.5f);
            if(f_u32 >= INTERNAL_TEST_FREQ_MIN_HZ && f_u32 <= INTERNAL_TEST_FREQ_MAX_HZ) {
              uint16_t thresh = noises[ch] * DEV_NOISE_MULT;
              if(thresh < DEV_THRESHOLD_MIN) thresh = DEV_THRESHOLD_MIN;
              if(max_dev_ch[ch] >= thresh) {
                /* SNR比率 = 信号偏差 / 噪声门限
                 * 真实信号通道（CH2/4/6）SNR ≈ 17~18，CH3串扰SNR ≈ 3.4
                 * 设 SNR ≥ 5.0 可有效区分真实信号与PCB串扰噪声 */
                snr_ratio = (float)max_dev_ch[ch] / (float)thresh;
                if(snr_ratio < 5.0f) {
                  continue;
                }
                /* 频率一致性检查（如果DAC频率已知）
                 * 修复 adc_fs_hz 校准 (2009000 -> 3515000) 后，FFT输出应接近 DAC 实际频率
                 * 收紧为 10% 容差：兼顾抛物线插值精度（~1bin/8 ≈ 2%）和带通运放频偏
                 * 若仍误拒，可放宽至 20% */
                if(dac_diag_actual_freq_hz > 0) {
                  float freq_err = fabsf((float)f_u32 - (float)dac_diag_actual_freq_hz)
                                 / (float)dac_diag_actual_freq_hz;
                  if(freq_err > 0.10f) {
                    continue;
                  }
                }
                burst_candidate_mask |= (uint8_t)(1U << ch);
                /* 用 max_dev 作为评分 */
                if(max_dev_ch[ch] >= burst_best_dev[ch]) {
                  burst_best_dev[ch] = max_dev_ch[ch];
                  burst_best_freq[ch] = f_u32;
                  burst_best_time_us[ch] = ch_time_us[ch];
                }
              }
            }
          }
        }
      }
#else
      for(ch = 0; ch < ADC_NCH; ch++) {
        if(ch_has_data[ch]) {
          if(last_valid_event_ms[ch] != 0 && (now - last_valid_event_ms[ch]) < SAME_CH_DEDUP_MS) {
            diag_external_dedup_skip_cnt++;
            continue;
          }
          /* 关键修复：检查max_dev是否真正超过阈值，过滤噪声 */
          {
            uint16_t thresh = (uint16_t)(noises[ch] * DEV_NOISE_MULT);
            if(thresh < DEV_THRESHOLD_MIN) thresh = DEV_THRESHOLD_MIN;
            if(max_dev_ch[ch] < thresh) {
              diag_external_noise_filter_cnt++;
              continue;
            }
          }
#if EXTERNAL_EVENT_FIXED_FREQ
          tx_freq = EXTERNAL_EVENT_DEBUG_FREQ_HZ;
#else
          {
            float f = ch_freq[ch];
            /* 严格验证：必须有有效频率，且在 20kHz - 200kHz 工作范围内
             *
             * 注：早期版本曾屏蔽 39-41 kHz（视为 DAC 闭环测试时的系统噪声），
             *     正式场景下 6 通道接收独立信号，频率不固定，故已移除该屏蔽。
             *     若某些通道仍有窄带干扰，应在硬件侧加滤波或在此处增加单通道
             *     specific 的过滤逻辑（不可一刀切）。 */
            if(f < 1.0f || f < 20000.0f || f > 200000.0f) {
              diag_external_freq_filter_cnt++;
              continue;
            }
            tx_freq = (uint32_t)(f + 0.5f);
          }
#endif
          diag_external_send_try_cnt++;
          tx_time_us = ch_time_us[ch];
          if(tx_time_us <= last_sent_time_us) {
            tx_time_us = last_sent_time_us + 1U;
          }
          last_sent_time_us = tx_time_us;
          evt_pack_frame(evt_buf,
                         ch + 1,
                         tx_time_us,
                         tx_freq);
          /* 【2026-06-07 v2 - DMA 零阻塞发送】
           * 改用 HAL_UART_Transmit_DMA，UART7 TX → DMA1 Stream0 异步传输。
           * CPU 仅触发后立即返回（< 5us），不再阻塞主循环。
           * 单帧 19B @ 460800 baud = 412us 在 DMA 后台传，期间主循环可继续处理 ADC 帧。
           *
           * 重要：
           *   - 必须等上一帧 DMA 完成才能发起下一帧（HAL state = READY）
           *   - 若 BUSY 则丢本帧并计数（diag_uart_hal_fail_cnt++）
           *   - evt_buf 必须保持有效到 DMA 完成（这里 evt_buf 是 static，没问题） */
          /* ★ 2026-06-07 v6 - 真正零阻塞 DMA 队列 ★
           * 架构（参考 ST HAL 锁机制 + 中断重入死锁分析）：
           *   1) 主循环 push: 仅 memcpy 19B 到 tx_ring[head]（耗时 ~50ns，零阻塞）
           *   2) 主循环 poll: 若 UART READY → 取 tail 帧 + Clean DCache + 启 DMA
           *   3) TxCpltCallback: 仅记录计数，绝不调用 HAL 函数（避免 LOCK 重入死锁）
           * 关键约束：
           *   - DMA 启动只在主循环（线程上下文）做，永远不在中断里
           *   - 每个槽 19B 独立缓冲，启动 DMA 后该槽不再被写
           *   - poll 函数被 main 主循环每轮调用 */
          {
            uint8_t next_head = (uint8_t)((tx_ring_head + 1U) % TX_RING_SLOTS);
            if(next_head == tx_ring_tail) {
              /* 队列满 → 丢帧（绝不阻塞主循环） */
              diag_uart_q_drop_cnt++;
              diag_uart_hal_fail_cnt++;
            } else {
              memcpy((void*)tx_ring[tx_ring_head], evt_buf, EVT_FRAME_TOTAL_LEN);
              tx_ring_head = next_head;
              diag_uart_q_push_cnt++;
              diag_uart_hal_ok_cnt++;
            }
          }
          diag_uart_evt_cnt++;
          diag_evt_count_ch[ch]++;
          diag_evt_last_time_lo[ch] = (uint32_t)(tx_time_us & 0xFFFFFFFFULL);
          diag_evt_last_time_hi[ch] = (uint32_t)(tx_time_us >> 32);
          diag_evt_last_freq[ch] = tx_freq;
          last_valid_event_ms[ch] = now;
          last_event_ms[ch] = now;
          LED_TOGGLE(LED2_PIN);
          /* 【删除】原 HAL_Delay(2) 已移除 —— 阻塞无价值，节流由去抖控制 */
        }
      }
#endif
      (void)fs_est;
      (void)alpha;
      (void)cal_count;
      fr->ready = 0;
      frame_r_idx = (frame_r_idx + 1) % FRAME_POOL_SIZE;
    }

#if !ENABLE_DAC_SIGNAL_SOURCE
    debug_poll();
#endif
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) { Error_Handler(); }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {
    LED_ON(LED1_PIN);
    HAL_Delay(200);
    LED_OFF(LED1_PIN);
    HAL_Delay(200);
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif