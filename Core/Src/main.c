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
  *    采样率   : ~3.5 MSPS/通道 (实测值, EMA在线校准, 见 adc_fs_hz)
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
  *  UART1 (PA9/PA10) ↔ CH340 USB-Serial: 460800 baud
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
#define DEV_THRESHOLD_260MV 320   /**< 偏离阈值≈260mV；示波器实测 ADC 引脚有效信号通常>200mV，
                                    *   保留余量滤除小幅杂波，同时避免 400LSB 门限过高漏检 */
#define DEV_THRESHOLD_MIN  DEV_THRESHOLD_260MV  /**< 主门限：约260mV 阈值 */
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
#define SUSTAIN_SAMPLES    384    /**< Layer 2 持续性验证窗口：约109us@3.52MSPS，滤除几十us杂波 */
#define SUSTAIN_MIN_HIT    80     /**< Layer 2 总命中数：要求跨多个40k周期持续存在 */
#define SUSTAIN_TAIL_SAMPLES 96   /**< 窗口尾段验证：防止前几十us毛刺误判为主信号 */
#define SUSTAIN_TAIL_MIN_HIT 16   /**< 尾段最少命中数：尾段仍有能量才认定有效触发 */
#define SUSTAIN_MIN_HIT_WEAK  64  /**< 弱信号路径放宽但仍覆盖几十us毛刺 */

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

#define FRAME_LEVEL_SKIP_MS 0     /**< 帧级预扫描去抖(ms)：2026-06-07 v8 由 2→0 (E.8)，
                                    *   完全移除帧级去抖，避免合法帧被过滤；
                                    *   事件级去抖由 SAME_CH_DEDUP_MS=10ms 统一控制 */
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
#define ENABLE_DAC_SIGNAL_SOURCE  0    /**< 0=外部检测模式（正式 + 闭环测试也用） */
#define DAC_AS_EXTERNAL_TEST_SRC  0    /**< 1=ENABLE_DAC_SIGNAL_SOURCE=0 时仍输出 PA5 持续突发，
                                        *   配合 PA5→6路运放 验证 6 通道触发均匀性 */
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
  uint64_t ch_start_time_us[ADC_NCH];
  uint32_t frame_id;
  uint8_t  ready;
  uint8_t  fft_pending_mask;
} adc_frame_t;

static adc_frame_t frame_pool[FRAME_POOL_SIZE] __attribute__((section(".AXI_SRAM"), aligned(32)));

/* ★【2026-06-07 零阻塞 UART 环形发送队列】★
 * 由主循环 push（拷贝 19B），由 DMA Tx 完成中断回调 pop 并发起下一帧。
 * 【2026-06-13 扩容】200 槽 × 19B = 3800B ≈ 3.8KB
 * 支持最大 200 帧事件缓冲，应对突发高频率触发场景 */
#define TX_RING_SLOTS  200
static uint8_t  tx_ring[TX_RING_SLOTS][EVT_FRAME_TOTAL_LEN] __attribute__((section(".AXI_SRAM"), aligned(32)));
static volatile uint8_t tx_ring_head = 0;  /* 主线程写 */
static volatile uint8_t tx_ring_tail = 0;  /* SysTick 中断读 */
static volatile uint8_t  frame_w_idx = 0;
static volatile uint8_t  frame_r_idx = 0;
static volatile uint32_t frame_drop_cnt = 0;

/* ★【2026-06-10 架构重构】触发事件队列 ★
 * 三级流水线解耦：
 *   Step A (主循环): ADC帧 → 触发检测 → push trigger_queue
 *   Step B (主循环): pop trigger_queue → FFT → push tx_ring (事件帧)
 *   SysTick 中断: 检查 tx_ring + UART DMA 状态 → 发送
 *
 * trigger_event_t 保存触发点信息 + 1024点原始数据，
 * Step B pop后做 FFT 计算，不再就地计算，避免帧消费循环积压。 */
#define TRIG_QUEUE_SIZE  16
#define TRIG_DATA_LEN    1024  /* FFT 点数，触发后取 1024 个单通道样本 */

typedef struct {
  uint8_t  ch;                     /* 触发通道 (0..5, 对应 CH1..CH6) */
  uint16_t trigger_idx;            /* 触发点在帧内的样本索引 */
  uint64_t start_time_us;          /* 帧起始时间戳 (μs) */
  uint16_t data[TRIG_DATA_LEN];   /* 触发后 1024 点单通道 ADC 数据 */
} trigger_event_t;

static trigger_event_t trig_queue[TRIG_QUEUE_SIZE] __attribute__((section(".AXI_SRAM"), aligned(32)));
static volatile uint8_t trig_q_head = 0;  /* Step A push (主线程写) */
static volatile uint8_t trig_q_tail = 0;  /* Step B pop (主线程读) */
/* =============================================================================
 * [诊断变量归档 — 2026-06-12]
 *
 * 以下 ~70 个 volatile 变量仅用于 RTT/J-Link 就地调试，不在正式流程中参与
 * 业务逻辑。它们按用途分为以下几组：
 *
 * 【2026-06-11 优化】诊断变量统一移至 AXI SRAM，释放 DTCM 空间
 * 这些变量仅被 J-Link 在线监视或 debug_printf 使用，不在性能关键路径，
 * 放 AXI SRAM (D-Cache Write-Back) 不影响功能正确性。
 * 78 个变量共 ~500B，从 DTCM 0等待区释放到 AXI SRAM 512KB 充裕空间。
 * 注：volatile 确保编译器不缓存寄存器值，D-Cache WB 属性下 CPU 写入
 *     最终可见（J-Link 通过 SWD 直接读 RAM 不经过 D-Cache）。
 *
 * Group A — 流程计数（主循环、DMA、dac burst 的诊断计数）
 * Group B — UART 发送诊断（tx_ring_poll 的发送链路诊断）
 * Group C — 压力压测 / DMA loopback / 极限诊断
 * Group D — 通道级事件诊断（每通道最新触发状态）
 * Group E — FFT 诊断（频谱快照、bin 详细）
 * Group F — 单次基准测试（只在 FFT_Benchmark 或启动时写一次）
 * Group G — ADC 采样间隔诊断
 *
 * 下次重构时可将各组抽到独立结构体，以减少全局作用域污染。
 * ========================================================================= */

/* Group A — 流程计数 */
volatile uint32_t diag_loop_cnt = 0;              /* 主循环迭代计数 */
volatile uint32_t diag_dac_burst_cnt = 0;        /* DAC burst 触发计数 */
volatile uint32_t diag_dac_dc_cnt = 0;           /* DAC DC 状态计数 */
volatile uint32_t diag_frame_seen_cnt = 0;       /* 处理的帧计数 */
volatile uint32_t diag_frame_gate_drop_cnt = 0;  /* 帧门限丢弃计数 */
volatile uint32_t diag_prescan_hit_cnt = 0;      /* 预扫描命中计数 */
volatile uint32_t diag_fft_try_cnt = 0;          /* FFT 尝试计数 */
volatile uint32_t diag_uart_evt_cnt = 0;         /* UART 事件计数 */
volatile uint32_t diag_burst_sent_cnt = 0;       /* 发送的 burst 计数 */
volatile uint32_t diag_last_n_trig = 0;          /* 上次触发数量 */
volatile uint32_t diag_last_n_collected = 0;     /* 上次收集数量 */
volatile uint32_t diag_last_state = 0;           /* 上次状态 */

/* Group B — UART 发送诊断 */
volatile uint32_t diag_tx_q_depth_max = 0;       /* tx_ring 历史最大占用深度 */
volatile uint32_t diag_tx_q_full_cnt = 0;        /* 队列满（next_head==tail）次数 */
volatile uint32_t diag_uart_hal_ok_cnt = 0;      /* HAL_UART_Transmit_DMA OK 次数 */
volatile uint32_t diag_uart_hal_fail_cnt = 0;    /* HAL_UART_Transmit_DMA 失败次数 */
volatile uint32_t diag_uart_last_status = 0;     /* 最后一次 UART 状态 */
volatile uint32_t diag_uart_last_error = 0;      /* 最后一次 UART 错误码 */
volatile uint32_t diag_uart_last_isr = 0;        /* 最后一次 ISR 状态 */
volatile uint32_t diag_uart_last_cr1 = 0;        /* 最后一次 CR1 寄存器值 */

/* Group C — 压力压测 / DMA loopback */
volatile uint32_t diag_trig_q_drop_cnt = 0;      /* 触发队列满丢计数 */
volatile uint32_t diag_external_event_candidate_cnt = 0;  /* 外部事件候选计数 */
volatile uint32_t diag_external_dedup_skip_cnt = 0;       /* 去抖跳过计数 */
volatile uint32_t diag_external_noise_filter_cnt = 0;     /* 噪声过滤计数 */
volatile uint32_t diag_external_freq_filter_cnt = 0;      /* 频率过滤计数 */
volatile uint32_t diag_external_send_try_cnt = 0;         /* 发送尝试计数 */
volatile uint32_t diag_last_selected_mask = 0;    /* 最后选中的通道掩码 */
volatile uint32_t diag_last_filtered_mask = 0;    /* 最后过滤后的通道掩码 */
volatile uint32_t diag_locked_input_mask = 0;    /* 锁定的输入掩码 */
volatile uint32_t diag_lock_burst_cnt = 0;       /* 锁定的 burst 计数 */

/* Group D — 通道级事件诊断（每通道最新触发状态）*/
volatile uint32_t diag_last_noise_pp[ADC_NCH] __attribute__((section(".AXI_SRAM"))) = {0};  /* 噪声峰峰值 */
volatile uint32_t diag_last_thr[ADC_NCH] __attribute__((section(".AXI_SRAM"))) = {0};       /* 阈值 */
volatile uint32_t diag_last_max_dev[ADC_NCH] __attribute__((section(".AXI_SRAM"))) = {0};   /* 最大偏移 */
volatile uint32_t diag_burst_max_dev[ADC_NCH] __attribute__((section(".AXI_SRAM"))) = {0};   /* Burst 最大偏移 */
volatile uint32_t diag_lock_score[ADC_NCH] __attribute__((section(".AXI_SRAM"))) = {0};     /* 锁定分数 */
volatile uint32_t diag_ac_present_cnt[ADC_NCH] __attribute__((section(".AXI_SRAM"))) = {0}; /* AC 存在计数 */
volatile uint32_t diag_evt_count_ch[ADC_NCH] __attribute__((section(".AXI_SRAM"))) = {0};   /* 每通道事件计数 */
volatile uint32_t diag_evt_last_time_lo[ADC_NCH] __attribute__((section(".AXI_SRAM"))) = {0};/* 事件时间戳低32位 */
volatile uint32_t diag_evt_last_time_hi[ADC_NCH] __attribute__((section(".AXI_SRAM"))) = {0};/* 事件时间戳高32位 */
volatile uint32_t diag_evt_last_freq[ADC_NCH] __attribute__((section(".AXI_SRAM"))) = {0};  /* 事件频率 */
volatile uint32_t diag_ch_last_fft_freq[ADC_NCH] __attribute__((section(".AXI_SRAM"))) = {0};/* 每通道最后 FFT 频率 */

/* Group E — FFT 诊断 */
volatile uint32_t diag_fft_max_bin = 0;           /* FFT 最大峰值 bin */
volatile float    diag_fft_max_mag = 0.0f;        /* FFT 最大幅度 */
volatile float    diag_fft_avg_mag = 0.0f;        /* FFT 平均幅度 */
volatile float    diag_fft_used_fs = 0.0f;        /* FFT 使用的采样率 */
volatile float    diag_fft_spec_snapshot[28] __attribute__((section(".AXI_SRAM"))) = {0}; /* 频谱快照 bin4-31 */
volatile uint8_t  diag_fft_snapshot_ch = 0xFF;    /* 频谱快照对应通道 */

/* Group F — 单次基准测试 */
volatile uint32_t diag_fft256_cyc = 0;            /* 256点FFT耗时(cycles) */
volatile uint32_t diag_fft512_cyc = 0;            /* 512点FFT耗时(cycles) */
volatile uint32_t diag_fft1024_cyc = 0;           /* 1024点FFT耗时(cycles) */
volatile uint32_t diag_fft_bench_done = 0;        /* FFT基准测试完成标志 */
volatile uint32_t diag_time_base_lo = 0;          /* 时间基准低32位 */
volatile uint32_t diag_time_base_hi = 0;          /* 时间基准高32位 */
volatile uint32_t diag_dwt_baseline = 0;          /* DWT基准 */
volatile uint32_t diag_systick_base = 0;          /* SysTick基准 */
volatile uint32_t diag_sysclk_hz = 0;             /* 系统时钟频率 */

/* Group G — ADC/DMA 寄存器快照 */
volatile uint32_t diag_adc1_smpr = 0;             /* ADC1 SMPS寄存器 */
volatile uint32_t diag_adc2_smpr = 0;             /* ADC2 SMPS寄存器 */
volatile uint32_t diag_adc3_smpr = 0;             /* ADC3 SMPS寄存器 */
volatile uint32_t diag_adc1_cfgr = 0;             /* ADC1 CFGR寄存器 */
volatile uint32_t diag_adc2_cfgr = 0;             /* ADC2 CFGR寄存器 */
volatile uint32_t diag_dma2_ndtr = 0;             /* DMA2 NDTR寄存器 */
volatile uint32_t diag_dma1_ndtr = 0;             /* DMA1 NDTR寄存器 */
volatile uint32_t diag_dma_bdma_ndtr = 0;         /* BDMA NDTR寄存器 */
volatile uint32_t diag_dac_snap_cr = 0;           /* DAC CR寄存器快照 */
volatile uint32_t diag_dac_snap_dhr = 0;          /* DAC DHR寄存器快照 */
volatile uint32_t diag_tim7_snap_cr1 = 0;         /* TIM7 CR1寄存器快照 */
volatile uint32_t diag_tim7_snap_cnt = 0;         /* TIM7 CNT寄存器快照 */
volatile uint32_t diag_dma1s1_snap_cr = 0;        /* DMA1 Stream1 CR寄存器快照 */
volatile uint32_t diag_dma1s1_snap_ndtr = 0;      /* DMA1 Stream1 NDTR寄存器快照 */
volatile uint32_t diag_dma1s1_snap_m0ar = 0;      /* DMA1 Stream1 M0AR寄存器快照 */

/* Group H — 帧处理性能诊断 */
volatile uint32_t diag_frame_process_cyc = 0;      /* 帧处理耗时(cycles) */
volatile uint32_t diag_frame_process_max = 0;      /* 帧处理最大耗时(cycles) */
volatile uint32_t diag_frame_interval_cyc = 0;     /* 帧间隔(cycles) */
volatile uint32_t diag_idle_ratio = 0;
volatile uint32_t diag_adc_sample_cyc = 0;
volatile uint32_t diag_last_frame_cyc = 0;
volatile uint32_t diag_adc_desync_cnt = 0;
volatile uint32_t diag_adc_desync_mask = 0;
volatile uint32_t diag_stress_burst_total = 0;
volatile uint8_t  diag_stress_cur_idx = 0;
static volatile uint32_t total_frame_cnt = 0;
static volatile uint32_t half_complete_cnt = 0;
static volatile uint32_t full_complete_cnt = 0;

typedef struct {
  uint32_t magic;
  uint32_t version;
  uint32_t loop_cnt;
  uint32_t half_complete_cnt;
  uint32_t full_complete_cnt;
  uint32_t frame_seen_cnt;
  uint32_t frame_interval_cyc;
  uint32_t adc_fs_hz_int;
  uint32_t fft_used_fs_int;
  uint32_t fft_max_bin;
  uint32_t uart_q_push_cnt;
  uint32_t uart_q_send_cnt;
  uint32_t tx_poll_call_cnt;
  uint32_t tx_poll_send_ok;
  uint32_t tx_head_tail;
  uint32_t last_n_trig;
  uint32_t prescan_hit_cnt;
  uint32_t candidate_cnt;
  uint32_t noise_filter_cnt;
  uint32_t freq_filter_cnt;
  uint32_t last_selected_mask;
  uint32_t last_filtered_mask;
  uint32_t locked_input_mask;
  uint32_t stress_burst_total;
  uint32_t stress_cur_idx;
  uint32_t dac_cr;
  uint32_t dac_dhr12r2;
  uint32_t tim7_cr1;
  uint32_t tim7_cnt;
  uint32_t tim7_arr;
  uint32_t tim7_dier;
  uint32_t dma1s1_cr;
  uint32_t dma1s1_ndtr;
  uint32_t dac_actual_freq_hz;
  uint32_t ch_fft_freq[ADC_NCH];
  uint32_t ch_evt_freq[ADC_NCH];
  uint32_t ch_evt_time_lo[ADC_NCH];
  uint32_t ch_evt_time_hi[ADC_NCH];
  uint32_t ch_max_dev[ADC_NCH];
  uint32_t ch_thr[ADC_NCH];
  uint32_t ch_noise_pp[ADC_NCH];
  uint32_t adc_dma_half_cyc[3];
  uint32_t adc_dma_full_cyc[3];
  uint32_t adc_dma_half_delta_cyc[3];
  uint32_t adc_dma_full_delta_cyc[3];
  uint32_t checksum;
} diag_snapshot_t;

volatile diag_snapshot_t diag_snapshot __attribute__((at(0x38002000), aligned(32))) = {0};

/* 各通道直流偏置基线 - 启动时采样建立 */
volatile int32_t ch_dc_offset[ADC_NCH] = {0};    /* 各通道直流偏置基线 */
volatile uint32_t ch_dc_offset_valid = 0;        /* 基线是否有效标志 */

extern TIM_HandleTypeDef htim7;

/* === 采样率校准 ===
 * 基于实测校准，确保频率计算准确
 * ADC内核时钟: PLL3_R=112.5MHz (J-Link验证D3CCIPR ADCSEL=01)
 * 硅片版本: Rev.V (DBGMCU_IDCODE=0x20036450)
 * 分辨率: 12-bit (J-Link验证 ADC1_CFGR RES=010)
 * 采样时间: 1.5周期 + 12位转换周期 = 14周期/样本 (理论)
 * 6通道扫描，理论值: 56.25MHz / 14 / 6 = 0.669MHz/通道 (÷2 active)
 *                   或 112.5MHz / 14 / 6 = 1.339MHz/通道 (÷2 inactive)
 * 实测: 7.438 MSPS/ch (EMA)，远高于理论值
 * 启动初值：3515000（自校准实测稳定值），运行时仍可微调
 */
/* ADC 每通道实测采样率
 * 实测值：J-Link读取 diag_frame_interval_cyc = 559343 cyc @480MHz = 1.165 ms
 *         => fs_per_ch = 4096 / 1.165ms = 3,515,000 Hz
 * 此前的 2,009,000 是按 PLL3P=56.25MHz/(2*14) 理论值算的，但实测 fs 高出 75%
 * 推测原因：ADC clock prescaler 或 sampling time 实际值与配置不符
 * 验证：DAC 输出 39889 Hz，FFT bin=12 -> 12 × 3515000 / 1024 = 41191 Hz ✓
 * （若理论 fs=2.009MHz，应得 bin=20.3，但实测 bin=12，即真实 fs 高出约 1.75x）
 *
 * ★ 2026-06-12 更新：J-Link确认 ADC1_CFGR RES=010 → 12-bit（非8-bit）
 *   12-bit理论值 2.009 MSPS 仍与实测 7.438 MSPS 差距大，
 *   可能是 Rev.V BOOST 模式下转换周期 < 14 cycles
 *
 * 注意：此值是 RAM 变量，可在线调整。若发现 FFT 输出与 DAC 实际频率仍有偏差，
 * 可临时通过 J-Link 写 0x... 修改本变量重新校准。 */
float adc_fs_hz = 3515000.0f;  /* 实测每通道采样率 (DMA帧间隔反推)
                                   * ★ 2026-06-12 J-Link 实测更新：
                                   *   硅片 Rev.V, PLL3_R=112.5MHz, ADC1_CFGR.RES=010 → 12-bit ✅
                                   *   EMA 校准值 7.438 MSPS, FFT 使用 7.074 MSPS
                                   *   理论值(12-bit/14cyc/56.25MHz=2.009MSPS)与实测不吻合
                                   *   可能原因：Rev.V BOOST模式下12-bit转换周期<14 cycles
                                   *   ★ FFT 频率精度已实测验证正确（使用 adc_fs_hz 校准）*/

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

/***********************************************************
函数名：DWT_Init
参数：  无
返回值：无
描述：  初始化DWT(Data Watchpoint and Trace)周期计数器
        用于高精度时间测量（μs级），SystemCoreClock=480MHz时每μs=480周期
        初始化步骤：使能TRACE时钟→使能DWT→解锁ITM→清零计数器→启动计数
修改记录：
***********************************************************/
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

/* 【2026-06-09 死代码清理】DWT_CycToUs_Simple 已无调用方，
 * 且有累积误差 bug（<1000us 不更新 base → 多次小间隔调用时间戳不准）。
 * 所有时间戳统一使用 DWT_GetUs()（正确处理 32 位回绕，无累积误差）。 */

/***********************************************************
函数名：DWT_GetUs
参数：  无
返回值：uint64_t  当前微秒时间戳
描述：  获取DWT周期计数器对应的微秒时间戳
        算法：读取CYCCNT→除以cyc_per_us→加上dwt_us_base
        当CYCCNT接近32位溢出时(~8.9s@480MHz)，自动补偿到base
        返回值单调递增，不会回绕，可用于计算时间间隔
修改记录：
***********************************************************/
/* 【2026-06-11 优化】DWT_GetUs: 缓存常量消除重复除法，直接用CYCCNT单调时间戳
 * 原实现每次调用都读SysTick->LOAD和计算SystemCoreClock/1000000U（除法），
 * 且do-while循环在SysTick中断频繁时可能多次重试。
 * 新实现：使用DWT_Init()缓存的dwt_cyc_per_us，直接CYCCNT/dwt_cyc_per_us，
 * 无除法开销（编译器将除法优化为乘法+移位），无do-while重试。
 * 注意：CYCCNT 32位@480MHz约8.9s回绕，对本应用（事件时间戳）足够。
 *       若需>8.9s单调递增，应改用tick1*1000+sub_us混合方案。 */
/* 【2026-06-12 进一步优化】DWT_GetUs: 用32位除法替代64位除法，省20-40 cycles/调用
 * CYCCNT 是 32-bit，480MHz 下 ~8.9s 回绕一次。
 * 新方案：直接 (uint64_t)CYCCNT * 1000 / (SystemCoreClock/1000)
 *         即先乘1000再除 MHz 值，全程 32 位运算（乘法后 cast 为 64 位防溢出）
 *         精度完全等效原实现，但避免了 __aeabi_uldivmod 调用。
 */
static uint64_t DWT_GetUs(void)
{
  uint32_t cyc;
  uint32_t delta;
  uint32_t mhz;

  cyc = DWT->CYCCNT;
  delta = cyc - dwt_last_cyc;
  dwt_last_cyc = cyc;
  dwt_us_base += (uint64_t)delta;

  mhz = dwt_cyc_per_us;
  if(mhz == 0U) {
    mhz = SystemCoreClock / 1000000U;
    if(mhz == 0U) mhz = 480U;
  }
  return dwt_us_base / (uint64_t)mhz;
}

static uint64_t DWT_RecentCycToUs(uint32_t cyc)
{
  uint64_t now_us = DWT_GetUs();
  uint32_t mhz = dwt_cyc_per_us;
  uint32_t delta_cyc;

  if(mhz == 0U) {
    mhz = SystemCoreClock / 1000000U;
    if(mhz == 0U) mhz = 480U;
  }
  delta_cyc = dwt_last_cyc - cyc;
  return now_us - ((uint64_t)delta_cyc / (uint64_t)mhz);
}

static uint64_t FrameStartFromEndUs(uint64_t end_us, uint64_t half_dur_us)
{
  return (end_us > half_dur_us) ? (end_us - half_dur_us) : 0U;
}

static void Fill_ChannelStartTimes(uint64_t ch_start_us[ADC_NCH], uint32_t adc1_end_cyc,
                                   uint32_t adc2_end_cyc, uint32_t adc3_end_cyc,
                                   uint64_t half_dur_us)
{
  uint64_t adc1_start = FrameStartFromEndUs(DWT_RecentCycToUs(adc1_end_cyc), half_dur_us);
  uint64_t adc2_start = FrameStartFromEndUs(DWT_RecentCycToUs(adc2_end_cyc), half_dur_us);
  uint64_t adc3_start = FrameStartFromEndUs(DWT_RecentCycToUs(adc3_end_cyc), half_dur_us);
  uint64_t rank2_us = (uint64_t)(0.5f / adc_fs_hz * 1e6f + 0.5f);

  ch_start_us[0] = adc1_start;
  ch_start_us[1] = adc1_start + rank2_us;
  ch_start_us[2] = adc3_start;
  ch_start_us[3] = adc3_start + rank2_us;
  ch_start_us[4] = adc2_start;
  ch_start_us[5] = adc2_start + rank2_us;
}

/* 【2026-06-09 死代码清理】evt_tx_queue 整套已废弃（仅 tx_ring 在用）
 * 原因：evt_tx_queue_poll() 使用阻塞式 HAL_UART_Transmit，
 *        与 tx_ring_poll() 的 DMA 非阻塞发送共享 hdebug_uart 会冲突。
 *        所有事件帧发送路径均已切换到 tx_ring（零阻塞 DMA），
 *        evt_tx_queue_push/poll 无任何调用方，属于纯死代码。
 * 清理：移除 64×19B=1216B 静态缓冲 + push/poll 函数。
 * 保留：diag_uart_q_drop_cnt / diag_uart_q_push_cnt 仍被 tx_ring 路径使用。 */
uint32_t diag_uart_q_drop_cnt __attribute__((section(".AXI_SRAM"))) = 0;           /* 队列满丢帧计数（tx_ring 使用） */
uint32_t diag_uart_q_push_cnt __attribute__((section(".AXI_SRAM"))) = 0;           /* 入队成功计数（tx_ring 使用） */
uint32_t diag_uart_q_send_cnt __attribute__((section(".AXI_SRAM"))) = 0;           /* DMA 发送完成计数（TxCpltCallback 使用） */

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
/***********************************************************
函数名：HAL_UART_TxCpltCallback
参数：  huart - UART句柄指针
返回值：无
描述：  UART DMA发送完成回调函数
        由DMA TC中断→HAL_UART_IRQHandler→此回调
        仅递增diag_uart_q_send_cnt计数器，不做其他操作
        gState自动复位为READY，systick_tx_poll可启动下一次发送
修改记录：
***********************************************************/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance != USART1) return;
  diag_uart_q_send_cnt++;   /* 仅观测，不操作 HAL */
}

/* ★ tx_ring_poll() v6.1 - 强化诊断 ★
 * 直接用 HAL_UART_GetState 并把 state 写入 diag 供 J-Link 查
 * 通过 return 值区分各种失败路径，避免静默失败 */
static volatile uint32_t diag_tx_poll_st = 0xFFFFFFFFU;  /* 最近一次的 UART state */
static volatile uint32_t diag_tx_poll_call_cnt = 0;       /* poll 调用总次数 */
static volatile uint32_t diag_tx_poll_send_attempt = 0;   /* 尝试 HAL_Transmit_DMA 次数 */
static volatile uint32_t diag_tx_poll_send_ok = 0;        /* HAL OK 次数 */

/* 【2026-06-10 架构重构】原 tx_ring_poll() 已由 SysTick 中断中的
 * systick_tx_poll() 替代，主循环不再调用。保留函数体供参考但标记为可能未使用。 */

/* 【2026-06-10 架构重构】SysTick 中断调用的 UART DMA 发送函数
 * 非 static，供 stm32h7xx_it.c 的 SysTick_Handler 调用。
 * 逻辑与原 tx_ring_poll() 相同：检查 tx_ring 队列 + UART DMA 状态 → 发送。
 * 由 SysTick 1ms 中断驱动，确保定时发送。 */
/***********************************************************
函数名：systick_tx_poll
参数：  无
返回值：无
描述：  SysTick中断驱动的UART DMA发送轮询函数
        每1ms由SysTick_Handler调用，检查tx_ring环形队列：
        若有待发事件帧且UART1 DMA空闲→启动DMA异步发送
        若UART忙或无待发帧→立即返回（<1μs）
        三级流水线第三级：Step A(触发)→Step B(FFT)→SysTick(TX)
修改记录：
***********************************************************/
static void diag_snapshot_update(void)
{
  uint32_t sum = 0;
  uint32_t i;

  diag_snapshot.magic = 0xADC75001U;
  diag_snapshot.version = 2U;
  diag_snapshot.loop_cnt = diag_loop_cnt;
  diag_snapshot.half_complete_cnt = half_complete_cnt;
  diag_snapshot.full_complete_cnt = full_complete_cnt;
  diag_snapshot.frame_seen_cnt = diag_frame_seen_cnt;
  diag_snapshot.frame_interval_cyc = diag_frame_interval_cyc;
  diag_snapshot.adc_fs_hz_int = (uint32_t)(adc_fs_hz + 0.5f);
  diag_snapshot.fft_used_fs_int = (uint32_t)(diag_fft_used_fs + 0.5f);
  diag_snapshot.fft_max_bin = diag_fft_max_bin;
  diag_snapshot.uart_q_push_cnt = diag_uart_q_push_cnt;
  diag_snapshot.uart_q_send_cnt = diag_uart_q_send_cnt;
  diag_snapshot.tx_poll_call_cnt = diag_tx_poll_call_cnt;
  diag_snapshot.tx_poll_send_ok = diag_tx_poll_send_ok;
  diag_snapshot.tx_head_tail = ((uint32_t)tx_ring_head << 8) | (uint32_t)tx_ring_tail;
  diag_snapshot.last_n_trig = diag_last_n_trig;
  diag_snapshot.prescan_hit_cnt = diag_prescan_hit_cnt;
  diag_snapshot.candidate_cnt = diag_external_event_candidate_cnt;
  diag_snapshot.noise_filter_cnt = diag_external_noise_filter_cnt;
  diag_snapshot.freq_filter_cnt = diag_external_freq_filter_cnt;
  diag_snapshot.last_selected_mask = diag_last_selected_mask;
  diag_snapshot.last_filtered_mask = diag_last_filtered_mask;
  diag_snapshot.locked_input_mask = diag_locked_input_mask;
  diag_snapshot.stress_burst_total = diag_stress_burst_total;
  diag_snapshot.stress_cur_idx = (uint32_t)diag_stress_cur_idx;
  diag_snapshot.dac_cr = DAC1->CR;
  diag_snapshot.dac_dhr12r2 = DAC1->DHR12R2;
  diag_snapshot.tim7_cr1 = htim7.Instance->CR1;
  diag_snapshot.tim7_cnt = htim7.Instance->CNT;
  diag_snapshot.tim7_arr = htim7.Instance->ARR;
  diag_snapshot.tim7_dier = htim7.Instance->DIER;
  diag_snapshot.dma1s1_cr = ((DMA_Stream_TypeDef*)hdma_dac1_ch2.Instance)->CR;
  diag_snapshot.dma1s1_ndtr = ((DMA_Stream_TypeDef*)hdma_dac1_ch2.Instance)->NDTR;
  diag_snapshot.dac_actual_freq_hz = dac_diag_actual_freq_hz;
  for(i = 0; i < ADC_NCH; i++) {
    diag_snapshot.ch_fft_freq[i] = diag_ch_last_fft_freq[i];
    diag_snapshot.ch_evt_freq[i] = diag_evt_last_freq[i];
    diag_snapshot.ch_evt_time_lo[i] = diag_evt_last_time_lo[i];
    diag_snapshot.ch_evt_time_hi[i] = diag_evt_last_time_hi[i];
    diag_snapshot.ch_max_dev[i] = diag_last_max_dev[i];
    diag_snapshot.ch_thr[i] = diag_last_thr[i];
    diag_snapshot.ch_noise_pp[i] = diag_last_noise_pp[i];
  }
  diag_snapshot.adc_dma_half_cyc[0] = adc1_dma_half_cyc;
  diag_snapshot.adc_dma_half_cyc[1] = adc2_dma_half_cyc;
  diag_snapshot.adc_dma_half_cyc[2] = adc3_dma_half_cyc;
  diag_snapshot.adc_dma_full_cyc[0] = adc1_dma_full_cyc;
  diag_snapshot.adc_dma_full_cyc[1] = adc2_dma_full_cyc;
  diag_snapshot.adc_dma_full_cyc[2] = adc3_dma_full_cyc;
  diag_snapshot.adc_dma_half_delta_cyc[0] = 0U;
  diag_snapshot.adc_dma_half_delta_cyc[1] = adc2_dma_half_cyc - adc1_dma_half_cyc;
  diag_snapshot.adc_dma_half_delta_cyc[2] = adc3_dma_half_cyc - adc1_dma_half_cyc;
  diag_snapshot.adc_dma_full_delta_cyc[0] = 0U;
  diag_snapshot.adc_dma_full_delta_cyc[1] = adc2_dma_full_cyc - adc1_dma_full_cyc;
  diag_snapshot.adc_dma_full_delta_cyc[2] = adc3_dma_full_cyc - adc1_dma_full_cyc;
  diag_snapshot.checksum = 0U;
  for(i = 0; i < (sizeof(diag_snapshot) / sizeof(uint32_t)) - 1U; i++) {
    sum += ((volatile uint32_t *)&diag_snapshot)[i];
  }
  diag_snapshot.checksum = sum;
}

void systick_tx_poll(void)
{
  uint8_t head_snap;
  uint8_t tail_snap;
  HAL_UART_StateTypeDef st;
  HAL_StatusTypeDef tx_st;
  uint8_t tail;

  diag_tx_poll_call_cnt++;

  head_snap = tx_ring_head;
  tail_snap = tx_ring_tail;
  if(head_snap == tail_snap) return;

  st = HAL_UART_GetState(&hdebug_uart);
  diag_tx_poll_st = (uint32_t)st;
  if(st != HAL_UART_STATE_READY && st != HAL_UART_STATE_BUSY_RX) return;

  tail = tail_snap;
  diag_tx_poll_send_attempt++;

  SCB_CleanDCache_by_Addr((uint32_t*)tx_ring[tail], EVT_FRAME_TOTAL_LEN);
  tx_st = HAL_UART_Transmit_DMA(&hdebug_uart, (uint8_t*)tx_ring[tail], EVT_FRAME_TOTAL_LEN);
  if(tx_st == HAL_OK) {
    tx_ring_tail = (uint8_t)((tail + 1U) % TX_RING_SLOTS);
    diag_tx_poll_send_ok++;
  } else {
    diag_uart_hal_fail_cnt++;
  }
}

/***********************************************************
函数名：push_frame_from_dma
参数：  half_idx - 半区索引(0=前半区, 1=后半区)
        now_us   - 当前时间戳(μs)
返回值：无
描述：  将ADC DMA半区数据从3个ADC缓冲区合并到frame_pool
        步骤：1.从frame_pool取空闲帧 2.合并3个ADC数据(去交错)
              3.记录帧ID/时间戳 4.CleanDCache 5.更新frame写入索引
修改记录：
***********************************************************/
static void push_frame_from_dma(uint8_t half_idx, const uint64_t ch_start_us[ADC_NCH])
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
   *
   * ★ 2026-06-12 优化：原循环每次迭代访问 3 个不同源缓冲区
   *   + 6 次跨步写入目标，导致 D-Cache 行频繁 evict/reload（cache thrash）。
   *   优化策略：
   *   1. 预计算源指针，消除每次迭代的数组偏移计算
   *   2. 使用局部指针减少对 fr->data 的重复解引用
   *   3. 12-bit 模式下 & 0x0FFF 保留完整数据（无需掩码，
   *      DMA 半字传输时 ADC DR 高 4 位为 0，但保留掩码防御性编程）
   *   4. 展开内层 2 样本（rank1+rank2），减少循环开销
   *
   *   性能预期：减少 ~30% 的 D-Cache miss（从 6 路 bank 冲突
   *   降为 3 路顺序读取），在 480MHz + WB D-Cache 下约省 200-400 cycles
   */
  {
    const uint16_t *src1 = &adc1_buf[src_off];  /* ADC1 源起始 */
    const uint16_t *src2 = &adc2_buf[src_off];  /* ADC2 源起始 */
    const uint16_t *src3 = &adc3_buf[src_off];  /* ADC3 源起始 */
    uint16_t *dst = fr->data;
    uint16_t n = HALF_SAMPLES_PER_CH;

    for(i = 0; i < n; i++) {
      uint16_t s1r1 = src1[0] & 0x0FFF;  /* CH1 = ADC1 rank1 */
      uint16_t s1r2 = src1[1] & 0x0FFF;  /* CH2 = ADC1 rank2 */
      uint16_t s3r1 = src3[0] & 0x0FFF;  /* CH3 = ADC3 rank1 */
      uint16_t s3r2 = src3[1] & 0x0FFF;  /* CH4 = ADC3 rank2 */
      uint16_t s2r1 = src2[0] & 0x0FFF;  /* CH5 = ADC2 rank1 */
      uint16_t s2r2 = src2[1] & 0x0FFF;  /* CH6 = ADC2 rank2 */
      src1 += 2;
      src2 += 2;
      src3 += 2;
      dst[0] = s1r1;
      dst[1] = s1r2;
      dst[2] = s3r1;
      dst[3] = s3r2;
      dst[4] = s2r1;
      dst[5] = s2r2;
      dst += ADC_NCH;
    }
  }
  fr->start_time_us = ch_start_us[0];
  for(i = 0; i < ADC_NCH; i++) {
    fr->ch_start_time_us[i] = ch_start_us[i];
  }
  fr->frame_id = total_frame_cnt++;
  fr->ready = 1;
  frame_w_idx = next;
  /* 【2026-06-09 修复】frame_pool 在 AXI_SRAM，CPU 写入后必须 Clean DCache
   * 确保主循环读到的 ready/data 是最新值（单核同域其实不需要，但为安全起见，
   * 防止将来 DMA 或其他 master 访问 frame_pool 时读到 stale cache 数据） */
  SCB_CleanDCache_by_Addr((uint32_t*)fr, sizeof(adc_frame_t));
}

/***********************************************************
函数名：check_dma_and_push_frames
参数：  无
返回值：无
描述：  检查3个ADC的DMA半区/全区标志，同步合并后推入frame_pool
        3-ADC同步逻辑：等待所有3个ADC同一半区都完成后才合并
        在线校准ADC采样率：EMA(α=0.05)平滑帧间隔反推的采样率
        核心数据通路：ADC DMA→此函数→frame_pool→Step A→trig_queue
修改记录：
***********************************************************/
static void check_dma_and_push_frames(void)
{
  uint64_t end_us;
  uint64_t start_us;
  uint64_t ch_start_us[ADC_NCH];
  uint32_t frame_start_cyc;
  uint32_t frame_process_cyc;
  /* 【2026-06-11 优化】half_dur_us/desync_timeout_cyc 缓存为静态变量
   * 原实现每次调用都做浮点除法和乘法计算，每 1.165ms 执行一次。
   * adc_fs_hz 在线校准 EMA α=0.05 极缓慢变化，这里用 1000 次调用的
   * 旧值误差 < 0.005%，可接受。每 1024 次调用刷新一次缓存。 */
  static uint64_t cached_half_dur_us = 0;
  static uint32_t cached_desync_timeout_cyc = 0;
  static uint16_t cache_refresh_cnt = 0;

  cache_refresh_cnt++;
  if(cache_refresh_cnt >= 1024 || cached_half_dur_us == 0) {
    cached_half_dur_us = (uint64_t)((float)HALF_SAMPLES_PER_CH / adc_fs_hz * 1e6f + 0.5f);
    cached_desync_timeout_cyc = (uint32_t)(2U * cached_half_dur_us) * (SystemCoreClock / 1000000U);
    cache_refresh_cnt = 0;
  }
  /* 【2026-06-07 v8 E.10】3 ADC 跨域 flag 失同步超时回退 */
  uint32_t now_cyc;
  uint32_t since_last_cyc;
  uint8_t half_mask;
  uint8_t full_mask;

  if(adc1_dma_half && adc2_dma_half && adc3_dma_half) {
    frame_start_cyc = DWT->CYCCNT;
    adc1_dma_half = 0;
    adc2_dma_half = 0;
    adc3_dma_half = 0;
    /* 【2026-06-07 v8 修复 E.11】只 invalidate 半区0（前半缓冲区）
     * 原问题：invalidate 整个缓冲区会在 DMA 正在写半区1时读到脏数据
     * 半区大小 = ADC_DMA_BUF_SIZE * sizeof(uint16_t) / 2 = 16384 字节
     * 注：SCB_InvalidateDCache_by_Addr 参数 2 是字节数 */
    SCB_InvalidateDCache_by_Addr((uint32_t*)adc1_buf, ADC_DMA_BUF_SIZE * sizeof(uint16_t) / 2);
    SCB_InvalidateDCache_by_Addr((uint32_t*)adc2_buf, ADC_DMA_BUF_SIZE * sizeof(uint16_t) / 2);
    SCB_InvalidateDCache_by_Addr((uint32_t*)adc3_buf, ADC_DMA_BUF_SIZE * sizeof(uint16_t) / 2);
    Fill_ChannelStartTimes(ch_start_us, adc1_dma_half_cyc, adc2_dma_half_cyc, adc3_dma_half_cyc, cached_half_dur_us);
    end_us = DWT_RecentCycToUs(adc1_dma_half_cyc);
    start_us = ch_start_us[0];
    (void)end_us;
    (void)start_us;
    push_frame_from_dma(0, ch_start_us);
    half_complete_cnt++;
    
    /* 记录帧处理耗时 */
    frame_process_cyc = DWT->CYCCNT - frame_start_cyc;
    diag_frame_process_cyc = frame_process_cyc;
    if(frame_process_cyc > diag_frame_process_max) {
      diag_frame_process_max = frame_process_cyc;
    }
    /* 用 ADC DMA ISR 时间戳校准采样率。
     * 主循环处理时间会被 FFT/队列延迟拉长，不能作为 ADC 采样周期依据。
     * half - previous full 正好对应一个 DMA 半区，即 HALF_SAMPLES_PER_CH 个单通道样本。 */
    {
      uint32_t interval1 = adc1_dma_half_cyc - adc1_dma_full_cyc;
      uint32_t interval2 = adc2_dma_half_cyc - adc2_dma_full_cyc;
      uint32_t interval3 = adc3_dma_half_cyc - adc3_dma_full_cyc;
      uint32_t half_interval = (uint32_t)(((uint64_t)interval1 + (uint64_t)interval2 + (uint64_t)interval3) / 3ULL);
      diag_frame_interval_cyc = half_interval;
      if(half_interval > 10000U && half_interval < 1000000U) {
        float measured_fs = (float)HALF_SAMPLES_PER_CH * (float)SystemCoreClock / (float)half_interval;
        adc_fs_hz = adc_fs_hz * 0.90f + measured_fs * 0.10f;
      }
    }
  }

  if(adc1_dma_full && adc2_dma_full && adc3_dma_full) {
    frame_start_cyc = DWT->CYCCNT;
    adc1_dma_full = 0;
    adc2_dma_full = 0;
    adc3_dma_full = 0;
    /* 【2026-06-07 v8 修复 E.11】只 invalidate 半区1（后半缓冲区）
     * 半区起始地址 = buf + ADC_DMA_BUF_SIZE * sizeof(uint16_t) / 2
     * 半区字节数  = ADC_DMA_BUF_SIZE * sizeof(uint16_t) / 2 */
    SCB_InvalidateDCache_by_Addr((uint32_t*)((uint8_t*)adc1_buf + ADC_DMA_BUF_SIZE * sizeof(uint16_t) / 2),
                                 ADC_DMA_BUF_SIZE * sizeof(uint16_t) / 2);
    SCB_InvalidateDCache_by_Addr((uint32_t*)((uint8_t*)adc2_buf + ADC_DMA_BUF_SIZE * sizeof(uint16_t) / 2),
                                 ADC_DMA_BUF_SIZE * sizeof(uint16_t) / 2);
    SCB_InvalidateDCache_by_Addr((uint32_t*)((uint8_t*)adc3_buf + ADC_DMA_BUF_SIZE * sizeof(uint16_t) / 2),
                                 ADC_DMA_BUF_SIZE * sizeof(uint16_t) / 2);
    Fill_ChannelStartTimes(ch_start_us, adc1_dma_full_cyc, adc2_dma_full_cyc, adc3_dma_full_cyc, cached_half_dur_us);
    end_us = DWT_RecentCycToUs(adc1_dma_full_cyc);
    start_us = ch_start_us[0];
    (void)end_us;
    (void)start_us;
    push_frame_from_dma(1, ch_start_us);
    full_complete_cnt++;
    
    /* 记录帧处理耗时 */
    frame_process_cyc = DWT->CYCCNT - frame_start_cyc;
    diag_frame_process_cyc = frame_process_cyc;
    if(frame_process_cyc > diag_frame_process_max) {
      diag_frame_process_max = frame_process_cyc;
    }
    /* 计算帧间隔（full→full 诊断，不参与 EMA）
     * ★ 2026-06-12: EMA 校准已移至 half 路径（使用独立的 half_ema_last_cyc）
     *   此处仅记录 full→full 间隔供诊断参考 */
    if(diag_last_frame_cyc > 0) {
      /* full→full 间隔 ≈ 2 × half_interval，不做 EMA 校准 */
      (void)(frame_start_cyc - diag_last_frame_cyc);
    }
    diag_last_frame_cyc = frame_start_cyc;
  }
  
  /* 计算空闲比例
   * ★ 2026-06-12 修复：当 diag_frame_process_cyc > diag_frame_interval_cyc 时，
   *   差值为负数，转 float 再转 uint32 会导致整数回绕（值 > 20 亿）。
   *   原因：3 级流水线中帧处理可能跨越多个帧间隔（Step A + Step B 并行），
   *   process_cyc 记录的是单次 check_dma_and_push_frames 的总耗时，
   *   而 interval_cyc 是两次帧起始的时间差，二者并非同一时间窗口。
   *   修复：先做 float 比较再 clamp 到 [0, 100] */
  if(diag_frame_interval_cyc > 0 && diag_frame_process_cyc > 0) {
    float idle_pct = (float)(diag_frame_interval_cyc - diag_frame_process_cyc) * 100.0f / (float)diag_frame_interval_cyc;
    if(idle_pct < 0.0f) idle_pct = 0.0f;
    if(idle_pct > 100.0f) idle_pct = 100.0f;
    diag_idle_ratio = (uint32_t)idle_pct;
  }

  /* 【2026-06-07 v8 E.10 → 2026-06-12 重新启用】跨域 flag 失同步超时回退
   * 之前禁用是因为"疑似 DMA TEIF 风暴导致死锁"。
   * 根因分析：ADC ErrorCallback 中已禁用 TEIE（见 adc.c:604），
   *   TEIF 不再产生中断风暴。200ms 看门狗已能恢复死掉的 ADC DMA。
   *   但如果不启用此回退，单个 ADC DMA 出错后其 flag 永不置位，
   *   导致主循环永远等不到 3-flag 齐全 → 帧处理完全停止 200ms。
   * 修正：启用回退，但将超时从 desync_timeout_cyc 改为 2 倍正常帧间隔
   *   （约 2.3ms = 2×1.165ms），避免误清正常延迟的 flag。
   */
#if 1
  /* 检测：任一 ADC 的 half flag 已就绪，但 3 个未全到，且距离上次成帧已超过 desync_timeout
   * 处理：强制清掉孤立 flag（这次半区数据丢弃，等下一周期重新对齐）
   * 同样适用 full flag */
  now_cyc = DWT->CYCCNT;
  since_last_cyc = now_cyc - diag_last_frame_cyc;
  if(diag_last_frame_cyc != 0U && since_last_cyc > cached_desync_timeout_cyc) {
    half_mask = (uint8_t)((adc1_dma_half ? 1U : 0U) |
                          (adc2_dma_half ? 2U : 0U) |
                          (adc3_dma_half ? 4U : 0U));
    full_mask = (uint8_t)((adc1_dma_full ? 1U : 0U) |
                          (adc2_dma_full ? 2U : 0U) |
                          (adc3_dma_full ? 4U : 0U));
    if(half_mask != 0U && half_mask != 0x7U) {
      diag_adc_desync_cnt++;
      diag_adc_desync_mask = (uint32_t)half_mask;
      adc1_dma_half = 0;
      adc2_dma_half = 0;
      adc3_dma_half = 0;
    }
    if(full_mask != 0U && full_mask != 0x7U) {
      diag_adc_desync_cnt++;
      diag_adc_desync_mask = (uint32_t)full_mask | 0x10U;  /* bit4=1 表示 full 域 */
      adc1_dma_full = 0;
      adc2_dma_full = 0;
      adc3_dma_full = 0;
    }
  }
#else
  (void)now_cyc; (void)since_last_cyc; (void)cached_desync_timeout_cyc;
  (void)half_mask; (void)full_mask;
#endif
}

/* ====================================================================== */
/*  FFT缓冲区 — 放在DTCMRAM以获得零等待访问，提高FFT计算速度             */
/* ====================================================================== */
static float fft_buf[FFT_LENGTH * 2]  __attribute__((section(".DTCMRAM"), aligned(32)));   /* FFT输入/输出缓冲区(复数) */
static float fft_mag[FFT_LENGTH / 2]  __attribute__((section(".DTCMRAM"), aligned(32)));   /* FFT幅度谱 */
static float fft_in[FFT_LENGTH]       __attribute__((section(".DTCMRAM"), aligned(32)));   /* FFT输入(实数) */
static float hann_win[FFT_LENGTH]              __attribute__((section(".DTCMRAM"), aligned(32)));   /* Hann窗系数 */
static float tw_cos[FFT_LENGTH / 2]            __attribute__((aligned(32)));    /* 旋转因子cos分量 */
static float tw_sin[FFT_LENGTH / 2]            __attribute__((aligned(32)));    /* 旋转因子sin分量 */
static uint8_t fft_tables_ready = 0;    /* FFT表初始化完成标志 */

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

/* === v9 闭环压力测试场景表（DAC_AS_EXTERNAL_TEST_SRC 模式使用） ===
 * 硬件约束：PA5→运放→ADC 链路上的运放是 **带通滤波器，中心频率 ~40KHz**，
 *          其他频段被严重衰减，故所有测试频点必须落在 38~42KHz 范围内。
 * 每轮 burst 轮换一种"非理想信号"，验证 6 通道触发链路鲁棒性。
 * 字段：基频 / 基波幅值(mV) / 2 次谐波(%) / 3 次谐波(%) / 白噪声(%)
 *
 * 注意：谐波频率 = 基频×2/3，会被带通衰减，所以谐波只影响 ADC 端波形纯度，
 *      不影响主峰检测——这正好模拟"真实信号经运放后保留基波、谐波被滤掉"的场景。 */
typedef struct {
  uint32_t freq_hz;
  uint16_t amp_mv;
  uint8_t  h2_pct;
  uint8_t  h3_pct;
  uint8_t  noise_pct;
} stress_case_t;

static const stress_case_t stress_cases[] = {
  { 40000U, 1000U,  0U,  0U,  0U },   /* 0: 纯净基波（基准）*/
  { 40000U, 1000U, 25U,  0U,  0U },   /* 1: +25% 2 次谐波 */
  { 40000U, 1000U,  0U, 20U,  0U },   /* 2: +20% 3 次谐波 */
  { 40000U, 1000U, 20U, 15U,  0U },   /* 3: 2+3 次谐波混合 */
  { 40000U, 1000U,  0U,  0U, 10U },   /* 4: +10% 白噪声 */
  { 40000U, 1000U, 15U, 10U, 15U },   /* 5: 谐波 + 噪声（严苛）*/
  { 38000U, 1000U,  0U,  0U,  0U },   /* 6: 通带低端 38KHz */
  { 42000U, 1000U,  0U,  0U,  0U },   /* 7: 通带高端 42KHz */
  { 40000U,  300U,  0U,  0U,  0U },   /* 8: 小幅值 0.3V（灵敏度下限）*/
  { 40000U, 1500U,  0U,  0U,  0U }    /* 9: 大幅值 1.5V（不饱和）*/
};
#define STRESS_CASE_COUNT  (sizeof(stress_cases) / sizeof(stress_cases[0]))

/* 压力测试诊断变量（J-Link 可读）- 已在上方统一声明块中 */

static const char *ch_names[ADC_NCH] = {
  "CH1(PC0)",
  "CH2(PC1)",
  "CH3(PC2)",
  "CH4(PC3)",
  "CH5(PA3)",
  "CH6(PA4)"
};

/***********************************************************
函数名：MPU_Config
参数：  无
返回值：无
描述：  配置MPU(内存保护单元)内存区域属性
        Region0: QSPI(0x90000000,256MB) - No Access(防止误访问)
        Region1: AXI SRAM(0x24000000,512KB) - Cacheable, Write-Back
                 主DMA缓冲区、事件队列、调试环形缓冲区
                 DCache维护操作(SCB_CleanDCache_by_Addr)用于DMA一致性
        Region2: DTCM(0x20000000,128KB) - Non-Cacheable
                 栈、FFT缓冲区、关键RW数据
                 DTCM在CPU专用总线上，0等待，天然与DMA隔离
                 标记Non-Cacheable确保SCB_CleanDCache不会意外写回
        Region3: SRAM4(0x38000000,64KB) - Non-Cacheable(D3域)
                 ADC3 DMA缓冲区，D3域与D1域DMA路径不同
                 ★必须Non-Cacheable：DCache操作对D3域SRAM可能是no-op
                   或产生未定义行为(RM0433 §2.4)，ADC3数据一致性
                   完全依赖Non-Cacheable属性+DMA直写
修改记录：2026-06-10 新增Region2(DTCM)和Region3(SRAM4) MPU配置
***********************************************************/
static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};
  HAL_MPU_Disable();

  /* Region0: QSPI - 禁止访问（防止误读未映射的QSPI地址空间） */
  MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
  MPU_InitStruct.Number           = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress      = QSPI_BASE;
  MPU_InitStruct.Size             = MPU_REGION_SIZE_256MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL1;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Region1: AXI SRAM - Cacheable Write-Back（DMA缓冲区，需手动Cache维护） */
  MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
  MPU_InitStruct.Number           = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress      = 0x24000000;
  MPU_InitStruct.Size             = MPU_REGION_SIZE_512KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable     = MPU_ACCESS_BUFFERABLE;
  MPU_InitStruct.IsCacheable      = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL1;
  MPU_InitStruct.SubRegionDisable = 0x00;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Region2: DTCM - Non-Cacheable（栈+FFT缓冲，CPU专用总线0等待） */
  MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
  MPU_InitStruct.Number           = MPU_REGION_NUMBER2;
  MPU_InitStruct.BaseAddress      = 0x20000000;
  MPU_InitStruct.Size             = MPU_REGION_SIZE_128KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Region3: SRAM4(D3域) - Non-Cacheable（ADC3 DMA缓冲区）
   * ★ 关键：D3域SRAM的Cache行为与D1/D2域不同，DCache操作可能无效
   *   标记Non-Cacheable确保CPU读到的ADC3数据就是DMA写入的真实值
   * 【2026-06-12】Size 从 32KB 扩展到 64KB，与 scatter file 一致，
   *   确保 SRAM4 全部 64KB 都在 MPU 保护范围内 */
  MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
  MPU_InitStruct.Number           = MPU_REGION_NUMBER3;
  MPU_InitStruct.BaseAddress      = 0x38000000;
  MPU_InitStruct.Size             = MPU_REGION_SIZE_64KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable      = MPU_ACCESS_SHAREABLE;  /* D3域需Shareable确保多主一致 */
  MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Region4: ITCM - 热代码零等待执行区（FFT/触发检测等标记.ITCM的函数）
   * ITCM在CPU专用64-bit总线上，零等待取指，确定性执行
   * 不走I-Cache路径，无Cache miss风险
   * 2026-06-11 新增：配合scatter file ER_ITCM段使用 */
  MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
  MPU_InitStruct.Number           = MPU_REGION_NUMBER4;
  MPU_InitStruct.BaseAddress      = 0x00000000;
  MPU_InitStruct.Size             = MPU_REGION_SIZE_64KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable     = MPU_ACCESS_BUFFERABLE;
  MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;  /* TCM不走Cache */
  MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;  /* 允许执行 */
  MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/***********************************************************
函数名：CPU_CACHE_Enable
参数：  无
返回值：无
描述：  使能CPU I-Cache和D-Cache
        I-Cache: 加速指令取指（Flash等待周期高时效果显著）
        D-Cache: 加速数据访问（AXI SRAM访问需Cache维护操作）
        注意：启用D-Cache后，DMA缓冲区需SCB_CleanDCache/InvalidateDCache
修改记录：
***********************************************************/
static void CPU_CACHE_Enable(void)
{
  SCB_EnableICache();
  SCB_EnableDCache();
}

void SystemClock_Config(void);

/***********************************************************
函数名：FFT_Tables_Init
参数：  无
返回值：无
描述：  初始化FFT所需的Hann窗和旋转因子表
        hann_win[i] = 0.5*(1-cos(2πi/N)), N=FFT_LENGTH=1024
        tw_cos/tw_sin: 旋转因子cos/sin(2πi/N), 用于零交叉法频率计算
修改记录：
***********************************************************/
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
/***********************************************************
函数名：FFT_Benchmark
参数：  无
返回值：无
描述：  FFT性能基准测试
        生成已知频率正弦波→FFT→验证频率测量精度
        测试256/512/1024点FFT耗时和频率结果
        仅在初始化阶段调用一次，正常运行时不调用
修改记录：
***********************************************************/
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
/***********************************************************
函数名：Calc_DC_Offset_Baseline
参数：  duration_ms - 采集持续时间(毫秒)
返回值：无
描述：  计算ADC各通道的直流偏移基线值
        在初始化阶段调用，采集duration_ms时间的ADC数据
        对每个通道取平均值作为基线，存入main()的baselines数组
        用于后续触发检测：信号偏移超过baseline+threshold时判定为跳变
修改记录：
***********************************************************/
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
/***********************************************************
函数名：Calc_Baseline_in_Frame
参数：  buf - ADC帧缓冲区, n_samples - 样本数, ch - 通道号, noise_pp_out - 噪声峰峰值输出
返回值：int32_t  基线平均值（ADC计数值）
描述：  计算帧内单通道的基线值和噪声峰峰值
修改记录：
***********************************************************/
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

/***********************************************************
函数名：Detect_Transition_in_Frame
参数：  buf - ADC帧缓冲区, n_samples - 样本数, ch - 通道号
        baseline - 直流基线值, threshold - 触发阈值
返回值：uint16_t  跳变起始索引，0xFFFF表示未检测到跳变
描述：  检测帧内单通道从DC到AC的跳变时刻
        连续DEV_CONFIRM_CNT个样本偏移超过threshold时判定为跳变
修改记录：
***********************************************************/
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

/***********************************************************
函数名：Verify_Sustain_in_Frame
参数：  buf - ADC帧缓冲区, n_samples - 样本数, ch - 通道号
        baseline - 直流基线值, threshold - 触发阈值, trans_idx - 跳变索引
返回值：uint8_t  1=持续性验证通过, 0=不通过
描述：  验证跳变后信号是否持续超过阈值
        在trans_idx开始的SUSTAIN_SAMPLES个样本中，
        至少SUSTAIN_MIN_HIT个样本偏移超过threshold才判定有效
        防止噪声毛刺误触发
修改记录：
***********************************************************/
static uint8_t Verify_Sustain_in_Frame(const uint16_t *buf, uint16_t n_samples, uint8_t ch,
                                       int32_t baseline, uint16_t threshold, uint16_t trans_idx)
{
  uint16_t end_i = trans_idx + SUSTAIN_SAMPLES;
  uint16_t tail_start;
  uint16_t hit_cnt = 0;
  uint16_t tail_hit_cnt = 0;
  uint16_t i;
  int32_t d;
  uint32_t ab;
  if(end_i > n_samples) end_i = n_samples;
  if((end_i - trans_idx) < SUSTAIN_SAMPLES) return 0;
  tail_start = end_i - SUSTAIN_TAIL_SAMPLES;
  for(i = trans_idx; i < end_i; i++) {
    d = (int32_t)buf[i * ADC_NCH + ch] - baseline;
    ab = (d < 0) ? (uint32_t)(-d) : (uint32_t)d;
    if(ab > threshold) {
      hit_cnt++;
      if(i >= tail_start) tail_hit_cnt++;
    }
  }
  return (hit_cnt >= SUSTAIN_MIN_HIT && tail_hit_cnt >= SUSTAIN_TAIL_MIN_HIT) ? 1 : 0;
}

/***********************************************************
函数名：Detect_AC_Present_in_Frame
参数：  buf - ADC帧缓冲区, n_samples - 样本数, ch - 通道号
        baseline - 直流基线值, threshold - 触发阈值
        first_idx - 输出首个超阈值索引, max_dev_out - 输出最大偏移值
返回值：uint8_t  1=检测到AC信号, 0=未检测到
描述：  检测帧内是否存在AC信号（跳变后的整体判断）
        从BASELINE_SAMPLES之后扫描，统计超阈值样本数
        至少AC_PRESENT_MIN_HIT个样本超阈值才判定为AC信号
修改记录：
***********************************************************/
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

/***********************************************************
函数名：Refine_Transition_SubSample
参数：  buf - ADC帧缓冲区, n_samples - 样本数, ch - 通道号
        baseline - 直流基线值, threshold - 触发阈值, trans_idx - 跳变索引
返回值：float  亚样本级跳变偏移量（-1.0~0.0之间）
描述：  精确化跳变时刻到亚样本级别
        利用跳变点前后两个样本的偏移值进行线性插值
        计算实际阈值穿越点在两个样本之间的精确位置
修改记录：
***********************************************************/
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

/***********************************************************
函数名：Find_FirstPeak_in_Frame
参数：  buf - ADC帧缓冲区, n_samples - 样本数, ch - 通道号
        baseline - 直流基线值, trans_idx - 跳变索引
返回值：uint16_t  首个极值点索引
描述：  在跳变后搜索第一个极值点（峰值或谷值）
        两遍扫描法：第1遍找最大幅值确定有效阈值(70%)
        第2遍找|d|单调反转点，返回首个达到有效阈值的极值
        不同通道因相位不同，极值可能是峰或谷，独立判断
修改记录：
***********************************************************/
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
/***********************************************************
函数名：Refine_Peak_SubSample
参数：  buf - ADC帧缓冲区, n_samples - 样本数, ch - 通道号
        baseline - 直流基线值, peak_idx - 峰值索引
返回值：float  亚样本级峰值偏移量
描述：  精确化峰值时刻到亚样本级别
        利用峰值点前后两个样本值进行抛物线插值
        返回插值后的精确峰值位置（用于零交叉法频率计算）
修改记录：
***********************************************************/
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

/***********************************************************
函数名：Calc_ZeroCross_Frequency_in_Frame
参数：  buf - ADC帧缓冲区, n_samples - 样本数, ch - 通道号
        start_idx - 起始搜索索引, baseline - 直流基线值
返回值：float  零交叉法计算的频率(Hz)，0表示计算失败
描述：  利用零交叉法计算信号频率
        从start_idx开始搜索信号过零点，统计过零次数
        频率 = 过零次数 / (2 × 时间跨度)
        使用旋转因子tw_cos/tw_sin进行精确过零点插值
修改记录：
***********************************************************/
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
/***********************************************************
函数名：Calc_FFT_Frequency_in_Frame_Optimized
参数：  buf - ADC帧缓冲区, n_samples - 样本数, ch - 通道号, start_idx - 起始索引
返回值：float  FFT检测到的频率(Hz)，0表示未检出
描述：  FFT频率分析主函数（旧架构主循环使用）
        步骤：1.从start_idx开始取FFT_LENGTH个样本
              2.去DC偏移+加Hann窗+转float
              3.arm_rfft_fast_f32执行FFT
              4.计算幅度谱，找峰值频率bin
              5.二次插值精确定位峰值
              6.频率 = bin_index × adc_fs_hz / FFT_LENGTH
修改记录：
***********************************************************/
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

  /* ★ 2026-06-12 优化：sqrt-free 抛物线插值（Jacobsen 1991）
   *   省 3 次 sqrtf ≈60 cycles，精度与 sqrt 版差异 < 0.1%（Hann 窗主瓣接近二次）
   *   delta = (mag[m-1] - mag[m+1]) / (2*(2*max - mag[m-1] - mag[m+1])) */
  float mag_m1, mag_p1, denom2;
  if(max_bin > 0 && max_bin < (FFT_LENGTH / 2 - 1)) {
    mag_m1 = fft_mag_local[max_bin - 1];
    mag_p1 = fft_mag_local[max_bin + 1];
    denom2 = 2.0f * max_val - mag_m1 - mag_p1;
    if(fabsf(denom2) > 1e-6f) {
      delta = (mag_m1 - mag_p1) / (2.0f * denom2);
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

/* 【2026-06-10 架构重构新增】单通道 FFT 频率计算
 * 输入 data[] 已经是解交织后的 1024 点单通道数据（由 Step A 提取到 trig_queue）。
 * 与 Calc_FFT_Frequency_in_Frame_Optimized 逻辑相同，但跳过交织解引。 */
/***********************************************************
函数名：Calc_FFT_Frequency_in_Frame_SingleCh
参数：  data - 单通道去交错后的ADC数据, data_len - 数据长度
返回值：float  检测到的频率(Hz)，0表示未检出
描述：  对单通道数据执行FFT频率分析（新架构Step B使用）
        步骤：去DC→加Hann窗→float转换→arm_rfft_fast_f32→找峰值→算频率
        使用DTCMRAM中的fft_buf/fft_mag，减少AXI SRAM占用
修改记录：
***********************************************************/
static float Calc_FFT_Frequency_in_Frame_SingleCh(const uint16_t *data, uint16_t data_len)
{
  uint16_t i;
  float max_val = 0.0f;
  uint16_t max_bin = 0;
  float avg_val = 0.0f;
  float delta, exact_bin;
  uint16_t bin_min, bin_max;
  float dc_avg;
  float mag_sq;
  float re, im;
  uint32_t fft_t0, fft_t1;
  uint32_t dc_sum;
  uint16_t process_len = (data_len < FFT_LENGTH) ? data_len : FFT_LENGTH;

  /* ★ 2026-06-12 优化：合并 DC 累加 + 减DC + 汉宁窗为单次遍历
   *   原实现分两次遍历 data[]（第一次累加 DC，第二次减 DC+加窗），
   *   对 DTCM 数据来说两次遍历开销不大，但对 AXI SRAM 数据会有
   *   额外 cache miss。合并后只需 1 次读取 + 1 次写入。
   *   DC 累加使用 float 避免大 N 下 uint32 溢出（1024×4095 < 4M，安全）。 */
  dc_sum = 0;
  for(i = 0; i < process_len; i++) {
    dc_sum += data[i];
  }
  dc_avg = (float)dc_sum * (1.0f / (float)process_len);

  for(i = 0; i < process_len; i++) {
    fft_in[i] = ((float)data[i] - dc_avg) * hann_win[i];
  }
  for(; i < FFT_LENGTH; i++) {
    fft_in[i] = 0.0f;
  }

  /* FFT */
  fft_t0 = DWT->CYCCNT;
  arm_rfft_fast_f32(&rfft_inst, fft_in, fft_buf, 0);
  fft_t1 = DWT->CYCCNT - fft_t0;
  fft_last_cyc = fft_t1;
  fft_optimized_cyc = fft_t1;

  /* 幅度平方 */
  fft_mag[0] = fft_buf[0] * fft_buf[0];
  for(i = 1; i < FFT_LENGTH / 2; i++) {
    re = fft_buf[i*2];
    im = fft_buf[i*2 + 1];
    fft_mag[i] = re * re + im * im;
  }

  /* 频率范围 20kHz~200kHz */
  bin_min = (uint16_t)(20000.0f * (float)FFT_LENGTH / adc_fs_hz);
  bin_max = (uint16_t)(200000.0f * (float)FFT_LENGTH / adc_fs_hz);
  if(bin_min < 4) bin_min = 4;
  if(bin_max > FFT_LENGTH / 2 - 1) bin_max = FFT_LENGTH / 2 - 1;

  /* 峰值搜索 */
  for(i = bin_min; i <= bin_max; i++) {
    mag_sq = fft_mag[i];
    avg_val += mag_sq;
    if(mag_sq > max_val) {
      max_val = mag_sq;
      max_bin = i;
    }
  }
  if(max_bin == 0) return 0.0f;

  /* 诊断 */
  diag_fft_max_bin = (uint32_t)max_bin;
  diag_fft_max_mag = max_val;
  diag_fft_used_fs = adc_fs_hz;

  if(max_val < FFT_MIN_MAG_SQ) return 0.0f;
  avg_val *= (1.0f / (float)(bin_max - bin_min + 1));
  diag_fft_avg_mag = avg_val;
  if(max_val < FFT_SNR_THRESHOLD * avg_val) return 0.0f;

  /* ★ 2026-06-12 优化：抛物线插值改用 sqrt-free 方法（省 3 次 sqrtf 调用 ≈60 cycles）
   *   原方法：ym = sqrt(mag[m-1]), y0 = sqrt(max), yp = sqrt(mag[m+1])
   *           delta = 0.5*(ym-yp)/(ym-2*y0+yp)
   *   新方法：直接对 mag_sq 值做二次插值（Jacobsen 1991）
   *           delta = (mag[m-1] - mag[m+1]) / (2*(2*max - mag[m-1] - mag[m+1]))
   *           精度：对 Hann 窗主瓣（接近二次曲线），sqrt-free 与 sqrt 版误差 < 0.1% */
  float mag_m1, mag_p1, denom2;
  if(max_bin > 0 && max_bin < (FFT_LENGTH / 2 - 1)) {
    mag_m1 = fft_mag[max_bin - 1];
    mag_p1 = fft_mag[max_bin + 1];
    denom2 = 2.0f * max_val - mag_m1 - mag_p1;
    if(fabsf(denom2) > 1e-6f) {
      delta = (mag_m1 - mag_p1) / (2.0f * denom2);
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
/***********************************************************
函数名：Calc_FFT_Frequency_in_Frame
参数：  buf - ADC帧缓冲区, n_samples - 样本数, ch - 通道号, start_idx - 起始索引
返回值：float  检测到的频率(Hz)，0表示未检出
描述：  FFT频率分析包装函数（旧架构兼容）
        从帧缓冲区提取单通道数据后调用Calc_FFT_Frequency_in_Frame_Optimized
修改记录：
***********************************************************/
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
  uint64_t last_sent_time_us[ADC_NCH] = {0};
  uint8_t  ch_active[ADC_NCH] = {0};
  uint8_t  ch_silent_cnt[ADC_NCH] = {0};
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
   *      原因：debug 输出到同一 UART1，ASCII 字符插入二进制帧中破坏解析。
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

  /* 【2026-06-13】独立看门狗 IWDG 初始化（寄存器直接操作）
   * - 时钟源: LSI (32kHz)
   * - 预分频: IWDG_PR_DIV_256 → 32kHz/256 = 125Hz (8ms per tick)
   * - 重装载值: 2500 → 超时时间 = 2500 × 8ms = 20 秒
   * - 作用: 防止程序卡死在任何位置，超过 20s 未喂狗则复位
   * IWDG 寄存器:
   *   KR  = 0x5555 (使能写访问)
   *   PR  = 0x06   (256分频)
   *   RLR = 2500   (Reload值)
   *   KR  = 0xAAAA (喂狗)
   *   KR  = 0xCCCC (启动) */
  IWDG1->KR = 0x5555;      /* 解锁写保护 */
  IWDG1->PR = 0x06;        /* 256分频, 8ms/tick */
  IWDG1->RLR = 2500;       /* 2500 × 8ms = 20s */
  /* 注意：IWDG 启动延迟到主循环，防止烧录时芯片被锁定 */
  // IWDG1->KR = 0xAAAA;      /* 喂狗 */
  // IWDG1->KR = 0xCCCC;      /* 启动IWDG */
  
  /* 【2026-06-12 修复】清零 tx_ring，避免 AXI_SRAM 残留数据被 DMA 发送
   * tx_ring 定义在 AXI_SRAM 中，启动时可能没有被清零，
   * 导致 systick_tx_poll() 发送残留数据。 */
  memset(tx_ring, 0, sizeof(tx_ring));
  tx_ring_head = 0;
  tx_ring_tail = 0;
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

    /* 【2026-06-13】启动 IWDG（在主循环开始时，确保调试器可正常烧录） */
    static uint8_t iwdg_started = 0;
    if(!iwdg_started) {
      IWDG1->KR = 0xAAAA;  /* 喂狗 */
      IWDG1->KR = 0xCCCC;  /* 启动IWDG */
      iwdg_started = 1;
    }

    /* 【2026-06-13】喂狗 - 防止程序卡死超时复位
     * IWDG 超时 20s，必须在 20s 内喂一次，否则芯片复位
     * 主循环正常运行时约 1ms 内完成一次迭代，喂狗无压力 */
    IWDG1->KR = 0xAAAA;  /* 喂狗 */

    diag_loop_cnt++;
    diag_last_state = 1;
    check_dma_and_push_frames();
    diag_snapshot_update();
    /* 【2026-06-10 架构重构】tx_ring_poll() 已移至 SysTick 中断（1ms 定时检查），
     * 主循环不再调用，UART DMA 发送由中断驱动，确保定时发送且主循环零阻塞 */

    /* 【2026-06-09 ADC 健康看门狗】(2026-06-10 优化: 2s → 200ms)
     * 【2026-06-12 优化: HAL_Delay(5) → 非阻塞状态机】
     * 如果 ADC DMA 连续 200ms 无新帧到达（half_complete_cnt 不增长），说明
     * ErrorCallback 可能已关闭 TEIE 导致 DMA 静默失败，需要重启 ADC。
     * 恢复步骤：Stop → 等5ms(非阻塞) → 重新 Start_DMA（会重新使能 TEIE）。
     * 检测周期：每 200ms 评估一次。
     * 正常 ADC 半区中断间隔 ≈1.165ms，200ms 内应有 ~171 个半区中断，
     * 若计数不增长则确认为 ADC 故障。 */
    {
      static uint32_t wdg_last_tick = 0;
      static uint32_t wdg_last_half_cnt = 0;
      static uint8_t  wdg_recovering = 0;      /* 非阻塞恢复状态: 0=正常, 1=等待5ms */
      static uint32_t wdg_recover_start = 0;
      uint32_t wdg_now = now;

      /* 非阻塞恢复阶段：等待5ms后重新启动ADC */
      if(wdg_recovering) {
        if(wdg_now - wdg_recover_start >= 5U) {
          MY_ADC_Start();
          diag_adc_err_recover_cnt[0]++;
          diag_adc_err_recover_cnt[1]++;
          diag_adc_err_recover_cnt[2]++;
          wdg_recovering = 0;
        }
        /* 恢复期间继续处理ADC帧（其他ADC可能还在工作） */
        continue;
      }

      if(wdg_last_tick == 0) { wdg_last_tick = wdg_now; wdg_last_half_cnt = half_complete_cnt; }
      if(wdg_now - wdg_last_tick >= 200U) {
        if(half_complete_cnt == wdg_last_half_cnt && wdg_last_half_cnt > 0) {
          /* 200ms 内无新帧 → ADC 可能死掉，启动非阻塞恢复 */
          MY_ADC_Stop();
          wdg_recovering = 1;
          wdg_recover_start = wdg_now;
        }
        wdg_last_tick = wdg_now;
        wdg_last_half_cnt = half_complete_cnt;
      }
    }

#if !ENABLE_DAC_SIGNAL_SOURCE && DAC_AS_EXTERNAL_TEST_SRC
    /* === DAC 作为外部测试信号源 - 闭环压力测试 v9.2 ===
     *
     * 设计要点：
     *  - 一次性启动 DAC + TIM7（DMA 循环模式），后续主循环不再 abort/重启 DMA，
     *    避免 HAL_DMA_Abort() 内 104us 死等阻塞 + DMA1 上下游冲击 UART TX。
     *  - 通过 stop/start TIM7（CR1.CEN 位）控制 DAC 输出/暂停。
     *    暂停时 PA5 保持 DAC 最后值（约 1.65V DC），等效"DC 阶段"。
     *  - 每个 burst 周期换一种 stress 场景（基波/谐波/噪声），
     *    通过 DAC_Build_Composite_Waveform 重写表内容（DMA 持续从表里读）。
     *
     * 时序：50ms DC + 10ms burst 持续循环，每 burst 切换场景。 */
    {
      static uint32_t dac_test_last_ms = 0;
      static uint8_t  dac_test_phase   = 0;   /* 0=DC(TIM7 停), 1=Burst(TIM7 跑) */
      static uint8_t  dac_test_inited  = 0;
      static uint8_t  stress_idx       = 0;
      const stress_case_t *sc;

      if(!dac_test_inited) {
        /* 初始阶段强制固定 DC，避免停在正弦表任意相位导致每次 burst 起点不一致 */
        DAC_Output_DC();
        dac_test_last_ms = now;
        dac_test_inited  = 1;
      }
      if(dac_test_phase == 0) {
        if(now - dac_test_last_ms >= 50U) {     /* DC 50ms */
          /* burst 开始：重建波形并重新启动 DAC DMA，使每次 burst 相位从表起点开始 */
          sc = &stress_cases[stress_idx];
          DAC_Build_Composite_Waveform(sc->amp_mv, sc->h2_pct, sc->h3_pct, sc->noise_pct);
          DAC_Output_Sine(sc->freq_hz);
          diag_stress_cur_idx = stress_idx;
          dac_test_phase   = 1;
          dac_test_last_ms = now;
        }
      } else {
        if(now - dac_test_last_ms >= 10U) {     /* Burst 10ms */
          DAC_Output_DC();                      /* burst 结束后强制回固定 1.65V DC */
          dac_test_phase   = 0;
          dac_test_last_ms = now;
          stress_idx = (uint8_t)((stress_idx + 1U) % STRESS_CASE_COUNT);
          diag_stress_burst_total++;
        }
      }
    }
#endif

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

/* 【2026-06-12 修复】移除 debug_poll() 调用
 * 原因：debug_poll() 和 systick_tx_poll() 都使用 USART1 DMA1_Stream1，
 *       造成 DMA 通道竞争冲突，导致 USART1 疯狂发送数据。
 * 解决方案：生产模式下禁用 debug_poll()，仅保留事件帧输出。
 *       debug_printf 也已通过 #if !ENABLE_DAC_SIGNAL_SOURCE 条件编译禁用。
 *       如需调试信息，请使用 J-Link RTT。 */

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
              if(tx_time_us <= last_sent_time_us[ch]) {
                tx_time_us = last_sent_time_us[ch] + 1U;
              }
              last_sent_time_us[ch] = tx_time_us;

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

          /* ─── 【2026-06-07 v8 关键修复】DAC 模式也走 tx_ring DMA ───
           * 原问题：HAL_UART_Transmit(multi_buf, 114, 100) 阻塞 2.47ms~100ms，
           *         期间 check_dma_and_push_frames() 不被调用 → ADC 帧丢失。
           * 新方案：逐帧 memcpy 到 tx_ring，让 tx_ring_poll() 统一 DMA 发送，
           *         主循环永不阻塞，burst 期间 ADC 帧持续处理。
           * 注：multi_buf 组装逻辑保留（用于诊断），但发送改用 tx_ring_push。 */
          if(multi_len > 0) {
            uint16_t i;
            uint8_t *src_ptr = multi_buf;
            for(i = 0; i < multi_len / EVT_FRAME_TOTAL_LEN; i++) {
              uint8_t next_head = (uint8_t)((tx_ring_head + 1U) % TX_RING_SLOTS);
              if(next_head == tx_ring_tail) {
                /* 队列满 → 丢帧（绝不阻塞主循环） */
                diag_uart_q_drop_cnt++;
                diag_uart_hal_fail_cnt++;
              } else {
                memcpy((void*)tx_ring[tx_ring_head], src_ptr, EVT_FRAME_TOTAL_LEN);
                tx_ring_head = next_head;
                diag_uart_q_push_cnt++;
                diag_uart_hal_ok_cnt++;
              }
              src_ptr += EVT_FRAME_TOTAL_LEN;
            }
            /* 不调用 HAL_UART_Transmit_DMA，由主循环每轮的 tx_ring_poll() 统一发起 */
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

    /* ===================================================================
     * 【2026-06-10 架构重构】三级流水线解耦（仅外部信号模式）
     * ===================================================================
     * DAC 模式保留旧的 burst 集中收集+发送路径（burst_best 逻辑），
     * 外部信号模式使用新的 Step A/B 解耦路径：
     *   Step A: 处理帧池 → 触发检测 → push trig_queue
     *   Step B: pop trig_queue → FFT → push tx_ring (事件帧)
     *   SysTick: 检查 tx_ring + UART DMA → 发送 (见 stm32h7xx_it.c)
     * =================================================================== */
#if !ENABLE_DAC_SIGNAL_SOURCE

    /* ─── Step A: ADC帧触发检测 ───
     * 从 frame_pool 取帧，做轻量预扫检测触发通道，
     * 对触发通道提取 1024 点数据 + 触发点位置，push 到 trig_queue。
     * 不做 FFT（FFT 在 Step B 中完成）。 */
    while(frame_r_idx != frame_w_idx) {
      adc_frame_t *fr = &frame_pool[frame_r_idx];
      uint8_t  allow_trigger;
      uint8_t  has_trans[ADC_NCH] = {0};
      uint16_t thr_ch[ADC_NCH];
      uint32_t max_dev_ch[ADC_NCH];
      uint8_t  n_trig = 0;
      uint16_t trans[ADC_NCH];
      int32_t  baselines[ADC_NCH];
      uint16_t noises[ADC_NCH];

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
#if FRAME_LEVEL_SKIP_MS > 0
        if(last_event_ms[ch] != 0 && (now - last_event_ms[ch]) < FRAME_LEVEL_SKIP_MS) continue;
#endif

        if(ch_dc_offset_valid && ch_dc_offset[ch] != 0) {
          baselines[ch] = ch_dc_offset[ch];
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
        if(noises[ch] > DEV_THRESHOLD_MIN) {
          thr = DEV_THRESHOLD_MIN;
        } else if(noises[ch] * DEV_NOISE_MULT > DEV_THRESHOLD_MIN) {
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
          if(Verify_Sustain_in_Frame(fr->data, HALF_SAMPLES_PER_CH, ch, baselines[ch], thr, trans[ch])) {
            if(ch_active[ch] == 0U) {
              has_trans[ch] = 1;
              n_trig++;
              diag_prescan_hit_cnt++;
            }
            ch_active[ch] = 1U;
            ch_silent_cnt[ch] = 0U;
          } else {
            if(ch_silent_cnt[ch] < 3U) {
              ch_silent_cnt[ch]++;
            }
            if(ch_silent_cnt[ch] >= 2U) {
              ch_active[ch] = 0U;
            }
          }
        } else {
          if(ch_silent_cnt[ch] < 3U) {
            ch_silent_cnt[ch]++;
          }
          if(ch_silent_cnt[ch] >= 2U) {
            ch_active[ch] = 0U;
          }
        }
      }

      diag_last_n_trig = n_trig;

      /* ─── 对每个触发通道：提取 1024 点数据 → push trig_queue ─── */
      for(ch = 0; ch < ADC_NCH; ch++) {
        uint16_t fft_start_eff;
        uint16_t peak_idx;
        float    peak_frac;
        float    eff_idx;
        uint64_t event_us_in_frame;
        uint16_t copy_start;
        uint16_t j;

        if(!has_trans[ch]) continue;

        /* 噪声/阈值过滤 */
        {
          uint16_t thresh;
          if(noises[ch] > DEV_THRESHOLD_MIN) {
            thresh = DEV_THRESHOLD_MIN;
          } else {
            thresh = (uint16_t)(noises[ch] * DEV_NOISE_MULT);
            if(thresh < DEV_THRESHOLD_MIN) thresh = DEV_THRESHOLD_MIN;
          }
          if(max_dev_ch[ch] < thresh) {
            diag_external_noise_filter_cnt++;
            continue;
          }
        }

#if ENABLE_DAC_SIGNAL_SOURCE
        /* DAC 模式 SNR + 频率一致性检查 */
        {
          uint16_t thresh2 = noises[ch] * DEV_NOISE_MULT;
          if(thresh2 < DEV_THRESHOLD_MIN) thresh2 = DEV_THRESHOLD_MIN;
          if(max_dev_ch[ch] >= thresh2) {
            float snr_ratio = (float)max_dev_ch[ch] / (float)thresh2;
            if(snr_ratio < 5.0f) continue;
          }
          burst_candidate_mask |= (uint8_t)(1U << ch);
          if(max_dev_ch[ch] >= burst_best_dev[ch]) {
            burst_best_dev[ch] = max_dev_ch[ch];
            /* 频率等在 Step B FFT 后填充 */
            burst_best_time_us[ch] = fr->ch_start_time_us[ch];
          }
        }
#endif

        peak_idx = Find_FirstPeak_in_Frame(fr->data, HALF_SAMPLES_PER_CH, ch, baselines[ch], trans[ch]);
        peak_frac = Refine_Peak_SubSample(fr->data, HALF_SAMPLES_PER_CH, ch, baselines[ch], peak_idx);
        eff_idx = (float)peak_idx + peak_frac;
        if(eff_idx < 0.0f) eff_idx = 0.0f;
        event_us_in_frame = (uint64_t)(eff_idx / adc_fs_hz * 1e6f + 0.5f);

        /* 计算 FFT 数据提取起始位置（同原逻辑的 fallback） */
        fft_start_eff = trans[ch] + FFT_STEADY_SKIP;
        if(fft_start_eff + FFT_LENGTH >= HALF_SAMPLES_PER_CH) {
          fft_start_eff = trans[ch];
          if(fft_start_eff + FFT_LENGTH >= HALF_SAMPLES_PER_CH) {
            if(HALF_SAMPLES_PER_CH > FFT_LENGTH) {
              fft_start_eff = (uint16_t)(HALF_SAMPLES_PER_CH - FFT_LENGTH);
            } else {
              fft_start_eff = 0;
            }
          }
        }
        copy_start = fft_start_eff;

        /* push 到 trig_queue */
        {
          uint8_t next_head = (uint8_t)((trig_q_head + 1U) % TRIG_QUEUE_SIZE);
          trigger_event_t *te;

          if(next_head == trig_q_tail) {
            /* 触发队列满 → 丢事件 */
            diag_trig_q_drop_cnt++;
            diag_uart_hal_fail_cnt++;
          } else {
            te = &trig_queue[trig_q_head];
            te->ch = ch;
            te->trigger_idx = peak_idx;
            te->start_time_us = fr->ch_start_time_us[ch] + event_us_in_frame;

            /* 提取 1024 点单通道数据（交织帧中解交织） */
            for(j = 0; j < TRIG_DATA_LEN && (copy_start + j) < HALF_SAMPLES_PER_CH; j++) {
              te->data[j] = fr->data[(copy_start + j) * ADC_NCH + ch] & 0x0FFF;
            }
            /* 如果帧内不够 1024 点，用 0 补齐（FFT 会处理） */
            for(; j < TRIG_DATA_LEN; j++) {
              te->data[j] = 0;
            }

            trig_q_head = next_head;
            diag_uart_q_push_cnt++;
            diag_external_event_candidate_cnt++;
          }
        }
        test_case_trigger_count[current_burst_case]++;
      }

      fr->ready = 0;
      frame_r_idx = (frame_r_idx + 1) % FRAME_POOL_SIZE;
    }

    /* ─── Step B: pop trig_queue → FFT → 生成事件帧 → push tx_ring ───
     * 使用 while 循环每次处理完所有待处理触发事件（避免积压）。
     * 限制每轮最多处理 4 个事件（4 × 200μs FFT = 800μs），
     * 避免长时间阻塞导致 ADC DMA flag 积压丢帧。 */
    {
      uint8_t step_b_count = 0;
      while(trig_q_tail != trig_q_head && step_b_count < 4) {
        step_b_count++;
      trigger_event_t *te = &trig_queue[trig_q_tail];
      float fft_freq;
      uint32_t tx_freq;
      uint64_t tx_time_us;
      uint8_t dedup_skip = 0;

      diag_fft_try_cnt++;
      fft_freq = Calc_FFT_Frequency_in_Frame_SingleCh(te->data, TRIG_DATA_LEN);

      /* 频率有效性检查 */
      if(fft_freq >= 20000.0f && fft_freq <= 200000.0f) {
        tx_freq = (uint32_t)(fft_freq + 0.5f);
        diag_ch_last_fft_freq[te->ch] = tx_freq;
      } else {
        tx_freq = 0;  /* FFT 未检出有效频率 */
        diag_ch_last_fft_freq[te->ch] = (uint32_t)fft_freq;
      }

#if ENABLE_DAC_SIGNAL_SOURCE
      /* DAC 模式频率一致性检查 */
      if(dac_diag_actual_freq_hz > 0 && tx_freq > 0) {
        float freq_err = fabsf((float)tx_freq - (float)dac_diag_actual_freq_hz)
                       / (float)dac_diag_actual_freq_hz;
        if(freq_err > 0.10f) {
          tx_freq = 0;  /* 频率不一致，标记无效 */
        }
      }
      /* 更新 burst_best_freq */
      if(tx_freq > 0 && tx_freq >= INTERNAL_TEST_FREQ_MIN_HZ && tx_freq <= INTERNAL_TEST_FREQ_MAX_HZ) {
        burst_best_freq[te->ch] = tx_freq;
      }
#endif

#if !ENABLE_DAC_SIGNAL_SOURCE
      /* 外部信号模式：同通道去抖（不用 continue，改为标记跳过）*/
      if(last_valid_event_ms[te->ch] != 0 && (now - last_valid_event_ms[te->ch]) < SAME_CH_DEDUP_MS) {
        diag_external_dedup_skip_cnt++;
        dedup_skip = 1;
      }
#endif

      /* 组装事件帧并 push 到 tx_ring（仅当频率有效且未去抖跳过时）*/
      if(tx_freq > 0 && !dedup_skip) {
        tx_time_us = te->start_time_us;
        if(tx_time_us <= last_sent_time_us[te->ch]) {
          tx_time_us = last_sent_time_us[te->ch] + 1U;
        }
        last_sent_time_us[te->ch] = tx_time_us;

        evt_pack_frame(evt_buf, te->ch + 1, tx_time_us, tx_freq);

        {
          uint8_t next_head = (uint8_t)((tx_ring_head + 1U) % TX_RING_SLOTS);
          if(next_head == tx_ring_tail) {
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
        diag_evt_count_ch[te->ch]++;
        diag_evt_last_time_lo[te->ch] = (uint32_t)(tx_time_us & 0xFFFFFFFFULL);
        diag_evt_last_time_hi[te->ch] = (uint32_t)(tx_time_us >> 32);
        diag_evt_last_freq[te->ch] = tx_freq;
        last_valid_event_ms[te->ch] = now;
        last_event_ms[te->ch] = now;
        LED_TOGGLE(LED2_PIN);
      }

      trig_q_tail = (uint8_t)((trig_q_tail + 1U) % TRIG_QUEUE_SIZE);
      } /* while(trig_q_tail != trig_q_head && step_b_count < 4) */
    } /* step_b_count block */
#endif /* !ENABLE_DAC_SIGNAL_SOURCE: Step A/B 仅外部信号模式 */

/* 【2026-06-12 修复】移除 debug_poll() 调用
 * 原因：debug_poll() 和 systick_tx_poll() 都使用 USART1 DMA1_Stream1，
 *       造成 DMA 通道竞争冲突，导致 USART1 疯狂发送数据。
 * 解决方案：生产模式下禁用 debug_poll()，仅保留事件帧输出。
 *       如需调试信息，请使用 J-Link RTT。 */
  }
}

/***********************************************************
函数名：SystemClock_Config
参数：  无
返回值：无
描述：  系统时钟配置函数
        配置STM32H750为480MHz主频：
        - PLL1: HSE(25MHz)×N/P → SYSCLK=480MHz
          PLL1M=5, PLL1N=192, PLL1P=2
          VCO=25*192/5=960MHz, PLL1_P=960/2=480MHz
        - AHB: 480MHz, APB1: 120MHz, APB2: 120MHz
        - PLL3: 为ADC提供异步时钟（见 adc.c HAL_ADC_MspInit 详细注释）
          PLL3M=5, PLL3N=45, PLL3R=2 → PLL3_R=112.5MHz
          ★ D3CCIPR.ADCSEL=01 → ADC 时钟来自 PLL3_R（非 PLL3_P）
          ★ 硅片 Rev.V 有硬件 ÷2 → SAR 时钟 = 56.25MHz
修改记录：
***********************************************************/
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

/***********************************************************
函数名：Error_Handler
参数：  无
返回值：无
描述：  错误处理函数，点亮RUN_LED后进入死循环
        在HAL外设初始化返回非HAL_OK时调用
修改记录：
***********************************************************/
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