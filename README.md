# STM32H750 6 通道实时 ADC + FFT 事件帧上送系统

> **状态**：稳定运行（2026 年迭代至 v3.0 事件帧协议）
> **目标硬件**：STM32H750VBT6（WeAct Studio 北极星板）
> **目标 PC**：Windows + CH340 USB-Serial @ 460800 baud
> **维护责任人**：待填

---

## 一句话功能描述

实时检测 6 路独立 ADC 通道（PC0/PC1/PC2/PC3/PA3/PA4）从 DC 跳变到 AC burst 的触发事件，
通过 CMSIS-DSP RFFT 1024 点计算基波频率，组装 19 字节二进制事件帧（含 DWT 微秒时间戳）
通过 UART7 上送到 PC 上位机。

---

## 关键参数（来自 main.c 头注释）

| 参数 | 值 |
|---|---|
| MCU 主频 | 480 MHz |
| ADC 分辨率 | 12 bit |
| ADC 单通道采样率（实测） | 3.515 MSPS |
| FFT 点数 | 1024 |
| FFT 单次耗时 | 200 μs |
| 事件帧长度 | 19 字节固定 |
| UART 波特率 | 460800 |
| 最大事件率 | ~120 evt/s（受 CH340 USB Bulk 包合包速率限制）|

---

## 目录结构

| 路径 | 用途 |
|---|---|
| Core/Src/main.c | 主程序：事件检测主循环、UART 环形队列、FFT 调用、DAC burst 控制 |
| Core/Src/adc.c | ADC1/2/3 + DMA 初始化与启动 |
| Core/Src/tim.c | TIM7（DAC 触发）、TIM8（ADC 触发）|
| Core/Src/dac.c | DAC1/2 + 正弦表（仅 ENABLE_DAC_SIGNAL_SOURCE=1 时使用） |
| Core/Src/debug.c | RTT + UART debug 输出（生产模式下禁用 ASCII 干扰） |
| Core/Src/usart.c | UART7 + DMA1_Stream0 配置 |
| Core/Inc/event_frame.h | 19B 事件帧协议定义（与 PC 端 verify_event_system.py 强耦合）|
| Middlewares/CMSIS/DSP | CMSIS-DSP RFFT 库（来自 ST）|
| MDK-ARM/ | Keil 工程文件 |
| docs/ | 工程文档（含 ARCHIVE_INDEX、CODE_REVIEW_NOTES）|
| STM32_ADC_FFT_开发指南.md | 开发详细说明书 |

---

## 快速开始

### 烧录与运行
```
# 用 J-Link 烧录
JLink.exe -CommanderScript download.jlink

# 抓串口
python serial_capture.py --port COM3 --baud 460800
```

### PC 端验证
脚本位于独立仓库：`F:/ADC_FFT/test_scripts/verify_event_system.py`
打包好的可执行：`F:/ADC_FFT/test_scripts/dist/verify_event_system.exe`

---

## 测试模式开关（main.c 顶部宏）

| 宏 | 默认值 | 含义 |
|---|---|---|
| ENABLE_DAC_SIGNAL_SOURCE | 0 | 1=板内 DAC 闭环自检；0=外部独立信号源 |
| ENABLE_UART_SELF_TEST    | 0 | 1=固定 2Hz 发测试帧，验证 UART 链路 |
| ENABLE_BOOT_SELF_TEST    | 0 | 1=上电每通道发 1 帧自检帧 |
| ENABLE_DMA_LOOPBACK_TEST | 0 | 1=DMA 链路压测 |
| UART_STRESS_RATE_HZ      | 0 | >0 = UART 极限压测速率 |

---

## 协议、报告、历史

- 事件帧 v3.0 协议详见 [Core/Inc/event_frame.h](Core/Inc/event_frame.h)
- 历史/废弃文件登记 → [docs/ARCHIVE_INDEX.md](docs/ARCHIVE_INDEX.md)
- 代码评审备注 → [docs/CODE_REVIEW_NOTES.md](docs/CODE_REVIEW_NOTES.md)
- 开发详细指南 → [STM32_ADC_FFT_开发指南.md](STM32_ADC_FFT_开发指南.md)

---

## 已知限制

1. **PA4 双重用途**：PA4 既是 DAC1_OUT1 也是 ADC2 INP18。仅在 ENABLE_DAC_SIGNAL_SOURCE=1 时同时使用，正式模式下 DAC 关闭，PA4 仅为 ADC 输入。
2. **采样率实测 vs 理论不符**：实测 3.515 MSPS，理论 2.009 MSPS（PLL3P=56.25MHz/2/14）。FFT 用实测值 `adc_fs_hz = 3515000.0f` 进行频率换算。
3. **事件率上限 ~120 evt/s**：CH340 + Windows + pyserial 链路限制。
4. **UART 不可同时输出 ASCII 调试**：与二进制事件帧冲突；ENABLE_DEBUG_LOG=0 时所有 debug_printf 被预处理器替换为 no-op。

---

## 许可

代码使用 STMicroelectronics BSD 3-Clause（继承自 HAL 模板）。
