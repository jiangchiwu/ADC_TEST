# Draft: ADC_FFT_TEST 工程代码优化

## Original Request
> 检查 F:\ADC_FFT\Myproject\ADC_FFT_TEST 工程代码，提出优化建议

## Intent Classification
- **Type**: Mixed — likely **Refactoring + Research** intent
- **Reasoning**: 用户请求是开放式 "提出优化建议"，需要先盘点现状，再缩小范围
- **Strategy**: 先并行 explore 摸底 → 反向访谈确认优化优先级 → 形成 ONE 单一计划

## Project Snapshot (initial scan)
- **MCU**: STM32H750 (Cortex-M7 @ 480 MHz, with D-Cache/I-Cache)
- **Toolchain**: Keil MDK-ARM (.uvprojx); 同时有 CubeMX .ioc 文件
- **Domain**: ADC 采样 + CMSIS-DSP FFT 频谱分析
- **Core 源文件** (≈12 个): main.c, test.c, fft_backup.c, adc.c, tim.c, dac.c, usart.c, debug.c, gpio.c, stm32h7xx_it.c, system_stm32h7xx.c, stm32h7xx_hal_msp.c
- **DSP**: CMSIS-DSP（rfft_fast / cfft / radix2 / radix8 / bitreversal）
- **没有 CMakeLists.txt** — 纯 MDK 工程

## Observed Red Flags (待 explore 确认)
1. **根目录污染严重** (~80+ 文件)
   - ~30 个 .jlink 脚本 (download_*, flash_*, verify_* 等大量重复)
   - 10+ 个 serial_log_*.txt（疑似临时调试日志）
   - 10+ 个 verify_*.txt 和 bench_*.txt
   - 多份重叠 .md 报告 (FINAL_ANALYSIS_REPORT / TEST_REPORT_FINAL / TEST_RESULTS / WorkLog ...)
2. **可疑文件 `fft_backup.c`** — 命名暗示是手动备份，应该删除或归档
3. **缺乏构建/Git 卫生** — 没看到 .gitignore，大量临时产物未被忽略

## Research In Progress
- [ ] explore-1: 核心应用代码深度分析（架构/性能/坏味/Bug）— BG dispatched
- [ ] explore-2: 工程卫生与产物文件分类（保留/归档/删除）— BG dispatched

## Round 1 Interview Results (确认)

### 用户回答
| 维度 | 选择 | 解读 |
|---|---|---|
| 优化方向 | **全部 6 个**（工程整洁/代码质量/性能/构建/测试/文档） | 全景式优化 |
| 激进度 | **保守模式：只删/移/改注释，不动可执行代码** | **CRITICAL: 任何任务都不能修改可执行逻辑** |
| 验证手段 | 硬件在环 + 静态验证 + RTT + 顺便建设新系统 | 多管齐下 |
| 历史产物处置 | **只标注 OBSOLETE，不动文件** | **CRITICAL: 不允许 delete/move/archive** |

### ⚠️ 张力分析（必须 Round 2 澄清）
1. **"全部 6 方向" vs "不动可执行代码"** → 性能优化/代码质量/构建系统的大部分任务**不可能在不动可执行代码的前提下完成**。
2. **"全部 6 方向" vs "不动文件"** → 工程整洁也不能动文件，那"整洁"只能靠注释/README 来表达？
3. **构建系统现代化** → 这必须修改 MDK-ARM/CMakeLists 类文件，与"不动可执行代码"是否冲突？

### ⚠️ 重新解读用户意图（hypothesis）
保守模式可能是在说：**"先给我一份计划，但任何动手都要分阶段、有评审"**，而不是真的"什么都不能动"。
也可能是：**"本次规划聚焦在最安全的部分（清理 + 文档 + 注释 + 标注），后续再做激进改动"** → 这是合理且常见的渐进策略。

需要 Round 2 锁定真实意图。

## Key Findings From Initial Scan (in main.h & event_frame.h)
### Working System Confirmation
- **项目已经运转**：v3.0 二进制事件帧格式 + DWT 时间戳 + 460800baud UART7 上送
- **6 通道 ADC 实时方案**：PC0/PC1/PC2/PC3/PA3/PA4 三组 ADC × 2 通道，AXI_SRAM
- **DMA 缓冲已正确对齐 32B**（adc.h:60-62: `__attribute__((section(".AXI_SRAM"), aligned(32)))`），D-Cache 友好
- **debug 模块设计成熟**：环形缓冲 + 非阻塞 push + ENABLE_DEBUG_LOG 开关（避免 ASCII 干扰二进制帧）

### Issues Already Visible
1. **README 严重过时** — README.md 只有 3 行 "ADC test, read Vrefint"，与实际 6 通道实时 FFT 系统完全不符
2. **main.h 的 `CURRENT_TEST_MODE = TEST_MODE_PWM`** — 默认走的是 PWM 测试模式，不是生产路径，疑似遗留
3. **存在 fft_backup.c** — 命名暗示是手动备份；正式项目不应该有 _backup 文件
4. **根目录污染**：~30 个 .jlink 脚本（download_30_50, download_safe, download_test, download_v2, download_recover, just_go, restart_board, run_board, start_board, reset_only ... 大量功能重复）
5. **历史日志/产物未清理**：bench_FINAL_STABLE / bench_FLASH_baseline / bench_FLASH_FINAL / bench_flash_v2/v3 / bench_FLASH_v4 / verify_fft_v5..v8 / serial_log_diag/diag2/diag3/diag4 / TEST_REPORT_FINAL / TEST_RESULTS / TEST_LOG_20260525_0130 ...
6. **多份重叠 .md**：FINAL_ANALYSIS_REPORT / TEST_ANALYSIS_REPORT / TEST_PLAN / TEST_REPORT_FINAL / TEST_RESULTS / TEST_LOG_REALTIME / WorkLog / STM32_ADC_FFT_开发指南 — 真相分散，迭代过程留下太多快照
7. **两份 GDB cfg**：jlink_debug.cfg + stlink_debug.cfg 同时存在
8. **CubeMX 与手写代码并存**：07-ADC_Test.ioc 在根目录，但 adc.h 注释表明 ADC 实际配置已被手动重写（如 PC2/PC3 故意分给 ADC3）→ 重新生成会破坏代码

## Pending Deep Analysis (BG tasks running)
- bg_d1562b97 (核心代码分析) - 等待完成
- bg_2fb60307 (工程卫生分析) - 等待完成

## Scope Boundaries (TBD)
- INCLUDE: 待确认
- EXCLUDE: HAL/CMSIS 库代码不动（除非配置层面）

## Technical Decisions
- 待 explore 结果回来后再敲定

## Test Strategy Decision
- **Infrastructure exists**: 待确认（embedded 项目通常需要硬件在环）
- **Automated tests**: 待用户回答
- **Agent-Executed QA**: ALWAYS（即使硬件相关，也可以靠串口/RTT 日志 + Python 脚本验证）
