# ADC_FFT_TEST 工程优化建议（推荐书）

> **本文件性质**：**纯建议文档**，不含可执行任务。Prometheus（规划顾问）输出，供你/他人后续按需挑选实施。
> **本次范围**：仅"零风险"建议（清理、文档、注释、标注），不涉及修改可执行代码。
> **激进改动（性能/架构/协议/测试自动化）**列入文档末尾 **Section E: Deferred Backlog**，供以后立项。

---

## TL;DR

> **项目现状**：一个**已经稳定运行**的 6 通道 STM32H750 实时 ADC+FFT 上送系统（v3.0 二进制事件帧、AXI_SRAM DMA、CMSIS-DSP RFFT 1024 点、UART7 460800 baud）。功能正确、性能达标，但**工程卫生严重恶化**：根目录被 30+ .jlink 脚本、10+ 串口日志、10+ benchmark txt、6 份重叠测试报告 .md、`fft_backup.c` 备份文件、严重过时的 README 淹没。`main.c` 单文件超过 3000 行，含 70+ 全局 `diag_*` 调试变量、3 套并存的 UART 发送路径、多处历史 `#if 0` 测试开关。
>
> **核心矛盾**：代码本身已经"能用"，但**新人接手或自己半年后回看几乎不可能快速定位真相**。
>
> **本次建议（零风险）**：
> - 建立 `docs/ARCHIVE_INDEX.md` 集中登记所有"历史/疑似废弃"文件的状态
> - 重写 `README.md`（当前 3 行，与实际系统完全不符）
> - 在每个疑似废弃文件顶部添加标准化 `STATUS:` 头注释
> - 合并 6 份重叠的测试报告 .md 为单一 `docs/TEST_HISTORY.md`（保留原始文件作为引用）
> - 在 `main.c`、`debug.c`、`adc.c` 顶部补齐/规范化文件级 doxygen 注释
> - 在 `main.h` 中标注 `CURRENT_TEST_MODE = TEST_MODE_PWM` 与生产路径的语义不一致问题（不改宏值，仅加注释）
> - 列出 ~50 个可考虑加入 `.gitignore` 的产物（仅作为建议清单，不写文件）
>
> **本次不做**：删/移文件、改可执行代码、改 `.uvprojx`、改 `.ioc`、改 `arm_cfft_*` 等 CMSIS 库代码。
>
> **预估接手工作量**：1 名熟悉项目的工程师 4-6 小时即可完成 Section A-D 所有建议项。

---

## Context

### 项目背景
- **MCU**：STM32H750VBT6 @ 480 MHz（Cortex-M7 with I-Cache/D-Cache）
- **域**：实时 6 通道 ADC + DMA 双缓冲 + CMSIS-DSP RFFT 1024 点 + 二进制事件帧 UART 上送
- **状态**：已通过端到端硬件在环验证（实测 6 通道无丢帧，120 evt/s 稳定）
- **构建**：Keil MDK-ARM（`.uvprojx`），同时残留 CubeMX `.ioc` 文件（手写代码已偏离生成器）
- **PC 端**：`F:/ADC_FFT/test_scripts/verify_event_system.py`（独立仓）

### 关键发现快照（驱动本建议书的事实依据）
| 维度 | 现状 | 影响 |
|---|---|---|
| 根目录文件数 | 92 个条目 | 信噪比极低 |
| .jlink 脚本数 | ~30 个 | 大量功能重复（download_v2/v3/v4/safe/recover/temp/test...）|
| 串口日志数 | 10+ 个 `serial_log_*.txt` | 临时调试产物从未清理 |
| benchmark 数 | 8+ 个 `bench_*.txt` | 历次实验快照混在源码同级 |
| verify 数 | 9+ 个 `verify_*.txt` | 同上 |
| 测试报告 .md 数 | 6 份（FINAL_ANALYSIS_REPORT / TEST_ANALYSIS_REPORT / TEST_PLAN / TEST_REPORT_FINAL / TEST_RESULTS / TEST_LOG_REALTIME） | 真相分散，无单一权威来源 |
| README.md | 仅 3 行 "ADC test, read Vrefint" | **与实际系统完全不符** |
| `fft_backup.c` | 存在 | 命名暗示是手动备份，非标准实践 |
| `main.c` 行数 | 估 ≥ 3000 行（前 1239 行已超 50KB） | 单一职责严重违背 |
| 全局 `diag_*` 变量数 | 70+ 个（line 300-390, 595-605, 762-765 etc.）| 调试遗留遍布，重构高风险 |
| UART 发送路径 | 3 套并存（`evt_tx_queue_push/poll`、`tx_ring_poll` DMA、HAL_UART_Transmit 直接）| 注释说 v6 已切到 DMA，但旧路径未删 |
| 测试开关默认值 | `ENABLE_DAC_SIGNAL_SOURCE=0`、`ENABLE_UART_SELF_TEST=0`、`ENABLE_BOOT_SELF_TEST=0`、`UART_STRESS_RATE_HZ=0`、`ENABLE_DMA_LOOPBACK_TEST=0` | 多个开关已不再使用但保留 |
| `main.h::CURRENT_TEST_MODE` | `= TEST_MODE_PWM`（line 87）| **与生产路径语义不一致**：main.c 实际走 6 通道 ADC+FFT，TEST_MODE_PWM 看起来像调试遗留 |
| 调试器配置 | `jlink_debug.cfg` + `stlink_debug.cfg` 并存 | 实际只用 J-Link？ |
| CubeMX `.ioc` | `07-ADC_Test.ioc` 在根目录 | 重新生成会破坏手写代码（adc.h 注释证实手写）|

### 用户决策摘要（来自 Round 1 + Round 2 访谈）
- **方向**：覆盖工程整洁/代码质量/性能/构建/测试/文档共 6 个方向，**但本次仅做零风险子集**
- **激进度**：保守模式 —— **不动可执行代码**
- **文件处置**：**原地加头注释 + 在 `docs/ARCHIVE_INDEX.md` 集中登记**，不删不移
- **验证手段**：硬件在环 + 串口抓帧 + RTT + Python 解析（多手段并行）
- **最终澄清**：用户只要建议文档，实际工作交给别人

---

## 建议的目录结构（本次新增）

```
ADC_FFT_TEST/
├── docs/                                   ★ 新增目录
│   ├── ARCHIVE_INDEX.md                    ★ 历史/废弃文件登记表
│   ├── TEST_HISTORY.md                     ★ 6 份测试报告合并版
│   ├── CODE_REVIEW_NOTES.md                ★ 代码评审备注（含 main.h test mode 异议、UART 三套路径冗余、CMakeLists 缺失等）
│   ├── GITIGNORE_CANDIDATES.md             ★ 推荐 gitignore 清单（仅建议，不写 .gitignore）
│   └── DEFERRED_BACKLOG.md                 ★ 后续可选优化项的详细论证
└── (其它一切保持原样)
```

**为什么用 `docs/`**：
- 与 `Core/`、`Drivers/`、`Middlewares/`、`MDK-ARM/` 平级，符合 STM32 CubeMX 工程惯例
- 不污染 Core/Inc/Src（避免影响 Keil 工程文件包含路径）
- 与 `STM32_ADC_FFT_开发指南.md` 等根目录孤立 .md 文件区分（后者后续可移入 `docs/`）

---

## 建议分级说明

每条建议附带以下标签，便于实施者快速分类：

- **【P0】关键** —— 影响新人理解、可读性，应优先做
- **【P1】重要** —— 显著提升工程整洁度
- **【P2】常规** —— 锦上添花
- **【风险:零】** —— 仅注释/新文件，不动现有可执行代码
- **【风险:低】** —— 改注释或非编译文件（如 .md、.cfg）
- **【可回滚】** —— Git revert 即可完整恢复

---

## Section A：工程卫生（根目录清理）

> **目标**：让一个新人打开根目录，能在 30 秒内分清"活的"和"死的"文件。
> **原则**：不删不移；通过 `STATUS:` 头注释 + 集中索引文件 `docs/ARCHIVE_INDEX.md` 标注。

---

### A.1【P0｜风险:零｜可回滚】创建 `docs/ARCHIVE_INDEX.md`

**为什么需要**：根目录 92 个文件中，至少 60 个属于"历史产物或重复"，但无法靠文件名分辨哪个还在用。需要一份单一权威索引。

**建议文件内容模板**：

```markdown
# 文件归档索引

> 本表登记 ADC_FFT_TEST 根目录所有疑似废弃、历史快照、临时产物。
> 状态约定：ACTIVE=仍在用 | OBSOLETE=确认废弃 | UNCERTAIN=待复核 | ARCHIVE=保留参考
> 最后更新：YYYY-MM-DD

## J-Link 脚本（.jlink）

| 文件名 | 状态 | 被谁取代 / 用途 | 可删日期 |
|---|---|---|---|
| download.jlink           | ACTIVE     | 主烧录脚本                       | -        |
| download_v2.jlink        | OBSOLETE   | 被 download.jlink 取代           | 90 天后  |
| download_v3.jlink        | OBSOLETE   | 同上                             | 90 天后  |
| download_recover.jlink   | UNCERTAIN  | 看名字疑似 brick 恢复脚本，待核实 | -        |
| download_safe.jlink      | UNCERTAIN  | 同上                             | -        |
| download_test.jlink      | OBSOLETE   | 早期 quick test，已被取代         | 90 天后  |
| download_temp.jlink      | OBSOLETE   | 临时文件                          | 30 天后  |
| download_verify.jlink    | UNCERTAIN  | 烧录+校验流程，待核实             | -        |
| download_lcd1_8.jlink    | ARCHIVE    | 与 LCD 测试相关，但本工程不含 LCD | 永久      |
| download_30_50.jlink     | UNCERTAIN  | 命名含义不清                      | -        |
| flash_and_run.jlink      | UNCERTAIN  | 与 just_go/run_board 重复？      | -        |
| flash_firmware.jlink     | UNCERTAIN  | 同上                             | -        |
| flash_run.jlink          | UNCERTAIN  | 同上                             | -        |
| jlink_burn.jlink         | UNCERTAIN  | 同上                             | -        |
| jlink_flash.jlink        | UNCERTAIN  | 同上                             | -        |
| jlink_download.jlink     | UNCERTAIN  | 同上                             | -        |
| jlink_auto.jlink         | UNCERTAIN  | 自动化批处理？                    | -        |
| jlink_rtt.jlink          | ACTIVE     | RTT 调试入口                      | -        |
| just_go.jlink            | OBSOLETE   | 命名非正式，被 run_board 取代     | 30 天后  |
| start_board.jlink        | OBSOLETE   | 同上                             | 30 天后  |
| restart_board.jlink      | OBSOLETE   | 同上                             | 30 天后  |
| run_board.jlink          | ACTIVE     | 启动板子                          | -        |
| reset_board.jlink        | ACTIVE     | 复位                              | -        |
| reset_only.jlink         | OBSOLETE   | 与 reset_board 重复               | 30 天后  |
| dwt_check.jlink          | ARCHIVE    | DWT 计数器调试快照                | 永久      |
| read_mem.jlink           | ARCHIVE    | 内存读取工具                      | 永久      |
| read_rcc.jlink           | ARCHIVE    | RCC 寄存器读取                    | 永久      |
| read_rtt.jlink           | ARCHIVE    | RTT 一次性读取                    | 永久      |
| read_tim8.jlink          | ARCHIVE    | TIM8 寄存器读取                   | 永久      |
| test_connect.jlink       | ACTIVE     | 连通性测试                        | -        |
| check_status.jlink       | ACTIVE     | 状态检查                          | -        |

## 串口日志（serial_log_*.txt）

| 文件名 | 大小 | 采集日期 | 关联报告 | 状态 |
|---|---|---|---|---|
| serial_log_60s.txt        | -    | -        | 早期测试           | OBSOLETE |
| serial_log_dedup.txt      | -    | -        | 去抖调试           | OBSOLETE |
| serial_log_dedup2.txt     | -    | -        | 去抖调试 v2        | OBSOLETE |
| serial_log_diag.txt       | -    | -        | 诊断日志           | OBSOLETE |
| serial_log_diag2.txt..4   | -    | -        | 诊断日志 v2/3/4    | OBSOLETE |
| serial_log_final.txt      | -    | -        | TEST_REPORT_FINAL  | ARCHIVE  |
| serial_log_final2.txt     | -    | -        | 同上 第二次         | OBSOLETE |
| serial_log_fix1.txt       | -    | -        | 修复后日志         | OBSOLETE |
| serial_log_latest.txt     | -    | -        | "最新" 不确定        | UNCERTAIN |
| serial_log_long.txt       | -    | -        | 长时间运行测试     | ARCHIVE  |
| serial_log_peak.txt       | -    | -        | 峰值压测           | ARCHIVE  |
| serial_log_robust.txt     | -    | -        | 鲁棒性测试         | ARCHIVE  |

## Benchmark 输出（bench_*.txt）

| 文件名 | 关联实验 | 状态 |
|---|---|---|
| bench_flash.txt              | 初版 flash 基准            | OBSOLETE |
| bench_FLASH_baseline.txt     | 基线                       | ARCHIVE  |
| bench_flash_v2.txt           | v2                         | OBSOLETE |
| bench_flash_v3.txt           | v3                         | OBSOLETE |
| bench_FLASH_v4.txt           | v4                         | OBSOLETE |
| bench_FLASH_FINAL.txt        | FINAL（实际还是中间版？）  | UNCERTAIN |
| bench_STAGE_A_FLASH.txt      | Stage A                    | ARCHIVE  |
| bench_FINAL_STABLE.txt       | 当前稳定基线               | ACTIVE   |

## Verify 输出（verify_*.txt）

| 文件名 | 状态 |
|---|---|
| verify_fft_v5..v8.txt        | OBSOLETE（迭代快照） |
| verify_fft_caller.txt        | OBSOLETE |
| verify_fft_debug.txt         | OBSOLETE |
| verify_fft_final.txt         | ARCHIVE  |
| verify_fft_noinline.txt      | OBSOLETE |
| verify_final.txt / final2.txt / final3.txt | OBSOLETE |
| verify_trigger_v2.txt        | OBSOLETE |
| verify_trigger_continuous.txt| ARCHIVE  |
| verify_6ch_full.txt          | ACTIVE   |

## 测试报告 .md

| 文件名 | 状态 | 处置建议 |
|---|---|---|
| FINAL_ANALYSIS_REPORT.md | ARCHIVE | 合并到 TEST_HISTORY.md |
| TEST_ANALYSIS_REPORT.md  | ARCHIVE | 同上 |
| TEST_PLAN.md             | ARCHIVE | 同上 |
| TEST_REPORT_FINAL.md     | ARCHIVE | 同上 |
| TEST_RESULTS.md          | ARCHIVE | 同上 |
| TEST_LOG_REALTIME.md     | ARCHIVE | 同上 |
| TEST_LOG_20260525_0130.txt | ARCHIVE | 同上 |
| WorkLog.md               | ACTIVE  | 保留为开发日志，但建议改名 DEVLOG.md |

## 源码备份

| 文件名 | 状态 | 说明 |
|---|---|---|
| Core/Src/fft_backup.c | OBSOLETE | 命名暗示是 main.c FFT 代码段的手动备份，应该靠 Git 管理。**强烈建议从 Keil 工程中排除（exclude from build）+ 加 STATUS 头注释**。下次 commit 后 30 天可考虑删除。 |

## 调试器配置

| 文件名 | 状态 |
|---|---|
| jlink_debug.cfg  | ACTIVE（如果实际用 J-Link）|
| stlink_debug.cfg | UNCERTAIN（如果不再用 ST-Link，OBSOLETE）|

## Python 辅助脚本

| 文件名 | 状态 | 用途 |
|---|---|---|
| analyze_realtime_test.py | UNCERTAIN | 待确认是否仍使用 |
| analyze_test_log.py      | UNCERTAIN | 同上 |
| download_firmware.py     | UNCERTAIN | 与 .jlink 脚本功能重复？ |
| serial_capture.py        | ACTIVE    | 串口抓帧 |
| test_6ch_verify.py       | ACTIVE    | 6 通道验证 |

## 临时 / 杂项

| 文件名 | 状态 |
|---|---|
| capture_serial.bat | UNCERTAIN |
| debug.log          | OBSOLETE（日志输出，应在 .gitignore）|
```

**接手时要做的**：
1. 在 `docs/` 下新建 `ARCHIVE_INDEX.md`，按上表填充
2. **逐行复核 UNCERTAIN 项**，问当前作者确认状态
3. 把所有"OBSOLETE，可删日期已到"的清单提交给作者，确认后再做 Section B 的合并

---

### A.2【P0｜风险:零｜可回滚】在每个 OBSOLETE 文件顶部添加 STATUS 头注释

**目标**：即使有人没看 ARCHIVE_INDEX.md，打开文件第一眼就能看见状态。

**.jlink 文件统一头注释模板**（J-Link 脚本以 `//` 开始注释行）：
```
// =============================================================================
// STATUS: OBSOLETE  (2026-06-07)
// REPLACED-BY: download.jlink
// KEPT-FOR: 历史参考，预计 2026-09-07 后可删除
// SEE: docs/ARCHIVE_INDEX.md
// =============================================================================
// (原内容保持不变 ↓)
```

**.txt 串口日志统一头注释模板**：
```
=== STATUS: ARCHIVE/OBSOLETE ===
=== 采集日期: ____ ===
=== 关联报告: docs/TEST_HISTORY.md::<section> ===
=== 参见: docs/ARCHIVE_INDEX.md ===

(原内容)
```

**.md 测试报告统一头注释模板**：
```markdown
> **STATUS: ARCHIVE** (2026-06-07)
> 本文件已并入 `docs/TEST_HISTORY.md::<section>`。
> 保留原文件仅作引用追溯。请阅读合并版以获得最新真相。
```

**.c/.h 源码备份文件**（仅 `Core/Src/fft_backup.c`）：
```c
/**
 * ============================================================================
 * STATUS: OBSOLETE BACKUP — 不参与编译
 * 本文件是 main.c FFT 代码段在 YYYY-MM-DD 的手动备份。
 *
 * 维护方式应使用 Git，而非 _backup 后缀。
 *
 * 处置计划：
 *   - 阶段1（本次）：本注释 + 从 Keil 工程 "Exclude from build" 排除
 *   - 阶段2（确认无人引用后 30 天）：从仓库删除
 *
 * 参见：docs/ARCHIVE_INDEX.md::源码备份
 * ============================================================================
 */
```

**注意**：
- **本步骤不修改 .uvprojx**，是否"Exclude from build"由实施者用 Keil UI 操作（这是 GUI 操作，不算 Prometheus 任务）
- 头注释字符必须用对应语言的合法注释语法，绝不影响文件用途
- `STATUS:` 关键字便于将来用 `grep -r "STATUS:" .` 一键列表

---

### A.3【P1｜风险:零｜可回滚】考虑创建 `docs/GITIGNORE_CANDIDATES.md`（不写 .gitignore）

**为什么不直接写 .gitignore**：Keil 工程的 .gitignore 涉及构建产物筛选，可能误伤；用户的"保守模式"约束下，建议先列清单让作者复核。

**建议文件内容**：

```gitignore
# === MDK-ARM 构建产物 ===
MDK-ARM/*/
!MDK-ARM/*.uvprojx
!MDK-ARM/*.uvoptx
!MDK-ARM/*.s
!MDK-ARM/*.sct
!MDK-ARM/RTE/

# === 串口/调试日志 ===
serial_log_*.txt
debug.log
*.log

# === Benchmark / verify 临时输出 ===
bench_*.txt
verify_*.txt
TEST_LOG_*.txt

# === Python 缓存 ===
__pycache__/
*.pyc

# === IDE ===
.vscode/
.idea/

# === OS ===
Thumbs.db
.DS_Store

# === 临时 ===
*.tmp
*.bak
*~

# 注意：以下文件即使产生也建议保留在仓库，便于追溯
# - docs/ARCHIVE_INDEX.md 引用的所有 ARCHIVE 状态文件
# - 历次重要 .jlink 脚本（即使 OBSOLETE 状态）
```

**接手时要做的**：作者复核此清单 → 自行决定是否写入实际 `.gitignore`。

---

## Section B：文档整合

> **目标**：让外部读者（含半年后的你）能用 1 份 README + 2 份说明书快速理解整个系统。
> **原则**：所有原 .md 不删不动；建立合并版作为新的"单一权威来源"。

---

### B.1【P0｜风险:零｜可回滚】重写 `README.md`

**现状问题**：当前 README.md 仅 3 行：
```
# WeAct Studio
## ADC Test
ADC 测试，通过ADC读取Vrefint,温度,vbat
ADC test, read Vrefint, temperature,vbat by ADC
```
**与实际系统完全不符**（系统是 6 通道实时 ADC + DMA + FFT + 事件帧上送，根本不是 Vrefint 测试）。

**建议的新 README.md 结构**：

```markdown
# STM32H750 6 通道实时 ADC + FFT 事件帧上送系统

> **状态**：稳定运行（2026 年迭代至 v3.0 事件帧协议）
> **目标硬件**：STM32H750VBT6（WeAct Studio 北极星板）
> **目标 PC**：Windows + CH340 USB-Serial @ 460800 baud
> **维护责任人**：（待填）

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
| docs/ | 工程文档（含 ARCHIVE_INDEX、TEST_HISTORY、CODE_REVIEW_NOTES、DEFERRED_BACKLOG）|
| STM32_ADC_FFT_开发指南.md | 开发详细说明书（建议移入 docs/ ）|

---

## 快速开始

### 烧录与运行
\`\`\`
# 用 J-Link 烧录
JLink.exe -CommanderScript download.jlink

# 抓串口
python serial_capture.py --port COM3 --baud 460800
\`\`\`

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
| CURRENT_TEST_MODE (main.h)| TEST_MODE_PWM | **⚠️ 此值与生产路径不一致，详见 docs/CODE_REVIEW_NOTES.md** |

---

## 协议、报告、历史

- 事件帧 v3.0 协议详见 [Core/Inc/event_frame.h](Core/Inc/event_frame.h)
- 历次测试报告合并版 → [docs/TEST_HISTORY.md](docs/TEST_HISTORY.md)
- 历史/废弃文件登记 → [docs/ARCHIVE_INDEX.md](docs/ARCHIVE_INDEX.md)
- 代码评审备注 → [docs/CODE_REVIEW_NOTES.md](docs/CODE_REVIEW_NOTES.md)
- 后续优化 backlog → [docs/DEFERRED_BACKLOG.md](docs/DEFERRED_BACKLOG.md)
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
```

---

### B.2【P0｜风险:零｜可回滚】创建 `docs/TEST_HISTORY.md`（6 份测试报告合并）

**为什么**：当前 6 份 .md 测试报告（FINAL_ANALYSIS / TEST_ANALYSIS / TEST_PLAN / TEST_REPORT_FINAL / TEST_RESULTS / TEST_LOG_REALTIME）彼此重叠且时间线混乱，真相分散。新人不可能读 6 份找一致结论。

**合并策略**：
- 不删原 6 份（按 A.2 加 STATUS: ARCHIVE 头）
- 在 docs/TEST_HISTORY.md 按时间线整理：每份原报告占一个 `## YYYY-MM-DD 报告名` 章节
- 在文件顶部加"当前最新结论"摘要章节
- 每个章节末尾加 "→ 原文件：../FINAL_ANALYSIS_REPORT.md" 反向链接

**建议结构**：

```markdown
# 测试历史汇编

> **最新结论摘要**（last updated YYYY-MM-DD）
>
> - 实测系统在 6 通道独立信号下 120 evt/s 稳定无丢帧
> - FFT 1024 点单次耗时 200 μs @ 480 MHz
> - 当前噪声门限：120 ADC counts (≈96 mV)，噪声倍数 8
> - 同通道去抖 10 ms
> - 已知瓶颈：CH340 USB Bulk 合包导致 evt 率上限 ~120 Hz
>
> 详细历史按下面时间线展开。

---

## YYYY-MM-DD ｜ TEST_PLAN

【原 TEST_PLAN.md 全文 copy-paste，附 STATUS 头】

→ 原文件：../TEST_PLAN.md

---

## YYYY-MM-DD ｜ TEST_RESULTS

【原 TEST_RESULTS.md 全文 copy-paste】

→ 原文件：../TEST_RESULTS.md

---

## (依此类推合并所有 6 份)

---

## 附：测试数据快照对应关系

| 报告章节 | 对应数据文件 | 对应 .jlink 验证脚本 |
|---|---|---|
| TEST_REPORT_FINAL  | serial_log_final.txt    | verify_fft_final.txt |
| TEST_LOG_REALTIME  | serial_log_latest.txt   | -                    |
| FINAL_ANALYSIS     | bench_FINAL_STABLE.txt  | -                    |
```

**接手时要做的**：
1. 按时间线（git log 各 .md 创建时间）排序
2. 逐份 copy-paste 全文到对应章节
3. 顶部摘要由接手者根据最新理解填写
4. 完成后给 6 份原报告加 A.2 中的 STATUS 头注释

---

### B.3【P1｜风险:零｜可回滚】把 `STM32_ADC_FFT_开发指南.md` 内容范围标注清楚

**现状**：该文件位于根目录，与 README 平级，但用途不明（可能与 README、TEST_HISTORY 重叠）。

**建议**：
1. 在该文件顶部追加摘要章节（不删原内容）：
   ```markdown
   > **文档定位**：本文件是**开发者级**详细说明，包含：
   >   - 硬件电路接法
   >   - CubeMX 配置详情
   >   - 寄存器级 ADC/DMA/TIM 配置说明
   >   - 调试技巧与坑点
   >
   > **本文件不是**：API 手册、测试报告（→ docs/TEST_HISTORY.md）、协议规范（→ Core/Inc/event_frame.h）。
   >
   > **建议位置**：未来可移入 docs/DEV_GUIDE.md（本次保持原位置）。
   ```
2. 若与 TEST_HISTORY 有重叠章节，加 `> 同步备注：参见 docs/TEST_HISTORY.md::<section>` 反向链接

---

### B.4【P1｜风险:零｜可回滚】把 `WorkLog.md` 改为开发日志规范化模板（仅追加，不改原内容）

**现状**：WorkLog.md 是日记式开发记录，但格式松散，混入设计决策与日常调试。

**建议**（仅追加顶部模板，不动既有内容）：

```markdown
> **本文件性质**：纯开发日志（按时间倒序）。
> **本文件不包含**：架构决策（→ docs/CODE_REVIEW_NOTES.md）、测试结果（→ docs/TEST_HISTORY.md）。
>
> 后续记录建议遵循模板：
>
> ## YYYY-MM-DD ｜ 任务摘要
> - **背景**：（一句话）
> - **改动**：（文件 + 行号 + 一句话）
> - **验证**：（命令 + 结果）
> - **决策**：（如有重要选择）

---

【以下为既有内容，保持原样】
```

---

## Section C：代码质量建议

> **目标**：改善可读性与可维护性，**不修改任何可执行代码**（仅注释、命名建议、标注）。
> **红线**：不更改 `#define` 值、不拆函数、不删变量、不更改任何 `if()` 条件。

---

### C.1【P0｜风险:零｜可回滚】在 `main.h` 中标注 CURRENT_TEST_MODE 语义不一致问题

**位置**：`Core/Inc/main.h` line 87

**现状**：
```c
#define CURRENT_TEST_MODE  TEST_MODE_PWM
```

**问题**：系统实际生产路径（`ENABLE_DAC_SIGNAL_SOURCE=0`）走的是 6 通道 ADC+FFT 检测主循环，
根本不走 `test_pwm_main()` 路径。`CURRENT_TEST_MODE` 的值似乎从未被 main.c 的运行时逻辑使用
（main.c 中所有 `test_mode` 相关代码在 `#if` 块中已被条件编译隔离）。

**建议追加注释**（不改宏值，不删宏定义）：

```c
/* ⚠️ 警告：此宏定义的测试模式在 6 通道 ADC+FFT 生产路径下不被使用。
 *
 * main.c 中 main() 函数实际走以下路径（由 ENABLE_DAC_SIGNAL_SOURCE 控制）：
 *   - ENABLE_DAC_SIGNAL_SOURCE=0（生产）：6 通道 ADC+FFT 事件检测循环
 *   - ENABLE_DAC_SIGNAL_SOURCE=1（测试）：DAC 闭环测试
 *
 * TEST_MODE_PWM 是 CubeMX 模板遗留/早期实验的 PWM 独立测试模式，
 * 在 2026-06-07 的最新代码中看不见被正式路径使用。
 *
 * 此宏的值不应被依赖。后续重构建议完全移除这组 TEST_MODE_* 宏
 * 或将其统一到 ENABLE_DAC_SIGNAL_SOURCE / 运行时按键选择模式。
 *
 * 另见：docs/CODE_REVIEW_NOTES.md::测试模式与生产路径 */
#define CURRENT_TEST_MODE  TEST_MODE_PWM
```

---

### C.2【P1｜风险:零｜可回滚】在 `main.c` 中标注三条 UART 发送路径并存问题

**位置**：`Core/Src/main.c` 三组函数声明/正文附近

**情况**：
1. **`evt_tx_queue_push()` / `evt_tx_queue_poll()`**（line 502-579）—— 自主环形队列 + `HAL_UART_Transmit` 阻塞发送，已标记为"v6 之前的老路径"
2. **`tx_ring_poll()`**（line 607-648）—— `HAL_UART_Transmit_DMA` 零阻塞路径，注释"v7 DMA 模式"
3. **`HAL_UART_Transmit(&hdebug_uart, ...)` 直接调用**（如 line 570 在 `evt_tx_queue_poll()` 内部）—— 阻塞发送

**建议追加注释**（分别在 evt_tx_queue_push 和 tx_ring_poll 函数定义上方）：

```c
/* ★ v7 DMA 路径：tx_ring + tx_ring_poll + HAL_UART_Transmit_DMA — 当前活跃路径
 * ★ v6 及之前路径：evt_tx_queue + HAL_UART_Transmit 阻塞 — 已经被 v7 取代但仍保留定义
 *
 * 【2026-06-07 存档】v7 已验证 DMA 零阻塞发送稳定、0 丢帧
 * （通过 ENABLE_DMA_LOOPBACK_TEST=1 验证：diag_tx_poll_send_attempt == diag_tx_poll_send_ok）。
 *
 * evt_tx_queue 仍在取用（由外部事件路径调用 evt_tx_queue_push），
 * 但它里面调的是 HAL_UART_Transmit（阻塞），在正式事件流中不应该被走通。
 * 如果未来重构，建议：
 *   - 把 evt_tx_queue_push 的定义替换为调用 tx_ring_push
 *   - 删除 evt_tx_queue_poll 和 evt_tx_queue 整个缓冲
 *   - 统一为 tx_ring + tx_ring_poll 一套路径
 * 参见：docs/CODE_REVIEW_NOTES.md::UART 发送路径冗余 */
```

---

### C.3【P1｜风险:零｜可回滚】在 `main.c` 中标注 70+ diag_* 全局变量的元意图

**位置**：`Core/Src/main.c` line 300-390 + 595-605 + 761-765

**情况**：70+ 个 `volatile uint32_t diag_*` 变量散布在整个 `main.c` 中，没有分组注释，也没有收敛点。其中有几个特征：
- 很多只在 RTT/J-Link 调试时读取，在正式运行时不被任何函数读取
- 部分是"一次性诊断"（如 `diag_fft256_cyc` line 369，仅在 `FFT_Benchmark()` 里写一次）
- 部分是"监控诊断"（如 `diag_frame_process_cyc` line 385，每帧都更新）

**建议**（仅在变量声明块上方追加"诊断变量分类"注释，不改变任何变量定义）：

```c
/* =============================================================================
 * [诊断变量归档 — 2026-06-07]
 *
 * 以下 ~70 个 volatile 变量仅用于 RTT/J-Link 就地调试，不在正式流程中参与
 * 业务逻辑。它们按用途分为以下几组：
 *
 * Group A — 流程计数（主循环、DMA、dac burst 的诊断计数）
 *   line 300-311: diag_loop_cnt, diag_dac_burst_cnt, diag_fft_try_cnt, ...
 *
 * Group B — UART 发送诊断（tx_ring_poll 的发送链路诊断）
 *   line 595-605: diag_tx_poll_st, diag_tx_poll_call_cnt, ...
 *
 * Group C — 压力压测 / DMA loopback / 极限诊断
 *   line 313-322: diag_tx_q_depth_max, diag_tx_q_full_cnt, ...
 *
 * Group D — 通道级事件诊断（每通道最新触发状态）
 *   line 323-342: diag_last_noise_pp[6], diag_evt_count_ch[6], ...
 *
 * Group E — FFT 诊断（频谱快照、bin 详细）
 *   line 343-358: diag_fft_max_bin, diag_fft_spec_snapshot[28], ...
 *
 * Group F — 单次基准测试（只在 FFT_Benchmark 或启动时写一次）
 *   line 361-372: diag_fft256_cyc, diag_dwt_baseline, ...
 *
 * Group G — ADC 采样间隔诊断
 *   line 374-390: diag_frame_process_cyc, diag_frame_interval_cyc, ...
 *
 * Group H — 各通道直流偏置基线（仅 ch_dc_offset[6] — 业务变量，不算诊断）
 *   line 393-394: ch_dc_offset[6], ch_dc_offset_valid
 *
 * 下次重构时可将各组抽到独立结构体，以减少全局作用域污染。
 * ========================================================================= */
```

**注意**：这是最长的一条注释（约 30 行，不含 WCET），但对新人定位 debug 变量至关重要。

---

### C.4【P1｜风险:零｜可回滚】在 `adc.h` 中备注 DMA buffer 大小取值逻辑

**位置**：`Core/Inc/adc.h` line 57-58

**现状**：
```c
#define ADC_DMA_BUF_SIZE     (ADC_HALF_SCANS * ADC_CH_PER_INST * 2)  /* 16384 = full buf per ADC */
```

**问题**：乘 `*2` 是因为双缓冲（半区 + 全区），但刚接触的工程师容易误以为是"2 通道"的乘数。
`ADC_CH_PER_INST` 已经是 2 通道了。让计算公式逻辑自文档化。

**建议追加注释**：
```c
/* ADC_DMA_BUF_SIZE = ADC_HALF_SCANS(每通道半区样本数) × ADC_CH_PER_INST(每 ADC 通道数 2) × 2(双缓冲)
 *   = 4096 × 2 × 2 = 16384 uint16_t per ADC (= 32 KB)
 * 布局：buf[0..8191] = 半区0，buf[8192..16383] = 半区1
 * 每个半区内：数据按 rank1, rank2, rank1, rank2,... 交织排列 */
```

---

### C.5【P2｜风险:零｜可回滚】在 `event_frame.h` 中指明协议版本与 PC 端验证工具的耦合要求

**位置**：`Core/Inc/event_frame.h` line 111

**现状**已有：
```c
/* 帧格式常量 (与 verify_event_system.py 完全对应，禁止单方面修改) */
```

**建议**增加一个指向 ARCHIVE_INDEX 的反向链接（因为协议修改涉及整个端到端系统）：

```c
/* 帧格式常量 (与 verify_event_system.py 完全对应，禁止单方面修改)
 * ⚠️ 任何帧格式变更（字段偏移、类型、大小）必须同步修改：
 *   1. PC 端: F:/ADC_FFT/test_scripts/verify_event_system.py
 *   2. 本头文件 event_frame.h
 *   3. 通讯录 docs/TEST_HISTORY.md::附录：协议版本变更日志
 * 违反此规则会导致 PC 端解析失败，且问题难以诊断。 */
```

---

### C.6【P2｜风险:零｜可回滚】在 `debug.h` 中标注 `debug_printf` 行为被 `ENABLE_DEBUG_LOG` 宏控制的机制

**位置**：`Core/Inc/debug.h` line 59-64

**现状**：通过预处理器把 `debug_printf(...)` 替换为 `do{}while(0)`。但新人第一次看到 `debug_printf` 调用以为是正常日志输出，不知道它在生产固件中无效果。

**建议追加注释**（在宏定义附近）：
```c
/* ⚠️ 注意：ENABLE_DEBUG_LOG 控制 debug_printf 是否输出。
 * = 0（生产）：所有 debug_printf 宏展开为 do{}while(0)，零运行时开销但参数仍受语法检查
 * = 1（调试）：正常输出到 RTT/UART（与二进制事件帧竞争带宽）
 * ────────────────────────────────────────────────
 * 生产固件中想看到调试输出？
 * → 用 RTT（SEGGER_RTT_printf），不通过 UART，不与二进制帧冲突
 * ────────────────────────────────────────────────
 * 注意：debug_printf 被定义三次：
 *   1. 本文件的宏替换（第 64 行）
 *   2. debug.c 中通过 #define DEBUG_PRINTF_INTERNAL 避开替换后的真实函数定义
 *   3. 外部文件包含 debug.h 时只能看到宏替换版本 */
```

---

### C.7【P2｜风险:零｜可回滚】在 `stm32h7xx_it.c` 中标注 DMA ISR 之间"逻辑与"等待关系

**位置**：`Core/Src/stm32h7xx_it.c`（DMA 中断处理函数）

**现状**（从 `check_dma_and_push_frames` line 694/720 可见逻辑）：
```c
if(adc1_dma_half && adc2_dma_half && adc3_dma_half) { ... }
```
这意味着 ADC 半区完成 ISR 设置 flag，但主循环等待**所有三个 ADC 都完成**后才开始处理。
这是正确的设计（防止时间戳不一致），但如果在 ISR 函数里没控制好 D-Cache coherence，
某个 ADC 的 flag 可能迟迟不为 1，导致对应 ISR 永远不会被完整执行。

**建议**在 ISR 函数周围加注释强调"必须做 D-Cache invalidate 或使用非缓存变量"。

---

### C.8【P0｜风险:零｜可回滚】创建 `docs/CODE_REVIEW_NOTES.md`

将所有不适合放在代码注释中的大规模评审意见集中到一份文档中：

```markdown
# 代码评审备注

> 本文档记录通过人工审查发现的设计/架构问题，当前不宜通过修改代码解决。
> 未来立项优化时以此为输入。

---

## 1. 测试模式与生产路径（TEST_MODE vs ENABLE_DAC_SIGNAL_SOURCE）

- main.h 中定义 6 种测试模式（LED/PWM/DAC/ADC/UART/NONE）
- CURRENT_TEST_MODE 默认 = TEST_MODE_PWM
- **但与生产路径完全无关**：实际行为由 main.c 顶部的 ENABLE_DAC_SIGNAL_SOURCE 控制
- 两组宏的语义重叠且不一致
- 建议远期：统一为单一 `#define PRODUCTION_MODE` 或运行时按键选择

---

## 2. UART 发送路径三套并存

| 路径 | 实现 | 当前活跃？ |
|---|---|---|
| evt_tx_queue_push/poll | 环形缓冲 + HAL_UART_Transmit（阻塞）| 保留但不被生产路径调用 |
| tx_ring_poll | 环形缓冲 + HAL_UART_Transmit_DMA（零阻塞）| ✅ 生产路径 |
| HAL_UART_Transmit 直接调用 | 无缓冲，直接阻塞 | 仅 evt_tx_queue_poll 内使用 |

- 注释说 "v7 已验证 DMA 零阻塞"但 evt_tx_queue 函数未被删除
- evt_tx_queue_push 被外部事件路径调用 —— 但实际上那条路径最终走到 HAL_UART_Transmit？
- **风险**：新人以为 evt_tx_queue_push 是正确入队函数，但它在正式事件路径下应该被取代

---

## 3. main.c 单文件 > 3000 行

- 包含：DWT 初始化、FFT 表加载、FFT 基准测试、DC 偏移校准、事件检测、DAC burst 控制、UART 发送队列、调试诊断
- 至少应有 3-4 个独立模块：signal_detection.c、event_tx.c、calibration.c
- 但这需要动可执行代码，超出本次 A 阶段范围

---

## 4. PA4 双重用途（硬件级约束）

- adc.h 注释（line 11-18）和 main.c 头部（line 38-41）都已标注
- DAC1_OUT1 与 ADC2_INP18 共用 PA4
- 短期：已经在 ADC2 中做调整，确保 HAL 初始化顺序不冲突
- 长期：考虑硬件改板，或用外部多路复用器分离

---

## 5. 全局 diag 变量聚集

- 70+ volatile uint32_t 散布在 main.c 中
- 建议未来抽取为 `typedef struct { ... } diag_t;` + 单一 extern
- 在 RTT 调试时也更方便计算结构体偏移

---

## 6. ADC 采样率实测与理论不符

- 实测 3.515 MSPS vs 理论 2.009 MSPS
- `adc_fs_hz = 3515000.0f` 是 RAM 变量（可在线调整）— 存在环节风险
- 如果代码中任何其他函数用了错误的采样率，频率计算会弥漫性偏差
- 建议最优先在性能审计阶段查清根源

---

## 7. CubeMX .ioc 现存但不可再生

- `07-ADC_Test.ioc` 在根目录
- 但 ADC 配置代码已经手工编辑（adc.h 注释说 "PC2/PC3 分配到 ADC3 以分散负载"）
- 从 .ioc 重新生成代码会覆盖手写修改
- 建议：存档 .ioc 至 archive/ 并标注"仅作参考，不可再生代码"
```

---
