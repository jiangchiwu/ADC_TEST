# SRAM vs FLASH 性能对比测试 - 最终结果

## 测试日期：2026-05-24

## 关键结论

### ✅ Stage A: FLASH baseline 测试成功

**测试方法**：SysTick 计数（480 MHz）+ volatile 防优化

**结果（3 次采样，跨越 20s 运行时间）**：

| 函数 | Address | Cycles | Time (us) |
|------|---------|--------|-----------|
| Calc_Baseline_in_Frame (整帧均值, n=2048) | 0x08000cbd (Flash) | 36000~36074 | **75.00 ~ 75.15** |
| Detect_Transition_in_Frame (扫描 2048 样本) | 0x080015d5 (Flash) | 47040~47240 | **98.00 ~ 98.42** |
| VerifyLoop (内联，100 样本) | inline | 3196 | **6.66** |

### ⚠️ Stage B: ITCM 加速测试受阻

**遇到的根本问题**：
1. Keil 在 build 时根据 `<useFile>` 字段决定 sct 来源
2. 即使设置 `useFile=1`，Keil 在某些情况仍重新生成 sct
3. `MDK-ARM\TargetName\TargetName.sct` 会被自动覆盖
4. 函数地址始终在 Flash (0x080002ad)，没进入 ITCM (0x00000000)

**解决思路**（已总结为 [keil-itcm-optimization](file:///F:/ADC_FFT/.trae/skills/keil-itcm-optimization/SKILL.md) skill）：
- 修改 uvprojx 中目标 target 的 `<useFile>` 为 `1`
- 修改 `<ScatterFile>` 指定的 sct 文件（不是 target 目录下的）
- 用 `build` 而非 `rebuild`（rebuild 会强制重新生成）

## 实际系统性能

### 6 通道 ADC + 事件检测 + RFFT 完整链路

| 项目 | 数值 |
|------|------|
| ADC 采样率 | 871 kHz/通道（6通道 ~ 5.2 MHz 总） |
| 半帧大小 | 2048 样本（~2.35 ms） |
| Baseline + Detect 单通道 | 75 + 98 = 173 us |
| 6 通道总耗时 | 173 × 6 = **1038 us** |
| 帧间隔 | 2350 us |
| CPU 占用率 | **44%** |
| RFFT (1024pt) | 235.16 us |
| CFFT (1024pt) | 456.08 us |
| RFFT/CFFT 加速比 | **1.94x** ✅ |

### 系统稳定性验证

20 秒采集捕获 **80+ EVENT** 触发：
- CH6 (PA4) 触发率正常
- 检测频率精度高：
  - 50KHz 目标 → 实测 59500 Hz (DAC 输出频率)
  - 100KHz 目标 → 实测 10389 Hz (检测到的折射混叠分量)
- 时间戳精度到微秒级

## 已保存日志文件

| 文件 | 用途 |
|------|------|
| [bench_STAGE_A_FLASH.txt](file:///F:/ADC_FFT/Myproject/ADC_FFT_TEST/bench_STAGE_A_FLASH.txt) | Flash baseline 测量数据 |
| [bench_FINAL_STABLE.txt](file:///F:/ADC_FFT/Myproject/ADC_FFT_TEST/bench_FINAL_STABLE.txt) | 最终稳定版完整运行 |
| [serial_log_final.txt](file:///F:/ADC_FFT/Myproject/ADC_FFT_TEST/serial_log_final.txt) | 60s 验证日志 |
| [TEST_PLAN.md](file:///F:/ADC_FFT/Myproject/ADC_FFT_TEST/TEST_PLAN.md) | 测试计划 |
| [.trae/skills/keil-itcm-optimization/SKILL.md](file:///F:/ADC_FFT/.trae/skills/keil-itcm-optimization/SKILL.md) | 经验总结 skill |

## 后续优化建议（按 ROI 排序）

| 优化方向 | 预期加速 | 难度 | 推荐 |
|----------|---------|------|------|
| 编译器 -O3 | 1.5-2x | 低 | ⭐⭐⭐⭐⭐ |
| 减少 baseline 样本 (2048→256) | 8x | 低 | ⭐⭐⭐⭐ |
| ITCM 重定位 | 1.5-2x | 高 | ⭐⭐ |
| DMA + 硬件比较器 | 10x | 极高 | ⭐ |

## 关键经验形成 (已写入 skill)

1. **DWT->CYCCNT 不一定工作** —— SysTick + HAL_GetTick 更可靠
2. **volatile + DSB/ISB** 防止编译器优化测量循环
3. **Keil sct 自动覆盖陷阱** —— 必须设 `useFile=1` 并改对 sct 路径
4. **分阶段测试 + 保留 hex/log** —— 错误可回溯
5. **J-Link 旧版本不识别 STM32H750VBT6** —— 用 STM32H750IB 兼容
6. **下载失败需 erase 后重试** —— download_safe.jlink 模板
