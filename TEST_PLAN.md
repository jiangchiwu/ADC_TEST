# SRAM vs FLASH 性能对比测试

## 测试目标
对比关键热点函数在 Flash 和 ITCM (SRAM) 中执行的耗时差异，验证 SRAM 加速效果。

## 测试函数
1. `Calc_Baseline_in_Frame` - 计算 baseline + noise (含整帧均值)
2. `Detect_Transition_in_Frame` - 扫描帧中超阈值样本
3. `Verify_Sustain_in_Frame` - 验证持续性

## 测试方法
- 使用 DWT CYCCNT 计数器测量每个函数运行的 CPU cycle
- 480MHz CPU, 1 cycle = 2.08 ns
- 每个函数运行 N=1000 次取平均，消除测量误差
- 在每次定期打印中输出 BENCH 数据

## 测试阶段

### Stage A: Flash baseline
- 函数放在默认 Flash 段 (0x08000000)
- 测量耗时 BENCH_FLASH
- 文件备份: serial_log_flash_baseline.txt

### Stage B: ITCM 加速
- 函数放在 ITCM 段 (0x00000000)
- 测量耗时 BENCH_ITCM
- 文件备份: serial_log_itcm.txt

## 评判标准
- 加速比 = T_Flash / T_ITCM
- 若加速比 > 1.5x 则采用 ITCM 方案
- 若加速比 < 1.2x 则保留 Flash 方案（节省 ITCM 资源）

## 阶段状态

| 阶段 | 状态 | 备份文件 |
|------|------|----------|
| Stage 0: 计划 | ✅ | TEST_PLAN.md |
| Stage 1: 加入 BENCH | - | - |
| Stage 2: Flash 基准 | - | - |
| Stage 3: ITCM 修复 | - | - |
| Stage 4: ITCM 测量 | - | - |
| Stage 5: 结论 | - | - |
