# 文件归档索引

> 本表登记 ADC_FFT_TEST 根目录所有疑似废弃、历史快照、临时产物。
> 状态约定：ACTIVE=仍在用 | OBSOLETE=确认废弃 | UNCERTAIN=待复核 | ARCHIVE=保留参考
> 最后更新：2026-06-12

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
