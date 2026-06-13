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
- 但这需要动可执行代码，超出当前零风险优化范围

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

---

## 8. 诊断变量缺乏统一管理

- diag_* 变量散布在 main.c 多个位置，缺乏分组和注释
- 建议：按功能分组（流程计数、UART诊断、FFT诊断等），每组加注释说明用途
- 增加 diag_* 变量的生命周期说明（单次写/每帧更新/仅调试时读取）

---

## 9. MPU 配置与内存布局

- 当前 MPU 配置已涵盖 AXI SRAM、DTCM、SRAM4、ITCM
- 建议：在 adc.h 中增加 DMA 缓冲区地址与 MPU 区域的对应关系注释
- 确保未来修改缓冲区位置时同步更新 MPU 配置

---

## 10. 事件帧协议版本管理

- event_frame.h 定义的 19 字节帧格式与 PC 端强耦合
- 建议：增加协议版本号字段，支持向后兼容
- 在文档中记录协议变更历史

---

## 附录：代码质量改进建议

### A. 命名规范统一

- 函数命名：`MY_ADC1_Init` vs `HAL_ADC_MspInit` — HAL 风格不一致
- 变量命名：`diag_*` 前缀统一，但缺乏子分类（如 `diag_fft_*`、`diag_uart_*`）
- 建议：制定项目级命名规范文档

### B. 注释质量提升

- 函数头注释应包含：功能描述、参数说明、返回值、修改记录
- 复杂算法应增加设计决策注释（如 FFT 窗口选择、触发阈值计算逻辑）
- 硬件相关代码应增加寄存器级注释（如 ADC 时钟配置、DMA 通道映射）

### C. 错误处理增强

- 当前 Error_Handler() 仅无限循环
- 建议：增加错误码记录、错误恢复策略、看门狗集成

### D. 测试覆盖

- 当前测试依赖硬件在环验证
- 建议：增加单元测试框架（如 Unity）
- 增加 Mock 层便于脱离硬件测试算法逻辑
