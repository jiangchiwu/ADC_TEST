# STM32H7 多通道ADC+FFT开发指南

## 一、项目概述

### 目标
- 2通道ADC实时采样，单通道采样率>1Msps
- 可配置上升沿检测（0-3V/1-2V/自适应模式，μs级精度）
- FFT基频测量（20-200KHz，精度<2KHz）
- ST7735 LCD显示
- Flash占用<128KB（片内Flash）

### 硬件平台
- STM32H750VBT6 Core Board
- Cortex-M7 @ 480MHz
- 128KB Flash / 1MB RAM
- ST7735 1.8" LCD

---

## 二、ADC架构设计

### 方案选择

| 方案 | ADC配置 | 通道数 | 采样率 | 复杂度 |
|------|---------|--------|--------|--------|
| A: 双同步+独立 | ADC1+2双同步 + ADC3独立 | 6 | ~4.17Msps | 中 |
| B: 3独立轮询 | ADC1/2/3各2通道 | 6 | ~1.5Msps | 低 |
| C: 单ADC轮询 | 单ADC 6通道 | 6 | <1Msps | 低 |

**最终选择：方案A**

### 硬件引脚分配

| 通道 | ADC | 引脚 | 说明 |
|------|-----|------|------|
| CH1 | ADC1 | PA0_C | Vref+连接 |
| CH2 | ADC1 | PA7 | 通用模拟输入 |
| CH3 | ADC2 | PA1_C | Vref+连接 |
| CH4 | ADC2 | PA6 | 通用模拟输入 |
| CH5 | ADC3 | PF10 | 需打开模拟开关 |
| CH6 | ADC3 | PF11 | 需打开模拟开关 |

### 时钟配置

```c
// PLL3配置：M=5, N=200, P=5
// f_PLL3_input = HSE / M = 25MHz / 5 = 5MHz
// f_PLL3_vco = 5MHz * 200 = 1000MHz
// f_ADC_clock = 1000MHz / 5 / 2 = 100MHz
// Boost模式使能（ADCCLK>25MHz必须）
```

---

## 三、关键代码实现

### 1. ADC双同步模式初始化

```c
// adc.c - MY_ADC12_DMA_Init()
// 关键配置：
hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
hadc1.Init.Resolution = ADC_RESOLUTION_8B;
hadc1.Init.ScanConvMode = ENABLE;
hadc1.Init.NbrOfConversion = 2;

// 双同步模式配置
ADC12_COMMON->CCR |= ADC_CCR_MULTI_1 | ADC_CCR_MULTI_2;
ADC12_COMMON->CCR |= ADC_CCR_DMACFG;
ADC12_COMMON->CCR |= (2UL << ADC_CCR_DMA_Pos);  // DMA模式2
```

### 2. DMA缓冲区配置

```c
// 缓冲区放置在AXI SRAM (BDMA只能访问AXI SRAM)
uint32_t adc12_buf[1024] __attribute__((section(".AXISRAM")))
                          __attribute__((aligned(32)));
uint16_t adc3_buf[2048]  __attribute__((section(".AXISRAM")))
                          __attribute__((aligned(32)));

// 通道缓冲区在DTCM（零等待）
uint8_t ch_buf[ADC_NCH][ADC_HALF_SCANS]
              __attribute__((section(".DTCMRAM")));
```

### 3. 数据解交织

```c
void ADC_DeInterleave(uint8_t half) {
  // ADC12: CDR 32位打包 [ADC2:ADC1]
  // ADC3:  16位交替 [CH5, CH6]
  
  uint32_t *src12 = (half == 0) ? adc12_buf : 
                     &adc12_buf[ADC12_DMA_BUF_SIZE/2];
  uint16_t *src3  = (half == 0) ? adc3_buf :
                     &adc3_buf[ADC3_DMA_BUF_SIZE/2];
  
  for(i = 0; i < ADC_HALF_SCANS; i++) {
    uint32_t sample = src12[i];
    ch_buf[0][i] = (uint8_t)(sample & 0xFF);        // ADC1 CH1
    ch_buf[2][i] = (uint8_t)((sample >> 16) & 0xFF); // ADC2 CH3
    // 第二个采样点...
  }
}
```

### 4. 自定义FFT算法（无CMSIS-DSP依赖）

```c
// Cooley-Tukey基2FFT，分离实部虚部
// 优势：无库依赖，Flash占用小
static void my_cfft_f32(float *re, float *im, int n, int inverse)
{
  int i, j, k, m, bit;
  float t_re, t_im, w_re, w_im;
  float wm_re, wm_im, pi_div_m;
  
  // 位反转置换
  for(i = 1, j = 0; i < n; i++) {
    bit = n >> 1;
    while(j & bit) { j ^= bit; bit >>= 1; }
    j ^= bit;
    if(i < j) {
      t_re = re[i]; t_im = im[i];
      re[i] = re[j]; im[i] = im[j];
      re[j] = t_re; im[j] = t_im;
    }
  }
  
  // 蝶形运算
  for(k = 2; k <= n; k <<= 1) {
    m = k >> 1;
    w_re = 1.0f; w_im = 0.0f;
    pi_div_m = 3.14159265358979f / (float)m;
    wm_re = cosf((inverse ? 1.0f : -1.0f) * pi_div_m);
    wm_im = sinf((inverse ? 1.0f : -1.0f) * pi_div_m);
    
    for(j = 0; j < m; j++) {
      for(i = j; i < n; i += k) {
        t_re = w_re * re[i+m] - w_im * im[i+m];
        t_im = w_re * im[i+m] + w_im * re[i+m];
        re[i+m] = re[i] - t_re;
        im[i+m] = im[i] - t_im;
        re[i] = re[i] + t_re;
        im[i] = im[i] + t_im;
      }
      t_re = w_re * wm_re - w_im * wm_im;
      t_im = w_re * wm_im + w_im * wm_re;
      w_re = t_re; w_im = t_im;
    }
  }
  
  if(inverse) {
    for(i = 0; i < n; i++) {
      re[i] /= (float)n;
      im[i] /= (float)n;
    }
  }
}
```

### 5. Quinn插值频率精化

```c
// 频率精度提升：从 bin宽度 提升约 10倍
// f_bin = fs / N = 4.17MHz / 512 = 8140Hz
// Quinn插值后：约 800Hz 误差
if(max_idx > 0 && max_idx < (FFT_LENGTH/2 - 1)) {
  float y_m1 = fft_mag_sq[max_idx - 1];
  float y_0  = fft_mag_sq[max_idx];
  float y_p1 = fft_mag_sq[max_idx + 1];
  
  float ap = (y_p1 > 0) ? (y_p1 / y_0) : 0;
  float am = (y_m1 > 0) ? (y_m1 / y_0) : 0;
  float delta = (ap - am) / (1.0f - ap - am + 1e-10f);
  
  if(delta > 0.5f) delta = 0.5f;
  if(delta < -0.5f) delta = -0.5f;
  
  exact_freq = (max_idx + delta) * fs / FFT_LENGTH;
}
```

### 6. 三级上升沿检测

```c
static uint16_t Find_RisingEdge(uint8_t ch_num, uint16_t len)
{
  uint16_t i, consec_high = 0, edge_idx = 0xFFFF;
  uint8_t sig_min = 255, sig_max = 0;
  uint8_t thresh_low, thresh_high;
  uint16_t swing;
  uint8_t *buf = ch_buf[ch_num];
  
  // Level 1: 自适应阈值（前256点基线）
  for(i = 0; i < len && i < 256; i++) {
    if(buf[i] < sig_min) sig_min = buf[i];
    if(buf[i] > sig_max) sig_max = buf[i];
  }
  
  swing = sig_max - sig_min;
  if(swing < SIGNAL_SWING_MIN) return 0xFFFF;
  
  thresh_low  = sig_min + (uint8_t)(swing * 0.25f);
  thresh_high = sig_min + (uint8_t)(swing * 0.60f);
  
  // Level 2: 迟滞检测 + Level 3: 连续点确认
  for(i = 0; i < len; i++) {
    if(buf[i] > thresh_high) {
      consec_high++;
      if(consec_high >= EDGE_CONFIRM_PTS) {
        edge_idx = i - EDGE_CONFIRM_PTS + 1;
        break;
      }
    } else if(buf[i] < thresh_low) {
      consec_high = 0;
    }
  }
  
  return edge_idx;
}
```

---

## 四、Flash优化策略

### 1. HAL模块裁剪

```c
// stm32h7xx_hal_conf.h
#define HAL_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_SPI_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_HSEM_MODULE_ENABLED

// 禁用以下模块节省Flash:
// #define HAL_I2C_MODULE_ENABLED
// #define HAL_MDMA_MODULE_ENABLED
// #define HAL_UART_MODULE_ENABLED
```

### 2. 避免CMSIS-DSP依赖

| 方案 | Flash占用 | 优缺点 |
|------|-----------|--------|
| 自定义FFT | ~8KB | 无依赖，代码可控 |
| CMSIS-DSP + arm_cortexM7lfdp_math.lib | ~40KB+ | 优化好，体积大 |

### 3. 编译器优化

```c
// Keil MDK:
// Optimization: -O1 (平衡性能与体积)
// -Osize: 最小体积（FFT性能下降约20%）
// LTO: Link Time Optimization 使能
```

---

## 五、坑点记录

### 1. BDMA地址限制
**问题**：ADC3的BDMA只能访问AXI SRAM (0x24000000+)，不能访问DTCM (0x20000000)
**现象**：DMA传输无响应，ADC DR有数据但DMA不搬运
**解决**：所有BDMA缓冲区必须放在AXI SRAM段

### 2. PF10/PF11模拟开关
**问题**：PF10/PF11默认模拟开关关闭，ADC读取值异常
**现象**：ADC读数固定在中间值或全0
**解决**：
```c
HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PA0, SYSCFG_SWITCH_PA0_OPEN);
HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PA1, SYSCFG_SWITCH_PA1_OPEN);
```

### 3. 双ADC模式DMA配置
**问题**：DMA访问模式选择错误导致数据格式错误
**现象**：采样数据只有低字节有效，高字节全0
**解决**：使用DMA模式2，32位打包读取CDR寄存器

### 4. 双ADC采样顺序
**问题**：CDR寄存器格式：高16位=ADC2，低16位=ADC1
**现象**：通道数据错位
**解决**：解交织时注意字节顺序

### 5. CubeMX代码覆盖
**问题**：在CubeMX中重新生成代码会覆盖USER CODE外的所有修改
**解决**：
1. 关键代码必须放在 `/* USER CODE BEGIN X */` 和 `/* USER CODE END X */` 之间
2. 重写的文件（如adc.c）不要在CubeMX中重新生成对应外设
3. 修改前备份

### 6. C90编译器限制（ARMCC v5）
**问题**：变量必须在函数开头声明，不能在代码中间声明
**现象**：编译错误：`expected an expression`
**解决**：所有局部变量移到函数顶部

---

## 六、分散加载文件配置

```sct
; 07-ADC_Test_LCD1_8.sct
LR_IROM1 0x08000000 0x00020000  {  ; 128KB Flash
  ER_IROM1 0x08000000 0x00020000  {
    *.o (RESET, +First)
    *(InRoot$$Sections)
    .ANY (+RO)
  }
  
  RW_IRAM1 0x20000000 UNINIT 0x00020000  {  ; DTCM 128KB
    .ANY (.DTCMRAM)
  }
  
  RW_IRAM2 0x24000000 UNINIT 0x00080000  {  ; AXI SRAM 512KB
    .ANY (.AXISRAM)
    .ANY (+RW +ZI)
  }
}
```

---

## 七、主程序流程

```
启动
  ├── MPU配置
  ├── 使能Cache
  ├── HAL初始化
  ├── 系统时钟配置 (480MHz)
  ├── LCD初始化
  ├── ADC1+2双同步模式初始化
  ├── ADC3独立模式初始化
  ├── 启动DMA传输
  └── 采样率校准（1秒统计）

主循环
  ├── 处理DMA半完成/完成中断
  │   ├── Cache无效化
  │   └── 数据解交织到通道缓冲区
  ├── 轮询FFT（每次2通道，3次覆盖6通道）
  │   ├── DC移除
  │   ├── FFT运算
  │   ├── 幅度平方计算
  │   ├── SNR检测
  │   └── Quinn插值计算精确频率
  ├── 边沿检测 + TDOA计算（3轮后）
  └── LCD更新（200ms限流）
```

---

## 八、性能指标

| 参数 | 指标 |
|------|------|
| 每通道采样率 | ~4.17 Msps |
| FFT点数 | 512 |
| 频率分辨率 | 8140 Hz (bin宽度) |
| Quinn插值后精度 | ~800 Hz |
| 上升沿检测精度 | ~0.24 μs |
| 6通道FFT总耗时 | ~1.5 ms |
| Flash占用估计 | ~80-100 KB |
| RAM占用 | ~15 KB |

---

## 九、编译验证清单

- [ ] 代码编译无错误
- [ ] 代码编译无警告（或仅有预期警告）
- [ ] Flash size < 128KB
- [ ] RAM size < 512KB
- [ ] 分散加载文件配置正确
- [ ] 所有缓冲区段属性正确
- [ ] 中断向量表正确
- [ ] 时钟树配置正确

---

## 十、模块测试模式

### 测试模式配置

项目支持模块化测试，可以单独验证各个功能模块：

```c
// main.c - 测试模式选择
#define TEST_MODE       TEST_MODE_FULL  /* 默认完整功能测试 */
//#define TEST_MODE       TEST_MODE_PWM   /* 仅测试PWM输出 */
//#define TEST_MODE       TEST_MODE_DAC   /* 仅测试DAC输出 */
//#define TEST_MODE       TEST_MODE_ADC   /* 仅测试ADC采样 */
//#define TEST_MODE       TEST_MODE_LCD   /* 仅测试LCD显示 */
```

### 测试函数说明

| 测试函数 | 功能说明 | 初始化模块 |
|----------|----------|------------|
| `Test_PWM()` | 测试PWM输出 | TIM1 |
| `Test_DAC()` | 测试DAC正弦波输出 | TIM6, DAC1 |
| `Test_ADC()` | 测试ADC采样 | ADC1 + DMA |
| `Test_LCD()` | 测试LCD显示 | SPI4, ST7735 |

### 使用方法

1. 取消注释对应的`#define TEST_MODE`行
2. 注释其他模式定义
3. 重新编译下载
4. 通过LED闪烁判断模块是否正常工作

---

## 十一、下一步优化方向

1. **进一步Flash优化**
   - 启用LTO（链接时优化）
   - 使用-Osize优化级别
   - 移除未使用的HAL模块

2. **性能优化**
   - FFT定点化（Q15格式）
   - 使用CMSIS-DSP优化版本
   - 硬件加速器（H7无FFT硬件）

3. **功能扩展**
   - 增加UART数据输出
   - 增加FFT频谱显示
   - 增加触发模式

---

## 附：文件修改清单

| 文件 | 修改内容 |
|------|----------|
| Core/Inc/adc.h | 重写：ADC12+ADC3双模式API |
| Core/Src/adc.c | 重写：MY_ADC12/3_DMA_Init, Start/Stop, DeInterleave |
| Core/Src/stm32h7xx_it.c | 添加DMA2_Stream0和BDMA_Channel0中断处理 |
| Core/Inc/stm32h7xx_hal_conf.h | 裁剪HAL模块，禁用I2C/MDMA |
| MDK-ARM/*.sct | 添加.DTCMRAM和.AXISRAM段 |
| Core/Src/main.c | FFT算法 + 边沿检测 + LCD显示主逻辑 |

---

*文档版本：v1.0 | 更新日期：2026-05-10*
