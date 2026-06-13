# Debug: channel time delta

Status: [OPEN]

## Symptom

不同通道触发事件帧上送的时间差，与示波器观察到的 ADC 引脚实际信号时间差不一致。

## Evidence plan

本工程是 MCU 固件，无法直接通过 HTTP 日志上报运行时数据；本次使用已有 J-Link + SRAM4 诊断快照方式采集运行证据，并在必要时只增加 SRAM4 诊断字段作为 instrumentation。

## Hypotheses

1. H1: 当前事件时间用统一 `fr->start_time_us`，但 ADC1/ADC2/ADC3 DMA 半帧完成时间并不完全一致，导致不同 ADC 实例的通道使用了错误时间原点。
2. H2: DMA 半帧起始时间计算使用了错误的 half/full ISR 时间戳，导致半区边界附近事件的绝对时间偏移。
3. H3: 波峰/波谷定位只返回峰谷样本序号，但没有补偿 ADC rank 交织采样的通道内采样偏移，导致同 ADC 内两个 rank 存在固定采样相位误差。
4. H4: “第一个波峰/波谷”搜索从阈值点后开始，但阈值点可能受噪声/窗口判定影响提前或滞后，导致选到的不是示波器关注的那个首个有效峰谷。
5. H5: PC 端显示的 GAP 计算方式与固件事件时间定义不一致，导致视觉上看起来时间差不一致。

## Current step

Collected source evidence and runtime diagnostic evidence.

## Evidence

- Source evidence: `check_dma_and_push_frames()` used `adc1_dma_half_cyc` / `adc1_dma_full_cyc` to compute one shared `fr->start_time_us` for all 6 channels.
- Channel mapping evidence: CH1/CH2 come from ADC1, CH3/CH4 from ADC3, CH5/CH6 from ADC2.
- Runtime snapshot evidence from `diag_snapshot`:
  - half ISR cycles: ADC1=`0x1118883D`, ADC2=`0x11188B8A`, ADC3=`0x11188554`
  - full ISR cycles: ADC1=`0x1116669F`, ADC2=`0x111669A2`, ADC3=`0x1116637F`
  - half delta vs ADC1: ADC2=`+0x34D` cycles ≈ `+1.76us`, ADC3=`-0x2E9` cycles ≈ `-1.55us`
  - full delta vs ADC1: ADC2=`+0x303` cycles ≈ `+1.61us`, ADC3=`-0x320` cycles ≈ `-1.67us`
- Conclusion: H1 confirmed. Different ADC instances cannot share ADC1 frame time origin.
- H3 partially confirmed by design: rank2 within each 2-rank ADC scan is sampled after rank1, so rank2 needs half-sample-period compensation for sub-us precision.

## Fix plan

- Store per-channel DMA frame start time in `adc_frame_t`.
- Use ADC1 start time for CH1/CH2, ADC3 start time for CH3/CH4, ADC2 start time for CH5/CH6.
- Add `+0.5 sample` time compensation for rank2 channels CH2/CH4/CH6.

## Fix implemented

- `adc_frame_t` now stores `ch_start_time_us[ADC_NCH]`.
- `push_frame_from_dma()` receives per-channel start times instead of one shared start time.
- `Fill_ChannelStartTimes()` converts ADC1/ADC2/ADC3 DMA ISR timestamps to their own half-frame start times.
- Event timestamp now uses `fr->ch_start_time_us[ch] + event_us_in_frame`.

## Verification

- Keil build: `uv4_time_delta_fix.log`, 0 errors, 65 warnings.
- BIN generated: `F:\work\ADC_TEST\MDK-ARM\ADC_TEST\ADC_TEST.bin`, timestamp `2026/6/14 0:31:30`.
- J-Link flash: Program + Verify completed.
- Runtime snapshot: `magic=ADC75001`, ADC/main loop counters increasing, `adc_fs_hz≈0x0035ACE7≈3.519MHz`.

Status: firmware fixed; PC display issue found from screenshot follow-up.

## Screenshot follow-up

User provided screenshot in `F:\work\ADC_TEST\test\测试波形.png`.

Additional finding:

- PC GUI originally calculated Event Stream `EVENT GAP` by serial receive order.
- Firmware may enqueue events in channel scan order CH1..CH6 for the same DMA frame, while true event order is determined by `TS_US`.
- Therefore PC `EVENT GAP` can differ from oscilloscope channel timing even when firmware timestamps are correct.

PC fix:

- Sort each batch of received events by `TS_US` before calculating global Event Stream gap.
- Keep channel card gap as same-channel adjacent-event interval.
- Updated `event_frame_protocol.md` to document this display semantics.
- Rebuilt `F:\ADC_FFT\test_scripts\dist\event_monitor_gui_frame12_115200.exe`.
