# Debug: PC no trigger

Status: [OPEN]

## Symptom

输入信号存在，但 PC 工具一直没有触发时间/事件帧显示。

## Constraints

- 在获得运行证据前不修改业务逻辑。
- 优先通过现有 SRAM4/J-Link 诊断快照和构建日志确认问题。

## Hypotheses

1. H1: 最新固件尚未下载到板子，PC 当前运行的不是刚构建的跨 DMA 缓存版本。
2. H2: 新增 `PEAK_TRIGGER_400MV=496 LSB` 峰谷门限过高，实际输入峰谷相对基准未超过 400mV，导致候选全部被过滤。
3. H3: `DMA_EVENT_DELAY_FRAMES=3` 后，frame_pool 积压/丢帧，导致 Step A 无法稳定进入触发判断。
4. H4: 跨帧 FFT 数据提取或时间戳生成后，FFT 频率未通过 20k~200k 有效范围，事件未上送。
5. H5: UART/PC 端链路正常但没有事件帧进入 tx_ring，问题在固件触发/FFT过滤阶段。

## Evidence plan

- 读取当前代码关键路径，确认新增门限和延迟队列行为。
- 用现有 J-Link/SRAM4 诊断快照读取计数器：frame_seen、candidate、noise_filter、freq_filter、tx_ring push、uart evt、last max_dev/threshold/freq。
- 根据证据判断是没有触发候选、被 400mV 过滤、被 FFT 过滤、还是 UART 未发送。

## Runtime evidence 1

J-Link SRAM4 snapshot at `0x38002000` after 5s run:

- `magic=ADC75001`, firmware diagnostic snapshot exists.
- `loop_cnt=0x8FD`, `half_complete_cnt=0xAF1`, `full_complete_cnt=0xAF1`, `frame_seen_cnt=0x11F8`; ADC DMA and main frame processing are running.
- `uart_q_push_cnt=0`, `uart_q_send_cnt=0`, `tx_poll_send_ok=0`; no event frame entered UART tx path.
- `last_n_trig=0`, `prescan_hit_cnt=0`, `candidate_cnt=0`, `noise_filter_cnt=0`, `freq_filter_cnt=0`; no trigger candidate was produced, so the failure is before FFT/UART.
- `ch_max_dev≈13~14 LSB`, `ch_thr=0x140=320 LSB`, `ch_noise_pp≈3~4 LSB`; the ADC input seen by firmware is only noise-level and far below both the 260mV prescan threshold and the 400mV peak/valley threshold.

Current hypothesis status:

- H3 rejected for this snapshot: frame processing is running (`frame_seen_cnt` increasing), not blocked by the 3-frame delay queue.
- H4 rejected for this snapshot: FFT was not reached because there was no trigger candidate.
- H5 confirmed partially: UART has no frame because firmware did not generate trigger candidates.
- H2 not reached in this snapshot: the signal did not even pass the 320 LSB prescan threshold; it would also fail 496 LSB if unchanged.
- H1 still possible: the latest cross-DMA-cache build was built but not yet downloaded in the previous step.

## Runtime evidence 2

After flashing the latest BIN with J-Link and reading SRAM4 snapshot again after 5s:

- Latest BIN was programmed successfully to `0x08000000`.
- `frame_seen_cnt=0x5844`; ADC DMA and delayed frame processing are running.
- `tx_poll_call_cnt=0x2FD2`, but `uart_q_push_cnt=0`, `uart_q_send_cnt=0`, `tx_poll_send_ok=0`; UART path is idle because no event was generated.
- `last_n_trig=0`, `prescan_hit_cnt=0`, `candidate_cnt=0`, `noise_filter_cnt=0`, `freq_filter_cnt=0`; detection never reached candidate/FFT/UART stages.
- `ch_max_dev≈10~13 LSB`, `ch_thr=0x140=320 LSB`, `ch_noise_pp≈5~7 LSB`; ADC input is still noise-level only, approximately 8~10mV at 3.3V/12bit.

Conclusion from evidence 2:

- H1 rejected: latest firmware is now downloaded and running.
- H3 rejected: delayed DMA queue is not stuck; frames are processed continuously.
- H4 rejected: FFT is not reached.
- H5 confirmed: PC has no frame because firmware generates no event.
- Root observation: MCU ADC pins are not seeing the expected input amplitude during the snapshot window, or the signal amplitude at ADC pin is far below current thresholds.

## Logic review after user confirmation

User confirmed:

- Oscilloscope probes are on ADC pins, not pre-stage nodes.
- CH1/CH2 wiring is still PC0/ADC1_INP10 and PC1/ADC1_INP11.
- Signal is common-grounded with MCU.
- Signal peak/valley relative to channel baseline really exceeds 400mV.

Static logic review found two likely blocking points introduced by the recent changes:

1. The pre-scan threshold was `DEV_THRESHOLD_260MV=320 LSB`, not the previous ~200mV behavior. This can reject candidate starts earlier than intended. It is changed to `DEV_THRESHOLD_200MV=248 LSB` for candidate start detection, while keeping the final first-peak/valley threshold at `PEAK_TRIGGER_400MV=496 LSB`.
2. The previous `prev_tail_active + Has_PreTrigger_Silence` gate conflicts with the new design of delaying by 3 DMA frames and judging with a cached multi-frame window. It can suppress valid signals that are already active at the oldest cached frame. This gate was removed. Detection now uses:
   - oldest cached frame transition candidate,
   - sustain validation across the 4-frame cached window,
   - final first peak/valley amplitude >400mV,
   - FFT validity and same-channel dedup in Step B.

Build after logic cleanup:

- Keil build succeeded: 0 errors, 61 warnings.
- Build timestamp: 2026-06-14T07:47:23+08:00.

## Event interval follow-up

User reported PC now has events, but event-frame intervals still differ from oscilloscope.

Runtime snapshot with events showed:

- `candidate_cnt=0x63`, `uart_evt_cnt=0x4c`: event generation and UART upload are working.
- This also revealed the side effect of the previous no-trigger fix: removing `ch_active` caused a sustained input burst to generate candidates repeatedly on consecutive delayed DMA windows. PC intervals could then reflect software window cadence or same-channel dedup timing instead of the oscilloscope burst interval.

Fix:

- Restored per-channel active latch and silent-frame release:
  - first valid sustained trigger while inactive generates one event;
  - the channel stays active while signal remains present;
  - only after consecutive silent frames is the channel allowed to trigger again.
- Kept the 200mV candidate threshold and 400mV first peak/valley threshold.
- Kept multi-frame sustain/peak/FFT extraction.

Build/flash:

- Keil build succeeded: 0 errors, 61 warnings.
- Build timestamp: 2026-06-14T07:56:45+08:00.
- Latest BIN was flashed successfully by J-Link.

## PA5 closed-loop test source

User suggested using PA5 to output DC + sine signal through the external processing chain for closed-loop debugging.

Implemented configuration:

- `ENABLE_DAC_SIGNAL_SOURCE=0`: keep normal external ADC event-detection path.
- `DAC_AS_EXTERNAL_TEST_SRC=1`: PA5/DAC1_OUT2 outputs test bursts while event detection still treats ADC as external input.
- Test waveform fixed to stress case 0: 40kHz, 1000mV fundamental, no harmonic/noise rotation.
- Timing: 50ms DC + 10ms sine burst.

Build/flash:

- Keil build succeeded: 0 errors, 60 warnings.
- Build timestamp: 2026-06-14T07:58:43+08:00.
- Latest BIN was flashed successfully by J-Link.

Runtime snapshot after PA5 closed-loop firmware:

- `stress_burst_total=0x29D4`: PA5 test burst state machine is running.
- `dac_cr=0x80`, `dac_dhr12r2=0x7C`, `tim7_arr=0x80`, `dac_actual_freq_hz=0x25540`: DAC/TIM7 diagnostics are active.
- `candidate_cnt=0`, `uart_q_push_cnt=0`, `ch_max_dev≈8~12 LSB`: ADC side did not receive a detectable signal during snapshot.

Conclusion:

- Firmware is generating the PA5 test source.
- The external processing chain/output is not yet reaching the ADC pins at detectable amplitude during the snapshot, or the PA5 path is not connected/enabled as expected.
- Once PA5→external processing→ADC is connected and ADC max_dev rises above threshold, this closed-loop mode can be used to compare DAC burst cadence vs PC event cadence.
