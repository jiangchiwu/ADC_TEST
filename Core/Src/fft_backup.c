/**
 * ============================================================================
 * STATUS: OBSOLETE BACKUP — 不参与编译
 * 本文件是 main.c FFT 代码段在 2026-06-04 的手动备份。
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

/* 原始FFT计算函数 - 保存用于对比 */
static float Calc_FFT_Frequency_in_Frame_Original(const uint16_t *buf, uint16_t n_samples, uint8_t ch, uint16_t start_idx)
{
  uint16_t i;
  float max_val = 0.0f;
  uint16_t max_bin = 0;
  float avg_val = 0.0f;
  float delta, exact_bin;
  uint16_t bin_min, bin_max;
  uint16_t max_samples = n_samples - start_idx;
  uint32_t dc_sum = 0;
  const uint16_t *src;
  float dc_avg;

  if(max_samples < FFT_LENGTH) return 0.0f;
  max_samples = FFT_LENGTH;

  src = buf + start_idx * ADC_NCH + ch;

  /* 使用全局直流偏置基线（如果有效），否则计算本地平均 */
  if(ch_dc_offset_valid && ch_dc_offset[ch] != 0) {
    dc_avg = (float)ch_dc_offset[ch];
  } else {
    for(i = 0; i < max_samples; i++) {
      dc_sum += src[i * ADC_NCH];
    }
    dc_avg = (float)dc_sum / (float)FFT_LENGTH;
  }

  static float fft_in[FFT_LENGTH] __attribute__((section(".DTCMRAM"), aligned(32)));
  for(i = 0; i < max_samples; i++) {
    fft_in[i] = (float)src[i * ADC_NCH] - dc_avg;
  }
  for(; i < FFT_LENGTH; i++) fft_in[i] = 0.0f;

  uint32_t fft_t0 = DWT->CYCCNT;
  arm_rfft_fast_f32(&rfft_inst, fft_in, fft_buf, 0);
  *(volatile uint32_t*)&fft_last_cyc = DWT->CYCCNT - fft_t0;

  fft_mag[0] = fft_buf[0] * fft_buf[0];
  arm_cmplx_mag_squared_f32(fft_buf + 2, fft_mag + 1, FFT_LENGTH / 2 - 1);

  /* 固定频率范围：20kHz ~ 200kHz */
  bin_min = (uint16_t)(20000.0f * (float)FFT_LENGTH / adc_fs_hz);
  bin_max = (uint16_t)(200000.0f * (float)FFT_LENGTH / adc_fs_hz);
  if(bin_min < 1) bin_min = 1;
  if(bin_max > FFT_LENGTH / 2 - 1) bin_max = FFT_LENGTH / 2 - 1;

  /* 找频率范围内的最大峰值 */
  for(i = bin_min; i <= bin_max; i++) {
    if(fft_mag[i] > max_val) {
      max_val = fft_mag[i];
      max_bin = i;
    }
  }
  if(max_bin == 0) return 0.0f;

  for(i = bin_min; i <= bin_max; i++) avg_val += fft_mag[i];
  avg_val /= (float)(bin_max - bin_min + 1);
  if(max_val < 1.5f * avg_val) return 0.0f;

  /* 抛物线插值 */
  if(max_bin > 0 && max_bin < (FFT_LENGTH / 2 - 1)) {
    float ym, y0, yp, denom;
    ym = sqrtf(fft_mag[max_bin - 1]);
    y0 = sqrtf(fft_mag[max_bin]);
    yp = sqrtf(fft_mag[max_bin + 1]);
    denom = ym - 2.0f * y0 + yp;
    if(fabsf(denom) > 1e-9f) {
      delta = 0.5f * (ym - yp) / denom;
      if(delta > 0.5f) delta = 0.5f;
      if(delta < -0.5f) delta = -0.5f;
    } else {
      delta = 0.0f;
    }
  } else {
    delta = 0.0f;
  }
  exact_bin = (float)max_bin + delta;
  return exact_bin * adc_fs_hz / (float)FFT_LENGTH;
}
