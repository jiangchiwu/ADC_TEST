/* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        arm_const_structs.c (F32-only trimmed for STM32H7)
 * Description:  仅保留 arm_cfft_sR_f32_len* 实例，避免链接 q31/q15 表
 * -------------------------------------------------------------------- */

#include "arm_const_structs.h"

/* Floating-point structs (F32 only) */
const arm_cfft_instance_f32 arm_cfft_sR_f32_len16 = {
    16, twiddleCoef_16, armBitRevIndexTable16, ARMBITREVINDEXTABLE_16_TABLE_LENGTH
};

const arm_cfft_instance_f32 arm_cfft_sR_f32_len32 = {
    32, twiddleCoef_32, armBitRevIndexTable32, ARMBITREVINDEXTABLE_32_TABLE_LENGTH
};

const arm_cfft_instance_f32 arm_cfft_sR_f32_len64 = {
    64, twiddleCoef_64, armBitRevIndexTable64, ARMBITREVINDEXTABLE_64_TABLE_LENGTH
};

const arm_cfft_instance_f32 arm_cfft_sR_f32_len128 = {
    128, twiddleCoef_128, armBitRevIndexTable128, ARMBITREVINDEXTABLE_128_TABLE_LENGTH
};

const arm_cfft_instance_f32 arm_cfft_sR_f32_len256 = {
    256, twiddleCoef_256, armBitRevIndexTable256, ARMBITREVINDEXTABLE_256_TABLE_LENGTH
};

const arm_cfft_instance_f32 arm_cfft_sR_f32_len512 = {
    512, twiddleCoef_512, armBitRevIndexTable512, ARMBITREVINDEXTABLE_512_TABLE_LENGTH
};

const arm_cfft_instance_f32 arm_cfft_sR_f32_len1024 = {
    1024, twiddleCoef_1024, armBitRevIndexTable1024, ARMBITREVINDEXTABLE_1024_TABLE_LENGTH
};

const arm_cfft_instance_f32 arm_cfft_sR_f32_len2048 = {
    2048, twiddleCoef_2048, armBitRevIndexTable2048, ARMBITREVINDEXTABLE_2048_TABLE_LENGTH
};

const arm_cfft_instance_f32 arm_cfft_sR_f32_len4096 = {
    4096, twiddleCoef_4096, armBitRevIndexTable4096, ARMBITREVINDEXTABLE_4096_TABLE_LENGTH
};
