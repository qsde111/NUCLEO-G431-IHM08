#ifndef COMPONENTS_IQ_LUT_COMP_H
#define COMPONENTS_IQ_LUT_COMP_H

#include <stdint.h>

/**
 * @brief Iq 全周补偿：按机械角度(raw21)查表，输出 Iq_comp (A)
 *
 * 说明：
 * - raw21 来自 MT6835 的 21bit 绝对角度（0..2^21-1），与机械角度一一对应。
 * - 当前实现只接入单组 LUT（低速实验），不做速度维度插值/切换。
 */

float IqLutComp_SampleRaw21(uint32_t raw21);

#endif /* COMPONENTS_IQ_LUT_COMP_H */
