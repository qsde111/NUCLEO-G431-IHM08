#include "mt6835_angle_corr.h"

#ifndef MT6835_ANGLE_CORR_ENABLE
#define MT6835_ANGLE_CORR_ENABLE (1U)
#endif

#if (MT6835_ANGLE_CORR_ENABLE != 0U)
#include "mt6835_angle_lut_1024.h"
#define MT6835_ANGLE_CORR_LUT_PTR (mt6835_angle_lut_raw21_1024)
#define MT6835_ANGLE_CORR_SHIFT (MT6835_ANGLE_LUT_SHIFT)
#define MT6835_ANGLE_CORR_FULL_SCALE (MT6835_ANGLE_FULL_SCALE)
#define MT6835_ANGLE_CORR_MASK ((uint32_t)(MT6835_ANGLE_CORR_FULL_SCALE - 1UL))

_Static_assert((MT6835_ANGLE_LUT_SIZE & (MT6835_ANGLE_LUT_SIZE - 1U)) == 0U, "MT6835 angle LUT size must be power-of-two");
_Static_assert(MT6835_ANGLE_LUT_SHIFT < 21U, "MT6835 angle LUT shift must be less than 21");
#endif

uint32_t Mt6835AngleCorr_ApplyRaw21(uint32_t raw21)
{
#if (MT6835_ANGLE_CORR_ENABLE != 0U)
    const uint32_t raw = raw21 & MT6835_ANGLE_CORR_MASK;
    const uint32_t idx = raw >> MT6835_ANGLE_CORR_SHIFT;
    const uint32_t frac_mask = (1UL << MT6835_ANGLE_CORR_SHIFT) - 1UL;
    const uint32_t frac = raw & frac_mask;

    const uint32_t y0 = MT6835_ANGLE_CORR_LUT_PTR[idx];
    const uint32_t y1 = MT6835_ANGLE_CORR_LUT_PTR[idx + 1U];
    const uint32_t corr = y0 + (uint32_t)(((uint64_t)(y1 - y0) * (uint64_t)frac) >> MT6835_ANGLE_CORR_SHIFT);
    return corr & MT6835_ANGLE_CORR_MASK;
#else
    return raw21 & 0x1FFFFFU;
#endif
}
