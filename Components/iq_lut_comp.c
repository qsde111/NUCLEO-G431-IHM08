#include "iq_lut_comp.h"

/* Select which low-speed LUT to use for experiments (manual switch).
 * - 15: include `iq_comp_lut_v15.h`
 * - 20: include `iq_comp_lut_v20.h`
 */
#ifndef IQ_LUT_COMP_TABLE_V_RAD_S
#define IQ_LUT_COMP_TABLE_V_RAD_S (20)
#endif

#if (IQ_LUT_COMP_TABLE_V_RAD_S == 15)
#include "iq_comp_lut_v15.h"
#define IQ_LUT_COMP_LUT_PTR (iq_comp_lut_a_v15)
#elif (IQ_LUT_COMP_TABLE_V_RAD_S == 20)
#include "iq_comp_lut_v20.h"
#define IQ_LUT_COMP_LUT_PTR (iq_comp_lut_a_v20)
#else
#error "Unsupported IQ_LUT_COMP_TABLE_V_RAD_S (supported: 15, 20)"
#endif

_Static_assert((IQ_COMP_LUT_SIZE & (IQ_COMP_LUT_SIZE - 1U)) == 0U, "IqLutComp LUT size must be power-of-two");
_Static_assert(IQ_COMP_LUT_SIZE <= (1U << 21U), "IqLutComp LUT size too large for raw21 indexing");

static inline uint8_t IqLutComp_ShiftForSize(uint16_t size)
{
    /* raw21 is 21-bit, so shift = 21 - log2(size) */
    switch (size)
    {
    case 256U:
        return 13U;
    case 512U:
        return 12U;
    case 1024U:
        return 11U;
    case 2048U:
        return 10U;
    default:
        return 0U;
    }
}

static inline float IqLutComp_Lerp(float a, float b, float t)
{
    return a + ((b - a) * t);
}

static inline float IqLutComp_SampleLutRaw21(const float *lut, uint16_t size, uint32_t raw21)
{
    if ((lut == 0) || (size == 0U) || ((size & (size - 1U)) != 0U))
    {
        return 0.0f;
    }

    const uint8_t shift = IqLutComp_ShiftForSize(size);
    if (shift == 0U)
    {
        return 0.0f;
    }

    const uint32_t raw = raw21 & 0x1FFFFFU;
    const uint32_t idx = raw >> shift; /* 0..size-1 */
    const uint32_t frac_mask = (1U << shift) - 1U;
    const uint32_t frac = raw & frac_mask;
    const uint32_t idx2 = (idx + 1U) & (uint32_t)(size - 1U);

    const float a = lut[idx];
    const float b = lut[idx2];
    const float t = (float)frac * (1.0f / (float)(1U << shift));
    return IqLutComp_Lerp(a, b, t);
}

float IqLutComp_SampleRaw21(uint32_t raw21)
{
    return IqLutComp_SampleLutRaw21(IQ_LUT_COMP_LUT_PTR, (uint16_t)IQ_COMP_LUT_SIZE, raw21);
}
