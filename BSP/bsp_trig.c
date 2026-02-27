#include "bsp_trig.h"

#include "main.h"

#include <math.h>

#if defined(CORDIC)
#include "stm32g4xx_ll_cordic.h"
#endif

void BspTrig_Init(void)
{
#if defined(CORDIC)
    LL_CORDIC_Config(CORDIC, LL_CORDIC_FUNCTION_COSINE, LL_CORDIC_PRECISION_6CYCLES, LL_CORDIC_SCALE_0, LL_CORDIC_NBWRITE_1,
                     LL_CORDIC_NBREAD_2, LL_CORDIC_INSIZE_32BITS, LL_CORDIC_OUTSIZE_32BITS);
#endif
}

/* 根据电角度theta_rad，算出sin cos并赋值到*sin_out *cos_out */
void BspTrig_SinCos(float theta_rad, float *sin_out, float *cos_out)
{
    if ((sin_out == 0) || (cos_out == 0))
    {
        return;
    }

#if defined(CORDIC)
    const float pi = 3.14159265358979323846f;
    const float two_pi = 6.28318530717958647692f;

    /* CORDIC expects angle in Q1.31, representing [-pi, pi) mapped to [-1, 1). */
    float t = theta_rad;
    while (t >= pi)
    {
        t -= two_pi;
    }
    while (t < -pi)
    {
        t += two_pi;
    }

    const float inv_pi = 0.31830988618379067154f;
    const float q31_scale = 2147483648.0f; /* 2^31 */
    const int32_t arg_q31 = (int32_t)(t * inv_pi * q31_scale);

    LL_CORDIC_WriteData(CORDIC, (uint32_t)arg_q31);
    while (LL_CORDIC_IsActiveFlag_RRDY(CORDIC) == 0U)
    {
    }

    const int32_t cos_q31 = (int32_t)LL_CORDIC_ReadData(CORDIC);
    const int32_t sin_q31 = (int32_t)LL_CORDIC_ReadData(CORDIC);

    const float q31_to_float = 4.656612873077392578125e-10f; /* 1/2^31 */
    *cos_out = (float)cos_q31 * q31_to_float;
    *sin_out = (float)sin_q31 * q31_to_float;
#else
    *sin_out = sinf(theta_rad);
    *cos_out = cosf(theta_rad);
#endif
}
