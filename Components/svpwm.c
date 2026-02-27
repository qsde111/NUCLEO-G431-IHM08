#include "svpwm.h"

static float Svpwm_Max3(float a, float b, float c)
{
    float m = (a > b) ? a : b;
    return (m > c) ? m : c;
}

static float Svpwm_Min3(float a, float b, float c)
{
    float m = (a < b) ? a : b;
    return (m < c) ? m : c;
}

static float Svpwm_Clamp01(float x)
{
    if (x < 0.0f)
    {
        return 0.0f;
    }
    if (x > 1.0f)
    {
        return 1.0f;
    }
    return x;
}

static uint8_t Svpwm_Sector(float u_alpha, float u_beta)
{
    const float k = 0.86602540378f; /* sqrt(3)/2 */
    const float x = u_beta;
    const float y = (k * u_alpha) - (0.5f * u_beta);
    const float z = (-k * u_alpha) - (0.5f * u_beta);

    const uint8_t sx = (x > 0.0f) ? 1U : 0U;
    const uint8_t sy = (y > 0.0f) ? 1U : 0U;
    const uint8_t sz = (z > 0.0f) ? 1U : 0U;
    const uint8_t code = (uint8_t)(sx | (uint8_t)(sy << 1) | (uint8_t)(sz << 2));

    /* Standard (x,y,z) sign mapping */
    switch (code)
    {
    case 3U:
        return 1U;
    case 1U:
        return 2U;
    case 5U:
        return 3U;
    case 4U:
        return 4U;
    case 6U:
        return 5U;
    case 2U:
        return 6U;
    default:
        return 0U;
    }
}

/* 用α β轴给定电压进行反Clarke变换，零序电压注入算出三相占空比*/
void Svpwm_Calc(float u_alpha, float u_beta, SvpwmOut *out)
{
    if (out == 0)
    {
        return;
    }

    /* Min-max common-mode injection SVPWM (u_* in V/Vbus, typical magnitude <= ~0.577) */
    float va = u_alpha;
    float vb = (-0.5f * u_alpha) + (0.86602540378f * u_beta);
    float vc = (-0.5f * u_alpha) - (0.86602540378f * u_beta);

    const float v_max = Svpwm_Max3(va, vb, vc);
    const float v_min = Svpwm_Min3(va, vb, vc);
    const float v_off = -0.5f * (v_max + v_min);

    va += v_off;
    vb += v_off;
    vc += v_off;

    out->duty_a = Svpwm_Clamp01(va + 0.5f);
    out->duty_b = Svpwm_Clamp01(vb + 0.5f);
    out->duty_c = Svpwm_Clamp01(vc + 0.5f);
    out->sector = Svpwm_Sector(u_alpha, u_beta);
}
