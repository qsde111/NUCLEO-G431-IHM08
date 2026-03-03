#ifndef COMPONENTS_FOC_SPEED_CTRL_H
#define COMPONENTS_FOC_SPEED_CTRL_H

#include <stdint.h>

typedef struct
{
    float kp;         /* A / (rad/s) */
    float ki;         /* A / (rad/s*s) */
    float dt_s;       /* speed loop period */
    float iq_limit_a; /* output clamp */

    float iq_int_a;
} FocSpeedCtrl;

static inline void FocSpeedCtrl_Init(FocSpeedCtrl *ctx, float kp, float ki, float dt_s, float iq_limit_a)
{
    if ((ctx == 0) || (dt_s <= 0.0f) || (iq_limit_a <= 0.0f))
    {
        return;
    }

    ctx->kp = kp;
    ctx->ki = ki;
    ctx->dt_s = dt_s;
    ctx->iq_limit_a = iq_limit_a;
    ctx->iq_int_a = 0.0f;
}

/* 速度环积分器清零 */
static inline void FocSpeedCtrl_Reset(FocSpeedCtrl *ctx)
{
    if (ctx == 0)
    {
        return;
    }
    ctx->iq_int_a = 0.0f;
}

static inline float FocSpeedCtrl_Clamp(float x, float lo, float hi)
{
    if (x < lo)
    {
        return lo;
    }
    if (x > hi)
    {
        return hi;
    }
    return x;
}

static inline float FocSpeedCtrl_Step(FocSpeedCtrl *ctx, float omega_ref_rad_s, float omega_meas_rad_s)
{
    if ((ctx == 0) || (ctx->dt_s <= 0.0f) || (ctx->iq_limit_a <= 0.0f))
    {
        return 0.0f;
    }

    const float e = omega_ref_rad_s - omega_meas_rad_s;
    const float p = ctx->kp * e;
    const float i_next = ctx->iq_int_a + (ctx->ki * e * ctx->dt_s);

    const float iq_unsat = p + i_next;
    const float iq = FocSpeedCtrl_Clamp(iq_unsat, -ctx->iq_limit_a, ctx->iq_limit_a);

    /* anti-windup: back-calculate i-term to match saturated output */
    ctx->iq_int_a = iq - p;
    return iq;
}

#endif /* COMPONENTS_FOC_SPEED_CTRL_H */
