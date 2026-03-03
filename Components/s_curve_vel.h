#ifndef COMPONENTS_S_CURVE_VEL_H
#define COMPONENTS_S_CURVE_VEL_H

#include <stdint.h>

typedef struct
{
    float dt_s;
    float a_max;
    float j_max;
    float k_a; /* accel command gain: a_des = k_a*(v_tgt - v) */

    float v;
    float a;
} SCurveVel;

static inline void SCurveVel_Init(SCurveVel *ctx, float dt_s, float a_max, float j_max, float k_a)
{
    if ((ctx == 0) || (dt_s <= 0.0f) || (a_max <= 0.0f) || (j_max <= 0.0f) || (k_a <= 0.0f))
    {
        return;
    }
    ctx->dt_s = dt_s;
    ctx->a_max = a_max;
    ctx->j_max = j_max;
    ctx->k_a = k_a;
    ctx->v = 0.0f;
    ctx->a = 0.0f;
}

static inline void SCurveVel_Reset(SCurveVel *ctx, float v_now)
{
    if (ctx == 0)
    {
        return;
    }
    ctx->v = v_now;
    ctx->a = 0.0f;
}

static inline float SCurveVel_Clamp(float x, float lo, float hi)
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

static inline float SCurveVel_Step(SCurveVel *ctx, float v_tgt)
{
    if ((ctx == 0) || (ctx->dt_s <= 0.0f) || (ctx->a_max <= 0.0f) || (ctx->j_max <= 0.0f) || (ctx->k_a <= 0.0f))
    {
        return v_tgt;
    }

    const float e = v_tgt - ctx->v;
    float a_des = ctx->k_a * e;
    a_des = SCurveVel_Clamp(a_des, -ctx->a_max, ctx->a_max);

    const float da_max = ctx->j_max * ctx->dt_s;
    const float da = SCurveVel_Clamp(a_des - ctx->a, -da_max, da_max);
    ctx->a = SCurveVel_Clamp(ctx->a + da, -ctx->a_max, ctx->a_max);
    ctx->v += ctx->a * ctx->dt_s;

    return ctx->v;
}

#endif /* COMPONENTS_S_CURVE_VEL_H */

