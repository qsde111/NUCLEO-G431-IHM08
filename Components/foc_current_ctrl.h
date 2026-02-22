#ifndef COMPONENTS_FOC_CURRENT_CTRL_H
#define COMPONENTS_FOC_CURRENT_CTRL_H

#include <math.h>
#include <stdint.h>

typedef struct
{
    float kp;         /* V/A */
    float ki;         /* V/(A*s) */
    float dt_s;       /* control period */
    float vbus_v;     /* DC bus voltage */
    float v_limit_pu; /* modulation limit in per-unit of vbus (e.g. 1/sqrt(3) for SVPWM linear region) */

    float id_int_v;
    float iq_int_v;
} FocCurrentCtrl;

typedef struct
{
    float id_a;
    float iq_a;
    float ud_v;
    float uq_v;
    float ud_pu;
    float uq_pu;
} FocCurrentCtrlOut;

static inline void FocCurrentCtrl_Init(FocCurrentCtrl *ctx, float kp, float ki, float dt_s, float vbus_v, float v_limit_pu)
{
    if (ctx == 0)
    {
        return;
    }

    ctx->kp = kp;
    ctx->ki = ki;
    ctx->dt_s = dt_s;
    ctx->vbus_v = vbus_v;
    ctx->v_limit_pu = v_limit_pu;
    ctx->id_int_v = 0.0f;
    ctx->iq_int_v = 0.0f;
}

static inline void FocCurrentCtrl_Reset(FocCurrentCtrl *ctx)
{
    if (ctx == 0)
    {
        return;
    }
    ctx->id_int_v = 0.0f;
    ctx->iq_int_v = 0.0f;
}

static inline float FocCurrentCtrl_Clamp(float x, float lo, float hi)
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

static inline void FocCurrentCtrl_StepSc(FocCurrentCtrl *ctx, float ia_a, float ib_a, float ic_a, float sin_theta_e,
                                         float cos_theta_e, float id_ref_a, float iq_ref_a, FocCurrentCtrlOut *out)
{
    (void)ic_a; /* only needed for validity checks; Clarke assumes ia+ib+ic=0 */

    if ((ctx == 0) || (out == 0) || (ctx->dt_s <= 0.0f) || (ctx->vbus_v <= 0.0f))
    {
        return;
    }

    const float inv_sqrt3 = 0.57735026919f;
    const float i_alpha = ia_a;
    const float i_beta = (ia_a + 2.0f * ib_a) * inv_sqrt3;

    const float id_a = (i_alpha * cos_theta_e) + (i_beta * sin_theta_e);
    const float iq_a = (-i_alpha * sin_theta_e) + (i_beta * cos_theta_e);

    const float ed = id_ref_a - id_a;
    const float eq = iq_ref_a - iq_a;

    const float v_limit_v = (ctx->v_limit_pu > 0.0f) ? (ctx->vbus_v * ctx->v_limit_pu) : ctx->vbus_v;

    /* d-axis PI with integrator clamping (anti-windup) */
    const float pd = ctx->kp * ed;
    float id_int = ctx->id_int_v + (ctx->ki * ed * ctx->dt_s);
    float ud_v = pd + id_int;
    ud_v = FocCurrentCtrl_Clamp(ud_v, -v_limit_v, v_limit_v);
    ctx->id_int_v = ud_v - pd;

    /* q-axis PI with integrator clamping (anti-windup) */
    const float pq = ctx->kp * eq;
    float iq_int = ctx->iq_int_v + (ctx->ki * eq * ctx->dt_s);
    float uq_v = pq + iq_int;
    uq_v = FocCurrentCtrl_Clamp(uq_v, -v_limit_v, v_limit_v);
    ctx->iq_int_v = uq_v - pq;

    out->id_a = id_a;
    out->iq_a = iq_a;
    out->ud_v = ud_v;
    out->uq_v = uq_v;
    out->ud_pu = ud_v / ctx->vbus_v;
    out->uq_pu = uq_v / ctx->vbus_v;
}

static inline void FocCurrentCtrl_Step(FocCurrentCtrl *ctx, float ia_a, float ib_a, float ic_a, float theta_e_rad, float id_ref_a,
                                       float iq_ref_a, FocCurrentCtrlOut *out)
{
    const float s = sinf(theta_e_rad);
    const float c = cosf(theta_e_rad);
    FocCurrentCtrl_StepSc(ctx, ia_a, ib_a, ic_a, s, c, id_ref_a, iq_ref_a, out);
}

#endif /* COMPONENTS_FOC_CURRENT_CTRL_H */
