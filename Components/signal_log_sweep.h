#ifndef COMPONENTS_SIGNAL_LOG_SWEEP_H
#define COMPONENTS_SIGNAL_LOG_SWEEP_H

#include <math.h>
#include <stdint.h>

typedef struct
{
    float amp;
    float f_start_hz;
    float f_end_hz;
    float duration_s;
    float dt_s;

    float t_s;
    float phase_rad;
    float f_hz;
    float f_ratio_per_step;

    uint8_t active;
    uint8_t done;
} SignalLogSweep;

static inline void SignalLogSweep_Reset(SignalLogSweep *ctx)
{
    if (ctx == 0)
    {
        return;
    }
    ctx->t_s = 0.0f;
    ctx->phase_rad = 0.0f;
    ctx->f_hz = 0.0f;
    ctx->f_ratio_per_step = 1.0f;
    ctx->active = 0U;
    ctx->done = 0U;
}

static inline void SignalLogSweep_Start(SignalLogSweep *ctx, float amp, float f_start_hz, float f_end_hz, float duration_s, float dt_s)
{
    if ((ctx == 0) || (dt_s <= 0.0f) || (duration_s <= 0.0f) || (f_start_hz <= 0.0f) || (f_end_hz <= 0.0f))
    {
        return;
    }

    ctx->amp = amp;
    ctx->f_start_hz = f_start_hz;
    ctx->f_end_hz = f_end_hz;
    ctx->duration_s = duration_s;
    ctx->dt_s = dt_s;

    ctx->t_s = 0.0f;
    ctx->phase_rad = 0.0f;
    ctx->f_hz = f_start_hz;

    const float r = f_end_hz / f_start_hz;
    const float steps = duration_s / dt_s;
    if (steps > 0.0f)
    {
        ctx->f_ratio_per_step = expf(logf(r) / steps);
    }
    else
    {
        ctx->f_ratio_per_step = 1.0f;
    }

    ctx->active = 1U;
    ctx->done = 0U;
}

static inline void SignalLogSweep_Stop(SignalLogSweep *ctx)
{
    if (ctx == 0)
    {
        return;
    }
    ctx->active = 0U;
    ctx->done = 1U;
}

static inline float SignalLogSweep_Step(SignalLogSweep *ctx)
{
    if ((ctx == 0) || (ctx->active == 0U))
    {
        return 0.0f;
    }

    if (ctx->t_s >= ctx->duration_s)
    {
        ctx->active = 0U;
        ctx->done = 1U;
        return 0.0f;
    }

    const float y = ctx->amp * sinf(ctx->phase_rad);

    const float two_pi = 6.28318530718f;
    ctx->phase_rad += two_pi * ctx->f_hz * ctx->dt_s;
    while (ctx->phase_rad >= two_pi)
    {
        ctx->phase_rad -= two_pi;
    }
    while (ctx->phase_rad < 0.0f)
    {
        ctx->phase_rad += two_pi;
    }

    ctx->f_hz *= ctx->f_ratio_per_step;
    ctx->t_s += ctx->dt_s;

    return y;
}

#endif /* COMPONENTS_SIGNAL_LOG_SWEEP_H */

