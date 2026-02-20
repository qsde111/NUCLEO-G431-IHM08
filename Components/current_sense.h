#ifndef COMPONENTS_CURRENT_SENSE_H
#define COMPONENTS_CURRENT_SENSE_H

#include <stdint.h>

typedef struct
{
    uint32_t target_samples;
    uint32_t sample_count;
    uint32_t sum_a;
    uint32_t sum_b;
    uint16_t offset_a;
    uint16_t offset_b;
    uint8_t ready;
} CurrentSenseOffset2;

static inline void CurrentSenseOffset2_Init(CurrentSenseOffset2 *ctx, uint32_t target_samples)
{
    if (ctx == 0)
    {
        return;
    }

    ctx->target_samples = (target_samples == 0U) ? 1U : target_samples;
    ctx->sample_count = 0U;
    ctx->sum_a = 0U;
    ctx->sum_b = 0U;
    ctx->offset_a = 0U;
    ctx->offset_b = 0U;
    ctx->ready = 0U;
}

static inline void CurrentSenseOffset2_Push(CurrentSenseOffset2 *ctx, uint16_t raw_a, uint16_t raw_b)
{
    if (ctx == 0)
    {
        return;
    }
    if (ctx->ready != 0U)
    {
        return;
    }

    ctx->sum_a += (uint32_t)raw_a;
    ctx->sum_b += (uint32_t)raw_b;
    ctx->sample_count++;

    if (ctx->sample_count >= ctx->target_samples)
    {
        ctx->offset_a = (uint16_t)(ctx->sum_a / ctx->sample_count);
        ctx->offset_b = (uint16_t)(ctx->sum_b / ctx->sample_count);
        ctx->ready = 1U;
    }
}

static inline uint8_t CurrentSenseOffset2_Ready(const CurrentSenseOffset2 *ctx)
{
    return (ctx != 0) ? ctx->ready : 0U;
}

static inline uint16_t CurrentSenseOffset2_OffsetA(const CurrentSenseOffset2 *ctx)
{
    return (ctx != 0) ? ctx->offset_a : 0U;
}

static inline uint16_t CurrentSenseOffset2_OffsetB(const CurrentSenseOffset2 *ctx)
{
    return (ctx != 0) ? ctx->offset_b : 0U;
}

static inline float CurrentSense_RawToCurrentA(uint16_t raw,
                                              uint16_t offset_raw,
                                              float adc_max_counts,
                                              float vref_v,
                                              float shunt_ohm,
                                              float gain_v_per_v)
{
    if ((adc_max_counts <= 0.0f) || (vref_v <= 0.0f) || (shunt_ohm <= 0.0f) || (gain_v_per_v <= 0.0f))
    {
        return 0.0f;
    }

    const int32_t diff = (int32_t)raw - (int32_t)offset_raw;
    const float v = ((float)diff) * (vref_v / adc_max_counts);
    return v / (shunt_ohm * gain_v_per_v);
}

#endif /* COMPONENTS_CURRENT_SENSE_H */
