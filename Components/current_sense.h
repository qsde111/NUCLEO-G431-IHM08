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

/* 将双路ADC采集值放到求和累加器中，计数完毕后取均值放到ctx->offset_a、offset_b中 */
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

/* 返回a路采样offset电流值 */
static inline uint16_t CurrentSenseOffset2_OffsetA(const CurrentSenseOffset2 *ctx)
{
    return (ctx != 0) ? ctx->offset_a : 0U;
}

/* 返回b路采样offset电流值 */
static inline uint16_t CurrentSenseOffset2_OffsetB(const CurrentSenseOffset2 *ctx)
{
    return (ctx != 0) ? ctx->offset_b : 0U;
}

static inline float CurrentSense_RawToCurrentA(uint16_t raw, uint16_t offset_raw, float adc_max_counts, float vref_v,
                                               float shunt_ohm, float gain_v_per_v)
{
    if ((adc_max_counts <= 0.0f) || (vref_v <= 0.0f) || (shunt_ohm <= 0.0f) || (gain_v_per_v <= 0.0f))
    {
        return 0.0f;
    }

    const int32_t diff = (int32_t)raw - (int32_t)offset_raw;
    const float v = ((float)diff) * (vref_v / adc_max_counts);
    return v / (shunt_ohm * gain_v_per_v);
}

typedef enum
{
    CURRENT_SENSE_PHASE_A = 0,
    CURRENT_SENSE_PHASE_B = 1,
    CURRENT_SENSE_PHASE_C = 2,
    CURRENT_SENSE_PHASE_COUNT = 3
} CurrentSensePhase;

typedef enum
{
    CURRENT_SENSE_PAIR_AB = 0,
    CURRENT_SENSE_PAIR_AC = 1,
    CURRENT_SENSE_PAIR_BC = 2,
    CURRENT_SENSE_PAIR_COUNT = 3
} CurrentSensePair;

enum
{
    CURRENT_SENSE_PHASE_MASK_A = (1U << CURRENT_SENSE_PHASE_A),
    CURRENT_SENSE_PHASE_MASK_B = (1U << CURRENT_SENSE_PHASE_B),
    CURRENT_SENSE_PHASE_MASK_C = (1U << CURRENT_SENSE_PHASE_C)
};

typedef struct
{
    CurrentSensePair pair;
    uint16_t low_window_a_ticks;
    uint16_t low_window_b_ticks;
    uint16_t low_window_c_ticks;
    uint8_t valid_mask;
    uint8_t valid_count;
    uint8_t pair_valid;
} CurrentSense3ShuntDecision;

static inline float CurrentSense_Clamp01(float x)
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

static inline uint16_t CurrentSense_LowWindowTicks(float duty, uint32_t pwm_period_ticks)
{
    const float low_ticks = (1.0f - CurrentSense_Clamp01(duty)) * (float)pwm_period_ticks;
    if (low_ticks <= 0.0f)
    {
        return 0U;
    }
    if (low_ticks >= (float)pwm_period_ticks)
    {
        return (uint16_t)pwm_period_ticks;
    }
    return (uint16_t)(low_ticks + 0.5f);
}

static inline uint8_t CurrentSense_CountValidBits3(uint8_t valid_mask)
{
    uint8_t count = 0U;
    if ((valid_mask & CURRENT_SENSE_PHASE_MASK_A) != 0U)
    {
        count++;
    }
    if ((valid_mask & CURRENT_SENSE_PHASE_MASK_B) != 0U)
    {
        count++;
    }
    if ((valid_mask & CURRENT_SENSE_PHASE_MASK_C) != 0U)
    {
        count++;
    }
    return count;
}

static inline CurrentSensePhase CurrentSense_PairPhase1(CurrentSensePair pair)
{
    switch (pair)
    {
    case CURRENT_SENSE_PAIR_AB:
    case CURRENT_SENSE_PAIR_AC:
        return CURRENT_SENSE_PHASE_A;
    case CURRENT_SENSE_PAIR_BC:
        return CURRENT_SENSE_PHASE_B;
    default:
        return CURRENT_SENSE_PHASE_A;
    }
}

static inline CurrentSensePhase CurrentSense_PairPhase2(CurrentSensePair pair)
{
    switch (pair)
    {
    case CURRENT_SENSE_PAIR_AB:
        return CURRENT_SENSE_PHASE_B;
    case CURRENT_SENSE_PAIR_AC:
        return CURRENT_SENSE_PHASE_C;
    case CURRENT_SENSE_PAIR_BC:
        return CURRENT_SENSE_PHASE_C;
    default:
        return CURRENT_SENSE_PHASE_B;
    }
}

static inline uint8_t CurrentSense_PhaseMask(CurrentSensePhase phase)
{
    if ((uint32_t)phase >= (uint32_t)CURRENT_SENSE_PHASE_COUNT)
    {
        return 0U;
    }
    return (uint8_t)(1U << (uint32_t)phase);
}

static inline void CurrentSense3Shunt_SelectPair(float duty_a, float duty_b, float duty_c, uint32_t pwm_period_ticks,
                                                 uint16_t min_window_ticks, CurrentSensePair preferred_pair,
                                                 CurrentSense3ShuntDecision *out)
{
    static const CurrentSensePair pair_order[CURRENT_SENSE_PAIR_COUNT] = {
        CURRENT_SENSE_PAIR_AB,
        CURRENT_SENSE_PAIR_AC,
        CURRENT_SENSE_PAIR_BC,
    };

    if (out == 0)
    {
        return;
    }

    const uint16_t windows[CURRENT_SENSE_PHASE_COUNT] = {
        CurrentSense_LowWindowTicks(duty_a, pwm_period_ticks),
        CurrentSense_LowWindowTicks(duty_b, pwm_period_ticks),
        CurrentSense_LowWindowTicks(duty_c, pwm_period_ticks),
    };

    uint8_t valid_mask = 0U;
    if (windows[CURRENT_SENSE_PHASE_A] >= min_window_ticks)
    {
        valid_mask |= CURRENT_SENSE_PHASE_MASK_A;
    }
    if (windows[CURRENT_SENSE_PHASE_B] >= min_window_ticks)
    {
        valid_mask |= CURRENT_SENSE_PHASE_MASK_B;
    }
    if (windows[CURRENT_SENSE_PHASE_C] >= min_window_ticks)
    {
        valid_mask |= CURRENT_SENSE_PHASE_MASK_C;
    }

    const uint8_t valid_count = CurrentSense_CountValidBits3(valid_mask);

    CurrentSensePair best_pair = preferred_pair;
    uint8_t best_pair_valid = 0U;
    uint16_t best_primary = 0U;
    uint16_t best_secondary = 0U;
    uint8_t have_best = 0U;

    for (uint32_t i = 0U; i < (uint32_t)CURRENT_SENSE_PAIR_COUNT; ++i)
    {
        const CurrentSensePair pair = pair_order[i];
        const CurrentSensePhase p1 = CurrentSense_PairPhase1(pair);
        const CurrentSensePhase p2 = CurrentSense_PairPhase2(pair);
        const uint16_t w1 = windows[p1];
        const uint16_t w2 = windows[p2];
        const uint8_t pair_valid = (uint8_t)(((CurrentSense_PhaseMask(p1) & valid_mask) != 0U) &&
                                             ((CurrentSense_PhaseMask(p2) & valid_mask) != 0U));

        if ((valid_count >= 2U) && (pair_valid == 0U))
        {
            continue;
        }

        const uint16_t primary = (w1 < w2) ? w1 : w2;
        const uint16_t secondary = (uint16_t)(w1 + w2);
        const uint8_t better = (uint8_t)((have_best == 0U) || (primary > best_primary) ||
                                         ((primary == best_primary) && (secondary > best_secondary)) ||
                                         ((primary == best_primary) && (secondary == best_secondary) &&
                                          (pair == preferred_pair) && (best_pair != preferred_pair)));

        if (better != 0U)
        {
            have_best = 1U;
            best_pair = pair;
            best_pair_valid = pair_valid;
            best_primary = primary;
            best_secondary = secondary;
        }
    }

    out->pair = best_pair;
    out->low_window_a_ticks = windows[CURRENT_SENSE_PHASE_A];
    out->low_window_b_ticks = windows[CURRENT_SENSE_PHASE_B];
    out->low_window_c_ticks = windows[CURRENT_SENSE_PHASE_C];
    out->valid_mask = valid_mask;
    out->valid_count = valid_count;
    out->pair_valid = best_pair_valid;
}

static inline float CurrentSense_RawToSignedCurrentA(uint16_t raw, uint16_t offset_raw, float adc_max_counts, float vref_v,
                                                     float shunt_ohm, float gain_v_per_v, float sign)
{
    return sign * CurrentSense_RawToCurrentA(raw, offset_raw, adc_max_counts, vref_v, shunt_ohm, gain_v_per_v);
}

static inline void CurrentSense3Shunt_Reconstruct(CurrentSensePair pair, uint16_t adc1_raw, uint16_t adc2_raw,
                                                  const uint16_t offset_raw[CURRENT_SENSE_PHASE_COUNT],
                                                  float adc_max_counts, float vref_v, float shunt_ohm, float gain_v_per_v,
                                                  float sign, float *ia_a, float *ib_a, float *ic_a)
{
    if ((offset_raw == 0) || (ia_a == 0) || (ib_a == 0) || (ic_a == 0))
    {
        return;
    }

    switch (pair)
    {
    case CURRENT_SENSE_PAIR_AB:
        *ia_a = CurrentSense_RawToSignedCurrentA(adc1_raw, offset_raw[CURRENT_SENSE_PHASE_A], adc_max_counts, vref_v,
                                                 shunt_ohm, gain_v_per_v, sign);
        *ib_a = CurrentSense_RawToSignedCurrentA(adc2_raw, offset_raw[CURRENT_SENSE_PHASE_B], adc_max_counts, vref_v,
                                                 shunt_ohm, gain_v_per_v, sign);
        *ic_a = -(*ia_a + *ib_a);
        break;
    case CURRENT_SENSE_PAIR_AC:
        *ia_a = CurrentSense_RawToSignedCurrentA(adc1_raw, offset_raw[CURRENT_SENSE_PHASE_A], adc_max_counts, vref_v,
                                                 shunt_ohm, gain_v_per_v, sign);
        *ic_a = CurrentSense_RawToSignedCurrentA(adc2_raw, offset_raw[CURRENT_SENSE_PHASE_C], adc_max_counts, vref_v,
                                                 shunt_ohm, gain_v_per_v, sign);
        *ib_a = -(*ia_a + *ic_a);
        break;
    case CURRENT_SENSE_PAIR_BC:
        *ib_a = CurrentSense_RawToSignedCurrentA(adc1_raw, offset_raw[CURRENT_SENSE_PHASE_B], adc_max_counts, vref_v,
                                                 shunt_ohm, gain_v_per_v, sign);
        *ic_a = CurrentSense_RawToSignedCurrentA(adc2_raw, offset_raw[CURRENT_SENSE_PHASE_C], adc_max_counts, vref_v,
                                                 shunt_ohm, gain_v_per_v, sign);
        *ia_a = -(*ib_a + *ic_a);
        break;
    default:
        *ia_a = 0.0f;
        *ib_a = 0.0f;
        *ic_a = 0.0f;
        break;
    }
}

#endif /* COMPONENTS_CURRENT_SENSE_H */
