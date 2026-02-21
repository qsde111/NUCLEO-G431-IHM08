#include "bsp_adc_inj_pair.h"

static BspAdcInjPair *g_ctx = 0;

void BspAdcInjPair_Init(BspAdcInjPair *ctx, ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2)
{
    if (ctx == 0)
    {
        return;
    }

    ctx->hadc1 = hadc1;
    ctx->hadc2 = hadc2;
    ctx->adc1_ch = LL_ADC_CHANNEL_1;
    ctx->adc2_ch = LL_ADC_CHANNEL_7;
    ctx->last_adc1 = 0U;
    ctx->last_adc2 = 0U;
    ctx->have1 = 0U;
    ctx->have2 = 0U;
    ctx->user = 0;
    ctx->on_pair = 0;

    g_ctx = ctx;

    if ((hadc1 != 0) && (hadc1->Instance != 0))
    {
        LL_ADC_SetChannelSamplingTime(hadc1->Instance, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_12CYCLES_5);
        LL_ADC_SetChannelSamplingTime(hadc1->Instance, LL_ADC_CHANNEL_6, LL_ADC_SAMPLINGTIME_12CYCLES_5);
        LL_ADC_SetChannelSamplingTime(hadc1->Instance, LL_ADC_CHANNEL_7, LL_ADC_SAMPLINGTIME_12CYCLES_5);
    }
    if ((hadc2 != 0) && (hadc2->Instance != 0))
    {
        LL_ADC_SetChannelSamplingTime(hadc2->Instance, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_12CYCLES_5);
        LL_ADC_SetChannelSamplingTime(hadc2->Instance, LL_ADC_CHANNEL_6, LL_ADC_SAMPLINGTIME_12CYCLES_5);
        LL_ADC_SetChannelSamplingTime(hadc2->Instance, LL_ADC_CHANNEL_7, LL_ADC_SAMPLINGTIME_12CYCLES_5);
    }
}

void BspAdcInjPair_RegisterCallback(BspAdcInjPair *ctx, void *user, BspAdcInjPair_OnPair on_pair)
{
    if (ctx == 0)
    {
        return;
    }
    ctx->user = user;
    ctx->on_pair = on_pair;
}

HAL_StatusTypeDef BspAdcInjPair_Start(BspAdcInjPair *ctx)
{
    if ((ctx == 0) || (ctx->hadc1 == 0) || (ctx->hadc2 == 0))
    {
        return HAL_ERROR;
    }

    (void)HAL_ADCEx_Calibration_Start(ctx->hadc1, ADC_SINGLE_ENDED);
    (void)HAL_ADCEx_Calibration_Start(ctx->hadc2, ADC_SINGLE_ENDED);

    // 双重主从模式下，建议先启动从机(ADC2)，再启动主机(ADC1)
    if (HAL_ADCEx_InjectedStart(ctx->hadc2) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_ADCEx_InjectedStart_IT(ctx->hadc1) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

void BspAdcInjPair_SetRank1Channels(BspAdcInjPair *ctx, uint32_t adc1_ch, uint32_t adc2_ch)
{
    if ((ctx == 0) || (ctx->hadc1 == 0) || (ctx->hadc2 == 0) || (ctx->hadc1->Instance == 0) || (ctx->hadc2->Instance == 0))
    {
        return;
    }

    ctx->adc1_ch = adc1_ch;
    ctx->adc2_ch = adc2_ch;

    LL_ADC_INJ_SetSequencerRanks(ctx->hadc1->Instance, LL_ADC_INJ_RANK_1, adc1_ch);
    LL_ADC_INJ_SetSequencerRanks(ctx->hadc2->Instance, LL_ADC_INJ_RANK_1, adc2_ch);
}

uint32_t BspAdcInjPair_Adc1Ch(const BspAdcInjPair *ctx)
{
    return (ctx != 0) ? ctx->adc1_ch : 0U;
}

uint32_t BspAdcInjPair_Adc2Ch(const BspAdcInjPair *ctx)
{
    return (ctx != 0) ? ctx->adc2_ch : 0U;
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if ((g_ctx == 0) || (hadc == 0))
    {
        return;
    }

    /*
     * ADC1/ADC2 injected conversions might run in dual mode where only the master
     * triggers callback handling logic (common pattern: handle only ADC1).
     * Therefore, fire the pair callback on ADC1 completion and read ADC2 value directly.
     */
    if (hadc != g_ctx->hadc1)
    {
        return;
    }

    g_ctx->last_adc1 = (uint16_t)HAL_ADCEx_InjectedGetValue(g_ctx->hadc1, ADC_INJECTED_RANK_1);
    g_ctx->last_adc2 = (uint16_t)HAL_ADCEx_InjectedGetValue(g_ctx->hadc2, ADC_INJECTED_RANK_1);

    if (g_ctx->on_pair != 0)
    {
        g_ctx->on_pair(g_ctx->user, g_ctx->last_adc1, g_ctx->last_adc2);
    }
}
