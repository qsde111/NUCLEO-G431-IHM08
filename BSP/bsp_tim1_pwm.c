#include "bsp_tim1_pwm.h"

static float BspTim1Pwm_Clamp01(float x)
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

void BspTim1Pwm_Init(BspTim1Pwm *ctx, TIM_HandleTypeDef *htim)
{
    if (ctx == 0)
    {
        return;
    }

    ctx->htim = htim;
    ctx->period = (htim != 0) ? (uint32_t)htim->Init.Period : 0U;
    ctx->outputs_enabled = 0U;
}

HAL_StatusTypeDef BspTim1Pwm_StartTrigger(BspTim1Pwm *ctx)
{
    if ((ctx == 0) || (ctx->htim == 0))
    {
        return HAL_ERROR;
    }

    /* Start timer base + CH4 (NoOutput) so TRGO2 can trigger ADC injected conversions */
    (void)HAL_TIM_Base_Start(ctx->htim);
    return HAL_TIM_PWM_Start(ctx->htim, TIM_CHANNEL_4);
}

HAL_StatusTypeDef BspTim1Pwm_EnableOutputs(BspTim1Pwm *ctx)
{
    if ((ctx == 0) || (ctx->htim == 0))
    {
        return HAL_ERROR;
    }

    /* Ensure safe initial duty */
    BspTim1Pwm_SetNeutral(ctx);

    if (HAL_TIM_PWM_Start(ctx->htim, TIM_CHANNEL_1) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_TIM_PWM_Start(ctx->htim, TIM_CHANNEL_2) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_TIM_PWM_Start(ctx->htim, TIM_CHANNEL_3) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (HAL_TIMEx_PWMN_Start(ctx->htim, TIM_CHANNEL_1) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_TIMEx_PWMN_Start(ctx->htim, TIM_CHANNEL_2) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_TIMEx_PWMN_Start(ctx->htim, TIM_CHANNEL_3) != HAL_OK)
    {
        return HAL_ERROR;
    }

    ctx->outputs_enabled = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef BspTim1Pwm_DisableOutputs(BspTim1Pwm *ctx)
{
    if ((ctx == 0) || (ctx->htim == 0))
    {
        return HAL_ERROR;
    }

    (void)HAL_TIMEx_PWMN_Stop(ctx->htim, TIM_CHANNEL_1);
    (void)HAL_TIMEx_PWMN_Stop(ctx->htim, TIM_CHANNEL_2);
    (void)HAL_TIMEx_PWMN_Stop(ctx->htim, TIM_CHANNEL_3);

    (void)HAL_TIM_PWM_Stop(ctx->htim, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(ctx->htim, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Stop(ctx->htim, TIM_CHANNEL_3);

    ctx->outputs_enabled = 0U;
    return HAL_OK;
}

void BspTim1Pwm_SetDuty(BspTim1Pwm *ctx, float duty_a, float duty_b, float duty_c)
{
    if ((ctx == 0) || (ctx->htim == 0) || (ctx->period == 0U))
    {
        return;
    }

    const float da = BspTim1Pwm_Clamp01(duty_a);
    const float db = BspTim1Pwm_Clamp01(duty_b);
    const float dc = BspTim1Pwm_Clamp01(duty_c);

    const uint32_t ccr1 = (uint32_t)(da * (float)ctx->period);
    const uint32_t ccr2 = (uint32_t)(db * (float)ctx->period);
    const uint32_t ccr3 = (uint32_t)(dc * (float)ctx->period);

    __HAL_TIM_SET_COMPARE(ctx->htim, TIM_CHANNEL_1, ccr1);
    __HAL_TIM_SET_COMPARE(ctx->htim, TIM_CHANNEL_2, ccr2);
    __HAL_TIM_SET_COMPARE(ctx->htim, TIM_CHANNEL_3, ccr3);
}

void BspTim1Pwm_SetNeutral(BspTim1Pwm *ctx)
{
    BspTim1Pwm_SetDuty(ctx, 0.5f, 0.5f, 0.5f);
}

