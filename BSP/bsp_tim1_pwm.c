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
    const HAL_StatusTypeDef st = HAL_TIM_PWM_Start(ctx->htim, TIM_CHANNEL_4);

    /* HAL_TIM_PWM_Start() enables MOE on TIM1; keep outputs gated until explicitly enabled */
    (void)BspTim1Pwm_ArmIdleOutputs(ctx);
    return st;
}

HAL_StatusTypeDef BspTim1Pwm_ArmIdleOutputs(BspTim1Pwm *ctx)
{
    if ((ctx == 0) || (ctx->htim == 0))
    {
        return HAL_ERROR;
    }

    /* Force outputs into configured idle state (CHx=RESET, CHxN=SET) */
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(ctx->htim);

    /* Enable CH1-3 and CH1N-3N so pins are actively driven (not Hi-Z) */
    ctx->htim->Instance->CCER |=
        (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE);

    /* Keep a known compare value; does not affect output while MOE=0 */
    BspTim1Pwm_SetNeutral(ctx);

    ctx->outputs_enabled = 0U;
    return HAL_OK;
}

HAL_StatusTypeDef BspTim1Pwm_EnableOutputs(BspTim1Pwm *ctx)
{
    if ((ctx == 0) || (ctx->htim == 0))
    {
        return HAL_ERROR;
    }

    if (ctx->outputs_enabled != 0U)
    {
        return HAL_OK;
    }

    BspTim1Pwm_SetNeutral(ctx);

    /* Make sure outputs are armed and in idle state before enabling MOE */
    if (BspTim1Pwm_ArmIdleOutputs(ctx) != HAL_OK)
    {
        return HAL_ERROR;
    }

    __HAL_TIM_MOE_ENABLE(ctx->htim);

    ctx->outputs_enabled = 1U;
    return HAL_OK;
}

/* 关闭MOE使六路PWM进入IDLE状态，置位CCR于50%中性点位置准备下一次启动 */
HAL_StatusTypeDef BspTim1Pwm_DisableOutputs(BspTim1Pwm *ctx)
{
    if ((ctx == 0) || (ctx->htim == 0))
    {
        return HAL_ERROR;
    }

    /* Do NOT stop channels: keep CCxE/CCxNE enabled so pins stay driven to idle state.
     * Just gate outputs via MOE (OSSI/OSSR must be enabled in TIM1 config). */
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(ctx->htim);
    BspTim1Pwm_SetNeutral(ctx);

    ctx->outputs_enabled = 0U;
    return HAL_OK;
}

/* 通过三相占空比修改三相CCR值并输出 */
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

/* 50% 占空比的物理意义：零矢量，任意两相线压差为0；自举电容充电 */
void BspTim1Pwm_SetNeutral(BspTim1Pwm *ctx)
{
    BspTim1Pwm_SetDuty(ctx, 0.5f, 0.5f, 0.5f);
}
