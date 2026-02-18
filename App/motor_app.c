#include "motor_app.h"

#include "tim.h"

#include <string.h>

#define MOTORAPP_POLE_CALIB_CCR (80u)

static void MotorApp_PwmAllOff(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);

    (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

    __HAL_TIM_MOE_DISABLE(&htim1);
    __HAL_TIM_DISABLE(&htim1);
}

static void MotorApp_PwmPoleCalibStart(uint32_t ccr_u)
{
    const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
    if (ccr_u > arr)
    {
        ccr_u = arr;
    }

    MotorApp_PwmAllOff();
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr_u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    (void)HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    __HAL_TIM_MOE_ENABLE(&htim1);
}

void MotorApp_Init(MotorApp *ctx, UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    if (ctx == 0)
    {
        return;
    }

    memset(ctx, 0, sizeof(*ctx));

    BspUartDma_Init(&ctx->uart, huart);
    BspSpi3Fast_Init(&ctx->spi, hspi, cs_port, cs_pin);

    Mt6835BusOps bus = {
        .user = &ctx->spi,
        .transfer8 = BspSpi3Fast_Transfer8,
        .cs_low = BspSpi3Fast_CsLow,
        .cs_high = BspSpi3Fast_CsHigh,
    };
    Mt6835_Init(&ctx->encoder, &bus);

    ctx->last_stream_tick_ms = HAL_GetTick();
    ctx->pole_calib_enabled = 0;
    ctx->pole_calib_pwm_running = 0;

    MotorApp_PwmAllOff();
}

void MotorApp_SetPoleCalibEnabled(MotorApp *ctx, uint8_t enabled)
{
    if (ctx == 0)
    {
        return;
    }

    enabled = (enabled != 0u) ? 1u : 0u;
    if (enabled == ctx->pole_calib_enabled)
    {
        return;
    }
    ctx->pole_calib_enabled = enabled;

    if (enabled)
    {
        MotorApp_PwmPoleCalibStart(MOTORAPP_POLE_CALIB_CCR);
        ctx->pole_calib_pwm_running = 1;
        ctx->last_stream_tick_ms = (uint32_t)(HAL_GetTick() - 1u);
    }
    else
    {
        MotorApp_PwmAllOff();
        ctx->pole_calib_pwm_running = 0;
        ctx->last_stream_tick_ms = HAL_GetTick();
    }
}

void MotorApp_Loop(MotorApp *ctx)
{
    if (ctx == 0)
    {
        return;
    }
    if (ctx->pole_calib_enabled == 0u)
    {
        return;
    }

    const uint32_t now = HAL_GetTick();
    if (now == ctx->last_stream_tick_ms)
    {
        return;
    }
    ctx->last_stream_tick_ms = now;

    ctx->raw21 = Mt6835_ReadRaw21(&ctx->encoder);
    ctx->pos_mech_rad = Mt6835_Raw21ToRad(ctx->raw21);

    JustFloat_Pack4((float)ctx->raw21, ctx->pos_mech_rad, 0.0f, 0.0f, ctx->tx_frame);
    (void)BspUartDma_Send(&ctx->uart, ctx->tx_frame, (uint16_t)sizeof(ctx->tx_frame));
}
