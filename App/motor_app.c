#include "motor_app.h"

#include <string.h>

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
}

void MotorApp_Loop(MotorApp *ctx)
{
    if (ctx == 0)
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
