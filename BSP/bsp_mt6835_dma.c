#include "bsp_mt6835_dma.h"

#define MT6835_STREAM_CMD_H (0xA0U)
#define MT6835_STREAM_CMD_L (0x03U)

static BspMt6835Dma *g_ctx = 0;

void BspMt6835Dma_Init(BspMt6835Dma *ctx, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    if (ctx == 0)
    {
        return;
    }

    ctx->hspi = hspi;
    ctx->cs_port = cs_port;
    ctx->cs_pin = cs_pin;
    ctx->busy = 0U;
    ctx->have_raw21 = 0U;
    ctx->raw21 = 0U;

    ctx->tx[0] = MT6835_STREAM_CMD_H;
    ctx->tx[1] = MT6835_STREAM_CMD_L;
    ctx->tx[2] = 0x00U;
    ctx->tx[3] = 0x00U;
    ctx->tx[4] = 0x00U;

    if (ctx->cs_port != 0)
    {
        HAL_GPIO_WritePin(ctx->cs_port, ctx->cs_pin, GPIO_PIN_SET);
    }

    g_ctx = ctx;
}

uint8_t BspMt6835Dma_TryStart(BspMt6835Dma *ctx)
{
    if ((ctx == 0) || (ctx->hspi == 0) || (ctx->hspi->Instance == 0) || (ctx->cs_port == 0))
    {
        return 0U;
    }
    if (ctx->busy != 0U)
    {
        return 0U;
    }

    ctx->busy = 1U;

    HAL_GPIO_WritePin(ctx->cs_port, ctx->cs_pin, GPIO_PIN_RESET);
    if (HAL_SPI_TransmitReceive_DMA(ctx->hspi, ctx->tx, ctx->rx, (uint16_t)sizeof(ctx->tx)) != HAL_OK)
    {
        HAL_GPIO_WritePin(ctx->cs_port, ctx->cs_pin, GPIO_PIN_SET);
        ctx->busy = 0U;
        return 0U;
    }

    return 1U;
}

uint8_t BspMt6835Dma_PopRaw21(BspMt6835Dma *ctx, uint32_t *raw21_out)
{
    if ((ctx == 0) || (raw21_out == 0))
    {
        return 0U;
    }
    if (ctx->have_raw21 == 0U)
    {
        return 0U;
    }

    *raw21_out = ctx->raw21;
    ctx->have_raw21 = 0U;
    return 1U;
}

static void BspMt6835Dma_OnDone(BspMt6835Dma *ctx)
{
    if ((ctx == 0) || (ctx->cs_port == 0))
    {
        return;
    }

    HAL_GPIO_WritePin(ctx->cs_port, ctx->cs_pin, GPIO_PIN_SET);

    const uint8_t b20_13 = ctx->rx[2];
    const uint8_t b12_5 = ctx->rx[3];
    const uint8_t b4_0_status = ctx->rx[4];
    ctx->raw21 = (((uint32_t)b20_13 << 13) | ((uint32_t)b12_5 << 5) | ((uint32_t)b4_0_status >> 3)) & 0x1FFFFFU;
    ctx->have_raw21 = 1U;
    ctx->busy = 0U;
}

static void BspMt6835Dma_OnError(BspMt6835Dma *ctx)
{
    if ((ctx == 0) || (ctx->cs_port == 0))
    {
        return;
    }

    HAL_GPIO_WritePin(ctx->cs_port, ctx->cs_pin, GPIO_PIN_SET);
    ctx->busy = 0U;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if ((g_ctx == 0) || (hspi == 0))
    {
        return;
    }
    if (hspi != g_ctx->hspi)
    {
        return;
    }
    BspMt6835Dma_OnDone(g_ctx);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if ((g_ctx == 0) || (hspi == 0))
    {
        return;
    }
    if (hspi != g_ctx->hspi)
    {
        return;
    }
    BspMt6835Dma_OnError(g_ctx);
}
