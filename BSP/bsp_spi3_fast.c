#include "bsp_spi3_fast.h"

void BspSpi3Fast_Init(BspSpi3Fast *ctx, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    if (ctx == 0)
    {
        return;
    }

    ctx->hspi = hspi;
    ctx->cs_port = cs_port;
    ctx->cs_pin = cs_pin;
    BspSpi3Fast_CsHigh(ctx);
}

void BspSpi3Fast_CsLow(void *user)
{
    BspSpi3Fast *ctx = (BspSpi3Fast *)user;
    if ((ctx == 0) || (ctx->cs_port == 0))
    {
        return;
    }
    HAL_GPIO_WritePin(ctx->cs_port, ctx->cs_pin, GPIO_PIN_RESET);
}

void BspSpi3Fast_CsHigh(void *user)
{
    BspSpi3Fast *ctx = (BspSpi3Fast *)user;
    if ((ctx == 0) || (ctx->cs_port == 0))
    {
        return;
    }
    HAL_GPIO_WritePin(ctx->cs_port, ctx->cs_pin, GPIO_PIN_SET);
}

uint8_t BspSpi3Fast_Transfer8(void *user, uint8_t tx_byte)
{
    BspSpi3Fast *ctx = (BspSpi3Fast *)user;
    if ((ctx == 0) || (ctx->hspi == 0) || (ctx->hspi->Instance == 0))
    {
        return 0U;
    }

    SPI_TypeDef *spi = ctx->hspi->Instance;
    if ((spi->CR1 & SPI_CR1_SPE) == 0U)
    {
        __HAL_SPI_ENABLE(ctx->hspi);
    }

    while ((spi->SR & SPI_SR_TXE) == 0U)
    {
    }
    *(__IO uint8_t *)&spi->DR = tx_byte;
    while ((spi->SR & SPI_SR_RXNE) == 0U)
    {
    }
    const uint8_t rx = *(__IO uint8_t *)&spi->DR;

    while ((spi->SR & SPI_SR_BSY) != 0U)
    {
    }

    return rx;
}
