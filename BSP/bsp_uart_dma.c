#include "bsp_uart_dma.h"

void BspUartDma_Init(BspUartDma *ctx, UART_HandleTypeDef *huart)
{
    if (ctx == 0)
    {
        return;
    }
    ctx->huart = huart;
}

uint8_t BspUartDma_TxReady(const BspUartDma *ctx)
{
    if ((ctx == 0) || (ctx->huart == 0))
    {
        return 0U;
    }
    return (ctx->huart->gState == HAL_UART_STATE_READY) ? 1U : 0U;
}

HAL_StatusTypeDef BspUartDma_Send(BspUartDma *ctx, const uint8_t *data, uint16_t len)
{
    if ((ctx == 0) || (ctx->huart == 0) || (data == 0) || (len == 0U))
    {
        return HAL_ERROR;
    }
    if (!BspUartDma_TxReady(ctx))
    {
        return HAL_BUSY;
    }
    return HAL_UART_Transmit_DMA(ctx->huart, (uint8_t *)data, len);
}

