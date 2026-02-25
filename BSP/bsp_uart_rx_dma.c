#include "bsp_uart_rx_dma.h"

static uint16_t BspUartRxDma_DmaPos(const BspUartRxDma *ctx)
{
    if ((ctx == 0) || (ctx->huart == 0) || (ctx->huart->hdmarx == 0) || (ctx->rx_buf == 0) || (ctx->rx_buf_len == 0U))
    {
        return 0U;
    }

    const uint16_t remaining = (uint16_t)__HAL_DMA_GET_COUNTER(ctx->huart->hdmarx);

    uint16_t pos = (uint16_t)(ctx->rx_buf_len - remaining);
    if (pos >= ctx->rx_buf_len)
    {
        pos = 0U;
    }
    return pos;
}

void BspUartRxDma_Init(BspUartRxDma *ctx, UART_HandleTypeDef *huart, uint8_t *rx_buf, uint16_t rx_buf_len)
{
    if (ctx == 0)
    {
        return;
    }

    ctx->huart = huart;
    ctx->rx_buf = rx_buf;
    ctx->rx_buf_len = rx_buf_len;
    ctx->last_pos = 0U;
}

HAL_StatusTypeDef BspUartRxDma_Start(BspUartRxDma *ctx)
{
    if ((ctx == 0) || (ctx->huart == 0) || (ctx->huart->hdmarx == 0) || (ctx->rx_buf == 0) || (ctx->rx_buf_len == 0U))
    {
        return HAL_ERROR;
    }

    ctx->last_pos = 0U;
    const HAL_StatusTypeDef st = HAL_UART_Receive_DMA(ctx->huart, ctx->rx_buf, ctx->rx_buf_len);
    if (st != HAL_OK)
    {
        return st;
    }

    __HAL_DMA_DISABLE_IT(ctx->huart->hdmarx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(ctx->huart->hdmarx, DMA_IT_TC);

    return HAL_OK;
}

/**
 * @brief 轮询串口DMA接收状态，并回调处理新数据 (位于BSP层)
 * @param ctx       指向BSP层UART DMA句柄的指针
 * @param user      透传给回调函数的用户上下文指针 (实际传入算法层结构体)
 * @param on_bytes  当有新数据到来时触发的回调函数
 * @return uint16_t 本次处理的字节数
 */
uint16_t BspUartRxDma_Poll(BspUartRxDma *ctx, void *user, BspUartRxDma_OnBytes on_bytes)
{
    if ((ctx == 0) || (ctx->huart == 0) || (ctx->huart->hdmarx == 0) || (ctx->rx_buf == 0) || (ctx->rx_buf_len == 0U))
    {
        return 0U;
    }

    const uint16_t pos = BspUartRxDma_DmaPos(ctx);
    const uint16_t last = ctx->last_pos;

    if (pos == last)
    {
        return 0U;
    }

    uint16_t count = 0U;
    if (pos > last)
    {
        count = (uint16_t)(pos - last);
        if ((on_bytes != 0) && (count != 0U))
        {
            on_bytes(user, &ctx->rx_buf[last], count);
        }
    }
    else
    {
        const uint16_t n1 = (uint16_t)(ctx->rx_buf_len - last);
        count = (uint16_t)(n1 + pos);
        if ((on_bytes != 0) && (n1 != 0U))
        {
            on_bytes(user, &ctx->rx_buf[last], n1);
        }
        if ((on_bytes != 0) && (pos != 0U))
        {
            on_bytes(user, &ctx->rx_buf[0], pos);
        }
    }

    ctx->last_pos = pos;
    return count;
}
