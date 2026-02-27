#include "host_cmd_app.h"

#include "main.h"

/* Host 命令接收链路：UART RX DMA circular -> Poll 取增量字节 -> Feed 解析器 -> Pop 给上层。 */

static void HostCmdApp_OnBytes(void *user, const uint8_t *data, uint16_t len)
{
    HostCmdParser *parser = (HostCmdParser *)user;
    HostCmdParser_Feed(parser, data, len);
}

void HostCmdApp_Init(HostCmdApp *ctx, UART_HandleTypeDef *huart)
{
    if (ctx == 0)
    {
        return;
    }

    HostCmdParser_Init(&ctx->parser);
    ctx->last_rx_tick_ms = HAL_GetTick();

    // rx_dma_buf是HostCmdApp *ctx下的数组uint8_t rx_dma_buf[HOST_CMD_APP_RX_DMA_BUF_LEN];,调用函数传参ctx->rx_dma_buf等价于写
    // &(ctx->rx_dma_buf[0])。所以它就是传入首位地址
    BspUartRxDma_Init(&ctx->rx, huart, ctx->rx_dma_buf, (uint16_t)sizeof(ctx->rx_dma_buf));
}

HAL_StatusTypeDef HostCmdApp_Start(HostCmdApp *ctx)
{
    if (ctx == 0)
    {
        return HAL_ERROR;
    }

    ctx->last_rx_tick_ms = HAL_GetTick();
    return BspUartRxDma_Start(&ctx->rx);
}

void HostCmdApp_Loop(HostCmdApp *ctx)
{
    if (ctx == 0)
    {
        return;
    }

    const uint16_t got = BspUartRxDma_Poll(&ctx->rx, &ctx->parser, HostCmdApp_OnBytes);
    if (got != 0U)
    {
        ctx->last_rx_tick_ms = HAL_GetTick();
    }
    else if (HostCmdParser_HasPartialLine(&ctx->parser) != 0U)
    {
        const uint32_t now = HAL_GetTick();
        if ((now - ctx->last_rx_tick_ms) > (uint32_t)HOST_CMD_APP_IDLE_FLUSH_MS)
        {
            HostCmdParser_FlushLine(&ctx->parser);
        }
    }
}

/* 将ctx->queue中存解析好的指令搬运到外部传入的HostCmd *out中 */
uint8_t HostCmdApp_Pop(HostCmdApp *ctx, HostCmd *out)
{
    if (ctx == 0)
    {
        return 0U;
    }
    return HostCmdParser_Pop(&ctx->parser, out);
}
