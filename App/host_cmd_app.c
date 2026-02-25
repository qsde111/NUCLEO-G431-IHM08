#include "host_cmd_app.h"

#include <string.h>

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

    (void)memset(&ctx->parser, 0, sizeof(ctx->parser));
    BspUartRxDma_Init(&ctx->rx, huart, ctx->rx_dma_buf, (uint16_t)sizeof(ctx->rx_dma_buf));
}

HAL_StatusTypeDef HostCmdApp_Start(HostCmdApp *ctx)
{
    if (ctx == 0)
    {
        return HAL_ERROR;
    }

    return BspUartRxDma_Start(&ctx->rx);
}

void HostCmdApp_Loop(HostCmdApp *ctx)
{
    if (ctx == 0)
    {
        return;
    }

    (void)BspUartRxDma_Poll(&ctx->rx, &ctx->parser, HostCmdApp_OnBytes);
}

uint8_t HostCmdApp_Pop(HostCmdApp *ctx, HostCmd *out)
{
    if (ctx == 0)
    {
        return 0U;
    }
    return HostCmdParser_Pop(&ctx->parser, out);
}
