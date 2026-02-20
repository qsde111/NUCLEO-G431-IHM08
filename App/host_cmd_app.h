#ifndef APP_HOST_CMD_APP_H
#define APP_HOST_CMD_APP_H

#include "bsp_uart_rx_dma.h"
#include "host_cmd_parser.h"

#include <stdint.h>

#ifndef HOST_CMD_APP_RX_DMA_BUF_LEN
#define HOST_CMD_APP_RX_DMA_BUF_LEN (256U)
#endif

#ifndef HOST_CMD_APP_IDLE_FLUSH_MS
#define HOST_CMD_APP_IDLE_FLUSH_MS (10U)
#endif

typedef struct
{
    BspUartRxDma rx;
    HostCmdParser parser;
    uint32_t last_rx_tick_ms;
    uint8_t rx_dma_buf[HOST_CMD_APP_RX_DMA_BUF_LEN];
} HostCmdApp;

void HostCmdApp_Init(HostCmdApp *ctx, UART_HandleTypeDef *huart);
HAL_StatusTypeDef HostCmdApp_Start(HostCmdApp *ctx);
void HostCmdApp_Loop(HostCmdApp *ctx);
uint8_t HostCmdApp_Pop(HostCmdApp *ctx, HostCmd *out);

#endif /* APP_HOST_CMD_APP_H */

