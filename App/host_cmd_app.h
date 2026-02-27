#ifndef APP_HOST_CMD_APP_H
#define APP_HOST_CMD_APP_H

/**
 * @file host_cmd_app.h
 * @brief Host 命令接收应用层（UART RX DMA circular + HostCmdParser）。
 *
 * 数据流转：
 * 1) BSP 层 `BspUartRxDma_Poll()` 从 DMA 环形缓冲区取“新增字节片段”；
 * 2) 回调把新增字节喂给 `HostCmdParser_Feed()`；
 * 3) 主循环使用 `HostCmdApp_Pop()` 取出已解析的命令。
 *
 * 结束符：命令通常以 `\r` / `\n` / `;` 结束。
 *
 * 可选增强：若上位机不发送结束符，上层可用“idle flush”策略：
 * - 连续一段时间（默认 10ms）没有新字节，则把当前缓冲区 flush 当作一条命令。
 */

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

/**
 * @brief 初始化 HostCmdApp。
 * @param ctx  HostCmdApp 上下文。
 * @param huart 目标 UART（使用其 RX DMA）。
 */
void HostCmdApp_Init(HostCmdApp *ctx, UART_HandleTypeDef *huart);

/**
 * @brief 启动 UART RX DMA circular 接收。
 * @param ctx HostCmdApp 上下文。
 * @return HAL 状态码。
 */
HAL_StatusTypeDef HostCmdApp_Start(HostCmdApp *ctx);

/**
 * @brief 轮询 DMA 新增数据并喂给解析器。
 * @param ctx HostCmdApp 上下文。
 */
void HostCmdApp_Loop(HostCmdApp *ctx);

uint8_t HostCmdApp_Pop(HostCmdApp *ctx, HostCmd *out);

#endif /* APP_HOST_CMD_APP_H */
