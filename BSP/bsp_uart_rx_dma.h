#ifndef BSP_UART_RX_DMA_H
#define BSP_UART_RX_DMA_H

#include "usart.h"
#include <stdint.h>

typedef void (*BspUartRxDma_OnBytes)(void *user, const uint8_t *data, uint16_t len);

typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t *rx_buf;
    uint16_t rx_buf_len;
    uint16_t last_pos;
} BspUartRxDma;

void BspUartRxDma_Init(BspUartRxDma *ctx, UART_HandleTypeDef *huart, uint8_t *rx_buf, uint16_t rx_buf_len);
HAL_StatusTypeDef BspUartRxDma_Start(BspUartRxDma *ctx);
uint16_t BspUartRxDma_Poll(BspUartRxDma *ctx, void *user, BspUartRxDma_OnBytes on_bytes);

#endif /* BSP_UART_RX_DMA_H */

