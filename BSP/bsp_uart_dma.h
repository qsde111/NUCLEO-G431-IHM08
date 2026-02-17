#ifndef BSP_UART_DMA_H
#define BSP_UART_DMA_H

#include "usart.h"
#include <stdint.h>

typedef struct
{
    UART_HandleTypeDef *huart;
} BspUartDma;

void BspUartDma_Init(BspUartDma *ctx, UART_HandleTypeDef *huart);
uint8_t BspUartDma_TxReady(const BspUartDma *ctx);
HAL_StatusTypeDef BspUartDma_Send(BspUartDma *ctx, const uint8_t *data, uint16_t len);

#endif /* BSP_UART_DMA_H */

