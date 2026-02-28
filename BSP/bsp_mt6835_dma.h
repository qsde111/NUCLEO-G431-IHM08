#ifndef BSP_MT6835_DMA_H
#define BSP_MT6835_DMA_H

#include "main.h"
#include "spi.h"

#include <stdint.h>

typedef struct
{
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;

    uint8_t tx[5];
    uint8_t rx[5];

    volatile uint8_t busy;
    volatile uint8_t have_raw21;
    volatile uint32_t raw21;
} BspMt6835Dma;

void BspMt6835Dma_Init(BspMt6835Dma *ctx, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);
uint8_t BspMt6835Dma_TryStart(BspMt6835Dma *ctx);
uint8_t BspMt6835Dma_PopRaw21(BspMt6835Dma *ctx, uint32_t *raw21_out);

#endif /* BSP_MT6835_DMA_H */

