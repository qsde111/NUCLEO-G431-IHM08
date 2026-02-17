#ifndef BSP_SPI3_FAST_H
#define BSP_SPI3_FAST_H

#include "main.h"
#include "spi.h"
#include <stdint.h>

typedef struct
{
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
} BspSpi3Fast;

void BspSpi3Fast_Init(BspSpi3Fast *ctx, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);
void BspSpi3Fast_CsLow(void *user);
void BspSpi3Fast_CsHigh(void *user);
uint8_t BspSpi3Fast_Transfer8(void *user, uint8_t tx_byte);

#endif /* BSP_SPI3_FAST_H */
