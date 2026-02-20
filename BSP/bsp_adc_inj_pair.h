#ifndef BSP_ADC_INJ_PAIR_H
#define BSP_ADC_INJ_PAIR_H

#include "adc.h"
#include "stm32g4xx_ll_adc.h"

#include <stdint.h>

typedef void (*BspAdcInjPair_OnPair)(void *user, uint16_t adc1, uint16_t adc2);

typedef struct
{
    ADC_HandleTypeDef *hadc1;
    ADC_HandleTypeDef *hadc2;

    uint32_t adc1_ch;
    uint32_t adc2_ch;

    volatile uint16_t last_adc1;
    volatile uint16_t last_adc2;
    volatile uint8_t have1;
    volatile uint8_t have2;

    void *user;
    BspAdcInjPair_OnPair on_pair;
} BspAdcInjPair;

void BspAdcInjPair_Init(BspAdcInjPair *ctx, ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2);
void BspAdcInjPair_RegisterCallback(BspAdcInjPair *ctx, void *user, BspAdcInjPair_OnPair on_pair);

HAL_StatusTypeDef BspAdcInjPair_Start(BspAdcInjPair *ctx);

void BspAdcInjPair_SetRank1Channels(BspAdcInjPair *ctx, uint32_t adc1_ch, uint32_t adc2_ch);
uint32_t BspAdcInjPair_Adc1Ch(const BspAdcInjPair *ctx);
uint32_t BspAdcInjPair_Adc2Ch(const BspAdcInjPair *ctx);

#endif /* BSP_ADC_INJ_PAIR_H */

