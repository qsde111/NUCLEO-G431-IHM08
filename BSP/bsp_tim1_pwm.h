#ifndef BSP_TIM1_PWM_H
#define BSP_TIM1_PWM_H

#include "tim.h"
#include <stdint.h>

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t period;
    uint8_t outputs_enabled;
} BspTim1Pwm;

void BspTim1Pwm_Init(BspTim1Pwm *ctx, TIM_HandleTypeDef *htim);
HAL_StatusTypeDef BspTim1Pwm_StartTrigger(BspTim1Pwm *ctx);

HAL_StatusTypeDef BspTim1Pwm_EnableOutputs(BspTim1Pwm *ctx);
HAL_StatusTypeDef BspTim1Pwm_DisableOutputs(BspTim1Pwm *ctx);

void BspTim1Pwm_SetDuty(BspTim1Pwm *ctx, float duty_a, float duty_b, float duty_c);
void BspTim1Pwm_SetNeutral(BspTim1Pwm *ctx);

#endif /* BSP_TIM1_PWM_H */

