#ifndef APP_MOTOR_APP_H
#define APP_MOTOR_APP_H

#include "bsp_spi3_fast.h"
#include "bsp_uart_dma.h"
#include "justfloat.h"
#include "mt6835.h"

typedef struct
{
    BspUartDma uart;
    BspSpi3Fast spi;
    Mt6835 encoder;

    uint8_t tx_frame[20];
    uint32_t last_stream_tick_ms;
    uint8_t pole_calib_enabled;
    uint8_t pole_calib_pwm_running;
    uint32_t raw21;
    float pos_mech_rad;
} MotorApp;

void MotorApp_Init(MotorApp *ctx,
                   UART_HandleTypeDef *huart,
                   SPI_HandleTypeDef *hspi,
                   GPIO_TypeDef *cs_port,
                   uint16_t cs_pin);
void MotorApp_SetPoleCalibEnabled(MotorApp *ctx, uint8_t enabled);
void MotorApp_Loop(MotorApp *ctx);

#endif /* APP_MOTOR_APP_H */
