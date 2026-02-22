#ifndef APP_MOTOR_APP_H
#define APP_MOTOR_APP_H

#include "bsp_adc_inj_pair.h"
#include "bsp_spi3_fast.h"
#include "bsp_tim1_pwm.h"
#include "bsp_trig.h"
#include "bsp_uart_dma.h"
#include "current_sense.h"
#include "foc_current_ctrl.h"
#include "host_cmd_app.h"
#include "justfloat.h"
#include "motor_calib.h"
#include "mt6835.h"
#include "svpwm.h"

typedef struct
{
    BspUartDma uart;
    HostCmdApp host_cmd;
    BspTim1Pwm pwm;
    BspAdcInjPair adc_inj;
    BspSpi3Fast spi;
    Mt6835 encoder;

    uint8_t tx_frame[20];
    uint32_t last_stream_tick_ms;
    uint32_t raw21;
    float pos_mech_rad;
    uint16_t enc_div_countdown;

    uint16_t adc1_raw;
    uint16_t adc2_raw;
    CurrentSenseOffset2 i_ab_offset;
    uint16_t i_u_offset_raw;
    uint16_t i_v_offset_raw;
    uint16_t i_w_offset_raw;
    uint8_t i_offset_stage;
    uint8_t i_offset_ready;
    float ia_a;
    float ib_a;
    float ic_a;

    uint32_t adc_isr_count;
    uint8_t dbg_calib_state;
    float dbg_theta_e;
    float dbg_ud;
    float dbg_uq;
    float dbg_duty_a;
    float dbg_duty_b;
    float dbg_duty_c;
    float dbg_id_a;
    float dbg_iq_a;
    float dbg_ud_pu;
    float dbg_uq_pu;

    HostCmd last_host_cmd;
    uint32_t last_host_cmd_tick_ms;

    float target_pos_deg;
    float target_vel_rad_s;
    uint8_t calib_request;
    uint8_t calib_request_pending;

    MotorCalib calib;
    int8_t elec_dir;
    float elec_zero_offset_rad;
    uint8_t calib_done;
    uint8_t calib_fail;

    FocCurrentCtrl i_ctrl;
    float id_ref_a;
    float iq_ref_a;
    float i_limit_a;
    uint8_t i_loop_enabled;
    uint8_t i_loop_enable_pending;

    uint8_t tx_debug_toggle;
    uint8_t stream_page;

    uint8_t vtest_active;
    float vtest_ud;
    float vtest_uq;
} MotorApp;

void MotorApp_Init(MotorApp *ctx, UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin,
                   TIM_HandleTypeDef *htim_pwm, ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2);
void MotorApp_Loop(MotorApp *ctx);

#endif /* APP_MOTOR_APP_H */
